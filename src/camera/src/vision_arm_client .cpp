#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

using namespace cv;
using PoseStamped = geometry_msgs::msg::PoseStamped;

// ========== 视觉辅助函数（原样保留）==========
Vec4f rotationMatrixToQuaternion(Mat R)
{
    double trace = R.at<double>(0,0)+R.at<double>(1,1)+R.at<double>(2,2);
    double qw,qx,qy,qz;
    if(trace>0){
        double s=sqrt(trace+1.0)*2;
        qw=0.25*s; qx=(R.at<double>(2,1)-R.at<double>(1,2))/s;
        qy=(R.at<double>(0,2)-R.at<double>(2,0))/s; qz=(R.at<double>(1,0)-R.at<double>(0,1))/s;
    } else if(R.at<double>(0,0)>R.at<double>(1,1)&&R.at<double>(0,0)>R.at<double>(2,2)){
        double s=sqrt(1.0+R.at<double>(0,0)-R.at<double>(1,1)-R.at<double>(2,2))*2;
        qw=(R.at<double>(2,1)-R.at<double>(1,2))/s; qx=0.25*s;
        qy=(R.at<double>(0,1)+R.at<double>(1,0))/s; qz=(R.at<double>(0,2)+R.at<double>(2,0))/s;
    } else if(R.at<double>(1,1)>R.at<double>(2,2)){
        double s=sqrt(1.0+R.at<double>(1,1)-R.at<double>(0,0)-R.at<double>(2,2))*2;
        qw=(R.at<double>(0,2)-R.at<double>(2,0))/s; qx=(R.at<double>(0,1)+R.at<double>(1,0))/s;
        qy=0.25*s; qz=(R.at<double>(1,2)+R.at<double>(2,1))/s;
    } else {
        double s=sqrt(1.0+R.at<double>(2,2)-R.at<double>(0,0)-R.at<double>(1,1))*2;
        qw=(R.at<double>(1,0)-R.at<double>(0,1))/s; qx=(R.at<double>(0,2)+R.at<double>(2,0))/s;
        qy=(R.at<double>(1,2)+R.at<double>(2,1))/s; qz=0.25*s;
    }
    return Vec4f(qx,qy,qz,qw);
}

Vec3f rotationMatrixToEuler(Mat R)
{
    float sy=sqrt(R.at<double>(0,0)*R.at<double>(0,0)+R.at<double>(1,0)*R.at<double>(1,0));
    bool singular=sy<1e-6;
    float x,y,z;
    if(!singular){ x=atan2(R.at<double>(2,1),R.at<double>(2,2)); y=atan2(-R.at<double>(2,0),sy); z=atan2(R.at<double>(1,0),R.at<double>(0,0)); }
    else { x=atan2(-R.at<double>(1,2),R.at<double>(1,1)); y=atan2(-R.at<double>(2,0),sy); z=0; }
    return Vec3f(x,y,z);
}

class VisionArmNode : public rclcpp::Node
{
public:
    VisionArmNode() : Node("vision_arm_node")
    {
        // 话题发布，订阅这个话题即可
        pub_ = this->create_publisher<PoseStamped>("robotic_task_", 10);

        // RealSense 初始化
        cfg_.enable_stream(RS2_STREAM_COLOR,1280,720,RS2_FORMAT_BGR8,30);
        cfg_.enable_stream(RS2_STREAM_DEPTH,1280,720,RS2_FORMAT_Z16,30);
        pipe_.start(cfg_);

        K_ = (Mat_<double>(3,3) << 956.65,0,683.6, 0,961.97,319.24, 0,0,1);
        D_ = Mat::zeros(1,5,CV_64F);

        float s=175;
        objectPts_ = {{-s,-s,0},{s,-s,0},{s,s,0},{-s,s,0}};
        
        // 30ms 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&VisionArmNode::vision_loop, this));

        RCLCPP_INFO(this->get_logger(), "VisionArmNode 启动");
    }

private:
    rclcpp::Publisher<PoseStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rs2::pipeline pipe_;
    rs2::config cfg_;
    Mat K_, D_;
    std::vector<Point3f> objectPts_;

    // 缓存上一次成功检测的位姿
    bool has_last_pose_ = false;
    PoseStamped last_pose_;

    void vision_loop()
    {
        // 非阻塞取帧，没有新帧时用缓存位姿继续发布
        rs2::frameset frames;
        if (!pipe_.poll_for_frames(&frames))
        {
            if (has_last_pose_)
            {
                last_pose_.header.stamp = this->now();
                pub_->publish(last_pose_);
            }
            return;
        }
        rs2::video_frame color_frame = frames.get_color_frame();
        if (!color_frame) return;

        Mat frame(Size(color_frame.get_width(),color_frame.get_height()),CV_8UC3,
                  (void*)color_frame.get_data(),Mat::AUTO_STEP);

        // ===== 视觉算法（原样保留）=====
        Mat hsv;
        cvtColor(frame,hsv,COLOR_BGR2HSV);

        Mat mask1,mask2,redMask;
        inRange(hsv,Scalar(0,100,80),Scalar(10,255,255),mask1);
        inRange(hsv,Scalar(170,100,80),Scalar(180,255,255),mask2);
        redMask=mask1|mask2;

        Mat kernel=getStructuringElement(MORPH_RECT,Size(5,5));
        morphologyEx(redMask,redMask,MORPH_CLOSE,kernel);
        morphologyEx(redMask,redMask,MORPH_OPEN,kernel);

        std::vector<std::vector<Point>> contours;
        findContours(redMask,contours,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);

        double maxArea=0;
        std::vector<Point> best;
        for(auto &c:contours){ double a=contourArea(c); if(a>maxArea){maxArea=a;best=c;} }

        if(best.size()>10)
        {
            RotatedRect rect=minAreaRect(best);
            Point2f pts[4]; rect.points(pts);

            std::vector<Point2f> imagePts(4);
            for(int i=0;i<4;i++){
                float x=std::min(std::max(pts[i].x,0.f),float(frame.cols-1));
                float y=std::min(std::max(pts[i].y,0.f),float(frame.rows-1));
                imagePts[i]=Point2f(x,y);
            }

            std::vector<float> sums(4),diffs(4);
            for(int i=0;i<4;i++){sums[i]=imagePts[i].x+imagePts[i].y; diffs[i]=imagePts[i].x-imagePts[i].y;}
            int tl=0,tr=0,br=0,bl=0;
            for(int i=1;i<4;i++){
                if(sums[i]<sums[tl])tl=i; if(sums[i]>sums[br])br=i;
                if(diffs[i]>diffs[tr])tr=i; if(diffs[i]<diffs[bl])bl=i;
            }
            std::vector<Point2f> sortedPts(4);
            sortedPts[0]=imagePts[tl]; sortedPts[1]=imagePts[tr];
            sortedPts[2]=imagePts[br]; sortedPts[3]=imagePts[bl];
            imagePts=sortedPts;

            Mat gray; cvtColor(frame,gray,COLOR_BGR2GRAY);
            TermCriteria criteria(TermCriteria::EPS+TermCriteria::MAX_ITER,30,0.001);
            cornerSubPix(gray,imagePts,Size(5,5),Size(-1,-1),criteria);

            for(int i=0;i<4;i++){
                line(frame,imagePts[i],imagePts[(i+1)%4],Scalar(0,255,0),3);
                circle(frame,imagePts[i],5,Scalar(0,0,255),-1);
                putText(frame,std::to_string(i),imagePts[i],FONT_HERSHEY_SIMPLEX,0.8,Scalar(255,0,0),2);
            }

            Mat rvec,tvec;
            if(solvePnP(objectPts_,imagePts,K_,D_,rvec,tvec))
            {
                Mat R; Rodrigues(rvec,R);
                Vec3f euler=rotationMatrixToEuler(R);
                Vec4f quat=rotationMatrixToQuaternion(R);

                // pos 单位 mm -> m
                double pos[3]={tvec.at<double>(0)/1000.0,
                               tvec.at<double>(1)/1000.0,
                               tvec.at<double>(2)/1000.0};

                // ===== 发布话题 =====
                PoseStamped msg;
                msg.header.stamp    = this->now();
                msg.header.frame_id = "camera_link";
                msg.pose.position.x = pos[0];
                msg.pose.position.y = pos[1];
                msg.pose.position.z = pos[2];
                msg.pose.orientation.x = quat[0];
                msg.pose.orientation.y = quat[1];
                msg.pose.orientation.z = quat[2];
                msg.pose.orientation.w = quat[3];
                pub_->publish(msg);

                // 缓存本次成功位姿
                last_pose_ = msg;
                has_last_pose_ = true;

                // ===== 视觉显示（原样保留）=====
                std::vector<Point3f> axes={{0,0,0},{100,0,0},{0,100,0},{0,0,100}};
                std::vector<Point2f> imgPts;
                projectPoints(axes,rvec,tvec,K_,D_,imgPts);
                line(frame,imgPts[0],imgPts[1],Scalar(0,0,255),3);
                line(frame,imgPts[0],imgPts[2],Scalar(0,255,0),3);
                line(frame,imgPts[0],imgPts[3],Scalar(255,0,0),3);

                double dist=sqrt(tvec.at<double>(0)*tvec.at<double>(0)+
                                 tvec.at<double>(1)*tvec.at<double>(1)+
                                 tvec.at<double>(2)*tvec.at<double>(2));
                float minX=1e9,minY=1e9;
                for(auto&p:imagePts){if(p.x<minX)minX=p.x;if(p.y<minY)minY=p.y;}
                Point textOrg(minX,minY-40);

                char buf[200];
                sprintf(buf,"Dist: %.1f mm",dist);
                putText(frame,buf,textOrg,FONT_HERSHEY_SIMPLEX,0.7,Scalar(0,255,255),2);
                sprintf(buf,"Euler: %.1f %.1f %.1f",euler[0]*180/CV_PI,euler[1]*180/CV_PI,euler[2]*180/CV_PI);
                putText(frame,buf,Point(textOrg.x,textOrg.y+25),FONT_HERSHEY_SIMPLEX,0.7,Scalar(0,255,0),2);
                sprintf(buf,"Quat: %.3f %.3f %.3f %.3f",quat[0],quat[1],quat[2],quat[3]);
                putText(frame,buf,Point(textOrg.x,textOrg.y+50),FONT_HERSHEY_SIMPLEX,0.7,Scalar(255,200,0),2);
            }
        }

        // 检测失败时发布上一次成功的位姿
        if (!has_last_pose_ && best.size() <= 10)
        {
            // 首次就检测失败，无缓存，不发布
        }
        else if (best.size() <= 10 && has_last_pose_)
        {
            last_pose_.header.stamp = this->now();
            pub_->publish(last_pose_);
        }

        // ===== 视觉显示（原样保留）=====
        imshow("Frame",frame);
        imshow("HSV",hsv);
        imshow("RedMask",redMask);
        waitKey(1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionArmNode>());
    rclcpp::shutdown();
    return 0;
}
