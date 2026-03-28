#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_interfaces/action/catch.hpp"
#include <robot_interfaces/action/detail/catch__struct.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <memory>
#include <string>

using namespace cv;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using robot_interfaces::action::Catch;



// ========== 视觉辅助函数（原样保留）==========
/**
 * @brief 将旋转矩阵转换为四元数表示
 * 
 * 根据旋转矩阵的迹（trace）选择不同的计算路径，确保数值稳定性。
 * 使用经典的 Shepperd 算法将 3x3 旋转矩阵转换为单位四元数。
 * 
 * @param R 3x3 旋转矩阵，类型为 CV_64F
 * @return Vec4f 四元数 (qx, qy, qz, qw)，其中 qw 为实部
 */
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

/**
 * @brief 将旋转矩阵转换为欧拉角表示
 * 
 * 使用反正切函数从旋转矩阵中提取 ZYX 顺序的欧拉角。
 * 处理万向节锁奇异点情况，当 pitch 角接近±90°时采用备用计算方案。
 * 
 * @param R 3x3 旋转矩阵，类型为 CV_64F
 * @return Vec3f 欧拉角 (x, y, z)，单位为弧度，分别对应 roll、pitch、yaw
 */
Vec3f rotationMatrixToEuler(Mat R)
{
    float sy=sqrt(R.at<double>(0,0)*R.at<double>(0,0)+R.at<double>(1,0)*R.at<double>(1,0));
    bool singular=sy<1e-6;
    float x,y,z;
    if(!singular){ x=atan2(R.at<double>(2,1),R.at<double>(2,2)); y=atan2(-R.at<double>(2,0),sy); z=atan2(R.at<double>(1,0),R.at<double>(0,0)); }
    else { x=atan2(-R.at<double>(1,2),R.at<double>(1,1)); y=atan2(-R.at<double>(2,0),sy); z=0; }
    return Vec3f(x,y,z);
}

/**
 * @brief 视觉 - 机械臂协同节点类
 * 
 * 该节点基于 Intel RealSense 深度相机实现红色物体的实时位姿估计。
 * 主要功能包括：
 * 1. 采集 RGB-D 图像数据
 * 2. 通过 HSV 颜色空间分割提取红色区域
 * 3. 使用轮廓检测和角点排序定位物体四个顶点
 * 4. 通过 PnP 算法解算物体相对于相机的 6DOF 位姿
 * 5. 发布 geometry_msgs/PoseStamped 类型的话题供机械臂控制使用
 * 
 * 特性：
 * - 30Hz 实时处理频率
 * - 检测失败时自动保持发布上一帧有效位姿
 * - 可视化显示检测结果和坐标系
 */
class VisionArmNode : public rclcpp::Node
{
public:

    using Catch = robot_interfaces::action::Catch;
    using GoalHandleCatch = rclcpp_action::ClientGoalHandle<Catch>;


    /**
     * @brief 构造函数，初始化节点所有组件
     * 
     * 初始化流程：
     * 1. 创建话题发布者用于发布位姿信息
     * 2. 配置并启动 RealSense 管道，同时采集彩色图和深度图
     * 3. 设置相机内参矩阵和畸变系数（针对 Realsense D435 标定）
     * 4. 定义物体坐标系下的 4 个角点位置（边长 350mm 正方形）
     * 5. 启动 30ms 定时器触发视觉处理循环
     */
    VisionArmNode() : Node("vision_arm_node")
    {
        // 话题发布，订阅这个话题即可
        pub_ = this->create_publisher<PoseStamped>("robotic_task_", 10);
        client_ = rclcpp_action::create_client<Catch>(this, "robotic_task");

        // RealSense 初始化
        cfg_.enable_stream(RS2_STREAM_COLOR,1280,720,RS2_FORMAT_BGR8,30);
        cfg_.enable_stream(RS2_STREAM_DEPTH,1280,720,RS2_FORMAT_Z16,30);
        pipe_.start(cfg_);

        K_ = (Mat_<double>(3,3) << 956.65, 0, 683.6, 0, 961.97, 319.24, 0, 0, 1);
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
    // ROS 2 发布器，用于发布物体位姿话题
    rclcpp::Publisher<PoseStamped>::SharedPtr pub_;
    rclcpp_action::Client<Catch>::SharedPtr client_;

    // 定时器，控制视觉处理频率为 30Hz    
    rclcpp::TimerBase::SharedPtr timer_;

    // RealSense 管道，负责帧数据流管理
    rs2::pipeline pipe_;

    // RealSense 配置对象，设置流参数
    rs2::config cfg_;

    // 相机内参矩阵 [fx,0,cx; 0,fy,cy; 0,0,1]
    // 相机畸变系数 [k1,k2,p1,p2,k3]
    Mat K_, D_;


    // 物体在自身坐标系下的 4 个角点坐标（单位：mm）
    std::vector<Point3f> objectPts_;

    // 缓存上一次成功检测的位姿
    bool has_last_pose_ = false;    
    PoseStamped last_pose_;


     /**
     * @brief 视觉处理主循环函数
     * 
     * 该函数每 30ms 被定时器调用一次，执行完整的视觉处理流程：
     * 
     * 1. 帧获取阶段：
     *    - 非阻塞方式查询新帧，避免程序等待
     *    - 无新帧时使用缓存位姿继续发布，保证话题连续性
     * 
     * 2. 图像处理阶段：
     *    - 转换到 HSV 颜色空间
     *    - 双阈值分割提取红色区域（覆盖 0-10°和 170-180°色相）
     *    - 形态学闭运算和开运算去除噪声
     * 
     * 3. 特征提取阶段：
     *    - 提取最大面积轮廓作为目标物体
     *    - 计算最小外接矩形获取四个顶点
     *    - 按左上、右上、右下、左下顺序排序角点
     *    - 亚像素级角点优化提高精度
     * 
     * 4. 位姿解算阶段：
     *    - 使用 PnP 算法求解物体相对于相机的位姿
     *    - 转换为欧拉角和四元数两种表示
     *    - 坐标单位从 mm 转换到 m
     * 
     * 5. 结果输出阶段：
     *    - 发布 PoseStamped 话题到 robotic_task_
     *    - 更新缓存位姿
     *    - 绘制坐标轴和位姿信息叠加层
     * 
     * 容错机制：
     * - 轮廓点少于 10 个视为检测失败
     * - 首次检测失败不发布消息
     * - 后续检测失败则重复发布上次成功位姿
     */
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





















        auto goal_msg = Catch::Goal();
        goal_msg.action_type = 2; // 抓取动作
        auto send_goal_options = rclcpp_action::Client<Catch>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&VisionArmNode::goal_response_cb, this, std::placeholders::_1);

        send_goal_options.feedback_callback =
            std::bind(&VisionArmNode::feedback_cb, this,
                std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&VisionArmNode::result_cb, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);

        





















        
























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

    void goal_response_cb(std::shared_ptr<GoalHandleCatch> handle)
    {
        
        if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝！");
        } else {
            RCLCPP_INFO(this->get_logger(), "目标已被接受！");
        }
    }

    void feedback_cb(
        std::shared_ptr<GoalHandleCatch> /*unused*/,
        const std::shared_ptr<const Catch::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(),
            "收到反馈: state=%d 描述=%s",
            feedback->current_state,
            feedback->state_describe.c_str());
    }

    void result_cb(const GoalHandleCatch::WrappedResult &result)
    {
        RCLCPP_INFO(this->get_logger(), "===== 任务完成 =====");
        RCLCPP_INFO(this->get_logger(), "最终结果: %s", result.result->reason.c_str());
        RCLCPP_INFO(this->get_logger(), "最终 kfs_num = %d", result.result->kfs_num);

        rclcpp::shutdown(); // 测试完成后关闭节点
    }
};








/**
 * @brief 程序入口函数
 * 
 * 初始化 ROS 2 环境并启动节点事件循环。
 * spin 函数会阻塞运行直到节点被终止，期间持续处理回调函数。
 * 
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码，正常退出返回 0
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionArmNode>());
    rclcpp::shutdown();
    return 0;
}
