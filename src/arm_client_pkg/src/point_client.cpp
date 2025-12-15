#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "robot_interfaces/action/catch.hpp"
#include <opencv2/opencv.hpp>
#include "inference.h"
#include <librealsense2/rs.hpp>
#include <tf2/LinearMath/Quaternion.h>

using robot_interfaces::action::Catch;

struct Pose3D
{
    float x = 0, y = 0, z = 0;          // 三维坐标
    float roll = 0, pitch = 0, yaw = 0; // 欧拉角
};

// ------------------------ RealSense 封装 ------------------------
class D435Capture
{
public:
    D435Capture(int width = 640, int height = 480, int fps = 60)
        : width_(width), height_(height), fps_(fps)
    {
        cfg_.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_BGR8, fps_);
        cfg_.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
        pipe_.start(cfg_);
    }
    ~D435Capture() { pipe_.stop(); }

    bool read(cv::Mat &colorFrame, cv::Mat &depthFrame)
    {
        try
        {
            rs2::frameset frames = pipe_.wait_for_frames();
            rs2::video_frame color = frames.get_color_frame();
            rs2::depth_frame depth = frames.get_depth_frame();
            if (!color || !depth)
                return false;

            colorFrame = cv::Mat(cv::Size(color.get_width(), color.get_height()),
                                 CV_8UC3, (void *)color.get_data(), cv::Mat::AUTO_STEP)
                             .clone();
            depthFrame = cv::Mat(cv::Size(depth.get_width(), depth.get_height()),
                                 CV_16UC1, (void *)depth.get_data())
                             .clone();
            return true;
        }
        catch (const rs2::error &e)
        {
            std::cerr << "RealSense 错误: " << e.what() << std::endl;
            return false;
        }
    }

    void getCameraIntrinsics(cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
    {
        auto profile = pipe_.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        auto intr = profile.get_intrinsics();
        cameraMatrix = (cv::Mat_<double>(3, 3) << intr.fx, 0, intr.ppx, 0, intr.fy, intr.ppy, 0, 0, 1);
        distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

        fx_ = intr.fx;
        fy_ = intr.fy;
        ppx_ = intr.ppx;
        ppy_ = intr.ppy;
    }

    float getDepth(const cv::Mat &depthFrame, int x, int y)
    {
        x = std::clamp(x, 0, depthFrame.cols - 1);
        y = std::clamp(y, 0, depthFrame.rows - 1);
        return depthFrame.at<uint16_t>(y, x) * 0.001f;
    }

    cv::Point3f getPointXYZ(const cv::Mat &depthFrame, int x, int y)
    {
        float z = getDepth(depthFrame, x, y);
        float X = (x - ppx_) * z / fx_;
        float Y = (y - ppy_) * z / fy_;
        return cv::Point3f(X, Y, z);
    }

private:
    rs2::pipeline pipe_;
    rs2::config cfg_;
    int width_, height_, fps_;
    float fx_, fy_, ppx_, ppy_;
};

// ------------------------ PnP + 坐标计算函数 ------------------------
Pose3D computePosePnP(
    const cv::Mat &colorFrame,
    const cv::Mat &depthFrame,
    const cv::Rect &box,
    const cv::Mat &cameraMatrix,
    const cv::Mat &distCoeffs,
    D435Capture &cap)
{
    Pose3D pose;

    // PnP object points
    float objSize = 0.35f; // 根据实际物体尺寸调整
    std::vector<cv::Point3f> objectPoints = {{0, 0, 0}, {objSize, 0, 0}, {objSize, objSize, 0}, {0, objSize, 0}};
    std::vector<cv::Point2f> imagePoints = {
        {float(box.x), float(box.y)},
        {float(box.x + box.width), float(box.y)},
        {float(box.x + box.width), float(box.y + box.height)},
        {float(box.x), float(box.y + box.height)}};

    cv::Mat rvec, tvec;
    if (cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec))
    {
        cv::Mat R;
        cv::Rodrigues(rvec, R);

        // 欧拉角
        pose.roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2)) * 180.0 / CV_PI;
        pose.pitch = atan2(-R.at<double>(2, 0),
                           sqrt(R.at<double>(2, 1) * R.at<double>(2, 1) + R.at<double>(2, 2) * R.at<double>(2, 2))) *
                     180.0 / CV_PI;
        pose.yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180.0 / CV_PI;

        // 框中心三维坐标
        int centerX = box.x + box.width / 2;
        int centerY = box.y + box.height / 2;
        cv::Point3f xyz = cap.getPointXYZ(depthFrame, centerX, centerY);
        pose.x = xyz.x;
        pose.y = xyz.y;
        pose.z = xyz.z;
    }

    return pose;
}

class ActionClientNode : public rclcpp::Node
{
public:
    using GoalHandleCatch = rclcpp_action::ClientGoalHandle<Catch>;

    ActionClientNode()
        : Node("action_client_node"),
          cap_(640, 480, 60),
          inf_("src/best.onnx", cv::Size(640, 640),
               "src/classes.txt", true)
    {
        RCLCPP_INFO(this->get_logger(), "Action Client 启动，准备连接 Action Server...");

        // 创建客户端
        client_ = rclcpp_action::create_client<Catch>(this, "robotic_task");

        cap_.getCameraIntrinsics(cameraMatrix_, distCoeffs_);

        // 主循环：显示相机图像并检测按键
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&ActionClientNode::update_frame, this));
    }

private:
    void send_goal()
    {
        if (goal_sent_)
            return;
        goal_sent_ = true;

        // if (!client_->wait_for_action_server(std::chrono::seconds(3)))
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Action Server 未启动，无法发送请求！");
        //     return;
        // }

        cv::Mat colorFrame, depthFrame;
        if (!cap_.read(colorFrame, depthFrame))
#include <moveit/utils/moveit_error_code.hpp>
        {
            RCLCPP_ERROR(this->get_logger(), "相机读取失败，无法发送目标");
            return;
        }

        auto result = inf_.runInference(colorFrame);
        if (result.empty())
        {
            RCLCPP_WARN(this->get_logger(), "未检测到目标，无法发送目标");
            return;
        }

        auto det = result[0];
        Pose3D pose = computePosePnP(colorFrame, depthFrame, det.box, cameraMatrix_, distCoeffs_, cap_);

        tf2::Quaternion q;
        q.setRPY(pose.roll * CV_PI / 180.0, pose.pitch * CV_PI / 180.0, pose.yaw * CV_PI / 180.0);

        auto goal_msg = Catch::Goal();
        goal_msg.target_pose.position.x = pose.x;
        goal_msg.target_pose.position.y = pose.y;
        goal_msg.target_pose.position.z = pose.z;

        goal_msg.target_pose.orientation.w = q.getW();
        goal_msg.target_pose.orientation.x = q.getX();
        goal_msg.target_pose.orientation.y = q.getY();
        goal_msg.target_pose.orientation.z = q.getZ();

        goal_msg.action_type = 3;

        auto send_goal_options = rclcpp_action::Client<Catch>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&ActionClientNode::goal_response_cb, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&ActionClientNode::feedback_cb, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&ActionClientNode::result_cb, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);

        RCLCPP_INFO(this->get_logger(),
            "发送目标: position=(%.3f, %.3f, %.3f), orientation=(%.3f, %.3f, %.3f, %.3f), action_type=%d",
            pose.x, pose.y, pose.z,
            q.getW(), q.getX(), q.getY(), q.getZ(),
            goal_msg.action_type);


        RCLCPP_INFO(this->get_logger(), "已发送基于相机检测的抓取目标");
    }

    void update_frame()
    {
        if (!cap_.read(last_color_, last_depth_))
            return;
        if (last_color_.empty())
            return;

        cv::Mat frame = last_color_.clone();

        // YOLO 推理和可视化
        auto result = inf_.runInference(frame);
        for (const auto &det : result)
        {
            cv::rectangle(frame, det.box, det.color, 2);
            std::ostringstream label;
            label.precision(2);
            label << det.className << " " << det.confidence;
            int baseline = 0;
            cv::Size textSize = cv::getTextSize(label.str(), cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &baseline);
            cv::rectangle(frame, cv::Point(det.box.x, det.box.y - textSize.height - 10),
                          cv::Point(det.box.x + textSize.width, det.box.y), det.color, cv::FILLED);
            cv::putText(frame, label.str(), cv::Point(det.box.x, det.box.y - 5),
                        cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(0, 0, 0), 1);
        }

        cv::imshow("RealSense YOLO + PnP", frame);
        int key = cv::waitKey(1);
        if (key == 'r' || key == 'R')
        {
            RCLCPP_INFO(this->get_logger(), "'r'键按下，发送抓取请求");
            send_goal();
        }
    }

    void goal_response_cb(std::shared_ptr<GoalHandleCatch> handle)
    {
        if (!handle)
        {
            RCLCPP_ERROR(this->get_logger(), "目标被拒绝！");
        }
        else
        {
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
        if(result.result->reason.c_str() == "成功完成机械臂的执行")
        {
            goal_sent_ = false;
        }
        
        RCLCPP_INFO(this->get_logger(), "===== 任务完成 =====");
        RCLCPP_INFO(this->get_logger(), "最终结果: %s", result.result->reason.c_str());
        RCLCPP_INFO(this->get_logger(), "最终 kfs_num = %d", result.result->kfs_num);
        rclcpp::shutdown();
    }

    rclcpp_action::Client<Catch>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    D435Capture cap_;
    Inference inf_;
    cv::Mat cameraMatrix_, distCoeffs_;
    cv::Mat last_color_, last_depth_;
    bool goal_sent_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
