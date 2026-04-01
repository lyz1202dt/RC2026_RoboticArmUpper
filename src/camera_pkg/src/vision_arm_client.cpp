#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_interfaces/action/catch.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

using namespace cv;
using PoseStamped = geometry_msgs::msg::PoseStamped;

namespace {
Vec4f rotationMatrixToQuaternion(const Mat& R)
{
    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;

    if (trace > 0.0) {
        double s = std::sqrt(trace + 1.0) * 2.0;
        qw = 0.25 * s;
        qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
        qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
        qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
    } else if (R.at<double>(0, 0) > R.at<double>(1, 1) && R.at<double>(0, 0) > R.at<double>(2, 2)) {
        double s = std::sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2)) * 2.0;
        qw = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
        qx = 0.25 * s;
        qy = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
        qz = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
    } else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
        double s = std::sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2)) * 2.0;
        qw = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
        qx = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
        qy = 0.25 * s;
        qz = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
    } else {
        double s = std::sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1)) * 2.0;
        qw = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
        qx = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
        qy = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
        qz = 0.25 * s;
    }

    return Vec4f(static_cast<float>(qx), static_cast<float>(qy), static_cast<float>(qz), static_cast<float>(qw));
}

Vec3f rotationMatrixToEuler(const Mat& R)
{
    float sy = std::sqrt(
        static_cast<float>(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0)));
    bool singular = sy < 1e-6f;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    if (!singular) {
        x = std::atan2(static_cast<float>(R.at<double>(2, 1)), static_cast<float>(R.at<double>(2, 2)));
        y = std::atan2(static_cast<float>(-R.at<double>(2, 0)), sy);
        z = std::atan2(static_cast<float>(R.at<double>(1, 0)), static_cast<float>(R.at<double>(0, 0)));
    } else {
        x = std::atan2(static_cast<float>(-R.at<double>(1, 2)), static_cast<float>(R.at<double>(1, 1)));
        y = std::atan2(static_cast<float>(-R.at<double>(2, 0)), sy);
        z = 0.0f;
    }
    return Vec3f(x, y, z);
}
}  // namespace

class VisionArmNode : public rclcpp::Node
{
public:
    using Catch = robot_interfaces::action::Catch;
    using GoalHandleCatch = rclcpp_action::ClientGoalHandle<Catch>;

    VisionArmNode() : Node("vision_arm_node")
    {
        pose_topic_ = this->declare_parameter<std::string>("pose_topic", "robotic_task_");
        action_name_ = this->declare_parameter<std::string>("action_name", "robotic_task");
        const int64_t action_type_param = this->declare_parameter<int64_t>("action_type", 2);
        action_type_ = static_cast<int>(action_type_param);
        const int64_t action_wait_param = this->declare_parameter<int64_t>("action_server_wait_ms", 200);
        action_server_wait_ms_ = static_cast<int>(action_wait_param);

        auto_send_action_once_ = this->declare_parameter<bool>("auto_send_action_once", true);
        reset_goal_when_target_lost_ = this->declare_parameter<bool>("reset_goal_when_target_lost", true);
        const int64_t lost_reset_frames_param = this->declare_parameter<int64_t>("target_lost_reset_frames", 20);
        target_lost_reset_frames_ = static_cast<int>(lost_reset_frames_param);

        use_realsense_ = this->declare_parameter<bool>("use_realsense", true);
        fallback_to_sim_on_rs_failure_ = this->declare_parameter<bool>("fallback_to_sim_on_rs_failure", true);
        enable_visualization_ = this->declare_parameter<bool>("enable_visualization", true);
        const int64_t min_contour_points_param = this->declare_parameter<int64_t>("detection_min_contour_points", 10);
        detection_min_contour_points_ = static_cast<int>(min_contour_points_param);

        sim_rate_hz_ = this->declare_parameter<double>("sim_rate_hz", 20.0);
        sim_frame_id_ = this->declare_parameter<std::string>("sim_frame_id", "camera_link");
        sim_target_position_ = this->declare_parameter<std::vector<double>>("sim_target_position", {0.45, 0.0, 0.28});
        sim_target_orientation_ = this->declare_parameter<std::vector<double>>("sim_target_orientation", {0.0, 0.0, 0.0, 1.0});

        pub_ = this->create_publisher<PoseStamped>(pose_topic_, 10);
        if (auto_send_action_once_) {
            client_ = rclcpp_action::create_client<Catch>(this, action_name_);
        }

        K_ = (Mat_<double>(3, 3) << 956.65, 0, 683.6, 0, 961.97, 319.24, 0, 0, 1);
        D_ = Mat::zeros(1, 5, CV_64F);

        float s = 175.0f;
        objectPts_ = {{-s, -s, 0}, {s, -s, 0}, {s, s, 0}, {-s, s, 0}};

        if (use_realsense_) {
            try {
                cfg_.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
                cfg_.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
                pipe_.start(cfg_);
                realsense_ready_ = true;
                RCLCPP_INFO(this->get_logger(), "视觉节点运行在 RealSense 模式");
            } catch (const std::exception& e) {
                realsense_ready_ = false;
                if (fallback_to_sim_on_rs_failure_) {
                    use_realsense_ = false;
                    RCLCPP_WARN(this->get_logger(), "RealSense 启动失败，切换仿真模式: %s", e.what());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "RealSense 启动失败: %s", e.what());
                }
            }
        }

        const double rate = use_realsense_ ? 30.0 : std::max(sim_rate_hz_, 1.0);
        const auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / rate));
        timer_ = this->create_wall_timer(period, std::bind(&VisionArmNode::mainLoop, this));

        RCLCPP_INFO(this->get_logger(), "VisionArmNode 启动: topic=%s, auto_send_action_once=%d", pose_topic_.c_str(), auto_send_action_once_ ? 1 : 0);
    }

private:
    void mainLoop()
    {
        bool valid = false;
        PoseStamped msg;

        if (use_realsense_ && realsense_ready_) {
            valid = processRealSenseFrame(msg);
        } else {
            valid = buildSimTarget(msg);
        }

        if (valid) {
            lost_frames_ = 0;
            last_pose_ = msg;
            has_last_pose_ = true;
            pub_->publish(msg);
            trySendActionOnce(msg);
            return;
        }

        ++lost_frames_;
        if (has_last_pose_) {
            last_pose_.header.stamp = this->now();
            pub_->publish(last_pose_);
        }

        if (reset_goal_when_target_lost_ && goal_sent_ && lost_frames_ >= std::max(target_lost_reset_frames_, 1)) {
            goal_sent_ = false;
            RCLCPP_INFO(this->get_logger(), "目标丢失达到阈值，允许下一次重新触发 action");
        }
    }

    bool buildSimTarget(PoseStamped& out)
    {
        if (sim_target_position_.size() < 3 || sim_target_orientation_.size() < 4) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "sim_target_position/sim_target_orientation 参数维度错误");
            return false;
        }

        out.header.stamp = this->now();
        out.header.frame_id = sim_frame_id_;
        out.pose.position.x = sim_target_position_[0];
        out.pose.position.y = sim_target_position_[1];
        out.pose.position.z = sim_target_position_[2];
        out.pose.orientation.x = sim_target_orientation_[0];
        out.pose.orientation.y = sim_target_orientation_[1];
        out.pose.orientation.z = sim_target_orientation_[2];
        out.pose.orientation.w = sim_target_orientation_[3];
        return true;
    }

    bool processRealSenseFrame(PoseStamped& out)
    {
        rs2::frameset frames;
        if (!pipe_.poll_for_frames(&frames)) {
            return false;
        }

        rs2::video_frame color_frame = frames.get_color_frame();
        if (!color_frame) {
            return false;
        }

        Mat frame(Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3,
                  const_cast<void*>(color_frame.get_data()), Mat::AUTO_STEP);

        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);

        Mat mask1, mask2, redMask;
        inRange(hsv, Scalar(0, 100, 80), Scalar(10, 255, 255), mask1);
        inRange(hsv, Scalar(170, 100, 80), Scalar(180, 255, 255), mask2);
        redMask = mask1 | mask2;

        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(redMask, redMask, MORPH_CLOSE, kernel);
        morphologyEx(redMask, redMask, MORPH_OPEN, kernel);

        std::vector<std::vector<Point>> contours;
        findContours(redMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        double maxArea = 0.0;
        std::vector<Point> best;
        for (auto& c : contours) {
            double a = contourArea(c);
            if (a > maxArea) {
                maxArea = a;
                best = c;
            }
        }

        if (best.size() <= static_cast<size_t>(std::max(detection_min_contour_points_, 1))) {
            if (enable_visualization_) {
                imshow("Frame", frame);
                imshow("HSV", hsv);
                imshow("RedMask", redMask);
                waitKey(1);
            }
            return false;
        }

        RotatedRect rect = minAreaRect(best);
        Point2f pts[4];
        rect.points(pts);

        std::vector<Point2f> imagePts(4);
        for (int i = 0; i < 4; ++i) {
            float x = std::min(std::max(pts[i].x, 0.0f), static_cast<float>(frame.cols - 1));
            float y = std::min(std::max(pts[i].y, 0.0f), static_cast<float>(frame.rows - 1));
            imagePts[i] = Point2f(x, y);
        }

        std::vector<float> sums(4), diffs(4);
        for (int i = 0; i < 4; ++i) {
            sums[i] = imagePts[i].x + imagePts[i].y;
            diffs[i] = imagePts[i].x - imagePts[i].y;
        }

        int tl = 0, tr = 0, br = 0, bl = 0;
        for (int i = 1; i < 4; ++i) {
            if (sums[i] < sums[tl]) tl = i;
            if (sums[i] > sums[br]) br = i;
            if (diffs[i] > diffs[tr]) tr = i;
            if (diffs[i] < diffs[bl]) bl = i;
        }

        std::vector<Point2f> sortedPts(4);
        sortedPts[0] = imagePts[tl];
        sortedPts[1] = imagePts[tr];
        sortedPts[2] = imagePts[br];
        sortedPts[3] = imagePts[bl];
        imagePts = sortedPts;

        Mat gray;
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        TermCriteria criteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001);
        cornerSubPix(gray, imagePts, Size(5, 5), Size(-1, -1), criteria);

        Mat rvec, tvec;
        if (!solvePnP(objectPts_, imagePts, K_, D_, rvec, tvec)) {
            return false;
        }

        Mat R;
        Rodrigues(rvec, R);
        Vec3f euler = rotationMatrixToEuler(R);
        Vec4f quat = rotationMatrixToQuaternion(R);

        out.header.stamp = this->now();
        out.header.frame_id = "camera_link";
        out.pose.position.x = tvec.at<double>(0) / 1000.0;
        out.pose.position.y = tvec.at<double>(1) / 1000.0;
        out.pose.position.z = tvec.at<double>(2) / 1000.0;
        out.pose.orientation.x = quat[0];
        out.pose.orientation.y = quat[1];
        out.pose.orientation.z = quat[2];
        out.pose.orientation.w = quat[3];

        if (enable_visualization_) {
            for (int i = 0; i < 4; ++i) {
                line(frame, imagePts[i], imagePts[(i + 1) % 4], Scalar(0, 255, 0), 3);
                circle(frame, imagePts[i], 5, Scalar(0, 0, 255), -1);
                putText(frame, std::to_string(i), imagePts[i], FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 0, 0), 2);
            }

            char buf[200];
            std::snprintf(
                buf,
                sizeof(buf),
                "Euler: %.1f %.1f %.1f",
                euler[0] * 180.0 / CV_PI,
                euler[1] * 180.0 / CV_PI,
                euler[2] * 180.0 / CV_PI);
            putText(frame, buf, Point(20, 40), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(255, 200, 0), 2);

            imshow("Frame", frame);
            imshow("HSV", hsv);
            imshow("RedMask", redMask);
            waitKey(1);
        }

        return true;
    }

    void trySendActionOnce(const PoseStamped& pose)
    {
        if (!auto_send_action_once_ || goal_sent_ || !client_) {
            return;
        }

        if (!client_->wait_for_action_server(std::chrono::milliseconds(std::max(action_server_wait_ms_, 1)))) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "action server %s 尚未可用，先仅发布目标话题",
                action_name_.c_str());
            return;
        }

        Catch::Goal goal_msg;
        goal_msg.action_type = action_type_;
        goal_msg.target_pose = pose.pose;

        auto send_goal_options = rclcpp_action::Client<Catch>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&VisionArmNode::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&VisionArmNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&VisionArmNode::resultCallback, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
        goal_sent_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "检测到有效目标，已触发一次 action: type=%d, pos=(%.3f, %.3f, %.3f)",
            action_type_,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z);
    }

    void goalResponseCallback(const std::shared_ptr<GoalHandleCatch>& handle)
    {
        if (!handle) {
            RCLCPP_ERROR(this->get_logger(), "one-shot action 目标被拒绝");
            goal_sent_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "one-shot action 目标已被接受");
        }
    }

    void feedbackCallback(
        const std::shared_ptr<GoalHandleCatch>&,
        const std::shared_ptr<const Catch::Feedback>& feedback)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "action反馈: state=%d, desc=%s",
            feedback->current_state,
            feedback->state_describe.c_str());
    }

    void resultCallback(const GoalHandleCatch::WrappedResult& result)
    {
        if (!result.result) {
            RCLCPP_WARN(this->get_logger(), "action 结果为空");
            return;
        }
        RCLCPP_INFO(
            this->get_logger(),
            "action完成: reason=%s, kfs_num=%d",
            result.result->reason.c_str(),
            result.result->kfs_num);
    }

private:
    rclcpp::Publisher<PoseStamped>::SharedPtr pub_;
    rclcpp_action::Client<Catch>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rs2::pipeline pipe_;
    rs2::config cfg_;
    bool realsense_ready_ = false;

    Mat K_;
    Mat D_;
    std::vector<Point3f> objectPts_;

    bool has_last_pose_ = false;
    PoseStamped last_pose_;
    bool goal_sent_ = false;
    int lost_frames_ = 0;

    std::string pose_topic_;
    std::string action_name_;
    int action_type_ = 2;
    int action_server_wait_ms_ = 200;

    bool auto_send_action_once_ = true;
    bool reset_goal_when_target_lost_ = true;
    int target_lost_reset_frames_ = 20;

    bool use_realsense_ = true;
    bool fallback_to_sim_on_rs_failure_ = true;
    bool enable_visualization_ = true;
    int detection_min_contour_points_ = 10;

    double sim_rate_hz_ = 20.0;
    std::string sim_frame_id_ = "camera_link";
    std::vector<double> sim_target_position_;
    std::vector<double> sim_target_orientation_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionArmNode>());
    rclcpp::shutdown();
    return 0;
}
