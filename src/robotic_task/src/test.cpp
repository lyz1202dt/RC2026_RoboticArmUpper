#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "robot_interfaces/action/catch.hpp"
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <memory>
#include <string>

class ActionTestNode : public rclcpp::Node {
public:
    using Catch = robot_interfaces::action::Catch;
    using GoalHandleCatch = rclcpp_action::ClientGoalHandle<Catch>;

    ActionTestNode() : Node("action_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "ActionTestNode 启动，准备连接 Action Server...");

        client_ = rclcpp_action::create_client<Catch>(this, "robotic_task");

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("box_pose", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(10), // 100 ms
            std::bind(&ActionTestNode::send_goal, this) // [this]() {
        );

        timer_2_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 20 Hz
            
            [this]() {
                publish_pose_topic_once();
            }
        );

        publish_pose_topic_once();
        send_goal();
    }

private:
    geometry_msgs::msg::Pose make_test_pose() const
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.7;
        pose.position.y = 0.0;
        pose.position.z = 0.5;
        pose.orientation.w = 1.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 0.0;
        return pose;
    }

    rclcpp_action::Client<Catch>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_2_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    bool goal_sent_ = false;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped transform_stamped_;

    void publish_pose_topic_once()
    {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "base_link";

        pose_msg.pose = make_test_pose();

        try {
            // 使用最新可用 TF，避免因时间戳略早于 TF 缓存起点导致 past extrapolation。
            auto tf = tf_buffer_->lookupTransform(
                "camera_link",
                "base_link",
                tf2::TimePointZero,
                tf2::durationFromSec(0.2));

            geometry_msgs::msg::PoseStamped transformed_pose;
            tf2::doTransform(pose_msg, transformed_pose, tf);
            pose_pub_->publish(transformed_pose);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "已将 base_link 下位姿转换到 camera_link: [%.3f, %.3f, %.3f],[%.3f, %.3f, %.3f, %.3f]",
                transformed_pose.pose.position.x,
                transformed_pose.pose.position.y,
                transformed_pose.pose.position.z,
                transformed_pose.pose.orientation.w,
                transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "坐标变换失败(等待TF就绪): %s",
                ex.what());
        }
    }


    // 目标发送函数详解
    void send_goal()
    {
        // 确保整个生命周期内只发送一次目标
        if (goal_sent_) return;
        goal_sent_ = true;

        if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Action Server 未启动，退出。");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Action Server 已连接，开始发送 ROBOTIC_ARM_TASK_MOVE 目标");

        auto goal_msg = Catch::Goal();

        // ===== 填写目标位姿（camera_link 下的一个简单坐标）=====

        // 复用与 timer_2_ 相同的数据源，确保发布位姿与 action 目标一致
        goal_msg.target_pose = make_test_pose();

        

        // action 类型为 “移动”
        // goal_msg.action_type =  1;

        // action 类型为 “抓取”
        goal_msg.action_type =  2;      //捕获目标在这个坐标的物体
        
        auto send_goal_options = rclcpp_action::Client<Catch>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            std::bind(&ActionTestNode::goal_response_cb, this, std::placeholders::_1);

        send_goal_options.feedback_callback =
            std::bind(&ActionTestNode::feedback_cb, this,
                std::placeholders::_1, std::placeholders::_2);

        send_goal_options.result_callback =
            std::bind(&ActionTestNode::result_cb, this, std::placeholders::_1);

        client_->async_send_goal(goal_msg, send_goal_options);
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ActionTestNode>());
    rclcpp::shutdown();
    return 0;
}
