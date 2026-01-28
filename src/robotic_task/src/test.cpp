#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "robot_interfaces/action/catch.hpp"

class ActionTestNode : public rclcpp::Node {
public:
    using Catch = robot_interfaces::action::Catch;
    using GoalHandleCatch = rclcpp_action::ClientGoalHandle<Catch>;

    ActionTestNode() : Node("action_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "ActionTestNode 启动，准备连接 Action Server...");

        client_ = rclcpp_action::create_client<Catch>(this, "robotic_task");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&ActionTestNode::send_goal, this)
        );

        send_goal();
    }

private:
    rclcpp_action::Client<Catch>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_sent_ = false;

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


        //模拟的抓取位置姿
            // 在 rviz2 中的显示为z轴方向
        goal_msg.target_pose.position.x = 0.0;//0.664748;
            // 在 rviz2 中的显示为y轴方向
        goal_msg.target_pose.position.y = -0.1;//-0.001824;
            // 在 rviz2 中的显示为x轴方向
        goal_msg.target_pose.position.z = 0.5;//0.256471;

        goal_msg.target_pose.orientation.w = 1.0; // 0.004481;  // 单位四元数
        goal_msg.target_pose.orientation.x = 0.0; // 0.708322;
        goal_msg.target_pose.orientation.y = 0.0; // -0.004257;
        goal_msg.target_pose.orientation.z = 0.0; // -0.705862;

        

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
