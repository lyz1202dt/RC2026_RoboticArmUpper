#include "arm_handle_node.hpp"
#include "robot_interfaces/action/catch.hpp"
#include "robot_interfaces/msg/arm.hpp"
#include "robot_interfaces/msg/joint.hpp"
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_action/server.hpp>
#include <thread>

ArmHandleNode::ArmHandleNode()
    : Node("robotic_task") {
    arm_task_thread = std::make_unique<std::thread>(
        std::bind(&ArmHandleNode::arm_catch_task_handle, this)); // 创建机械臂任务执行线程
    arm_handle_server = rclcpp_action::create_server<robot_interfaces::action::Catch>(
        this, "robotic_task",                                    // 创建服务端
        std::bind(&ArmHandleNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ArmHandleNode::cancel_goal, this, std::placeholders::_1),
        std::bind(&ArmHandleNode::handle_accepted, this, std::placeholders::_1));
    arm_target_publisher = this->create_publisher<robot_interfaces::msg::Arm>(
        "myjoints_target", 10); // 创建发布者，发布关节空间的目标
}

ArmHandleNode::~ArmHandleNode() {
    this->queue_closed = true;
    if (arm_task_thread->joinable())
        arm_task_thread->join();
}

rclcpp_action::GoalResponse ArmHandleNode::handle_goal(const rclcpp_action::GoalUUID& uuid,std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmHandleNode::cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>>goal_handle) {
    cancle_current_task = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmHandleNode::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>>goal_handle) {

}

// 机械臂
void ArmHandleNode::arm_catch_task_handle() {
    robot_interfaces::msg::Arm arm_msg;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lk(arm_task_queue_mtx);
        wakeup_condition_var.wait(lk, [this]() {
            return (!arm_task_queue.empty()) || queue_closed;
        }); // 等待队列中有数据或者请求退出线程
        if (this->queue_closed)
            return;
        // 从队列中取到了最新的轨迹请求
        plan = arm_task_queue.front();
        arm_task_queue.pop();        // 取出轨迹
        lk.unlock();                 // 解锁队列

        auto start_time = std::chrono::high_resolution_clock::time_point::clock::now();
        for (auto const& point : plan.trajectory.joint_trajectory.points) {
            double time_from_start =
                point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            std::this_thread::sleep_until(
                start_time + std::chrono::duration<double>(time_from_start));

            if (point.positions.size() != 6) {
                RCLCPP_WARN(this->get_logger(), "不符合的关节数");
                continue;
            }

            for (int i = 0; i < 6; i++) {
                arm_msg.joints[i].rad   = (float)point.positions[i];
                arm_msg.joints[i].omega = (float)point.velocities[i];
            }
            RCLCPP_INFO(this->get_logger(), "发布关节角期望");
            arm_target_publisher->publish(arm_msg);

            if (cancle_current_task) // 如果要求退出当前的动作执行
            {
                for (int i = 0; i < 6; i++) {
                    arm_msg.joints[i].omega = 0.0f;
                }
                arm_target_publisher->publish(arm_msg);
                break;
            }
        }
    }
}