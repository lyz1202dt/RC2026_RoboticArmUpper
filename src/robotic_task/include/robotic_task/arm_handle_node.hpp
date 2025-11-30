#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include "robot_interfaces/action/catch.hpp"
#include "robot_interfaces/msg/arm.hpp"
#include "robot_interfaces/msg/joint.hpp"
#include <condition_variable>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <robot_interfaces/action/catch.hpp>
#include <thread>
#include <chrono>
#include <memory>
#include <vector>

typedef struct{
    int a;
}ArmTask_t;

typedef enum{
    ROBOTIC_ARM_IDEL,
    ROBOTIC_ARM_MOVE_TO_READY_CATCH_POINT,
    ROBOTIC_ARM_MOVE_TO_CATCH_POINT,
    ROBOTIC_ARM_CATCH_TARGET,
    ROBOTIC_ARM_MOVE_TO_RELEASE_POINT,
    ROBOTIC_ARM_RELESE_TARGET,

    ROBOTIC_ARM_NORMAL_MOVE=10
}ArmTaskStage;

class ArmHandleNode : public  rclcpp::Node{
public:
    ArmHandleNode();
    ~ArmHandleNode();
private:
    rclcpp_action::Server<robot_interfaces::action::Catch>::SharedPtr arm_handle_server;  //暴露的机械臂任务接口
    rclcpp::Publisher<robot_interfaces::msg::Arm> ::SharedPtr arm_target_publisher;

    bool queue_closed{false};
    bool cancle_current_task{false};
    std::condition_variable wakeup_condition_var;   //队列非空时挂起阻塞
    std::unique_ptr<std::thread> arm_task_thread;    //执行期望，解析plan并发布节点的线程
    int current_kfs_num{0};


    void arm_catch_task_handle();
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal);
    rclcpp_action::CancelResponse cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle);
};
