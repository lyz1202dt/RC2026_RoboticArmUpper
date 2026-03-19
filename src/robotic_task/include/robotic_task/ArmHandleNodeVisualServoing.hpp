#pragma once

#include "SegmentedVelocityController.hpp"
#include "TragectorySmoother.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp_action/create_client.hpp>
#include <robot_interfaces/action/catch.hpp>

#include <thread>
#include <memory>
#include <atomic>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <vector>


class VisualServoingArmHandleNode {
public:
    explicit VisualServoingArmHandleNode(const rclcpp::Node::SharedPtr node);
    ~VisualServoingArmHandleNode();

private:
    // current 
    geometry_msgs::msg::PoseStamped CurrentPose_;
    geometry_msgs::msg::PoseStamped TargetPose_;
    geometry_msgs::msg::PoseStamped LastTargetPose_;

    rclcpp::Node::SharedPtr node_;
    Eigen::Vector3d path_vector_;

    // calculate path vector
    Eigen::Vector3d CalculatePath(Eigen::Vector3d& current_position, Eigen::Vector3d& target_position);

    // calculate twist
    geometry_msgs::msg::Twist CalculateTwist(Eigen::Vector3d& path_vector);

    // send twist to controller
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;



};

VisualServoingArmHandleNode::VisualServoingArmHandleNode(const rclcpp::Node::SharedPtr node) : node_(node) {
    // 初始化路径向量
    path_vector_ = Eigen::Vector3d::Zero();
}

VisualServoingArmHandleNode::~VisualServoingArmHandleNode() {
    // TODO: 析构函数
}

Eigen::Vector3d VisualServoingArmHandleNode::CalculatePath(
    Eigen::Vector3d& current_position, Eigen::Vector3d& target_position
) {
    Eigen::Vector3d path_vector_ = target_position - current_position; // 计算路径向量
    return path_vector_;
}

geometry_msgs::msg::Twist VisualServoingArmHandleNode::CalculateTwist(Eigen::Vector3d& path_vector) {
    geometry_msgs::msg::Twist twist_msg;
    double max_liner_velocity = 1.0; // 最大线速度
    double max_angular_velocity = 10.0; // 最大角速度

    // 计算线速度
    twist_msg.linear.x = path_vector.x();
    twist_msg.linear.y = path_vector.y();
    twist_msg.linear.z = path_vector.z();
    double liner_velocity_magnitude = path_vector.norm(); // 计算路径向量的大小
    if (liner_velocity_magnitude > max_liner_velocity) {
        twist_msg.linear.x = (path_vector.x() / liner_velocity_magnitude) * max_liner_velocity;
        twist_msg.linear.y = (path_vector.y() / liner_velocity_magnitude) * max_liner_velocity;
        twist_msg.linear.z = (path_vector.z() / liner_velocity_magnitude) * max_liner_velocity;
    }

    // TODO: 计算角速度
    twist_msg.angular.x = 0.0; // 这里暂时设置为0，实际应用中需要根据路径向量计算合适的角速度
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;


    return twist_msg;
}













