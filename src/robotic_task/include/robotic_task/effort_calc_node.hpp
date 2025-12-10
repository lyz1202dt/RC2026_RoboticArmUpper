#pragma once

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <memory>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

class EffortCalcNode{
    public:
    EffortCalcNode(const rclcpp::Node::SharedPtr &node);
    private:
    KDL::Tree tree;
    KDL::Chain chain;
    std::string urdf_xml;
    std::shared_ptr<moveit::core::RobotState> robot_state;
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscriber_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
};