#pragma once

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <thread>
#include <memory>
#include <string>

class EffortCalcNode{
    public:
    EffortCalcNode(const rclcpp::Node::SharedPtr node);
    private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Node::SharedPtr param_node;
    KDL::Tree tree;
    KDL::Chain chain;
    std::string urdf_xml;
    KDL::JntArray DynamicCalc(const KDL::JntArray &q,const KDL::JntArray &q_dot,const KDL::JntArray &q_ddot);
    
    std::shared_ptr<rclcpp::SyncParametersClient> robot_description_param_;


    rclcpp_action::GoalResponse server_goal_handle(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse server_cancle_handle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void server_accepted_handle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr follow_joint_trajectory_server;

    std::shared_ptr<std::thread> trajectory_deal_thread;
    void trajectory_handle();
};
