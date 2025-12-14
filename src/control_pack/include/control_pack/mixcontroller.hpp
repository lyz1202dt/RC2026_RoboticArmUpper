#pragma once

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <Eigen/src/Core/Matrix.h>
#include <controller_interface/controller_interface_base.hpp>
#include <joint_trajectory_controller/interpolation_methods.hpp>
#include <kdl/jntarray.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>
#include <memory>
#include <vector>
#include <string>
#include <vector>

namespace mixcontroller {

class MixController : public controller_interface::ControllerInterface {
public:
    MixController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

    controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:

    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_server_;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> activate_goal_handle_;
    control_msgs::action::FollowJointTrajectory::Feedback::SharedPtr feedback_msg;
    control_msgs::action::FollowJointTrajectory::Result::SharedPtr result_msg;
    trajectory_msgs::msg::JointTrajectory current_trajectory_;
    size_t trajectory_index_;
    rclcpp::Time trajectory_start_time;

    std::vector<std::string> joint_names_;

    KDL::Tree tree;
    KDL::Chain chain;
    std::string urdf_xml;
    rclcpp::Node::SharedPtr param_node;
    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;

    //动力学参数计算
    KDL::Vector gravity;
    std::shared_ptr<KDL::ChainDynParam> dyn;
    KDL::JntSpaceInertiaMatrix M_kdl;
    KDL::JntArray C_kdl, G_kdl;
    KDL::JntArray q_kdl,dq_kdl,ddq_kdl;


    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

    bool is_execut_trajectory{false};
    bool cancle_execut{false};
    bool finished_execut{false};

    
    Eigen::Vector<double, 6> dynamicCalc();
};

} // namespace mixcontroller