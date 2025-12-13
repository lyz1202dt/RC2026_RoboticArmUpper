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
#include <string>
#include <vector>

namespace mixcontroller {

class MixController : public joint_trajectory_controller::JointTrajectoryController {
public:
    MixController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:

    KDL::Tree tree;
    KDL::Chain chain;
    std::string urdf_xml;
    rclcpp::Node::SharedPtr param_node;

    joint_trajectory_controller::TrajectoryPointConstIter segment_start_;
    joint_trajectory_controller::TrajectoryPointConstIter segment_end_;

    rclcpp::SyncParametersClient::SharedPtr robot_description_param_;
    KDL::JntArray dynamicCalc(const KDL::JntArray& q, const KDL::JntArray& q_dot, const KDL::JntArray& q_ddot);
    
    std::vector<hardware_interface::LoanedCommandInterface> velocity_command_interfaces_;
    std::vector<hardware_interface::LoanedCommandInterface> effort_command_interfaces_;

    // Store joint names locally
    std::vector<std::string> joint_names_;

};

} // namespace mixcontroller