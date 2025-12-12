#pragma once

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

    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    // effort command handles
    std::vector<hardware_interface::LoanedCommandInterface> effort_command_interfaces_;

    // Store joint names locally
    std::vector<std::string> joint_names_;

    // ---- your dynamics model entry point ----
    std::vector<double> compute_torque_ff(const std::vector<double>& q, const std::vector<double>& dq, const std::vector<double>& ddq);
};

} // namespace mixcontroller