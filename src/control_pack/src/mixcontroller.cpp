#include "control_pack/mixcontroller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace mixcontroller
{

MixController::MixController()
  : joint_trajectory_controller::JointTrajectoryController()
{
}

controller_interface::CallbackReturn MixController::on_init()
{
  RCLCPP_INFO(this->get_node()->get_logger(), "HybridJointTrajectoryController::on_init()");
  return joint_trajectory_controller::JointTrajectoryController::on_init();
}

controller_interface::CallbackReturn MixController::on_configure(
  const rclcpp_lifecycle::State& previous_state)
{
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MixController::on_activate(
  const rclcpp_lifecycle::State& previous_state)
{
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_activate(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Bind effort command interfaces
  effort_command_interfaces_.clear();
  for (const auto& joint : joint_names_) {
    for (auto& ci : command_interfaces_) {
      if (ci.get_name() == joint + "/effort") {
        effort_command_interfaces_.push_back(std::move(ci));
      }
    }
  }

  if (effort_command_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Not enough effort interfaces found");
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MixController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  // 1. Parent controller handles trajectory sampling and writes position commands
  auto ret = joint_trajectory_controller::JointTrajectoryController::update(time, period);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  // 2. Read current (state) and desired (command) values
  std::vector<double> q(joint_names_.size());
  std::vector<double> dq(joint_names_.size());
  std::vector<double> ddq(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const auto& si_pos = state_interfaces_[2 * i + 0];
    const auto& si_vel = state_interfaces_[2 * i + 1];
    q[i]  = si_pos.get_value();
    dq[i] = si_vel.get_value();

    // get desired acceleration from trajectory
    // already computed by parent
    //ddq[i] = computed_command_acc_[i];
  }

  // 3. Compute torque feedforward
  std::vector<double> tau_ff = compute_torque_ff(q, dq, ddq);

  // 4. Write torque commands
  // for (size_t i = 0; i < tau_ff.size(); ++i) {
  //   effort_command_interfaces_[i].set_value(tau_ff[i]);
  // }

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration
MixController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& name : joint_names_) {
    cfg.names.push_back(name + "/position");
    cfg.names.push_back(name + "/effort");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
MixController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto& name : joint_names_) {
    cfg.names.push_back(name + "/position");
    cfg.names.push_back(name + "/velocity");
  }
  return cfg;
}

// ------------------- dynamics model -------------------

std::vector<double> MixController::compute_torque_ff(
    const std::vector<double>& q,
    const std::vector<double>& dq,
    const std::vector<double>& ddq)
{
  // User should replace with actual dynamics model
  std::vector<double> tau(q.size(), 0.0);

  // For example: tau = M(q)*ddq + C(q,dq) + G(q);
  // Here, return zero for template.

  return tau;
}

}  // namespace mixcontroller

PLUGINLIB_EXPORT_CLASS(
  mixcontroller::MixController,
  controller_interface::ControllerInterface)
