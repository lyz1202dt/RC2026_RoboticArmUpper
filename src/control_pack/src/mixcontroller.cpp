#include "control_pack/mixcontroller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace mixcontroller {

MixController::MixController()
    : joint_trajectory_controller::JointTrajectoryController() {}

controller_interface::CallbackReturn MixController::on_init() {
    RCLCPP_INFO(this->get_node()->get_logger(), "HybridJointTrajectoryController::on_init()");
    return joint_trajectory_controller::JointTrajectoryController::on_init();
}

controller_interface::CallbackReturn MixController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    auto ret = joint_trajectory_controller::JointTrajectoryController::on_configure(previous_state);
    if (ret != controller_interface::CallbackReturn::SUCCESS) {
        return ret;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MixController::on_activate(const rclcpp_lifecycle::State& previous_state) {
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

controller_interface::return_type MixController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
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
        q[i]               = si_pos.get_value();
        dq[i]              = si_vel.get_value();

        // get desired acceleration from trajectory
        // already computed by parent
        // ddq[i] = computed_command_acc_[i];
    }

    // 3. Compute torque feedforward
    //std::vector<double> tau_ff = compute_torque_ff(q, dq, ddq);

    // 4. Write torque commands
    // for (size_t i = 0; i < tau_ff.size(); ++i) {
    //   effort_command_interfaces_[i].set_value(tau_ff[i]);
    // }

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration MixController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joint_names_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration MixController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joint_names_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
    }
    return cfg;
}


KDL::JntArray MixController::dynamicCalc(const KDL::JntArray& q, const KDL::JntArray& q_dot, const KDL::JntArray& q_ddot) {
    KDL::JntArray effort(6);
    // 1. 重力向量
    KDL::Vector gravity(0.0, 0.0, -9.81);
    // 2. KDL 动力学对象
    KDL::ChainDynParam dyn(chain, gravity);


    // trajectory_ff.header.frame_id=msg->header.frame_id;
    // trajectory_ff.header.stamp=this->node->get_clock()->now();
    // trajectory_ff.joint_names=msg->joint_names;
    // trajectory_ff.points=msg->points;

    // 4. KDL 动力学计算输出
    KDL::JntSpaceInertiaMatrix M_kdl(6);
    KDL::JntArray C_kdl(6), G_kdl(6);

    // 5. 调用 KDL 动力学函数
    dyn.JntToMass(q, M_kdl);
    dyn.JntToCoriolis(q, q_dot, C_kdl);
    dyn.JntToGravity(q, G_kdl);

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    Eigen::Matrix<double, 6, 6> M_mat;
    Eigen::Matrix<double, 6, 1> C, G, qddot;

    for (int i = 0; i < 6; ++i) {
        C(i)     = C_kdl(i);
        G(i)     = G_kdl(i);
        qddot(i) = q_ddot(i);
        for (int j = 0; j < 6; ++j) {
            M_mat(i, j) = M_kdl(i, j);
        }
    }

    // 7. 计算前馈力矩 tau
    auto tau = M_mat * qddot + C + G;

    for (int i = 0; i < 6; i++) {
        effort(i) = tau[i]; // 赋值给轨迹上的点
    }
    return effort;
}



} // namespace mixcontroller

PLUGINLIB_EXPORT_CLASS(mixcontroller::MixController, controller_interface::ControllerInterface)
