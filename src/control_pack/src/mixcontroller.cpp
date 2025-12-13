#include "control_pack/mixcontroller.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <controller_interface/controller_interface_base.hpp>
#include <cstddef>
#include <joint_trajectory_controller/interpolation_methods.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace mixcontroller {

MixController::MixController()
    : joint_trajectory_controller::JointTrajectoryController() {}

controller_interface::CallbackReturn MixController::on_init() {
    RCLCPP_INFO(this->get_node()->get_logger(), "混合控制器初始化");
    param_node = std::make_shared<rclcpp::Node>("param_node");
    return joint_trajectory_controller::JointTrajectoryController::on_init();
}

controller_interface::CallbackReturn MixController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    auto ret = joint_trajectory_controller::JointTrajectoryController::on_configure(previous_state);
    if (ret != controller_interface::CallbackReturn::SUCCESS) {
        return ret;
    }
    // TODO:加载并解析URDF
    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(param_node, "/robot_state_publisher");

    auto params = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml    = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return controller_interface::CallbackReturn::ERROR;
    }

    kdl_parser::treeFromString(urdf_xml, tree);
    tree.getChain("base_link", "link6", chain);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MixController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    auto ret = joint_trajectory_controller::JointTrajectoryController::on_activate(previous_state);
    if (ret != controller_interface::CallbackReturn::SUCCESS) {
        return ret;
    }

    // 从父类维护的command_interfaces_拿到力矩接口，方便自行控制
    effort_command_interfaces_.clear();
    velocity_command_interfaces_.clear();
    for (const auto& joint : joint_names_) {
        for (auto& ci : command_interfaces_) {
            if (ci.get_name() == joint + "/effort") {
                effort_command_interfaces_.push_back(std::move(ci));
            } else if (ci.get_name() == joint + "/velocity") {
                velocity_command_interfaces_.push_back(std::move(ci));
            }
        }
    }

    if (effort_command_interfaces_.size() != joint_names_.size() || velocity_command_interfaces_.size() != joint_names_.size()) { // 确认是否拿全
        RCLCPP_ERROR(this->get_node()->get_logger(), "控制器接口不全");
        return controller_interface::CallbackReturn::ERROR;
    }

    segment_start_ = current_trajectory_->begin();
    segment_end_   = std::next(segment_start_);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MixController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {

    auto ret = joint_trajectory_controller::JointTrajectoryController::update(time, period); // 父类执行一次update维护轨迹更新
    if (ret != controller_interface::return_type::OK) {
        return ret;
    }


    trajectory_msgs::msg::JointTrajectoryPoint output_state;
    auto valid = current_trajectory_->sample(
        time, joint_trajectory_controller::interpolation_methods::InterpolationMethod::VARIABLE_DEGREE_SPLINE, output_state, segment_start_,
        segment_end_
    );

    if (!valid) {
        return controller_interface::return_type::OK;
    }

    KDL::JntArray q(joint_names_.size());
    KDL::JntArray dq(joint_names_.size());
    KDL::JntArray ddq(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); i++)                                         // 填写轨迹位置/速度/加速度信息
    {
        q(i)   = output_state.positions[i];
        dq(i)  = output_state.velocities[i];
        ddq(i) = output_state.accelerations[i];
    }

    auto effort = dynamicCalc(q, dq, ddq);                                                   // 计算力矩前馈值

    for (size_t i = 0; i < joint_names_.size(); i++)                                         // 将计算结果写入硬件层
    {
        velocity_command_interfaces_[i].set_value(dq(i));
        effort_command_interfaces_[i].set_value(effort(i));
    }

    RCLCPP_INFO(this->get_node()->get_logger(), "控制器更新");
    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration MixController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joint_names_) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
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
