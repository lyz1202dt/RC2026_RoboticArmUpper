#include "control_pack/mixcontroller.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <memory>
#include <rclcpp/time.hpp>
#include <rclcpp_action/server.hpp>


namespace mixcontroller {

MixController::MixController() {}

controller_interface::CallbackReturn MixController::on_init() {
    RCLCPP_INFO(this->get_node()->get_logger(), "混合控制器初始化");
    param_node = std::make_shared<rclcpp::Node>("param_node");

    trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        get_node(), "robotic_arm_controller/arm_command", std::bind(&MixController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MixController::handle_cancel, this, std::placeholders::_1), std::bind(&MixController::handle_accepted, this, std::placeholders::_1)
    );

    result_msg   = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    feedback_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    size_t dof=joint_names_.size();
    q_kdl.resize(dof);
    dq_kdl.resize(dof);
    ddq_kdl.resize(dof);
    C_kdl.resize(dof);
    M_kdl.resize(dof);
    G_kdl.resize(dof);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MixController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    // TODO:加载并解析URDF
    RCLCPP_INFO(get_node()->get_logger(), "尝试解析URDF");
    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(param_node, "/robot_state_publisher");

    auto params = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml    = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return controller_interface::CallbackReturn::ERROR;
    }

    kdl_parser::treeFromString(urdf_xml, tree);
    tree.getChain("base_link", "link6", chain);

    gravity.x(0.0);
    gravity.y(0.0);
    gravity.z(-9.81);


    dyn = std::make_shared<KDL::ChainDynParam>(chain, gravity);

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MixController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(this->get_node()->get_logger(), "激活控制器");
    // reset_trajectory();
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MixController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(this->get_node()->get_logger(), "停用控制器");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MixController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (!is_execut_trajectory) {
        //RCLCPP_INFO(this->get_node()->get_logger(), "控制器更新(未发送)");
        return controller_interface::return_type::OK;
    }
    trajectory_msgs::msg::JointTrajectoryPoint output_state;
    // TODO:插值计算输出
    auto now = get_node()->get_clock()->now();

    while ((now - trajectory_start_time > current_trajectory_.points[trajectory_index_].time_from_start)) // 计算索引
    {
        if (trajectory_index_ < current_trajectory_.points.size())
            trajectory_index_++;
        else
        {
            trajectory_index_ =current_trajectory_.points.size()-1;
            break;
        }
    }


    // TODO:之后做更精确的插值
    output_state = current_trajectory_.points[trajectory_index_];



    for (size_t i = 0; i < joint_names_.size(); i++) // 填写轨迹位置/速度/加速度信息
    {
        q_kdl(i)   = output_state.positions[i];
        dq_kdl(i)  = output_state.velocities[i];
        ddq_kdl(i) = output_state.accelerations[i];
    }

    Eigen::Vector<double, 6> torque = dynamicCalc(); // 计算力矩前馈值

    for (size_t i = 0; i < joint_names_.size(); i++) // 将计算结果写入硬件层
    {
        command_interfaces_[i * 3 + 0].set_value(q_kdl(i));
        command_interfaces_[i * 3 + 1].set_value(dq_kdl(i));
        command_interfaces_[i * 3 + 2].set_value(torque(i));
    }


    // TODO:实时反馈
    feedback_msg->joint_names        = joint_names_;
    feedback_msg->desired.positions  = current_trajectory_.points[trajectory_index_].positions;
    feedback_msg->desired.velocities = current_trajectory_.points[trajectory_index_].velocities;

    feedback_msg->actual.positions.resize(joint_names_.size());
    feedback_msg->actual.velocities.resize(joint_names_.size());
    feedback_msg->actual.effort.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        feedback_msg->actual.positions[i]  = state_interfaces_[i * 2 + 0].get_value();
        feedback_msg->actual.velocities[i] = state_interfaces_[i * 2 + 1].get_value();
    }
    feedback_msg->header.stamp = get_node()->now();
    activate_goal_handle_->publish_feedback(feedback_msg);

    // 通知轨迹完成
    if (trajectory_index_ == current_trajectory_.points.size() - 1) { // 如果这点是最后一个点，那么发送完成状态，然后重置执行器状态
        is_execut_trajectory     = false;
        result_msg->error_code   = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        result_msg->error_string = "Trajectory finished";
        activate_goal_handle_->succeed(result_msg);
    }

    //RCLCPP_INFO(this->get_node()->get_logger(), "控制器更新");
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

rclcpp_action::GoalResponse MixController::handle_goal(
    const rclcpp_action::GoalUUID& uuid, const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal
) {
    if (is_execut_trajectory)
        return rclcpp_action::GoalResponse::REJECT;
    cancle_execut = false;

    current_trajectory_ = goal->trajectory;
    trajectory_index_   = 0;

    RCLCPP_INFO(this->get_node()->get_logger(), "接受轨迹");
    return rclcpp_action::GoalResponse ::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
    MixController::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
    is_execut_trajectory = false;
    cancle_execut        = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MixController::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
    is_execut_trajectory  = true;
    activate_goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_node()->get_logger(), "执行轨迹");
    trajectory_start_time = get_node()->get_clock()->now();
}

Eigen::Vector<double, 6> MixController::dynamicCalc() {
    // 5. 调用 KDL 动力学函数
    dyn->JntToMass(q_kdl, M_kdl);
    dyn->JntToCoriolis(q_kdl, dq_kdl, C_kdl);
    dyn->JntToGravity(q_kdl, G_kdl);

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    Eigen::Matrix<double, 6, 6> M_mat;
    Eigen::Matrix<double, 6, 1> C, G, ddq;

    for (int i = 0; i < 6; ++i) {
        C(i)   = C_kdl(i);
        G(i)   = G_kdl(i);
        ddq(i) = ddq_kdl(i);
        for (int j = 0; j < 6; ++j) {
            M_mat(i, j) = M_kdl(i, j);
        }
    }
    // 7. 计算前馈力矩 tau
    return (M_mat * ddq + C + G);
}


} // namespace mixcontroller

PLUGINLIB_EXPORT_CLASS(mixcontroller::MixController, controller_interface::ControllerInterface)
