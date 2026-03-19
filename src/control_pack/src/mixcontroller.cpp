#include "control_pack/mixcontroller.hpp" 
#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <memory>
#include <rclcpp/time.hpp>
#include <rclcpp_action/server.hpp>


namespace mixcontroller {

// 一.五次多项式轨迹参数类实现
void QuinticParam::set_param(
    const double t0, const double t1, const double p0, const double v0, const double a0, const double pt, const double v1, const double at
) {
    double T  = t1 - t0; // 轨迹持续时间
    double T2 = T * T;  // 二次
    double T3 = T2 * T; // 三次
    double T4 = T3 * T; // 四次
    double T5 = T4 * T; // 五次

    // 五次多项式求解啊
    f = p0; // 常数项设为起点位置
    e = v0; // 将一次项系数设为起点速度
    d = a0 / 2.0;   // 将二次项系数设为起点加速度的一半
    a = (12 * (pt - p0) - 6 * (v1 + v0) * T - (at - a0) * T2) / (2 * T5);
    b = (-30 * (pt - p0) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * at) * T2) / (2 * T4);
    c = (20 * (pt - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - at) * T2) / (2 * T3);

    this->t0 = t0;
    this->t1 = t1;
}

// 计算任意时刻 t 处的轨迹位置值
double QuinticParam::get_pos(const double t) {
    if (t <= t0)     // 边界返回起点位置
        return f;
    if (t >= t1) {  // 边界返回终点位置     
        const double T = t1 - t0;
        return (a * T * T * T * T * T + b * T * T * T * T + c * T * T * T + d * T * T + e * T + f);
    }

    const double tau = t - t0;
    return (a * tau * tau * tau * tau * tau + b * tau * tau * tau * tau + c * tau * tau * tau + d * tau * tau + e * tau + f);
}

// 计算任意时刻 t 处的轨迹速度值
double QuinticParam::get_vel(const double t) {
    if (t <= t0)
        return e;
    if (t >= t1) {
        const double T = t1 - t0;
        return (5 * a * T * T * T * T + 4 * b * T * T * T + 3 * c * T * T + 2 * d * T + e);
    }

    const double tau = t - t0;
    return (5 * a * tau * tau * tau * tau + 4 * b * tau * tau * tau + 3 * c * tau * tau + 2 * d * tau + e);
}

// 计算任意时刻 t 处的轨迹加速度值
double QuinticParam::get_acc(const double t) {
    if (t <= t0)
        return 2.0 * d;
    if (t >= t1) {
        const double T = t1 - t0;
        return (20 * a * T * T * T + 12 * b * T * T + 6 * c * T + 2 * d);
    }

    const double tau = t - t0;
    return (20 * a * tau * tau * tau + 12 * b * tau * tau + 6 * c * tau + 2 * d);
}

// 将当前轨迹段索引 cur_index 初始化为零
ContinuousTrajectory::ContinuousTrajectory() { cur_index = 0; }

// 根据当前时间查询并计算轨迹插值结果
bool ContinuousTrajectory::get_target(const rclcpp::Time& time, trajectory_msgs::msg::JointTrajectoryPoint& output) {
    bool success = true;
    auto dt      = time - start_time; // 时间间隔
    while (dt >= trajectory.points[cur_index].time_from_start) { // 已经经过的时间大于当前轨迹点的时间戳
        // time_from_start 是相对于轨迹开始时间的时间戳,相对于轨迹起点的时间偏移量。

        cur_index++;
        if (cur_index == trajectory.points.size())
            break;

        // 将 ROS 2 的 Duration 类型（秒和纳秒分离存储）转换为双精度浮点数（总秒数）。
        double t0 = trajectory.points[cur_index - 1].time_from_start.sec + trajectory.points[cur_index - 1].time_from_start.nanosec * 1e-9;
        double t1 = trajectory.points[cur_index].time_from_start.sec + trajectory.points[cur_index].time_from_start.nanosec * 1e-9;
        
        for (int i = 0; i < 6; i++) {
            auto& P0 = trajectory.points[cur_index - 1]; // 轨迹段的起点
            auto& PT = trajectory.points[cur_index]; // 轨迹段的终点

            // 为单个关节在指定时间区间内创建满足边界条件的五次多项式轨迹。
            // 计算每个时间点的系数（位置，速度，加速度）
            line[i].set_param(t0, t1, P0.positions[i], P0.velocities[i], P0.accelerations[i], PT.positions[i], PT.velocities[i], PT.accelerations[i]);
        }
    }
    if (trajectory.points.size() == cur_index) // 轨迹已经执行完毕
        return false;
    for (int i = 0; i < 6; i++)                // 计算插值结果
    {
        output.positions[i]     = line[i].get_pos(dt.seconds());
        output.velocities[i]    = line[i].get_vel(dt.seconds());
        output.accelerations[i] = line[i].get_acc(dt.seconds());
    }
    return success;
}

void ContinuousTrajectory::start_track(rclcpp::Time now) {
    // std::move 把目标转换成
    this->start_time = std::move(now); // 记录轨迹开始时间
    cur_index        = 0; // 从第一个轨迹点开始
}

// 把要执行点的轨迹保存到控制器里
void ContinuousTrajectory::set_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) { this->trajectory = trajectory; }

// 创建一个 ROS 2 节点，用于获取机器人参数
MixController::MixController() { param_node = std::make_shared<rclcpp::Node>("param_node"); }

// 控制器初始化：创建 Action 服务器、分配内存、准备数据结构
controller_interface::CallbackReturn MixController::on_init() {

    // get_node(), 获取ros2节点指针
    RCLCPP_INFO(this->get_node()->get_logger(), "混合控制器初始化");

    // 作用：创建一个服务器，接收轨迹命令
    trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        get_node(), "robotic_arm_controller/arm_command", std::bind(&MixController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MixController::handle_cancel, this, std::placeholders::_1), std::bind(&MixController::handle_accepted, this, std::placeholders::_1)
    );
    // 实时消息
    result_msg   = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    feedback_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
    // 设置关节名称
    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    // 为 6 个关节准备存储空间
    size_t dof = joint_names_.size(); // dof = 6（自由度数量）
    q_kdl.resize(dof); // 关节位置
    dq_kdl.resize(dof); // 关节速度
    ddq_kdl.resize(dof); // 关节加速度
    C_kdl.resize(dof); // 科里奥利力
    M_kdl.resize(dof); // 惯性矩阵
    G_kdl.resize(dof); // 重力

    // 存储插值计算的结果，resize（把向量的大小调整成指定值），
    output_state.positions.resize(dof);
    output_state.velocities.resize(dof);
    output_state.accelerations.resize(dof);

    feedback_msg->actual.positions.resize(joint_names_.size());
    feedback_msg->actual.velocities.resize(joint_names_.size());
    feedback_msg->actual.effort.resize(joint_names_.size());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MixController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    // TODO:加载并解析URDF
    RCLCPP_INFO(get_node()->get_logger(), "尝试解析URDF");

    // 参数客户端
    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(param_node, "/robot_state_publisher");

    // 获取urdf
    auto params = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml    = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return controller_interface::CallbackReturn::ERROR;
    }

    // 解析urdf
    kdl_parser::treeFromString(urdf_xml, tree); // 构建 KDL 树（机器人的运动学结构）
    tree.getChain("base_link", "link6", chain); // 提取运动链（从基座到末端的关节-连杆顺序）

    // 设置重力
    gravity.x(0.0);
    gravity.y(0.0);
    gravity.z(-9.81);

    // 创建一个动力学计算器的对象
    dyn = std::make_shared<KDL::ChainDynParam>(chain, gravity); // 调整电机输出

    return controller_interface::CallbackReturn::SUCCESS;
}

// 激活控制器
controller_interface::CallbackReturn MixController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_node()->get_logger(), "激活控制器");
    return controller_interface::CallbackReturn::SUCCESS;
}

// 停用控制器
controller_interface::CallbackReturn MixController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    RCLCPP_INFO(this->get_node()->get_logger(), "停用控制器");
    return controller_interface::CallbackReturn::SUCCESS;
}

// 控制器主循环
controller_interface::return_type MixController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // 没有轨迹要执行
    if (!is_execut_trajectory) {
        // RCLCPP_INFO(this->get_node()->get_logger(), "控制器更新(未发送)");
        return controller_interface::return_type::OK;
    }

    // 五次多项式插值计算输出
    // get_target 读取当前播放位置
    bool ret = continue_trajectory.get_target(time, output_state);

    // 填充 KDL 数据结构
    for (size_t i = 0; i < joint_names_.size(); i++) // 填写轨迹位置/速度/加速度信息
    {
        q_kdl(i)   = output_state.positions[i];
        dq_kdl(i)  = output_state.velocities[i];
        ddq_kdl(i) = output_state.accelerations[i];
    }

    // 动力学计算
    Eigen::Vector<double, 6> torque = dynamicCalc(); // 计算力矩前馈值，计算所需力矩大小

    for (size_t i = 0; i < joint_names_.size(); i++) // 将计算结果写入硬件层
    {
        command_interfaces_[i * 3 + 0].set_value(q_kdl(i));         // 写入位置
        command_interfaces_[i * 3 + 1].set_value(dq_kdl(i));        // 写入速度
        command_interfaces_[i * 3 + 2].set_value(torque(i)); // 写入力矩
    }


    // TODO:实时反馈
    feedback_msg->joint_names        = joint_names_;
    feedback_msg->desired.positions  = output_state.positions;
    feedback_msg->desired.velocities = output_state.velocities;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        feedback_msg->actual.positions[i]  = state_interfaces_[i * 2 + 0].get_value();
        feedback_msg->actual.velocities[i] = state_interfaces_[i * 2 + 1].get_value();
    }
    feedback_msg->header.stamp = get_node()->now();
    activate_goal_handle_->publish_feedback(feedback_msg);

    // 通知轨迹完成
    if (!ret) { // 如果这点是最后一个点，那么发送完成状态
        is_execut_trajectory     = false;
        result_msg->error_code   = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        result_msg->error_string = "Trajectory finished";
        activate_goal_handle_->succeed(result_msg);
    }

    // RCLCPP_INFO(this->get_node()->get_logger(), "控制器更新");
    return controller_interface::return_type::OK;
}

// 说明控制器需要的命令接口
controller_interface::InterfaceConfiguration MixController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg; // 创建配置对象
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL; // 逐个声明

    // 在update中，把命令写入接口
    for (const auto& name : joint_names_) {
        cfg.names.push_back(name + "/position"); // 位置接口
        cfg.names.push_back(name + "/velocity"); // 速度接口
        cfg.names.push_back(name + "/effort");   // 力矩接口
    }
    return cfg;
}

// 说明控制器需要的状态读取接口
controller_interface::InterfaceConfiguration MixController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joint_names_) {
        cfg.names.push_back(name + "/position"); // 读取实际位置
        cfg.names.push_back(name + "/velocity"); // 读取实际速度
    }
    return cfg;
}

// 处理轨迹执行请求
rclcpp_action::GoalResponse MixController::handle_goal(
    const rclcpp_action::GoalUUID& uuid, // 目标唯一标识符
    const std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal // 目标内容
) {
    // 如果正在执行其他轨迹，拒绝新目标
    if (is_execut_trajectory)
        return rclcpp_action::GoalResponse::REJECT;

    // 重置取消执行
    cancle_execut = false;

    // 将目标中的轨迹数据保存到控制器
    // set_trajectory 接收轨迹
    continue_trajectory.set_trajectory(goal->trajectory);            // 设置要执行的轨迹

    RCLCPP_INFO(this->get_node()->get_logger(), "接收轨迹");
    return rclcpp_action::GoalResponse ::ACCEPT_AND_EXECUTE;
}

// 处理轨迹取消请求
rclcpp_action::CancelResponse
    MixController::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
    // 标记未使用的参数
    (void)goal_handle;

    // 停止轨迹执行
    is_execut_trajectory = false;

    // 标记已取消
    cancle_execut        = true;

    // 接受取消请求
    return rclcpp_action::CancelResponse::ACCEPT;
}


// 开始执行轨迹
void MixController::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
    // 标记开始执行轨迹
    is_execut_trajectory  = true;

    // 保存目标句柄（用于后续发送反馈和结果）
    activate_goal_handle_ = goal_handle;

    RCLCPP_INFO(this->get_node()->get_logger(), "执行轨迹");

    // 设置轨迹开始时间为当前时间
    // start_track 执行轨迹
    continue_trajectory.start_track(get_node()->get_clock()->now()); // 开始执行轨迹
}

// 计算机器人动力学，计算前馈力矩
Eigen::Vector<double, 6> MixController::dynamicCalc() {
    // 5. 调用 KDL 动力学函数
    dyn->JntToMass(q_kdl, M_kdl); // 计算惯性矩阵
    dyn->JntToCoriolis(q_kdl, dq_kdl, C_kdl); // 计算科里奥利力
    dyn->JntToGravity(q_kdl, G_kdl); // 计算重力

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    Eigen::Matrix<double, 6, 6> M_mat; // 惯性矩阵
    Eigen::Matrix<double, 6, 1> C, G, ddq; // 向量

    for (int i = 0; i < 6; ++i) {
        C(i)   = C_kdl(i); // 科里奥利力
        G(i)   = G_kdl(i); // 重力
        ddq(i) = ddq_kdl(i); // 加速度
        for (int j = 0; j < 6; ++j) {
            M_mat(i, j) = M_kdl(i, j); // 惯性矩阵元素
        }
    }
    // 7. 计算前馈力矩 tau
    // 计算前馈力矩：τ = M·ddq + C + G
    return (M_mat * ddq + C + G);
}


} // namespace mixcontroller

// 把控制器类导出为 ROS 2 插件
PLUGINLIB_EXPORT_CLASS(mixcontroller::MixController, controller_interface::ControllerInterface)
