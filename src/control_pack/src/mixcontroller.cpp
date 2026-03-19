#include "control_pack/mixcontroller.hpp" 
#include "control_pack/realtime_ik_solver.hpp"
#include "control_pack/twist_to_trajectory.hpp"
#include "control_pack/safety_checker.hpp"
#include "control_pack/diagnostics_publisher.hpp"
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

    // 第三阶段初始化将在 on_configure 中完成（需要 KDL chain）
    
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

    // 第三阶段：初始化 IK 求解器和相关组件
    // 1. 创建实时 IK 求解器
    ik_solver_ = std::make_shared<RealtimeIKSolver>(chain, get_node()->get_logger());
    
    // 2. 创建 Twist 到轨迹的转换器
    twist_converter_ = std::make_shared<TwistToTrajectoryConverter>(
        ik_solver_, 0.01, get_node()->get_logger());
    
    // 3. 创建安全检查器
    safety_checker_ = std::make_shared<SafetyChecker>(6, get_node()->get_logger());
    
    // 4. 创建诊断发布器
    diagnostics_publisher_ = std::make_shared<DiagnosticsPublisher>(get_node());
    
    // 5. 订阅 Twist 命令话题
    twist_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "arm_controller/twist_command", 
        1,
        std::bind(&MixController::twist_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(get_node()->get_logger(), "第三阶段组件初始化完成");

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
    // 检查平滑停止标志
    if (smooth_stop_flag_) {
        // 生成减速轨迹
        std::vector<double> current_positions(output_state.positions);
        std::vector<double> current_velocities(output_state.velocities);
        auto decel_traj = generate_deceleration_trajectory(current_positions, current_velocities, 0.2);
        
        // 清空缓冲队列，只执行减速轨迹
        {
            std::lock_guard<std::mutex> lock(trajectory_queue_mutex_);
            trajectory_queue_.clear();
        }
        
        continue_trajectory.set_trajectory(decel_traj);
        continue_trajectory.start_track(time);
        smooth_stop_flag_ = false;
        is_execut_trajectory = true;
        
        RCLCPP_INFO(this->get_node()->get_logger(), "执行平滑停止减速轨迹");
    }

    // 没有轨迹要执行
    if (!is_execut_trajectory) {
        return controller_interface::return_type::OK;
    }

    // 五次多项式插值计算输出
    bool ret = continue_trajectory.get_target(time, output_state);

    // 填充 KDL 数据结构
    for (size_t i = 0; i < joint_names_.size(); i++) {
        q_kdl(i)   = output_state.positions[i];
        dq_kdl(i)  = output_state.velocities[i];
        ddq_kdl(i) = output_state.accelerations[i];
    }

    // 动力学计算
    Eigen::Vector<double, 6> torque = dynamicCalc();

    for (size_t i = 0; i < joint_names_.size(); i++) {
        command_interfaces_[i * 3 + 0].set_value(q_kdl(i));
        command_interfaces_[i * 3 + 1].set_value(dq_kdl(i));
        command_interfaces_[i * 3 + 2].set_value(torque(i));
    }

    // 实时反馈
    feedback_msg->joint_names        = joint_names_;
    feedback_msg->desired.positions  = output_state.positions;
    feedback_msg->desired.velocities = output_state.velocities;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        feedback_msg->actual.positions[i]  = state_interfaces_[i * 2 + 0].get_value();
        feedback_msg->actual.velocities[i] = state_interfaces_[i * 2 + 1].get_value();
    }
    feedback_msg->header.stamp = get_node()->now();
    activate_goal_handle_->publish_feedback(feedback_msg);

    // 轨迹完成时的处理
    if (!ret) {
        load_next_trajectory_from_queue();
        
        // 如果没有更多轨迹，发送完成反馈
        if (!is_execut_trajectory) {
            result_msg->error_code   = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
            result_msg->error_string = "Trajectory finished";
            activate_goal_handle_->succeed(result_msg);
            RCLCPP_INFO(this->get_node()->get_logger(), "所有轨迹执行完成");
        }
    }

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
    // 重置取消执行标志
    cancle_execut = false;
    smooth_stop_flag_ = false;

    // 尝试追加轨迹到缓冲队列
    if (!append_trajectory(goal->trajectory)) {
        RCLCPP_WARN(this->get_node()->get_logger(), "轨迹缓冲队列已满，拒绝新 Goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 如果当前没有执行轨迹，立即启动
    if (!is_execut_trajectory) {
        is_execut_trajectory = true;
        continue_trajectory.set_trajectory(goal->trajectory);
        RCLCPP_INFO(this->get_node()->get_logger(), "立即启动新轨迹执行");
    } else {
        RCLCPP_INFO(this->get_node()->get_logger(), "轨迹已追加到缓冲队列 (队列大小: %zu)", get_queue_size());
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// 处理轨迹取消请求
rclcpp_action::CancelResponse
    MixController::handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
    // 标记未使用的参数
    (void)goal_handle;

    // 设置平滑停止标志（而非立即硬停）
    smooth_stop_flag_ = true;

    RCLCPP_INFO(this->get_node()->get_logger(), "启动平滑停止流程");

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

// 轨迹缓冲队列管理函数
bool MixController::append_trajectory(const trajectory_msgs::msg::JointTrajectory& trajectory) {
    std::lock_guard<std::mutex> lock(trajectory_queue_mutex_);
    
    // 检查缓冲队列是否满
    if (trajectory_queue_.size() >= MAX_BUFFER_SIZE) {
        RCLCPP_WARN(this->get_node()->get_logger(), 
            "轨迹缓冲队列已满 (大小: %zu/%zu)，丢弃最旧的轨迹", 
            trajectory_queue_.size(), MAX_BUFFER_SIZE);
        trajectory_queue_.pop_front();
    }
    
    trajectory_queue_.push_back(trajectory);
    return true;
}

size_t MixController::get_queue_size() const {
    std::lock_guard<std::mutex> lock(trajectory_queue_mutex_);
    return trajectory_queue_.size();
}

// 加载缓冲队列中的下一个轨迹
void MixController::load_next_trajectory_from_queue() {
    std::lock_guard<std::mutex> lock(trajectory_queue_mutex_);
    
    if (trajectory_queue_.empty()) {
        is_execut_trajectory = false;
        RCLCPP_INFO(this->get_node()->get_logger(), "缓冲队列为空，轨迹执行结束");
        return;
    }
    
    auto next_trajectory = trajectory_queue_.front();
    trajectory_queue_.pop_front();
    
    // 生成过渡轨迹：当前位置 → 下一轨迹起点
    std::vector<double> current_pos = output_state.positions;
    std::vector<double> next_start_pos = next_trajectory.points[0].positions;
    
    trajectory_msgs::msg::JointTrajectory transition_traj;
    transition_traj.joint_names = joint_names_;
    
    // 生成 10 个过渡点（100ms），每个 10ms
    int num_transition_points = static_cast<int>(trajectory_transition_time_ * 100);
    transition_traj.points.resize(num_transition_points);
    
    for (int i = 0; i < num_transition_points; ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        double alpha = static_cast<double>(i) / num_transition_points;
        
        point.positions.resize(6);
        point.velocities.resize(6);
        point.accelerations.resize(6);
        
        for (size_t j = 0; j < 6; ++j) {
            // 线性插值位置
            point.positions[j] = current_pos[j] + alpha * (next_start_pos[j] - current_pos[j]);
            // 速度和加速度初始化为 0
            point.velocities[j] = 0.0;
            point.accelerations[j] = 0.0;
        }
        
        point.time_from_start.sec = i / 100;
        point.time_from_start.nanosec = (i % 100) * 10000000;  // 10ms in nanoseconds
        transition_traj.points[i] = point;
    }
    
    // 执行过渡轨迹然后是下一个轨迹
    // 先执行过渡，完成后会自动加载下一个
    continue_trajectory.set_trajectory(transition_traj);
    continue_trajectory.start_track(get_node()->get_clock()->now());
    
    // 将原来的下一个轨迹重新放回队列，待过渡完成后执行
    trajectory_queue_.push_front(next_trajectory);
    
    RCLCPP_INFO(this->get_node()->get_logger(), "加载过渡轨迹，剩余缓冲: %zu", trajectory_queue_.size());
}

// 生成减速轨迹，用于平滑停止
trajectory_msgs::msg::JointTrajectory MixController::generate_deceleration_trajectory(
    const std::vector<double>& current_positions,
    const std::vector<double>& current_velocities,
    double deceleration_time
) {
    trajectory_msgs::msg::JointTrajectory decel_traj;
    decel_traj.joint_names = joint_names_;
    
    int num_points = static_cast<int>(deceleration_time * 100);  // 100Hz for 0.2s = 20 points
    decel_traj.points.resize(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        double time_ratio = static_cast<double>(i) / num_points;
        double velocity_scale = 1.0 - time_ratio;  // 线性衰减速度
        
        point.positions.resize(6);
        point.velocities.resize(6);
        point.accelerations.resize(6);
        
        for (size_t j = 0; j < 6; ++j) {
            // 位置：当前位置 + 速度 * 时间（速度线性衰减）
            point.positions[j] = current_positions[j] + 
                                 current_velocities[j] * time_ratio * (2.0 - time_ratio) * deceleration_time;
            // 速度线性衰减到 0
            point.velocities[j] = current_velocities[j] * velocity_scale;
            // 加速度：(-v/T)
            point.accelerations[j] = -current_velocities[j] / deceleration_time;
        }
        
        point.time_from_start.sec = i / 100;
        point.time_from_start.nanosec = (i % 100) * 10000000;
        decel_traj.points[i] = point;
    }
    
    RCLCPP_INFO(this->get_node()->get_logger(), "生成 %d 个减速轨迹点，共 %.1f 秒", num_points, deceleration_time);
    return decel_traj;
}

// Twist 回调函数
void MixController::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    {
        std::lock_guard<std::mutex> lock(twist_mutex_);
        latest_twist_ = *msg;
        twist_enabled_ = true;
    }
    
    // 生成并追加微轨迹
    generate_and_queue_twist_trajectory();
}

// 生成并追加 Twist 轨迹到缓冲队列
void MixController::generate_and_queue_twist_trajectory() {
    if (!twist_enabled_ || !ik_solver_ || !twist_converter_) {
        return;
    }

    try {
        // 记录 IK 求解时间
        auto ik_start = std::chrono::steady_clock::now();

        // 使用 Twist 转换器生成微轨迹
        geometry_msgs::msg::Twist current_twist;
        {
            std::lock_guard<std::mutex> lock(twist_mutex_);
            current_twist = latest_twist_;
        }

        auto micro_traj = twist_converter_->convert_twist_to_micro_trajectory(
            current_twist,
            q_kdl,
            dq_kdl,
            joint_names_,
            3  // 生成 3 个轨迹点
        );

        // 计算 IK 求解时间
        auto ik_end = std::chrono::steady_clock::now();
        last_ik_solve_time_ms_ = std::chrono::duration<double, std::milli>(ik_end - ik_start).count();

        if (!micro_traj.points.empty()) {
            // 追加到缓冲队列
            append_trajectory(micro_traj);
            
            // 如果没有执行轨迹，立即启动
            if (!is_execut_trajectory) {
                is_execut_trajectory = true;
                continue_trajectory.set_trajectory(micro_traj);
                continue_trajectory.start_track(get_node()->get_clock()->now());
                RCLCPP_DEBUG(get_node()->get_logger(), "启动 Twist 轨迹执行");
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(get_node()->get_logger(), "生成 Twist 轨迹出错: %s", e.what());
    }
}


} // namespace mixcontroller

// 把控制器类导出为 ROS 2 插件
PLUGINLIB_EXPORT_CLASS(mixcontroller::MixController, controller_interface::ControllerInterface)
