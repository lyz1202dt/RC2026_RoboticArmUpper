#include "control_pack/mixcontroller.hpp" 
#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/server.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <kdl/chainjnttojacsolver.hpp>


#include <kdl/chainiksolvervel_pinv.hpp>
#include <robot_interfaces/msg/detail/moveit__struct.hpp>


namespace mixcontroller {

namespace {

constexpr char kStateTopic[] = "myjoints_state";
constexpr char kTargetTopic[] = "myjoints_target";

}  // namespace

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
    // 检查轨迹是否为空
    if (trajectory.points.empty()) {
        return false;
    }
    
    bool success = true;
    auto dt      = time - start_time; // 时间间隔
    
    // 先检查 cur_index 是否越界，再访问 trajectory.points[cur_index]
    while (cur_index < trajectory.points.size() && dt >= trajectory.points[cur_index].time_from_start) {
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
    
    // 确保 output 向量有足够空间
    if (output.positions.size() < 6) output.positions.resize(6);
    if (output.velocities.size() < 6) output.velocities.resize(6);
    if (output.accelerations.size() < 6) output.accelerations.resize(6);
    
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
MixController::MixController() { 
    // 注意：订阅在 on_init() 中创建，而不是在构造函数中
    // 这样可以确保订阅使用的是控制器的节点
}

// 控制器初始化：创建 Action 服务器、分配内存、准备数据结构
controller_interface::CallbackReturn MixController::on_init() {

    // get_node(), 获取ros2节点指针
    RCLCPP_INFO(this->get_node()->get_logger(), "混合控制器初始化");
    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    if (!this->get_node()->has_parameter("control_mode")) {
        this->get_node()->declare_parameter<std::string>("control_mode", "moveit");
    }
    const std::string control_mode = this->get_node()->get_parameter("control_mode").as_string();
    mujoco_mode_ = (control_mode == "mujoco");
    RCLCPP_INFO(this->get_node()->get_logger(), "control_mode=%s, mujoco_mode=%d", control_mode.c_str(), mujoco_mode_ ? 1 : 0);
    if (!this->get_node()->has_parameter("joint_torque_filter_gate")) {
        this->get_node()->declare_parameter<double>("joint_torque_filter_gate", 0.8);
    }
    if (!this->get_node()->has_parameter("joint_omega_filter_gate")) {
        this->get_node()->declare_parameter<double>("joint_omega_filter_gate", 0.8);
    }
    if (!this->get_node()->has_parameter("command_effort_limit")) {
        this->get_node()->declare_parameter<double>("command_effort_limit", 80.0);
    }
    joint_kp_.resize(joint_names_.size(), 50.0);
    joint_kd_.resize(joint_names_.size(), 3.0);
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const std::string kp_name = "joint" + std::to_string(i + 1) + "_kp";
        const std::string kd_name = "joint" + std::to_string(i + 1) + "_kd";
        if (!this->get_node()->has_parameter(kp_name)) {
            this->get_node()->declare_parameter<double>(kp_name, 50.0);
        }
        if (!this->get_node()->has_parameter(kd_name)) {
            this->get_node()->declare_parameter<double>(kd_name, 3.0);
        }
    }

    // 专用于参数查询的独立节点，避免在已托管控制器节点上触发 executor 冲突
    param_query_node_ = std::make_shared<rclcpp::Node>("robotic_arm_controller_param_client");

    // 创建订阅（在 on_init 时创建，确保使用控制器的节点和 executor）
    twist_subscriber_ = this->get_node()->create_subscription<geometry_msgs::msg::Twist>(
        "twist_command", 10, 
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            // 处理接收到的 Twist 消息
            RCLCPP_DEBUG_THROTTLE(
                this->get_node()->get_logger(),
                *this->get_node()->get_clock(),
                3000,
                "Received Twist command: linear=(%f, %f, %f), angular=(%f, %f, %f)",
                msg->linear.x,
                msg->linear.y,
                msg->linear.z,
                msg->angular.x,
                msg->angular.y,
                msg->angular.z
            );
            twist_command_ = *msg; // 保存接收到的 Twist 命令
            kdl_twist_command_.vel = KDL::Vector(twist_command_.linear.x, twist_command_.linear.y, twist_command_.linear.z);
            kdl_twist_command_.rot = KDL::Vector(twist_command_.angular.x, twist_command_.angular.y, twist_command_.angular.z);
            auto ik_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain); // 创建逆运动学求解器
            const size_t state_stride = mujoco_mode_ ? 3 : 2;
            if (state_interfaces_.size() < joint_names_.size() * state_stride) {
                return;
            }

            KDL::JntArray q_current(joint_names_.size()); // 当前关节位置
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                q_current(i) = state_interfaces_[i * state_stride + 0].get_value(); // 从状态接口获取当前关节位置
            }

            if (q_dot_.rows() != static_cast<unsigned int>(joint_names_.size())) {
                q_dot_.resize(joint_names_.size());
            }

            int ik_result = ik_solver_->CartToJnt(q_current, kdl_twist_command_, q_dot_); // 计算逆运动学，得到关节速度命令
            if (ik_result < 0) {
                RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to compute IK solution for the given twist command");
                return;
            }
        });

    initial_joint_trajectory_subscriber_ = this->get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "initial_joint_trajectory", 10,
        [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
             RCLCPP_INFO_THROTTLE(
                this->get_node()->get_logger(),
                *this->get_node()->get_clock(),
                3000,
                "[接收到轨迹消息] header.stamp=(sec=%d, nsec=%u), joint_names.size=%zu, points.size=%zu",
                static_cast<int>(msg->header.stamp.sec),
                static_cast<unsigned int>(msg->header.stamp.nanosec),
                msg->joint_names.size(),
                msg->points.size()
            );

            // 检查轨迹点是否有效
            if (msg->points.empty()) {
                RCLCPP_WARN(this->get_node()->get_logger(), "Received empty trajectory");
                return;
            }

            // 存储实时目标点
            {
                std::lock_guard<std::mutex> lock(realtime_target_mutex_);
                realtime_target_ = msg->points[0];
                
                // 验证接收到的数据
                RCLCPP_INFO_THROTTLE(
                    this->get_node()->get_logger(),
                    *this->get_node()->get_clock(),
                    3000,
                    "[接收数据验证] positions.size=%zu, velocities.size=%zu, accelerations.size=%zu",
                    msg->points[0].positions.size(),
                    msg->points[0].velocities.size(),
                    msg->points[0].accelerations.size()
                );
                
                // 打印前 3 个位置和速度数据
                if (!msg->points[0].positions.empty()) {
                    RCLCPP_INFO_THROTTLE(
                        this->get_node()->get_logger(),
                        *this->get_node()->get_clock(),
                        3000,
                        "[接收数据示例] pos[0-2]=[%.6f, %.6f, %.6f], vel[0-2]=[%.6f, %.6f, %.6f]",
                        msg->points[0].positions[0],
                        msg->points[0].positions.size() > 1 ? msg->points[0].positions[1] : 0.0,
                        msg->points[0].positions.size() > 2 ? msg->points[0].positions[2] : 0.0,
                        msg->points[0].velocities.size() > 0 ? msg->points[0].velocities[0] : 0.0,
                        msg->points[0].velocities.size() > 1 ? msg->points[0].velocities[1] : 0.0,
                        msg->points[0].velocities.size() > 2 ? msg->points[0].velocities[2] : 0.0
                    );
                }
            }

            // 设置实时流模式标志
            is_realtime_stream_.store(true, std::memory_order_relaxed);
            RCLCPP_INFO_THROTTLE(
                this->get_node()->get_logger(),
                *this->get_node()->get_clock(),
                3000,
                "已设置实时流模式标志"
            );
        }
    );

    // 作用：创建一个服务器，接收轨迹命令
    trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
        get_node(), "robotic_arm_controller/arm_command", std::bind(&MixController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&MixController::handle_cancel, this, std::placeholders::_1), std::bind(&MixController::handle_accepted, this, std::placeholders::_1)
    );

    moveit_subscriber_ = get_node()->create_subscription<robot_interfaces::msg::Moveit>(
        "moveit_command", 10,
        [this](const robot_interfaces::msg::Moveit::SharedPtr msg) {
            UseMoveit.store(msg->use_moveit, std::memory_order_relaxed);
            RCLCPP_DEBUG_THROTTLE(
                this->get_node()->get_logger(),
                *this->get_node()->get_clock(),
                3000,
                "接受到使用 moveit: use_moveit=%d",
                UseMoveit.load(std::memory_order_relaxed)
            );
            is_realtime_stream_.store(UseMoveit.load(std::memory_order_relaxed));
        }
    );

    // 无论初始 mode 如何，始终建立 Mujoco 话题链路，避免参数加载时序导致未订阅目标命令。
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    state_publisher_ = this->get_node()->create_publisher<robot_interfaces::msg::Arm>(kStateTopic, reliable_qos);
    target_subscriber_ = this->get_node()->create_subscription<robot_interfaces::msg::Arm>(
        kTargetTopic,
        reliable_qos,
        [this](const robot_interfaces::msg::Arm::SharedPtr msg) {
            joints_target_ = *msg;
            target_received_.store(true, std::memory_order_relaxed);
        }
    );
    // 实时消息
    result_msg   = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    feedback_msg = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
    // 设置关节名称
    joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    // 先按关节名数量初始化，on_configure 后会按 KDL 实际自由度重设
    size_t dof = joint_names_.size();
    q_kdl.resize(dof);
    dq_kdl.resize(dof);
    ddq_kdl.resize(dof);
    C_kdl.resize(dof);
    M_kdl.resize(dof);
    G_kdl.resize(dof);
    q_dot_.resize(dof);
    kdl_dof_ = dof;

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
    const std::string control_mode = this->get_node()->get_parameter("control_mode").as_string();
    mujoco_mode_ = (control_mode == "mujoco");
    RCLCPP_INFO(this->get_node()->get_logger(), "on_configure control_mode=%s, mujoco_mode=%d", control_mode.c_str(), mujoco_mode_ ? 1 : 0);

    if (mujoco_mode_) {
        joint_torque_filter_gate_ = this->get_node()->get_parameter("joint_torque_filter_gate").as_double();
        joint_omega_filter_gate_ = this->get_node()->get_parameter("joint_omega_filter_gate").as_double();
        command_effort_limit_ = std::max(this->get_node()->get_parameter("command_effort_limit").as_double(), 0.0);
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            joint_kp_[i] = this->get_node()->get_parameter("joint" + std::to_string(i + 1) + "_kp").as_double();
            joint_kd_[i] = this->get_node()->get_parameter("joint" + std::to_string(i + 1) + "_kd").as_double();
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    // TODO:加载并解析URDF
    RCLCPP_INFO(get_node()->get_logger(), "尝试解析URDF");

    if (!param_query_node_) {
        RCLCPP_ERROR(get_node()->get_logger(), "参数查询节点未初始化");
        return controller_interface::CallbackReturn::ERROR;
    }

    // 参数客户端
    robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(param_query_node_, "/robot_state_publisher");

    if (!robot_description_param_->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(get_node()->get_logger(), "参数服务 /robot_state_publisher 不可用");
        return controller_interface::CallbackReturn::ERROR;
    }

    // 获取urdf
    auto params = robot_description_param_->get_parameters({"robot_description"});
    urdf_xml    = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "无法读取URDF文件，不能进行动力学计算");
        return controller_interface::CallbackReturn::ERROR;
    }

    // 解析urdf
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
        RCLCPP_ERROR(get_node()->get_logger(), "URDF 解析失败，无法构建 KDL 树");
        return controller_interface::CallbackReturn::ERROR;
    }
    if (!tree.getChain("base_link", "link6", chain)) {
        RCLCPP_ERROR(get_node()->get_logger(), "KDL 链提取失败: base_link -> link6");
        return controller_interface::CallbackReturn::ERROR;
    }

    kdl_dof_ = chain.getNrOfJoints();
    if (kdl_dof_ == 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "KDL 链关节数为 0，无法进行动力学计算");
        return controller_interface::CallbackReturn::ERROR;
    }

    q_kdl.resize(kdl_dof_);
    dq_kdl.resize(kdl_dof_);
    ddq_kdl.resize(kdl_dof_);
    C_kdl.resize(kdl_dof_);
    M_kdl.resize(kdl_dof_);
    G_kdl.resize(kdl_dof_);
    q_dot_.resize(kdl_dof_);

    if (kdl_dof_ != joint_names_.size()) {
        RCLCPP_WARN(
            get_node()->get_logger(),
            "KDL DoF(%zu) 与控制器关节数(%zu)不一致，将按最小维度计算动力学",
            kdl_dof_,
            joint_names_.size()
        );
    }

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
    // // 没有轨迹要执行
    // if (!is_execut_trajectory) {
    //     // RCLCPP_INFO(this->get_node()->get_logger(), "控制器更新(未发送)");
    //     return controller_interface::return_type::OK;
    // }




















    (void)period;

    if (mujoco_mode_) {
        const size_t dof = joint_names_.size();
        if (command_interfaces_.size() < dof) {
            RCLCPP_ERROR_THROTTLE(
                this->get_node()->get_logger(),
                *this->get_node()->get_clock(),
                3000,
                "Mujoco command interface 数量不足：expected >= %zu, actual=%zu",
                dof,
                command_interfaces_.size());
            return controller_interface::return_type::ERROR;
        }
        const bool has_effort_state = state_interfaces_.size() >= dof * 3;
        const bool has_pos_vel_state = state_interfaces_.size() >= dof * 2;
        if (!has_pos_vel_state) {
            RCLCPP_ERROR_THROTTLE(
                this->get_node()->get_logger(),
                *this->get_node()->get_clock(),
                3000,
                "Mujoco state interface 数量不足：expected >= %zu(position+velocity), actual=%zu",
                dof * 2,
                state_interfaces_.size());
            return controller_interface::return_type::ERROR;
        }

        for (size_t i = 0; i < dof; ++i) {
            const size_t base = has_effort_state ? i * 3 : i * 2;
            joints_state_.joints[i].rad = static_cast<float>(state_interfaces_[base + 0].get_value());
            joints_state_.joints[i].omega = static_cast<float>(
                joint_omega_filter_gate_ * joints_state_.joints[i].omega +
                (1.0 - joint_omega_filter_gate_) * state_interfaces_[base + 1].get_value());

            const double measured_effort = has_effort_state ? state_interfaces_[base + 2].get_value() : 0.0;
            joints_state_.joints[i].torque = static_cast<float>(
                joint_torque_filter_gate_ * joints_state_.joints[i].torque +
                (1.0 - joint_torque_filter_gate_) * measured_effort);
        }

        if (state_publisher_) {
            state_publisher_->publish(joints_state_);
        }

        // 检查实时流模式（视觉伺服）
        const bool is_realtime = is_realtime_stream_.load(std::memory_order_relaxed);
        
        const bool trajectory_active =
            is_execut_trajectory || (activate_goal_handle_ && activate_goal_handle_->is_active());

        // MuJoCo模式下优先执行FollowJointTrajectory，避免被myjoints_target缺失逻辑覆盖。
        // 如果在实时流模式（视觉伺服），则使用实时目标点
        if (is_realtime) {
            // 实时流模式：直接使用接收到的视觉伺服目标点
            trajectory_msgs::msg::JointTrajectoryPoint target;
            {
                std::lock_guard<std::mutex> lock(realtime_target_mutex_);
                target = realtime_target_;
            }

            // 验证数据完整性
            if (target.positions.size() >= dof && target.velocities.size() >= dof) {
                for (size_t i = 0; i < dof; ++i) {
                    joints_target_.joints[i].rad = static_cast<float>(target.positions[i]);
                    joints_target_.joints[i].omega = static_cast<float>(target.velocities[i]);
                    joints_target_.joints[i].torque = 0.0f;
                }
                RCLCPP_INFO_THROTTLE(
                    this->get_node()->get_logger(),
                    *this->get_node()->get_clock(),
                    3000,
                    "[实时流执行] pos[0-2]=[%.6f, %.6f, %.6f], vel[0-2]=[%.6f, %.6f, %.6f]",
                    target.positions[0],
                    target.positions.size() > 1 ? target.positions[1] : 0.0,
                    target.positions.size() > 2 ? target.positions[2] : 0.0,
                    target.velocities[0],
                    target.velocities.size() > 1 ? target.velocities[1] : 0.0,
                    target.velocities.size() > 2 ? target.velocities[2] : 0.0
                );
            } else {
                RCLCPP_WARN_THROTTLE(
                    this->get_node()->get_logger(),
                    *this->get_node()->get_clock(),
                    3000,
                    "[实时流警告] 目标点数据不完整: pos.size=%zu, vel.size=%zu, expected=%zu",
                    target.positions.size(), target.velocities.size(), dof
                );
            }
        } else if (trajectory_active) {
            const bool has_point = continue_trajectory.get_target(time, output_state);
            if (has_point) {
                for (size_t i = 0; i < dof; ++i) {
                    const double pos = (i < output_state.positions.size()) ? output_state.positions[i] : joints_state_.joints[i].rad;
                    const double vel = (i < output_state.velocities.size()) ? output_state.velocities[i] : 0.0;
                    joints_target_.joints[i].rad = static_cast<float>(pos);
                    joints_target_.joints[i].omega = static_cast<float>(vel);
                    joints_target_.joints[i].torque = 0.0f;
                }
            }

            auto goal_handle = activate_goal_handle_;
            if (goal_handle && goal_handle->is_active()) {
                feedback_msg->joint_names = joint_names_;
                feedback_msg->desired = output_state;
                feedback_msg->actual.positions.resize(dof);
                feedback_msg->actual.velocities.resize(dof);
                for (size_t i = 0; i < dof; ++i) {
                    feedback_msg->actual.positions[i] = joints_state_.joints[i].rad;
                    feedback_msg->actual.velocities[i] = joints_state_.joints[i].omega;
                }
                feedback_msg->header.stamp = get_node()->now();
                try {
                    goal_handle->publish_feedback(feedback_msg);
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_node()->get_logger(), "publish_feedback failed: %s", e.what());
                }
            }

            if (!has_point) {
                is_execut_trajectory = false;
                result_msg->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
                result_msg->error_string = "Trajectory finished";
                auto done_handle = activate_goal_handle_;
                activate_goal_handle_.reset();
                if (done_handle && done_handle->is_active()) {
                    try {
                        done_handle->succeed(result_msg);
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_node()->get_logger(), "succeed failed: %s", e.what());
                    }
                }
            }
        } else if (!target_received_.load(std::memory_order_relaxed)) {
            RCLCPP_WARN_THROTTLE(
                this->get_node()->get_logger(),
                *this->get_node()->get_clock(),
                3000,
                "尚未收到 myjoints_target，当前按零目标输出（可忽略，收到目标后自动恢复）");
        }

        for (size_t i = 0; i < dof; ++i) {
            double effort = joint_kp_[i] * (static_cast<double>(joints_target_.joints[i].rad) - static_cast<double>(joints_state_.joints[i].rad)) +
                            joint_kd_[i] * (static_cast<double>(joints_target_.joints[i].omega) - static_cast<double>(joints_state_.joints[i].omega)) +
                            static_cast<double>(joints_target_.joints[i].torque);
            effort = std::clamp(effort, -command_effort_limit_, command_effort_limit_);
            command_interfaces_[i].set_value(effort);
        }

        return controller_interface::return_type::OK;
    }

    const size_t controlled_dof = std::min(joint_names_.size(), kdl_dof_);
    if (controlled_dof == 0) {
        return controller_interface::return_type::ERROR;
    }

    // 检查实时流模式
    const bool is_realtime = is_realtime_stream_.load(std::memory_order_relaxed);

    if (is_realtime != was_realtime_mode_) {
        RCLCPP_INFO(
            this->get_node()->get_logger(),
            "控制模式切换: %s",
            is_realtime ? "实时流模式(initial_joint_trajectory)" : "轨迹执行模式(FollowJointTrajectory)"
        );
        was_realtime_mode_ = is_realtime;
    }

    if (is_realtime) {
        RCLCPP_INFO(this->get_node()->get_logger(), "正在使用实时流模式");

        // 实时流模式：直接使用接收到的目标点
        trajectory_msgs::msg::JointTrajectoryPoint target;
        {
            std::lock_guard<std::mutex> lock(realtime_target_mutex_);
            target = realtime_target_;
        }

        // 验证接收到的数据大小
        RCLCPP_DEBUG(this->get_node()->get_logger(), 
            "[Update] realtime_target_: pos.size=%zu, vel.size=%zu, acc.size=%zu",
            target.positions.size(), target.velocities.size(), target.accelerations.size());

        // 填充 KDL 数据结构
        for (size_t i = 0; i < controlled_dof; i++) {
            q_kdl(i)   = (i < target.positions.size()) ? target.positions[i] : 0.0;
            dq_kdl(i)  = (i < target.velocities.size()) ? target.velocities[i] : 0.0;
            ddq_kdl(i) = (i < target.accelerations.size()) ? target.accelerations[i] : 0.0;
            RCLCPP_INFO_THROTTLE(
                this->get_node()->get_logger(),
                *this->get_node()->get_clock(),
                3000,
                "[DEBUG] q_kdl(%zu) = %.6f, dq_kdl(%zu) = %.6f, ddq_kdl(%zu) = %.6f",
                i, q_kdl(i), i, dq_kdl(i), i, ddq_kdl(i)
            );
        }

        // 动力学计算
        Eigen::VectorXd torque = dynamicCalc();

        // 写入硬件接口
        for (size_t i = 0; i < joint_names_.size(); i++) {
            const double pos = (i < controlled_dof) ? q_kdl(i) : 0.0;
            const double vel = (i < controlled_dof) ? dq_kdl(i) : 0.0;
            const double eff = (i < static_cast<size_t>(torque.size())) ? torque(static_cast<Eigen::Index>(i)) : 0.0;
            command_interfaces_[i * 3 + 0].set_value(pos);
            command_interfaces_[i * 3 + 1].set_value(vel);
            command_interfaces_[i * 3 + 2].set_value(eff);
            RCLCPP_INFO_THROTTLE(
                this->get_node()->get_logger(),
                *this->get_node()->get_clock(),
                3000,
                "[DEBUG] pos(%zu) = %.6f, vel(%zu) = %.6f, eff(%zu) = %.6f",
                i, pos, i, vel, i, eff
            );
        }

        return controller_interface::return_type::OK;
    }



    
    // 轨迹模式：执行预定义轨迹
    if (!is_execut_trajectory) {
        return controller_interface::return_type::OK;
    }

    bool ret = true;
    const bool use_moveit = UseMoveit.load(std::memory_order_relaxed);


    // 五次多项式插值计算输出
    // get_target 读取当前播放位置
    ret = continue_trajectory.get_target(time, output_state);

    // 填充 KDL 数据结构
    for (size_t i = 0; i < controlled_dof; i++) // 填写轨迹位置/速度/加速度信息
    {
        q_kdl(i)   = output_state.positions[i];
        dq_kdl(i)  = output_state.velocities[i];
        ddq_kdl(i) = output_state.accelerations[i];
    }












    // 动力学计算
    Eigen::VectorXd torque = dynamicCalc(); // 计算力矩前馈值，计算所需力矩大小

    for (size_t i = 0; i < joint_names_.size(); i++) // 将计算结果写入硬件层
    {
        const double pos = (i < controlled_dof) ? q_kdl(i) : 0.0;
        const double vel = (i < controlled_dof) ? dq_kdl(i) : 0.0;
        const double eff = (i < static_cast<size_t>(torque.size())) ? torque(static_cast<Eigen::Index>(i)) : 0.0;
        command_interfaces_[i * 3 + 0].set_value(pos);         // 写入位置
        command_interfaces_[i * 3 + 1].set_value(vel);        // 写入速度
        command_interfaces_[i * 3 + 2].set_value(eff); // 写入力矩
    }


    auto goal_handle = activate_goal_handle_;
    if (!goal_handle) {
        is_execut_trajectory = false;
        return controller_interface::return_type::OK;
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

    if (goal_handle->is_active()) {
        try {
            goal_handle->publish_feedback(feedback_msg);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_node()->get_logger(), "publish_feedback failed: %s", e.what());
            is_execut_trajectory = false;
            activate_goal_handle_.reset();
            return controller_interface::return_type::ERROR;
        }
    }

    // 通知轨迹完成（仅 MoveIt 轨迹模式）
    if (!use_moveit && !ret) { // 如果这点是最后一个点，那么发送完成状态
        is_execut_trajectory     = false;
        result_msg->error_code   = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        result_msg->error_string = "Trajectory finished";
        auto done_handle = activate_goal_handle_;
        activate_goal_handle_.reset();
        if (done_handle && done_handle->is_active()) {
            try {
                done_handle->succeed(result_msg);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_node()->get_logger(), "succeed failed: %s", e.what());
            }
        }
    }

    // RCLCPP_INFO(this->get_node()->get_logger(), "控制器更新");
    return controller_interface::return_type::OK;
}

// 说明控制器需要的命令接口
controller_interface::InterfaceConfiguration MixController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg; // 创建配置对象
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL; // 逐个声明

    if (mujoco_mode_) {
        for (const auto& name : joint_names_) {
            cfg.names.push_back(name + "/effort");
        }
        return cfg;
    }

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

    if (mujoco_mode_) {
        for (const auto& name : joint_names_) {
            cfg.names.push_back(name + "/position");
            cfg.names.push_back(name + "/velocity");
            cfg.names.push_back(name + "/effort");
        }
        return cfg;
    }

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
    (void)uuid;

    // 如果正在执行其他轨迹，拒绝新目标
    if (is_execut_trajectory || (activate_goal_handle_ && activate_goal_handle_->is_active()))
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
    // 停止轨迹执行
    is_execut_trajectory = false;

    // 标记已取消
    cancle_execut        = true;

    if (goal_handle && goal_handle->is_active()) {
        auto cancel_result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        cancel_result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        cancel_result->error_string = "Trajectory canceled";
        try {
            goal_handle->canceled(cancel_result);
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_node()->get_logger(), "canceled() failed: %s", e.what());
        }
    }
    activate_goal_handle_.reset();

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
Eigen::VectorXd MixController::dynamicCalc() {
    const Eigen::Index tau_size = static_cast<Eigen::Index>(joint_names_.size());
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(tau_size);
    const size_t dof = kdl_dof_;
    if (!dyn || dof == 0) {
        return tau;
    }

    // 5. 调用 KDL 动力学函数
    dyn->JntToMass(q_kdl, M_kdl); // 计算惯性矩阵
    dyn->JntToCoriolis(q_kdl, dq_kdl, C_kdl); // 计算科里奥利力
    dyn->JntToGravity(q_kdl, G_kdl); // 计算重力

    // 6. 转换 KDL 输出到 Eigen，方便矩阵运算
    const Eigen::Index dof_idx = static_cast<Eigen::Index>(dof);
    Eigen::MatrixXd M_mat = Eigen::MatrixXd::Zero(dof_idx, dof_idx); // 惯性矩阵
    Eigen::VectorXd C = Eigen::VectorXd::Zero(dof_idx);
    Eigen::VectorXd G = Eigen::VectorXd::Zero(dof_idx);
    Eigen::VectorXd ddq = Eigen::VectorXd::Zero(dof_idx);

    for (Eigen::Index i = 0; i < dof_idx; ++i) {
        C(i)   = C_kdl(static_cast<unsigned int>(i)); // 科里奥利力
        G(i)   = G_kdl(static_cast<unsigned int>(i)); // 重力
        ddq(i) = ddq_kdl(static_cast<unsigned int>(i)); // 加速度
        for (Eigen::Index j = 0; j < dof_idx; ++j) {
            M_mat(i, j) = M_kdl(static_cast<unsigned int>(i), static_cast<unsigned int>(j)); // 惯性矩阵元素
        }
    }
    // 7. 计算前馈力矩 tau
    // 计算前馈力矩：τ = M·ddq + C + G
    Eigen::VectorXd tau_dyn = (M_mat * ddq + C + G);
    const size_t n = std::min(static_cast<size_t>(tau_dyn.size()), joint_names_.size());
    for (size_t i = 0; i < n; ++i) {
        tau(static_cast<Eigen::Index>(i)) = tau_dyn(static_cast<Eigen::Index>(i));
    }
    return tau;
}


} // namespace mixcontroller

// 把控制器类导出为 ROS 2 插件
PLUGINLIB_EXPORT_CLASS(mixcontroller::MixController, controller_interface::ControllerInterface)
