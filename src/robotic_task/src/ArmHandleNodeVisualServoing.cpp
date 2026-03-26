#include "ArmHandleNodeVisualServoing.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <limits>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <rclcpp/duration.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>

VisualServoingArmHandleNode::VisualServoingArmHandleNode(const rclcpp::Node::SharedPtr node) : node_(node) {
    // 初始化路径向量
    path_vector_ = Eigen::Vector3d::Zero();
    // 创建发布器
    twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>("twist_command", 10);
    initial_joint_trajectory_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("initial_joint_trajectory", 10);


    // 初始化KDL
    if (!initKDL()) {
        RCLCPP_ERROR(node_->get_logger(), "KDL初始化失败");
    }

    ik_position_tolerance_m_ = node_->declare_parameter<double>(
        "visual_servo.ik_position_tolerance_m", ik_position_tolerance_m_);
    ik_orientation_tolerance_rad_ = node_->declare_parameter<double>(
        "visual_servo.ik_orientation_tolerance_rad", ik_orientation_tolerance_rad_);
    ik_orientation_tolerance_relaxed_rad_ = node_->declare_parameter<double>(
        "visual_servo.ik_orientation_tolerance_relaxed_rad", ik_orientation_tolerance_relaxed_rad_);
    ik_fail_relax_after_n_ = node_->declare_parameter<int>(
        "visual_servo.ik_fail_relax_after_n", ik_fail_relax_after_n_);
    joint_state_timeout_sec_ = node_->declare_parameter<double>(
        "visual_servo.joint_state_timeout_sec", joint_state_timeout_sec_);
    external_seed_timeout_sec_ = node_->declare_parameter<double>(
        "visual_servo.external_seed_timeout_sec", external_seed_timeout_sec_);
    RCLCPP_INFO(
        node_->get_logger(),
        "视觉伺服IK阈值: trans=%.6f m, rot=%.6f rad (relaxed=%.6f rad, fail_relax_after=%d)",
        ik_position_tolerance_m_,
        ik_orientation_tolerance_rad_,
        ik_orientation_tolerance_relaxed_rad_,
        ik_fail_relax_after_n_);
    
    // 订阅关节状态
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&VisualServoingArmHandleNode::jointStateCallback, this, std::placeholders::_1)
    );
}

VisualServoingArmHandleNode::~VisualServoingArmHandleNode() = default;

void VisualServoingArmHandleNode::resetServoState(
    const geometry_msgs::msg::PoseStamped& actual_position,
    const geometry_msgs::msg::PoseStamped& final_desired_position) {
    actual_position_ = actual_position;
    final_desired_position_ = final_desired_position;
    crrent_desired_position_ = actual_position;
    LastTargetPose_ = final_desired_position;

    current_desired_velocity_ = geometry_msgs::msg::Twist{};
    last_desired_velocity_ = geometry_msgs::msg::Twist{};

    initial_trajectory_point_.pose = actual_position;
    initial_trajectory_point_.velocity = geometry_msgs::msg::Twist{};
    initial_trajectory_point_.acceleration = geometry_msgs::msg::Twist{};
    initial_trajectory_point_.timestamp = node_->now();

    servo_state_initialized_ = true;
    is_first_iteration_ = false;

    RCLCPP_INFO(
        node_->get_logger(),
        "重置视觉伺服状态: actual_pos=(%.4f, %.4f, %.4f), target_pos=(%.4f, %.4f, %.4f)",
        actual_position.pose.position.x,
        actual_position.pose.position.y,
        actual_position.pose.position.z,
        final_desired_position.pose.position.x,
        final_desired_position.pose.position.y,
        final_desired_position.pose.position.z
    );
}

bool VisualServoingArmHandleNode::initKDL() {
    // 获取URDF参数
    robot_description_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/robot_state_publisher");
    
    // 等待参数服务
    while (!robot_description_client_->wait_for_service(std::chrono::seconds(2))) {
        if (!rclcpp::ok()) return false;
        RCLCPP_WARN(node_->get_logger(), "等待 /robot_state_publisher 服务...");
    }
    
    // 获取URDF
    auto params = robot_description_client_->get_parameters({"robot_description"});
    std::string urdf_xml = params[0].as_string();
    if (urdf_xml.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "无法获取URDF");
        return false;
    }
    
    // 解析KDL树
    if (!kdl_parser::treeFromString(urdf_xml, kdl_tree_)) {
        RCLCPP_ERROR(node_->get_logger(), "URDF解析失败");
        return false;
    }
    
    // 提取链 (base_link -> link6)
    if (!kdl_tree_.getChain("base_link", "link6", kdl_chain_)) {
        RCLCPP_ERROR(node_->get_logger(), "KDL链提取失败");
        return false;
    }
    
    // 初始化关节数组
    unsigned int nj = kdl_chain_.getNrOfJoints();
    current_joint_positions_.resize(nj);
    
    // 创建IK求解器
    ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
    Eigen::Matrix<double, 6, 1> l_pos_priority;
    l_pos_priority << 1.0, 1.0, 1.0, 1e-4, 1e-4, 1e-4;
    ik_solver_position_priority_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(
        kdl_chain_, l_pos_priority, 1E-5, 500, 1E-15);
    Eigen::Matrix<double, 6, 1> l_pos_priority;
    l_pos_priority << 1.0, 1.0, 1.0, 1e-4, 1e-4, 1e-4;
    ik_solver_position_priority_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(
        kdl_chain_, l_pos_priority, 1E-5, 500, 1E-15);
    
    // 创建雅可比求解器
    jacobian_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(kdl_chain_);
    
    RCLCPP_INFO(node_->get_logger(), "KDL初始化成功，关节数: %d", nj);
    return true;
}

void VisualServoingArmHandleNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    // 兼容不同驱动的关节命名风格，只有全部6个关节都匹配到才认为关节状态有效。
    const std::array<std::array<std::string, 3>, 6> joint_name_candidates = {{
        {{"joint1", "joint_1", "j1"}},
        {{"joint2", "joint_2", "j2"}},
        {{"joint3", "joint_3", "j3"}},
        {{"joint4", "joint_4", "j4"}},
        {{"joint5", "joint_5", "j5"}},
        {{"joint6", "joint_6", "j6"}}
    }};

    const auto name_matches = [](const std::string& actual, const std::string& expected) {
        if (actual == expected) {
            return true;
        }
        if (actual.size() > expected.size() &&
            actual.compare(actual.size() - expected.size(), expected.size(), expected) == 0) {
            const size_t prefix_end = actual.size() - expected.size();
            return prefix_end > 0 && (actual[prefix_end - 1] == '/' || actual[prefix_end - 1] == '_');
        }
        return false;
    };

    size_t matched_joint_count = 0;
    for (size_t i = 0; i < joint_name_candidates.size(); ++i) {
        bool found_this_joint = false;
        for (size_t msg_idx = 0; msg_idx < msg->name.size(); ++msg_idx) {
            if (msg_idx >= msg->position.size()) {
                continue;
            }
            for (const auto& candidate : joint_name_candidates[i]) {
                if (name_matches(msg->name[msg_idx], candidate)) {
                    current_joint_positions_(i) = msg->position[msg_idx];
                    found_this_joint = true;
                    break;
                }
            }
            if (found_this_joint) {
                break;
            }
        }
        if (found_this_joint) {
            ++matched_joint_count;
        }
    }

    if (matched_joint_count == joint_name_candidates.size()) {
        joint_state_received_ = true;
        last_joint_state_stamp_ = msg->header.stamp.nanosec == 0 && msg->header.stamp.sec == 0
            ? node_->get_clock()->now()
            : rclcpp::Time(msg->header.stamp);
    } else {
        joint_state_received_ = false;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "joint_states关节名匹配不完整: matched=%zu/6, msg_names_count=%zu",
            matched_joint_count,
            msg->name.size()
        );
    }
}

void VisualServoingArmHandleNode::updateExternalJointSeed(const std::vector<double>& joint_positions) {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);

    const unsigned int expected_joint_count = current_joint_positions_.rows();
    if (expected_joint_count == 0) {
        return;
    }
    if (joint_positions.size() != expected_joint_count) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            2000,
            "外部关节种子维度不匹配: got=%zu, expected=%u",
            joint_positions.size(),
            expected_joint_count
        );
        return;
    }

    external_joint_seed_.resize(expected_joint_count);
    for (unsigned int i = 0; i < expected_joint_count; ++i) {
        external_joint_seed_(i) = joint_positions[i];
    }
    has_external_joint_seed_ = true;
    external_joint_seed_stamp_ = node_->get_clock()->now();
}

Eigen::Vector3d VisualServoingArmHandleNode::CalculatePath(
    Eigen::Vector3d& current_position, Eigen::Vector3d& target_position
) {
    Eigen::Vector3d path_vector_ = target_position - current_position; // 计算路径向量
    return path_vector_;
}

geometry_msgs::msg::Twist VisualServoingArmHandleNode::CalculateTwist(Eigen::Vector3d& path_vector) {
    geometry_msgs::msg::Twist twist_msg;
    double max_liner_velocity = 1.0; // 最大线速度
    double max_angular_velocity = 10.0; // 最大角速度

    // 1.当前时刻期望位置
    // 2.最终期望位置
    // 3.实际位置
    // 初始化：当前期望位置 = 实际位置
    // 当前期望速度=0
    // 上次期望速度=0
    // 控制周期: 
    // 当前期望速度=（最终期望位置-当前期望位置） * kp
    // 当前期望加速度=limit（当前期望速度-上次期望速度）/dt
    // 当前期望速度=上次期望速度+当前期望加速度*dt

    // 当前期望位位置=当前期望位位置+当前期望速度*dt
    // 当前机械臂目标=当前期望位置/当前期望速度/当前期望加速度











    // 计算线速度
    twist_msg.linear.x = path_vector.x();
    twist_msg.linear.y = path_vector.y();
    twist_msg.linear.z = path_vector.z();
    double liner_velocity_magnitude = path_vector.norm(); // 计算路径向量的大小
    if (liner_velocity_magnitude > max_liner_velocity) {
        twist_msg.linear.x = (path_vector.x() / liner_velocity_magnitude) * max_liner_velocity;
        twist_msg.linear.y = (path_vector.y() / liner_velocity_magnitude) * max_liner_velocity;
        twist_msg.linear.z = (path_vector.z() / liner_velocity_magnitude) * max_liner_velocity;
    }

    // 计算角速度
    // 1. 根据路径向量方向计算目标朝向
    Eigen::Quaterniond target_orientation = CalculateTargetOrientation(path_vector);
    
    // 2. 获取当前末端朝向
    Eigen::Quaterniond current_orientation(
        CurrentPose_.pose.orientation.w,
        CurrentPose_.pose.orientation.x,
        CurrentPose_.pose.orientation.y,
        CurrentPose_.pose.orientation.z
    );
    
    // 3. 计算两个朝向之间的相对旋转
    Eigen::Quaterniond relative_rotation = current_orientation.inverse() * target_orientation;
    
    // 4. 将相对旋转转换为轴-角表示，提取夹角作为角速度大小
    Eigen::AngleAxisd angle_axis(relative_rotation);
    double angle_magnitude = angle_axis.angle(); // 夹角大小
    
    // 5. 获取旋转轴
    Eigen::Vector3d rotation_axis = angle_axis.axis();
    
    // 6. 计算角速度矢量（旋转轴 * 夹角大小）
    Eigen::Vector3d angular_velocity = rotation_axis * angle_magnitude;
    
    // 7. 进行角速度限制
    double angular_velocity_magnitude = angular_velocity.norm();
    if (angular_velocity_magnitude > max_angular_velocity) {
        angular_velocity = (angular_velocity / angular_velocity_magnitude) * max_angular_velocity;
    }
    
    // 8. 将角速度赋值给Twist消息
    twist_msg.angular.x = angular_velocity.x();
    twist_msg.angular.y = angular_velocity.y();
    twist_msg.angular.z = angular_velocity.z();

    return twist_msg;
}

void VisualServoingArmHandleNode::SendTwistCommand(const geometry_msgs::msg::Twist& twist_msg) {
    twist_publisher_->publish(twist_msg); // 发布Twist消息
}

void VisualServoingArmHandleNode::SendTrajectoryCommand() {
    // 验证消息数据完整性
    if (initial_joint_trajectory_.points.empty()) {
        RCLCPP_WARN(node_->get_logger(), "[WARNING] 轨迹消息为空！");
        return;
    }
    
    // 确保消息头被正确设置
    if (initial_joint_trajectory_.header.stamp.sec == 0 && initial_joint_trajectory_.header.stamp.nanosec == 0) {
        initial_joint_trajectory_.header.stamp = node_->get_clock()->now();
        RCLCPP_WARN(node_->get_logger(), "[WARNING] 轨迹消息header未初始化，已重新设置");
    }
    
    std::cout << "[DEBUG] 发送关节轨迹命令: header.stamp=(" << initial_joint_trajectory_.header.stamp.sec 
              << "." << initial_joint_trajectory_.header.stamp.nanosec << "), positions=[";
    for (size_t i = 0; i < initial_joint_trajectory_.points[0].positions.size(); ++i) {
        std::cout << initial_joint_trajectory_.points[0].positions[i];
        if (i != initial_joint_trajectory_.points[0].positions.size() - 1) std::cout << ", ";
    }
    std::cout << "], velocities=[";
    for (size_t i = 0; i < initial_joint_trajectory_.points[0].velocities.size(); ++i) {
        std::cout << initial_joint_trajectory_.points[0].velocities[i];
        if (i != initial_joint_trajectory_.points[0].velocities.size() - 1) std::cout << ", ";
    }   
    std::cout << "], accelerations=[";
    for (size_t i = 0; i < initial_joint_trajectory_.points[0].accelerations.size(); ++i) {
        std::cout << initial_joint_trajectory_.points[0].accelerations[i];
        if (i != initial_joint_trajectory_.points[0].accelerations.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    RCLCPP_DEBUG(node_->get_logger(), "Publishing trajectory: joint_names.size=%zu, points.size=%zu, first_pos.size=%zu",
        initial_joint_trajectory_.joint_names.size(),
        initial_joint_trajectory_.points.size(),
        initial_joint_trajectory_.points[0].positions.size());
    initial_joint_trajectory_publisher_->publish(initial_joint_trajectory_);
}





Eigen::Quaterniond VisualServoingArmHandleNode::CalculateTargetOrientation(const Eigen::Vector3d& path_vector) {
    // 将路径向量作为目标朝向的Z轴方向（向前方向）
    Eigen::Vector3d z_axis = path_vector.normalized();
    
    // 定义参考向上方向（世界坐标系的Z轴正方向）
    Eigen::Vector3d up_direction(0.0, 0.0, 1.0);
    
    // 如果路径向量过于接近向上或向下方向，使用不同的参考向上方向
    if (std::abs(z_axis.dot(up_direction)) > 0.99) {
        up_direction = Eigen::Vector3d(0.0, 1.0, 0.0); // 使用Y轴作为参考
    }
    
    // 计算右向量（X轴）：z轴 × 参考上方向
    Eigen::Vector3d x_axis = z_axis.cross(up_direction).normalized();
    
    // 重新计算上向量（Y轴）：x轴 × z轴
    Eigen::Vector3d y_axis = x_axis.cross(z_axis).normalized();
    
    // 从三个正交向量构造旋转矩阵
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix.col(0) = x_axis;
    rotation_matrix.col(1) = y_axis;
    rotation_matrix.col(2) = z_axis;
    
    // 将旋转矩阵转换为四元数
    Eigen::Quaterniond target_orientation(rotation_matrix);
    
    return target_orientation;
}

void VisualServoingArmHandleNode::TotalPackaing(
    Eigen::Vector3d& current_position, Eigen::Vector3d& target_position,
    geometry_msgs::msg::PoseStamped& actual_position, geometry_msgs::msg::PoseStamped& final_desired_position
    ) {
    (void)current_position;
    (void)target_position;
    
    // 设置当前位姿和目标位姿
    CurrentPose_ = actual_position;
    TargetPose_ = final_desired_position;
    actual_position_ = actual_position;
    final_desired_position_ = final_desired_position;

    const Eigen::Vector3d actual_position_eigen(
        actual_position.pose.position.x,
        actual_position.pose.position.y,
        actual_position.pose.position.z
    );
    const Eigen::Vector3d previous_desired_position_eigen(
        crrent_desired_position_.pose.position.x,
        crrent_desired_position_.pose.position.y,
        crrent_desired_position_.pose.position.z
    );
    const Eigen::Vector3d previous_target_position_eigen(
        LastTargetPose_.pose.position.x,
        LastTargetPose_.pose.position.y,
        LastTargetPose_.pose.position.z
    );

    const bool should_reset_state =
        !servo_state_initialized_ ||
        is_first_iteration_ ||
        (actual_position_eigen - previous_desired_position_eigen).norm() > 0.05 ||
        (target_position - previous_target_position_eigen).norm() > 0.10;

    if (should_reset_state) {
        resetServoState(actual_position, final_desired_position);
    } else {
        LastTargetPose_ = final_desired_position;
    }

    // 计算路径向量
    //Eigen::Vector3d path_vector = CalculatePath(current_position, target_position);

    // 计算Twist命令
    // geometry_msgs::msg::Twist twist_msg = CalculateTwist(path_vector);

    ComputationalSpeed(); // 计算当前期望速度和位置
    
    // 将末端数据转换为关节轨迹
    const bool ik_ok = PointToTrajectoryPoint();

    // 发送轨迹命令
    if (ik_ok) {
        SendTrajectoryCommand();
    } else {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "IK未收敛，本周期跳过轨迹发送，避免发布异常速度/加速度");
    }

    // if (path_vector.norm() < 0.01) { // 如果路径向量的大小小于某个阈值，认为已经到达目标位置
    //     RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "已到达目标位置");
    //     twist_msg.linear.x = 0.0; // 停止移动
    //     twist_msg.linear.y = 0.0;
    //     twist_msg.linear.z = 0.0;
    //     SendTwistCommand(twist_msg);
    // } else {
    //     RCLCPP_DEBUG_THROTTLE(
    //         node_->get_logger(),
    //         *node_->get_clock(),
    //         2000,
    //         "正在移动，当前路径向量大小: %f",
    //         path_vector.norm()
    //     );
    // }
}


    // 1.当前时刻期望位置
    // 2.最终期望位置
    // 3.实际位置
    // 初始化：当前期望位置 = 实际位置
    // 当前期望速度=0
    // 上次期望速度=0
    // 控制周期: 
    // 当前期望速度=（最终期望位置-当前期望位置） * kp
    // 当前期望加速度=limit（当前期望速度-上次期望速度）/dt
    // 当前期望速度=上次期望速度+当前期望加速度*dt

    // 当前期望位位置=当前期望位位置+当前期望速度*dt
    // 当前机械臂目标=当前期望位置/当前期望速度/当前期望加速度
void VisualServoingArmHandleNode::ComputationalSpeed() {
    if (dt_ <= 1e-6) {
        RCLCPP_ERROR(node_->get_logger(), "dt_过小(%.9f)，无法进行速度/加速度积分", dt_);
        return;
    }

    if(is_first_iteration_) {
        actual_position_ = CurrentPose_;
        final_desired_position_ = TargetPose_;

        crrent_desired_position_ = actual_position_; // 初始化当前期望位置为实际位置
        current_desired_velocity_.linear.x = 0.0; // 初始化当前期望速度为0
        current_desired_velocity_.linear.y = 0.0;
        current_desired_velocity_.linear.z = 0.0;
        current_desired_velocity_.angular.x = 0.0;
        current_desired_velocity_.angular.y = 0.0;
        current_desired_velocity_.angular.z = 0.0;
        last_desired_velocity_.linear.x = 0.0; // 初始化上次期望速度为0
        last_desired_velocity_.linear.y = 0.0;
        last_desired_velocity_.linear.z = 0.0;
        last_desired_velocity_.angular.x = 0.0;
        last_desired_velocity_.angular.y = 0.0;
        last_desired_velocity_.angular.z = 0.0;
        is_first_iteration_ = false; // 标记第一次迭代完成

        // 保存t0时刻的完整轨迹点
        initial_trajectory_point_.pose = CurrentPose_;
        initial_trajectory_point_.velocity.linear.x = 0.0;
        initial_trajectory_point_.velocity.linear.y = 0.0;
        initial_trajectory_point_.velocity.linear.z = 0.0;
        initial_trajectory_point_.velocity.angular.x = 0.0;
        initial_trajectory_point_.velocity.angular.y = 0.0;
        initial_trajectory_point_.velocity.angular.z = 0.0;
        initial_trajectory_point_.acceleration.linear.x = 0.0;
        initial_trajectory_point_.acceleration.linear.y = 0.0;
        initial_trajectory_point_.acceleration.linear.z = 0.0;
        initial_trajectory_point_.acceleration.angular.x = 0.0;
        initial_trajectory_point_.acceleration.angular.y = 0.0;
        initial_trajectory_point_.acceleration.angular.z = 0.0;
        initial_trajectory_point_.timestamp = node_->now();

        RCLCPP_INFO(
            node_->get_logger(),
            "ComputationalSpeed init: actual_pos=(%.4f, %.4f, %.4f), target_pos=(%.4f, %.4f, %.4f)",
            actual_position_.pose.position.x,
            actual_position_.pose.position.y,
            actual_position_.pose.position.z,
            final_desired_position_.pose.position.x,
            final_desired_position_.pose.position.y,
            final_desired_position_.pose.position.z
        );
    }

    // 计算当前期望速度
    current_desired_velocity_.linear.x = (final_desired_position_.pose.position.x - crrent_desired_position_.pose.position.x) * kp_;
    current_desired_velocity_.linear.y = (final_desired_position_.pose.position.y - crrent_desired_position_.pose.position.y) * kp_;
    current_desired_velocity_.linear.z = (final_desired_position_.pose.position.z - crrent_desired_position_.pose.position.z) * kp_;

    tf2::Quaternion final_quaternion(
        final_desired_position_.pose.orientation.x,
        final_desired_position_.pose.orientation.y,
        final_desired_position_.pose.orientation.z,
        final_desired_position_.pose.orientation.w
    );
    tf2::Quaternion current_quaternion(
        crrent_desired_position_.pose.orientation.x,
        crrent_desired_position_.pose.orientation.y,
        crrent_desired_position_.pose.orientation.z,
        crrent_desired_position_.pose.orientation.w
    );
    final_quaternion.normalize();
    current_quaternion.normalize();
    double roll, pitch, yaw, final_roll, final_pitch, final_yaw;
    tf2::Matrix3x3(final_quaternion).getRPY(final_roll, final_pitch, final_yaw);
    tf2::Matrix3x3(current_quaternion).getRPY(roll, pitch, yaw);
    current_desired_velocity_.angular.x = (final_roll - roll) * kp_;
    current_desired_velocity_.angular.y = (final_pitch - pitch) * kp_;
    current_desired_velocity_.angular.z = (final_yaw - yaw) * kp_;

    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "PoseErr: dpos=(%.5f, %.5f, %.5f), drot=(%.5f, %.5f, %.5f), angle=%.5f",
        final_desired_position_.pose.position.x - crrent_desired_position_.pose.position.x,
        final_desired_position_.pose.position.y - crrent_desired_position_.pose.position.y,
        final_desired_position_.pose.position.z - crrent_desired_position_.pose.position.z,
        current_desired_velocity_.angular.x,
        current_desired_velocity_.angular.y,
        current_desired_velocity_.angular.z,
        relative_angle
    );
    

    double max_acceleration_ = 0.5; // 最大加速度 (单位: m/s^2 或 rad/s^2)

    // 计算当前期望加速度
    geometry_msgs::msg::Twist current_desired_acceleration;
    
    current_desired_acceleration.linear.x = (current_desired_velocity_.linear.x - last_desired_velocity_.linear.x) / dt_;
    current_desired_acceleration.linear.y = (current_desired_velocity_.linear.y - last_desired_velocity_.linear.y) / dt_;
    current_desired_acceleration.linear.z = (current_desired_velocity_.linear.z - last_desired_velocity_.linear.z) / dt_;
    current_desired_acceleration.angular.x = (current_desired_velocity_.angular.x - last_desired_velocity_.angular.x) / dt_;
    current_desired_acceleration.angular.y = (current_desired_velocity_.angular.y - last_desired_velocity_.angular.y) / dt_;
    current_desired_acceleration.angular.z = (current_desired_velocity_.angular.z - last_desired_velocity_.angular.z) / dt_;

    double acc_x = std::clamp(current_desired_acceleration.linear.x, -max_acceleration_, max_acceleration_);
    double acc_y = std::clamp(current_desired_acceleration.linear.y, -max_acceleration_, max_acceleration_);
    double acc_z = std::clamp(current_desired_acceleration.linear.z, -max_acceleration_, max_acceleration_);
    double acc_angular_x = std::clamp(current_desired_acceleration.angular.x, -max_acceleration_, max_acceleration_);
    double acc_angular_y = std::clamp(current_desired_acceleration.angular.y, -max_acceleration_, max_acceleration_);
    double acc_angular_z = std::clamp(current_desired_acceleration.angular.z, -max_acceleration_, max_acceleration_);
    current_desired_acceleration.linear.x = acc_x;
    current_desired_acceleration.linear.y = acc_y;
    current_desired_acceleration.linear.z = acc_z;
    current_desired_acceleration.angular.x = acc_angular_x;
    current_desired_acceleration.angular.y = acc_angular_y;
    current_desired_acceleration.angular.z = acc_angular_z;

    // 更新当前期望速度
    current_desired_velocity_.linear.x = last_desired_velocity_.linear.x + current_desired_acceleration.linear.x * dt_;
    current_desired_velocity_.linear.y = last_desired_velocity_.linear.y + current_desired_acceleration.linear.y * dt_;
    current_desired_velocity_.linear.z = last_desired_velocity_.linear.z + current_desired_acceleration.linear.z * dt_;
    current_desired_velocity_.angular.x = last_desired_velocity_.angular.x + current_desired_acceleration.angular.x * dt_;
    current_desired_velocity_.angular.y = last_desired_velocity_.angular.y + current_desired_acceleration.angular.y * dt_;
    current_desired_velocity_.angular.z = last_desired_velocity_.angular.z + current_desired_acceleration.angular.z * dt_;

    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "DesiredVel: linear=(%.5f, %.5f, %.5f), angular=(%.5f, %.5f, %.5f)",
        current_desired_velocity_.linear.x,
        current_desired_velocity_.linear.y,
        current_desired_velocity_.linear.z,
        current_desired_velocity_.angular.x,
        current_desired_velocity_.angular.y,
        current_desired_velocity_.angular.z
    );

    // 更新当前期望位置
    crrent_desired_position_.pose.position.x += current_desired_velocity_.linear.x * dt_;
    crrent_desired_position_.pose.position.y += current_desired_velocity_.linear.y * dt_;
    crrent_desired_position_.pose.position.z += current_desired_velocity_.linear.z * dt_;
    // 用四元数乘法积分角速度，避免直接叠加四元数分量导致姿态漂移。
    tf2::Quaternion desired_q(
        crrent_desired_position_.pose.orientation.x,
        crrent_desired_position_.pose.orientation.y,
        crrent_desired_position_.pose.orientation.z,
        crrent_desired_position_.pose.orientation.w
    );
    desired_q.normalize();

    tf2::Quaternion delta_q;
    delta_q.setRPY(
        current_desired_velocity_.angular.x * dt_,
        current_desired_velocity_.angular.y * dt_,
        current_desired_velocity_.angular.z * dt_
    );
    desired_q = desired_q * delta_q;
    desired_q.normalize();

    crrent_desired_position_.pose.orientation.x = desired_q.x();
    crrent_desired_position_.pose.orientation.y = desired_q.y();
    crrent_desired_position_.pose.orientation.z = desired_q.z();
    crrent_desired_position_.pose.orientation.w = desired_q.w();

    const double q_norm = std::sqrt(
        crrent_desired_position_.pose.orientation.x * crrent_desired_position_.pose.orientation.x +
        crrent_desired_position_.pose.orientation.y * crrent_desired_position_.pose.orientation.y +
        crrent_desired_position_.pose.orientation.z * crrent_desired_position_.pose.orientation.z +
        crrent_desired_position_.pose.orientation.w * crrent_desired_position_.pose.orientation.w
    );

    RCLCPP_DEBUG_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        500,
        "DesiredPose: pos=(%.5f, %.5f, %.5f), quat=(%.5f, %.5f, %.5f, %.5f), |q|=%.6f",
        crrent_desired_position_.pose.position.x,
        crrent_desired_position_.pose.position.y,
        crrent_desired_position_.pose.position.z,
        crrent_desired_position_.pose.orientation.x,
        crrent_desired_position_.pose.orientation.y,
        crrent_desired_position_.pose.orientation.z,
        crrent_desired_position_.pose.orientation.w,
        q_norm
    );

    // 当前机械臂目标=当前期望位置/当前期望速度/当前期望加速度

    const rclcpp::Time previous_timestamp = initial_trajectory_point_.timestamp;

    initial_trajectory_point_.pose = crrent_desired_position_;
    initial_trajectory_point_.velocity.linear.x = current_desired_velocity_.linear.x;
    initial_trajectory_point_.velocity.linear.y = current_desired_velocity_.linear.y;
    initial_trajectory_point_.velocity.linear.z = current_desired_velocity_.linear.z;
    initial_trajectory_point_.velocity.angular.x = current_desired_velocity_.angular.x;
    initial_trajectory_point_.velocity.angular.y = current_desired_velocity_.angular.y;
    initial_trajectory_point_.velocity.angular.z = current_desired_velocity_.angular.z;
    initial_trajectory_point_.acceleration.linear.x = current_desired_acceleration.linear.x;
    initial_trajectory_point_.acceleration.linear.y = current_desired_acceleration.linear.y;
    initial_trajectory_point_.acceleration.linear.z = current_desired_acceleration.linear.z;
    initial_trajectory_point_.acceleration.angular.x = current_desired_acceleration.angular.x;
    initial_trajectory_point_.acceleration.angular.y = current_desired_acceleration.angular.y;
    initial_trajectory_point_.acceleration.angular.z = current_desired_acceleration.angular.z;

    last_desired_velocity_ = current_desired_velocity_;
    
    rclcpp::Time now = node_->get_clock()->now();
    rclcpp::Duration dt = now - previous_timestamp;
    duration_ns_ = dt.nanoseconds();
    duration_sec_ = dt.seconds();
}


TrajectoryPoint VisualServoingArmHandleNode::getInitialTrajectory() const {
    return initial_trajectory_point_;
}


// 使用KDL将末端数据转换成关节数据
bool VisualServoingArmHandleNode::PointToTrajectoryPoint() {
    if (!ik_solver_ || !ik_solver_position_priority_ || !jacobian_solver_) {
        RCLCPP_ERROR(node_->get_logger(), "KDL求解器未初始化");
        return false;
    }
    // 1. 从末端位姿构建KDL::Frame
    const auto& pose = initial_trajectory_point_.pose.pose;

    const double q_norm = std::sqrt(
        pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y +
        pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w
    );

    if (q_norm < 1e-9 ||
        !std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(pose.position.z) ||
        !std::isfinite(pose.orientation.x) || !std::isfinite(pose.orientation.y) ||
        !std::isfinite(pose.orientation.z) || !std::isfinite(pose.orientation.w)) {
        RCLCPP_ERROR(node_->get_logger(),
            "目标位姿非法，跳过IK: pos=(%.6f, %.6f, %.6f), quat=(%.6f, %.6f, %.6f, %.6f), |q|=%.9f",
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w, q_norm);
        return;
    }

    KDL::Frame target_frame(
        KDL::Rotation::Quaternion(
            pose.orientation.x / q_norm,
            pose.orientation.y / q_norm,
            pose.orientation.z / q_norm,
            pose.orientation.w / q_norm
        ),
        KDL::Vector(pose.position.x, pose.position.y, pose.position.z)
    );
    double roll, pitch, yaw;
    target_frame.M.GetRPY(roll, pitch, yaw);
    RCLCPP_INFO(node_->get_logger(),
        "KDL Frame position: x=%.3f, y=%.3f, z=%.3f. KDL Frame orientation (RPY): roll=%.3f, pitch=%.3f, yaw=%.3f, raw|q|=%.6f",
        target_frame.p.x(), target_frame.p.y(), target_frame.p.z(),
        roll, pitch, yaw, raw_quaternion_norm
    );

    
    // 2. 获取关节初值（优先级: 外部种子 -> joint_states -> 上次成功解）
    KDL::JntArray q_init, q_result;
    std::string seed_source = "none";
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        const rclcpp::Time now = node_->get_clock()->now();
        const bool external_seed_fresh =
            has_external_joint_seed_ &&
            external_joint_seed_.rows() == current_joint_positions_.rows() &&
            external_joint_seed_stamp_.nanoseconds() > 0 &&
            (now - external_joint_seed_stamp_).seconds() <= external_seed_timeout_sec_;
        const bool joint_state_fresh =
            joint_state_received_ &&
            last_joint_state_stamp_.nanoseconds() > 0 &&
            (now - last_joint_state_stamp_).seconds() <= joint_state_timeout_sec_;

        if (external_seed_fresh) {
            q_init = external_joint_seed_;
            seed_source = "external_moveit";
        } else if (joint_state_fresh) {
            q_init = current_joint_positions_;
            seed_source = "joint_states";
        } else if (has_last_successful_joint_positions_ &&
                   last_successful_joint_positions_.rows() == current_joint_positions_.rows()) {
            q_init = last_successful_joint_positions_;
            seed_source = "last_successful";
        }
    }

    if (seed_source == "none" || q_init.rows() == 0) {
        ++consecutive_ik_failures_;
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(),
            *node_->get_clock(),
            1000,
            "无可用IK初值(外部种子/joint_states均不可用)，跳过本次IK"
        );
        return false;
    }

    KDL::JntArray q_result(q_init.rows());

    // 3. IK求解关节位置（优先使用当前关节，再回退到上次成功解作为初值）
    int ik_result = std::numeric_limits<int>::min();
    ik_result = ik_solver_->CartToJnt(q_init, target_frame, q_result);
    if (ik_result < 0 && has_last_successful_joint_positions_ &&
        last_successful_joint_positions_.rows() == q_init.rows()) {
        KDL::JntArray q_retry(q_init.rows());
        q_retry = last_successful_joint_positions_;
        ik_result = ik_solver_->CartToJnt(q_retry, target_frame, q_result);
        RCLCPP_WARN(node_->get_logger(), "IK使用当前关节初值失败，已尝试上次成功解作为初值");
    }

    if (ik_result < 0) {
        RCLCPP_WARN(node_->get_logger(),
            "标准IK失败(%d: %s)，尝试位置优先IK。lastTransDiff=%.6f, lastRotDiff=%.6f, iter=%d",
            ik_result,
            ik_solver_->strError(ik_result),
            ik_solver_->lastTransDiff,
            ik_solver_->lastRotDiff,
            ik_solver_->lastNrOfIter
        );

        ik_result = ik_solver_position_priority_->CartToJnt(q_init, target_frame, q_result);
        if (ik_result < 0 && has_last_successful_joint_positions_ &&
            last_successful_joint_positions_.rows() == q_init.rows()) {
            KDL::JntArray q_retry(q_init.rows());
            q_retry = last_successful_joint_positions_;
            ik_result = ik_solver_position_priority_->CartToJnt(q_retry, target_frame, q_result);
            RCLCPP_WARN(node_->get_logger(), "位置优先IK使用当前关节初值失败，已尝试上次成功解作为初值");
        }
    }

    if (ik_result < 0) {
        RCLCPP_ERROR(node_->get_logger(),
            "IK求解失败: %d (%s), standard(lastTransDiff=%.6f,lastRotDiff=%.6f,iter=%d), relaxed(lastTransDiff=%.6f,lastRotDiff=%.6f,iter=%d)",
            ik_result,
            ik_solver_position_priority_->strError(ik_result),
            ik_solver_->lastTransDiff,
            ik_solver_->lastRotDiff,
            ik_solver_->lastNrOfIter,
            ik_solver_position_priority_->lastTransDiff,
            ik_solver_position_priority_->lastRotDiff,
            ik_solver_position_priority_->lastNrOfIter
        );
        double roll, pitch, yaw;

        target_frame.M.GetRPY(roll, pitch, yaw);

        RCLCPP_ERROR(node_->get_logger(),
            "目标位姿: position=(%.3f, %.3f, %.3f), orientation(RPY)=(%.3f, %.3f, %.3f)",
            target_frame.p.x(), target_frame.p.y(), target_frame.p.z(),
            roll, pitch, yaw
        );

        ++consecutive_ik_failures_;
        return false;
    }

    last_successful_joint_positions_.resize(q_result.rows());
    last_successful_joint_positions_ = q_result;
    has_last_successful_joint_positions_ = true;
    consecutive_ik_failures_ = 0;

    std::ostringstream ss;
    ss << "q_result: [";
    for (unsigned int i = 0; i < q_result.rows(); ++i) {
        ss << q_result(i);
        if (i != q_result.rows() - 1) ss << ", ";
    }
    ss << "]";
    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
    
    // 4. 计算雅可比矩阵，转换速度和加速度
    KDL::Jacobian jacobian(kdl_chain_.getNrOfJoints());
    jacobian_solver_->JntToJac(q_result, jacobian);
    
    // 雅可比伪逆 (6xN -> Nx6)
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, jacobian.columns());
    for (unsigned int i = 0; i < 6; ++i) {
        for (unsigned int j = 0; j < jacobian.columns(); ++j) {
            J(i, j) = jacobian(i, j);
        }
    }
    Eigen::MatrixXd J_pinv = J.completeOrthogonalDecomposition().pseudoInverse();
    
    // 末端速度 -> 关节速度
    Eigen::VectorXd cart_vel(6);
    cart_vel << initial_trajectory_point_.velocity.linear.x,
                initial_trajectory_point_.velocity.linear.y,
                initial_trajectory_point_.velocity.linear.z,
                initial_trajectory_point_.velocity.angular.x,
                initial_trajectory_point_.velocity.angular.y,
                initial_trajectory_point_.velocity.angular.z;
    Eigen::VectorXd joint_vel = J_pinv * cart_vel;
    
    // 末端加速度 -> 关节加速度
    Eigen::VectorXd cart_acc(6);
    cart_acc << initial_trajectory_point_.acceleration.linear.x,
                initial_trajectory_point_.acceleration.linear.y,
                initial_trajectory_point_.acceleration.linear.z,
                initial_trajectory_point_.acceleration.angular.x,
                initial_trajectory_point_.acceleration.angular.y,
                initial_trajectory_point_.acceleration.angular.z;
    Eigen::VectorXd joint_acc = J_pinv * cart_acc;
    
    // 5. 填充JointTrajectory
    initial_joint_trajectory_.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    initial_joint_trajectory_.points.resize(1);
    
    // 设置消息头（关键！）
    initial_joint_trajectory_.header.stamp = node_->get_clock()->now();
    initial_joint_trajectory_.header.frame_id = "base_link";
    
    auto& point = initial_joint_trajectory_.points[0];
    point.positions.resize(6);
    point.velocities.resize(6);
    point.accelerations.resize(6);
    
    for (size_t i = 0; i < 6; ++i) {
        point.positions[i] = q_result(i);
        point.velocities[i] = joint_vel(static_cast<Eigen::Index>(i));
        point.accelerations[i] = joint_acc(static_cast<Eigen::Index>(i));
    }
    point.effort.clear();
    point.time_from_start = rclcpp::Duration::from_seconds(dt_);

    RCLCPP_DEBUG(node_->get_logger(), "JointTrajectory packed: header.stamp=%d.%u, points.size=%zu",
        initial_joint_trajectory_.header.stamp.sec,
        initial_joint_trajectory_.header.stamp.nanosec,
        initial_joint_trajectory_.points.size());
    
    RCLCPP_INFO(node_->get_logger(), "末端数据转关节轨迹完成");
    return true;
}




