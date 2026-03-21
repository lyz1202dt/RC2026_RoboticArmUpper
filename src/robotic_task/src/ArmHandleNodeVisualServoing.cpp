#include "ArmHandleNodeVisualServoing.hpp"
#include <cmath>
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
    
    // 订阅关节状态
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&VisualServoingArmHandleNode::jointStateCallback, this, std::placeholders::_1)
    );
}

VisualServoingArmHandleNode::~VisualServoingArmHandleNode() = default;

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
    
    // 创建雅可比求解器
    jacobian_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(kdl_chain_);
    
    RCLCPP_INFO(node_->get_logger(), "KDL初始化成功，关节数: %d", nj);
    return true;
}

void VisualServoingArmHandleNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    
    // 关节名称映射
    std::vector<std::string> joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    
    for (size_t i = 0; i < joint_names.size(); ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
        if (it != msg->name.end()) {
            size_t idx = std::distance(msg->name.begin(), it);
            if (idx < msg->position.size()) {
                current_joint_positions_(i) = msg->position[idx];
            }
        }
    }
    joint_state_received_ = true;
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
    twist_publisher_->publish(current_desired_velocity_); // 发布Twist消息
}

void VisualServoingArmHandleNode::SendTrajectoryCommand() {
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

void VisualServoingArmHandleNode::TotalPackaing(Eigen::Vector3d& current_position, Eigen::Vector3d& target_position) {
    // 计算路径向量
    Eigen::Vector3d path_vector = CalculatePath(current_position, target_position);

    // 计算Twist命令
    // geometry_msgs::msg::Twist twist_msg = CalculateTwist(path_vector);

    ComputationalSpeed(); // 计算当前期望速度和位置
    
    // 发送Twist命令
    // SendTwistCommand(twist_msg);

    SendTrajectoryCommand();

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
    // Quaternion -> Euler angles
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
        "PoseErr: dpos=(%.5f, %.5f, %.5f), dRPY=(%.5f, %.5f, %.5f)",
        final_desired_position_.pose.position.x - crrent_desired_position_.pose.position.x,
        final_desired_position_.pose.position.y - crrent_desired_position_.pose.position.y,
        final_desired_position_.pose.position.z - crrent_desired_position_.pose.position.z,
        final_roll - roll,
        final_pitch - pitch,
        final_yaw - yaw
    );
    
    
    // 计算当前期望加速度
    geometry_msgs::msg::Twist current_desired_acceleration;
    current_desired_acceleration.linear.x = (current_desired_velocity_.linear.x - last_desired_velocity_.linear.x) / dt_;
    current_desired_acceleration.linear.y = (current_desired_velocity_.linear.y - last_desired_velocity_.linear.y) / dt_;
    current_desired_acceleration.linear.z = (current_desired_velocity_.linear.z - last_desired_velocity_.linear.z) / dt_;
    current_desired_acceleration.angular.x = (current_desired_velocity_.angular.x - last_desired_velocity_.angular.x) / dt_;
    current_desired_acceleration.angular.y = (current_desired_velocity_.angular.y - last_desired_velocity_.angular.y) / dt_;
    current_desired_acceleration.angular.z = (current_desired_velocity_.angular.z - last_desired_velocity_.angular.z) / dt_;

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
    // 更新当前期望朝向 
    tf2::Quaternion current_desired_velocity_quaternion;
    current_desired_velocity_quaternion.setRPY(
        current_desired_velocity_.angular.x ,
        current_desired_velocity_.angular.y,
        current_desired_velocity_.angular.z 
    );
    crrent_desired_position_.pose.orientation.x += current_desired_velocity_quaternion.x() * dt_;
    crrent_desired_position_.pose.orientation.y += current_desired_velocity_quaternion.y() * dt_;
    crrent_desired_position_.pose.orientation.z += current_desired_velocity_quaternion.z() * dt_;
    crrent_desired_position_.pose.orientation.w += current_desired_velocity_quaternion.w() * dt_;

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
    
    rclcpp::Time now = node_->get_clock()->now();
    rclcpp::Duration dt = now - initial_joint_trajectory_.header.stamp;
    duration_ns_ = dt.nanoseconds();
    duration_sec_ = dt.seconds();
}


TrajectoryPoint VisualServoingArmHandleNode::getInitialTrajectory() const {
    return initial_trajectory_point_;
}


// 使用KDL将末端数据转换成关节数据
void VisualServoingArmHandleNode::PointToTrajectoryPoint() {
    if (!ik_solver_ || !jacobian_solver_) {
        RCLCPP_ERROR(node_->get_logger(), "KDL求解器未初始化");
        return;
    }
    
    // 检查是否收到关节状态
    if (!joint_state_received_) {
        RCLCPP_WARN(node_->get_logger(), "未收到关节状态，无法进行IK求解");
        return;
    }
    
    // 1. 从末端位姿构建KDL::Frame
    const auto& pose = initial_trajectory_point_.pose.pose;
    KDL::Frame target_frame(
        KDL::Rotation::Quaternion(
            pose.orientation.x, pose.orientation.y, 
            pose.orientation.z, pose.orientation.w
        ),
        KDL::Vector(pose.position.x, pose.position.y, pose.position.z)
    );
    
    // 2. 获取当前关节位置作为初值
    KDL::JntArray q_init, q_result;
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        q_init = current_joint_positions_;
    }
    q_result.resize(q_init.rows());
    
    // 3. IK求解关节位置
    int ik_result = ik_solver_->CartToJnt(q_init, target_frame, q_result);
    if (ik_result < 0) {
        RCLCPP_ERROR(node_->get_logger(), "IK求解失败: %d", ik_result);
        return;
    }
    
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

    
    
    RCLCPP_INFO(node_->get_logger(), "末端数据转关节轨迹完成");
}






