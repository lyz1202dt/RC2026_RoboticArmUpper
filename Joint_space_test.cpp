#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <algorithm>
#include <cmath>
#include <utility>

class HybridMotionPlanner {
private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface::SharedPtr move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 规划参数配置
    const double SWITCH_DISTANCE_THRESHOLD = 0.15;  // 切换距离阈值（米）
    const double CARTESIAN_GOAL_TOLERANCE = 0.001;   // 笛卡尔空间目标容差
    const double JOINT_GOAL_TOLERANCE = 0.005;       // 关节空间目标容差
    const double VELOCITY_SCALING = 0.5;             // 速度缩放因子
    const double ACCELERATION_SCALING = 0.5;         // 加速度缩放因子
    
public:
    enum class PlanningMode {
        JOINT_SPACE,    // 关节空间规划模式
        CARTESIAN_SPACE, // 笛卡尔空间规划模式
        HYBRID          // 混合模式（自动切换）
    };
    
    HybridMotionPlanner(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
        // 初始化 MoveGroup 接口
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, "manipulator");
        
        // 配置基本参数
        move_group_->setPlanningTime(10.0);
        move_group_->setMaxVelocityScalingFactor(VELOCITY_SCALING);
        move_group_->setMaxAccelerationScalingFactor(ACCELERATION_SCALING);
        
        // 初始化 TF 监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        RCLCPP_INFO(node_->get_logger(), "混合规划器初始化完成");
    }
    
    // 主入口：混合模式规划
    bool planHybridMotion(const geometry_msgs::msg::Pose& target_pose,
                         const std::string& reference_frame = "base_link") {
        // 获取当前末端位置
        geometry_msgs::msg::Pose current_pose = getCurrentEndEffectorPose(reference_frame);
        
        // 计算到目标的距离
        double distance = calculateDistance(current_pose, target_pose);
        RCLCPP_INFO(node_->get_logger(), "当前末端到目标距离: %.4f m", distance);
        
        // 根据距离选择规划模式
        if (distance > SWITCH_DISTANCE_THRESHOLD) {
            RCLCPP_INFO(node_->get_logger(), "使用关节空间规划（大段移动）");
            return planJointSpaceToApproach(target_pose, distance, reference_frame);
        } else {
            RCLCPP_INFO(node_->get_logger(), "使用笛卡尔空间规划（末端接近）");
            return planCartesianSpace(target_pose, reference_frame);
        }
    }
    
private:
    // 关节空间规划：移动到接近目标的安全位置
    bool planJointSpaceToApproach(const geometry_msgs::msg::Pose& target_pose,
                                  double full_distance,
                                  const std::string& reference_frame) {
        // 计算接近位置（在目标位置一定距离之外）
        double approach_distance = SWITCH_DISTANCE_THRESHOLD * 1.2;
        geometry_msgs::msg::Pose approach_pose = calculateApproachPose(
            target_pose, approach_distance, reference_frame);
        
        // 设置规划参数
        move_group_->setPoseTarget(approach_pose, "tool0");
        move_group_->setGoalOrientationTolerance(0.1); // 关节空间对姿态要求放宽
        move_group_->setGoalPositionTolerance(SWITCH_DISTANCE_THRESHOLD * 0.5);
        
        // 执行关节空间规划（虽然设置的是位姿目标，但 MoveIt 会尝试 IK 求解）
        auto plan = move_group_->plan();
        
        if (plan && plan->trajectory.joint_trajectory.points.size() > 0) {
            RCLCPP_INFO(node_->get_logger(), "关节空间规划成功，规划 %d 个轨迹点",
                       (int)plan->trajectory.joint_trajectory.points.size());
            
            // 可选：直接执行
            // move_group_->execute(*plan);
            
            // 执行后检查是否需要继续笛卡尔空间规划
            return checkAndContinueCartesianMotion(target_pose, reference_frame);
        }
        
        RCLCPP_WARN(node_->get_logger(), "关节空间规划失败，尝试备用方案");
        return planFallbackMotion(target_pose, reference_frame);
    }
    
    // 笛卡尔空间规划：末端精确接近目标
    bool planCartesianSpace(const geometry_msgs::msg::Pose& target_pose,
                           const std::string& reference_frame) {
        // 获取当前位姿作为起始点
        geometry_msgs::msg::Pose start_pose = getCurrentEndEffectorPose(reference_frame);
        
        // 构建笛卡尔路径请求
        std::vector<geometry_msgs::msg::Pose> waypoints;
        
        // 生成中间路径点以提高成功率
        int num_intermediate_points = 5;
        for (int i = 1; i <= num_intermediate_points; ++i) {
            double t = static_cast<double>(i) / num_intermediate_points;
            geometry_msgs::msg::Pose intermediate_pose = interpolatePose(
                start_pose, target_pose, t);
            waypoints.push_back(intermediate_pose);
        }
        
        // 设置笛卡尔空间规划参数
        move_group_->setPoseTarget(target_pose, "tool0");
        move_group_->setGoalPositionTolerance(CARTESIAN_GOAL_TOLERANCE);
        move_group_->setGoalOrientationTolerance(CARTESIAN_GOAL_TOLERANCE);
        
        // 执行笛卡尔空间规划
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = move_group_->computeCartesianPath(
            waypoints, 
            0.01,    // 步长（米）
            0.0,     // 跳跃阈值（0 表示不允许跳跃）
            trajectory,
            true,    // 是否避免碰撞
            nullptr  // 路径约束
        );
        
        RCLCPP_INFO(node_->get_logger(), "笛卡尔路径规划完成度: %.2f%%", fraction * 100);
        
        if (fraction >= 0.9) {
            RCLCPP_INFO(node_->get_logger(), "笛卡尔空间规划成功");
            // 可选：执行轨迹
            // move_group_->execute(trajectory);
            return true;
        } else if (fraction >= 0.5) {
            RCLCPP_WARN(node_->get_logger(), "笛卡尔规划部分成功，尝试执行已规划部分");
            // 可以执行已规划的部分，然后继续尝试
            return false;
        }
        
        RCLCPL_ERROR(node_->get_logger(), "笛卡尔空间规划失败");
        return false;
    }
    
    // 检查是否需要继续笛卡尔空间运动
    bool checkAndContinueCartesianMotion(const geometry_msgs::msg::Pose& target_pose,
                                         const std::string& reference_frame) {
        geometry_msgs::msg::Pose current_pose = getCurrentEndEffectorPose(reference_frame);
        double remaining_distance = calculateDistance(current_pose, target_pose);
        
        if (remaining_distance > SWITCH_DISTANCE_THRESHOLD * 0.8) {
            // 需要继续笛卡尔空间规划
            RCLCPP_INFO(node_->get_logger(), "剩余距离 %.4f m，继续笛卡尔规划",
                       remaining_distance);
            return planCartesianSpace(target_pose, reference_frame);
        }
        
        RCLCPP_INFO(node_->get_logger(), "已到达接近位置，无需额外运动");
        return true;
    }
    
    // 备用方案：当主规划失败时使用
    bool planFallbackMotion(const geometry_msgs::msg::Pose& target_pose,
                           const std::string& reference_frame) {
        // 尝试使用分步规划
        std::vector<geometry_msgs::msg::Pose> intermediate_goals;
        
        // 将目标分解为多个中间点
        geometry_msgs::msg::Pose current_pose = getCurrentEndEffectorPose(reference_frame);
        
        for (int i = 1; i <= 3; ++i) {
            double ratio = static_cast<double>(i) / 4.0;
            geometry_msgs::msg::Pose intermediate = interpolatePose(
                current_pose, target_pose, ratio);
            
            // 限制每次移动的距离
            geometry_msgs::msg::Pose limited_pose = limitStepSize(
                current_pose, intermediate, SWITCH_DISTANCE_THRESHOLD * 0.5);
            
            intermediate_goals.push_back(limited_pose);
            
            // 依次规划每个中间点
            move_group_->setPoseTarget(limited_pose, "tool0");
            auto plan = move_group_->plan();
            
            if (!plan || plan->trajectory.joint_trajectory.points.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "在中间点 %d 处规划失败", i);
                return false;
            }
            
            // 执行当前段
            move_group_->execute(*plan);
            current_pose = getCurrentEndEffectorPose(reference_frame);
        }
        
        return true;
    }
    
public:
    // 辅助函数：获取当前末端执行器位姿
    geometry_msgs::msg::Pose getCurrentEndEffectorPose(const std::string& reference_frame) {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = reference_frame;
        pose_stamped.header.stamp = rclcpp::Time(0);
        pose_stamped.pose = move_group_->getCurrentPose("tool0").pose;
        return pose_stamped.pose;
    }
    
    // 辅助函数：计算两个位姿之间的距离
    double calculateDistance(const geometry_msgs::msg::Pose& pose1,
                            const geometry_msgs::msg::Pose& pose2) {
        double dx = pose1.position.x - pose2.position.x;
        double dy = pose1.position.y - pose2.position.y;
        double dz = pose1.position.z - pose2.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    
    // 辅助函数：计算接近位置（沿目标进入方向的反方向）
    geometry_msgs::msg::Pose calculateApproachPose(const geometry_msgs::msg::Pose& target,
                                                   double approach_distance,
                                                   const std::string& reference_frame) {
        // 获取当前末端到目标的向量
        geometry_msgs::msg::Pose current_pose = getCurrentEndEffectorPose(reference_frame);
        
        // 计算进入方向（从当前末端指向目标）
            // 三维向量，用于表示从当前位置到目标位置的方向和距离。
        tf2::Vector3 approach_direction(
            target.position.x - current_pose.position.x,
            target.position.y - current_pose.position.y,
            target.position.z - current_pose.position.z
        );

            //  归一化 approach_direction
        approach_direction.normalize();
        
        // 计算接近位置（在目标前方 approach_distance 处）
        geometry_msgs::msg::Pose approach_pose = target;
        approach_pose.position.x -= approach_direction.x() * approach_distance;
        approach_pose.position.y -= approach_direction.y() * approach_distance;
        approach_pose.position.z -= approach_direction.z() * approach_distance;
        
        return approach_pose;
    }
    
    // 辅助函数：位姿插值
    geometry_msgs::msg::Pose interpolatePose(const geometry_msgs::msg::Pose& start,
                                             const geometry_msgs::msg::Pose& end,
                                             double t) {
        geometry_msgs::msg::Pose result;
        
        // 线性插值位置
        result.position.x = start.position.x + t * (end.position.x - start.position.x);
        result.position.y = start.position.y + t * (end.position.y - start.position.y);
        result.position.z = start.position.z + t * (end.position.z - start.position.z);
        
        // 使用四元数插值姿态（简化版：直接使用目标姿态）
        // 实际应用中应使用 slerp 进行平滑插值
        result.orientation = end.orientation;
        
        return result;
    }
    
    // 辅助函数：限制单步移动距离
    geometry_msgs::msg::Pose limitStepSize(const geometry_msgs::msg::Pose& current,
                                           const geometry_msgs::msg::Pose& desired,
                                           double max_step) {
        geometry_msgs::msg::Pose result = desired;
        double distance = calculateDistance(current, desired);
        
        if (distance > max_step) {
            double scale = max_step / distance;
            result.position.x = current.position.x + 
                (desired.position.x - current.position.x) * scale;
            result.position.y = current.position.y + 
                (desired.position.y - current.position.y) * scale;
            result.position.z = current.position.z + 
                (desired.position.z - current.position.z) * scale;
        }
        
        return result;
    }
    
    // 设置切换距离阈值
    void setSwitchDistanceThreshold(double threshold) {
        // 确保阈值在合理范围内
        if (threshold > 0.01 && threshold < 1.0) {
            SWITCH_DISTANCE_THRESHOLD = threshold;
            RCLCPP_INFO(node_->get_logger(), "切换阈值设置为: %.4f m", threshold);
        }
    }
    
    // 获取当前规划模式
    PlanningMode getCurrentMode() const {
        return current_mode_;
    }
    
private:
    PlanningMode current_mode_ = PlanningMode::HYBRID;
};
