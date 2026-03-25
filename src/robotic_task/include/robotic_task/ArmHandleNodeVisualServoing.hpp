#pragma once

#include "SegmentedVelocityController.hpp"
#include "TragectorySmoother.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp_action/create_client.hpp>
#include <robot_interfaces/action/catch.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


// 轨迹点结构体
struct TrajectoryPoint {
    geometry_msgs::msg::PoseStamped pose;      // 位置和朝向
    geometry_msgs::msg::Twist velocity;        // 速度
    geometry_msgs::msg::Twist acceleration;    // 加速度
    rclcpp::Time timestamp;                    // 时间戳
};

class VisualServoingArmHandleNode {
public:
    explicit VisualServoingArmHandleNode(const rclcpp::Node::SharedPtr node);
    ~VisualServoingArmHandleNode();

    
    // total package control flow
    void TotalPackaing(
        Eigen::Vector3d& current_position, Eigen::Vector3d& target_position,
        geometry_msgs::msg::PoseStamped& actual_position, geometry_msgs::msg::PoseStamped& final_desired_position);

        // calculate path vector
    Eigen::Vector3d CalculatePath(Eigen::Vector3d& current_position, Eigen::Vector3d& target_position);

private:
    void resetServoState(const geometry_msgs::msg::PoseStamped& actual_position,
                         const geometry_msgs::msg::PoseStamped& final_desired_position);

    // current 
    geometry_msgs::msg::PoseStamped CurrentPose_;
    geometry_msgs::msg::PoseStamped TargetPose_;
    geometry_msgs::msg::PoseStamped LastTargetPose_;

    rclcpp::Node::SharedPtr node_;
    Eigen::Vector3d path_vector_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;



    // calculate twist
    geometry_msgs::msg::Twist CalculateTwist(Eigen::Vector3d& path_vector);

    // calculate target orientation from path vector
    Eigen::Quaterniond CalculateTargetOrientation(const Eigen::Vector3d& path_vector);

    // send twist command
    void SendTwistCommand(const geometry_msgs::msg::Twist& twist_msg);

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

    geometry_msgs::msg::PoseStamped crrent_desired_position_; // 当前期望位置
    geometry_msgs::msg::PoseStamped actual_position_; // 实际位置
    geometry_msgs::msg::PoseStamped final_desired_position_; // 最终期望位置
    
    TrajectoryPoint initial_trajectory_point_; // t0时刻的完整轨迹点
    trajectory_msgs::msg::JointTrajectory initial_joint_trajectory_;

    geometry_msgs::msg::Twist current_desired_velocity_; // 当前期望速度
    geometry_msgs::msg::Twist last_desired_velocity_; // 上次期望速度
    double kp_ = 1.0; // 比例增益
    double dt_ = 0.01; // 控制周期
    bool is_first_iteration_ = true; // 是否是第一次迭代
    bool servo_state_initialized_ = false; // 视觉伺服内部状态是否已初始化
    
    void ComputationalSpeed();

    void SendTrajectoryCommand();

    // 获取t0时刻的轨迹点
    TrajectoryPoint getInitialTrajectory() const;

    void PointToTrajectoryPoint();

    // KDL相关
    KDL::Chain kdl_chain_;
    KDL::Tree kdl_tree_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
    
    // 关节状态
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    KDL::JntArray current_joint_positions_;
    std::mutex joint_state_mutex_;
    bool joint_state_received_{false};
    
    // URDF参数客户端
    rclcpp::SyncParametersClient::SharedPtr robot_description_client_;
    
    // 初始化KDL
    bool initKDL();
    
    // 关节状态回调
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    int64_t duration_ns_;

    double duration_sec_;
    
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr initial_joint_trajectory_publisher_;



};






