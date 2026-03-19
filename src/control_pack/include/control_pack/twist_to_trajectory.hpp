#pragma once

#include "control_pack/realtime_ik_solver.hpp"
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <kdl/jntarray.hpp>
#include <memory>
#include <vector>
#include <rclcpp/logging.hpp>

namespace mixcontroller {

/**
 * @brief Twist 到轨迹的动态转换器
 * 
 * 将末端执行器的 Twist 命令（实时速度）转换为关节轨迹段
 * 支持生成微轨迹（3-5 个点），用于轨迹缓冲队列
 */
class TwistToTrajectoryConverter {
public:
    /**
     * @brief 构造函数
     * @param ik_solver IK 求解器对象
     * @param dt 控制周期（秒），默认 0.01（100Hz）
     * @param logger ROS 2 日志对象
     */
    explicit TwistToTrajectoryConverter(
        std::shared_ptr<RealtimeIKSolver> ik_solver,
        double dt = 0.01,
        rclcpp::Logger logger = rclcpp::get_logger("TwistToTrajectoryConverter")
    );

    /**
     * @brief 将 Twist 转换为单个轨迹点
     * 
     * @param twist 末端执行器 Twist 命令
     * @param current_q 当前关节位置
     * @param current_dq 当前关节速度
     * @param joint_names 关节名称
     * @return 转换后的轨迹点
     */
    trajectory_msgs::msg::JointTrajectoryPoint convert_twist_to_trajectory_point(
        const geometry_msgs::msg::Twist& twist,
        const KDL::JntArray& current_q,
        const KDL::JntArray& current_dq,
        const std::vector<std::string>& joint_names
    );

    /**
     * @brief 将 Twist 转换为微轨迹（多个点）
     * 
     * @param twist 末端执行器 Twist 命令
     * @param current_q 当前关节位置
     * @param current_dq 当前关节速度
     * @param joint_names 关节名称
     * @param num_points 生成的轨迹点数，通常 3-5
     * @return 生成的轨迹消息
     */
    trajectory_msgs::msg::JointTrajectory convert_twist_to_micro_trajectory(
        const geometry_msgs::msg::Twist& twist,
        const KDL::JntArray& current_q,
        const KDL::JntArray& current_dq,
        const std::vector<std::string>& joint_names,
        size_t num_points = 3
    );

    /**
     * @brief 设置加速度平滑因子
     * @param factor 平滑因子（0-1），值越高平滑度越高但响应越慢
     */
    void set_smoothing_factor(double factor) { smoothing_factor_ = factor; }

    /**
     * @brief 启用/禁用速度平滑滤波
     */
    void set_enable_smoothing(bool enable) { enable_smoothing_ = enable; }

private:
    std::shared_ptr<RealtimeIKSolver> ik_solver_;
    double dt_;  // 控制周期
    KDL::JntArray last_joint_velocities_;  // 前一个周期的关节速度
    KDL::JntArray last_joint_accelerations_;  // 前一个周期的关节加速度
    rclcpp::Logger logger_;
    
    // 平滑参数
    double smoothing_factor_ = 0.1;  // 加速度平滑因子
    bool enable_smoothing_ = true;

    /**
     * @brief 对关节速度应用低通滤波平滑
     * @param qd 关节速度
     * @param qdd 关节加速度
     */
    void apply_smoothing(KDL::JntArray& qd, KDL::JntArray& qdd);

    /**
     * @brief 限制加速度变化率
     * @param qdd 关节加速度
     * @param max_jerk 最大加速度变化率
     */
    void limit_acceleration_derivative(std::vector<double>& qdd, double max_jerk = 10.0);
};

}  // namespace mixcontroller
