#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <Eigen/Dense>
#include <memory>
#include <rclcpp/logging.hpp>

namespace mixcontroller {

/**
 * @brief 实时 IK 求解器，使用 KDL 的雅可比矩阵进行快速求解
 * 
 * 将末端执行器的 Twist（线速度和角速度）转换为关节速度
 * 使用阻尼的伪逆（DLS - Damped Least Squares）处理奇异点
 */
class RealtimeIKSolver {
public:
    /**
     * @brief 构造函数
     * @param chain KDL 运动链
     * @param logger ROS 2 日志对象（用于调试）
     */
    explicit RealtimeIKSolver(const KDL::Chain& chain,
                             rclcpp::Logger logger = rclcpp::get_logger("RealtimeIKSolver"));

    /**
     * @brief 求解 Twist 到关节速度的映射
     * 
     * @param twist 末端执行器的 Twist 消息（线速度 + 角速度）
     * @param current_q 当前关节位置
     * @param joint_velocities [out] 计算得到的关节速度
     * @param damping_factor 阻尼因子（用于 DLS），默认 0.01
     * @return 是否求解成功
     */
    bool solveTwistToJointVelocity(
        const geometry_msgs::msg::Twist& twist,
        const KDL::JntArray& current_q,
        KDL::JntArray& joint_velocities,
        double damping_factor = 0.01
    );

    /**
     * @brief 设置关节速度的上限
     * @param max_velocities 每个关节的最大速度（rad/s）
     */
    void set_max_velocities(const std::vector<double>& max_velocities);

    /**
     * @brief 检查当前位置是否接近奇异点
     * @param current_q 当前关节位置
     * @return 奇异度指数（0-1，越接近 1 越接近奇异点）
     */
    double check_singularity(const KDL::JntArray& current_q);

private:
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver_;
    KDL::Jacobian jacobian_;
    std::vector<double> max_velocities_;
    rclcpp::Logger logger_;

    /**
     * @brief 计算 Jacobian 的广义逆（使用 SVD + 阻尼）
     * @param J 原始 Jacobian 矩阵
     * @param damping_factor 阻尼因子
     * @return 伪逆矩阵
     */
    Eigen::MatrixXd compute_damped_pseudoinverse(
        const Eigen::MatrixXd& J,
        double damping_factor
    );

    /**
     * @brief 对关节速度应用限制
     * @param qd 关节速度（被修改）
     */
    void apply_velocity_limits(std::vector<double>& qd);
};

}  // namespace mixcontroller
