#pragma once

#include <vector>
#include <string>
#include <rclcpp/logging.hpp>

namespace mixcontroller {

/**
 * @brief 两层安全限制检查器
 * 
 * 第一层：控制器层级检查
 * - 验证关节速度、加速度、力矩是否在限制范围内
 * - 自动调节超限值
 * 
 * 第二层：驱动层级检查（在 mycontrol.cpp 中）
 * - USB 发送前再次验证
 */
class SafetyChecker {
public:
    /**
     * @brief 关节限制参数结构体
     */
    struct JointLimits {
        std::vector<double> max_velocity;      // 关节最大速度（rad/s）
        std::vector<double> max_acceleration;  // 关节最大加速度（rad/s^2）
        std::vector<double> max_effort;        // 关节最大力矩（Nm）
        std::vector<double> min_position;      // 关节最小位置（rad）
        std::vector<double> max_position;      // 关节最大位置（rad）
    };

    /**
     * @brief 构造函数
     * @param num_joints 关节数量
     * @param logger ROS 2 日志对象
     */
    explicit SafetyChecker(size_t num_joints,
                          rclcpp::Logger logger = rclcpp::get_logger("SafetyChecker"));

    /**
     * @brief 设置关节限制参数
     * @param limits 限制参数结构体
     */
    void set_limits(const JointLimits& limits) { limits_ = limits; }

    /**
     * @brief 检查并限制关节速度
     * 
     * @param velocities 关节速度值
     * @return 是否有限制被触发
     */
    bool check_and_limit_velocities(std::vector<double>& velocities);

    /**
     * @brief 检查并限制关节加速度
     * 
     * @param accelerations 关节加速度值
     * @return 是否有限制被触发
     */
    bool check_and_limit_accelerations(std::vector<double>& accelerations);

    /**
     * @brief 检查并限制关节力矩
     * 
     * @param efforts 关节力矩值
     * @return 是否有限制被触发
     */
    bool check_and_limit_efforts(std::vector<double>& efforts);

    /**
     * @brief 检查并限制关节位置
     * 
     * @param positions 关节位置值
     * @return 是否有限制被触发
     */
    bool check_and_limit_positions(std::vector<double>& positions);

    /**
     * @brief 获取上一次的安全检查状态
     */
    bool get_last_safety_triggered() const { return last_safety_triggered_; }

    /**
     * @brief 重置安全检查状态
     */
    void reset() { last_safety_triggered_ = false; }

private:
    size_t num_joints_;
    JointLimits limits_;
    rclcpp::Logger logger_;
    bool last_safety_triggered_{false};

    /**
     * @brief 对单个值应用限制
     * @param value 要限制的值
     * @param min_val 最小值
     * @param max_val 最大值
     * @return 是否被限制
     */
    static bool clamp_value(double& value, double min_val, double max_val);
};

}  // namespace mixcontroller
