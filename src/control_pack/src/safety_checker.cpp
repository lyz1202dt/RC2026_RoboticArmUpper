#include "control_pack/safety_checker.hpp"
#include <cmath>
#include <algorithm>

namespace mixcontroller {

SafetyChecker::SafetyChecker(size_t num_joints, rclcpp::Logger logger)
    : num_joints_(num_joints), logger_(logger) {
    // 初始化默认限制值
    limits_.max_velocity.resize(num_joints, 2.0);      // rad/s
    limits_.max_acceleration.resize(num_joints, 5.0);  // rad/s^2
    limits_.max_effort.resize(num_joints, 200.0);      // Nm
    limits_.min_position.resize(num_joints, -M_PI);    // -180°
    limits_.max_position.resize(num_joints, M_PI);     // +180°

    RCLCPP_INFO(logger_, "SafetyChecker 初始化完成，关节数: %zu", num_joints);
}

bool SafetyChecker::check_and_limit_velocities(std::vector<double>& velocities) {
    if (velocities.size() != num_joints_) {
        RCLCPP_ERROR(logger_, "速度向量大小不匹配: %zu != %zu", velocities.size(), num_joints_);
        return false;
    }

    bool triggered = false;
    for (size_t i = 0; i < num_joints_; ++i) {
        if (clamp_value(velocities[i], 
                       -limits_.max_velocity[i],
                        limits_.max_velocity[i])) {
            triggered = true;
            RCLCPP_WARN(logger_, "关节 %zu 速度被限制: %.3f → %.3f rad/s",
                       i, velocities[i], velocities[i]);
        }
    }

    if (triggered) {
        last_safety_triggered_ = true;
    }
    return triggered;
}

bool SafetyChecker::check_and_limit_accelerations(std::vector<double>& accelerations) {
    if (accelerations.size() != num_joints_) {
        RCLCPP_ERROR(logger_, "加速度向量大小不匹配: %zu != %zu", accelerations.size(), num_joints_);
        return false;
    }

    bool triggered = false;
    for (size_t i = 0; i < num_joints_; ++i) {
        if (clamp_value(accelerations[i],
                       -limits_.max_acceleration[i],
                        limits_.max_acceleration[i])) {
            triggered = true;
            RCLCPP_WARN(logger_, "关节 %zu 加速度被限制: %.3f → %.3f rad/s^2",
                       i, accelerations[i], accelerations[i]);
        }
    }

    if (triggered) {
        last_safety_triggered_ = true;
    }
    return triggered;
}

bool SafetyChecker::check_and_limit_efforts(std::vector<double>& efforts) {
    if (efforts.size() != num_joints_) {
        RCLCPP_ERROR(logger_, "力矩向量大小不匹配: %zu != %zu", efforts.size(), num_joints_);
        return false;
    }

    bool triggered = false;
    for (size_t i = 0; i < num_joints_; ++i) {
        if (clamp_value(efforts[i],
                       -limits_.max_effort[i],
                        limits_.max_effort[i])) {
            triggered = true;
            RCLCPP_WARN(logger_, "关节 %zu 力矩被限制: %.3f → %.3f Nm",
                       i, efforts[i], efforts[i]);
        }
    }

    if (triggered) {
        last_safety_triggered_ = true;
    }
    return triggered;
}

bool SafetyChecker::check_and_limit_positions(std::vector<double>& positions) {
    if (positions.size() != num_joints_) {
        RCLCPP_ERROR(logger_, "位置向量大小不匹配: %zu != %zu", positions.size(), num_joints_);
        return false;
    }

    bool triggered = false;
    for (size_t i = 0; i < num_joints_; ++i) {
        if (clamp_value(positions[i],
                       limits_.min_position[i],
                       limits_.max_position[i])) {
            triggered = true;
            RCLCPP_WARN(logger_, "关节 %zu 位置被限制在范围内 [%.3f, %.3f] rad",
                       i, limits_.min_position[i], limits_.max_position[i]);
        }
    }

    if (triggered) {
        last_safety_triggered_ = true;
    }
    return triggered;
}

bool SafetyChecker::clamp_value(double& value, double min_val, double max_val) {
    if (value < min_val) {
        value = min_val;
        return true;
    } else if (value > max_val) {
        value = max_val;
        return true;
    }
    return false;
}

}  // namespace mixcontroller
