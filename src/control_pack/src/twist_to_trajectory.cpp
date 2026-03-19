#include "control_pack/twist_to_trajectory.hpp"
#include <cmath>

namespace mixcontroller {

TwistToTrajectoryConverter::TwistToTrajectoryConverter(
    std::shared_ptr<RealtimeIKSolver> ik_solver,
    double dt,
    rclcpp::Logger logger)
    : ik_solver_(ik_solver), dt_(dt), logger_(logger) {
    last_joint_velocities_.resize(6);
    last_joint_accelerations_.resize(6);
    for (int i = 0; i < 6; ++i) {
        last_joint_velocities_(i) = 0.0;
        last_joint_accelerations_(i) = 0.0;
    }
    RCLCPP_INFO(logger_, "TwistToTrajectoryConverter 初始化完成，周期: %.3f s", dt);
}

trajectory_msgs::msg::JointTrajectoryPoint TwistToTrajectoryConverter::convert_twist_to_trajectory_point(
    const geometry_msgs::msg::Twist& twist,
    const KDL::JntArray& current_q,
    const KDL::JntArray& current_dq,
    const std::vector<std::string>& joint_names) {
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(joint_names.size());
    point.velocities.resize(joint_names.size());
    point.accelerations.resize(joint_names.size());

    // 使用 IK 求解器获取关节速度
    KDL::JntArray joint_velocities(joint_names.size());
    if (!ik_solver_->solveTwistToJointVelocity(twist, current_q, joint_velocities)) {
        RCLCPP_WARN(logger_, "IK 求解失败，返回当前状态");
        for (size_t i = 0; i < joint_names.size(); ++i) {
            point.positions[i] = current_q(i);
            point.velocities[i] = 0.0;
            point.accelerations[i] = 0.0;
        }
        return point;
    }

    // 计算加速度
    KDL::JntArray joint_accelerations(joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
        joint_accelerations(i) = (joint_velocities(i) - current_dq(i)) / dt_;
    }

    // 应用平滑
    if (enable_smoothing_) {
        apply_smoothing(joint_velocities, joint_accelerations);
    }

    // 积分获得新位置
    for (size_t i = 0; i < joint_names.size(); ++i) {
        point.positions[i] = current_q(i) + current_dq(i) * dt_ + 0.5 * joint_accelerations(i) * dt_ * dt_;
        point.velocities[i] = joint_velocities(i);
        point.accelerations[i] = joint_accelerations(i);
    }

    return point;
}

trajectory_msgs::msg::JointTrajectory TwistToTrajectoryConverter::convert_twist_to_micro_trajectory(
    const geometry_msgs::msg::Twist& twist,
    const KDL::JntArray& current_q,
    const KDL::JntArray& current_dq,
    const std::vector<std::string>& joint_names,
    size_t num_points) {
    
    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.joint_names = joint_names;
    trajectory.points.reserve(num_points);

    // 当前状态
    KDL::JntArray q = current_q;
    KDL::JntArray dq = current_dq;

    // 生成 N 个轨迹点
    for (size_t i = 0; i < num_points; ++i) {
        auto point = convert_twist_to_trajectory_point(twist, q, dq, joint_names);
        
        // 设置时间戳
        point.time_from_start.sec = i / 100;  // 10ms per point
        point.time_from_start.nanosec = (i % 100) * 10000000;
        
        trajectory.points.push_back(point);

        // 更新状态用于下一个点
        q.resize(joint_names.size());
        dq.resize(joint_names.size());
        for (size_t j = 0; j < joint_names.size(); ++j) {
            q(j) = point.positions[j];
            dq(j) = point.velocities[j];
        }
    }

    return trajectory;
}

void TwistToTrajectoryConverter::apply_smoothing(KDL::JntArray& qd, KDL::JntArray& qdd) {
    // 一阶低通滤波：q_new = α·q + (1-α)·q_old
    for (size_t i = 0; i < qd.rows(); ++i) {
        qd(i) = smoothing_factor_ * qd(i) + (1.0 - smoothing_factor_) * last_joint_velocities_(i);
        qdd(i) = smoothing_factor_ * qdd(i) + (1.0 - smoothing_factor_) * last_joint_accelerations_(i);
    }

    // 保存当前值用于下一次平滑
    last_joint_velocities_ = qd;
    last_joint_accelerations_ = qdd;
}

void TwistToTrajectoryConverter::limit_acceleration_derivative(std::vector<double>& qdd, double max_jerk) {
    for (size_t i = 0; i < qdd.size() && i < last_joint_accelerations_.rows(); ++i) {
        double acc_derivative = std::abs(qdd[i] - last_joint_accelerations_(i)) / dt_;
        if (acc_derivative > max_jerk) {
            // 限制加速度变化率
            double sign = (qdd[i] - last_joint_accelerations_(i) > 0) ? 1.0 : -1.0;
            qdd[i] = last_joint_accelerations_(i) + sign * max_jerk * dt_;
        }
    }
}

}  // namespace mixcontroller
