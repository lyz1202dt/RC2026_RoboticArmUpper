/*************************************
 提供两种轨迹平滑方法
*/

#pragma once
#include <cstddef>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <memory>
#include <algorithm>

class TrajectorySmoother{
    private :
        double max_velocity_;           // 最大速度限制
        double max_acceleration_;       // 最大加速度限制
        double velocity_scaling_;       // 速度缩放系数
        double acceleration_scaling_;   // 加速度缩放系数

        // Helper function to calculate S-curve profile
        // 实现 s 型曲线的数学计算
        double sCurveProfile(double t, double jerk_limit) {
            // t is normalized time [0, 1]
            // Returns normalized position [0, 1] following S-curve
            // 简化版 S-Curve 配置文件
            // t 范围 [0, 1]，返回归一化位移
            if (t <= 0.0) return 0.0;
            if (t >= 1.0) return 1.0;
            
            // Simple S-curve: 3t^2 - 2t^3
            // 使用正弦曲线近似 S-Curve
            // return 0.5 - 0.5 * std::cos(M_PI * t); // std::cos 提供余弦函数
            return 3.0 * t * t - 2.0 * t * t * t;
        }

    public:
        TrajectorySmoother(double max_vel = 1.0, double max_acc = 0.5,
                           double vel_scale = 0.5, double acc_scale = 0.5)
            : max_velocity_(max_vel), max_acceleration_(max_acc),
              velocity_scaling_(vel_scale), acceleration_scaling_(acc_scale) {}

        // 对轨迹进行速度和加速度限制平滑处理
        robot_trajectory::RobotTrajectoryPtr smoothTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory){

            // 创建空轨迹容器
            robot_trajectory::RobotTrajectoryPtr smoothed_trajectory = 
                std::make_shared<robot_trajectory::RobotTrajectory>(
                    trajectory->getRobotModel(), // 获取原始轨迹的机器人模型
                    trajectory->getGroupName()   // 获取原始轨迹的关节组名称
                );
            
            const moveit::core::JointModelGroup* joint_group = trajectory->getGroup(); // 从轨迹对象中获取关节组模型指针
            const std::vector<const moveit::core::JointModel*>& joint_models = joint_group->getActiveJointModels(); // 从关节组对象中获取该组包含的所有活动关节指针集合
            size_t num_points = trajectory->getWayPointCount(); // 轨迹由多少个位置点组成

            // 路径点过少，不能组成路径
            if(num_points < 2){
                // 第一个参数 0：路点索引，第二个参数 0：该路点距离上一个路点的时间
                smoothed_trajectory->setWayPointDurationFromPrevious(0, 0); // 设置时间戳
                return smoothed_trajectory; 
            }

            // ==================== 速度限制处理（第一遍扫描）=====================
            /**
            * 计算满足速度约束的最短时间。              
            */
            // 限制最大速度并重新计算时间戳
            // 表示轨迹中相邻两个路点之间允许的时间间隔
            double min_time_step = 0.02;    // 最小时间步长 20ms
            double max_time_step = 0.1;     // 最大时间步长 100ms

            // 第一遍：限制各段的最大速度
            // num_points：向量的初始大小（元素个数），0.0：每个元素的初始值
            std::vector<double> time_from_start(num_points, 0.0);
            double current_time = 0.0; // 当前时间

            for (size_t i = 1 ; i < num_points ; ++i){
                double max_delta_time = 0.0;
                const moveit::core::RobotState& prev_state = trajectory->getWayPoint(i - 1);
                const moveit::core::RobotState& curr_state = trajectory->getWayPoint(i);

                for(size_t j = 0 ; j < joint_models.size() ; ++j){

                    // 计算当前路点和前一路点之间某个关节的位置差（绝对值）       std::abs : 取绝对值
                    double delta = std::abs(
                        /*
                        第 1 步: joint_models[j]        → 获取第 j 个关节的指针
                        第 2 步: curr_state.getJointPositions(...) → 调用函数，传入关节指针
                        第 3 步: getJointPositions() 返回一个数组
                        第 4 步: [0]                    → 取数组的第一个元素
                        */
                        curr_state.getJointPositions(joint_models[j])[0] - 
                        prev_state.getJointPositions(joint_models[j])[0]);

                        // 第 i 个路点时间段内允许通过的最大位移（单位时间内的最大位移）
                        // getWayPointDurationFromPrevious : 原始轨迹中第 i 个路点距离前一个路点的时间间隔
                    double max_allowed = max_velocity_ * velocity_scaling_ * trajectory->getWayPointDurationFromPrevious(i);
                    if(max_allowed > 0){
                        double delta_time = delta / max_allowed; // 需要多少单位时间
                        max_delta_time = std::max(max_delta_time, delta_time); // 最大所需时间
                    }
                }

                // 计算时间间隔
                double duration = std::max(min_time_step,
                    std::min(max_time_step, max_delta_time));
                current_time += duration ;
                time_from_start[i] = current_time;                       
            }

            // ======================== 加速度限制处理（第二遍扫描）=======================
            /**
            *采用离散化的加速度近似计算。调整时间使轨迹的每个关节都满足加速度约束
            */
            for(size_t i = 2 ; i < num_points ; ++i){
                // time_from_start[i-1] = 第 i-1 个路点相对于轨迹起点的累积时间
                double time_diff = time_from_start[i] - time_from_start[i-1]; // 相邻两个路点之间的时间间隔
                double time_diff_prev = time_from_start[i-1] - time_from_start[i-2];

                if(time_diff < min_time_step || time_diff_prev < min_time_step){
                    continue;
                }

                // 限制加速度的变化，最大速度变化量
                double max_vel_change = max_acceleration_ * acceleration_scaling_ * (time_diff + time_diff_prev) / 2.0;

                for(size_t j = 0 ; j < joint_models.size() ; ++j){
                    const moveit::core::RobotState& prev_state = trajectory->getWayPoint(i - 1);
                    const moveit::core::RobotState& curr_state = trajectory->getWayPoint(i);

                    double delta_curr = std::abs( 
                        curr_state.getJointPositions(joint_models[j])[0] - 
                        prev_state.getJointPositions(joint_models[j])[0]);
                    double delta_prev = std::abs(
                        prev_state.getJointPositions(joint_models[j])[0] - 
                        trajectory->getWayPoint(i-2).getJointPositions(joint_models[j])[0]);
                    
                    double vel_curr = delta_curr / time_diff;       // 当前段速度
                    double vel_prev = delta_prev / time_diff_prev;  // 前一段速度
                    double vel_change = std::abs(vel_curr - vel_prev); // 速度变化量

                    // 当前速度变化量大于最大速度变化量，则
                    if(vel_change > max_vel_change){
                        // 需要调整时间戳
                        double adjustment_factor = max_vel_change / (vel_change + 1e-6) * 0.5; // 时间调整因子，加上一个极小值（1e-6）避免除零。
                        time_from_start[i] = time_from_start[i-1] + time_diff * (1.0 + adjustment_factor);
                        time_diff = time_from_start[i] - time_from_start[i-1];
                    }
                }
            }                

            // =========================== 构建平滑轨迹（第三遍扫描）===========================
            for(size_t i = 0 ; i < num_points ; ++i){
                moveit::core::RobotState waypoint = trajectory->getWayPoint(i);

                // 计算速度
                std::vector<double> velocities;
                if(i > 0 && i < num_points - 1){
                    double dt = time_from_start[i+1] - time_from_start[i-1];
                    if (dt > 0){
                        for (size_t j = 0 ; j < joint_models.size() ; ++j){
                            double vel = (
                                trajectory->getWayPoint(i+1).getJointPositions(joint_models[j])[0] - 
                                trajectory->getWayPoint(i-1).getJointPositions(joint_models[j])[0]) / dt;
                            velocities.push_back(std::clamp(vel, 
                            -max_velocity_ * velocity_scaling_, 
                            max_velocity_ * velocity_scaling_));
                        }
                    }
                }

                if(i > 0){
                    double dt = time_from_start[i] - time_from_start[i-1];
                    // addSuffixWayPoint 在轨迹末尾追加一个路点
                    smoothed_trajectory->addSuffixWayPoint(waypoint, dt);
                } else {
                    smoothed_trajectory->addSuffixWayPoint(waypoint, 0.0);
                }
            }
            return smoothed_trajectory;
        }

        // ========================= S 型曲线平滑函数 ===========================
        // 使用 S 型速度曲线 （S-Curve）平滑轨迹
        robot_trajectory::RobotTrajectoryPtr applySCurveSmoothing(
            const robot_trajectory::RobotTrajectoryPtr& trajectory, 
            double jerk_limit = 10.0){ // jerk_limit 加速度的变化率

                robot_trajectory::RobotTrajectoryPtr smoothed_trajectory = 
                    std::make_shared<robot_trajectory::RobotTrajectory>(
                        trajectory->getRobotModel(), trajectory->getGroupName());
            
                size_t num_points = trajectory->getWayPointCount();
                if(num_points < 2){
                    return trajectory;
                }

                const moveit::core::JointModelGroup* joint_group = trajectory->getGroup();
                const std::vector<const moveit::core::JointModel*>& joint_models = joint_group->getActiveJointModels();

                // ================== S 型曲线时间计算器 ==========================
                // 计算总时间和各关节最大需求
                double total_time = trajectory->getWayPointDurationFromStart(num_points - 1);
                std::vector<double> joint_deltas;  // 关节总位移
                std::vector<double> joint_max_vels;
                std::vector<double> joint_max_accs;

                for (size_t j = 0 ; j < joint_models.size() ; ++j){
                    double delta = std::abs(
                        trajectory->getWayPoint(num_points-1).getJointPositions(joint_models[j])[0] - 
                        trajectory->getWayPoint(0).getJointPositions(joint_models[j])[0]);
                    joint_deltas.push_back(delta);
                    joint_max_vels.push_back(max_velocity_ * velocity_scaling_);
                    joint_max_accs.push_back(max_acceleration_ * acceleration_scaling_);
                }

                // 为每个关节计算 S-Curve 时间
                double s_curve_time = 0.0 ;
                for (size_t j = 0 ; j < joint_models.size() ; ++j){
                    if (joint_deltas[j] > 1e-6){
                        // S-Curve 加速时间
                        double t_accel = joint_max_vels[j] / (joint_max_accs[j] + 1e-6);
                        // S-Curve 减速时间
                        double t_decel = joint_max_vels[j] / (joint_max_accs[j] + 1e-6);
                        // 匀速时间段
                        double t_constant = (joint_deltas[j] - 0.5 * joint_max_vels[j] * (t_accel + t_decel)) /
                            (joint_max_vels[j] + 1e-6);
                        t_constant = std::max(0.0, t_constant);

                        double joint_time = t_accel + t_constant + t_decel;
                        s_curve_time = std::max(s_curve_time, joint_time);
                    }
                }

                // =================== 轨迹重采样 =========================
                // 按比例缩放轨迹时间
                double scale_factor = s_curve_time / total_time;
                scale_factor = std::max(0.5 , std::min(2.0, scale_factor)); // 限制缩放范围

                // 重采样轨迹点
                int resample_count = static_cast<int>(num_points * scale_factor);
                resample_count = std::max(50, std::min(resample_count, 200)); // 限制点数

                for(int i = 0 ; i < resample_count ; ++i){
                    double t = static_cast<double>(i) / (resample_count - 1);
                    moveit::core::RobotState interpolated_state(
                        trajectory->getRobotModel());

                    for(size_t j = 0 ; j < joint_models.size() ; ++j){
                        double start_pos = trajectory->getWayPoint(0).getJointPositions(joint_models[j])[0];
                        double end_pos = trajectory->getWayPoint(num_points-1).getJointPositions(joint_models[j])[0];

                        // 使用 S-Curve 插值 
                        double s = sCurveProfile(t, jerk_limit);
                        double interpolated_pos = start_pos + s * (end_pos - start_pos);
                        interpolated_state.setJointPositions(joint_models[j], &interpolated_pos);
                    }

                    double dt = (i == 0) ? 0.0 : 
                        (total_time * scale_factor) / (resample_count - 1);
                    smoothed_trajectory->addSuffixWayPoint(interpolated_state, dt);
                }

                return smoothed_trajectory;
            }
};





// /**
//  * @file trajectory_smoother_complete.hpp
//  * @brief 轨迹平滑器完整实现 - 解决机械臂运动惯性和稳定性问题
//  */

// #ifndef TRAJECTORY_SMOOTHER_COMPLETE_HPP
// #define TRAJECTORY_SMOOTHER_COMPLETE_HPP

// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>
// #include <moveit/robot_trajectory/robot_trajectory.h>
// #include <moveit_msgs/msg/robot_trajectory.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.h>
// #include <vector>
// #include <cmath>
// #include <algorithm>
// #include <memory>
// #include <iostream>

// /**
//  * @brief 轨迹平滑处理类
//  * 
//  * 提供多种轨迹平滑算法，解决机械臂运动惯性和不稳定问题。
//  * 
//  * 使用方法：
//  * 1. 创建 TrajectorySmoother 实例
//  * 2. 配置参数（最大速度、加速度、平滑类型）
//  * 3. 对规划好的轨迹进行平滑处理
//  * 4. 执行平滑后的轨迹
//  */
// class TrajectorySmoother {
// private:
//     // 动力学限制参数
//     double max_velocity_;         // 最大速度 (rad/s 或 m/s)
//     double max_acceleration_;     // 最大加速度 (rad/s² 或 m/s²)
//     double max_jerk_;             // 最大加加速度 (rad/s³ 或 m/s³)
    
//     // 缩放因子
//     double velocity_scaling_;     // 速度缩放因子
//     double acceleration_scaling_; // 加速度缩放因子
    
//     // 平滑参数
//     int resample_count_;          // 重采样点数
//     double smoothness_weight_;    // 平滑权重
//     bool use_jerk_limiting_;      // 是否使用加加速度限制
    
// public:
//     /**
//      * @brief 构造函数 - 默认参数
//      */
//     TrajectorySmoother()
//         : max_velocity_(1.0),
//           max_acceleration_(0.5),
//           max_jerk_(10.0),
//           velocity_scaling_(0.4),
//           acceleration_scaling_(0.3),
//           resample_count_(100),
//           smoothness_weight_(0.5),
//           use_jerk_limiting_(true) {}
    
//     /**
//      * @brief 构造函数 - 带参数
//      * @param max_vel 最大速度
//      * @param max_acc 最大加速度
//      * @param vel_scale 速度缩放因子
//      * @param acc_scale 加速度缩放因子
//      */
//     TrajectorySmoother(double max_vel, double max_acc,
//                        double vel_scale = 0.4, double acc_scale = 0.3)
//         : max_velocity_(max_vel),
//           max_acceleration_(max_acc),
//           max_jerk_(10.0),
//           velocity_scaling_(vel_scale),
//           acceleration_scaling_(acc_scale),
//           resample_count_(100),
//           smoothness_weight_(0.5),
//           use_jerk_limiting_(true) {}
    
//     /**
//      * @brief 配置动力学参数
//      */
//     void configure(double max_vel, double max_acc, double max_j = 10.0) {
//         max_velocity_ = max_vel;
//         max_acceleration_ = max_acc;
//         max_jerk_ = max_j;
//     }
    
//     /**
//      * @brief 配置缩放因子
//      */
//     void configureScaling(double vel_scale, double acc_scale) {
//         velocity_scaling_ = std::clamp(vel_scale, 0.05, 1.0);
//         acceleration_scaling_ = std::clamp(acc_scale, 0.05, 1.0);
//     }
    
//     /**
//      * @brief 平滑轨迹 - 基础版
//      * 
//      * 对轨迹应用速度和加速度限制，返回平滑后的轨迹消息
//      * 
//      * @param input_trajectory 输入轨迹
//      * @return 平滑后的轨迹消息
//      */
//     moveit_msgs::msg::RobotTrajectory smooth(
//         const moveit_msgs::msg::RobotTrajectory& input_trajectory) {
        
//         moveit_msgs::msg::RobotTrajectory output = input_trajectory;
        
//         if (input_trajectory.joint_trajectory.points.empty()) {
//             return output;
//         }
        
//         // 应用速度限制
//         applyVelocityLimiting(output);
        
//         // 应用加速度限制
//         applyAccelerationLimiting(output);
        
//         return output;
//     }
    
//     /**
//      * @brief 平滑轨迹 - S-Curve 版（推荐）
//      * 
//      * 使用 S 型速度曲线进行平滑，效果最好
//      * 
//      * @param input_trajectory 输入轨迹
//      * @return 平滑后的轨迹消息
//      */
//     moveit_msgs::msg::RobotTrajectory smoothSCurve(
//         const moveit_msgs::msg::RobotTrajectory& input_trajectory) {
        
//         moveit_msgs::msg::RobotTrajectory output = input_trajectory;
        
//         if (input_trajectory.joint_trajectory.points.empty()) {
//             return output;
//         }
        
//         // 应用 S-Curve 速度曲线
//         applySCurveProfile(output);
        
//         return output;
//     }
    
//     /**
//      * @brief 平滑轨迹 - 高级版
//      * 
//      * 综合应用多种平滑技术
//      * 
//      * @param input_trajectory 输入轨迹
//      * @return 平滑后的轨迹消息
//      */
//     moveit_msgs::msg::RobotTrajectory smoothAdvanced(
//         const moveit_msgs::msg::RobotTrajectory& input_trajectory) {
        
//         moveit_msgs::msg::RobotTrajectory output = input_trajectory;
        
//         if (input_trajectory.joint_trajectory.points.empty()) {
//             return output;
//         }
        
//         // 1. 应用加加速度限制（如果启用）
//         if (use_jerk_limiting_) {
//             applyJerkLimiting(output);
//         }
        
//         // 2. 应用 S-Curve 速度曲线
//         applySCurveProfile(output);
        
//         // 3. 应用速度和加速度限制
//         applyVelocityLimiting(output);
//         applyAccelerationLimiting(output);
        
//         return output;
//     }
    
//     // 枚举类型：平滑方法
//     enum SmoothingType {
//         SMOOTH_BASIC,      // 基础平滑
//         SMOOTH_SCURVE,     // S-Curve 平滑（推荐）
//         SMOOTH_ADVANCED    // 高级平滑
//     };

//     /**
//      * @brief 对 RobotTrajectory 对象进行平滑
//      */
//     moveit::planning_interface::MoveGroupInterface::Plan smoothPlan(
//         moveit::planning_interface::MoveGroupInterface::Plan& plan,
//         SmoothingType type = SMOOTH_SCURVE) {
        
//         moveit_msgs::msg::RobotTrajectory smoothed;
        
//         switch (type) {
//             case SMOOTH_BASIC:
//                 smoothed = smooth(plan.trajectory);
//                 break;
//             case SMOOTH_SCURVE:
//                 smoothed = smoothSCurve(plan.trajectory);
//                 break;
//             case SMOOTH_ADVANCED:
//                 smoothed = smoothAdvanced(plan.trajectory);
//                 break;
//             default:
//                 // Unknown smoothing type: leave the trajectory unchanged
//                 smoothed = plan.trajectory;
//                 break;
//         }
        
//         // replace plan trajectory with the smoothed result and return the modified plan
//         plan.trajectory = smoothed;
//         return plan;
//     }
    
// private:
//     /**
//      * @brief 应用速度限制
//      */
//     void applyVelocityLimiting(moveit_msgs::msg::RobotTrajectory& trajectory) {
//         double max_vel = max_velocity_ * velocity_scaling_;
        
//         for (auto& point : trajectory.joint_trajectory.points) {
//             for (auto& vel : point.velocities) {
//                 if (std::abs(vel) > max_vel) {
//                     vel = (vel > 0) ? max_vel : -max_vel;
//                 }
//             }
//         }
//     }
    
//     /**
//      * @brief 应用加速度限制
//      */
//     void applyAccelerationLimiting(moveit_msgs::msg::RobotTrajectory& trajectory) {
//         double max_acc = max_acceleration_ * acceleration_scaling_;
        
//         for (size_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i) {
//             auto& curr = trajectory.joint_trajectory.points[i];
//             auto& prev = trajectory.joint_trajectory.points[i - 1];
            
//             double dt = getTimeDifference(curr.time_from_start, prev.time_from_start);
//             if (dt < 1e-6) continue;
            
//             for (size_t j = 0; j < curr.accelerations.size() && j < prev.accelerations.size(); ++j) {
//                 double acc = (curr.velocities[j] - prev.velocities[j]) / dt;
//                 if (std::abs(acc) > max_acc) {
//                     curr.velocities[j] = prev.velocities[j] + 
//                         (acc > 0 ? max_acc : -max_acc) * dt;
//                 }
//             }
//         }
//     }
    
//     /**
//      * @brief 应用加加速度限制
//      */
//     void applyJerkLimiting(moveit_msgs::msg::RobotTrajectory& trajectory) {
//         double max_j = max_jerk_;
        
//         for (size_t i = 2; i < trajectory.joint_trajectory.points.size(); ++i) {
//             auto& curr = trajectory.joint_trajectory.points[i];
//             auto& prev = trajectory.joint_trajectory.points[i - 1];
//             auto& prev2 = trajectory.joint_trajectory.points[i - 2];
            
//             double dt1 = getTimeDifference(prev.time_from_start, prev2.time_from_start);
//             double dt2 = getTimeDifference(curr.time_from_start, prev.time_from_start);
            
//             if (dt1 < 1e-6 || dt2 < 1e-6) continue;
            
//             for (size_t j = 0; j < curr.accelerations.size() && j < prev.accelerations.size(); ++j) {
//                 double acc_curr = (curr.velocities[j] - prev.velocities[j]) / dt2;
//                 acc_curr = (acc_curr + prev.accelerations[j]) * 0.5;  // 平均加速度
                
//                 double jerk = (acc_curr - prev.accelerations[j]) / dt1;
//                 if (std::abs(jerk) > max_j) {
//                     // 限制加加速度
//                     double correction = (jerk > 0 ? -1 : 1) * max_j * dt1;
//                     curr.velocities[j] = prev.velocities[j] + 
//                         (prev.accelerations[j] + correction * 0.5) * dt2;
//                 }
//             }
//         }
//     }
    
//     /**
//      * @brief 应用 S-Curve 速度曲线
//      */
//     void applySCurveProfile(moveit_msgs::msg::RobotTrajectory& trajectory) {
//         if (trajectory.joint_trajectory.points.size() < 2) return;
        
//         // 计算总时间
//         const auto& last_point = trajectory.joint_trajectory.points.back();
//         double total_time = last_point.time_from_start.sec + 
//                           last_point.time_from_start.nanosec * 1e-9;
        
//         if (total_time < 1e-6) return;
        
//         // 应用 S-Curve 配置文件
//         for (auto& point : trajectory.joint_trajectory.points) {
//             double t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
//             double normalized_t = t / total_time;  // 归一化到 [0, 1]
            
//             // S-Curve 配置文件
//             double s = sCurveProfile(normalized_t);
            
//             // 缩放时间戳
//             double new_t = s * total_time;
//             point.time_from_start.sec = static_cast<int32_t>(std::floor(new_t));
//             point.time_from_start.nanosec = static_cast<uint32_t>(
//                 (new_t - std::floor(new_t)) * 1e9);
            
//             // 缩放速度
//             if (!point.velocities.empty()) {
//                 // S-Curve 的一阶导数
//                 double ds_dt = sCurveDerivative(normalized_t);
//                 for (auto& vel : point.velocities) {
//                     vel = vel * ds_dt;
//                 }
//             }
            
//             // 缩放加速度
//             if (!point.accelerations.empty()) {
//                 // S-Curve 的二阶导数
//                 double d2s_dt2 = sCurveSecondDerivative(normalized_t);
//                 for (auto& acc : point.accelerations) {
//                     acc = acc * d2s_dt2;
//                 }
//             }
//         }
//     }
    
//     /**
//      * @brief S-Curve 配置文件 (余弦曲线)
//      * @param t 归一化时间 [0, 1]
//      * @return 归一化位移 [0, 1]
//      */
//     double sCurveProfile(double t) {
//         if (t <= 0) return 0.0;
//         if (t >= 1) return 1.0;
//         // 使用余弦曲线实现平滑加速和减速
//         return 0.5 - 0.5 * std::cos(M_PI * t);
//     }
    
//     /**
//      * @brief S-Curve 一阶导数
//      */
//     double sCurveDerivative(double t) {
//         if (t <= 0 || t >= 1) return 0.0;
//         return 0.5 * M_PI * std::sin(M_PI * t);
//     }
    
//     /**
//      * @brief S-Curve 二阶导数
//      */
//     double sCurveSecondDerivative(double t) {
//         if (t <= 0 || t >= 1) return 0.0;
//         return 0.5 * M_PI * M_PI * std::cos(M_PI * t);
//     }
    
//     /**
//      * @brief 获取时间差（秒）
//      */
//     double getTimeDifference(
//         const builtin_interfaces::msg::Duration& t1,
//         const builtin_interfaces::msg::Duration& t2) {
//         return (t1.sec - t2.sec) + (t1.nanosec - t2.nanosec) * 1e-9;
//     }
// };

// /**
//  * @brief 轨迹处理工具类
//  */
// class TrajectoryUtils {
// public:
//     /**
//      * @brief 打印轨迹信息（用于调试）
//      */
//     static void printTrajectoryInfo(
//         const moveit_msgs::msg::RobotTrajectory& trajectory,
//         const std::string& name = "Trajectory") {
        
//         std::cout << "=== " << name << " 信息 ===" << std::endl;
//         std::cout << "轨迹点数: " << trajectory.joint_trajectory.points.size() << std::endl;
//         std::cout << "关节数: " << trajectory.joint_trajectory.joint_names.size() << std::endl;
        
//         if (!trajectory.joint_trajectory.points.empty()) {
//             const auto& last = trajectory.joint_trajectory.points.back();
//             double duration = last.time_from_start.sec + last.time_from_start.nanosec * 1e-9;
//             std::cout << "总时长: " << duration << " 秒" << std::endl;
            
//             if (!last.velocities.empty()) {
//                 double max_vel = 0.0;
//                 for (const auto& v : last.velocities) {
//                     max_vel = std::max(max_vel, std::abs(v));
//                 }
//                 std::cout << "末端速度: " << max_vel << " rad/s" << std::endl;
//             }
//         }
//         std::cout << std::endl;
//     }
    
//     /**
//      * @brief 创建时间拉伸器
//      */
//     static void stretchTrajectory(
//         moveit_msgs::msg::RobotTrajectory& trajectory,
//         double factor) {
        
//         for (auto& point : trajectory.joint_trajectory.points) {
//             double t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
//             t *= factor;
//             point.time_from_start.sec = static_cast<int32_t>(std::floor(t));
//             point.time_from_start.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
            
//             if (!point.velocities.empty()) {
//                 for (auto& v : point.velocities) {
//                     v = v / factor;
//                 }
//             }
            
//             if (!point.accelerations.empty()) {
//                 for (auto& a : point.accelerations) {
//                     a = a / (factor * factor);
//                 }
//             }
//         }
//     }
// };

// #endif // TRAJECTORY_SMOOTHER_COMPLETE_HPP
