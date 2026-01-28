/*
 对于长距离运动和复杂路径，采用分段速度控制策略
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

class SegmentedVelocityController {
public:
    enum MotionPhase {
        ACCELERATION,
        CRUISE,
        DECELERATION,
        STOPPED
    };
    
private:
    MotionPhase current_phase_;
    double current_velocity_;
    double max_velocity_;
    double max_acceleration_;
    double target_position_;
    double current_position_;
    double acceleration_distance_;
    double deceleration_distance_;
    
public:
    SegmentedVelocityController(double max_vel = 1.0, double max_acc = 0.5)
        : current_phase_(STOPPED),
          current_velocity_(0.0),
          max_velocity_(max_vel),
          max_acceleration_(max_acc),
          target_position_(0.0),
          current_position_(0.0) {}
    
    void setMotionParameters(double max_vel, double max_acc) {
        max_velocity_ = max_vel;
        max_acceleration_ = max_acc;
    }
    
    void setTarget(double current_pos, double target_pos) {
        current_position_ = current_pos;
        target_position_ = target_pos;
        current_phase_ = ACCELERATION;
        current_velocity_ = 0.0;
    }
    
    // 根据当前位置和剩余距离计算期望速度
    double update(double delta_time) {
        double remaining_distance = std::abs(target_position_ - current_position_);
        
        // 计算减速距离（从最大速度减速到 0）
        deceleration_distance_ = (max_velocity_ * max_velocity_) / 
            (2.0 * max_acceleration_ + 1e-6);
        
        // 计算加速距离（从 0 加速到最大速度）
        acceleration_distance_ = (max_velocity_ * max_velocity_) / 
            (2.0 * max_acceleration_ + 1e-6);
        
        double total_distance = acceleration_distance_ + deceleration_distance_;
        
        if (remaining_distance > total_distance) {
            // 有足够的距离进行完整的加速-匀速-减速过程
            if (current_phase_ == STOPPED || current_phase_ == DECELERATION) {
                current_phase_ = ACCELERATION;
            }
        } else if (remaining_distance > deceleration_distance_) {
            // 进入匀速阶段
            current_phase_ = CRUISE;
        } else {
            // 进入减速阶段
            current_phase_ = DECELERATION;
        }
        
        // 根据阶段更新速度
        switch (current_phase_) {
            case ACCELERATION:
                current_velocity_ = std::min(max_velocity_,
                    current_velocity_ + max_acceleration_ * delta_time);
                break;
                
            case CRUISE:
                // 保持匀速
                break;
                
            case DECELERATION:
                {
                    double decel_rate = (current_velocity_ * current_velocity_) / 
                        (2.0 * remaining_distance + 1e-6);
                    decel_rate = std::max(max_acceleration_, decel_rate);
                    current_velocity_ = std::max(0.0,
                        current_velocity_ - decel_rate * delta_time);
                    
                    if (current_velocity_ < 1e-4) {
                        current_velocity_ = 0.0;
                        current_phase_ = STOPPED;
                    }
                }
                break;
                
            case STOPPED:
                current_velocity_ = 0.0;
                break;
        }
        
        return current_velocity_;
    }
    
    MotionPhase getCurrentPhase() const {
        return current_phase_;
    }
    
    bool isMoving() const {
        return current_phase_ != STOPPED;
    }
};
