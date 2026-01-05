#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "geometry_msgs/msg/pose.hpp"
#include <functional>

class ArmController : public rclcpp::Node
{
public:
    ArmController() : Node("arm_controller")
    {
        RCLCPP_INFO(this->get_logger(), "节点启动");
        
        // 使用 lambda 捕获 shared_from_this() 的延迟调用
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                timer_->cancel();
                initializeMoveGroup();
                planAndExecute();
            }
        );
    }
    
    void initializeMoveGroup()
    {
        // 延迟获取 shared_ptr，确保有效
        auto node_ptr = shared_from_this();
        
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_ptr,
            "robotic_arm"
        );
        
        move_group_->setPlanningTime(2.0);
        move_group_->setNumPlanningAttempts(10);
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.5);
        
        RCLCPP_INFO(this->get_logger(), "MoveGroupInterface 初始化成功");
    }
    
    void planAndExecute()
    {
        // 设置目标位姿
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = 0.34;
        target_pose.position.y = -0.205129;
        target_pose.position.z = 0.317940;
        target_pose.orientation.x = 0.0;
        target_pose.orientation.y = 0.0;
        target_pose.orientation.z = 0.707;
        target_pose.orientation.w = 0.707;
        
        move_group_->setPoseTarget(target_pose);
        
        // 规划并执行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "规划成功");
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "规划失败");
        }
        
        rclcpp::shutdown();
    }
    
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
