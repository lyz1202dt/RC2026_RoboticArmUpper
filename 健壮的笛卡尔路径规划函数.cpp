#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <algorithm>

class RobustCartesianPlanner {
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Node::SharedPtr node_;
    
public:
    RobustCartesianPlanner(rclcpp::Node::SharedPtr node, const std::string& group_name) 
        : node_(node) {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_name);
        
        // 设置基本配置
        move_group_->setPlanningTime(30.0);  // 增加规划时间
        move_group_->setNumPlanningAttempts(10);  // 增加规划尝试次数
        move_group_->setMaxVelocityScalingFactor(0.5);  // 速度限制
        move_group_->setMaxAccelerationScalingFactor(0.3);  // 加速度限制
    }
    
    // 改进的笛卡尔路径规划函数
    bool planCartesianPathImproved(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        moveit_msgs::msg::RobotTrajectory& trajectory,
        double& achieved_fraction,
        int max_retries = 5
    ) {
        // 定义参数搜索空间
        std::vector<double> eef_step_attempts = {0.02, 0.01, 0.005, 0.002};
        std::vector<double> jump_threshold_attempts = {0.0, 0.05, 0.1, 0.2};
        std::vector<bool> collision_avoidance = {true, false};
        
        for (int retry = 0; retry < max_retries; ++retry) {
            // 自动选择参数
            int eef_idx = std::min(retry / 2, (int)eef_step_attempts.size() - 1);
            int jump_idx = std::min(retry / 2, (int)jump_threshold_attempts.size() - 1);
            int collision_idx = retry % 2;
            
            double eef_step = eef_step_attempts[eef_idx];
            double jump_threshold = jump_threshold_attempts[jump_idx];
            bool avoid_collisions = collision_avoidance[collision_idx];
            
            RCLCPP_INFO(node_->get_logger(), 
                "尝试 %d: eef_step=%.4f, jump_threshold=%.4f, avoid_collisions=%s",
                retry + 1, eef_step, jump_threshold, 
                avoid_collisions ? "true" : "false");
            
            // 执行规划
            achieved_fraction = move_group_->computeCartesianPath(
                waypoints, eef_step, jump_threshold, trajectory,
                moveit_msgs::msg::Constraints(),  // 无额外约束
                avoid_collisions
            );
            
            RCLCPP_INFO(node_->get_logger(), "规划完成度: %.2f%%", achieved_fraction * 100.0);
            
            // 判断是否成功（90%以上即认为成功）
            if (achieved_fraction >= 0.9) {
                RCLCPP_INFO(node_->get_logger(), "规划成功！");
                return true;
            }
            
            // 检查是否完全失败
            if (achieved_fraction < 0.1 && retry > 2) {
                RCLCPP_WARN(node_->get_logger(), "规划成功率极低，检查路径点是否可达");
                // 可以在这里添加额外的诊断代码
            }
        }
        
        RCLCPP_ERROR(node_->get_logger(), "所有尝试均未达到满意的规划结果");
        return false;
    }
    
    // 路径点验证函数
    bool validateWaypoints(const std::vector<geometry_msgs::msg::Pose>& waypoints) {
        for (size_t i = 0; i < waypoints.size(); ++i) {
            const auto& pose = waypoints[i];
            
            // 检查位置范围
            double distance = std::sqrt(
                pose.position.x * pose.position.x + 
                pose.position.y * pose.position.y + 
                pose.position.z * pose.position.z
            );
            
            // 假设工作空间半径为1米
            if (distance > 1.0) {
                RCLCPP_WARN(node_->get_logger(), 
                    "路径点 %zu 超出工作空间: 距离原点 %.3f 米", i, distance);
                return false;
            }
            
            // 检查是否有NaN
            if (std::isnan(pose.position.x) || std::isnan(pose.position.y) || 
                std::isnan(pose.position.z)) {
                RCLCPP_ERROR(node_->get_logger(), "路径点 %zu 包含NaN值", i);
                return false;
            }
        }
        return true;
    }
    
    // 分解长路径为多个短路径
    bool planLongPathSegmented(
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Pose& end_pose,
        moveit_msgs::msg::RobotTrajectory& full_trajectory,
        double segment_length = 0.05  // 每段5厘米
    ) {
        // 计算总距离和方向
        geometry_msgs::msg::Vector3 direction;
        direction.x = end_pose.position.x - start_pose.position.x;
        direction.y = end_pose.position.y - start_pose.position.y;
        direction.z = end_pose.position.z - start_pose.position.z;
        
        double total_distance = std::sqrt(
            direction.x * direction.x + 
            direction.y * direction.y + 
            direction.z * direction.z
        );
        
        if (total_distance < 0.001) {
            RCLCPP_INFO(node_->get_logger(), "起点和终点距离太近，无需规划");
            return true;
        }
        
        // 归一化方向
        direction.x /= total_distance;
        direction.y /= total_distance;
        direction.z /= total_distance;
        
        // 计算分段数
        int num_segments = static_cast<int>(std::ceil(total_distance / segment_length));
        
        std::vector<geometry_msgs::msg::Pose> segment_waypoints;
        segment_waypoints.push_back(start_pose);
        
        // 生成分段路径点
        for (int i = 1; i < num_segments; ++i) {
            double distance = i * segment_length;
            geometry_msgs::msg::Pose pose;
            pose.position.x = start_pose.position.x + direction.x * distance;
            pose.position.y = start_pose.position.y + direction.y * distance;
            pose.position.z = start_pose.position.z + direction.z * distance;
            pose.orientation = start_pose.orientation;  // 保持姿态不变
            segment_waypoints.push_back(pose);
        }
        segment_waypoints.push_back(end_pose);
        
        // 规划并合并轨迹
        moveit_msgs::msg::RobotTrajectory segment_trajectory;
        double fraction;
        
        if (!planCartesianPathImproved(segment_waypoints, segment_trajectory, fraction)) {
            RCLCPP_ERROR(node_->get_logger(), "分段规划失败");
            return false;
        }
        
        // 合并轨迹（简化处理，实际可能需要更复杂的合并逻辑）
        full_trajectory = segment_trajectory;
        return true;
    }
};

// 使用示例
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cartesian_planner_node");
    
    RobustCartesianPlanner planner(node, "manipulator");
    
    // 创建路径点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    
    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = 0.4;
    start_pose.position.y = 0.0;
    start_pose.position.z = 0.4;
    start_pose.orientation.w = 1.0;
    waypoints.push_back(start_pose);
    
    geometry_msgs::msg::Pose end_pose = start_pose;
    end_pose.position.x = 0.6;  // 向X方向移动20厘米
    waypoints.push_back(end_pose);
    
    // 验证路径点
    if (!planner.validateWaypoints(waypoints)) {
        RCLCPP_ERROR(node->get_logger(), "路径点验证失败");
        return 1;
    }
    
    // 执行规划
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction;
    
    if (planner.planCartesianPathImproved(waypoints, trajectory, fraction)) {
        RCLCPP_INFO(node->get_logger(), "规划成功，完成度: %.2f%%", fraction * 100.0);
        
        // 可以在这里执行轨迹
        // planner.getMoveGroup()->execute(trajectory);
    } else {
        RCLCPP_ERROR(node->get_logger(), "规划失败");
    }
    
    rclcpp::shutdown();
    return 0;
}
