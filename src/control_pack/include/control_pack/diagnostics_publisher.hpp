#pragma once

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/publisher.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace mixcontroller {

/**
 * @brief 诊断信息发布器，用于实时监控和可视化
 * 
 * 发布以下信息：
 * 1. Marker 话题 — 末端执行器目标位置（Rviz 可视化）
 * 2. 诊断话题 — 速度、加速度、力矩等细节数据
 * 3. 状态话题 — 队列大小、安全状态等
 */
class DiagnosticsPublisher {
public:
    /**
     * @brief 构造函数
     * @param node ROS 2 节点指针（LifecycleNode）
     */
    explicit DiagnosticsPublisher(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

    /**
     * @brief 发布末端执行器目标位置 Marker（用于 Rviz 可视化）
     * 
     * @param position 末端执行器位置 (x, y, z)
     * @param frame_id 参考坐标系
     * @param color 颜色 (R, G, B, A)
     * @param marker_id Marker 的唯一 ID
     */
    void publish_target_pose_marker(
        const std::vector<double>& position,
        const std::string& frame_id = "base_link",
        const std::vector<float>& color = {0.0f, 1.0f, 0.0f, 1.0f},
        int marker_id = 0
    );

    /**
     * @brief 发布真实的末端执行器位置 Marker
     */
    void publish_actual_pose_marker(
        const std::vector<double>& position,
        const std::string& frame_id = "base_link",
        const std::vector<float>& color = {1.0f, 0.0f, 0.0f, 1.0f},
        int marker_id = 1
    );

    /**
     * @brief 发布关节状态诊断信息
     */
    struct JointDiagnostics {
        std::vector<double> desired_positions;
        std::vector<double> desired_velocities;
        std::vector<double> desired_accelerations;
        std::vector<double> actual_positions;
        std::vector<double> actual_velocities;
        std::vector<double> predicted_torques;
    };

    void publish_joint_diagnostics(const JointDiagnostics& diagnostics);

    /**
     * @brief 发布系统状态诊断信息
     */
    struct SystemDiagnostics {
        int queue_size;
        bool twist_enabled;
        double tracking_error;
        bool safety_triggered;
        double ik_solve_time_ms;
        int update_rate_hz;
    };

    void publish_system_diagnostics(const SystemDiagnostics& diagnostics);

private:
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    
    // 创建 Marker 消息的辅助函数
    visualization_msgs::msg::Marker create_sphere_marker(
        const geometry_msgs::msg::Point& position,
        const std::vector<float>& color,
        int marker_id,
        const std::string& frame_id,
        double scale = 0.02
    );
};

}  // namespace mixcontroller
