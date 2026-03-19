#include "control_pack/diagnostics_publisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace mixcontroller {

DiagnosticsPublisher::DiagnosticsPublisher(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
    : node_(node) {
    // 创建 Marker 发布器用于 Rviz 可视化
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "arm_controller/target_pose_marker", 10);
    
    RCLCPP_INFO(node_->get_logger(), "DiagnosticsPublisher 初始化完成");
}

void DiagnosticsPublisher::publish_target_pose_marker(
    const std::vector<double>& position,
    const std::string& frame_id,
    const std::vector<float>& color,
    int marker_id) {
    
    if (position.size() < 3) {
        return;
    }

    geometry_msgs::msg::Point point;
    point.x = position[0];
    point.y = position[1];
    point.z = position[2];

    auto marker = create_sphere_marker(point, color, marker_id, frame_id, 0.02);
    marker.text = "Target";
    
    marker_pub_->publish(marker);
}

void DiagnosticsPublisher::publish_actual_pose_marker(
    const std::vector<double>& position,
    const std::string& frame_id,
    const std::vector<float>& color,
    int marker_id) {
    
    if (position.size() < 3) {
        return;
    }

    geometry_msgs::msg::Point point;
    point.x = position[0];
    point.y = position[1];
    point.z = position[2];

    auto marker = create_sphere_marker(point, color, marker_id, frame_id, 0.015);
    marker.text = "Actual";
    
    marker_pub_->publish(marker);
}

visualization_msgs::msg::Marker DiagnosticsPublisher::create_sphere_marker(
    const geometry_msgs::msg::Point& position,
    const std::vector<float>& color,
    int marker_id,
    const std::string& frame_id,
    double scale) {
    
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = node_->get_clock()->now();
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position = position;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    
    marker.color.r = color.size() > 0 ? color[0] : 0.0f;
    marker.color.g = color.size() > 1 ? color[1] : 1.0f;
    marker.color.b = color.size() > 2 ? color[2] : 0.0f;
    marker.color.a = color.size() > 3 ? color[3] : 1.0f;
    
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 100000000;  // 100ms
    
    return marker;
}

void DiagnosticsPublisher::publish_joint_diagnostics(const JointDiagnostics& diagnostics) {
    // 这里可以发布到自定义消息或使用标准消息
    // 目前作为占位符，实际可以添加到专门的诊断话题
    RCLCPP_DEBUG(node_->get_logger(), 
        "关节诊断: 期望位置[0]=%.3f, 实际位置[0]=%.3f",
        diagnostics.desired_positions.empty() ? 0.0 : diagnostics.desired_positions[0],
        diagnostics.actual_positions.empty() ? 0.0 : diagnostics.actual_positions[0]);
}

void DiagnosticsPublisher::publish_system_diagnostics(const SystemDiagnostics& diagnostics) {
    RCLCPP_DEBUG(node_->get_logger(),
        "系统诊断: 队列=%d, Twist=%s, 追踪误差=%.3f, 安全触发=%s, IK耗时=%.2fms, 更新率=%dHz",
        diagnostics.queue_size,
        diagnostics.twist_enabled ? "Y" : "N",
        diagnostics.tracking_error,
        diagnostics.safety_triggered ? "Y" : "N",
        diagnostics.ik_solve_time_ms,
        diagnostics.update_rate_hz);
}

}  // namespace mixcontroller
