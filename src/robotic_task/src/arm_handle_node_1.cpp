
#include "arm_handle_node.hpp"
// #include "GraspOrientionController.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cassert>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/detail/constraints__struct.hpp>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
// #include <qt5/QtGui/qvalidator.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>

using namespace std::chrono_literals;




// bool ArmHandleNode::send_plan(const moveit_msgs::msg::RobotTrajectory& trajectory) {
//     robot_interfaces::msg::Arm arm_msg;
//     auto start_time = std::chrono::high_resolution_clock::time_point::clock::now();
//     for (auto const& point : trajectory.joint_trajectory.points) {
//         double time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
//         std::this_thread::sleep_until(start_time + std::chrono::duration<double>(time_from_start));

//         if (!rclcpp::ok())
//             return false;

//         if (point.positions.size() != 6) {
//             RCLCPP_WARN(node->get_logger(), "不符合的关节数");
//             continue;
//         }

//         for (int i = 0; i < 6; i++) {
//             arm_msg.joints[i].rad   = (float)point.positions[i];
//             arm_msg.joints[i].omega = (float)point.velocities[i];
//         }
//         RCLCPP_INFO(node->get_logger(), "发布关节角期望");
//         arm_target_publisher->publish(arm_msg);

//         if (cancle_current_task)                            // 如果要求退出当前的动作执行
//         {
//             for (int i = 0; i < 6; i++) {
//                 arm_msg.joints[i].omega = 0.0f;
//             }
//             arm_target_publisher->publish(arm_msg);
//             return false;
//         }
//     }
//     return true;
// }

geometry_msgs::msg::Pose ArmHandleNode::calculate_prepare_pos(
    const geometry_msgs::msg::Pose& box_pos, 
    double approach_distance, 
    geometry_msgs::msg::Pose &grasp_pose
    ) 
{
    constexpr double kBoxSize = 0.35;
    constexpr double kHalfSize = kBoxSize * 0.5;
    constexpr double kPrepareDistance = 0.05;  // 表面外 5cm
    constexpr double kGraspInsideDistance = 0.035;     // 表面内 3.5cm

    (void)approach_distance;

    const Eigen::Vector3d center(
        box_pos.position.x,
        box_pos.position.y,
        box_pos.position.z
    );

    // 机器人位于原点，取“机器人->物块中心”的XY方向。
    Eigen::Vector2d robot_to_box_xy(center.x(), center.y());
    if (robot_to_box_xy.norm() < 1e-9) {
        robot_to_box_xy = Eigen::Vector2d(1.0, 0.0);
    } else {
        robot_to_box_xy.normalize();
    }

    const Eigen::Vector3d inward_dir(robot_to_box_xy.x(), robot_to_box_xy.y(), 0.0);

    // 靠近机器人的侧面中心点。
    const Eigen::Vector3d side_center = center - inward_dir * kHalfSize;

    // 预抓取：表面外；抓取：表面内。
    const Eigen::Vector3d prepare_pos_vec = side_center - inward_dir * kPrepareDistance;
    const Eigen::Vector3d grasp_pos_vec = side_center + inward_dir * kGraspInsideDistance;

    // 固定侧抓姿态：末端朝向水平并垂直于物块侧面。
    tf2::Quaternion q;
    q.setRPY(0.0, -M_PI_2, 0.0);
    q.normalize();

    geometry_msgs::msg::Pose prepare_pose;
    prepare_pose.position.x = prepare_pos_vec.x();
    prepare_pose.position.y = prepare_pos_vec.y();
    prepare_pose.position.z = prepare_pos_vec.z();
    prepare_pose.orientation.w = q.w();
    prepare_pose.orientation.x = q.x();
    prepare_pose.orientation.y = q.y();
    prepare_pose.orientation.z = q.z();

    grasp_pose.position.x = grasp_pos_vec.x();
    grasp_pose.position.y = grasp_pos_vec.y();
    grasp_pose.position.z = grasp_pos_vec.z();
    grasp_pose.orientation.w = 0.7372;
    grasp_pose.orientation.x = 0.0;
    grasp_pose.orientation.y = -0.6759;
    grasp_pose.orientation.z = 0.0;

    // RCLCPP_INFO_THROTTLE(
    //     node->get_logger(),
    //     *node->get_clock(),
    //     1000,
    //     "calculate_prepare_pos: box_center=(%.3f, %.3f, %.3f), side_center=(%.3f, %.3f, %.3f)",
    //     center.x(), center.y(), center.z(),
    //     side_center.x(), side_center.y(), side_center.z()
    // );
    RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        1000,
        "calculate_prepare_pos: grasp=(%.3f, %.3f, %.3f), quat=(w:%.4f,x:%.4f,y:%.4f,z:%.4f)",
        grasp_pos_vec.x(), grasp_pos_vec.y(), grasp_pos_vec.z(),
        grasp_pose.orientation.w,
        grasp_pose.orientation.x,
        grasp_pose.orientation.y,
        grasp_pose.orientation.z
    );

    RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        1000,
        "calculate_prepare_pos: 严格按plan固定预抓取距离5cm、抓取进入3.5cm"
    );

    return prepare_pose;
}
































// ========== 备选方案：根据物体实际朝向调整抓取方向 ==========
geometry_msgs::msg::Pose ArmHandleNode::calculate_prepare_pos_with_orientation(
    const geometry_msgs::msg::Pose& box_pos, 
    double approach_distance, 
    geometry_msgs::msg::Pose &grasp_pose)
{
    // 如果需要保持物体原来的某些朝向特征，可以使用这个版本
    Eigen::Vector3d object_center(box_pos.position.x, box_pos.position.y, box_pos.position.z);
    Eigen::Quaterniond q(box_pos.orientation.w, box_pos.orientation.x, 
                         box_pos.orientation.y, box_pos.orientation.z);
    
    // 需求：沿x轴方向接近，吸盘朝+x方向
    // 使用物体的位置来确定x轴接近方向
    Eigen::Vector3d approach_direction(1.0, 0.0, 0.0);
    
    // 抓取位置：物体中心减去物体x方向尺寸
    const double object_half_size_x = 0.35;
    Eigen::Vector3d grasp_position = object_center - object_half_size_x * approach_direction;
    Eigen::Vector3d prepare_position = grasp_position + approach_distance * approach_direction;
    
    // 固定姿态：吸盘朝+x方向
    Eigen::Quaterniond q_eef(1.0, 0.0, 0.0, 0.0);
    
    // 构建返回Pose
    geometry_msgs::msg::Pose result;
    result.position.x = prepare_position.x();
    result.position.y = prepare_position.y();
    result.position.z = prepare_position.z();
    result.orientation.w = q_eef.w();
    result.orientation.x = q_eef.x();
    result.orientation.y = q_eef.y();
    result.orientation.z = q_eef.z();
    
    grasp_pose.position.x = grasp_position.x();
    grasp_pose.position.y = grasp_position.y();
    grasp_pose.position.z = grasp_position.z();
    grasp_pose.orientation.w = q_eef.w();
    grasp_pose.orientation.x = q_eef.x();
    grasp_pose.orientation.y = q_eef.y();
    grasp_pose.orientation.z = q_eef.z();
    
    return result;
}

/**
    @brief 根据父坐标系和期望位置，生成一个放置在某个位置的KFS
 */

// 将碰撞对象附着到 link6 上
bool ArmHandleNode::add_attached_kfs_collision() {
    moveit_msgs::msg::AttachedCollisionObject collision_object;
    collision_object.link_name              = "link6";
    collision_object.object.header.frame_id = "link6";
    collision_object.object.id              = "kfs";
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.35;
    primitive.dimensions[primitive.BOX_Y] = 0.35;
    primitive.dimensions[primitive.BOX_Z] = 0.35;

    collision_object.object.primitives.push_back(primitive);
    collision_object.object.primitive_poses.push_back(attached_kfs_pos);
    collision_object.object.operation = collision_object.object.ADD;
    psi->applyAttachedCollisionObject(collision_object);
    return true;
}

// 将碰撞对象从 link6 上移除
bool ArmHandleNode::remove_attached_kfs_collision() {
    moveit_msgs::msg::AttachedCollisionObject collision_object;
    collision_object.link_name              = "link6";
    collision_object.object.header.frame_id = "link6";
    collision_object.object.id              = "kfs";
    collision_object.object.operation       = collision_object.object.REMOVE;
    psi->applyAttachedCollisionObject(collision_object);

    remove_kfs_collision("kfs", move_group_interface->getPlanningFrame());
    return true;
}

bool ArmHandleNode::add_kfs_collision(const geometry_msgs::msg::Pose& pos, const std::string& object_id, const std::string& fram_id) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = fram_id;
    collision_object.id              = object_id;
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.35;
    primitive.dimensions[primitive.BOX_Y] = 0.35;
    primitive.dimensions[primitive.BOX_Z] = 0.35;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pos);
    collision_object.operation = collision_object.ADD;
    psi->applyCollisionObject(collision_object);
    return true;
}

bool ArmHandleNode::remove_kfs_collision(const std::string& object_id, const std::string& fram_id) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = fram_id;
    collision_object.id              = object_id;
    collision_object.operation       = collision_object.REMOVE;
    psi->applyCollisionObject(collision_object);
    return true;
}

bool ArmHandleNode::set_air_pump(bool enable){
    RCLCPP_INFO(node->get_logger(), "%s气泵", enable?"开启":"关闭");

    // 检查 driver_node 是否存在
    auto node_names = node->get_node_names();
    if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
        RCLCPP_INFO(node->get_logger(), "没有 driver_node 节点，不能%s气泵", enable?"开启":"关闭");
        return false;
    }


    // 使用 AsyncParametersClient 替代 SyncParametersClient，避免同步调用可能带来的问题
    // 注意：这里创建新的客户端而不是使用成员变量，避免状态问题
    auto temp_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "driver_node");

    // 等待服务器可用，最多等待 1s
    if(!temp_client->wait_for_service(1s)){
        RCLCPP_WARN(node->get_logger(), "driver_node 参数服务不可用");    
        return false;
    }
    
    // 异步设置参数
    auto future = temp_client->set_parameters({rclcpp::Parameter("enable_air_pump", enable)});
    
    // 等待结果，最多等待1秒
    try {
        const auto& results = future.get();

        // 检查结果向量是否非空
        if (results.empty()) {
            RCLCPP_WARN(node->get_logger(), "气泵%s失败：返回结果为空", enable ? "开启" : "关闭");
            return false;
        }

        // 检查第一个参数的结果（通常只需要检查第一个）
        const auto& result = results.front();
        if (result.successful) {
            RCLCPP_INFO(node->get_logger(), "气泵%s成功", enable ? "开启" : "关闭");
            return true;
        } else {
            RCLCPP_WARN(node->get_logger(), "气泵%s失败", enable ? "开启" : "关闭");
            return false;
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(node->get_logger(), "气泵%s异常: %s", enable ? "开启" : "关闭", e.what());
        return false;
    }
}

// bool ArmHandleNode:: planCartesianPathImproved(
//     const std::vector<geometry_msgs::msg::Pose>& waypoints,
//     moveit_msgs::msg::RobotTrajectory& trajectory,
//     double& achieved_fraction,
//     int max_retries = 10
// ){
//     std::vector<double> eef_step_attempts = {0.02, 0.01, 0.005, 0.002};
//     std::vector<double> jump_threshold_attempts = {0.0, 0.0, 0.0, 0.0};
//     bool collision_avoidance = true;

//     for (int retry = 0; retry < max_retries; ++retry){
//         // 自动选择参数
//         int eef_idx = std::min(retry / 2, (int)eef_step_attempts.size() - 1);
//         int jump_idx = std::min(retry / 2, (int)jump_threshold_attempts.size() - 1);
//         int collision_idx = retry % 2;

//         double eef_step = eef_step_attempts[eef_idx];
//         double jump_threshold = jump_threshold_attempts[jump_idx];
//         bool avoid_collisions = collision_avoidance;

//         RCLCPP_INFO(node->get_logger(), 
//             "尝试 %d: eef_step=%.4f, jump_threshold=%.4f, avoid_collisions=%s",
//                 retry + 1, eef_step, jump_threshold, 
//                 avoid_collisions ? "true" : "false");
        
//         // 执行规划
//         achieved_fraction = move_group_interface->computeCartesianPath(
//             waypoints, eef_step, jump_threshold, trajectory,
//             moveit_msgs::msg::Constraints(),
//             avoid_collisions
//         );

//         RCLCPP_INFO(node->get_logger(), "规划完成度 %.6f", achieved_fraction);

//         // 判断是否成功
//         if(achieved_fraction >= 0.995f){
//             RCLCPP_INFO(node->get_logger(), "路径规划成功");
//             return true;
//         }
//     }

//     RCLCPP_WARN(node->get_logger(), "所有尝试均未达到满意的规结果");
//     return false;
// }




bool ArmHandleNode::planLongPathSegmented(
    const geometry_msgs::msg::Pose& start_pose ,
    const geometry_msgs::msg::Pose& end_pose ,
    moveit_msgs::msg::RobotTrajectory& full_trajectory,
    double segment_length = 0.01  // 每段 1 厘米
){
    // 计算总距离和方向
    geometry_msgs::msg::Vector3 direction;
    direction.z = end_pose.position.z - start_pose.position.z;
    direction.y = end_pose.position.y - start_pose.position.y;
    direction.x = end_pose.position.x - start_pose.position.x;

    double total_distance = std::sqrt(
        direction.x * direction.x +
        direction.y * direction.y +
        direction.z * direction.z
    );

    if (total_distance < 0.001){
        RCLCPP_INFO(node->get_logger(), "起点和终点距离太近，无需规划");
        return true;
    }

    // 归一化方向
    direction.x /= total_distance;
    direction.y /= total_distance;
    direction.z /= total_distance;

    // 计算分段数
    int num_segments = static_cast<int>(std::ceil(total_distance / segment_length));  // std::ceil 向上取整

    std::vector<geometry_msgs::msg::Pose> segment_waypoints;
    segment_waypoints.push_back(start_pose);

    // 生成分段路径点
    for(int i = 1; i < num_segments; ++i){
        double distance = i *  segment_length;
        geometry_msgs::msg::Pose pose;
        pose.position.x = start_pose.position.x + direction.x * distance;
        pose.position.y = start_pose.position.y + direction.y * distance;
        pose.position.z = start_pose.position.z + direction.z * distance;
        pose.orientation = start_pose.orientation; // 保持姿态不变
        segment_waypoints.push_back(pose);
    }
    segment_waypoints.push_back(end_pose);

    // 规划并合并轨迹
    moveit_msgs::msg::RobotTrajectory segment_trajectory;

    double fraction ;
    count = 0;

    do {
        ++count;
        fraction = move_group_interface->computeCartesianPath(
            segment_waypoints, 0.01, 0.0, segment_trajectory, false
        );
        RCLCPP_WARN(node->get_logger(), "分段规划 %d/%d, 进度%.6f", count, MAX_COUNT_, fraction);
    } while (fraction < 0.995f && count <= MAX_COUNT_);

    
    if(fraction < 0.995f){
        RCLCPP_ERROR(node->get_logger(), "分段规划失败");
        return false ;
    } else {
        RCLCPP_INFO(node->get_logger(), "分段规划成功");
        return true;
    }

    // if(!planCartesianPathImproved(segment_waypoints, segment_trajectory, fraction, 10)){
    //     RCLCPP_ERROR(node->get_logger(), "分段规划失败");
    //     return false;
    // }


    // 合并轨迹（简化处理，实际可能需要更复杂的合并逻辑）
    full_trajectory = segment_trajectory;
    return true;
}

// 优化的笛卡尔路径规划函数
bool ArmHandleNode::computeCartesianPathOptimized(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    moveit_msgs::msg::RobotTrajectory& trajectory,
    double& fraction) {
    
    // 使用更小的步长进行初始规划
    double eef_step = 0.005;  // 5mm 步长
    double jump_threshold = 0.0;
    
    // 第一次尝试
    fraction = move_group.computeCartesianPath(waypoints, eef_step, 
        jump_threshold, trajectory, false);
    
    if (fraction >= 0.995) {
        // 规划成功，进行后处理
        return postProcessTrajectory(trajectory);
    }
    
    // 如果第一次失败，尝试减小步长
    std::vector<double> step_sizes = {0.002, 0.001};
    
    for (double step : step_sizes) {
        fraction = move_group.computeCartesianPath(waypoints, step,
            jump_threshold, trajectory, false);
        
        if (fraction >= 0.995) {
            return postProcessTrajectory(trajectory);
        }
    }
    
    // 如果仍然失败，返回原始轨迹
    return false;
}

// 轨迹后处理：平滑和速度优化
bool ArmHandleNode::postProcessTrajectory(
    moveit_msgs::msg::RobotTrajectory& trajectory) {
    
    if (trajectory.joint_trajectory.points.empty()) {
        return false;
    }
    
    // 1. 对轨迹点进行插值，增加轨迹点密度
    std::vector<double> time_stamps;
    for (const auto& point : trajectory.joint_trajectory.points) {
        double t = point.time_from_start.sec + 
            point.time_from_start.nanosec * 1e-9;
        time_stamps.push_back(t);
    }
    
    // 插值到更密的时间点
    std::vector<double> new_time_stamps;
    double min_dt = 0.01;  // 10ms 间隔
    double max_time = time_stamps.back();
    
    for (double t = 0; t <= max_time; t += min_dt) {
        new_time_stamps.push_back(t);
    }
    
    // 2. 应用速度限制
    double max_velocity = 0.5;  // 弧度/秒
    double max_acceleration = 0.3;  // 弧度/秒²
    
    for (size_t i = 1; i < trajectory.joint_trajectory.points.size(); ++i) {
        auto& point = trajectory.joint_trajectory.points[i];
        auto& prev_point = trajectory.joint_trajectory.points[i-1];
        
        double dt = (point.time_from_start.sec - prev_point.time_from_start.sec) +
            (point.time_from_start.nanosec - prev_point.time_from_start.nanosec) * 1e-9;
        
        if (dt > 1e-6) {
            for (size_t j = 0; j < point.velocities.size(); ++j) {
                double velocity = point.velocities[j];
                double limited_velocity = std::copysign(
                    std::min(std::abs(velocity), max_velocity), velocity);
                point.velocities[j] = limited_velocity;
                
                // 限制加速度
                double accel = (velocity - prev_point.velocities[j]) / dt;
                double limited_accel = std::copysign(
                    std::min(std::abs(accel), max_acceleration), accel);
                
                // 重新计算速度以符合加速度限制
                point.velocities[j] = prev_point.velocities[j] + 
                    limited_accel * dt;
            }
        }
    }
    
    return true;
}
