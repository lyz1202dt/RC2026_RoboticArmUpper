#include <chrono>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "joint_execut_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    auto logger = rclcpp::get_logger("joint_execut_node");

    auto joint_pos_state = std::vector<double>{0.1, -0.2, 0.3, 0.0, 0.0, 0.0};
    auto joint_pos_vel   = std::vector<double>{0.0, -0.0, 0.0, 0.0, 0.0, 0.0};
    
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(node, "robotic_arm");
    move_group_interface.setPlanningTime(1);
    move_group_interface.setNumPlanningAttempts(100);

    auto robot_model = move_group_interface.getRobotModel();
    

    moveit::core::RobotState start_state(robot_model);
    start_state.setJointGroupPositions("robotic_arm", joint_pos_state);
    start_state.setJointGroupVelocities("robotic_arm", joint_pos_vel);
    move_group_interface.setStartState(start_state);

    geometry_msgs::msg::Pose target_pos;
    target_pos.position.x    = 0.4;
    target_pos.position.y    = 0.0;
    target_pos.position.z    = 0.5;
    target_pos.orientation.w = 1.0;

    move_group_interface.setPoseTarget(target_pos);                      // 设置一个目标点

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success) {
        RCLCPP_INFO(logger, "规划失败!");
        rclcpp::shutdown();
        return -1;
    }

    const auto& traj = plan.trajectory.joint_trajectory;
    auto start_time_point=std::chrono::high_resolution_clock::now();
    for (const auto & pt : traj.points) {
         double t        = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9;
        const auto& q   = pt.positions;
        const auto& dq  = pt.velocities;
        const auto& ddq = pt.accelerations;

        // 打印或发送给你的底层驱动
        std::cout << "t=" << t << " positions: ";
        for (double v : q)
            std::cout << v << " ";
        std::cout << "\n";
        std::this_thread::sleep_until(start_time_point+std::chrono::duration<double>(t));
    }

    rclcpp::shutdown();
    return 0;
}
