#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include <chrono>
#include <memory>
#include <vector>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
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


    //设置障碍物的位置
    auto collision_object=[frame_id=move_group_interface.getPlanningFrame()]{
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id=frame_id;
        collision_object.id="box1";
        shape_msgs::msg::SolidPrimitive primitive;

        primitive.type=primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.35;
        primitive.dimensions[primitive.BOX_Y] = 0.35;
        primitive.dimensions[primitive.BOX_Z] = 0.35;

        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x=0.9;
        box_pose.position.y=0.3;
        box_pose.position.z=0.35/2;
        box_pose.orientation.w=1.0;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();


    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.addCollisionObjects({collision_object});



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



/*
上位机代码结构构想：
节点：robotic_task(一进入main函数，就无限永久spin)
    Action服务端：arm_handle_action
    Topic发布者：joint_target

    如果有订阅到动作请求，那么执行几次抓取规划放到队列中，然后让一个线程去操作Moveit的接口，解析plan和发布joint_target，
    最后抓取成功后返回SUCCESS，如果规划失败，返回REJECT，表示需要重新移动机器人位置方便机械臂能够抓取到。
    确认抓取成功后，自己再规划一次，将期望目标点放到队列中，根据当前机器人的状态（存储一个块，存储两个块）【状态保存】，确定机械臂要到达的位置和是否放下块.

节点：robot_driver(一进入main函数，就无限永久spin)
    Topic订阅者：myjoints_target    订阅电机目标，发送给MCU
    Topic发布者：joint_states       向Moveit发布关节的状态

节点（李振宇）：vision_search
    Action客户端 arm_handle_action:请求抓取一次物块
    Action服务端 get_kfs_action：接受robot_run节点的请求获取KFS的服务
    //其他部分

    任务：
    识别KFS，查询相机坐标系到机械臂base_link的TF变换，将坐标变换为base_link下的坐标后发送给arm_handle_action，
    并监听反馈，再将反馈继续传递给robot_run，如果直接识别失败也要通知robot_run

节点（张坤）：robot_run
    Action客户端get_kfs_action  :在特定时刻请求一次获取KFS，失败后尝试移动机器人再次尝试
    //其他部分
    规划机器人总体运动，目前先用Action写请求vision_search节点进行识别。
*/
