#pragma once

#include "visualization_msgs/msg/marker.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/publisher.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <robot_interfaces/msg/arm.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.h>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp_action/create_client.hpp>
#include <robot_interfaces/action/catch.hpp>

#include <thread>
#include <memory>
#include <atomic>
#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef enum{
    ROBOTIC_ARM_TASK_MOVE=1,           //移动到某个位姿
    ROBOTIC_ARM_TASK_CATCH_TARGET=2,   //捕获处于某个坐标下的KFS
    ROBOTIC_ARM_TASK_PLACE_TARGET=3    //将机器人上的KFS放置到某个坐标
}ArmTask;       //机械臂任务类型

typedef enum{
    ROBOTIC_ARM_STAGE_IDEL,
    ROBOTIC_ARM_STAGE_MOVE_TO_READY_CATCH_POINT,
    ROBOTIC_ARM_STAGE_MOVE_TO_CATCH_POINT,
    ROBOTIC_ARM_STAGE_CATCH_TARGET,
    ROBOTIC_ARM_STAGE_MOVE_TO_RELEASE_POINT,
    ROBOTIC_ARM_STAGE_RELESE_TARGET,
}ArmTaskStage;

class ArmHandleNode{
public:
    explicit ArmHandleNode(const rclcpp::Node::SharedPtr node);
    ~ArmHandleNode();

    // 关节空间
    enum class PlanningMode {
        JOINT_SPACE,    // 关节空间规划模式
        CARTESIAN_SPACE, // 笛卡尔空间规划模式
        HYBRID          // 混合模式（自动切换）
    };

    
private:
    rclcpp::Node::SharedPtr node;
    rclcpp_action::Server<robot_interfaces::action::Catch>::SharedPtr arm_handle_server;  //机械臂任务接口

    std::mutex task_mutex_;             //用于线程同步
    std::condition_variable task_cv_;
    bool has_new_task_{false}; // 是否开始新线程

    rclcpp::SyncParametersClient::SharedPtr param_client;
    std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> current_goal_handle;
    moveit::core::RobotModelConstPtr robot_module;
    std::unique_ptr<std::thread> arm_task_thread;    //执行期望，解析plan并发布节点的线程
    geometry_msgs::msg::Pose task_target_pos; // 目标位置
    std::atomic<bool> is_running_arm_task{false}; // 目标位置
    std::atomic<bool> cancle_current_task{false};
    std::atomic<int> current_task_type{0}; // 任务类型
    std::atomic<int> current_kfs_num{0}; // kfs的数量
    std::unique_ptr<tf2_ros::Buffer> camera_link0_tf_buffer; // 坐标变换
    std::shared_ptr<tf2_ros::TransformListener> camera_link0_tf_listener;
    geometry_msgs::msg::TransformStamped camera_link0_tf;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mark_pub_;


    geometry_msgs::msg::Pose attached_kfs_pos; // 附着在吸盘上的KFS与机器人的相对关系


    //障碍物/KFS定义
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> psi;
    //moveit_msgs::msg::AttachedCollisionObject attached_kfs;


    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal);
    rclcpp_action::CancelResponse cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle);
    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle);

    //机械臂动作处理
    void arm_catch_task_handle();
    //bool send_plan(const moveit_msgs::msg::RobotTrajectory& trajectory);
    // 计算从上方接近物体的准备位姿，返回准备位姿并通过 grasp_pose 输出最终抓取位姿（贴合上表面）
    geometry_msgs::msg::Pose calculate_prepare_pos(const geometry_msgs::msg::Pose &box_pos, double approach_distance, geometry_msgs::msg::Pose &grasp_pose);
    bool add_attached_kfs_collision();
    bool remove_attached_kfs_collision();
    bool add_kfs_collision(const geometry_msgs::msg::Pose &pos,const std::string &object_id,const std::string &fram_id);
    bool remove_kfs_collision(const std::string &object_id,const std::string &fram_id);

    ///******************************************************
    // 关节空间 
    //  */
    // 规划参数配置
    const double SWITCH_DISTANCE_THRESHOLD = 0.15;  // 切换距离阈值（米）
    const double CARTESIAN_GOAL_TOLERANCE = 0.001;   // 笛卡尔空间目标容差
    const double JOINT_GOAL_TOLERANCE = 0.005;       // 关节空间目标容差
    const double VELOCITY_SCALING = 0.5;             // 速度缩放因子
    const double ACCELERATION_SCALING = 0.5;         // 加速度缩放因子
    // rclcpp::Node::SharedPtr node_;
    // moveit::planning_interface::MoveGroupInterface::SharedPtr move_group_interface; 
    moveit::planning_interface::PlanningSceneInterface planning_scene_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // 坐标系变换
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // 接收和订阅坐标变换消息

};
