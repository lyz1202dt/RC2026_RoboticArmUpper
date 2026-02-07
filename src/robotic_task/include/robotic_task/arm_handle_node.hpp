#pragma once

// ========= 项目内部头文件 =========
// 分段速度控制器
// - 通常用于将整条运动划分为多个 segment
// - 每段可使用不同的速度规划或控制策略
#include "SegmentedVelocityController.hpp"

// 轨迹平滑器
// - 对离散轨迹进行平滑（速度 / 加速度连续）
// - 常见实现：五次多项式、滤波、时间重分配等
#include "TragectorySmoother.hpp"

// ========= ROS 可视化相关 =========

// RViz 可视化 Marker
// - 用于发布调试用的可视化信息（点、线、坐标轴等）
#include "visualization_msgs/msg/marker.hpp"

// ========= geometry_msgs（位姿与几何数据） =========

// Pose 的底层结构定义（一般不需要直接用，但某些模板或内部接口会依赖）
#include <geometry_msgs/msg/detail/pose__struct.hpp>

// 位姿（位置 + 四元数姿态）
// - 机器人末端位姿
// - 目标抓取位姿
#include <geometry_msgs/msg/pose.hpp>

// 带时间戳和坐标系的变换
// - TF 广播或监听用
#include <geometry_msgs/msg/transform_stamped.hpp>

// ========= MoveIt 轨迹与规划 =========

// MoveIt 生成的机器人轨迹结构
// - 包含 joint_trajectory / multi_dof_trajectory
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>

// ========= ROS2 基础设施 =========

// 参数客户端
// - 用于动态读取其他节点的参数
#include <rclcpp/parameter_client.hpp>

// ROS2 发布器接口
#include <rclcpp/publisher.hpp>

// ========= TF2（坐标变换） =========

// 四元数数学工具
// - roll/pitch/yaw 与 quaternion 转换
#include <tf2/LinearMath/Quaternion.h>

// TF 缓存
// - 存储并查询坐标变换树
#include <tf2_ros/buffer.h>

// TF 监听器
// - 从 /tf 和 /tf_static 接收变换
#include <tf2_ros/transform_listener.hpp>

// TF2 与 geometry_msgs 的转换工具
// - tf2::Transform <-> geometry_msgs
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include <geometry_msgs/msg/pose.hpp>

// ========= 自定义接口 =========

// 自定义机械臂消息
// - 通常用于发布机械臂状态或控制命令
#include <robot_interfaces/msg/arm.hpp>

// ========= MoveIt 高层接口 =========

// MoveGroupInterface
// - MoveIt 最常用的 C++ 接口
// - 负责规划、执行、设置目标
#include <moveit/move_group_interface/move_group_interface.h>

// PlanningSceneInterface
// - 管理场景中的障碍物
// - 动态添加 / 删除碰撞物体
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// 碰撞物体定义
#include <moveit_msgs/msg/collision_object.h>

// MoveIt 错误码
// - 判断规划或执行是否成功
#include <moveit/utils/moveit_error_code.h>

// ========= ROS2 核心 =========

// ROS2 节点、日志、时间等核心功能
#include <rclcpp/rclcpp.hpp>

// ========= ROS2 Action 相关 =========

// Action Server 接口
// - 用于实现 FollowJointTrajectory / Catch 等动作
#include <rclcpp_action/server.hpp>

// Action 目标句柄
// - 管理 goal 的状态（接受 / 执行 / 取消）
#include <rclcpp_action/server_goal_handle.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// Action Client 创建工具
#include <rclcpp_action/create_client.hpp>

// 自定义抓取 Action
// - 通常包含目标位姿、结果状态等
#include <robot_interfaces/action/catch.hpp>

#include <thread>
#include <memory>
#include <atomic>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>


typedef enum{
    ROBOTIC_ARM_TASK_MOVE=1,           //移动到某个位姿
    ROBOTIC_ARM_TASK_CATCH_TARGET=2,   //捕获处于某个坐标下的KFS
    ROBOTIC_ARM_TASK_PLACE_TARGET=3    //将机器人上的KFS放置到某个坐标
}ArmTask;       //机械臂任务类型

typedef enum{
    ROBOTIC_ARM_STAGE_IDEL,   // 空闲                  
    ROBOTIC_ARM_STAGE_MOVE_TO_READY_CATCH_POINT,    // 移动到预备抓取点
    ROBOTIC_ARM_STAGE_MOVE_TO_CATCH_POINT,    // 移动到抓取点
    ROBOTIC_ARM_STAGE_CATCH_TARGET,    // 抓取目标

    // =============== [VISUAL SERVO] =============
    ROBOTIC_ARM_STAGE_VISUAL_SERVOING,  // 视觉伺服控制

    ROBOTIC_ARM_STAGE_MOVE_TO_RELEASE_POINT,    // 移动到释放点
    ROBOTIC_ARM_STAGE_RELESE_TARGET,    // 释放目标
}ArmTaskStage;


// ===== [VISUAL SERVO] =====
// 视觉伺服速度控制器（PBVS）
class VisualServoController{
    public:
        struct ServoInput{
            Eigen::Vector3d position_error;  // 末端到目标的位姿误差
            Eigen::Vector3d orientation_error; //  旋转误差
        };

        struct ServoOutput{
            Eigen::VectorXd joint_velocity;   // 关节速度
        };

        bool compute(
            const ServoInput& input, 
            const Eigen::MatrixXd& jacobian, 
            ServoOutput& output
        );

    private:
        double kp_pos_ = 0.0;   // 位置比例增益
        double kp_ori_ = 0.6;   // 姿态比例增益
        double max_vel_ = 0.3;  // 最大关节速度
};








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
    bool success; // 规划或执行是否成功
    rclcpp::Node::SharedPtr node;
    rclcpp_action::Server<robot_interfaces::action::Catch>::SharedPtr arm_handle_server;  //机械臂任务接口

    std::mutex task_mutex_;             //用于线程同步
    std::condition_variable task_cv_;
    bool has_new_task_{false}; // 是否开始新线程

    rclcpp::AsyncParametersClient::SharedPtr param_client;
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
    enum class ApproachMode {
        AUTO = 0,      // 自动选择（根据与基座距离决定是否侧面抓取）
        TOP,           // 从上方抓取
        SIDE_ROBOT     // 从靠近机器人的侧面抓取
    };

    geometry_msgs::msg::Pose calculate_prepare_pos(const geometry_msgs::msg::Pose &box_pos, double approach_distance, geometry_msgs::msg::Pose &grasp_pose, ApproachMode mode = ApproachMode::AUTO);
    bool add_attached_kfs_collision();
    bool remove_attached_kfs_collision();
    bool add_kfs_collision(const geometry_msgs::msg::Pose &pos,const std::string &object_id,const std::string &fram_id);
    bool remove_kfs_collision(const std::string &object_id,const std::string &fram_id);

    bool set_air_pump(bool enable); // 设置气泵参数

    bool planCartesianPathImproved(
        const std::vector<geometry_msgs::msg::Pose>& waypoints,
        moveit_msgs::msg::RobotTrajectory& trajectory,
        double& achieved_fraction,
        int max_retries
    );  // 更好的笛卡尔路径规划实现函数

    bool planLongPathSegmented(
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Pose& end_pose,
        moveit_msgs::msg::RobotTrajectory& full_trajectory,
        double segment_length   // 每段5厘米
    ); // 分解长路经为多个短路经

    bool postProcessTrajectory(
    moveit_msgs::msg::RobotTrajectory& trajectory); // 轨迹后处理：平滑和速度优化

    bool computeCartesianPathOptimized(
    moveit::planning_interface::MoveGroupInterface& move_group,
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    moveit_msgs::msg::RobotTrajectory& trajectory,
    double& fraction); // 优化的笛卡尔路径规划函数

    ///******************************************************
    // 关节空间 
    //  */
    // 规划参数配置
    const double SWITCH_DISTANCE_THRESHOLD = 0.01;  // 切换距离阈值（米）
    const double CARTESIAN_GOAL_TOLERANCE = 0.001;   // 笛卡尔空间目标容差
    const double JOINT_GOAL_TOLERANCE = 0.005;       // 关节空间目标容差
    const double VELOCITY_SCALING = 0.4;             // 速度缩放因子
    const double ACCELERATION_SCALING = 0.3;         // 加速度缩放因子
    // rclcpp::Node::SharedPtr node_;
    // moveit::planning_interface::MoveGroupInterface::SharedPtr move_group_interface; 
    // moveit::planning_interface::PlanningSceneInterface planning_scene_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // 坐标系变换
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // 接收和订阅坐标变换消息
    geometry_msgs::msg::Pose calculate_prepare_pos_with_orientation(const geometry_msgs::msg::Pose& box_pos, 
    double approach_distance, 
    geometry_msgs::msg::Pose &grasp_pose, 
    ApproachMode mode);

    int count ; // 次数可以是任何事情的次数
    const int MAX_COUNT_ = 100;

    std::shared_ptr<TrajectorySmoother> trajectory_smoother_;
    std::shared_ptr<SegmentedVelocityController> segmented_velocity_controller_;




    // ============ [VISUAL SERVO] ================
    ArmTaskStage current_stage_{ROBOTIC_ARM_STAGE_IDEL};
    std::shared_ptr<VisualServoController> visual_servo_controller_;
    // 伺服阀值
    const double SERVO_POS_THRESHOLD = 0.002;  // 2mm
    const double SERVO_ORI_THRESHOLD = 0.03;   // ~2 deg
    // 视觉目标
    geometry_msgs::msg::Pose visual_target_pose;
    // ============ [VISUAL SERVO] =================
    bool visualServoLoop();
    bool computeVisualError(
        Eigen::Vector3d& pos_err, 
        Eigen::Vector3d& orii_err
    );
};
