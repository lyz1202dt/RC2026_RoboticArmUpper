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
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <memory>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/detail/constraints__struct.hpp>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
// #include <qt5/QtGui/qvalidator.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdint>

using namespace std::chrono_literals;

ArmHandleNode::ArmHandleNode(const rclcpp::Node::SharedPtr node) : node(node), visual_servoing_handler_(node) {

    this->node = node;                                                                                         // 以依赖注入的方式，传入要管理的节点
    param_client      = std::make_shared<rclcpp::AsyncParametersClient>(node, "driver_node");
    arm_task_thread   = nullptr; // 延后创建线程，确保构造完成后再启动
    arm_handle_server = rclcpp_action::create_server<robot_interfaces::action::Catch>(
        node, "robotic_task",                                                                                  // 创建动作服务-服务端
        std::bind(&ArmHandleNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ArmHandleNode::cancel_goal, this, std::placeholders::_1), 
        std::bind(&ArmHandleNode::handle_accepted, this, std::placeholders::_1)
    );












    camera_link0_tf_buffer   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    camera_link0_tf_listener = std::make_shared<tf2_ros::TransformListener>(*camera_link0_tf_buffer);
    tf_buffer_= std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    vision_subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "robotic_task_", 10,
    std::bind(&ArmHandleNode::visionCallback, this, std::placeholders::_1));


















    // TODO: 使用伺服时，发布假。使用moveit时，发布真。
    moveit_pub_ = node->create_publisher<robot_interfaces::msg::Moveit>("moveit_command", 10);


    // 坐标变换监听

   
   
    move_group_interface     = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robotic_arm");
    psi                      = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    mark_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("debug_mark", 10);

    node->create_wall_timer(100ms, [this]() {
        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            if(!is_running_arm_task)    //如果此时没有进行机械臂抓取，那么立即返回
                return;
        }

        // Marker 在三维空间中显示不同类型的可视化元素
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp    = this->node->now();

        marker.ns = "kfs_pos";
        marker.id = 0;

        marker.type   = visualization_msgs::msg::Marker::SPHERE; // 显示球体标记
        marker.action = visualization_msgs::msg::Marker::ADD; // 第一次发布：添加标记

        /*****************************************************************************
         *  marker.action = visualization_msgs::msg::Marker::DELETE;
         *  marker.id = 0;  // 指定要删除的标记 ID
         *  mark_pub_->publish(marker);
         *
         *  marker.action = visualization_msgs::msg::Marker::DELETEALL;
         *  mark_pub_->publish(marker);
         */

        // 球心位置
        marker.pose = task_target_pos;

        // 姿态（球体无关，但必须合法）球体看不出旋转效果，但需符合归一化要求
        marker.pose.orientation.w = 1.0;

        // 尺寸（直径，单位 m）
        marker.scale.x = 0.08;
        marker.scale.y = 0.08;
        marker.scale.z = 0.08;

        // 颜色
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;

        marker.lifetime.sec=1;  //不再发布位置1s后删除
        marker.lifetime.nanosec=0;

        mark_pub_->publish(marker);
    });


    attached_kfs_pos.orientation.w = 1.0;   //附着在吸盘上的KFS与机器人的相对关系
    attached_kfs_pos.position.x    = 0.0;
    attached_kfs_pos.position.y    = 0.0;
    attached_kfs_pos.position.z    = -0.24;


    // 配置基本参数
    move_group_interface->setPlanningTime(10.0); // 规划时间设置

    move_group_interface->setMaxVelocityScalingFactor(VELOCITY_SCALING); // 最大速度缩放因子

    move_group_interface->setMaxAccelerationScalingFactor(ACCELERATION_SCALING); // 最大加速度缩放因子
                            

    // 初始化TF2


    // 在构造函数末尾启动任务线程，避免部分成员尚未初始化时线程读取它们
    try {
        arm_task_thread = std::make_unique<std::thread>(std::bind(&ArmHandleNode::arm_catch_task_handle, this));
    } catch (const std::exception &e) {
        RCLCPP_WARN(node->get_logger(), "创建机械臂任务线程失败: %s", e.what());
        arm_task_thread = nullptr;
    }

}




ArmHandleNode::~ArmHandleNode() {
    task_mutex_.lock(); // 加锁，保护共享变量，其他线程暂时不能访问has_new_task_
    has_new_task_ = true;    // 置为 true，让线程退出循环时能通过条件判断。有新任务
    task_mutex_.unlock(); // 解锁
    task_cv_.notify_all(); // 唤醒等待条件变量的线程

    if (arm_task_thread && arm_task_thread->joinable()) // 检测线程是否还在运行
        arm_task_thread->join(); // 等待线程结束
}

rclcpp_action::GoalResponse
    ArmHandleNode::handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal) {
    (void)uuid;
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        if (is_running_arm_task) // 如果正在运行机械臂动作，那么拒绝新的请求
            return rclcpp_action::GoalResponse::REJECT;
    }

    try {
        camera_link0_tf = camera_link0_tf_buffer->lookupTransform("base_link", "camera_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) { 
        RCLCPP_WARN(node->get_logger(), "警告：相机坐标系和变换查询失败，拒绝机械臂目标请求");
        return rclcpp_action::GoalResponse::REJECT;
    }

    
    // Transform the pose manually
    // geometry_msgs::msg::Pose transformed_pose;
    // 将相机识别的物体位置转换到主坐标系下
    // 将相机识别的物体位置转换到主坐标系下，保存到 task_target_pos（不要覆盖为 camera_frame 的原始 pose）
    tf2::doTransform(goal->target_pose, task_target_pos, camera_link0_tf);
    RCLCPP_INFO(node->get_logger(), "原始目标位姿: Pos(%lf,%lf,%lf), Rot(%lf,%lf,%lf,%lf)",
    goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z,
    goal->target_pose.orientation.w, goal->target_pose.orientation.x, 
    goal->target_pose.orientation.y, goal->target_pose.orientation.z);



    // TODO:姿态一定是大约朝前的，所以这里定死姿态
    // RCLCPP_INFO(node->get_logger(),"实际位置为(%lf,%lf,%lf)",task_target_pos.position.x,task_target_pos.position.y,task_target_pos.position.z);

    // task_target_pos.orientation.w = 0.004481;
    // task_target_pos.orientation.x = 0.708322;
    // task_target_pos.orientation.y = -0.004257;
    // task_target_pos.orientation.z = -0.705862;

    // 四元数归一化处理
    auto qin=task_target_pos.orientation;
    tf2::Quaternion q(qin.x, qin.y, qin.z, qin.w);
    q.normalize();
    task_target_pos.orientation.w = q.w();
    task_target_pos.orientation.x = q.x();
    task_target_pos.orientation.y = q.y();
    task_target_pos.orientation.z = q.z();


    current_task_type = goal->action_type; // 任务类型
    RCLCPP_INFO(node->get_logger(), "Debug1");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
    ArmHandleNode::cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {
    (void)goal_handle;
    cancle_current_task = true;
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        is_running_arm_task = false;
    }

    // 删除之前添加的碰撞体，避免残留的虚拟碰撞体影响后续任务规划。
    // 缓存规划框名称
    remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame()); 
    

    // 关闭气泵
    set_air_pump(false);

    return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmHandleNode::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {
    current_goal_handle = goal_handle;
    // 设置取消标志，通知执行循环终止当前任务
    {
        std::lock_guard<std::mutex> lock(task_mutex_);
        is_running_arm_task = true;
    }

    // 更新任务状态，表示机械臂任务已停止
    cancle_current_task = false;

    task_mutex_.lock(); // 通知需要执行任务
    has_new_task_ = true;
    task_mutex_.unlock();
    task_cv_.notify_one();
    // TODO:给出信号量开始执行
}

// 机械臂
/*
实现了完整的机械臂抓取-放置任务流程，包括运动规划、轨迹执行、碰撞检测管理和任务状态反馈。
*/
void ArmHandleNode::arm_catch_task_handle() {
    RCLCPP_INFO(node->get_logger(), "进入机械臂任务处理线程");
    bool first_run = true;

    // 保存规划好的运动轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool continue_flag = false;

    // auto robot_module = move_group_interface->getRobotModel();
    auto feedback_msg = std::make_shared<robot_interfaces::action::Catch::Feedback>();
    auto finished_msg = std::make_shared<robot_interfaces::action::Catch::Result>();

    std::this_thread::sleep_for(5s); // 等待RVIZ2启动完成
    {                                // 为规划环境增加四个竖起来的杆（R2上底盘抬升部分需要）
        // 添加障碍物
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_interface->getPlanningFrame();
        collision_object.id              = "institution";
        collision_object.primitives.resize(4);
        collision_object.primitive_poses.resize(4);

        collision_object.primitive_poses[0].orientation.w = 1;
        collision_object.primitive_poses[0].position.x    = -0.60;
        collision_object.primitive_poses[0].position.y    = 0.35;
        collision_object.primitive_poses[0].position.z    = 0.3;

        collision_object.primitive_poses[1].orientation.w = 1;
        collision_object.primitive_poses[1].position.x    = 0.05;
        collision_object.primitive_poses[1].position.y    = 0.35;
        collision_object.primitive_poses[1].position.z    = 0.3;

        collision_object.primitive_poses[2].orientation.w = 1;
        collision_object.primitive_poses[2].position.x    = -0.60;
        collision_object.primitive_poses[2].position.y    = -0.35;
        collision_object.primitive_poses[2].position.z    = 0.3;

        collision_object.primitive_poses[3].orientation.w = 1;
        collision_object.primitive_poses[3].position.x    = 0.05;
        collision_object.primitive_poses[3].position.y    = -0.35;
        collision_object.primitive_poses[3].position.z    = 0.3;

        shape_msgs::msg::SolidPrimitive primitive;
        collision_object.primitives[0].type = primitive.BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[primitive.BOX_X] = 0.06;
        collision_object.primitives[0].dimensions[primitive.BOX_Y] = 0.06;
        collision_object.primitives[0].dimensions[primitive.BOX_Z] = 0.6;
        collision_object.primitives[3] = collision_object.primitives[2] = collision_object.primitives[1] = collision_object.primitives[0];

        collision_object.operation = collision_object.ADD;
        psi->applyCollisionObject(collision_object);                 // 应用障碍物

        RCLCPP_INFO(node->get_logger(), "应用障碍物");
    }

    do {
        // setStartStateToCurrentState 将规划的起始状态设置为当前实时状态
        move_group_interface->setStartStateToCurrentState();
        move_group_interface->setNamedTarget("start_pos_2");

        success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        count = 0;
        while(success == false && count <=  MAX_COUNT_){
            success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            RCLCPP_WARN(node->get_logger(), "起始位置规划失败，重行规划%d次", count+1);
            count ++ ;
        }
        if (!success) {
            finished_msg->kfs_num = current_kfs_num;
            finished_msg->reason  = "机械臂无法回到起始位置，路径规划失败";
            current_goal_handle->abort(finished_msg);
            continue_flag = true;
            break;
        } 
    } while (move_group_interface->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS);

    RCLCPP_INFO(node->get_logger(), "机械臂任务处理线程启动完成，等待任务请求");

    while (rclcpp::ok()) {
        continue_flag = false;

        {
            std::lock_guard<std::mutex> lock(task_mutex_);
            is_running_arm_task = false;
        }
        std::unique_lock<std::mutex> lock(task_mutex_);

        // 等待直到lambda表达式返回真
        bool ok       = task_cv_.wait_for(lock, 5s, [this]() { return has_new_task_; }); 
        has_new_task_ = false;
        lock.unlock();

        if (!ok) {
            // 超时：打印当前位姿并继续等待
            auto pos = move_group_interface->getCurrentPose();
            RCLCPP_INFO(
                node->get_logger(), "当前机械臂位姿:Pos(%lf,%lf,%lf),Rot(%lf,%lf,%lf,%lf)", pos.pose.position.x, pos.pose.position.y,
                pos.pose.position.z, pos.pose.orientation.w, pos.pose.orientation.x, pos.pose.orientation.y, pos.pose.orientation.z
            );
            continue;
        }

        RCLCPP_INFO(node->get_logger(), "接收到新的任务请求");


        if (first_run)                                                                   // 第一次执行时，设置一次规划器参数
        {
            // 容差
            move_group_interface->setGoalJointTolerance(0.01); // 关节容差 0.01 rad
            move_group_interface->setGoalPositionTolerance(0.005); // 位置容差 5mm
            move_group_interface->setGoalOrientationTolerance(0.05); // 姿态容差放宽，避免目标树无有效采样
            // 调用 setPlanningTime 方法设置规划器的最大规划时间为 5.0 秒
            move_group_interface->setPlanningTime(10.0);

            // 设置采样次数（最多尝试次数）
            move_group_interface->setNumPlanningAttempts(20);

            first_run = false;
        }

        if (!rclcpp::ok()) {
            break;
        }

        if (current_task_type == ROBOTIC_ARM_TASK_MOVE)           // 要求机械臂移动到某个位姿（因为移动动作在一个周期内完成，所以不再需要执行）
        {
            move_group_interface->setPoseTarget(task_target_pos); // 设置目标
            
            bool success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划当前位置到目标位置的曲线
            if (!success) {
                RCLCPP_WARN(node->get_logger(), "全姿态目标规划失败，尝试位置优先规划");
                move_group_interface->clearPoseTargets();
                move_group_interface->setStartStateToCurrentState();
                move_group_interface->setPositionTarget(
                    task_target_pos.position.x,
                    task_target_pos.position.y,
                    task_target_pos.position.z
                );
                success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            }
            // count = 0;
            // while(success == false && count <=  MAX_COUNT_){
            //     success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            //     RCLCPP_WARN(node->get_logger(), "准备位置规划失败，重行规划%d次", count+1);
            //     count ++ ;
            // }
            if (success) {                                                                               // 阻塞函数，发送轨迹

                // 执行规划好的轨迹并将结果保存到ret中
                auto ret = move_group_interface->execute(plan);
                if (ret != moveit::core::MoveItErrorCode::SUCCESS) {
                    finished_msg->reason  = "任务被客户端取消";
                    finished_msg->kfs_num = current_kfs_num;
                    current_goal_handle->abort(finished_msg);
                } else {
                    feedback_msg->current_state  = 1;
                    feedback_msg->state_describe = "机械臂移动到指定的位置";
                    current_goal_handle->publish_feedback(feedback_msg);

                    finished_msg->reason  = "任务执行完成";
                    finished_msg->kfs_num = current_kfs_num;
                    current_goal_handle->succeed(finished_msg);
                }
            } else {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "机械臂路径规划失败，目标位置可能不可达";
                current_goal_handle->abort(finished_msg);
            }
        } else if (current_task_type == ROBOTIC_ARM_TASK_CATCH_TARGET) { // 将地上的KFS吸起来放到车上（或者只是吸起来）

            // 步骤一：添加目标KFS碰撞体
            auto temp_target=task_target_pos;
            RCLCPP_INFO(node->get_logger(), "转换后目标位姿: Pos(%lf,%lf,%lf), Rot(%lf,%lf,%lf,%lf)",
            task_target_pos.position.x, task_target_pos.position.y, task_target_pos.position.z,
            task_target_pos.orientation.w, task_target_pos.orientation.x, 
            task_target_pos.orientation.y, task_target_pos.orientation.z);
            
                // 为目标KFS添加碰撞体
            add_kfs_collision(temp_target, "target_kfs", move_group_interface->getPlanningFrame()); 

            auto current_pose = move_group_interface->getCurrentPose();


            // 步骤二：计算准备位置并规划移动
                // 规划路径到目标位置前,调用 calculate_prepare_pos 函数计算目标位置前方的准备位置
            geometry_msgs::msg::Pose grasp_pose;
            auto prepare_pos = calculate_prepare_pos(task_target_pos, 0.1, grasp_pose);  
            RCLCPP_INFO(node->get_logger(), "计算得到的准备位置: Pos(%lf,%lf,%lf), ORI(w:%f,x:%f,y:%f,z:%f)",
                prepare_pos.position.x, prepare_pos.position.y, prepare_pos.position.z,
                prepare_pos.orientation.w, prepare_pos.orientation.x, prepare_pos.orientation.y, prepare_pos.orientation.z);


            

               // 设置过渡位置
            do{
                move_group_interface->setStartStateToCurrentState();    
                move_group_interface->setNamedTarget("kfs4_interim_2_pos");

                success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                // count = 0;
                // while(success == false && count <=  MAX_COUNT_){
                //     success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                //     RCLCPP_WARN(node->get_logger(), "过渡位置规划失败，重行规划%d次", count+1);
                //     count ++ ;
                // }
                if(success){
                    RCLCPP_INFO(node->get_logger(), "规划到过渡位置成功");
                } else {
                    // finished_msg->reason = "机械臂无法到达过渡位置，路径规划失败";
                    // // abort  → 终止当前目标，状态设为"失败",并返回finished_msg
                    // current_goal_handle->abort(finished_msg);
                    RCLCPP_WARN(node->get_logger(), "过渡位置规划失败，尝试直接规划到目标");
                    // 如果过渡位置也失败，尝试直接规划（可能原来能走的路径被阻挡）
                }
            } while(move_group_interface->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS);
            auto intermediate_pose = move_group_interface->getCurrentPose().pose;
                    RCLCPP_INFO(node->get_logger(), "执行到过渡位置: Pos(%.3f,%.3f,%.3f)",
                        intermediate_pose.position.x, intermediate_pose.position.y, intermediate_pose.position.z);
            if(continue_flag)
                continue;

            //    // ==================== 尝试在规划准备位置之前删除 kfs 的碰撞 ===========================
            remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame());

            //     // ==================== 对规划位置的逆运动学检查 =========================
            // move_group_interface->setPoseTarget(grasp_pose);
            // success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            // if(success){
            //     RCLCPP_INFO(node->get_logger(), "抓取位置可达");
            // } else {
            //     RCLCPP_INFO(node->get_logger(), "抓取位置不可打");
            // }
            

                // 然后将这个准备位置设置为规划目标
            move_group_interface->setStartStateToCurrentState();
            move_group_interface->setPoseTarget(prepare_pos);            // 设置目标
                // plan 函数进行运动规划
            success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划从当前位置到目标位置的曲线
            count = 0;
            while(success == false && count <=  MAX_COUNT_){
                success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                RCLCPP_WARN(node->get_logger(), "准备位置规划失败，重行规划%d次", count+1);
                count ++ ;
            }
            if (!success) {
                RCLCPP_WARN(node->get_logger(), "准备位姿规划失败，尝试位置优先规划");
                move_group_interface->clearPoseTargets();
                move_group_interface->setStartStateToCurrentState();
                move_group_interface->setPositionTarget(
                    prepare_pos.position.x,
                    prepare_pos.position.y,
                    prepare_pos.position.z
                );
                success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                count = 0;
                while(success == false && count <=  MAX_COUNT_){
                    success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    RCLCPP_WARN(node->get_logger(), "准备位置(位置优先)规划失败，重行规划%d次", count+1);
                    count ++ ;
                }
            }
            if(success){
                RCLCPP_INFO(node->get_logger(), "准备位置规划成功");
            } else {
                RCLCPP_WARN(node->get_logger(), "准备位置规划失败");
            }

            // 步骤三：规划失败处理
            if (!success) {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "机械臂路径规划失败，目标位姿可能不可达(ROBOTIC_ARM_TASK_CATCH_TARGET)";
                // 调用 abort 方法终止当前 Action 目标
                current_goal_handle->abort(finished_msg);
                // 在抓取前删除KFS防止因碰撞检测无法连接
                remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame());   
                continue;
            }

            // 步骤四：执行到准备位置
                // 调用 execute 函数执行规划好的轨迹
            move_group_interface->execute(plan);
            RCLCPP_INFO(node->get_logger(), "执行到准备位置");
                // 更新反馈消息的状态编号为 1
            feedback_msg->current_state  = 1;
            feedback_msg->state_describe = "机械臂移动到待抓取位置";
            current_goal_handle->publish_feedback(feedback_msg);

            // 步骤五：删除碰撞体进行抓取
                // getPlanningFrame 获取运动规划器的id
            remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame());   //在抓取前删除KFS防止因碰撞检测无法连接
            // RCLCPP_INFO(node->get_logger(), "Debug-1");
























            robot_interfaces::msg::Moveit moveit_msg;
            moveit_msg.use_moveit = true;
            moveit_pub_->publish(moveit_msg);
            // 以 100Hz 刷新当前位姿和目标位姿，避免无限循环阻塞任务线程。
            rclcpp::Rate loop_rate(100.0); // 100Hz

            auto calculate_distance_to_target = [this](const geometry_msgs::msg::PoseStamped& pose) {
                const Eigen::Vector3d current_position(
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z
                );
                const Eigen::Vector3d target_position(
                    detected_target_pose_on_base_link_.position.x,
                    detected_target_pose_on_base_link_.position.y,
                    detected_target_pose_on_base_link_.position.z
                );
                return (target_position - current_position).norm();
            };

            double distance_ = calculate_distance_to_target(current_pose);






            while (rclcpp::ok() && distance_ > SWITCH_DISTANCE_THRESHOLD) { // 当末端与目标的距离大于阈值时持续进行视觉伺服调整
                if (cancle_current_task) {
                    RCLCPP_WARN(node->get_logger(), "视觉伺服被取消");
                    break;
                }

                geometry_msgs::msg::Pose vision_target_in_camera;
                bool has_vision_target = false;
                {
                    std::lock_guard<std::mutex> lock(vision_target_mutex_);
                    if (has_vision_target_) {
                        vision_target_in_camera = detected_target_pose_;
                        has_vision_target = true;
                    }
                }



                auto current_pose_now = move_group_interface->getCurrentPose();
                calculate_prepare_pos(detected_target_pose_on_base_link_, 0.05, grasp_pose); // 10Hz 更新目标位置

                geometry_msgs::msg::PoseStamped final_desired_position;
                final_desired_position.header.frame_id = "base_link";
                final_desired_position.pose = grasp_pose;
                final_desired_position.header.stamp = node->get_clock()->now();

                Eigen::Vector3d current_pose_eigen{
                    current_pose_now.pose.position.x,
                    current_pose_now.pose.position.y,
                    current_pose_now.pose.position.z
                };

                Eigen::Vector3d final_desired_position_eigen{
                    final_desired_position.pose.position.x,
                    final_desired_position.pose.position.y,
                    final_desired_position.pose.position.z
                };

                visual_servoing_handler_.TotalPackaing(current_pose_eigen, final_desired_position_eigen, current_pose_now, final_desired_position);
                

                current_pose = move_group_interface->getCurrentPose(); // 获取当前末端位姿

                distance_ = calculate_distance_to_target(current_pose);

                RCLCPP_INFO(node->get_logger(), "当前末端位置: (%.3f, %.3f, %.3f), 目标位置: (%.3f, %.3f, %.3f), 距离: %.3f",
                    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                    detected_target_pose_on_base_link_.position.x, detected_target_pose_on_base_link_.position.y, detected_target_pose_on_base_link_.position.z,
                    distance_); 
                
                loop_rate.sleep();
            }
            
            moveit_msg.use_moveit = false;
            moveit_pub_->publish(moveit_msg);



















            // count = 0;
            // move_group_interface->setMaxVelocityScalingFactor(0.05);
            // move_group_interface->setMaxAccelerationScalingFactor(0.025);
            // move_group_interface->setStartStateToCurrentState();
            // move_group_interface->setPoseTarget(grasp_pose);
            // success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

            // while(success == false && count <= MAX_COUNT_){
            //         ++count;
            //         success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            //         RCLCPP_WARN(node->get_logger(), "位置规划失败，重新规划%d次", count);
            // }
            // if(success){
            //     RCLCPP_INFO(node->get_logger(), "抓取过程规划成功");
            // } else {
            //     RCLCPP_INFO(node->get_logger(), "抓取过程规划失败");
            // }

            // if (!success) {
            //     finished_msg->kfs_num = current_kfs_num;
            //     finished_msg->reason  = "机械臂路径规划失败，目标位姿可能不可达(ROBOTIC_ARM_TASK_CATCH_TARGET)";
            //     // 调用 abort 方法终止当前 Action 目标
            //     current_goal_handle->abort(finished_msg);
            //     // 在抓取前删除KFS防止因碰撞检测无法连接
            //     remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame());   
            //     continue;
            // }


            // move_group_interface->execute(plan);
            // RCLCPP_INFO(node->get_logger(), "执行抓取");
            // move_group_interface->setMaxAccelerationScalingFactor(ACCELERATION_SCALING);
            // move_group_interface->setMaxVelocityScalingFactor(VELOCITY_SCALING);























            // // 步骤六：设置笛卡尔路径点（从准备位姿沿表面法向量直线接近）
            //     // 创建包含准备位姿和抓取位姿的路点数组 way_points
            // std::vector<geometry_msgs::msg::Pose> way_points;
            // way_points.resize(2);
            // way_points[0] = prepare_pos; // 准备位姿（在表面外侧）grasp_pose
            // way_points[1] = grasp_pose;  // 抓取位姿（贴合表面）prepare_pos

            // // 步骤七：笛卡尔路径规划
            //     // RobotTrajectory 消息用于存储计算出的笛卡尔路径
            // moveit_msgs::msg::RobotTrajectory cart_trajectory;
            //     // 调用 computeCartesianPath 函数进行笛卡尔路径规划，该函数会计算从当前位姿沿着直线移动到目标路点的轨迹
            //     /*
            //     参数说明：第一个参数是路点数组，第二个参数 0.01 是路点之间的最大距离（单位：米），
            //     第三个参数 0.0 是跳转阈值，第四个参数存储计算出的轨迹，
            //     第五个参数 false 表示不使用参考框架。函数返回规划成功的比例（0.0 到 1.0），1.0 表示完全成功。
            //     */

            // // ============================================================================================

            // double fraction = 0.0; // move_group_interface->computeCartesianPath(way_points, 0.01, 0.0, cart_trajectory,false);
            // count = 0;

            // // RCLCPP_INFO(node->get_logger(), "Debug-2");
            // move_group_interface->setMaxAccelerationScalingFactor(0.07);
            // move_group_interface->setMaxVelocityScalingFactor(0.1);

            // fraction = move_group_interface->computeCartesianPath(way_points, 0.0001, 0.0, cart_trajectory, false);

            // if(fraction < 0.995f){
            //     do {
            //         RCLCPP_WARN(node->get_logger(), "笛卡尔路径规划失败(准备位置->抓取位置)，重试 %d/%d, 规划比例: %.6f", count+1, MAX_COUNT_, fraction);
            //         fraction = move_group_interface->computeCartesianPath(way_points, 0.01, 0.0, cart_trajectory, false);
            //         count++;
                    
            //     } while (fraction < 0.995f && count < MAX_COUNT_);

            //     // 分段长路经
            //     if(fraction < 0.995f)
            //     planLongPathSegmented(prepare_pos, grasp_pose, cart_trajectory, 0.01);
            
            // } else {
            //     // auto smoothed = trajectory_smoother_->applySCurveSmoothing(std::make_shared<robot_trajectory::RobotTrajectory>(
            //     //     move_group_interface->getRobotModel(), 
            //     //     move_group_interface->getName()
            //     // ));
            //     RCLCPP_INFO(node->get_logger(), "从准备位置到抓取位置的笛卡尔路径规划成功");
            // }

            // // 步骤八：笛卡尔规划失败处理
            // if (fraction < 0.995f)             // 如果轨迹生成失败
            // {
            //     finished_msg->kfs_num = current_kfs_num;
            //     finished_msg->reason  = "抓取时机械臂超出工作范围，抓取失败";
            //     // 调用 abort 终止目标
            //     current_goal_handle->abort(finished_msg);
            //     RCLCPP_WARN(node->get_logger(), "从准备位置到抓取位置的笛卡尔路径规划失败");
            //     continue;
            // } 

            // // 步骤十：等待气泵稳定并启动
            //     // 调用 sleep_for 让线程休眠 2 秒
            // std::this_thread::sleep_for(2s);
            
            // // 启动气泵
            std::vector<std::string> node_names = node->get_node_names();
            // if(std::find(node_names.begin(), node_names.end(), "/driver_node") != node_names.end()){
            //     set_air_pump(true);
            //     RCLCPP_INFO(node->get_logger(), "启动气泵成功");
            // } else {
            //     RCLCPP_WARN(node->get_logger(),"没有driver_node节点,不能启动气泵");
            // }

            // // 步骤九：执行笛卡尔路径并发布反馈
            // // RCLCPP_INFO(node->get_logger(), "Debug-3");

            // // // 将笛卡尔轨迹的时间拉长，以放慢从准备位姿到抓取位姿的执行速度
            // // // slow_down_factor > 1.0 会将轨迹总时间放大相应倍数，同时按比例缩小速度/加速度
            // // const double slow_down_factor = 3.0; // 倍速缩放因子（2.0 表示执行时间变为原来两倍）
            // // if (slow_down_factor > 1.0) {
            // //     RCLCPP_INFO(node->get_logger(), "放慢笛卡尔轨迹: slow_down_factor=%.2f", slow_down_factor);
            // //     for (auto &point : cart_trajectory.joint_trajectory.points) {
            // //         // joint_trajectory.points 存储的是一系列带有时间戳的路径点序列，每个点包含该时刻的关节位置、速度和加速度信息。
            // //         // 缩放 time_from_start
            // //         /*
            // //         首先将 sec（秒）和 nanosec（纳秒）合并成一个双精度浮点数 t，这样便于数学运算；
            // //         然后将 t 乘以 slow_down_factor 进行缩放；最后再将结果拆分回秒和纳秒两部分存回原结构体。
            // //         */
            // //         double t = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
            // //         t *= slow_down_factor;
            // //         int32_t sec = static_cast<int32_t>(std::floor(t));
            // //         uint32_t nsec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
            // //         point.time_from_start.sec = sec;
            // //         point.time_from_start.nanosec = nsec;

            // //         // 缩放速度和加速度（若存在）以匹配更长的执行时间
            // //         if (!point.velocities.empty()) {
            // //             for (auto &v : point.velocities)
            // //                 v = static_cast<double>(v) / slow_down_factor;
            // //         }
            // //         if (!point.accelerations.empty()) {
            // //             for (auto &a : point.accelerations)
            // //                 a = static_cast<double>(a) / slow_down_factor;
            // //         }
            // //     }
            // // }

            // // 在 execute 之前添加时间记录
            // auto start_time = std::chrono::high_resolution_clock::now();
            // move_group_interface->execute(cart_trajectory);
            // // 记录结束时间
            // auto end_time = std::chrono::high_resolution_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            // RCLCPP_INFO(node->get_logger(), "笛卡尔路径执行完成，实际耗时: %ld 毫秒", duration.count());



            // RCLCPP_INFO(node->get_logger(), "执行从准备位置到抓取位置的笛卡尔路径");
            // feedback_msg->current_state  = 2;
            // feedback_msg->state_describe = "机械臂到达吸取位置";
            // current_goal_handle->publish_feedback(feedback_msg);











            move_group_interface->setMaxAccelerationScalingFactor(ACCELERATION_SCALING);
            move_group_interface->setMaxVelocityScalingFactor(VELOCITY_SCALING);
            // RCLCPP_INFO(node->get_logger(), "Debug-4");
            // 步骤十：等待气泵稳定
                // 调用 sleep_for 让线程休眠 2 秒
            // std::this_thread::sleep_for(2s);
            // RCLCPP_INFO(node->get_logger(), "Debug-5");
            

            // 使用一个参数服务来启动气泵
            // std::vector<std::string> node_names = node->get_node_names();
            // if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
            //     RCLCPP_WARN(node->get_logger(),"没有driver_node节点,不能启动气泵");
            // }
            // else {
            //     param_client->set_parameters({rclcpp::Parameter("enable_air_pump", true)});
            // }
            // RCLCPP_INFO(node->get_logger(), "Debug-6");

            // 步骤十一：发布吸附启动反馈
            feedback_msg->current_state  = 3;
            feedback_msg->state_describe = "启动气泵吸取KFS";
            current_goal_handle->publish_feedback(feedback_msg);
            // RCLCPP_INFO(node->get_logger(), "Debug-7");

            // 步骤十二：添加附着碰撞体
                // 调用 add_attached_kfs_collision 函数将已吸附的 KFS 添加为机械臂末端的附着碰撞体
            add_attached_kfs_collision();
            // RCLCPP_INFO(node->get_logger(), "Debug-8");

            // 调用 setStartStateToCurrentState 将规划起点设置为机械臂当前位
            move_group_interface->setStartStateToCurrentState();

            // 步骤十三：根据KFS数量选择目标位置
            if (current_kfs_num == 0)                                   // 根据当前机器人上的情况设置目标
                move_group_interface->setNamedTarget("kfs1_touch_pos"); // 设置目标
            else if (current_kfs_num == 1)
                move_group_interface->setNamedTarget("kfs2_touch_pos");
            else
                move_group_interface->setNamedTarget("kfs3_hold_pos");
            // RCLCPP_INFO(node->get_logger(), "Debug-9");
            
            // 步骤十四：规划并执行到放置位置
            do {                
                success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                count = 0;
                while(success == false && count <=  MAX_COUNT_){
                    success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    RCLCPP_WARN(node->get_logger(), "放置位置规划失败，重行规划%d次", count+1);
                    count ++ ;
                }
                if (!success) {
                    finished_msg->kfs_num = current_kfs_num;
                    finished_msg->reason  = "机械臂无法到达放置KFS的位置，路径规划失败";
                    current_goal_handle->abort(finished_msg);
                    param_client->set_parameters({rclcpp::Parameter("enable_air_pump", false)});
                    remove_attached_kfs_collision();
                    continue_flag = true;
                    break;
                } else { 
                    RCLCPP_INFO(node->get_logger(), "放置 KFS 位置规划成功");
                }
            } while (move_group_interface->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS); // 如果执行失败那么尝试重新规划并执行
           
            RCLCPP_INFO(node->get_logger(), "机械臂到达放置KFS的位置");


            // 步骤十五：继续后续操作
            if (continue_flag)
                continue;
            // RCLCPP_INFO(node->get_logger(), "Debug-10");

            // 步骤十六：发布到达放置位置反馈
            feedback_msg->current_state  = 4;
            feedback_msg->state_describe = "机械臂到达放置KFS的位置";
            current_goal_handle->publish_feedback(feedback_msg);
            // RCLCPP_INFO(node->get_logger(), "Debug-11");

            // 步骤十七：检查是否需要手持KFS
            if (current_kfs_num
                == 2) // 如果当前机器人上有3个KFS，那么最后一个KFS只能用手拿着，因此在这里就判定成功然后退出，否则继续执行后面的关闭气泵，退出机械臂等操作
            {
                current_kfs_num       = current_kfs_num + 1;
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "成功完成机械臂的执行";
                current_goal_handle->succeed(finished_msg);
                // param_client->set_parameters({rclcpp::Parameter("enable_air_pump", false)});
                // remove_attached_kfs_collision("kfs", "link6");
                // RCLCPP_INFO(node->get_logger(), "Debug-12");
                continue;
            } 
            else {
                // RCLCPP_INFO(node->get_logger(), "Debug-13");
                if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
                    RCLCPP_WARN(node->get_logger(),"没有driver_node节点,不能关闭气泵");
                }
                else {
                    set_air_pump(false);
                } 
                // RCLCPP_INFO(node->get_logger(), "Debug-14");
                RCLCPP_INFO(node->get_logger(), "关闭气泵成功");
            }

            // if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
            //     RCLCPP_WARN(node->get_logger(),"没有driver_node节点,不能关闭气泵");
            // }
            // else {
            //     param_client->set_parameters({rclcpp::Parameter("enable_air_pump", false)});
            // }

            // 步骤十八：删除附着碰撞体
            
            remove_attached_kfs_collision(); // 将KFS从吸盘上移除(（防止机械臂回退时在一开始就碰到KFS导致规划失败）
            

            // if(current_kfs_num==0)
            //     add_kfs_collision(kfs1_pos, "kfs1", move_group_interface->getPlanningFrame());
            // else
            //     add_kfs_collision(kfs2_pos, "kfs2", move_group_interface->getPlanningFrame());

            // 步骤十九：返回空闲位置
            do {
                // setStartStateToCurrentState 将规划的起始状态设置为当前实时状态
                move_group_interface->setStartStateToCurrentState();
                move_group_interface->setNamedTarget("idel_pos");

                success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                count = 0;
                while(success == false && count <=  MAX_COUNT_){
                    success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    RCLCPP_WARN(node->get_logger(), "idel位置规划失败，重行规划%d次", count+1);
                    count ++ ;
                }
                if (!success) {
                    finished_msg->kfs_num = current_kfs_num;
                    finished_msg->reason  = "机械臂无法回到空闲位置，路径规划失败";
                    current_goal_handle->abort(finished_msg);
                    continue_flag = true;
                    break;
                } 
            } while (move_group_interface->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS);
            if (continue_flag)
                continue;

            // 步骤二十：发布到达空闲位置反馈
            feedback_msg->current_state  = 5;
            feedback_msg->state_describe = "机械臂到达空闲位置";
            current_goal_handle->publish_feedback(feedback_msg);

            // 步骤二十一：任务完成
            current_kfs_num       = current_kfs_num + 1;
            finished_msg->kfs_num = current_kfs_num;
            finished_msg->reason  = "成功抓取KFS并放置到机器人上";
            current_goal_handle->succeed(finished_msg);

        } else if (current_task_type == ROBOTIC_ARM_TASK_PLACE_TARGET) { // 将车上的KFS吸起来，然后放到某个坐标
            RCLCPP_INFO(node->get_logger(), "将KFS放到九宫格内");
            
            // 一、检查KFS可用性
            if (current_kfs_num == 0)                                    // 如果当前机器人上没有KFS，报告后等待下一个请求
            {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "无可用的KFS";
                // 调用 current_goal_handle->abort 方法将当前 Action 目标标记为终止（失败状态）
                current_goal_handle->abort(finished_msg);
                continue;

            // 二、判断KFS状态并选择处理方式
            } else if (current_kfs_num != 3) // 如果当前KFS是堆在机器人上的，那么使用机械臂取出来，如果是已经在手上拿着的，直接往目标位置放就可以了
            {
                std::string name_tag = "kfs1";
                if (current_kfs_num == 2)
                    name_tag = "kfs2";

            // 三、设置到达准备吸取位置
                move_group_interface->setNamedTarget(name_tag + "_detach_pos"); // 到达准备吸取KFS的位置

            // 四、规划到准备吸取位置

                do{
                    auto success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    count = 0;
                    while(success == false && count <=  MAX_COUNT_){
                        success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                        RCLCPP_WARN(node->get_logger(), "准备（touch）位置规划失败，重行规划%d次", count+1);
                        count ++ ;
                    }
                    if (success != true) {
                        finished_msg->kfs_num = current_kfs_num;
                        finished_msg->reason  = "到达准备吸取KFS的位置失败，可能不可达";
                        current_goal_handle->abort(finished_msg);
                        continue;
                    }
                } while (move_group_interface->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS);


            // 六、发布到达待吸取位置反馈
                feedback_msg->current_state  = 1;
                feedback_msg->state_describe = "机械臂到达待吸取位置";
                current_goal_handle->publish_feedback(feedback_msg);


            // 七、规划到吸取位置（带重试循环）
                // remove_kfs_collision(name_tag, move_group_interface->getPlanningFrame());
                do {
                    move_group_interface->setStartStateToCurrentState();
                    move_group_interface->setNamedTarget(name_tag + "_touch_pos"); // 到达吸取KFS的位置

                    success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    count = 0;
                    while(success == false && count <=  MAX_COUNT_){
                        success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                        RCLCPP_WARN(node->get_logger(), "touch位置规划失败，重行规划%d次", count+1);
                        count ++ ;
                    }
                    if (success != moveit::core::MoveItErrorCode::SUCCESS) {
                        finished_msg->kfs_num = current_kfs_num;
                        finished_msg->reason  = "到达吸取KFS的位置失败，可能不可达";
                        current_goal_handle->abort(finished_msg);
                        continue_flag = true;
                        break;
                    } else {
                            // 十、添加注释掉的气泵启动代码 在规划完
                        std::vector<std::string> node_names = node->get_node_names();
                        if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
                            RCLCPP_WARN(node->get_logger(),"没有driver_node节点,不能开启气泵");
                        }
                        else {
                            set_air_pump(true);
                        }
                    }

                } while (move_group_interface->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS);
                
            // 八、跳过失败任务的剩余代码
                if (continue_flag)
                    continue;

            // 九、发布到达吸取位置反馈
                feedback_msg->current_state  = 2;
                feedback_msg->state_describe = "机械臂到达吸取位置，启动气泵";
                current_goal_handle->publish_feedback(feedback_msg);

            // 十、添加注释掉的气泵启动代码
                // std::vector<std::string> node_names = node->get_node_names();
                //  if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
                //      RCLCPP_WARN(node->get_logger(),"没有driver_node节点,不能开启气泵");
                //  }
                //  else {
                //      param_client->set_parameters({rclcpp::Parameter("enable_air_pump", true)});
                //  }

            // 十一、添加附着碰撞体
                add_attached_kfs_collision();

            // 十二、等待气泵稳定
                std::this_thread::sleep_for(2s);
            }

            // 十三、规划到放置目标位置（带重试循环）
            do {
                move_group_interface->setStartStateToCurrentState();
                move_group_interface->setPoseTarget(task_target_pos); // 放置任务——要放置的坐标

                auto success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (!success) {
                    RCLCPP_WARN(node->get_logger(), "放置位姿规划失败，尝试位置优先规划");
                    move_group_interface->clearPoseTargets();
                    move_group_interface->setStartStateToCurrentState();
                    move_group_interface->setPositionTarget(
                        task_target_pos.position.x,
                        task_target_pos.position.y,
                        task_target_pos.position.z
                    );
                    success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                }
                count = 0;
                while(success == false && count <=  MAX_COUNT_){
                    success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    RCLCPP_WARN(node->get_logger(), "放置位置规划失败，重行规划%d次", count+1);
                    count ++ ;
                }
                if (success != true) {
                    finished_msg->kfs_num = current_kfs_num;
                    finished_msg->reason  = "到达放置KFS的位置失败，可能不可达";
                    current_goal_handle->abort(finished_msg);
                    continue_flag = true;
                    break;
                }
            } while (move_group_interface->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS); // 执行目标轨迹
            
            // 十四、跳过失败任务的剩余代码
            if (continue_flag)
                continue;

            // 发布到达准备放置位置反馈
            feedback_msg->current_state  = 3;
            feedback_msg->state_describe = "到达准备放置KFS的地方";
            current_goal_handle->publish_feedback(feedback_msg);

            // 设置笛卡尔路径点
            std::vector<geometry_msgs::msg::Pose> way_points;
            way_points.resize(2);
            way_points[0]   = task_target_pos; // 当前位姿
            auto temp       = task_target_pos;
            temp.position.x = temp.position.x + 0.4;
            way_points[1]   = temp;            // 最终抓取位姿为当前位姿+0.4m以便于将KFS放入格子

            // 笛卡尔路径规划
            moveit_msgs::msg::RobotTrajectory cart_trajectory;

            double fraction = move_group_interface->computeCartesianPath(way_points, 0.01, 0.0, cart_trajectory, false);
            
            count = 0;
            while(fraction < 0.995f && count < MAX_COUNT_){
                RCLCPP_WARN(node->get_logger(), "笛卡尔路径规划失败(放置KFS)，重试 %d/%d, 规划比例: %.6f", count+1, MAX_COUNT_, fraction);
                fraction = move_group_interface->computeCartesianPath(way_points, 0.01, 0.0, cart_trajectory, false);
                count++;
            }
            if (fraction < 0.995f)             // 如果轨迹生成失败
            {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "放置时机械臂超出工作范围，抓取失败";
                current_goal_handle->abort(finished_msg);
                continue;
            }

            // 执行笛卡尔轨迹
            move_group_interface->execute(cart_trajectory);

            // 发布放置完成反馈
            feedback_msg->current_state  = 4;
            feedback_msg->state_describe = "将KFS放入架子";
            current_goal_handle->publish_feedback(feedback_msg);

            // 关闭气泵
            std::vector<std::string> node_names = node->get_node_names();
            if (std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()) {
                RCLCPP_WARN(node->get_logger(), "没有driver_node节点,不能关闭气泵");
            } else {
                set_air_pump(false);
            }
            RCLCPP_INFO(node->get_logger(), "关闭气泵成功");

            // 删除附着碰撞体
            remove_attached_kfs_collision();

            // 规划返回路径
            temp          = way_points[0];
            way_points[0] = way_points[1];
            way_points[1] = temp;  // 交换起点和终点
            fraction      = move_group_interface->computeCartesianPath(way_points, 0.01, 0.0, cart_trajectory, false);
            
            count = 0;
            while(fraction < 0.995f && count < MAX_COUNT_){
                RCLCPP_WARN(node->get_logger(), "笛卡尔路径规划失败(返回轨迹)，重试 %d/%d, 规划比例: %.6f", count+1, MAX_COUNT_, fraction);
                fraction = move_group_interface->computeCartesianPath(way_points, 0.01, 0.0, cart_trajectory, false);
                count++;
            }
            if (fraction < 0.995f) // 如果轨迹生成失败
            {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "放置时机械臂超出工作范围，抓取失败";
                current_goal_handle->abort(finished_msg);
                continue;
            }

            // 执行返回轨迹
            move_group_interface->execute(cart_trajectory);

            // 发布收回机械臂反馈
            feedback_msg->current_state  = 5;
            feedback_msg->state_describe = "收回机械臂";
            current_goal_handle->publish_feedback(feedback_msg);

            // 更新KFS数量
            current_kfs_num--;

            // 返回空闲位置
            do {
                move_group_interface->setStartStateToCurrentState();
                move_group_interface->setNamedTarget("idel_pos");
                auto success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                count = 0;
                while(success == false && count <=  MAX_COUNT_){
                    success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                    RCLCPP_WARN(node->get_logger(), "idel位置规划失败，重行规划%d次", count+1);
                    count ++ ;
                }
                if (success != true) {
                    finished_msg->kfs_num = current_kfs_num;
                    finished_msg->reason  = "回到空闲位置失败";
                    current_goal_handle->abort(finished_msg);
                    continue_flag = true;
                    break;
                }
            } while (move_group_interface->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS);
            if (continue_flag)
                continue;

            // 任务完成
            finished_msg->kfs_num = current_kfs_num;
            finished_msg->reason  = "成功将KFS放到架子上";
            current_goal_handle->succeed(finished_msg);
        }
    }
    RCLCPP_INFO(node->get_logger(), "退出机械臂任务处理线程");
}






































void ArmHandleNode::visionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!rclcpp::ok()) {
        return;
    }

    if (!msg) {
        RCLCPP_WARN(node->get_logger(), "警告：接收到空的视觉消息");
        return;
    }

    geometry_msgs::msg::Pose pose_in_camera;
    bool candidate_valid = false;

    // 第一段短锁：读取/更新共享缓存（不做耗时TF）
    {
        std::lock_guard<std::mutex> lock(vision_target_mutex_);

        pose_in_camera = msg->pose;
        available_target_pose_ = pose_in_camera;
        candidate_valid = true;
    }

    if (!candidate_valid) {
        RCLCPP_WARN(node->get_logger(), "警告：视觉消息状态无效");
        return;
    }

    // 锁外做TF，避免阻塞读线程
    geometry_msgs::msg::Pose transformed_pose;
    try {
        // 若消息时间戳无效，则退化为当前时刻
        rclcpp::Time query_stamp = msg->header.stamp;
        if (query_stamp.nanoseconds() == 0) {
            query_stamp = node->now();
        }

        auto tf = camera_link0_tf_buffer->lookupTransform(
            "base_link",
            "camera_link",
            query_stamp,
            tf2::durationFromSec(0.02)
        );

        tf2::doTransform(pose_in_camera, transformed_pose, tf);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(node->get_logger(), "警告：TF变换失败: %s", ex.what());
        return;
    }

    // 四元数归一化（防止非单位四元数）
    tf2::Quaternion q(
        transformed_pose.orientation.x,
        transformed_pose.orientation.y,
        transformed_pose.orientation.z,
        transformed_pose.orientation.w
    );

    if (q.length2() > 1e-12) {
        q.normalize();
        transformed_pose.orientation.x = q.x();
        transformed_pose.orientation.y = q.y();
        transformed_pose.orientation.z = q.z();
        transformed_pose.orientation.w = q.w();
    }

    // 第二段短锁：一次性发布共享结果
    {
        std::lock_guard<std::mutex> lock(vision_target_mutex_);
        detected_target_pose_ = pose_in_camera;
        detected_target_pose_on_base_link_ = transformed_pose;
        has_vision_target_ = true;
    }

    RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        5000,
        "视觉目标更新(base_link): Pos(%.3f, %.3f, %.3f), Rot(%.3f, %.3f, %.3f, %.3f)",
        transformed_pose.position.x,
        transformed_pose.position.y,
        transformed_pose.position.z,
        transformed_pose.orientation.w,
        transformed_pose.orientation.x,
        transformed_pose.orientation.y,
        transformed_pose.orientation.z
    );
}




