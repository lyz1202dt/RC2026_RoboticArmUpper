#include "arm_handle_node.hpp"
#include "GraspOrientionController.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cassert>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <memory>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
// #include <qt5/QtGui/qvalidator.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_client.hpp>
#include <string>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>
#include <iostream>
#include <vector>

using namespace std::chrono_literals;

ArmHandleNode::ArmHandleNode(const rclcpp::Node::SharedPtr node) : node(node) {

    this->node = node;                                                                                         // 以依赖注入的方式，传入要管理的节点

    param_client      = std::make_shared<rclcpp::SyncParametersClient>(node, "driver_node");
    arm_task_thread   = std::make_unique<std::thread>(std::bind(&ArmHandleNode::arm_catch_task_handle, this)); // 创建机械臂任务执行线程
    arm_handle_server = rclcpp_action::create_server<robot_interfaces::action::Catch>(
        node, "robotic_task",                                                                                  // 创建动作服务-服务端
        std::bind(&ArmHandleNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ArmHandleNode::cancel_goal, this, std::placeholders::_1), 
        std::bind(&ArmHandleNode::handle_accepted, this, std::placeholders::_1)
    );

    // 坐标变换监听
    camera_link0_tf_buffer   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    camera_link0_tf_listener = std::make_shared<tf2_ros::TransformListener>(*camera_link0_tf_buffer);
    move_group_interface     = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robotic_arm");
    psi                      = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

    mark_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("debug_mark", 10);

    node->create_wall_timer(100ms, [this]() {
        if(!is_running_arm_task)    //如果此时没有进行机械臂抓取，那么立即返回
            return;
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

    move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, "robotic_arm");

    // 配置基本参数
    move_group_interface->setPlanningTime(10.0); // 规划时间设置

    move_group_interface->setMaxVelocityScalingFactor(VELOCITY_SCALING); // 最大速度缩放因子

    move_group_interface->setMaxAccelerationScalingFactor(ACCELERATION_SCALING); // 最大加速度缩放因子
                            

    // 初始化TF2
    tf_buffer_= std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}




ArmHandleNode::~ArmHandleNode() {
    task_mutex_.lock(); // 加锁，保护共享变量，其他线程暂时不能访问has_new_task_
    has_new_task_ = true;    // 置为 true，让线程退出循环时能通过条件判断。有新任务
    task_mutex_.unlock(); // 解锁
    task_cv_.notify_all(); // 唤醒等待条件变量的线程

    if (arm_task_thread->joinable()) // 检测线程是否还在运行
        arm_task_thread->join(); // 等待线程结束
}

rclcpp_action::GoalResponse
    ArmHandleNode::handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const robot_interfaces::action::Catch::Goal> goal) {
    (void)uuid;
    if (is_running_arm_task) // 如果正在运行机械臂动作，那么拒绝新的请求
        return rclcpp_action::GoalResponse::REJECT;

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

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
    ArmHandleNode::cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {
    (void)goal_handle;
    cancle_current_task = true;
    is_running_arm_task = false;

    // 删除之前添加的碰撞体，避免残留的虚拟碰撞体影响后续任务规划。
    remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame()); 
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmHandleNode::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {
    current_goal_handle = goal_handle;
    // 设置取消标志，通知执行循环终止当前任务
    is_running_arm_task = true;

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
        psi->applyCollisionObject(collision_object);                                     // 应用障碍物
    }

    RCLCPP_INFO(node->get_logger(), "机械臂任务处理线程启动完成，等待任务请求");

    while (rclcpp::ok()) {
        continue_flag = false;

        is_running_arm_task = false;
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
            move_group_interface->setGoalJointTolerance(0.13);
            move_group_interface->setGoalPositionTolerance(0.06);
            move_group_interface->setGoalOrientationTolerance(0.2);

            // 调用 setPlanningTime 方法设置规划器的最大规划时间为 5.0 秒
            move_group_interface->setPlanningTime(5.0);

            // 设置采样次数（最多尝试次数）
            move_group_interface->setNumPlanningAttempts(10);

            first_run = false;
        }

        if (!rclcpp::ok()) {
            break;
        }

        if (current_task_type == ROBOTIC_ARM_TASK_MOVE)           // 要求机械臂移动到某个位姿（因为移动动作在一个周期内完成，所以不再需要执行）
        {
            move_group_interface->setPoseTarget(task_target_pos); // 设置目标

            // 计算轨迹，存入plan并判断规划是否成功
            bool success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划当前位置到目标位置的曲线
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
                move_group_interface->setNamedTarget("kfs4_interim_1_pos");
                
                success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if(success){
                    RCLCPP_INFO(node->get_logger(), "规划到过渡位置成功");
                } else {
                    finished_msg->reason = "机械臂无法到达过渡位置，路径规划失败";
                    // abort  → 终止当前目标，状态设为"失败",并返回finished_msg
                    current_goal_handle->abort(finished_msg);
                    RCLCPP_WARN(node->get_logger(), "过渡位置规划失败，尝试直接规划到目标");
                    // 如果过渡位置也失败，尝试直接规划（可能原来能走的路径被阻挡）
                }
            } while(move_group_interface->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS);
            auto intermediate_pose = move_group_interface->getCurrentPose().pose;
                    RCLCPP_INFO(node->get_logger(), "执行到过渡位置: Pos(%.3f,%.3f,%.3f)",
                        intermediate_pose.position.x, intermediate_pose.position.y, intermediate_pose.position.z);
            if(continue_flag)
                continue;

                // ==================== 尝试在规划准备位置之前删除 kfs 的碰撞 ===========================
            remove_kfs_collision("target_kfs", move_group_interface->getPlannerId());

            //     // ==================== 对规划位置的逆运动学检查 =========================
            // move_group_interface->setPoseTarget(grasp_pose);
            // success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            // if(success){
            //     RCLCPP_INFO(node->get_logger(), "抓取位置可达");
            // } else {
            //     RCLCPP_INFO(node->get_logger(), "抓取位置不可打");
            // }
            

                // 然后将这个准备位置设置为规划目标
            move_group_interface->setPoseTarget(prepare_pos);            // 设置目标
                // plan 函数进行运动规划
            success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划从当前位置到目标位置的曲线
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

            // 步骤六：设置笛卡尔路径点（从准备位姿沿表面法向量直线接近）
                // 创建包含准备位姿和抓取位姿的路点数组 way_points
            std::vector<geometry_msgs::msg::Pose> way_points;
            way_points.resize(2);
            way_points[0] = prepare_pos; // 准备位姿（在表面外侧）grasp_pose
            way_points[1] = grasp_pose;  // 抓取位姿（贴合表面）prepare_pos

            // 步骤七：笛卡尔路径规划
                // RobotTrajectory 消息用于存储计算出的笛卡尔路径
            moveit_msgs::msg::RobotTrajectory cart_trajectory;
                // 调用 computeCartesianPath 函数进行笛卡尔路径规划，该函数会计算从当前位姿沿着直线移动到目标路点的轨迹
                /*
                参数说明：第一个参数是路点数组，第二个参数 0.01 是路点之间的最大距离（单位：米），
                第三个参数 0.0 是跳转阈值，第四个参数存储计算出的轨迹，
                第五个参数 false 表示不使用参考框架。函数返回规划成功的比例（0.0 到 1.0），1.0 表示完全成功。
                */
                
                // 调整速度因子以达到减慢速度的目的
            move_group_interface->setMaxVelocityScalingFactor(0.00001);
            double fraction = move_group_interface->computeCartesianPath(way_points, 0.01, 0.0, cart_trajectory,false);
            move_group_interface->setMaxVelocityScalingFactor(1);

            // 步骤八：笛卡尔规划失败处理
            if (fraction < 0.995f)             // 如果轨迹生成失败
            {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "抓取时机械臂超出工作范围，抓取失败";
                // 调用 abort 终止目标
                current_goal_handle->abort(finished_msg);
                RCLCPP_INFO(node->get_logger(), "从准备位置到抓取位置的笛卡尔路径规划失败");
                continue;
            }

            // 步骤九：执行笛卡尔路径并发布反馈
            move_group_interface->execute(cart_trajectory);
            RCLCPP_INFO(node->get_logger(), "执行从准备位置到抓取位置的笛卡尔路径");
            feedback_msg->current_state  = 2;
            feedback_msg->state_describe = "机械臂到达吸取位置";
            current_goal_handle->publish_feedback(feedback_msg);

            // 步骤十：等待气泵稳定
                // 调用 sleep_for 让线程休眠 2 秒
            std::this_thread::sleep_for(2s);


            // 使用一个参数服务来启动气泵
            // std::vector<std::string> node_names = node->get_node_names();
            // if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
            //     RCLCPP_WARN(node->get_logger(),"没有driver_node节点,不能启动气泵");
            // }
            // else {
            //     param_client->set_parameters({rclcpp::Parameter("enable_air_pump", true)});
            // }

            // 步骤十一：发布吸附启动反馈
            feedback_msg->current_state  = 3;
            feedback_msg->state_describe = "启动气泵吸取KFS";
            current_goal_handle->publish_feedback(feedback_msg);

            // 步骤十二：添加附着碰撞体
                // 调用 add_attached_kfs_collision 函数将已吸附的 KFS 添加为机械臂末端的附着碰撞体
            add_attached_kfs_collision();

            // 步骤十三：根据KFS数量选择目标位置
            if (current_kfs_num == 0)                                   // 根据当前机器人上的情况设置目标
                move_group_interface->setNamedTarget("kfs1_touch_pos"); // 设置目标
            else if (current_kfs_num == 1)
                move_group_interface->setNamedTarget("kfs2_touch_pos");
            else
                move_group_interface->setNamedTarget("kfs3_hold_pos");

            // 步骤十四：规划并执行到放置位置
            do {
                // 调用 setStartStateToCurrentState 将规划起点设置为机械臂当前位置
                move_group_interface->setStartStateToCurrentState();
                success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
                if (!success) {
                    finished_msg->kfs_num = current_kfs_num;
                    finished_msg->reason  = "机械臂无法到达放置KFS的位置，路径规划失败";
                    current_goal_handle->abort(finished_msg);
                    param_client->set_parameters({rclcpp::Parameter("enable_air_pump", false)});
                    remove_attached_kfs_collision();
                    continue_flag = true;
                    break;
                }
            } while (move_group_interface->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS); // 如果执行失败那么尝试重新规划并执行
            
            // 步骤十五：继续后续操作
            if (continue_flag)
                continue;

            // 步骤十六：发布到达放置位置反馈
            feedback_msg->current_state  = 4;
            feedback_msg->state_describe = "机械臂到达放置KFS的位置";
            current_goal_handle->publish_feedback(feedback_msg);

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
                continue;
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
                auto success = move_group_interface->plan(plan);
                if (success != moveit::core::MoveItErrorCode::SUCCESS) {
                    finished_msg->kfs_num = current_kfs_num;
                    finished_msg->reason  = "到达准备吸取KFS的位置失败，可能不可达";
                    current_goal_handle->abort(finished_msg);
                    continue;
                }

            // 五、执行到准备吸取位置
                success = move_group_interface->execute(plan);

            // 六、发布到达待吸取位置反馈
                feedback_msg->current_state  = 1;
                feedback_msg->state_describe = "机械臂到达待吸取位置";
                current_goal_handle->publish_feedback(feedback_msg);


            // 七、规划到吸取位置（带重试循环）
                // remove_kfs_collision(name_tag, move_group_interface->getPlanningFrame());
                do {
                    move_group_interface->setStartStateToCurrentState();
                    move_group_interface->setNamedTarget(name_tag + "_touch_pos"); // 到达吸取KFS的位置
                    success = move_group_interface->plan(plan);
                    if (success != moveit::core::MoveItErrorCode::SUCCESS) {
                        finished_msg->kfs_num = current_kfs_num;
                        finished_msg->reason  = "到达吸取KFS的位置失败，可能不可达";
                        current_goal_handle->abort(finished_msg);
                        continue_flag = true;
                        break;
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
                auto success = move_group_interface->plan(plan);
                if (success != moveit::core::MoveItErrorCode::SUCCESS) {
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
            // std::vector<std::string> node_names = node->get_node_names();
            // if (std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()) {
            //     RCLCPP_WARN(node->get_logger(), "没有driver_node节点,不能关闭气泵");
            // } else {
            //     param_client->set_parameters({rclcpp::Parameter("enable_air_pump", false)});
            // }

            // 删除附着碰撞体
            remove_attached_kfs_collision();

            // 规划返回路径
            temp          = way_points[0];
            way_points[0] = way_points[1];
            way_points[1] = temp;  // 交换起点和终点
            fraction      = move_group_interface->computeCartesianPath(way_points, 0.01, 0.0, cart_trajectory, false);
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
                auto success = move_group_interface->plan(plan);
                if (success != moveit::core::MoveItErrorCode::SUCCESS) {
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
    geometry_msgs::msg::Pose &grasp_pose, 
    ApproachMode mode) 
{
    RCLCPP_INFO(node->get_logger(), 
        "传入的box_pos: POS(%f, %f, %f), ORI(w:%f, x:%f, y:%f, z:%f)", 
        box_pos.position.x, box_pos.position.y, box_pos.position.z,
        box_pos.orientation.w, box_pos.orientation.x, 
        box_pos.orientation.y, box_pos.orientation.z);

    // ========== 步骤1：提取物体位置和方向 ==========
    Eigen::Vector3d object_center(box_pos.position.x, box_pos.position.y, box_pos.position.z);
    Eigen::Quaterniond q(box_pos.orientation.w, box_pos.orientation.x, 
                         box_pos.orientation.y, box_pos.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();

    // ========== 步骤2：根据模式选择抓取策略 ==========
    int select = 0;
    bool is_side_mode = false;
    
    if (mode == ApproachMode::AUTO) {
        double horiz_dist = std::hypot(box_pos.position.x, box_pos.position.y);
        if (horiz_dist < SWITCH_DISTANCE_THRESHOLD) {
            mode = ApproachMode::TOP;
            select = 1;
            is_side_mode = false;
        } else {
            mode = ApproachMode::SIDE_ROBOT;
            select = 2;
            is_side_mode = true;
        }
    } else {
        is_side_mode = (mode == ApproachMode::SIDE_ROBOT);
    }
    RCLCPP_INFO(node->get_logger(), "选择的抓取模式: %s (select=%d)", 
                is_side_mode ? "SIDE_ROBOT" : "TOP", select);

    // ========== 步骤3：计算接近方向向量 ==========
    Eigen::Vector3d approach_normal;
    bool use_side = is_side_mode;

    if (!use_side) {
        // TOP抓取：使用物体局部Z轴
        approach_normal = R * Eigen::Vector3d(0.0, 0.0, 1.0);
        RCLCPP_INFO(node->get_logger(), "TOP抓取: 法向量 = (%f, %f, %f)",
                    approach_normal.x(), approach_normal.y(), approach_normal.z());
    } else {
        // SIDE_ROBOT抓取：水平指向机器人
        Eigen::Vector3d horizontal_dir(box_pos.position.x, box_pos.position.y, 0.0);
        if (horizontal_dir.norm() < 1e-6) {
            RCLCPP_WARN(node->get_logger(), "无法确定侧面方向，回退到上方抓取");
            approach_normal = R * Eigen::Vector3d(0.0, 0.0, 1.0);
            use_side = false;
        } else {
            horizontal_dir.normalize();
            approach_normal = horizontal_dir;
            RCLCPP_INFO(node->get_logger(), "SIDE抓取: 法向量(指向机器人) = (%f, %f, %f)",
                        approach_normal.x(), approach_normal.y(), approach_normal.z());
        }
    }

    // ========== 步骤4：计算物体表面中心 ==========
    const double object_half_size = 0.35;
    Eigen::Vector3d surface_center;

    if (!use_side) {
        surface_center = object_center + object_half_size * approach_normal;
    } else {
        surface_center = object_center - object_half_size * approach_normal;
    }
    RCLCPP_INFO(node->get_logger(), "表面中心 surface_center = (%f, %f, %f)",
                surface_center.x(), surface_center.y(), surface_center.z());

    // ========== 步骤5：计算末端执行器姿态（先计算姿态，再计算位置） ==========
    Eigen::Vector3d eef_z_axis;

    if (!use_side) {
        // TOP抓取
        if (object_center.dot(approach_normal) >= 0.0) {
            eef_z_axis = approach_normal;
        } else {
            eef_z_axis = -approach_normal;
        }
    } else {
        // SIDE抓取：Z轴指向物体表面
        eef_z_axis = -approach_normal;
    }
    RCLCPP_INFO(node->get_logger(), "末端Z轴 eef_z_axis = (%f, %f, %f)",
                eef_z_axis.x(), eef_z_axis.y(), eef_z_axis.z());

    // 【关键修正】X轴使用全局固定方向，保证姿态连续性
    Eigen::Vector3d global_x(1.0, 0.0, 0.0);
    Eigen::Vector3d eef_x_axis = eef_z_axis.cross(global_x);
    
    if (eef_x_axis.norm() < 1e-6) {
        Eigen::Vector3d global_y(0.0, 1.0, 0.0);
        eef_x_axis = eef_z_axis.cross(global_y);
    }
    eef_x_axis.normalize();
    RCLCPP_INFO(node->get_logger(), "末端X轴 eef_x_axis = (%f, %f, %f)",
                eef_x_axis.x(), eef_x_axis.y(), eef_x_axis.z());

    Eigen::Vector3d eef_y_axis = eef_z_axis.cross(eef_x_axis);
    eef_y_axis.normalize();
    RCLCPP_INFO(node->get_logger(), "末端Y轴 eef_y_axis = (%f, %f, %f)",
                eef_y_axis.x(), eef_y_axis.y(), eef_y_axis.z());

    // ========== 步骤6：构造旋转矩阵和四元数 ==========
    Eigen::Matrix3d R_eef;
    R_eef.col(0) = eef_x_axis;
    R_eef.col(1) = eef_y_axis;
    R_eef.col(2) = eef_z_axis;
    Eigen::Quaterniond q_eef(R_eef);

    // ========== 步骤7：计算位置（使用相同的姿态） ==========
    const double tool_offset = 0.05;
    Eigen::Vector3d grasp_position;
    Eigen::Vector3d prepare_position;

    if (!use_side) {
        // TOP抓取
        grasp_position = surface_center - tool_offset * approach_normal;
        prepare_position = grasp_position - approach_distance * approach_normal;
    } else {
        // SIDE抓取
        grasp_position = surface_center + tool_offset * approach_normal;
        prepare_position = grasp_position + approach_distance * approach_normal;
    }
    RCLCPP_INFO(node->get_logger(), "抓取位置 grasp_position = (%f, %f, %f)",
                grasp_position.x(), grasp_position.y(), grasp_position.z());
    RCLCPP_INFO(node->get_logger(), "准备位置 prepare_position = (%f, %f, %f)",
                prepare_position.x(), prepare_position.y(), prepare_position.z());

    // ========== 步骤8：构造返回的Pose消息 ==========
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

    RCLCPP_INFO(node->get_logger(), "========== 计算结果汇总 ==========");
    RCLCPP_INFO(node->get_logger(), "抓取位姿: POS(%f, %f, %f), ORI(%f, %f, %f, %f)",
                grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z,
                grasp_pose.orientation.w, grasp_pose.orientation.x,
                grasp_pose.orientation.y, grasp_pose.orientation.z);
    RCLCPP_INFO(node->get_logger(), "准备位姿: POS(%f, %f, %f), ORI(%f, %f, %f, %f)",
                result.position.x, result.position.y, result.position.z,
                result.orientation.w, result.orientation.x,
                result.orientation.y, result.orientation.z);

    return result;
}



/**
    @brief 根据父坐标系和期望位置，生成一个放置在某个位置的KFS
 */
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



