#include "arm_handle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cassert>
#include <memory>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_client.hpp>
#include <string>
#include <thread>

using namespace std::chrono_literals;

ArmHandleNode::ArmHandleNode(const rclcpp::Node::SharedPtr node) {

    this->node = node;                                                                                         // 以依赖注入的方式，传入要管理的节点

    param_client      = std::make_shared<rclcpp::SyncParametersClient>(node, "driver_node");
    arm_task_thread   = std::make_unique<std::thread>(std::bind(&ArmHandleNode::arm_catch_task_handle, this)); // 创建机械臂任务执行线程
    arm_handle_server = rclcpp_action::create_server<robot_interfaces::action::Catch>(
        node, "robotic_task",                                                                                  // 创建动作服务-服务端
        std::bind(&ArmHandleNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&ArmHandleNode::cancel_goal, this, std::placeholders::_1), std::bind(&ArmHandleNode::handle_accepted, this, std::placeholders::_1)
    );
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

        marker.type   = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // 球心位置
        marker.pose = task_target_pos;

        // 姿态（球体无关，但必须合法）
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
}


ArmHandleNode::~ArmHandleNode() {
    task_mutex_.lock();
    has_new_task_ = true;    // 置为 true，让线程退出循环时能通过条件判断
    task_mutex_.unlock();
    task_cv_.notify_all();

    if (arm_task_thread->joinable())
        arm_task_thread->join();
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
    tf2::doTransform(goal->target_pose, task_target_pos, camera_link0_tf);

    // task_target_pos   = goal->target_pose;
    // TODO:姿态一定是大约朝前的，所以这里定死姿态
    RCLCPP_INFO(node->get_logger(),"实际位置为(%lf,%lf,%lf)",task_target_pos.position.x,task_target_pos.position.y,task_target_pos.position.z);

    task_target_pos.orientation.w = 0.004481;
    task_target_pos.orientation.x = 0.708322;
    task_target_pos.orientation.y = -0.004257;
    task_target_pos.orientation.z = -0.705862;
    auto qin=task_target_pos.orientation;
    tf2::Quaternion q(qin.x, qin.y, qin.z, qin.w);
    q.normalize();
    task_target_pos.orientation.w = q.w();
    task_target_pos.orientation.x = q.x();
    task_target_pos.orientation.y = q.y();
    task_target_pos.orientation.z = q.z();


    current_task_type = goal->action_type;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
    ArmHandleNode::cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {
    (void)goal_handle;
    cancle_current_task = true;
    is_running_arm_task = false;
    remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame()); //删除之前添加的碰撞体
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmHandleNode::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {
    current_goal_handle = goal_handle;
    is_running_arm_task = true;
    cancle_current_task = false;

    task_mutex_.lock(); // 通知需要执行任务
    has_new_task_ = true;
    task_mutex_.unlock();
    task_cv_.notify_one();
    // TODO:给出信号量开始执行
}

// 机械臂
void ArmHandleNode::arm_catch_task_handle() {
    RCLCPP_INFO(node->get_logger(), "进入机械臂任务处理线程");
    bool first_run = true;
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
        bool ok       = task_cv_.wait_for(lock, 5s, [this]() { return has_new_task_; }); // 等待直到lambda表达式返回真
        has_new_task_ = false;
        lock.unlock();

        if (!ok) {
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
            // move_group_interface->setGoalJointTolerance(0.13);
            // move_group_interface->setGoalPositionTolerance(0.06);
            // move_group_interface->setGoalOrientationTolerance(0.2);
            move_group_interface->setPlanningTime(5.0);
            first_run = false;
        }

        if (!rclcpp::ok()) {
            break;
        }

        if (current_task_type == ROBOTIC_ARM_TASK_MOVE)           // 要求机械臂移动到某个位姿（因为移动动作在一个周期内完成，所以不再需要执行）
        {
            move_group_interface->setPoseTarget(task_target_pos); // 设置目标
            bool success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划当前位置到目标位置的曲线
            if (success) {                                                                               // 阻塞函数，发送轨迹
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

            auto temp_target=task_target_pos;
            temp_target.position.x=temp_target.position.x+0.1+0.2;
            add_kfs_collision(temp_target, "target_kfs", move_group_interface->getPlanningFrame()); // 为目标KFS添加碰撞体
            auto prepare_pos = calculate_prepare_pos(task_target_pos);   // 规划路径到目标位置前
            move_group_interface->setPoseTarget(prepare_pos);            // 设置目标
            bool success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划从当前位置到目标位置的曲线
            if (!success) {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "机械臂路径规划失败，目标位姿可能不可达";
                current_goal_handle->abort(finished_msg);
                remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame());   //在抓取前删除KFS防止因碰撞检测无法连接
                continue;
            }
            move_group_interface->execute(plan);
            feedback_msg->current_state  = 1;
            feedback_msg->state_describe = "机械臂移动到待抓取位置";
            current_goal_handle->publish_feedback(feedback_msg);

            remove_kfs_collision("target_kfs", move_group_interface->getPlanningFrame());   //在抓取前删除KFS防止因碰撞检测无法连接

            auto start_pose = move_group_interface->getCurrentPose().pose;
            std::vector<geometry_msgs::msg::Pose> way_points;
            way_points.resize(2);
            way_points[0]   = task_target_pos; // 当前位姿
            auto temp       = way_points[0];
            temp.position.x = temp.position.x + 0.1;
            way_points[1]   = temp;            // 最终抓取位姿
            moveit_msgs::msg::RobotTrajectory cart_trajectory;
            double fraction = move_group_interface->computeCartesianPath(way_points, 0.01, 0.0, cart_trajectory,false);
            if (fraction < 0.995f)             // 如果轨迹生成失败
            {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "抓取时机械臂超出工作范围，抓取失败";
                current_goal_handle->abort(finished_msg);
                continue;
            }
            move_group_interface->execute(cart_trajectory);
            feedback_msg->current_state  = 2;
            feedback_msg->state_describe = "机械臂到达吸取位置";
            current_goal_handle->publish_feedback(feedback_msg);

            std::this_thread::sleep_for(2s);
            // 使用一个参数服务来启动气泵
            // std::vector<std::string> node_names = node->get_node_names();
            // if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
            //     RCLCPP_WARN(node->get_logger(),"没有driver_node节点,不能启动气泵");
            // }
            // else {
            //     param_client->set_parameters({rclcpp::Parameter("enable_air_pump", true)});
            // }

            feedback_msg->current_state  = 3;
            feedback_msg->state_describe = "启动气泵吸取KFS";
            current_goal_handle->publish_feedback(feedback_msg);
            // 为机械臂末端连接一个KFS用于碰撞计算
            add_attached_kfs_collision();

            if (current_kfs_num == 0)                                   // 根据当前机器人上的情况设置目标
                move_group_interface->setNamedTarget("kfs1_touch_pos"); // 设置目标
            else if (current_kfs_num == 1)
                move_group_interface->setNamedTarget("kfs2_touch_pos");
            else
                move_group_interface->setNamedTarget("kfs3_hold_pos");


            do {
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
            if (continue_flag)
                continue;

            feedback_msg->current_state  = 4;
            feedback_msg->state_describe = "机械臂到达放置KFS的位置";
            current_goal_handle->publish_feedback(feedback_msg);
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

            remove_attached_kfs_collision(); // 将KFS从吸盘上移除(（防止机械臂回退时在一开始就碰到KFS导致规划失败）


            // if(current_kfs_num==0)
            //     add_kfs_collision(kfs1_pos, "kfs1", move_group_interface->getPlanningFrame());
            // else
            //     add_kfs_collision(kfs2_pos, "kfs2", move_group_interface->getPlanningFrame());

            do {
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

            feedback_msg->current_state  = 5;
            feedback_msg->state_describe = "机械臂到达空闲位置";
            current_goal_handle->publish_feedback(feedback_msg);

            current_kfs_num       = current_kfs_num + 1;
            finished_msg->kfs_num = current_kfs_num;
            finished_msg->reason  = "成功抓取KFS并放置到机器人上";
            current_goal_handle->succeed(finished_msg);
        } else if (current_task_type == ROBOTIC_ARM_TASK_PLACE_TARGET) { // 将车上的KFS吸起来，然后放到某个坐标
            if (current_kfs_num == 0)                                    // 如果当前机器人上没有KFS，报告后等待下一个请求
            {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "无可用的KFS";
                current_goal_handle->abort(finished_msg);
                continue;
            } else if (current_kfs_num != 3) // 如果当前KFS是堆在机器人上的，那么使用机械臂取出来，如果是已经在手上拿着的，直接往目标位置放就可以了
            {
                std::string name_tag = "kfs1";
                if (current_kfs_num == 2)
                    name_tag = "kfs2";


                move_group_interface->setNamedTarget(name_tag + "_detach_pos"); // 到达准备吸取KFS的位置

                auto success = move_group_interface->plan(plan);
                if (success != moveit::core::MoveItErrorCode::SUCCESS) {
                    finished_msg->kfs_num = current_kfs_num;
                    finished_msg->reason  = "到达准备吸取KFS的位置失败，可能不可达";
                    current_goal_handle->abort(finished_msg);
                    continue;
                }
                success = move_group_interface->execute(plan);

                feedback_msg->current_state  = 1;
                feedback_msg->state_describe = "机械臂到达待吸取位置";
                current_goal_handle->publish_feedback(feedback_msg);


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
                if (continue_flag)
                    continue;


                feedback_msg->current_state  = 2;
                feedback_msg->state_describe = "机械臂到达吸取位置，启动气泵";
                current_goal_handle->publish_feedback(feedback_msg);

                // std::vector<std::string> node_names = node->get_node_names();
                //  if(std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()){
                //      RCLCPP_WARN(node->get_logger(),"没有driver_node节点,不能开启气泵");
                //  }
                //  else {
                //      param_client->set_parameters({rclcpp::Parameter("enable_air_pump", true)});
                //  }
                add_attached_kfs_collision();

                std::this_thread::sleep_for(2s);
            }
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
            if (continue_flag)
                continue;

            feedback_msg->current_state  = 3;
            feedback_msg->state_describe = "到达准备放置KFS的地方";
            current_goal_handle->publish_feedback(feedback_msg);

            std::vector<geometry_msgs::msg::Pose> way_points;
            way_points.resize(2);
            way_points[0]   = task_target_pos; // 当前位姿
            auto temp       = task_target_pos;
            temp.position.x = temp.position.x + 0.4;
            way_points[1]   = temp;            // 最终抓取位姿为当前位姿+0.4m以便于将KFS放入格子
            moveit_msgs::msg::RobotTrajectory cart_trajectory;
            double fraction = move_group_interface->computeCartesianPath(way_points, 0.01, 0.0, cart_trajectory, false);
            if (fraction < 0.995f)             // 如果轨迹生成失败
            {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "放置时机械臂超出工作范围，抓取失败";
                current_goal_handle->abort(finished_msg);
                continue;
            }
            move_group_interface->execute(cart_trajectory);


            feedback_msg->current_state  = 4;
            feedback_msg->state_describe = "将KFS放入架子";
            current_goal_handle->publish_feedback(feedback_msg);

            // std::vector<std::string> node_names = node->get_node_names();
            // if (std::find(node_names.begin(), node_names.end(), "/driver_node") == node_names.end()) {
            //     RCLCPP_WARN(node->get_logger(), "没有driver_node节点,不能关闭气泵");
            // } else {
            //     param_client->set_parameters({rclcpp::Parameter("enable_air_pump", false)});
            // }
            remove_attached_kfs_collision();

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
            move_group_interface->execute(cart_trajectory);

            feedback_msg->current_state  = 5;
            feedback_msg->state_describe = "收回机械臂";
            current_goal_handle->publish_feedback(feedback_msg);

            current_kfs_num--;

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

geometry_msgs::msg::Pose ArmHandleNode::calculate_prepare_pos(const geometry_msgs::msg::Pose& box_pos) {
    Eigen::Vector3d pos(box_pos.position.x, box_pos.position.y, box_pos.position.z);
    Eigen::Quaterniond q(box_pos.orientation.w, box_pos.orientation.x, box_pos.orientation.y,
                         box_pos.orientation.z);            // 构造物体的位置和平移矩阵
    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d n = R * Eigen::Vector3d(0.0, 0.0, 1.0); // 计算在车体坐标系下，被吸取面的法向量
    return box_pos;
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
