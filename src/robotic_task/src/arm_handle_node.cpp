#include "arm_handle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <memory>
#include <rclcpp/parameter_client.hpp>
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
    arm_target_publisher     = rclcpp::create_publisher<robot_interfaces::msg::Arm>(node, "myjoints_target", 10); // 创建发布者，发布关节空间的目标
    camera_link0_tf_buffer   = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    camera_link0_tf_listener = std::make_shared<tf2_ros::TransformListener>(*camera_link0_tf_buffer);
    move_group_interface     = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "robotic_arm");



    arm_idel_pos.orientation.w = 1.0;
    arm_idel_pos.position.x    = 0.1;
    arm_idel_pos.position.y    = 0.0;
    arm_idel_pos.position.z    = 0.8;


    attached_kfs_pos.orientation.w = 1.0;
    attached_kfs_pos.position.x    = 0.175;
    attached_kfs_pos.position.y    = 0.0;
    attached_kfs_pos.position.z    = 0.0;
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
    geometry_msgs::msg::Pose transformed_pose;
    tf2::doTransform(goal->target_pose, transformed_pose, camera_link0_tf);
    task_target_pos   = transformed_pose;
    current_task_type = goal->action_type;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
    ArmHandleNode::cancel_goal(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {
    (void)goal_handle;
    cancle_current_task = true;
    is_running_arm_task = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmHandleNode::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<robot_interfaces::action::Catch>> goal_handle) {
    (void)goal_handle;
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
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    move_group_interface->setPlanningTime(1);
    move_group_interface->setNumPlanningAttempts(100);
    // auto robot_module = move_group_interface->getRobotModel();
    auto feedback_msg = std::make_shared<robot_interfaces::action::Catch::Feedback>();
    auto finished_msg = std::make_shared<robot_interfaces::action::Catch::Result>();

    {   //为规划环境增加四个竖起来的杆（R2上底盘抬升部分需要）
        // 添加障碍物
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_interface->getPlanningFrame();
        collision_object.id              = "institution";
        collision_object.primitives.resize(4);
        collision_object.primitive_poses.resize(4);

        geometry_msgs::msg::Pose collision_pos[4];
        collision_pos[0].orientation.w = 1;
        collision_pos[0].position.x    = -0.2;
        collision_pos[0].position.y    = 0.2;
        collision_pos[0].position.z    = 0.3;

        collision_pos[1].orientation.w = 1;
        collision_pos[1].position.x    = 0.2;
        collision_pos[1].position.y    = 0.2;
        collision_pos[1].position.z    = 0.3;

        collision_pos[2].orientation.w = 1;
        collision_pos[2].position.x    = -0.2;
        collision_pos[2].position.y    = -0.2;
        collision_pos[2].position.z    = 0.3;

        collision_pos[3].orientation.w = 1;
        collision_pos[3].position.x    = 0.2;
        collision_pos[3].position.y    = -0.2;
        collision_pos[3].position.z    = 0.3;

        shape_msgs::msg::SolidPrimitive primitive;
        collision_object.primitives[0].type = primitive.BOX;
        collision_object.primitives[0].dimensions.resize(3);
        collision_object.primitives[0].dimensions[primitive.BOX_X] = 0.1;
        collision_object.primitives[0].dimensions[primitive.BOX_Y] = 0.1;
        collision_object.primitives[0].dimensions[primitive.BOX_Z] = 0.6;
        collision_object.primitives[3]=collision_object.primitives[2]=collision_object.primitives[1]=collision_object.primitives[0];

        collision_object.operation = collision_object.ADD;
        psi.applyCollisionObject(collision_object);         //应用障碍物
    }

    while (rclcpp::ok()) {
        is_running_arm_task = false;
        std::unique_lock<std::mutex> lock(task_mutex_);
        task_cv_.wait(lock, [this]() { return has_new_task_; });  // 等待直到lambda表达式返回真
        has_new_task_ = false;
        lock.unlock();
        
        if (!rclcpp::ok()) {
            break;
        }

        if (current_task_type == ROBOTIC_ARM_TASK_MOVE)           // 要求机械臂移动到某个位姿（因为移动动作在一个周期内完成，所以不再需要执行）
        {
            move_group_interface->setPoseTarget(task_target_pos); // 设置目标
            bool success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划当前位置到目标位置的曲线
            if (success) {
                bool finished = send_plan(plan.trajectory);                                              // 阻塞函数，发送轨迹

                if (!finished) {
                    finished_msg->reason  = "任务被客户端取消";
                    finished_msg->kfs_num = current_kfs_num;
                    current_goal_handle->canceled(finished_msg);
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
            auto prepare_pos = calculate_prepare_pos(task_target_pos);   // 规划路径到目标位置前
            move_group_interface->setPoseTarget(prepare_pos);            // 设置目标
            bool success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS); // 规划从当前位置到目标位置的曲线
            if (!success) {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "机械臂路径规划失败，目标位姿可能不可达";
                current_goal_handle->abort(finished_msg);
                continue;
            }
            bool trj_deal = send_plan(plan.trajectory);
            if (!trj_deal) {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "用户取消机械臂执行，任务失败";
                current_goal_handle->canceled(finished_msg);
                continue;
            }
            feedback_msg->current_state  = 1;
            feedback_msg->state_describe = "机械臂移动到待抓取位置";
            current_goal_handle->publish_feedback(feedback_msg);

            auto start_pose = move_group_interface->getCurrentPose().pose;
            std::vector<geometry_msgs::msg::Pose> way_points;
            way_points.resize(2);
            way_points[0] = start_pose;                                                                  // 当前位姿
            way_points[1] = task_target_pos;                                                             // 最终抓取位姿
            moveit_msgs::msg::RobotTrajectory cart_trajectory;
            double fraction = move_group_interface->computeCartesianPath(way_points, 0.01, cart_trajectory, false);
            if (fraction < 0.99f)                                                                        // 如果轨迹生成失败
            {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "抓取时机械臂超出工作范围，抓取失败";
                current_goal_handle->abort(finished_msg);
                continue;
            }
            trj_deal = send_plan(cart_trajectory);                                                       // 将轨迹发送给机械臂执行
            if (!trj_deal) {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "用户取消机械臂执行，任务失败";
                current_goal_handle->canceled(finished_msg);
                continue;
            }
            feedback_msg->current_state  = 2;
            feedback_msg->state_describe = "机械臂到达吸取位置，启动气泵吸取KFS";
            current_goal_handle->publish_feedback(feedback_msg);
            // 使用一个参数服务来启动气泵
            param_client->set_parameters({rclcpp::Parameter("enable_air_pump", true)});
            std::this_thread::sleep_for(2s);
            // 为机械臂末端连接一个KFS用于碰撞计算
            add_attached_kfs_collision(attached_kfs_pos, "kfs", "link6");

            if (current_kfs_num == 0)                              // 根据当前机器人上的情况设置目标
                move_group_interface->setPoseTarget(arm_box1_pos); // 设置目标
            else if (current_kfs_num == 1)
                move_group_interface->setPoseTarget(arm_box2_pos);
            else
                move_group_interface->setPoseTarget(arm_box3_hold_pos);

            success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (!success) {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "机械臂无法到达放置KFS的位置，路径规划失败";
                current_goal_handle->abort(finished_msg);
                param_client->set_parameters({rclcpp::Parameter("enable_air_pump", false)});
                remove_attached_kfs_collision("kfs", "link6");
                continue;
            }
            trj_deal = send_plan(plan.trajectory);
            if (!trj_deal) {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "用户取消机械臂执行，任务失败";
                current_goal_handle->canceled(finished_msg);
                param_client->set_parameters({rclcpp::Parameter("enable_air_pump", false)});
                remove_attached_kfs_collision("kfs", "link6");
                continue;
            }
            feedback_msg->current_state  = 3;
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
            param_client->set_parameters({rclcpp::Parameter("enable_air_pump", false)});
            std::this_thread::sleep_for(2s);

            remove_attached_kfs_collision("kfs", "link6");               // 清除附着的KFS
            if (current_kfs_num == 0)
                add_kfs_collision(kfs1_pos, "kfs1", "link6");
            else if (current_kfs_num == 1)
                add_kfs_collision(kfs2_pos, "kfs2", "link6");


            way_points[0] = move_group_interface->getCurrentPose().pose;

            if (current_kfs_num == 0)
                way_points[1] = arm_box1_ready_pos;
            else if (current_kfs_num == 1)
                way_points[1] = arm_box2_ready_pos;
            fraction = move_group_interface->computeCartesianPath(way_points, 0.01, cart_trajectory, false);
            if (fraction < 0.99) {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "机械臂回退与KFS分离时路径规划失败";
                current_goal_handle->abort(finished_msg);
                continue;
            }
            trj_deal = send_plan(cart_trajectory);                       // 远离KFS一段距离防止错误判定碰撞
            if (!trj_deal) {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "用户取消机械臂执行，任务失败";
                current_goal_handle->canceled(finished_msg);
                continue;
            }
            feedback_msg->current_state  = 4;
            feedback_msg->state_describe = "机械臂回退完成";
            current_goal_handle->publish_feedback(feedback_msg);

            move_group_interface->setPoseTarget(arm_idel_pos);
            success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            if (!success) {
                finished_msg->kfs_num = current_kfs_num;
                finished_msg->reason  = "机械臂无法回到空闲位置，路径规划失败";
                current_goal_handle->abort(finished_msg);
                continue;
            }
            feedback_msg->current_state  = 5;
            feedback_msg->state_describe = "机械臂到达空闲位置";
            current_goal_handle->publish_feedback(feedback_msg);

            current_kfs_num       = current_kfs_num + 1;
            finished_msg->kfs_num = current_kfs_num;
            finished_msg->reason  = "成功抓取KFS并放置到机器人上";
            current_goal_handle->succeed(finished_msg);
        } else if (current_task_type == ROBOTIC_ARM_TASK_PLACE_TARGET) { // 将车上的KFS吸起来，然后放到某个坐标
            // TODO:根据机器人上方块的实际位置计数，移动到固定的坐标准备抓取车上的KFS
        }
    }
    RCLCPP_INFO(node->get_logger(), "退出机械臂任务处理线程");
}

bool ArmHandleNode::send_plan(const moveit_msgs::msg::RobotTrajectory& trajectory) {
    robot_interfaces::msg::Arm arm_msg;
    auto start_time = std::chrono::high_resolution_clock::time_point::clock::now();
    for (auto const& point : trajectory.joint_trajectory.points) {
        double time_from_start = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9;
        std::this_thread::sleep_until(start_time + std::chrono::duration<double>(time_from_start));

        if (!rclcpp::ok())
            return false;

        if (point.positions.size() != 6) {
            RCLCPP_WARN(node->get_logger(), "不符合的关节数");
            continue;
        }

        for (int i = 0; i < 6; i++) {
            arm_msg.joints[i].rad   = (float)point.positions[i];
            arm_msg.joints[i].omega = (float)point.velocities[i];
        }
        RCLCPP_INFO(node->get_logger(), "发布关节角期望");
        arm_target_publisher->publish(arm_msg);

        if (cancle_current_task)                            // 如果要求退出当前的动作执行
        {
            for (int i = 0; i < 6; i++) {
                arm_msg.joints[i].omega = 0.0f;
            }
            arm_target_publisher->publish(arm_msg);
            return false;
        }
    }
    return true;
}

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
bool ArmHandleNode::add_attached_kfs_collision(const geometry_msgs::msg::Pose& pos, const std::string& object_id, const std::string& fram_id) {
    moveit_msgs::msg::AttachedCollisionObject collision_object;
    collision_object.link_name              = fram_id;
    collision_object.object.header.frame_id = fram_id;
    collision_object.object.id              = object_id;
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.35;
    primitive.dimensions[primitive.BOX_Y] = 0.35;
    primitive.dimensions[primitive.BOX_Z] = 0.35;

    collision_object.object.primitives.push_back(primitive);
    collision_object.object.primitive_poses.push_back(pos);
    collision_object.object.operation = collision_object.object.ADD;
    psi.applyAttachedCollisionObject(collision_object);
    return true;
}

bool ArmHandleNode::remove_attached_kfs_collision(const std::string& object_id, const std::string& fram_id) {
    moveit_msgs::msg::AttachedCollisionObject collision_object;
    collision_object.link_name              = fram_id;
    collision_object.object.header.frame_id = fram_id;
    collision_object.object.id              = object_id;
    collision_object.object.operation       = collision_object.object.REMOVE;
    psi.applyAttachedCollisionObject(collision_object);
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
    psi.applyCollisionObject(collision_object);
    return true;
}

bool ArmHandleNode::remove_kfs_collision(const std::string& object_id, const std::string& fram_id) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = fram_id;
    collision_object.id              = object_id;
    collision_object.operation       = collision_object.REMOVE;
    psi.applyCollisionObject(collision_object);
    return true;
}
