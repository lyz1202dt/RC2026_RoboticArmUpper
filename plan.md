# 视觉接口：
## 关于和相机通信
### @arm_handle_node 
```cpp
geometry_msgs::msg::Pose task_target_pos; // 目标位置
geometry_msgs::msg::Pose calculate_prepare_pos(
    const geometry_msgs::msg::Pose &box_pos,
    double approach_distance,
    geometry_msgs::msg::Pose &grasp_pose,
    ApproachMode mode = ApproachMode::AUTO
);

geometry_msgs::msg::Pose detected_target_pose_;               // 相机坐标系下的目标位姿（原始/回退）
geometry_msgs::msg::Pose detected_target_pose_on_base_link_;  // base_link下的目标位姿（TF后）
geometry_msgs::msg::Pose available_target_pose_;              // 最近一次可用目标位姿

std::unique_ptr<tf2_ros::Buffer> camera_link0_tf_buffer;      // TF Buffer
std::shared_ptr<tf2_ros::TransformListener> camera_link0_tf_listener;
geometry_msgs::msg::TransformStamped camera_link0_tf;

std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

std::mutex vision_target_mutex_;
bool has_vision_target_{false};

rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_subscription_;

vision_subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "robotic_task_", 10,
    std::bind(&ArmHandleNode::visionCallback, this, std::placeholders::_1)
);

// 回调函数
void visionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!rclcpp::ok()) {
        return;
    }

    if (!msg) {
        RCLCPP_WARN(node->get_logger(), "警告：接收到空的视觉消息");
        return;
    }

    geometry_msgs::msg::Pose pose_in_camera;
    bool candidate_valid = false;

    // 第一段短锁：读取/更新共享缓存
    {
        std::lock_guard<std::mutex> lock(vision_target_mutex_);

        
        if (msg->header.frame_id == "available_target") { // TODO: 根据实际情况调整视觉检测结果的判断标准
            pose_in_camera = msg->pose;                   // 现在暂且认为frame_id == "available_target" 时返回可用结果
            available_target_pose_ = pose_in_camera;
            candidate_valid = true;
        } else if (msg->header.frame_id == "unavailable_target") {
            if (has_vision_target_) {
                pose_in_camera = available_target_pose_;
                candidate_valid = true;
            } else {
                RCLCPP_WARN(node->get_logger(), "警告：无可用视觉目标且无历史缓存");
                return;
            }
        }
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

    RCLCPP_INFO(
        node->get_logger(),
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
```








