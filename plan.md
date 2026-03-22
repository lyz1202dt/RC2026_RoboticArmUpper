
# 视觉接口：
1.初始位置订阅话题，并进行 TF 到base_link。
2.根据最初目标位置，生成一次轨迹，并执行。
3.执行过程中时刻订阅视觉话题，直到达到相机不能检测。
4.视觉控制：根据相机检测到的物体的位置，生成新的轨迹，并执行。
5.固定相机不能检测到物体后的轨迹。
```cpp
/**
 * geometry_msgs::msg::PoseStamped
 * {
 *   std_msgs::msg::Header header
 *   {
 *     std_msgs::msg::Time stamp
 *     string frame_id
 *   }
 *   geometry_msgs::msg::Pose pose
 *   {
 *     geometry_msgs::msg::Point position
 *     {
 *       double x
 *       double y
 *       double z
 *     }
 *     geometry_msgs::msg::Quaternion orientation
 *     {
 *       double x
 *       double y
 *       double z
 *       double w
 *     }
 *   }
 * }
 */
rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_subscription_;

geometry_msgs::msg::Pose detected_target_pose_;
geometry_msgs::msg::Pose available_target_pose_;


vision_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/vision_target", 10,
    std::bind(&ClassName::visionCallback, this, std::placeholders::_1));

/**
 * available: 相机成功检测到目标位置
 * unavailable: 相机不能检测到目标位置
 */ 
void visionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (msg == available){
        detected_target_pose_ = msg->pose;
        available_target_pose_ = detected_terget_pose_;
    } else if (msg == unavailable){
        detected_terget_pose_ = available_target_pose_;
    }
}
```

# 视觉伺服：
1.当前时刻期望位置
2.最终期望位置
3.实际位置

初始化：当前期望位置 = 实际位置
当前期望速度=0
上次期望速度=0
控制周期: 
当前期望速度=（最终期望位置-当前期望位置） * kp
当前期望加速度=limit（当前期望速度-上次期望速度）/dt
当前期望速度=上次期望速度+当前期望加速度*dt

当前期望位位置=当前期望位位置+当前期望速度*dt
当前机械臂目标=当前期望位置/当前期望速度/当前期望加速度
```cpp

struct point_velocity{
  double link1;
  double link2;
  double link3;
  double link4;
  double link5;
  double link6;
}

struct point_position{
  double link1;
  double link2;
  double link3;
  double link4;
  double link5;
  double link6;
}

struct point_acceleration{
  double link1;
  double link2;
  double link3;
  double link4;
  double link5;
  double link6;
}

double kp = 1.0;
double dt = 0.01; // 10 Hz

const trajectory_msgs::JointTrajectory& joint_trajectory = plan.trajectory_.joint_trajectory;
point_position current_expected_position_;
point_velocity current_expected_velocity_;
point_velocity last_expected_velocity_;
point_position final_expected_position_;
point_acceleration current_expected_acceleration_;

std::vector<point_position> point_positions_ ;
std::vector<point_velocity> point_velocities_ ;
std::vector<point_acceleration> point_accelerations_ ;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

for(int i = 0 ; i < joint_trajectory.points.size() ; ++i){
  point_positions_.push_back({joint_trajectory.points[i].positions[0], joint_trajectory.points[i].positions[1], 
                              joint_trajectory.points[i].positions[2], joint_trajectory.points[i].positions[3],
                              joint_trajectory.points[i].positions[4], joint_trajectory.points[i].positions[5]});

  point_velocity_.push_back({joint_trajectory.points[i].velocities[0], joint_trajectory.points[i].velocities[1],
                             joint_trajectory.points[i].velocities[2], joint_trajectory.points[i].velocities[3],
                             joint_trajectory.points[i].velocities[4], joint_trajectory.points[i].velocities[5]});
}

// 初始化：当前期望位置 = 实际位置
// 当前期望速度=0
// 上次期望速度=0
void init_(){
    current_expected_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    last_expected_velocity_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    final_expected_position_ = point_positions_[point_positions_.size() - 1];
    joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, 
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            current_expected_position_ = {msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], 
            msg->position[5]};
        })
}

// 控制周期逻辑
void controlCycle(){
    // 当前期望速度=（最终期望位置-当前期望位置） * kp
    point_velocity target_velocity = {
        (final_expected_position_.link1 - current_expected_position_.link1) * kp,
        (final_expected_position_.link2 - current_expected_position_.link2) * kp,
        (final_expected_position_.link3 - current_expected_position_.link3) * kp,
        (final_expected_position_.link4 - current_expected_position_.link4) * kp,
        (final_expected_position_.link5 - current_expected_position_.link5) * kp,
        (final_expected_position_.link6 - current_expected_position_.link6) * kp
    };
    
    // 当前期望加速度=limit（当前期望速度-上次期望速度）/dt
    point_acceleration raw_acceleration = {
        (target_velocity.link1 - last_expected_velocity_.link1) / dt,
        (target_velocity.link2 - last_expected_velocity_.link2) / dt,
        (target_velocity.link3 - last_expected_velocity_.link3) / dt,
        (target_velocity.link4 - last_expected_velocity_.link4) / dt,
        (target_velocity.link5 - last_expected_velocity_.link5) / dt,
        (target_velocity.link6 - last_expected_velocity_.link6) / dt
    };
    
    // 限制加速度范围（假设最大加速度为max_accel）
    double max_accel = 10.0; // 根据实际机械臂参数调整
    current_expected_acceleration_ = {
        std::max(-max_accel, std::min(max_accel, raw_acceleration.link1)),
        std::max(-max_accel, std::min(max_accel, raw_acceleration.link2)),
        std::max(-max_accel, std::min(max_accel, raw_acceleration.link3)),
        std::max(-max_accel, std::min(max_accel, raw_acceleration.link4)),
        std::max(-max_accel, std::min(max_accel, raw_acceleration.link5)),
        std::max(-max_accel, std::min(max_accel, raw_acceleration.link6))
    };
    
    // 当前期望速度=上次期望速度+当前期望加速度*dt
    current_expected_velocity_ = {
        last_expected_velocity_.link1 + current_expected_acceleration_.link1 * dt,
        last_expected_velocity_.link2 + current_expected_acceleration_.link2 * dt,
        last_expected_velocity_.link3 + current_expected_acceleration_.link3 * dt,
        last_expected_velocity_.link4 + current_expected_acceleration_.link4 * dt,
        last_expected_velocity_.link5 + current_expected_acceleration_.link5 * dt,
        last_expected_velocity_.link6 + current_expected_acceleration_.link6 * dt
    };
    
    // 当前期望位置=当前期望位置+当前期望速度*dt
    current_expected_position_ = {
        current_expected_position_.link1 + current_expected_velocity_.link1 * dt,
        current_expected_position_.link2 + current_expected_velocity_.link2 * dt,
        current_expected_position_.link3 + current_expected_velocity_.link3 * dt,
        current_expected_position_.link4 + current_expected_velocity_.link4 * dt,
        current_expected_position_.link5 + current_expected_velocity_.link5 * dt,
        current_expected_position_.link6 + current_expected_velocity_.link6 * dt
    };
    
    // 更新上次期望速度
    last_expected_velocity_ = current_expected_velocity_;
    
    // 当前机械臂目标=当前期望位置/当前期望速度/当前期望加速度
    publishArmTarget(current_expected_position_, current_expected_velocity_, current_expected_acceleration_);
    
}

// 发布机械臂目标函数
void publishArmTarget(const point_position& pos, const point_velocity& vel, const point_acceleration& acc){
    // 实现发布逻辑
}



```


# 轨迹规划：
前80%使用moveit规划轨迹，进行位置控制；后20%使用视觉伺服，进行速度控制。
## 获取轨迹点/截取前80%轨迹点
```cpp
trajectory_msgs::JointTrajectory trajectory_x80;
trajectory_msgs::JoitnTrajectory trajectory_x20;
const trajectory_msgs::JointTrajectory& joint_trajectory = plan.trajectory_.joint_trajectory;
size_t total_points = joint_trajectory.points.size();
size_t points_x80 = static_cast<size_t>(total_points * 0.8);
if (points_x80 >= total_points) {
  points_x80 = total_points - 1;
}


trajectory_x80.joint_names = joint_trajectory.joint_names;
trajectory_x80.points.assign(joint_trajectory.points.begin(), joint_trajectory.points.begin() + points_x80 + 1);
trajectory_x20.joint_names = joint_trajectory.joint_names;
trajectory_x20.points.assign(joint_trajectory.points.begin() + points_x80 + 1, joint_trajectory.points.end());
```

# controller
@mixcontroller#L216 : 前80%使用位置规划，后20%使用视觉伺服




# 伺服控制
Plan based on the target pose transmitted by the camera. 
## 路径
1.生成一段由 机械臂末端的笛卡尔坐标 指向 相机返回的目标笛卡尔空间坐标 的向量。
2.将向量作为机械臂末端期望轨迹。
3.末端目标朝向为向量指向的方向。
## 速度控制
1.设置最大速度为 1m/s。 当速度大于最大速度时，进行速度限制。
2.设置最大角速度为10rad/s 。角速度为当前末端朝向和末端目标朝向的夹角大小,当角速度大于最大角速度时，进行速度限制。



## 计算逻辑
1.当前时刻期望位置
2.最终期望位置
3.实际位置
初始化：当前期望位置 = 实际位置
当前期望速度=0
上次期望速度=0
控制周期: 
当前期望速度=（最终期望位置-当前期望位置） * kp
当前期望加速度=limit（当前期望速度-上次期望速度）/dt
当前期望速度=上次期望速度+当前期望加速度*dt

当前期望位位置=当前期望位位置+当前期望速度*dt
当前机械臂目标=当前期望位置/当前期望速度/当前期望加速度





```cpp







```



## 接口




速度大小为向量大小，
角速度大小为当前末端朝向和目标末端朝向的夹角大小，
















## 关于和相机通信
### @arm_handle_node 
```cpp
geometry_msgs::msg::Pose task_target_pos; // 目标位置




geometry_msgs::msg::Pose calculate_prepare_pos(const geometry_msgs::msg::Pose &box_pos, double approach_distance, geometry_msgs::msg::Pose &grasp_pose, ApproachMode mode = ApproachMode::AUTO);

geometry_msgs::msg::Pose detected_target_pose_; // 从视觉系统获取的目标位姿
geometry_msgs::msg::Pose detected_target_pose_on_base_link_; // 转换到base_link坐标系下的目标位姿
geometry_msgs::msg::Pose available_target_pose_;


std::unique_ptr<tf2_ros::Buffer> camera_link0_tf_buffer; // 坐标变换
std::shared_ptr<tf2_ros::TransformListener> camera_link0_tf_listener;
geometry_msgs::msg::TransformStamped camera_link0_tf;
std::shared_ptr<tf2_ros::Buffer> tf_buffer_; // 坐标系变换
std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // 接收和订阅坐标变换消息












// TODO: 真正的相机返回判断条件
// 回调函数
void visionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    if (!rclcpp::ok()) {
        return;
    }

    if (msg == nullptr) {
        RCLCPP_WARN(node->get_logger(), "警告：接收到空的视觉消息，拒绝机械臂目标请求");
        return;
    }

    // 先只做状态判断和数据拷贝，锁尽量短
    {

    }







    std::lock_guard<std::mutex> lock(vision_target_mutex_);
    if (msg->header.frame_id == "available_target") { // 假设相机发布的消息中，header.frame_id 用于区分目标是否可用
        detected_target_pose_ = msg->pose;
        available_target_pose_ = detected_target_pose_;
        has_vision_target_ = true;
    } else if (msg->header.frame_id == "unavailable_target") {
        detected_target_pose_ = available_target_pose_;
        has_vision_target_ = true;
    }




    // TODO: 在订阅者的回调函数里进行TF变换，将detected_target_pose_转换到base_link坐标系下，存储在detected_target_pose_on_base_link_中

    try {
        camera_link0_tf = camera_link0_tf_buffer->lookupTransform("base_link", "camera_link", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) { 
        RCLCPP_WARN(node->get_logger(), "警告：相机坐标系和变换查询失败，拒绝机械臂目标请求");
    }

    tf2::doTransform(detected_target_pose_, detected_target_pose_on_base_link_, camera_link0_tf);
    RCLCPP_INFO(node->get_logger(), "原始目标位姿: Pos(%lf,%lf,%lf), Rot(%lf,%lf,%lf,%lf)",
    detected_target_pose_on_base_link_.position.x, detected_target_pose_on_base_link_.position.y, detected_target_pose_on_base_link_.position.z,
    detected_target_pose_on_base_link_.orientation.w, detected_target_pose_on_base_link_.orientation.x, 
    detected_target_pose_on_base_link_.orientation.y, detected_target_pose_on_base_link_.orientation.z);


    auto qin=detected_target_pose_on_base_link_.orientation;
    tf2::Quaternion q(qin.x, qin.y, qin.z, qin.w);
    q.normalize();
    detected_target_pose_on_base_link_.orientation.w = q.w();
    detected_target_pose_on_base_link_.orientation.x = q.x();
    detected_target_pose_on_base_link_.orientation.y = q.y();
    detected_target_pose_on_base_link_.orientation.z = q.z();
}
























```








