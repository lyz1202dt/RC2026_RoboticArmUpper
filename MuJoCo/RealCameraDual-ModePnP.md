# MuJoCo/Real Camera Dual-Mode PnP 复刻文档

## 1. 目标与结果

本次改造目标：
- 在 MuJoCo 启动链路中默认使用仿真相机。
- 在真实机器人启动链路中默认使用真实摄像头。
- pnp_ros_node 同时支持仿真图像和真实摄像头两种输入模式，并可参数切换。

落地结果：
- arm_mujoco_sim.launch.py 默认 camera_source=sim。
- real_robot.launch.py 默认 camera_source=real。
- MuJoCo 在 link5 处挂载了 camera_link 相机，并发布 /camera_link/color/image_raw。

## 2. 本次修改清单

1) PnP 双输入模式
- 文件: src/pnp_ros/src/pnp_ros.cpp
- 新参数:
	- camera_source: sim 或 real
	- sim_image_topic: 仿真图像话题，默认 /camera_link/color/image_raw
	- real_video_device_id: 真实摄像头设备号，默认 4
	- min_goal_send_interval_sec / goal_pos_threshold_m / goal_angle_threshold_rad
- 行为:
	- real: 走 OpenCV VideoCapture。
	- sim: 订阅 ROS 图像并缓存最新帧，后续复用同一套 PnP 与 TF 流程。

2) MuJoCo 启动默认使用仿真相机
- 文件: src/robotic_config/launch/arm_mujoco_sim.launch.py
- 关键参数:
	- camera_source 默认 sim
	- sim_image_topic 默认 /camera_link/color/image_raw
	- real_video_device_id 默认 4

3) 真实机器人启动默认使用真实摄像头
- 文件: src/robotic_config/launch/real_robot.launch.py
- 关键参数:
	- camera_source 默认 real
	- sim_image_topic 默认 /camera_link/color/image_raw
	- real_video_device_id 默认 4

4) MuJoCo 机载仿真相机挂载
- 文件: src/robotic_arm/model/robotic_arm.xml
- 在 link5 body 下新增:
	- <camera name="camera_link" pos="0.01 0.01 0" xyaxes="1 0 0 0 -1 0"/>

## 3. 复刻步骤

### 步骤 A: 构建

在工作区根目录执行:

colcon build --packages-select robotic_arm mujoco_ros2_control pnp_ros robotic_config

然后刷新环境:

source install/setup.bash

### 步骤 B: 启动 MuJoCo 仿真模式

默认命令:

ros2 launch robotic_config arm_mujoco_sim.launch.py

可覆盖参数示例:

ros2 launch robotic_config arm_mujoco_sim.launch.py camera_source:=sim sim_image_topic:=/camera_link/color/image_raw

### 步骤 C: 启动真实机器人模式

默认命令:

ros2 launch robotic_config real_robot.launch.py

可覆盖参数示例:

ros2 launch robotic_config real_robot.launch.py camera_source:=real real_video_device_id:=4

### 步骤 D: 基础验证

1. 检查仿真图像话题是否存在:

ros2 topic list | rg color/image_raw

2. 检查仿真图像频率:

ros2 topic hz /camera_link/color/image_raw

3. 检查 PnP 输出:

ros2 topic echo /box_pose

4. 检查 TF 链:

ros2 run tf2_ros tf2_echo base_link camera_optical_frame

## 4. 如何更改仿真相机位置和朝向

本项目中，相机几何和坐标链路分成两层：

- 图像渲染层（MuJoCo 相机实体）
	- 文件: src/robotic_arm/model/robotic_arm.xml
	- 影响看到的图像内容和视角。

- TF 链路层（ROS static TF）
	- 文件: src/robotic_config/launch/arm_mujoco_sim.launch.py
	- 影响 camera_link/camera_optical_frame 到 base_link 的空间关系。

建议：两层配置保持一致，避免视觉结果与 TF 不一致。

### 4.1 修改仿真相机位置

修改 MuJoCo 相机位置:
- 文件: src/robotic_arm/model/robotic_arm.xml
- 位置字段: camera 标签中的 pos="x y z"

示例:
- 当前位置: pos="0.01 0.01 0"
- 向前移动 2 cm: pos="0.03 0.01 0"

同时修改 TF 偏移:
- 文件: src/robotic_config/launch/arm_mujoco_sim.launch.py
- 启动参数 camera_x/camera_y/camera_z

示例:

ros2 launch robotic_config arm_mujoco_sim.launch.py camera_x:=0.03 camera_y:=0.01 camera_z:=0.0

### 4.2 修改仿真相机朝向

方法 1（推荐）: 在 MuJoCo 相机上改 xyaxes
- 文件: src/robotic_arm/model/robotic_arm.xml
- 字段: xyaxes="x1 x2 x3 y1 y2 y3"

含义:
- 前 3 个数是相机 X 轴方向（在父坐标系中）
- 后 3 个数是相机 Y 轴方向
- Z 轴由 X×Y 自动确定

例如:
- 当前值: xyaxes="1 0 0 0 -1 0"
- 若想先做小角度试验，建议每次只改一个轴分量并验证图像方向。

方法 2: 使用 quat 指定旋转
- 同文件 camera 标签可改用 quat="w x y z"
- 若使用 quat，建议移除 xyaxes，避免表达重复。

### 4.3 修改光学坐标系定义

文件: src/robotic_config/launch/arm_mujoco_sim.launch.py

节点: static_transform_publisher（camera_link -> camera_optical_frame）

当前参数:
- roll=-1.5708, pitch=0, yaw=-1.5708

若相机朝向策略变化，需要同步调整这个静态 TF，保证 PnP 在 camera_optical_frame 下的位姿定义不变。

## 5. 常见问题

1) sim 模式无图像
- 检查 camera_source 是否为 sim。
- 检查 sim_image_topic 是否存在且频率正常。
- 检查 robotic_arm.xml 中 camera name 与话题名是否一致。

2) 有图像但 TF 转换失败
- 检查 link5 -> camera_link 和 camera_link -> camera_optical_frame 两段 TF 是否都在发布。
- 用 tf2_echo 验证 base_link 到 camera_optical_frame。

3) 仿真画面方向和抓取方向不一致
- 优先核对 MuJoCo camera 的 xyaxes 或 quat。
- 再核对 camera_optical_frame 静态 TF 欧拉角。

## 6. 变更后推荐验证顺序

1. 先只改位置，不改朝向，确认目标位姿数值方向正确。
2. 再微调朝向，每次只改一个参数组（xyaxes 或 optical TF 二选一）。
3. 每次改动后都执行:
	 - 图像话题检查
	 - TF 链检查
	 - /box_pose 连续输出检查
