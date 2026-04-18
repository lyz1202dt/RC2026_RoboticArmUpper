# 通信接口整理

本工程的通信接口统一放在 `robot_interfaces` 包中，包含 3 个消息和 1 个动作：

## 接口定义

| 类型 | 文件 | 接口名 | 内容 |
| --- | --- | --- | --- |
| msg | [src/robot_interfaces/msg/Joint.msg](src/robot_interfaces/msg/Joint.msg) | Joint | 单个关节状态：`rad`、`omega`、`torque` |
| msg | [src/robot_interfaces/msg/Arm.msg](src/robot_interfaces/msg/Arm.msg) | Arm | 机械臂状态：`Joint[6] joints` |
| msg | [src/robot_interfaces/msg/Moveit.msg](src/robot_interfaces/msg/Moveit.msg) | Moveit | 轨迹规划模式开关：`use_moveit` |
| action | [src/robot_interfaces/action/Catch.action](src/robot_interfaces/action/Catch.action) | Catch | 抓取任务：`action_type`、`target_pose`、`reason`、`kfs_num`、`current_state`、`state_describe` |

## 依赖关系

- `Arm.msg` 依赖 `Joint.msg`
- `Catch.action` 依赖 `geometry_msgs/Pose`

## 主要使用位置

| 模块 | 使用接口 | 典型文件 |
| --- | --- | --- |
| `control_pack` | `Arm`、`Moveit`、`FollowJointTrajectory` | [src/control_pack/include/control_pack/mixcontroller.hpp](src/control_pack/include/control_pack/mixcontroller.hpp)、[src/control_pack/src/mycontrol.cpp](src/control_pack/src/mycontrol.cpp)、[src/control_pack/src/mixcontroller.cpp](src/control_pack/src/mixcontroller.cpp) |
| `robot_driver` | `Arm` | [src/robot_driver/include/robot_driver/serialnode.hpp](src/robot_driver/include/robot_driver/serialnode.hpp)、[src/robot_driver/src/serialnode.cpp](src/robot_driver/src/serialnode.cpp) |
| `pnp_ros` | `Catch` | [src/pnp_ros/src/pnp_ros.cpp](src/pnp_ros/src/pnp_ros.cpp) |
| `robotic_task` | `Arm`、`Moveit`、`Catch` | [src/robotic_task/include/robotic_task/arm_handle_node.hpp](src/robotic_task/include/robotic_task/arm_handle_node.hpp)、[src/robotic_task/include/robotic_task/ArmHandleNodeVisualServoing.hpp](src/robotic_task/include/robotic_task/ArmHandleNodeVisualServoing.hpp)、[src/robotic_task/src/test.cpp](src/robotic_task/src/test.cpp) |

## 说明

- 这些接口已经按 ROS2 规范拆分到独立的 `.msg` 和 `.action` 文件中，不能在语义上真正合并成一个 ROS 接口文件。
- 如果目的是“使用入口统一”，推荐在 `robot_interfaces` 包内再提供一个聚合头文件，由业务代码统一 include。
