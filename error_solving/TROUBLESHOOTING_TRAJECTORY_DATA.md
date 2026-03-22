# 轨迹数据传输与控制器配置问题诊断（完整闭环）

## 问题概览

本次联调经历了两个连续问题：

1. **阶段一：实时轨迹数据收不到**
- 发送端 `ArmHandleNodeVisualServoing` 已发布 `initial_joint_trajectory`
- 接收端 `MixController` 中 `q_kdl/dq_kdl/ddq_kdl` 全为 0

2. **阶段二：修复订阅后出现 configure 失败**
- 报错：`Node '/robotic_arm_controller' has already been added to an executor`
- 导致 `robotic_arm_controller` 处于 `unconfigured`
- MoveGroup 执行请求被中止（`Execute request aborted`）

---

## 阶段一诊断与修复

### 现象

发送端日志可见轨迹已发送：

```text
发送关节轨迹命令: positions=[...], velocities=[...], accelerations=[...]
```

控制器端实时模式中依然是零：

```text
q_kdl(i)=0, dq_kdl(i)=0, ddq_kdl(i)=0
```

### 根因

旧实现在构造函数里创建了独立节点 `param_node`，并把订阅挂在该节点上；该节点未被 executor 驱动，回调不执行。

### 修复

- 订阅全部改为挂在控制器节点 `this->get_node()`
- 订阅创建位置放在 `on_init()`

这样可确保回调由 controller_manager 的 executor 驱动。

---

## 阶段二诊断与修复

### 现象

阶段一修复后，控制器在 `on_configure()` 报错：

```text
Node '/robotic_arm_controller' has already been added to an executor
```

### 根因

在 `on_configure()` 中使用：

```cpp
robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(this->get_node(), "/robot_state_publisher");
```

`this->get_node()` 已由 controller_manager 管理，`SyncParametersClient` 同步调用会触发执行器冲突。

### 最终修复（已落地）

采用“订阅与参数查询解耦”：

1. **订阅仍使用控制器节点** `this->get_node()`（保持阶段一修复成果）
2. **参数查询使用独立节点** `param_query_node_`
3. 在 `on_configure()` 中：
   - 用 `param_query_node_` 创建 `SyncParametersClient`
   - 增加 `wait_for_service(std::chrono::seconds(2))` 检查
   - 再读取 `robot_description`

---

## 代码落地摘要

### 1) 头文件

文件：`src/control_pack/include/control_pack/mixcontroller.hpp`

- 新增成员：

```cpp
rclcpp::Node::SharedPtr param_query_node_;
```

### 2) 初始化

文件：`src/control_pack/src/mixcontroller.cpp`

- 在 `on_init()` 中创建：

```cpp
param_query_node_ = std::make_shared<rclcpp::Node>("robotic_arm_controller_param_client");
```

### 3) 配置阶段参数读取

文件：`src/control_pack/src/mixcontroller.cpp`

- 在 `on_configure()` 中改为：

```cpp
robot_description_param_ = std::make_shared<rclcpp::SyncParametersClient>(param_query_node_, "/robot_state_publisher");
if (!robot_description_param_->wait_for_service(std::chrono::seconds(2))) {
    return controller_interface::CallbackReturn::ERROR;
}
auto params = robot_description_param_->get_parameters({"robot_description"});
```

### 4) 附带修正

- 修复接收日志中 `stamp.sec/nanosec` 的格式化类型，避免格式化告警升级为构建问题。

---

## 当前验证状态

已完成：

1. `control_pack` 单包编译通过
2. 订阅链路保留在 `get_node()`，避免回退到“收不到数据”问题
3. 参数读取链路改为独立参数节点，规避 configure 阶段 executor 冲突

待运行时确认：

1. 启动后 `robotic_arm_controller` 可从 `unconfigured -> inactive -> active`
2. 不再出现 `already been added to an executor`
3. `initial_joint_trajectory` 回调可持续接收非零数据
4. MoveGroup 执行不再 `Execute request aborted`

---

## 关键结论

1. **订阅**应绑定 `this->get_node()`（让 controller_manager executor 处理）。
2. **同步参数查询**不要绑定已托管控制器节点；应使用独立参数查询节点或改异步参数客户端。
3. 这两条并不冲突，正确做法是“订阅与参数查询分离”。

---

## 最佳实践

对于 ROS 2 `controller_interface` 插件：

- 在 `on_init()` 中创建订阅/发布器，默认使用 `this->get_node()`
- 对同步参数服务调用，避免直接复用已托管控制器节点
- 对关键数据链路保留轻量验证日志（尺寸、首元素、模式切换）
- 避免在 `update()` 高频回路中长期使用大量 `std::cout` 刷屏
