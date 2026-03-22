## Plan: 修复控制器配置期执行器冲突

目标是在不回退订阅修复的前提下，消除 robotic_arm_controller 在 configure 阶段因同步参数客户端触发的执行器冲突，从而恢复控制器可配置、可激活和 MoveGroup 可执行。

**Steps**
1. 基线确认（诊断）
   记录当前失败链路：controller_manager 配置 robotic_arm_controller 时在 on_configure 报错 “Node '/robotic_arm_controller' has already been added to an executor”。
2. 方案选型（决策）
   优先采用“参数获取与控制器节点解耦”，避免在已被 executor 管理的控制器节点上执行 SyncParametersClient 同步调用。
3. 实施路径 A（推荐，稳定）
   在控制器中保留 get_node() 用于订阅（twist / initial_joint_trajectory），同时为 URDF 参数查询恢复独立参数节点（仅参数用途，不承载订阅）；on_configure 使用该独立节点创建 SyncParametersClient 并读取 robot_description。
4. 实施路径 B（备选，若不想保留独立参数节点）
   将 SyncParametersClient 改为 AsyncParametersClient，并在 on_configure 中只消费已缓存的 urdf_xml；必要时在 on_init 发起异步请求并设置就绪标记。*与步骤 3 互斥*
5. 安全检查（接口与赋值）
   校验 realtime 回调中的位置/速度/加速度尺寸一致性，update 中 controlled_dof 截断逻辑正确，command_interfaces 写入索引（i*3+0/1/2）与接口声明一致。
6. 运行验证
   启动后验证控制器状态从 unconfigured -> inactive -> active；验证初始关节轨迹回调触发；验证 q_kdl/dq_kdl/ddq_kdl 非零并与发布值一致；验证 MoveGroup execute 不再 aborted。

**Relevant files**
- /home/kyy/cpp_project/robot_arm/new_3/src/control_pack/src/mixcontroller.cpp — 关注 MixController::on_init、MixController::on_configure、MixController::update 的节点来源与参数查询路径。
- /home/kyy/cpp_project/robot_arm/new_3/src/control_pack/include/control_pack/mixcontroller.hpp — 关注参数客户端成员定义与（若采用路径 A）独立参数节点成员。
- /home/kyy/cpp_project/robot_arm/new_3/src/robotic_task/src/ArmHandleNodeVisualServoing.cpp — 发布端 topic 名称、消息尺寸与 header 填充保持不变，仅用于联调验证。

**Verification**
1. 编译：colcon build，确认 control_pack 无编译错误。
2. 启动系统并观察控制器 lifecycle 日志：不得再出现 already been added to an executor。
3. 运行 ros2 control list_controllers，确认 robotic_arm_controller 为 active（或至少 configure 成功后的 inactive）。
4. 发送一帧 initial_joint_trajectory，确认接收日志显示 points[0] 的 positions/velocities/accelerations 尺寸为 6。
5. 在 update 日志中核对 q_kdl(0..5)、dq_kdl(0..5)、ddq_kdl(0..5) 不为全 0，且与发布值数量级一致。
6. 触发一次 MoveGroup execute，确认 execute request 不再 aborted。

**Decisions**
- 包含范围：仅处理执行器冲突与数据链路可达性，不调整运动学/动力学算法本身。
- 不包含范围：吸盘方向计算策略、轨迹规划参数调优。
- 推荐决策：订阅必须继续挂在 get_node()；参数查询可单独使用独立节点或异步客户端，避免同步客户端绑定已托管控制器节点。

**Further Considerations**
1. 若后续要长期维护，建议统一日志级别：接收路径保留 INFO，周期 update 降为 DEBUG_THROTTLE，避免刷屏影响诊断。
