# 视觉伺服启动前机械臂突跳问题复盘

## 1. 问题现象

在进入视觉伺服流程前，机械臂会先出现一次短促突跳，然后才进入正常的视觉伺服跟踪。

## 2. 初步怀疑

当时有两个候选原因：

1. 单点轨迹计算不稳定，首帧轨迹过激导致跳变。
2. 控制器模式切换存在时序问题，在实时轨迹尚未就绪时输出了异常命令。

## 3. 关键排查路径

本次排查重点围绕两条链路：

1. 任务节点如何切换控制模式（发布 use_moveit）。
2. 控制器在实时流模式下如何处理首帧目标点。

### 3.1 任务节点行为

在任务侧，视觉伺服开始前会发布：

- use_moveit = true（切入实时流）
- 后续循环中持续发布 initial_joint_trajectory 单点目标

意味着：控制模式切换与首条实时轨迹到达之间，存在天然时间窗。

### 3.2 控制器行为（修复前）

控制器在进入 realtime 模式后，立即读取 realtime_target_ 并写入命令接口。

但 realtime_target_ 在首条 initial_joint_trajectory 到达前可能为空或未就绪，代码中会按默认值回退（位置/速度/加速度可能退化为 0），这会造成切换瞬间的异常命令，从而引发“突跳”。

## 4. 根因结论

主因是控制模式切换竞态：

- 先切实时模式。
- 首帧目标还未到。
- 控制器提前用未就绪目标写命令。

这比“单点轨迹计算不严谨”更符合现象。

## 5. 代码修复内容

### 5.1 新增实时目标就绪标志

文件：src/control_pack/include/control_pack/mixcontroller.hpp

新增成员：

- std::atomic_bool realtime_target_ready_{false};

用途：标记是否收到过有效 realtime 目标。

### 5.2 订阅实时轨迹时做有效性校验

文件：src/control_pack/src/mixcontroller.cpp

在 initial_joint_trajectory 回调中新增：

1. 校验 points 非空。
2. 校验 positions/velocities/accelerations 维度不少于关节数（6）。
3. 校验前 6 维全部为 finite。
4. 仅在数据有效时更新 realtime_target_ 并置 realtime_target_ready_=true。

无效数据直接丢弃并告警，不再污染实时目标。

### 5.3 控制模式切换时重置就绪标志

文件：src/control_pack/src/mixcontroller.cpp

在 moveit_command 回调中：

1. false -> true（准备进入实时模式）时，将 realtime_target_ready_ 置 false。
2. true -> false（退出实时模式）时，也置 false。

目的：每次切模式都要求新首帧重新就绪。

### 5.4 update() 实时分支增加门禁保护

文件：src/control_pack/src/mixcontroller.cpp

在 is_realtime 分支最前面加入：

- 如果 realtime_target_ready_ == false：
  - 命令位置保持当前 state position。
  - 速度设 0。
  - 力矩设 0。
  - 直接 return，不执行实时目标跟踪。

效果：切换后等待首帧，不会输出默认/脏命令，避免首跳。

## 6. 为什么这次修复有效

核心机制是把“模式切换”和“命令生效”解耦：

- 以前：切模式即立刻按 realtime_target_ 写命令。
- 现在：切模式后先保持当前姿态，直到收到有效首帧再跟踪。

因此竞态窗口内不再出现异常控制输出。

## 7. 验证建议

建议按以下步骤验证：

1. 启动系统并触发一次抓取流程。
2. 观察日志顺序，应出现“实时模式已启用但首条目标未就绪，保持当前位置等待轨迹首帧”（若存在短暂窗口）。
3. 观察机械臂在视觉伺服启动前是否消除“先跳一下”现象。
4. 连续多次触发任务，确认无回归。

可重点关注日志关键词：

- 控制模式切换
- 实时模式已启用但首条目标未就绪
- 接收到轨迹消息

## 8. 风险与后续优化

本次修复解决的是“切模式竞态首跳”。

仍可进一步增强：

1. 在视觉伺服侧增加首帧关节增量限幅（delta q clamp），进一步抑制 IK 跳变。
2. 对速度/加速度做关节级限幅和滤波。
3. 将实时目标消息改为显式携带有效标识和序列号，提升诊断能力。

## 9. 本次执行记录

1. 已完成代码修改并写入仓库。
2. 原计划进行 colcon build --packages-select control_pack 编译验证，但执行时被手动取消，尚未拿到本次修改后的编译结果。

建议下一步先进行一次最小包编译与实机/仿真回归测试。
