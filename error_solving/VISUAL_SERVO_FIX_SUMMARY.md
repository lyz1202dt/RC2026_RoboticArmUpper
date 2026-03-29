## 视觉伺服启动机械臂跳动问题 - 解决方案总结

### 问题根源
机械臂在启动视觉伺服前突然跳一下，是由于 **控制模式切换（MoveIt → 视觉伺服）时产生速度/加速度不连续**。

当从 MoveIt 轨迹执行切换到视觉伺服控制时：
- MoveIt 执行的最后一个轨迹可能有非零速度
- 视觉伺服启动时直接重置速度为 0
- 根据位置误差快速计算新速度：`v = (target - current) × kp`
- 由 kp=1.0 的硬编码系数和位置差异，产生很大的速度变化
- 在一个控制周期内，这导致很大的加速度
- 机械臂驱动器执行这个急剧的加速度命令，导致突跳

### 实施的三个修复

#### 修复1：参数化 kp 系数 ✅
**文件**: [ArmHandleNodeVisualServoing.hpp](ArmHandleNodeVisualServoing.hpp#L123-L127)

- 添加 `kp_startup_` = 0.2（启动阶段的较小增益）
- 添加 `kp_ramp_steps_` = 20（过渡步数）
- 添加 `servo_startup_smooth_enabled_` 开关
- 从参数服务器读取这些值

**效果**: 可以灵活调整启动行为，而不需要重新编译

#### 修复2：启动平滑过渡 ✅
**文件**: [ArmHandleNodeVisualServoing.cpp](ArmHandleNodeVisualServoing.cpp#L550-L590)

在 `ComputationalSpeed()` 中实现：
```cpp
// 启动阶段线性插值增益系数
kp_effective = kp_startup_ + (kp_ - kp_startup_) * (startup_iteration_count_ / kp_ramp_steps_)

// 然后用 kp_effective 代替硬编码的 kp
v = (target - current) × kp_effective
```

**原理**: 
- 前 20 个迭代（约 0.2 秒）：从 0.2 逐步增加到 1.0
- 初始速度 = (Δpos) × 0.2，而不是 × 1.0
- 加速度 = (0.2 × Δpos - 0) / dt，更温和
- 之后逐步恢复到正常控制增益

**效果**: 启动时速度和加速度逐步增加，避免突跳

#### 修复3：启动前等待机械臂稳定 ✅
**文件**: [arm_handle_node.cpp](arm_handle_node.cpp#L860-L900)

在发送 `moveit_msg` 前添加稳定性检查：
```cpp
// 连续检查位置变化 < 0.001m 的次数
// 当连续 5 帧位置变化很小时认为稳定
// 最多等待 1 秒
```

**效果**: 确保机械臂完全静止后再切换控制模式，避免在运动中切换导致的突跳

### 配置调整指南

参见 [VISUAL_SERVO_LAUNCH_CONFIG.yaml](VISUAL_SERVO_LAUNCH_CONFIG.yaml)

**默认推荐配置**:
```yaml
visual_servo:
  startup_smooth_enabled: true
  kp_startup: 0.2      # 启动初始增益
  kp: 1.0              # 正常增益  
  kp_ramp_steps: 20    # 过渡步数 (0.2秒 @ 100Hz)
```

**如果仍有跳动** → 降低参数:
```yaml
kp_startup: 0.1        # 更温和的启动
kp_ramp_steps: 40      # 更长的过渡时间
```

**如果启动太慢** → 提高参数:
```yaml
kp_startup: 0.3        # 更激进的启动
kp_ramp_steps: 10      # 更短的过渡时间
```

### 预期改进

| 方面 | 改进前 | 改进后 |
|------|--------|--------|
| 启动时机械臂行为 | 突然跳跃 | 平滑加速 |
| 加速度（初始） | 很大（突变） | 逐步增加 |
| 启动时间 | 立即 | +0.2秒（可配置） |
| 稳定性 | 不稳定 | 更稳定 |
| 可调整性 | 无法调整 | 可参数化 |

### 测试验证

启动视觉伺服后查看日志:
```
[视觉伺服] 等待机械臂稳定后启动视觉伺服...
[视觉伺服] 机械臂已稳定，启动视觉伺服
[视觉伺服启动平滑] 迭代 1/20, kp: 0.2 → 0.25
[视觉伺服启动平滑] 迭代 5/20, kp: 0.2 → 0.45
...
[视觉伺服启动平滑] 迭代 20/20, kp: 0.2 → 1.0
```

观察机械臂是否平滑加速而不是突跳。

### 代码变更总结

| 文件 | 变更类型 | 行数 |
|------|---------|------|
| ArmHandleNodeVisualServoing.hpp | 添加参数字段 | 4 行 |
| ArmHandleNodeVisualServoing.cpp | 参数读取、启动平滑逻辑、日志 | ~40 行 |
| arm_handle_node.cpp | 稳定性检查逻辑 | ~40 行 |

**总代码增加**: ~100 行（主要是日志和稳定性检查）

### 备注

- 所有修改都是**向后兼容**的，参数有默认值
- 可以通过配置文件动态调整，无需重新编译
- 添加了详细日志便于调试和参数调优
