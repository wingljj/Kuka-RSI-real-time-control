# 姿态架构增强设计（Attitude Architecture Hardening）

日期：2026-08-01
状态：设计已获用户逐节批准

## 背景

四元数姿态轮完成后，用户选择了外部 review（9 点建议）中的 4 个新项继续：
位置范数限幅、目标五次多项式轨迹、反馈异常剔除、7 态状态机。实时性已确认不受
影响（TargetTrajectory 为纯 O(1) 算术，无分配/阻塞/IO）。

## 范围

**做**（用户确认）：
- Part A 位置范数限幅（对齐姿态限幅语义）
- Part B `TargetTrajectory`（位置五次多项式 + 姿态 Slerp，速度/加速度连续）
- Part C 反馈异常剔除（无低通滤波）
- Part D 7 态状态机（Disconnected/WaitingFirstFrame/Syncing/Ready/Tracking/StaleFrame/Fault）

**不做**：
- WirePose/Transform 分层 + SI 单位重构（风险高、收益类型安全，暂缓）
- 反馈四元数低通（相位延迟风险，RIst 无噪声问题）
- 连续旋转累计监控（第 2 层已移除）
- 模拟器可视化（RL 库集成，独立工程，后续）

## Part A：位置范数限幅

`PoseController::step` 位置增量 `d = kp_pos × errPos` 从逐轴 `clampAbs` 改为
**欧氏范数限幅**：`‖d‖ > stepLimitPos` 时按比例缩放。消除三轴同时到限时合成
速度 √3× 超限。姿态已范数限幅（四元数轮），位置对齐。

## Part B：TargetTrajectory

### 类（纯函数，无 Qt 运行时依赖，可单测）

```cpp
class TargetTrajectory {
public:
    void setGoal(const Pose &start, const Pose &goal, double durationMs);
    // 位置：五次多项式 p(t)=p0+s(u)(p1-p0)，s(u)=10u³-15u⁴+6u⁵，u=t/T clamp [0,1]。
    // 姿态：Slerp(q0, q1, s(u))（同一 s，起点终点速度/加速度为 0）。
    Pose sample(double tSec) const;
    bool isFinished(double tSec) const;
private:
    Pose m_start, m_goal;          // 位置 mm + 姿态（A/B/C 度，内部转四元数）
    poseops::Quat m_q0, m_q1;
    double m_durationS = 0;
};
```

### PoseController 集成

- `applyTarget`（目标变化）→ `setGoal(当前实际位姿, 新目标, 时长)` 启动轨迹。
  start 用**当前实际**（从实际出发，避免从旧目标起跳）。
- `step` 每周期 `sample(会话时间)` 得到平滑目标 → 误差 = 平滑目标 vs 实际。
  轨迹完成（`isFinished`）后目标 = 最终目标（静止）。
- **替代**当前 `m_smoothTarget` 一阶低通（`m_alpha` 机制删除）。
- `resetToActual`/`beginSession`：轨迹立即完成（目标=实际）。
- 语义：从"指数逼近"（永远追不上）改为"固定时长轨迹"（到点即达）。

### 实时性（已确认）

`sample` 为纯 O(1) 算术：五次多项式（~15 乘加）+ Slerp（1 次 acos + 1 次 sin/cos）。
状态为 POD，局部栈变量，无堆分配/锁/IO。每周期微秒级，与现平滑器相当。

## Part C：反馈异常剔除（无低通）

`RsiWorker` 每有效帧检查 RIst 相对**上一有效帧**的跳变：

- 位置：`‖p_now − p_prev‖ > phys_vmax_pos × dt + margin`
- 旋转：`‖rotVec(quatError(q_now, q_prev))‖ > phys_vmax_rot × dt + margin`

超限 → **本周期回零增量**（不 `step`，不推进控制器）+ `stale` 连续计数；
**连续 `stale_frame_limit`（默认 10）次 → `forceFault`**（"feedback stale frames"）。
恢复（连续正常帧）→ 计数清零回 Tracking。

首帧/会话重启后首帧无可比帧，不检查。margin 默认 0（物理极限本身含余量）。

## Part D：7 态状态机

`TrackState` → `ControlState`：

```
Disconnected      未绑定
WaitingFirstFrame 已绑定未收帧
Syncing           首帧（目标同步实际）
Ready             可使能（等待操作员）
Tracking          正常 step
StaleFrame        异常帧（本周期回零增量）
Fault             锁存（resetToActual → Ready）
```

迁移：
- `Disconnected → WaitingFirstFrame`：bind 成功
- `WaitingFirstFrame → Syncing`：首帧（目标同步实际）→ `Ready`
- `Ready → Tracking`：使能
- `Tracking → StaleFrame`：异常帧（回零）；`StaleFrame → Tracking`：正常帧恢复；
  连续异常 → `Fault`
- 任意 → `Fault`：配置无效/非有限/KRC Delay/写失败/forceFault
- `Fault → Ready`：resetToActual（归零并复位）

UI 状态卡颜色：Disconnected 灰 / WaitingFirstFrame+Syncing 蓝 / Ready+Tracking 绿 /
StaleFrame 黄 / Fault 红。使能按钮状态机按 Ready/Tracking/Fault 映射（现有逻辑调整）。

## 配置字段（新增/语义变化）

| 字段 | 类型 | 默认 | 说明 |
|---|---|---|---|
| `target_trajectory_ms` | double | 1000 | 轨迹时长（替代 `target_smoothing_ms` 的指数低通语义；字段名变化，旧字段移除） |
| `phys_vmax_pos_mm_s` | double | 500 | 异常剔除位置物理极限 |
| `phys_vmax_rot_deg_s` | double | 60 | 异常剔除旋转物理极限 |
| `stale_frame_limit` | int | 10 | 连续异常帧 Fault 阈值 |

## 测试

- 新增 `tests/test_target_trajectory.cpp`：
  - 起点/终点值精确（u=0 → start，u=1 → goal）
  - 位置五次多项式：起点终点速度/加速度为 0（数值差分验证 s'(0)=s'(1)=0）
  - 姿态 Slerp：u=0/1 精确；中点 = 最短弧
  - `isFinished`
- `test_pose_controller`：
  - 位置范数限幅（三轴同限时合成范数 ≤ stepLimitPos）
  - 轨迹目标采样（阶跃目标 → 平滑目标按轨迹推进）
  - 状态机迁移（Ready→Tracking→StaleFrame→Fault→reset→Ready）
  - 现有用例适配（平滑器语义变化：`target_smoothing_ms` 测试改轨迹语义）
- `test_app_config`：新字段读取
- 异常剔除：RsiWorker 层（socket harness 受限，跳变检测抽纯函数测——`core/` 新
  函数如 `bool exceedsPhysicalJump(const Pose &a, const Pose &b, double dt, ...)`）
- 回归：verify_kinematics / verify_robustness

## 文件变更

- Create: `src/core/TargetTrajectory.h/.cpp`（挂 rsi_core）
- Modify: `src/core/PoseController.h/.cpp`（位置范数 + 轨迹 + ControlState）
- Modify: `src/core/PoseController.h` 的 `TrackState` → `ControlState`（或新枚举）
- Modify: `src/net/RsiWorker.cpp`（异常剔除 + 状态判定）
- Modify: `src/net/SharedState.h`（state 类型 + stale 计数）
- Modify: `src/ui/MainWindow.cpp`（状态卡颜色 + 使能状态机）
- Modify: `src/core/AppConfig.h/.cpp` + `config/rsi_config.json`
- Modify: `tools/loopback_test/main.cpp`（stateName 扩展）
- Modify: `tests/*`

## 分阶段

1. Part A 位置范数限幅 + 测试
2. Part B TargetTrajectory + 测试 + PoseController 集成（替换平滑器）
3. Part C 异常剔除（抽纯函数 + RsiWorker 接线）+ 配置
4. Part D 7 态状态机（枚举 + 迁移 + UI）
5. 回归（全单测 + verify_kinematics + verify_robustness）+ 文档

每阶段独立构建验证 + 提交。
