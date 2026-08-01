# 姿态架构增强 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 落实姿态架构 4 项增强：位置范数限幅、TargetTrajectory（五次多项式+Slerp）、反馈异常剔除、7 态状态机。

**Architecture:** 新增纯函数 `TargetTrajectory`（增量式轨迹）与 `poseops::exceedsPhysicalJump`（跳变检测）；`PoseController` 位置范数限幅 + 轨迹采样替代一阶低通；`RsiWorker` 异常剔除 + `ControlState` 会话状态机（PoseController 内部 TrackState 保留）；UI 状态卡按 ControlState 映射。

**Tech Stack:** C++17 / Qt 6.5.3 / CMake / Ninja / Qt Test。分支 `feature/communication-robustness`。

## Global Constraints

- **实时路径**（RsiWorker 通信线程）：禁堆分配/阻塞/IO。`TargetTrajectory` 与跳变检测为纯 O(1) 算术，POD 状态。
- **位置范数限幅**：`‖d_pos‖ ≤ m_stepLimitPos`（mm/周期），超限缩放，非逐轴 clamp。
- **TargetTrajectory 增量式**：`setGoal(start, goal, durMs)` / `advance(dtS)` / `sample()` / `isFinished()`；位置五次多项式 `s(u)=10u³-15u⁴+6u⁵`；姿态 Slerp（同一 s）。
- **`target_smoothing_ms` 字段改名 `target_trajectory_ms`**（语义从指数低通变为轨迹时长）；`m_alpha`/`m_smoothTarget` 机制删除。
- **异常剔除**：单帧位置/旋转跳变超物理极限 → 本周期回零增量（仍回包）+ stale 计数；连续 `stale_frame_limit` → `forceFault`。首帧/会话重启首帧不检查。
- **7 态 `ControlState`**：Disconnected/WaitingFirstFrame/Syncing/Ready/Tracking/StaleFrame/Fault。PoseController 内部 `TrackState`（Idle/Tracking/Fault）保留（控制语义），`ControlState` 由 RsiWorker 组合（会话态 + 控制态 + stale 标记）。UI 状态卡颜色：灰/蓝/蓝/绿/绿/黄/红。
- 测试用文件日志器；构建 PATH：MINGW/NINJA/QTBIN。

---

### Task 1: Part A — 位置范数限幅

**Files:**
- Modify: `src/core/PoseController.cpp`
- Modify: `tests/test_pose_controller.cpp`

**Interfaces:**
- Produces: `step()` 位置增量欧氏范数限幅（替代逐轴 clampAbs）。Task 2 依赖（同一 d 构造处）。

- [ ] **Step 1: Modify `PoseController.cpp`**

`step()` 里位置增量构造改为：

```cpp
    // 第 1 层限值：位置按欧氏范数限幅（三轴同时到限时合成速度不超 √3×——
    // 逐轴 clamp 会让对角运动达到 √3×limit），姿态按旋转向量范数限幅。
    Pose d;
    d.x = m_cfg.kpPos * errPos.x;
    d.y = m_cfg.kpPos * errPos.y;
    d.z = m_cfg.kpPos * errPos.z;
    const double posNorm = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (m_stepLimitPos > 0.0 && posNorm > m_stepLimitPos) {
        const double s = m_stepLimitPos / posNorm;
        d.x *= s; d.y *= s; d.z *= s;
    }
```

（替换现有的三行 `clampAbs` 位置增量。）

- [ ] **Step 2: Add regression test**

`tests/test_pose_controller.cpp` 加：

```cpp
    void positionDiagLimit_usesEuclideanNorm()
    {
        // 三轴各 0.5mm 误差：逐轴 clamp（0.6 限）各 0.5 → 合成 0.866 超限；
        // 范数限幅应把合成压到 ≤ 0.6（每轴 ~0.346）。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 0.0;
        c.kpPos = 1.0;
        c.vmaxPosMmS = 50.0;              // 12ms → 0.6mm/周期
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0.5, 0.5, 0.5, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        const double norm = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
        QVERIFY(norm <= 0.6 + 1e-9);
        QVERIFY(qAbs(d.x - d.y) < 1e-9);  // 等比缩放
    }
```

- [ ] **Step 3: Build + run**

```bash
cmake --build build
./build/tests/test_pose_controller.exe -o .superpowers/sdd/aa-t1.log,txt; cat .superpowers/sdd/aa-t1.log
```

Expected: 全绿（含新用例；现有位置用例如 `largeError_isClampedToStepLimit` 单轴不受影响）。

- [ ] **Step 4: Commit**

```bash
git add src/core/PoseController.cpp tests/test_pose_controller.cpp
git commit -m "feat(core): position increment Euclidean-norm limit"
```

---

### Task 2: Part B — TargetTrajectory + PoseController 集成

**Files:**
- Create: `src/core/TargetTrajectory.h`
- Create: `src/core/TargetTrajectory.cpp`
- Create: `tests/test_target_trajectory.cpp`
- Modify: `CMakeLists.txt`（rsi_core 加 TargetTrajectory.cpp）
- Modify: `tests/CMakeLists.txt`（加 test_target_trajectory）
- Modify: `src/core/AppConfig.h/.cpp`（`target_smoothing_ms` → `target_trajectory_ms`）
- Modify: `config/rsi_config.json`
- Modify: `src/core/PoseController.h/.cpp`（集成，删 m_alpha/m_smoothTarget）
- Modify: `tests/test_pose_controller.cpp`

**Interfaces:**
- Produces: `class TargetTrajectory { void setGoal(const Pose&,const Pose&,double durMs); void advance(double dtS); Pose sample() const; bool isFinished() const; }`。Task 3/4 依赖（PoseController 目标来源）。

- [ ] **Step 1: Write `TargetTrajectory.h`**

```cpp
#pragma once
#include "core/Pose.h"
#include "core/PoseOps.h"

// 目标轨迹：位置五次多项式 + 姿态 Slerp（速度/加速度连续），增量式推进。
// 纯 O(1) 算术、POD 状态，可在实时路径调用（无分配/阻塞/IO）。
class TargetTrajectory
{
public:
    void setGoal(const Pose &start, const Pose &goal, double durationMs);
    void advance(double dtS);          // 每周期推进轨迹时间
    Pose sample() const;               // 当前平滑目标（完成时 = goal）
    bool isFinished() const { return m_u >= 1.0; }

private:
    Pose m_start, m_goal;
    poseops::Quat m_q0, m_q1;
    double m_tS    = 0.0;
    double m_durS  = 0.0;
    double m_u     = 1.0;   // 归一化进度 [0,1]，未启动即完成
};
```

- [ ] **Step 2: Write `TargetTrajectory.cpp`**

```cpp
#include "core/TargetTrajectory.h"

#include <cmath>

namespace {
// 五次多项式：s(u)=10u³-15u⁴+6u⁵，s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0。
double quintic(double u)
{
    return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}
} // namespace

void TargetTrajectory::setGoal(const Pose &start, const Pose &goal, double durationMs)
{
    m_start = start;
    m_goal  = goal;
    m_q0    = poseops::quatFromABC(start.a, start.b, start.c);
    m_q1    = poseops::quatFromABC(goal.a, goal.b, goal.c);
    m_durS  = durationMs > 0.0 ? durationMs / 1000.0 : 0.0;
    m_tS    = 0.0;
    m_u     = m_durS > 0.0 ? 0.0 : 1.0;
}

void TargetTrajectory::advance(double dtS)
{
    if (m_u >= 1.0 || dtS <= 0.0)
        return;
    m_tS += dtS;
    m_u = std::min(1.0, m_tS / m_durS);
}

Pose TargetTrajectory::sample() const
{
    if (m_u >= 1.0)
        return m_goal;
    const double s = quintic(m_u);
    Pose p;
    p.x = m_start.x + s * (m_goal.x - m_start.x);
    p.y = m_start.y + s * (m_goal.y - m_start.y);
    p.z = m_start.z + s * (m_goal.z - m_start.z);
    const poseops::Quat q = poseops::quatSlerp(m_q0, m_q1, s);
    poseops::abcFromQuat(q, &p.a, &p.b, &p.c);
    return p;
}
```

（`poseops::quatSlerp` 若不存在，在 `PoseOps` 加：`Quat quatSlerp(const Quat&, const Quat&, double t)` —— 实现：`angle=acos(dot)，t=0/1 边界，q = (sin((1-t)a)q0 + sin(ta)q1)/sin(a)`，dot<0 取反 q1。加测试。）

- [ ] **Step 3: AppConfig 字段改名**

`AppConfig.h`：`double targetSmoothingMs = 50.0;` → `double targetTrajectoryMs = 1000.0;`
`AppConfig.cpp`：`readDouble(rsi, "target_smoothing_ms", ...)` → `"target_trajectory_ms"`
`config/rsi_config.json`：`"target_smoothing_ms": 50.0` → `"target_trajectory_ms": 1000.0`（注释更新：轨迹时长）
`test_pose_controller` 的 `testCfg()`：`c.targetSmoothingMs = 0.0` → `c.targetTrajectoryMs = 0.0`（0 = 轨迹立即完成 = 直通）

- [ ] **Step 4: `PoseController` 集成**

`PoseController.h`：删 `m_smoothTarget`/`m_alpha`；加成员 `TargetTrajectory m_traj;`、`Pose m_lastActual;`。
`configure()`：删 alpha 计算。
`resetToActual()`/`beginSession()`：`m_traj.setGoal(actual, actual, 0)`（立即完成）+ `m_lastActual = actual`。
`setTarget(t)`：
```cpp
    void setTarget(const Pose &t)
    {
        if (t.x != m_target.x || t.y != m_target.y || t.z != m_target.z
            || t.a != m_target.a || t.b != m_target.b || t.c != m_target.c) {
            m_traj.setGoal(m_lastActual, t, m_cfg.targetTrajectoryMs);
            m_target = t;
        }
    }
```
`step()`：平滑块替换为轨迹采样：
```cpp
    // 目标来源：轨迹未完成时用轨迹采样（五次多项式 + Slerp），完成即最终目标。
    Pose errSrc = m_traj.isFinished() ? m_target : m_traj.sample();
    if (!m_traj.isFinished())
        m_traj.advance(m_cfg.cycleMs / 1000.0);
```
（`m_lastActual = actual;` 在 step 末尾更新。）

- [ ] **Step 5: `test_target_trajectory.cpp`**

```cpp
#include <QtTest>
#include <cmath>
#include "core/TargetTrajectory.h"
#include "core/PoseOps.h"

class TestTrajectory : public QObject
{
    Q_OBJECT
private slots:
    void endpointsExact()
    {
        TargetTrajectory tr;
        tr.setGoal(Pose{0,0,0,0,0,0}, Pose{100,50,0,30,-20,10}, 1000);
        QCOMPARE(tr.sample().x, 0.0);
        for (int i = 0; i < 100; ++i) tr.advance(0.01);
        const Pose p = tr.sample();
        QVERIFY(qAbs(p.x - 100) < 1e-6);
        QVERIFY(qAbs(p.y - 50) < 1e-6);
        QVERIFY(tr.isFinished());
    }

    void quinticZeroVelAtEnds()
    {
        // 数值差分：u 接近 0/1 时速度 ≈ 0
        TargetTrajectory tr;
        tr.setGoal(Pose{0,0,0,0,0,0}, Pose{100,0,0,0,0,0}, 1000);
        tr.advance(0.001);
        const double vEarly = tr.sample().x / 0.001;
        // 推进到接近结束
        for (int i = 0; i < 998; ++i) tr.advance(0.001);
        tr.advance(0.001);
        const double vLate = (100.0 - tr.sample().x) / 0.001;
        QVERIFY(qAbs(vEarly) < 0.1);
        QVERIFY(qAbs(vLate) < 0.1);
    }

    void slerpMidpoint()
    {
        // 90° 旋转：u=0.5 时中点 45°
        TargetTrajectory tr;
        tr.setGoal(Pose{0,0,0,0,0,0}, Pose{0,0,0,0,0,90}, 1000);
        tr.advance(0.5);
        const Pose p = tr.sample();
        QVERIFY(qAbs(p.c - 45.0) < 1e-6);
    }

    void zeroDurationImmediate()
    {
        TargetTrajectory tr;
        tr.setGoal(Pose{1,2,3,0,0,0}, Pose{9,8,7,0,0,0}, 0);
        QVERIFY(tr.isFinished());
        QCOMPARE(tr.sample().x, 9.0);
    }
};
QTEST_MAIN(TestTrajectory)
#include "test_target_trajectory.moc"
```

- [ ] **Step 6: `test_pose_controller` 适配**

现有平滑测试（`smoothing_progressivelyApproachesStepTarget`、`smoothing_tauZero_isPassthrough`、`resetToActual_syncsSmoothTarget`、`smoothing_doesNotChangeSteadyState`、`smoothing_angularJumpTakesShortestPath`）：
- `smoothing_tauZero_isPassthrough`：`targetTrajectoryMs=0` → 直通（保持）。
- `smoothing_progressivelyApproachesStepTarget`：轨迹语义（首周期增量更小，五次多项式起点速度 0）。调整断言或删除（轨迹行为由 test_target_trajectory 覆盖）。
- `resetToActual_syncsSmoothTarget`：resetToActual 立即完成轨迹 → 误差 0（保持语义）。
- `smoothing_doesNotChangeSteadyState`：轨迹完成后稳态不变（保持）。
- `smoothing_angularJumpTakesShortestPath`：轨迹姿态 Slerp 最短弧（保持语义，数值可能变——调整）。
- 实现者按新语义调整这些用例，保留核心断言（稳态、直通、reset 同步）。

- [ ] **Step 7: Build + full suite**

```bash
cmake --build build
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state test_ipoc_tracker test_session_guard test_kr210_kinematics test_pose_ops test_target_trajectory; do
  ./build/tests/$t.exe -o .superpowers/sdd/aa-t2-$t.log,txt
done
```

Expected: 全绿。

- [ ] **Step 8: Commit**

```bash
git add CMakeLists.txt tests/CMakeLists.txt src/core/TargetTrajectory.h src/core/TargetTrajectory.cpp tests/test_target_trajectory.cpp src/core/AppConfig.h src/core/AppConfig.cpp config/rsi_config.json src/core/PoseController.h src/core/PoseController.cpp tests/test_pose_controller.cpp
git commit -m "feat(core): target trajectory (quintic + slerp) replaces first-order smoothing"
```

---

### Task 3: Part C — 反馈异常剔除

**Files:**
- Modify: `src/core/PoseOps.h/.cpp`（加 `exceedsPhysicalJump`）
- Modify: `src/net/RsiWorker.h/.cpp`
- Modify: `src/core/AppConfig.h/.cpp` + `config/rsi_config.json`
- Modify: `tests/test_pose_ops.cpp`

**Interfaces:**
- Produces: `bool poseops::exceedsPhysicalJump(const Pose &prev, const Pose &now, double dtS, double vmaxPosMmS, double vmaxRotDegS)`；`RsiWorker` stale 计数与 Fault。Task 4 的 StaleFrame 状态依赖。

- [ ] **Step 1: `PoseOps` 加跳变检测**

`PoseOps.h` 加：

```cpp
// 反馈异常剔除：单帧位置/旋转跳变是否超物理极限（v_max × dt）。
// dt<=0 或 prev==now 返回 false。纯 O(1) 算术，实时路径可调用。
bool exceedsPhysicalJump(const Pose &prev, const Pose &now, double dtS,
                         double vmaxPosMmS, double vmaxRotDegS);
```

`PoseOps.cpp`：

```cpp
bool exceedsPhysicalJump(const Pose &prev, const Pose &now, double dtS,
                         double vmaxPosMmS, double vmaxRotDegS)
{
    if (dtS <= 0.0)
        return false;
    const double dx = now.x - prev.x, dy = now.y - prev.y, dz = now.z - prev.z;
    const double dp = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (dp > vmaxPosMmS * dtS)
        return true;
    const Quat qP = quatFromABC(prev.a, prev.b, prev.c);
    const Quat qN = quatFromABC(now.a, now.b, now.c);
    const Quat qE = quatError(qN, qP);
    double rot[3];
    rotVecFromQuat(qE, rot);
    const double ang = std::sqrt(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2]);
    return ang > vmaxRotDegS * dtS * kDegToRad;
}
```

（PoseOps.cpp 需 `#include "core/Pose.h"`。）

- [ ] **Step 2: AppConfig 新字段**

`AppConfig.h` 加：`double physVmaxPosMmS = 500.0;`、`double physVmaxRotDegS = 60.0;`、`int staleFrameLimit = 10;`
`AppConfig.cpp`：`readDouble(ctl, "phys_vmax_pos_mm_s", ...)`、`readDouble(ctl, "phys_vmax_rot_deg_s", ...)`、`readInt(ctl, "stale_frame_limit", ...)`
`config/rsi_config.json` control 段加三个字段 + 注释。

- [ ] **Step 3: `RsiWorker` 接线**

`RsiWorker.h` 成员：`Pose m_prevValidPose; bool m_havePrevPose = false; int m_staleCount = 0;`

`onDatagram()` 有效帧分支（`switch(ev.kind)` 之后、`delta = step` 之前）加：

```cpp
            // 反馈异常剔除：单帧跳变超物理极限 → 本周期回零增量 + stale 计数；
            // 连续超限 → Fault。首帧/会话重启首帧无可比帧，不检查。
            bool stale = false;
            if (m_havePrevPose
                && poseops::exceedsPhysicalJump(
                    m_prevValidPose, f.rist, m_cfg.cycleMs / 1000.0,
                    m_cfg.physVmaxPosMmS, m_cfg.physVmaxRotDegS)) {
                stale = true;
                if (++m_staleCount >= m_cfg.staleFrameLimit
                    && m_ctl.state() == TrackState::Tracking) {
                    m_ctl.forceFault(QStringLiteral(
                        "feedback stale frames (jump beyond physical limit)"));
                }
            } else {
                m_staleCount = 0;
            }
            m_prevValidPose = f.rist;
            m_havePrevPose  = true;
```

并在 `delta` 计算处：`if (ev.kind == Normal || Gap)` 分支改为 `if (!stale && (ev.kind == Normal || ev.kind == Gap))`（异常帧回零增量）。

`start()` 重置 `m_havePrevPose=false; m_staleCount=0;`。

- [ ] **Step 4: `test_pose_ops` 加跳变检测用例**

```cpp
    void exceedsPhysicalJump_detectsPositionAndRotationJumps()
    {
        const Pose a{0,0,0,0,0,0};
        Pose b = a; b.x = 600.0;                       // 600mm 单帧 > 500mm/s×1s
        QVERIFY(poseops::exceedsPhysicalJump(a, b, 1.0, 500.0, 60.0));
        b = a; b.x = 100.0;                            // 100mm < 500 → 位置正常
        QVERIFY(!poseops::exceedsPhysicalJump(a, b, 1.0, 500.0, 60.0));
        b = a; b.a = 90.0;                             // 90° 单帧 > 60°/s×1s
        QVERIFY(poseops::exceedsPhysicalJump(a, b, 1.0, 500.0, 60.0));
        b = a; b.a = 30.0;
        QVERIFY(!poseops::exceedsPhysicalJump(a, b, 1.0, 500.0, 60.0));
        QVERIFY(!poseops::exceedsPhysicalJump(a, a, 1.0, 500.0, 60.0));
    }
```

- [ ] **Step 5: Build + full suite**

```bash
cmake --build build
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state test_ipoc_tracker test_session_guard test_kr210_kinematics test_pose_ops test_target_trajectory; do
  ./build/tests/$t.exe -o .superpowers/sdd/aa-t3-$t.log,txt
done
```

Expected: 全绿。

- [ ] **Step 6: Commit**

```bash
git add src/core/PoseOps.h src/core/PoseOps.cpp src/net/RsiWorker.h src/net/RsiWorker.cpp src/core/AppConfig.h src/core/AppConfig.cpp config/rsi_config.json tests/test_pose_ops.cpp
git commit -m "feat(core): stale-frame rejection — physical-jump detection, zero delta, consecutive-fault"
```

---

### Task 4: Part D — 7 态 ControlState + UI

**Files:**
- Modify: `src/net/SharedState.h`（`ControlState` 枚举 + `state` 字段类型）
- Modify: `src/net/RsiWorker.cpp`（组合 ControlState）
- Modify: `src/ui/MainWindow.cpp`（状态卡颜色 + 使能按钮）
- Modify: `tools/loopback_test/main.cpp`（stateName）

**Interfaces:**
- Produces: `enum class ControlState { Disconnected, WaitingFirstFrame, Syncing, Ready, Tracking, StaleFrame, Fault };` `StatusSnapshot.state` 改为此类型。Task 5 回归依赖。

- [ ] **Step 1: `SharedState.h` 定义 ControlState**

`StatusSnapshot` 的 `state` 字段从 `TrackState` 改为 `ControlState`；保留 `TrackState`（PoseController 内部用）：

```cpp
// 7 态会话/控制状态（UI 显示 + 行为语义）
enum class ControlState {
    Disconnected, WaitingFirstFrame, Syncing, Ready, Tracking, StaleFrame, Fault,
};
```

- [ ] **Step 2: `RsiWorker` 组合状态**

`publishSnapshot` 里计算 `ControlState`：

```cpp
    ControlState cs;
    if (m_ctl.state() == TrackState::Fault) {
        cs = ControlState::Fault;
    } else if (m_staleCount > 0) {
        cs = ControlState::StaleFrame;
    } else if (m_ctl.state() == TrackState::Tracking) {
        cs = ControlState::Tracking;
    } else if (m_haveFirstFrame) {
        cs = ControlState::Ready;
    } else if (m_peerLocked) {
        cs = ControlState::WaitingFirstFrame;
    } else {
        cs = ControlState::Disconnected;
    }
    s.state = cs;
```

（Syncing 是首帧瞬间，不持续显示——Ready 承载；如需在首帧同步目标瞬间显示 Syncing，可在 `wasFirstFrame` 时发布一次 `Syncing`，但立即转 Ready。实现者按此语义。）

- [ ] **Step 3: `MainWindow` 适配**

- 状态卡判色：`Fault` 红 / `StaleFrame` 黄（degraded 条件并入）/ `Tracking`+`Ready` 绿 / `WaitingFirstFrame`+`Syncing` 蓝 / `Disconnected` 灰。
- 使能按钮状态机：`Fault` → 「归零并复位」；`Tracking` → 「已使能跟踪」禁用；`Ready` → 「准备跟踪」可用；其它禁用。
- 现有 `s.state == TrackState::Fault` 判断改 `s.state == ControlState::Fault`；`degraded` 黄条件并入 `StaleFrame`。

- [ ] **Step 4: `loopback_test` stateName**

`stateName` 扩展 7 态字符串。

- [ ] **Step 5: Build + full suite + e2e**

```bash
cmake --build build
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state test_ipoc_tracker test_session_guard test_kr210_kinematics test_pose_ops test_target_trajectory; do
  ./build/tests/$t.exe -o .superpowers/sdd/aa-t4-$t.log,txt
done
bash tools/verify_kinematics.sh
bash tools/verify_robustness.sh
```

Expected: 全绿 + 两套 e2e PASS（注意 verify 脚本 grep 的 `state=Fault`/`state=Tracking` 字符串——stateName 输出格式保持 `Fault`/`Tracking` 字样，7 态名与该两处兼容）。

- [ ] **Step 6: Commit**

```bash
git add src/net/SharedState.h src/net/RsiWorker.cpp src/ui/MainWindow.cpp tools/loopback_test/main.cpp
git commit -m "feat(net): 7-state ControlState with UI mapping"
```

---

### Task 5: 回归 + 文档

**Files:**
- Modify: `docs/real-machine-deployment.md`（配置字段表 + 状态机说明）
- Modify: `docs/verification-matrix.md`（新增验证项）

- [ ] **Step 1: 文档**

`real-machine-deployment.md`：§7 配置表加 `target_trajectory_ms`/`phys_vmax_*`/`stale_frame_limit`；状态说明（7 态 + 异常帧行为）。
`verification-matrix.md`：加轨迹/异常剔除/状态机验证行。

- [ ] **Step 2: 全量回归**

```bash
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state test_ipoc_tracker test_session_guard test_kr210_kinematics test_pose_ops test_target_trajectory; do
  ./build/tests/$t.exe -o .superpowers/sdd/aa-t5-$t.log,txt
done
bash tools/verify_kinematics.sh
bash tools/verify_robustness.sh
```

Expected: 全绿 + 两套 e2e PASS。

- [ ] **Step 3: Commit**

```bash
git add docs/real-machine-deployment.md docs/verification-matrix.md
git commit -m "docs: trajectory, stale-frame rejection, 7-state machine"
```

---

## Self-Review 记录

- **Spec 覆盖**：Part A（Task 1）、Part B（Task 2）、Part C（Task 3）、Part D（Task 4）、回归/文档（Task 5）。无缺口。
- **占位符**：无 TBD/TODO。Task 2 的 `quatSlerp` 标注"若不存在则加"（实现者按需补，含测试）。
- **类型一致**：`TargetTrajectory::setGoal/advance/sample/isFinished`、`ControlState` 7 值、`exceedsPhysicalJump` 签名在任务间一致。
- **风险注意**：
  - Task 2 删 `m_alpha`/`m_smoothTarget` 后，`smoothing_*` 测试语义变化（按新轨迹语义调整，保留核心断言）。
  - Task 4 的 `ControlState` 改变 `StatusSnapshot.state` 类型——所有 `s.state` 消费点（MainWindow、loopback_test）同步改；verify 脚本 grep 的 `state=Fault`/`state=Tracking` 字符串需保持兼容（stateName 输出）。
  - 异常剔除的 `m_staleCount` 在 StaleFrame 显示（>0 即 StaleFrame）与连续 Fault 阈值共用——Fault 优先。
  - `target_smoothing_ms` → `target_trajectory_ms` 字段改名影响所有引用（config/AppConfig/测试）。

