---
name: kuka-rsi
description: Use when working on the KUKA RSI host application (real-time robot position correction via Ethernet), debugging RSIX files, modifying the control law or trajectory planner, building/signing releases, or commissioning on real hardware. Covers the command-ledger architecture, RSIX parameter rules, pose conventions, and release workflow.
---

# KUKA RSI 实时位姿跟踪

## 项目概述

Windows/Qt 上位机，通过 UDP 以太网与 KUKA 机器人控制器通信，在 **4ms IPO 周期**内发送 POSCORR 增量修正，实现实时位姿跟踪。

- **主代码**: `E:\kuka_rsi_win\.claude\worktrees\communication-robustness`
- **技术栈**: C++17, Qt 6.5.3 MinGW, Inno Setup 打包
- **核心模块**: `src/core/` (控制器/轨迹), `src/net/` (通信), `src/ui/` (界面)

## RSIX 文件规则

### IsRuntime 属性

`IsRuntime="false"` 只能出现在 toolbox 声明了 `CanBeSetAtRuntime="false"` 的参数上。完整集合:

| 参数 | 对象 | 规则 |
|------|------|------|
| ConfigFile | ETHERNET | IsRuntime="false" |
| RefCorrSys | POSCORR | IsRuntime="false" |
| IP, Channel | MONITOR | IsRuntime="false" |
| Type, Name, Order, Cutoff | IIRFILTER | IsRuntime="false" |
| DelayTime | DELAY | IsRuntime="false" |

PosCorrMon 的 MaxTrans/MaxRotAngle **不能**带 IsRuntime——加上会导致 `"general error: PosCorrMon_1: MaxTrans"`。

### ParamId 规则

**0-based**，是参数在自身类型空间内的顺序位置。枚举参数（如 RefCorrSys）有自己的索引空间，与数值参数的 ParamId 不冲突。

### RefCorrSys 枚举

`ParamValue` 是数字，不是字符串: World=0, **Base=1**, RobRoot=2, Tool=3, TTS=4。

### 极限保护链

三层递进，从单帧到总距:

| 层 | 对象 | 限值 | 衡量方式 |
|----|------|------|----------|
| 1 | Limit_X~C | ±20mm / ±20° | 单周期增量 |
| 2 | PosCorr LowerLim/UpperLim | ±25mm | 逐轴累积 |
| 3 | PosCorr MaxRotAngle | 25° | 旋转总量 |
| 4 | PosCorrMon MaxTrans/MaxRotAngle | 45mm / 45° | 欧氏总距 |

PosCorr 是逐轴 25mm——三轴各 25mm 的欧氏总距 = 43.3mm < PosCorrMon 的 45mm，保证外层不会先于内层触发。

## 控制架构

### 指令台账 (Command Ledger)

**闭环对象是 `m_cmd`（台账），不是实测 `RIst`。** 这是 2026-08-04 真机抖动修复的核心。

```
误差 = errSrc − m_cmd      ← 相对台账计算
增量 = kp × 误差 → 限幅 → 量化
m_cmd += 增量              ← 台账只记真正发出去的量
```

**稳定性证明**: 台账闭环下，下一周期误差 = `errSrc − (m_cmd + kp × (errSrc − m_cmd))` = `(1−kp) × (errSrc − m_cmd)`。**极点 = 1−kp**，与机器人动力学无关，无条件稳定。kp∈(0,1] 保证不振荡。

**为什么不能相对实测闭环**: RSI 是增量接口，KRC 侧 POSCORR 把增量积分成总修正。误差相对实测计算时，环路等效 = 「积分器 + 伺服/管线滞后」。4ms 周期 kp=0.1 的等效积分增益 = 25/s，远超几十毫秒伺服滞后的稳定边界——真机表现为在 vmax 限幅内满幅来回打（疯狂抖动），且累计指令无界（实验室测: 400 帧发出 240mm）。

**代价**: 网络丢一帧修正 → 台账与 KRC 侧实际总修正产生等量常差 → 误差显示不归零 → 操作员点「归零」重对齐台账。

### 核心数据流

```
操作员设目标 XYZABC
    │
    ▼
setTarget(): 起点 = Tracking? m_cmd : m_lastActual
           时长 = max(配置, 1875×距离/巡航)
           启动五次多项式轨迹
    │
    ▼
每 4ms 周期 step(actual, elapsedMs):
    │
    ├─ 1. 轨迹采样 → errSrc
    │      已完成: 直接返回 m_target
    │      未完成: quintic(s) 插值
    │
    ├─ 2. 误差计算
    │      位置: errPos = errSrc − m_cmd (逐轴)
    │      姿态: qE = quatError(qT, qA) → rotVec (SO(3) 最短旋转向量)
    │
    ├─ 3. 增益 + 限幅 + 量化
    │      dPos = kp × errPos, 欧氏范数 ≤ vmax × Δt
    │      dRot = kp × rotErr, 旋转向量范数 ≤ vmax_rot × Δt
    │      dABC = E⁻¹(m_cmd) × dRot × kRadToDeg
    │      |d| < 1e-4 → 0 (wire quantum 死区)
    │
    ├─ 4. 轨迹推进 advance(elapsed)
    │
    ├─ 5. 台账记账 m_cmd += d
    │
    └─ 6. return d → 发给 KRC
```

### 到位精修 (Settle-and-Trim)

可勾选功能（默认关），只修正路上丢失的微小残差:

1. 轨迹完成 + 增量连续静默 trim_settle_ms (200ms)
2. 残差(target−实测) 在 [trim_min, trim_max] 窗口内 (0.02~2mm/°)
3. 台账重对齐到实测 → 残差经正常管线补发
4. 同一目标最多 trim_max_attempts 次 (3)，间隔 ≥ trim_cooldown_ms (1000ms)

**为什么不会振荡**: 只在静止触发、限次限频——与 4ms 控制环差三个数量级时间尺度，是离散迭代不是连续反馈。

## 轨迹规划

### 五次多项式 + Slerp

```cpp
// 位置: 五次多项式 s(u)
s(u) = 10u³ − 15u⁴ + 6u⁵     u ∈ [0,1]
// s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0

// 姿态: 四元数 Slerp（按同一 s(u) 推进）
q(u) = quatSlerp(q_start, q_goal, s(u))

// 每周期采样
pose = start + s(u) × (goal − start)
```

**性质**: S 形起停，速度/加速度连续，端点加速度为零（五次比三次多的两个自由度正为此）。

### 巡航限速轨迹

配置 `target_cruise_mm_s > 0` 时启用:

```
五次峰值速度 = 1.875 × 平均速度
duration ≥ 1875 × 距离 / cruise_speed (ms)
```

远目标自动拉长时长，使峰值不超巡航速度——不再是 vmax 饱和爬行。

### 时间推进: 墙钟统一

轨迹和步长限值共用 `elapsedMs`（距上次 step 的真实墙钟，上界 cycleMs）。两个时间基必须同源: 积压排空时，只收紧幅值却照旧推进轨迹 → 恢复后第一帧顶格超速。同源后积压排空 41 帧只快 1 帧（= 排空真实占用的 3ms 墙钟），而非 41 帧。

## 姿态数学

### KUKA ABC 约定

ZYX 欧拉: **Rz(A) · Ry(B) · Rx(C)**。四元数构造:

```
q = qz(A) ⊗ qy(B) ⊗ qx(C)
```

### 姿态误差: SO(3) 最短旋转

**不用逐轴差（A_target − A_actual）**——B=±90° 奇异（万向节锁），±180° 跳变产生 360° 假误差。

```cpp
qE = quatError(qTarget, qActual)   // qT ⊗ qA⁻¹, w≥0 (短弧)
rotVec = axis × angle               // 世界坐标 rad
dRot   = kp_rot × rotVec
dABC   = E⁻¹(cmdA, cmdB, cmdC) × dRot  // 世界 ω → 欧拉角变化率
```

### E⁻¹ 矩阵

ZYX 欧拉角速率矩阵的逆: `E = [[0,−sinA,cosA·cosB],[0,cosA,sinA·cosB],[1,0,−sinB]]`, `det(E)=−cosB`。

B 超 ±84° 时 cosB→0，E⁻¹ 含 1/cosB 放大无界 → **退化为阻尼 E⁻¹**（1/cosB → 1/max(|cosB|, 0.1) 保号）。方向始终正确，增益 ≤10×，被范数限幅兜底。

### 增量台账的欧拉角分量加法

```
m_cmd.a += d.a; m_cmd.b += d.b; m_cmd.c += d.c;
```

这是 KRC 侧 POSCORR 行为的精确镜像——KRC 把 RKorr.ABC 逐周期加到被修正设定值的欧拉角分量上。**当前实机验证仅覆盖位置（x/y/z），姿态尚未在真机上做过单轴 ±2° 测试。**

## IPOC 与通信

### IPOC Step Learning

KRC 的 IPOC 毫秒计数器不一定每帧 +1（4ms IPO 时 +0 或 +4 都可能）。主机启动时学习最小正增量作为 per-frame step:

```cpp
const quint64 delta = ipoc − m_lastGood;
if (m_step == 0 || delta < m_step) m_step = delta;
if (delta == m_step) → Normal; else → Gap(gapCount = max(1, delta/step − 1));
```

### 安全机制

| 机制 | 行为 | 触发条件 |
|------|------|----------|
| 物理跳变剔除 | 单帧回零增量（仍回包） | 位移 > phys_vmax × Δt |
| 陈旧帧 Fault | stale 连续 stale_frame_limit 帧 | 仅 Tracking 下; 非 Tracking 只累计 |
| 看门狗 Fault | 可见锁存故障 | 连续丢包 miss_limit (25) 帧 |
| 延时守卫 | 3 帧延时连续上升 → Fault | 可能冲突积压排空 |
| 积压排空守卫 | hasPendingDatagrams 抑制看门狗误报 | 主机停顿后追赶时 |
| 非法帧守卫 | 零增量 + Fault 锁存 | 解析失败帧 |

### 已知边界

- **IPOC 32-bit wrap** (~49 天连续运行): delta 反向洪水，不可恢复，需停止/启动
- **KRL 快速重启** (<2s): 主机误判为 gap recovery 实则 KRC 已重置，台账失效

## 构建与发布

### 构建

```bash
# worktree 路径
cd E:\kuka_rsi_win\.claude\worktrees\communication-robustness

# 构建（需要 Qt 6.5.3 MinGW 在 PATH）
mkdir build && cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Release ..
ninja

# 测试（需要 Qt6Test.dll 在 PATH）
PATH="/d/Software/QT/content/6.5.3/mingw_64/bin:$PATH" ctest --output-on-failure
```

### 数字签名

```powershell
# 使用 tools/sign_dist.ps1（自签名证书 CN=wingljj, DigiCert 时间戳）
powershell -File tools/sign_dist.ps1
```

### 打包

```bash
# Inno Setup（ISCC.exe 路径见 .iss 文件头注释）
"D:/Software/innosetup/content/ISCC.exe" tools/kuka-rsi-installer.iss
# 输出: release/Kuka-RSI-实时位姿跟踪-{version}-离线安装包.exe
```

### GitHub Release (github.com 不可达时)

```bash
# 用 Git Data API 推送标签和发布
python tools/api_push.py
```

### 版本号位置

- `tools/kuka-rsi-installer.iss` → `#define MyAppVersion`
- 无其他硬编码版本号

### dist/config 与仓库默认值的差异

`dist/config/rsi_config.json` 是**手动维护的现场配置**，关键差异:
- `cycle_ms: 4.0` (dist/现场版) vs `12.0` (仓库版) ← 最关键的差异
- 打包脚本从 dist/ 复制，需确保 cycle_ms 与现场 IPO 节拍一致

## 配置速查

### 关键参数

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `cycle_ms` | 4.0 或 12.0 | 必须与示教器显示的实测周期一致 |
| `kp_pos` | 0.1 | 首次联机用，确认方向后再调 |
| `kp_rot` | 0.1 | 同上 |
| `vmax_pos_mm_s` | 10.0 | 速度上限（真正 mm/s，不是每帧配额） |
| `vmax_rot_deg_s` | 2.0 | 姿态速度上限 |
| `watchdog_miss_limit` | 25 | 连续丢包帧数 ≥ 此值 → Fault |
| `target_trajectory_ms` | 1000.0 | 轨迹时长; 0 = 直通（阶跃） |
| `target_cruise_mm_s` | 5.0 | 巡航速度; 0 = 固定时长 |
| `stale_frame_limit` | 10 | 连续陈旧帧数 → Fault |
| `session_gap_ms` | 2000.0 | 静默超此值 = 新会话（重锁锚点） |

### SessionGuard 校验规则

- `kp_pos` ∈ (0, 1], `kp_rot` ∈ (0, 1]
- `vmax × cycle_ms` ≤ 35mm (KRC 单帧极限)
- `watchdog_miss_limit > 0`, `stale_frame_limit > 0`
- `phys_vmax > 0`, `vmax > 0`
- trim 窗口: `trim_min < trim_max`（仅启用时校验）
- `target_cruise ≤ vmax`（仅 >0 时校验）

## 真机调试要点

1. **最先核对 cycle_ms**: 界面上显示的实测周期与配置文件一致——这是最容易出错的一步
2. **首次联机用 kp=0.1 / vmax=10mm/s**: 确认方向正确再往上调
3. **单轴先测**: 每次只动一个轴（X 或 Y），观察方向，不要设 ABC 都非零的复合目标
4. **姿态谨慎**: 单轴 ±2° 先测，确认 E⁻¹ 变换方向正确后再上复合姿态
5. **观察误差显示**: 稳态误差不归零 = 链路丢修正 = 网络或 KRC 侧有问题
6. **到位精修**: 正常情况下几乎从不触发（链路健康时没有残差可修）——如果频繁触发是诊断信号

## 常见陷阱

| 问题 | 现象 | 根因 | 修复 |
|------|------|------|------|
| 真机抖动 | 满幅来回打 | 误差相对实测闭环（非台账） | m_cmd 闭环 |
| 模拟器未暴露 | 同代码模拟 OK、真机炸 | 模拟器无动力学滞后 | 已修复模拟器 fidelity |
| IPOC 显示不连续 | 界面报警 | 误分类：以为每帧 +1 | step learning |
| cycleMs 配错 | vmax 配 10 实际 30mm/s | 旧码按配置周期发配额 | 按墙钟发放 |
| 积压排空超速 | 停顿后首帧顶格 | 幅值限+轨迹混合时间基 | 统一墙钟 |
| 轨迹锚点错 | 运动中改目标倒退 | setTarget 起点用实测非台账 | Tracking 下用 m_cmd |
| 使能吞首帧 | 使能后首个周期无输出 | 轨迹重规划在采样之后 | 重规划移到采样前 |
| 累积量漂移 | 静止后 9mm/h 漂移 | 尾数 < wire quantum 被量化但计入了台账 | 量化死区 1e-4 |
