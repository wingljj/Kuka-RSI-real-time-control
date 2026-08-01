# KUKA KR210 R3100 模拟器增强设计

日期：2026-08-01
状态：设计已获用户逐节批准

## 背景

现有 `tools/krc_simulator`（假 KRC）能发 `<Rob>`、收 `<Sen>` 累加 RKorr 做基础闭环，
但它是**无约束点模型**——没有真实几何、关节限位、速度/加速度限制。用户要求增强为
仿照 **KUKA KR210 R3100** 真实运动学的模拟器，在更真实的物理约束下充分验证主机
（限幅、平滑器、联锁、限值梯度）的正确性。

## 范围

**做**（在现有 `krc_simulator` 上增强，默认行为不变——所有新约束默认关闭）：
- KR210 R3100 DH 运动学（正解 + 雅可比伪逆）
- 关节限位（KR210 各轴范围，越限 clamp 继续发帧）
- 笛卡尔额外约束（可选位置/姿态范围，clamp）
- 速度/加速度限制（验证主机限幅 + 平滑器）
- 会话重启模拟（IPOC 重置）
- 通讯增强（AIPos/ASPos、IPOC 回绕、节拍抖动）
- 内置正解自检
- 配套验证（更新/扩展 verify_robustness.sh）

**不做**：
- 逆运动学（解析逆解）——用雅可比伪逆做笛卡尔→关节，避免球腕解析逆解复杂度
- 主动运动模式（用户确认：仅主机驱动）
- 完整 RSI 语义（如 IPOC 校验策略、Timeout 内部机制）——保留现有帧级模拟
- GUI 可视化

## 关键技术决策（用户逐项确认）

| # | 决策 | 内容 |
|---|---|---|
| 1 | 方向 | 现有 krc_simulator 增强 + 真实约束 |
| 2 | 运动学 | 仿照 KR210 R3100：DH 正解 + 雅可比伪逆 |
| 3 | 限位行为 | clamp 停住继续发帧（默认，可选断链未采用） |
| 4 | 驱动 | 仅主机驱动（模拟器响应 RKorr，目标由主机 GUI 设定） |

## 运动学：KR210 R3100

### DH 参数（modified DH，标准 KR 210 R3100 ultra）

| 轴 | α | a (mm) | d (mm) | 关节限位 |
|---|---|---|---|---|
| 1 | −90° | 350 | 675 | ±185° |
| 2 | 0° | 1150 | 0 | +155° … −135° |
| 3 | −90° | 41 | 0 | +75° … −175° |
| 4 | 90° | 0 | 1200 | ±350° |
| 5 | 90° | 0 | 0 | ±122.5° |
| 6 | 0° | 0 | 240 | ±350° |

### 正解（关节 → 笛卡尔）

- DH 连乘得齐次变换 `T = T1·T2·...·T6`。
- 位置 = T 的平移部分；姿态 = T 的旋转矩阵转 **KUKA A/B/C 欧拉角**：
  `R = Rz(A)·Ry(B)·Rx(C)`（ZYX 约定，与 RIst 的 A/B/C 一致）。
  - 注意奇异：B = ±90° 时 A/C 耦合，取 A=0 分支（与 KUKA 行为一致）。

### 雅可比伪逆（笛卡尔 RKorr → 关节增量）

- 标准 6 轴几何雅可比 `J(q)`（6×6：前 3 行位置、后 3 行姿态），从 DH 推导。
- 每周期：`Δq = J⁺(q) · Δx`，`J⁺` 为伪逆（非奇异时 `J⁻¹`；奇异/近奇异时
  `J⁺ = Jᵀ(JJᵀ)⁻¹` 加阻尼 `λ` 防止爆炸）。
- 关节增量受限幅/加速度限制后 `q += Δq`。

### 初始位形

- 默认 `q = [0, −60°, 30°, 0, 90°, 0]`（腕部非奇异；原 `[0,−45°,45°,0,0,0]` 因 q5=0 处于腕部奇异 σ_min=0，且原阻尼 λ≈1.6 对腕部模式衰减过强——Task 1 修正）。
- `--init-joints "q1 q2 q3 q4 q5 q6"`（度）覆盖。

## 约束模型

### 关节限位（KR210）

- 每周期 `q` 越限即 clamp 到限位内。clamp 后笛卡尔位姿由正解重新算出
  → 主机看到 RIst 停在限位、误差不再收敛（间接暴露主机限幅是否正确）。
- 越限时打印一次警告（避免刷屏）。

### 笛卡尔额外约束（可选）

- `--cart-limits "xmin xmax ymin ymax zmin zmax amin amax bmin bmax cmin cmax"`
  （默认关闭）。越限 clamp，同样由正解反映。

### 速度/加速度限制（验证主机限幅 + 平滑器）

- 对 RKorr 增量先速度限制（每周期 |Δ| ≤ `max_vel × cycle`）再加速度限制
  （变化率 |Δ−Δprev| ≤ `max_accel × cycle²`），作用于关节增量 `Δq`。
- 开关：`--max-vel-pos/--max-vel-rot`（mm/s、°/s）、
  `--max-accel-pos/--max-accel-rot`（mm/s²、°/s²）。默认 0 = 无限制（现状）。
- 效果：主机发大增量时模拟器响应受限 → RIst 滞后 → 验证主机平滑器/限幅在
  物理约束下正确（不震荡、误差合理收敛、不过冲）。

## 会话重启模拟

- `--restart-at-ms N --restart-gap-ms M`：N ms 后停帧 M ms 再恢复，**IPOC 重置**
  （模拟 KRL 程序重启、新 RSI 会话）。
- 主机应判定会话重启并 `beginSession`（清账本）。
- 区别于 loopback_test 的 stop/start（那是 socket 层）。

## 通讯增强

- `buildRob` 补发 `AIPos`/`ASPos`（6 关节角，对齐真机 ethernet.xml 请求）。
- `--ipoc-wrap-at N`：IPOC 达到 N 回绕到 0（模拟真机 32 位回绕；默认无）。
- `--jitter-us N`：发送节拍加 ±N µs 随机抖动（模拟真实 UDP/调度抖动）。

## 配置（CLI 参数，保持现有风格）

新增开关汇总：

| 开关 | 默认 | 说明 |
|---|---|---|
| `--init-joints` | `0 -60 30 0 90 0` | 初始关节角（度） |
| `--joint-limits` | KR210 标准 | 覆盖各轴限位 |
| `--cart-limits` | 关 | 笛卡尔位置/姿态范围 |
| `--max-vel-pos/rot` | 0 | 速度限制（0=无） |
| `--max-accel-pos/rot` | 0 | 加速度限制（0=无） |
| `--restart-at-ms` / `--restart-gap-ms` | 0 | 会话重启模拟 |
| `--ipoc-wrap-at` | 0 | IPOC 回绕点（0=无） |
| `--jitter-us` | 0 | 节拍抖动 |

所有新开关默认关闭/不改变现有行为 → `verify_robustness.sh` 现有 5 场景不受影响。

## 内置自检

- 启动时（`--self-test` 或默认）做正解一致性检查：对已知 q 的期望位姿断言
  （±容差），防 DH 参数/欧拉角约定错误。
- 具体：取 3 个已知 q（直立、工作区、腕部翻转），比较正解输出与期望（手算/文档值）。

## 验证

- **单元测试**（新增 `tests/test_kr210_kinematics.cpp`）：
  - 正解：已知 q → 期望位姿（几个典型位形）
  - 雅可比：数值微分 vs 解析雅可比一致性
  - 关节限位 clamp
  - 速度/加速度限制（极限情况）
  - 欧拉角 ↔ 旋转矩阵往返
- **端到端**：**新增** `tools/verify_kinematics.sh`（独立于现有 `verify_robustness.sh`，不破坏其 5 场景回归）：
  - 关节限位：主机给大目标 → 模拟器位姿 clamp → 主机误差停在非零（不震荡）
  - 速度限制：主机给目标 → 模拟器慢响应 → 误差最终收敛、无过冲
  - 会话重启：模拟器 IPOC 重置 → 主机 beginSession（accum 清零可断言）
  - 现有 5 场景回归（默认无约束，不受影响）

## 文件变更

- Modify: `tools/krc_simulator/main.cpp`（结构重构：运动学 + 约束层）
- Create: `tools/krc_simulator/kinematics.h/.cpp`（DH/正解/雅可比，纯函数可单测）
- Create: `tools/krc_simulator/CMakeLists.txt` 挂到 `rsi_core`（运动学纯函数）
- Modify: `tests/CMakeLists.txt`（test_kr210_kinematics）
- Create: `tools/verify_kinematics.sh`（真实约束端到端，独立于 verify_robustness.sh）

## 分阶段

1. `kinematics` 库（DH/正解/雅可比/欧拉角）+ `test_kr210_kinematics` 单测
2. `krc_simulator` 接入运动学（关节模型替代点模型）+ 内置自检
3. 约束层（关节限位 + 笛卡尔限位 + 速度/加速度限制）+ CLI 开关
4. 会话重启 + 通讯增强（AIPos/ASPos、IPOC 回绕、节拍抖动）
5. 验证脚本（扩展 verify_robustness 或新增）+ 回归
