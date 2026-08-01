# Review 反馈加固设计（Review Feedback Hardening）

日期：2026-08-01
状态：设计已获用户逐节批准

## 背景

PR #1（通信健壮性加固）之后收到一轮 review 反馈：2 个 P1 数学/控制问题（旋转误差、
平滑）与 8 条 UI 建议。本设计在 PR #1 基础上全部落地，不引入 Eigen。

## 范围

**做**（用户"全部做"确认）：
- Part A 旋转数学缓解（低成本，非 SO(3) 重写）
- Part B 目标一阶低通平滑器
- Part C UI 8 条渐进增强

**不做**：
- 引入 Eigen / 四元数 SO(3) 误差重写（新依赖 + 核心控制律重构；正常跟踪小误差下
  逐轴欧拉角差的一阶近似足够，且 KRC 侧层 4/5 兜底）
- 真机奇异区行为验证（待真机；本设计只加 UI 警告 + 文档）

## 关键技术现实（Part A 前提）

**KRC 回报的 `RIst` 姿态角可能本身是折返的**（A/B/C 通常 ±180°，转超 180° 折返；
模拟器即如此）。若数据源折返，主机无法从 RIst 得知真实累计圈数——"用连续未折返
角度"在数据层面不可行。这是固有局限，但 **KRC 侧 `POSCORR MaxRotAngle`（25°/45°）
基于 KRC 自己累计的修正量，不受折返影响，正确兜底**。

因此 Part A 的缓解目标不是"让主机测出真实圈数"，而是"在数据折返时仍然保守"。

## Part A：旋转数学缓解（P1-1）

1. **未折返累计修正作第二信号**：`PoseController::commandedSum()`（`m_accum`）已是
   **不折返**的累计命令增量之和。第 2 层姿态监控改为取保守值：
   ```
   rotMax = max(RIst 锚点位移姿态逐轴最大, |commandedSum 姿态逐轴最大|)
   ```
   即使 RIst 折返漏掉真实圈数，`commandedSum` 仍反映主机发出的累计修正。
   高估是安全方向（宁可误报不漏报）。位置部分维持 RIst 锚点位移欧氏范数（位置不折返）。
   - 注意：`commandedSum` 在丢包时会高估（主机以为发出去了但机器人没收到），
     但第 2 层本就是主机侧保护，高估→更早触发，安全方向。
   - 实现位置：`PoseController::step()` 的越限判定处，`rotMax` 计算从
     `max(|a|,|b|,|c|) 于 m_displacement` 改为两个来源取 max。
2. **限制单次目标变化**：由 Part B 平滑器覆盖（目标阶跃被削成指数逼近）。
3. **文案与文档**：
   - UI 与 `faultReason` 中姿态限值描述改为「逐轴最大累积角」，不叫"总旋转角"。
   - `docs/real-machine-deployment.md` 写明第 2 层姿态监控的折返局限 + 依赖 KRC
     层 4/5。
4. **奇异区警告**：UI 目标输入时任一姿态分量 B 的绝对值 ≥ 85° 时显示黄色警告标签
   （"接近欧拉奇异区，姿态控制可能退化"）。仅警告，不拦截。

## Part B：目标一阶低通平滑器（P1-2）

- **对控制目标做一阶低通，仅 Tracking 生效**，时间常数可配。
- 新配置字段：`target_smoothing_ms`（double，默认 50.0）。`0` 或负值 = 禁用平滑。
- `PoseController`：
  - 新成员 `Pose m_smoothTarget`；`configure()` 由 `cycleMs` 与 `targetSmoothingMs`
    计算 `alpha = cycleS / (cycleS + tauS)`（`tauS = targetSmoothingMs/1000`；
    `alpha` 在 tau≤0 时为 1，即直接跟随）。
  - `step()`：Tracking 且平滑启用时，先
    `m_smoothTarget += alpha × (m_target − m_smoothTarget)`（逐分量），
    误差 = `poseSub(m_smoothTarget, actual)`；否则误差 = `poseSub(m_target, actual)`。
  - `beginSession()` / `resetToActual()`：`m_smoothTarget = actual`
    （避免使能瞬间从旧值开始产生假误差）。
  - `target()` 返回 `m_target`（UI 显示原始目标，用户改什么显示什么）。
  - 第 1 层限幅（`clampAbs`）保持不变——平滑与限幅是正交的两道防线。
- 现有测试 `test_pose_controller` 的算术断言（`kp×误差` 精确值）会受影响：
  平滑后误差不再是 `target−actual`。对策：现有测试配置 `targetSmoothingMs=0`
  保持精确算术；新增平滑测试用例验证低通行为（阶跃目标 → 增量渐进收敛）。
  具体：`testCfg()` 设 `c.targetSmoothingMs = 0.0`。

## Part C：UI 渐进增强（8 条）

全部渐进式落地，每条独立可构建验证。UI 无自动化测试，验证靠构建 + 脚本驱动 GUI
（沿用旧会话的 `WM_KEYDOWN` 驱动法）。

### C1 状态卡
- `m_statusLabel` 拆成两个：`m_stateCard`（大字，颜色分级）+ `m_stateDetail`（次行）。
- 颜色判定（`onRefresh`）：
  - Fault / 超时 / 超限 → 红
  - 已连接且（周期抖动 > 10% 或 丢包>0 或 RSI Delay>0）→ 黄
  - 已连接 → 绿
  - 监听中（未收到帧）→ 蓝
  - 未监听 → 灰
- 次行详情：IPOC / 周期 / 最大回包 / 丢包（连续）/ 累计丢包 / RSI Delay / 对端 IP:port。

### C2 两阶段使能
- `m_trackCheck`（checkbox）→ `m_enableBtn`（QPushButton「准备跟踪」）。
- 点击流程：`SessionGuard::enableChecks`（复用，硬拦截无覆盖）→ 通过后弹确认框
  （操作员核对 BASE/TOOL、目标位姿、限值余量）→ 确认后 `setTracking(true)`。
- 状态机：按钮文本与可用性——
  - Idle 且未连接（未收首帧）：禁用
  - Idle 且已连接：「准备跟踪」（可用）
  - Tracking：「已使能跟踪」（禁用）
  - Fault：「归零并复位」（点击 → `resetToActual` → 回「准备跟踪」）
- 移除旧 `onTrackingToggled` 的 checkbox 语义，改为按钮点击槽。

### C3 软停止横幅
- `m_safetyNote` 升级为固定安全横幅：加大字号 + `⚠` 前缀 +
  "软件停止 = 目标归零并继续回包；急停只有示教器物理按钮"。
- 位置：按钮栏固定（不随滚动区滚动，现有位置已满足）。

### C4 小步进
- `buildTargetPanel` 每轴加一组步进按钮：
  `-10 / -1 / -0.1 / 归零 / +0.1 / +1 / +10`（位置 mm，姿态 °）。
- 归零 = 目标设为当前实际位姿（复用 `onZeroToActual` 语义的轴级版）。
- 滑块保留作粗调；数值框保留作精确输入。

### C5 参数高级页 + 运行锁定
- Kp/限速/累积上限从主界面移到「控制参数…」对话框（`QDialog`）。
- 主界面只显示当前生效值（只读）。
- **Tracking 状态下对话框参数禁用**（运行中锁定）。
- 对话框显示：单位 / 范围 / KRC 硬限（`krcPoscorrLimit*`）/ 当前值 / 剩余余量
  （`krcPoscorrLimit − accumLimit`）。
- 修改后仍走现有 `applyConfig` 排队通道。

### C6 图表
- `ErrorChart` 加空态：无数据时显示占位（"等待 RSI 数据… 请启动 KRL PoseTrack
  程序"），连接后显示。
- 位置误差与姿态误差拆成**两个** `ErrorChart` 实例（去掉双 Y 轴）。
- 周期抖动/丢包标记：不叠加图表点，由 C1 状态卡颜色 + 状态详情体现（黄/红）。
- 默认窗口从 `chart_window_s=20` 改为 10（config 默认值改）。

### C7 读数三列
- `buildReadoutPanel` 从 6 行网格改为三列卡片：
  当前位姿 / 目标误差 / 累积修正，每列 6 行（X Y Z A B C）。
- 三列并排，操作员同时看到"在哪 / 差多少 / 是否接近限值"。

### C8 诊断字段
- `StatusSnapshot` 新增（全部标量，避免 QString 在实时路径的分配问题）：
  - `quint32 peerIp4 = 0`（对端 IP，`QHostAddress::toIPv4Address()`）
  - `quint16 peerPort = 0`
  - `quint64 lifetimeLost = 0`（累计丢包，区别于连续 `missedCount`）
  - `Pose lastDelta`（最近一帧 RKorr）
- 界面显示：对端 IP:port、周期均值/max/P99（`RsiWorker` 内部统计）、
  连续丢包 / 累计丢包、RSI Delay（已有 `krcDelay`）、state（已有）、
  faultReason（已有）。

## 配置字段汇总

| 字段 | 类型 | 默认 | 用途 |
|---|---|---|---|
| `target_smoothing_ms` | double | 50.0 | 目标一阶低通时间常数（≤0 禁用） |
| `chart_window_s` | int | 10（改） | 图表默认窗口 |

## SharedState 变更

`StatusSnapshot` 新增：`quint32 peerIp4`、`quint16 peerPort`、`quint64 lifetimeLost`、
`Pose lastDelta`。全部标量/POD，实时 publish 路径无分配风险（peer 地址用 IP 数字，
不用 QString，避免引用计数在通信线程的 free()）。

## 测试策略

- `test_pose_controller`：`testCfg()` 设 `targetSmoothingMs=0` 保持现有精确算术断言
  有效；新增平滑用例（阶跃目标 → 增量渐进收敛、tau=0 直通、resetToActual 同步
  平滑目标）；新增第 2 层姿态监控的 commandedSum 兜底用例（RIst 折返但 commandedSum
  超限 → 触发）。
- `test_app_config`：新字段读取。
- `test_session_guard`：不变（平滑不影响联锁）。
- 端到端：`verify_robustness.sh` 回归（不受影响，simulator 不注入目标阶跃）。
- UI：构建验证 + 脚本驱动 GUI（状态卡颜色、两阶段使能、小步进、参数对话框锁定）。

## 分阶段

1. Part A（第 2 层姿态监控 commandedSum 兜底 + 文案）+ 测试
2. Part B（目标平滑器 + 配置字段）+ 测试
3. Part C 安全控制区（C1 状态卡 + C2 两阶段使能 + C3 软停止横幅）
4. Part C 操作区（C4 小步进 + C5 参数高级页）
5. Part C 状态区（C6 图表双图/空态 + C7 读数三列 + C8 诊断字段）
6. 文档（real-machine-deployment 折返局限 + UI 说明）+ 验证矩阵更新

每阶段独立构建验证 + 提交。
