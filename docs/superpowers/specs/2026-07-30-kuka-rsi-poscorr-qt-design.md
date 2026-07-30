# KUKA RSI POSCORR 实时位姿跟踪上位机 — 设计文档

- 日期：2026-07-30
- 状态：设计待审阅
- 目标平台：KUKA OfficeLite (KSS 8.6.2) + Windows 宿主机

---

## 1. 背景与目标

虚拟机中的 OfficeLite 上已有一套基于 **AXISCORR** 的 RSI 程序，控制的是关节轴（`AKorr.A1..A6`）。现在需要改为直接控制**位姿**。

**目标**：新增一套基于 **POSCORR** 的 RSI 实现，参考坐标系为 **BASE**，修正模式为 **RELATIVE**，配套一个 C++ Qt 上位机实现实时位姿跟踪。

**明确不做**：不修改现有 AXISCORR 程序。新程序以独立的 XML + `.src` 形式并列存在，原有轴控程序完整保留，可随时回退。两者**互斥运行**，不可同时激活（见 §4）。

---

## 2. 已确认的设计决策

| 维度 | 决策 |
|---|---|
| 目标位姿来源 | 界面手动控制（滑块 + 数值框） |
| 控制语义 | 目标位姿闭环跟踪（上位机算误差 → 限幅增量） |
| 技术栈 | C++ / Qt 6.5.3 + MinGW 11.2 + CMake |
| 监视能力 | 数值显示 + 实时误差曲线 |
| 线程架构 | 双线程（通信线程 + GUI 线程） |
| 启动方式 | 原地 BCO（`PTP $POS_ACT`），零初始位移 |
| 工具/基坐标系 | 沿用程序执行时激活的 `$TOOL` / `$BASE`，程序不覆盖 |

---

## 3. 运行环境

| 项 | 值 |
|---|---|
| 宿主机 | Windows 11 Pro，工作目录 `E:\kuka_rsi_win` |
| 虚拟机 | VMware Workstation，`KR C, V8.6.OL_Build02`，KSS 8.6.2 |
| 网络 | VMware **host-only VMnet1**；宿主 `192.168.44.1/24` ↔ guest `192.168.44.128/24`，延迟 0–1ms |
| Qt | `D:\Software\QT\content\6.5.3\mingw_64` |
| 编译器 | `D:\Software\QT\content\Tools\mingw1120_64`（MinGW 11.2.0） |
| CMake | 3.29.3（`D:\Software\QT\content\Tools\CMake_64`） |

网络为 host-only 而非桥接/NAT，原因是：物理网卡处于 Disconnected；WLAN 桥接的抖动不适合 4ms 级周期；NAT 需经 `vmnat` 用户态转发会引入额外延迟。host-only 的 vmnet 是内核驱动直通。

**注意**：guest 网卡为 DHCP，`192.168.44.128` 来自 VMnet1 的 DHCP 池（`192.168.44.128-254`）。RSI 中 guest IP 不需固定（RSI 主动连出），但宿主的 `192.168.44.1` 必须固定 —— 该值正是 `vmnetdhcp.conf` 为 VMnet1 声明的 `fixed-address`，与 VMware 自身配置一致。

另需注意：示教器「网络配置」页显示的 KLI 为 `172.31.1.147`，与 guest Windows 网卡实际地址不一致 —— 该 KSS 配置从未落到网卡上。**所有网络配置以 `192.168.44.128` 为准。**

---

## 4. RSI 关键约束

以下为设计前查证所得，直接约束实现方式：

1. **RELATIVE 模式下 `RKorr` 是「相对当前位置的修正增量」，不是目标位置。** 发送绝对坐标会被解释为要在一个 IPO 周期内完成的巨量运动。这是闭环必须放在上位机的根本原因。
2. **POSCORR 内建安全限值，`RKorr` 超过约 50mm 即拒绝并停机。** 因此限幅是功能必需项，不是优化项。
3. **参考坐标系的配置位置取决于 IPO 模式**：`IPO_FAST` 下必须在 **POSCORR 对象内**配置参考系；`IPO` 模式下由 `RSI_ON()` 指定，且只能配合 LIN/CIRC 运动修正路径。本设计采用 `IPO_FAST` + POSCORR 内配置 BASE，因为位姿跟踪要求机器人**静止时也能被修正驱动**，而 `IPO` 模式只能修正既有 LIN/CIRC 路径。
   **实际通信周期（4ms 或 12ms）取决于 KSS 配置，尚未确认（见 §12）。** 本设计对两者均适用：所有与周期相关的量（步长上限、超时预算、看门狗阈值）均由配置文件中的周期值推导，不在代码中假定具体数值。
4. **`RKorr` 与 `AKorr` 不可同时供值**，行为不可预测。新 POSCORR 程序与现有 AXISCORR 程序必须互斥运行。
5. **建议在 POSCORR 输入前串低通滤波对象**，抑制启动瞬态 —— RSI 会把任何输入都视为必须在 12ms/4ms 内完成的运动。
6. **`IPOC` 必须原样回显**，RSI 依此做时序同步，回错等同丢包。

---

## 5. 架构与数据流

每个 IPO 周期：

```
KRC ──UDP──> 上位机
   <Rob><RIst X.. Y.. Z.. A.. B.. C../><IPOC>n</IPOC></Rob>
                    │ 通信线程
                    ├─ 解析实际位姿
                    ├─ 误差 = 目标 − 实际
                    ├─ 限幅 P 控制 → ΔRKorr（远小于 50mm）
                    ↓
上位机 ──UDP──> KRC
   <Sen><RKorr X.. Y.. Z.. A.. B.. C../><IPOC>n</IPOC></Sen>
                    ↓
   ST_ETHERNET → 低通滤波 → 限幅 → POSCORR(BASE, RELATIVE) → 运动
```

---

## 6. KRC 侧设计

### 6.1 RSI 配置（`PoseTrackEthernet.xml`）

部署路径：`C:\KRC\ROBOTER\Config\User\Common\SensorInterface\`

对象图：`ST_ETHERNET` 接收 6 个 `RKorr` → 低通滤波 → 限幅 → `POSCORR`（参考系 BASE，模式 RELATIVE）。

- `<CONFIG>`：`IP_NUMBER` = 宿主 `192.168.44.1`，`PORT` 见 §12，`PROTOCOL` = UDP，`ONLYSEND` = FALSE
- `<SEND>`：至少含 `DEF_RIst`（实际位姿）、`DEF_RSol`（额定位姿）、`DEF_Delay`、`IPOC`
- `<RECEIVE>`：`RKorr.X/Y/Z/A/B/C`（6 个 DOUBLE）+ `IPOC`

XML 内的限幅对象构成安全防线第 3 层（见 §8），**该层不依赖上位机代码正确性**。

### 6.2 KRL 程序（`PoseTrack.src`）

启动序列：

```
1. 沿用当前 $TOOL / $BASE，程序不修改
2. PTP $POS_ACT          → 原地 BCO，零位移
3. RSI_CREATE 加载 XML   → 建立 UDP 连接
4. 上位机收首帧后把目标初始化为该帧实际位姿 → 误差 0 → Δ=0 → 机器人静止
5. RSI_MOVECORR()        → 启动修正，原地待命
6. 用户改目标后才开始跟随
```

第 2 步用 `PTP $POS_ACT` 而非 `PTP HOME`，保证 BCO 在原地完成、不产生任何实际位移。

第 4 步是**防止启动跳动的关键**：若目标位姿取界面默认值（如 0）而实际 TCP 不在原点，第一个周期即产生巨大误差，直接撞上 50mm 限值停机。**目标位姿初值必须来自机器人回传，不能来自界面默认值。**

---

## 7. 上位机设计

### 7.1 组件划分

边界按「能否脱机测试」划分：

| 单元 | 职责 | 依赖 |
|---|---|---|
| `RsiCodec` | `<Rob>` ↔ 结构体、结构体 → `<Sen>`，纯函数 | 无 IO |
| `PoseController` | (目标, 实际, 限值) → 增量，纯计算 | 无 IO |
| `RsiWorker` | UDP socket + 线程循环，组合上两者 | Qt Network |
| `MainWindow` | 输入、数值显示、误差曲线 | Qt Widgets |

前两个不含任何 IO，可完全脱离机器人做单元测试。

### 7.2 线程模型与通信

| 方向 | 机制 | 理由 |
|---|---|---|
| GUI → 通信线程 | `QMutex` 保护的参数结构体 | 锁持有时间仅够拷贝十几个 double |
| 通信线程 → GUI | 通信线程写 `SharedState`，GUI 以 33ms `QTimer` 拉取 | 周期 4–12ms 即 83–250Hz，逐帧发信号会让 GUI 事件队列积压 |
| 曲线数据 | 环形缓冲，通信线程写 / GUI 读 | 定长无分配，不在实时路径触发 malloc |

**铁律**：通信线程绝不触碰 GUI 对象，绝不执行可能阻塞的操作（无文件 IO、无日志落盘、无动态内存分配）。

### 7.3 界面布局

```
┌───────────────────────────────────────────────────────────────┐
│ ● 已连接 192.168.44.128   IPOC 128374  周期 12.0ms  丢包 0     │
├────────────────────────┬──────────────────────────────────────┤
│ 目标位姿 (BASE)        │ 跟踪误差曲线                         │
│  X ├──●───┤ 1250.00    │  ┌─────────────────────────────────┐ │
│  Y ├───●──┤    0.00    │  │            位置误差 mm          │ │
│  Z ├──●───┤ 1000.00    │  │            姿态误差 °           │ │
│  A ├───●──┤    0.00    │  └─────────────────────────────────┘ │
│  B ├──●───┤   90.00    ├──────────────────────────────────────┤
│  C ├──●───┤    0.00    │        当前位姿   误差   累积修正    │
│                        │  X     1250.00   0.00    +0.00      │
│ [归零到当前位姿]       │  Y        0.00   0.00    +0.00      │
│                        │  Z     1000.00   0.00    +0.00      │
│       位置   姿态      │  A        0.00   0.00    +0.00      │
│ Kp   [0.30] [0.30]     │  B       90.00   0.00    +0.00      │
│ 限速 [50]   [10]       │  C        0.00   0.00    +0.00      │
│ 累积 [30]   [15]       │                                      │
├────────────────────────┴──────────────────────────────────────┤
│ [开始监听] [□ 使能跟踪] [停止跟踪]         TOOL 1   BASE 1    │
└───────────────────────────────────────────────────────────────┘
```

每个自由度为「滑块 + 数值框」联动，滑块粗调、数值框精确输入。

左下参数区两列分别对应位置与姿态：`Kp` 无量纲；限速为 mm/s 与 °/s；累积上限为 mm 与 °。

### 7.4 状态机

```
Idle ──[开始监听]──> Listening ──[收到首帧 <Rob>]──> Connected
                                                        │
                            目标 = 实际，Δ=0，原地不动    │
                                              [使能跟踪] ↓
                                                    Tracking
                                                        │
                              丢包超限 / 累积越限 ───────┤
                                                        ↓
                                        Fault（Δ=0，持续回包，告警）
```

二段式使能：先连接、机器人原地不动、确认数值正常，再手动使能跟踪。不做「一连上就跟随」。

**「停止跟踪」按钮的定位**：把目标位姿拉回当前实际位姿使误差归零，机器人停在原地，但**仍持续回包** —— 一旦停止回包，RSI 会判定通信中断并报错停机。因此它是软停止，**不是急停**。真正的急停只有示教器上的物理急停按钮，界面任何按钮都不能替代。此说明须同时出现在代码注释与界面提示中。

---

## 8. 控制律与安全限值

每周期在通信线程内计算。位置与姿态量纲不同，全程分两组独立处理：

```
// ── 位置分量 (X, Y, Z)，单位 mm ──
误差_pos    = 目标_pos − 实际_pos
步长上限_pos = 速度上限_pos × 周期            // mm/s × s = mm
Δ_pos       = clamp(Kp_pos × 误差_pos, ±步长上限_pos)

// ── 姿态分量 (A, B, C)，单位 ° ──
误差_rot    = wrap180(目标_rot − 实际_rot)    // ±180° 归一化，避免绕远路
步长上限_rot = 速度上限_rot × 周期            // °/s × s = °
Δ_rot       = clamp(Kp_rot × 误差_rot, ±步长上限_rot)

// ── 累积（两组分别限幅）──
累积_pos += Δ_pos     // 对比 累积上限_pos (mm)
累积_rot += Δ_rot     // 对比 累积上限_rot (°)
```

`Kp_pos` 与 `Kp_rot` 是两个独立的无量纲增益，默认取相同值但可分别调整 —— 姿态环的动力学与位置环不同，实测中往往需要不同增益。

四层安全限值，由松到紧：

| 层 | 限制对象 | 值 | 越限行为 |
|---|---|---|---|
| 1 | 单周期增量 | 速度上限 × 周期（位置/姿态各一组） | 静默限幅 |
| 2 | 累积修正量 | `累积上限_pos` / `累积上限_rot`，位置默认远低于 50mm | 界面告警 + 停止累加 |
| 3 | XML 内限幅对象 | 略高于第 2 层 | RSI 侧兜底 |
| 4 | POSCORR 内建 | ~50mm（RSI 硬限） | RSI 报错停机 |

第 3 层置于 XML 内，是为了在上位机崩溃或发出异常值时仍有防线，且该防线不依赖上位机代码正确性。

默认参数（保守起步，联机后再调）：

| 参数 | 默认值 |
|---|---|
| `Kp_pos` / `Kp_rot` | 0.3 / 0.3 |
| `速度上限_pos` | 50 mm/s |
| `速度上限_rot` | 10 °/s |
| `累积上限_pos` | 30 mm |
| `累积上限_rot` | 15 ° |

---

## 9. 错误处理

核心原则：**无论出什么错，都要按时回包。**

| 错误 | 检测方式 | 处理 |
|---|---|---|
| XML 解析失败 | 解析器报错 | 仍回合法 `<Sen>` 且 Δ=0，IPOC 用上帧值，计数告警 |
| IPOC 不连续/回退 | 与上帧比较 | 记丢包，超阈值转 Fault |
| 长时间无包 | 看门狗定时器 | 退回 Listening，界面提示 |
| 累积修正越限 | 每周期检查 | 转 Fault，停止累加，Δ=0 |
| 目标值非法（NaN/超范围） | 输入即校验 | 拒绝写入，输入框标红 |
| UDP 端口被占用 | `bind()` 失败 | 启动即报错，提示占用进程 |

唯一不可接受的行为是「因出错而不回包」—— 那会让 RSI 直接报错停机，后果比任何软件错误都严重。

---

## 10. 测试策略

三层，前两层完全不需要机器人。

**第 1 层 · 单元测试**（Qt Test，Qt 6.5.3 自带，无额外依赖）

- `RsiCodec`：`<Rob>` 字符串 → 位姿 + IPOC 正确；结构体 → 合法 `<Sen>` 且 IPOC 正确回显；畸形输入不崩溃
- `PoseController`：限幅生效；**角度归一化**（目标 179° / 实际 −179°，应走 +2° 而非 −358°，此处最易出错）；累积限值触发

**第 2 层 · KRC 模拟器**（独立小程序）

按固定周期发 `<Rob>`、收 `<Sen>`，验证：回包率 100%、回包延迟分布（须远小于周期）、IPOC 回显、阶跃目标下的收敛曲线符合限幅预期。

此层价值最大：整条实时链路的正确性可在完全没有 RSI、甚至不开虚拟机的情况下验证完毕，也使上位机开发不被 guest 访问阻塞。

**第 3 层 · 联机测试**（OfficeLite）

T1 模式 → 累积上限 ±10mm、`Kp` 0.1 → 单轴单方向 → 逐步放开至六自由度。

---

## 11. 配置文件

IP、端口、周期、各层限值、增益默认值全部外置为 JSON，不硬编码。RSI 版本、实际 IPO 周期、端口号尚未从实际环境确认（见 §12），外置后只需改 JSON 即可适配，无需改代码重编译。

```json
{
  "network": {
    "listen_ip":   "192.168.44.1",
    "listen_port": 59152
  },
  "rsi": {
    "cycle_ms":            12.0,
    "sen_type":            "ImFree",
    "watchdog_miss_limit": 3
  },
  "control": {
    "kp_pos":              0.30,
    "kp_rot":              0.30,
    "vmax_pos_mm_s":       50.0,
    "vmax_rot_deg_s":      10.0,
    "accum_limit_pos_mm":  30.0,
    "accum_limit_rot_deg": 15.0
  },
  "ui": {
    "refresh_ms":     33,
    "chart_window_s": 20
  }
}
```

| 字段 | 含义 |
|---|---|
| `listen_ip` / `listen_port` | 上位机 UDP 监听地址与端口，须与 XML 内 `<IP_NUMBER>`/`<PORT>` 一致 |
| `cycle_ms` | RSI 通信周期，第 1 层步长上限由它推导；**必须与实际 IPO 周期一致** |
| `sen_type` | 回包 `<Sen Type="...">` 的类型标识，须与 XML 内 `SENTYPE` 一致 |
| `watchdog_miss_limit` | 连续丢包阈值，超过即转 Fault |
| `kp_pos` / `kp_rot` | 位置、姿态环比例增益（无量纲） |
| `vmax_*` | 速度上限，乘 `cycle_ms` 得单周期步长上限 |
| `accum_limit_*` | 第 2 层累积限值 |
| `refresh_ms` | GUI 拉取刷新间隔，与通信周期解耦 |
| `chart_window_s` | 误差曲线的时间窗长度 |

---

## 12. 部署前需确认的参数

以下取值须在能访问 guest 后从实际环境读取，逐项填入配置文件：

| 参数 | 确认位置 | 影响 |
|---|---|---|
| RSI 版本 | 示教器「帮助 → 信息 → 选项」或 XML 头部 | XML 对象图写法细节 |
| 实际 IPO 周期 | RSI 配置 / `$IPO_MODE` | 每周期步长上限、超时预算 |
| UDP 端口 | 自定，需避开宿主已占用端口 | 上位机监听设置 |
| `MaxTransCorr` / `MaxRotCorr` | POSCORR 对象参数 | 第 2、3 层限值取值 |
| 当前激活 TOOL / BASE 编号 | 示教器状态栏 | 界面显示与一致性核对 |

guest 访问途径（目前未打通）：SMB (`net use` 挂载 `C$`，需 guest Windows 账号密码，注意与示教器管理员密码 `kuka` 不是同一个) 或 RDP (`mstsc /v:192.168.44.128`，tcp/3389 已确认开放)。

---

## 13. 参考资料

- [RSI switching between POSCORR and AXISCORR — Robot-Forum](https://www.robot-forum.com/robotforum/thread/18034-rsi-switching-between-poscorr-and-axiscorr/)
- [RSI POSCORR — Robot-Forum](https://test.robot-forum.com/robotforum/thread/46331-rsi-poscorr/)
- [Motion Control in RSI 2.3 - Limits — Robot-Forum](https://www.robot-forum.com/robotforum/thread/14941-motion-control-in-rsi-2-3-limits/)
- [KUKA.RobotSensorInterface 3.1 手册 (PDF)](http://supportwop.com/IntegrationRobot/content/6-Syst%C3%A8mes_int%C3%A9grations/RobotSensorInterface/KST_RSI_31_en.pdf)
- [KUKA RSI 2.3 手册 (PDF)](http://www.wtech.com.tw/public/download/manual/kuka/KST-RSI-23-en.pdf)
