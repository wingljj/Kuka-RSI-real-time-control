# 通信健壮性加固设计（Communication Robustness）

日期：2026-08-01
状态：设计已获用户逐节批准

## 背景与目标

上位机 `rsi_host` 与 KRC 之间的 UDP RSI 链路存在一批已核实的健壮性问题，
集中在 **UDP 会话/IPOC 状态机** 与 **联锁缺失**。本设计在不改动任何 KRC 侧
文件的前提下加固主机侧行为，并建立可脱离真机运行的验证设施。

真实 KRC 首帧联机仍是最终验证点；本设计交付的验证工具（故障注入 + pcap 回放）
用于在真机前尽可能多地暴露主机侧错误。

## 范围

**做**：IpocTracker 状态机、Delay 解析、会话安全（对端/积压/写返回/接收缓冲）、
启动联锁（硬拦截无覆盖）、验证工具（simulator 故障注入 + pcap 回放）、测试与文档。

**不做**（用户明确决定）：
- RsiCodec 的防御性 XML 加固全部砍掉，**只新增 `<Delay>` 解析**（联锁硬前提）
- 三个 KRC 文件 `PoseTrack.rsix` / `PoseTrack.src` / `PoseTrack_ethernet.xml`
  全程只读（`<Delay>` 已在 ethernet.xml 的 SEND 里请求，无需改动）
- BASE/TOOL 校验：仅文档（RSI 帧无此信息，协议层面不可感知）
- QoS、CPU 亲和性、线程实时优先级：不做（Windows 收益甚微、引入平台依赖/权限问题）
- 真机 T1 验证（12ms/4ms）：等有机器人后执行，不在本次代码改动内

## 决策记录（用户逐项确认）

| # | 决策 | 内容 |
|---|---|---|
| 1 | 范围 | 全部代码加固 + 验证工具 |
| 2 | 对端校验 | 首帧锁定 + 同源校验；异源帧丢弃且不回包 |
| 3 | DEF_Delay | 显示 + 联锁输入 |
| 4 | 联锁呈现 | 硬拦截，无覆盖入口 |
| 5 | 底层优化 | 只做 socket 接收缓冲 |
| 6 | 验证工具 | simulator 故障注入 + pcap 回放，两者都要 |
| 7 | BASE/TOOL | 仅文档 |
| 8 | RsiCodec | 只保留 Delay 解析，其余加固全砍 |
| 9 | KRC 文件 | 三个文件只读 |

## 组件设计

### 新增 `core/IpocTracker`（纯 C++，无 Qt，可单测）

IPOC 状态机。输入当前帧 IPOC，输出分类。`RsiWorker` 依据分类决定增量与状态推进。

```cpp
struct IpocEvent {
    enum Kind { First, Normal, Gap, Duplicate, Backward } kind;
    quint64 gapCount = 0;   // 仅 Gap：缺失的周期数
};

class IpocTracker {
public:
    IpocEvent classify(quint64 ipoc);
    quint64 lastGood() const;
    bool haveFirst() const;
    void reset();           // 会话结束/重启时
};
```

**分类矩阵**：

| 分类 | 判定 | 增量 | step() | 更新 lastGood | 丢包计数 |
|---|---|---|---|---|---|
| First | 无 lastGood | 零 | 否（beginSession） | 是 | 0 |
| Normal | `ipoc == lastGood+1` | 计算值 | 是 | 是 | 0 |
| Gap（前向跳号） | `ipoc > lastGood+1` | 计算值 | 是 | 是 | `+= ipoc−lastGood−1` |
| Duplicate | `ipoc == lastGood` | **零** | 否 | 否 | `+= 1` |
| Backward | `ipoc < lastGood` | **零** | 否 | 否 | `+= 1` |

**语义决定（用户已确认）**：
- **Gap 帧回正常修正**：Gap 帧带全新 `RIst`（非旧位姿重放），用新鲜实际位姿算增量
  是 P 控制照常工作；缺的周期自然表现为更大的误差，由第 1 层限幅兜底。
- **Duplicate/Backward 回零增量**：是旧数据重放，用旧 `RIst` 再算一次增量才是
   "旧位姿再次产生修正"。不推进 `lastGood`，使后续正常帧仍能正确判断。
- 所有异常帧（Gap/Duplicate/Backward）**仍必须回包**（回零或计算增量 + 原样回显该帧
  IPOC）——维持"任何分支都必须回包"的硬约束。

### 修改 `core/RsiCodec`（只加 Delay 解析）

- `RobFrame` 新增 `quint64 delay = 0;`，解析 `<Delay D="n"/>` 的 `D` 属性。
- 解析失败不影响帧的 `valid`（Delay 是诊断量，缺失不拒绝整帧）。
- 其余解析行为保持现状，不做任何加固。

### 修改 `net/RsiWorker`（会话安全，四项）

1. **对端锁定**：首帧到来锁存 `(源IP, 源端口)`。此后同源帧正常处理；
   异源帧丢弃且**不回包**，计数 `peerRejected` 供界面显示。不回包理由：
   异源是假 KRC（如残留模拟器），回包会使其误以为掌控链路；真实 KRC 源固定，不受影响。
2. **积压上限**：`onDatagram()` 每轮最多处理 8 帧，到上限 `break`，
   剩余留在 socket 缓冲等下次 `readyRead`。UDP 处理不过来丢帧是正常语义。
3. **写返回值检查**：`writeDatagram()` 返回 `-1` 时计数 `sendFails`；
   Tracking 状态下**连续** 5 次失败 → 置 Fault 停跟踪（KRC 收不到修正时继续跟踪危险）。
   一次成功发送清零连续失败计数。
4. **接收缓冲**：`setSocketOption(ReceiveBufferSizeSocketOption, rx_buffer_bytes)`
   （默认 1MB），防突发积压时内核丢帧。

**接线改造**：将现有 `onDatagram()` 中的 IPOC 内联检查（`f.ipoc <= m_lastIpoc`）
替换为 `IpocTracker::classify` 驱动的分支：

```
Event = tracker.classify(f.ipoc)
if Event == First:    beginSession / resetToActual；更新 lastGood；正常首帧处理
if Event == Normal:   正常 step()，发计算增量
if Event == Gap:      step() 发计算增量；m_missed += gapCount
if Event in {Dup, Back}:  delta = 零；不 step()；不更新 lastGood；m_missed += 1
```

丢包计数语义：`m_missed` = 前向跳号的缺失周期数 + 重复/回退帧各 1。
Gap/Duplicate/Backward 帧**不**清零 `m_missed`（仅 Normal 帧清零）。

### 新增 `core/SessionGuard`（纯函数，可单测）

联锁规则集中一处。输入 `AppConfig` + 实测数据，输出通过/不通过 + 原因列表。

**静态联锁**（绑定后可评估，使能时复查）：
- `cycleMs > 0`
- `sessionGapMs > krc_timeout_cycles × cycleMs`（且 `sessionGapMs > 0`）——
  堵住 `session_gap_ms=0` 重开安全账本漏洞
- `accumLimitPosMm < krc_poscorr_limit_pos_mm` 且
  `accumLimitRotDeg < krc_poscorr_limit_rot_deg`——主机累计限值必须小于 KRC POSCORR
  限值。当前配置 `accum_limit_pos_mm=1000` 会被主动拦下，强迫收敛
- `senType` 非空（防拼出 `<Sen Type="">`；不做已知集合白名单，尊重用户决定）
- 实测周期 vs 配置周期偏差 ≤ 10%（会话开始后才有实测值，使能时评估）

**SENTYPE 匹配校验的落地形式**：主机无法直接读 KRC 的 SENTYPE；错配的症状是
KRC 静默丢弃每一帧回包 → KRC 自身 delay 计数增长。因此 SENTYPE 匹配校验由
运行中保护的 KRC Delay 增长间接实现（见下）。

**运行中保护**（使能后自动 Fault，非"拒绝使能"）：
- `KRC Delay` 在 Tracking 中**持续增长** → 转 Fault 停跟踪。
  判定：Tracking 期间 `delay` 值**连续 3 帧递增**（持平不算递增）→ Fault。
  时序理由：使能瞬间 Delay 可能为 0，"增长"只有运行中才可判定。

### 修改 `core/AppConfig`（新字段 + 校验）

新字段：

| 字段 | 类型 | 默认 | 用途 |
|---|---|---|---|
| `krc_timeout_cycles` | int | 100 | KRC ETHERNET Timeout（周期数），联锁比较 |
| `krc_poscorr_limit_pos_mm` | double | 25.0 | KRC POSCORR 位置累积限值 |
| `krc_poscorr_limit_rot_deg` | double | 25.0 | KRC POSCORR 姿态累积限值 |
| `rx_buffer_bytes` | int | 1048576 | socket 接收缓冲 |

`loadFromFile` 读取上述字段；`sessionGapMs` 的范围校验（> 0）在
`SessionGuard` 联锁层做，而非读取层——读取层保持"缺字段回退默认"的现有行为。

### 修改 `net/SharedState`

`StatusSnapshot` 新增：`quint64 krcDelay`、`int peerRejected`、`int sendFails`。

### 修改 `ui/MainWindow`

- 状态栏追加显示 `KRC 丢包 N`（krcDelay）、`异源 N`（peerRejected）。
- 「使能跟踪」勾选时运行 `SessionGuard` 动态联锁：不通过 → **不置勾** + 状态栏红字
  列出全部未通过项及期望值。硬拦截，无覆盖入口。
- 运行时（Tracking 中）若 worker 因 KRC Delay 增长转 Fault，沿用现有 Fault 呈现
  （勾选框强制取消 + 红字原因）。

## 验证工具

### `tools/krc_simulator` 故障注入开关

| 开关 | 模拟 | 覆盖场景 |
|---|---|---|
| `--drop N` | 每 N 帧不发 | 丢包 |
| `--ipoc-dup N` | 每 N 帧重发上一帧 IPOC | 重复 |
| `--ipoc-back N` | 每 N 帧发回退 IPOC | 回退 |
| `--ipoc-gap N` | 每 N 帧跳 N 个 IPOC | 前向跳号 |
| `--reorder N` | 每 N 帧交换相邻两帧发送顺序 | 乱序 |
| `--late-ms N` | 每 N 帧延迟 N ms 发送 | 迟到 |
| `--ignore-replies` | 收到 `<Sen>` 不回 + 递增 `<Delay>` | 错误 SENTYPE 症状 |
| `--send-delay D` | 固定/递增发 `<Delay>` | KRC Delay 保护 |

`--ignore-replies` 模拟"SENTYPE 错配 → KRC 静默丢回包 → 主机显示丢包 0 但 KRC
Delay 增长"，驱动运行中保护。

### `tools/pcap_replay.py`（新建，Python 标准库）

- 用 `struct` 解析 pcap 全局头/记录头 + 以太网/VLAN/IP/UDP 头，提取发往目标端口的
  `<Rob>` 帧。
- 按原始时间戳（或 `--speed` 加速）重发到目标 IP:端口。
- 支持 `--ipoc-shift`、`--drop`、`--loop`。
- **非闭环**（不回包）：对主机做真实抓包序列的回放验证。
- 不依赖 scapy。

### 验证矩阵文档

场景 × 工具 × 预期行为 × 判定方法。真机 T1（12ms/4ms）部分标记为待执行。

## 测试

- 新增 `tests/test_ipoc_tracker.cpp`：分类矩阵全覆盖 + `gapCount` 精确值断言。
- 新增 `tests/test_session_guard.cpp`：静态联锁逐条（含 `session_gap_ms=0`、
  `accum_limit=1000` 被拦的用例）。
- 扩展 `tests/test_rsi_codec.cpp`：`<Delay D="n"/>` 解析、缺失 Delay 不影响 `valid`。
- 端到端：bash 验证脚本跑 `krc_simulator --ipoc-dup/--drop/...` + `loopback_test`，
  断言主机行为。

## 文档

- 更新 `docs/real-machine-deployment.md` §7 已知缺口：移除已修复项（IPOC 异常帧、
  writeDatagram 返回值、`session_gap_ms` 范围、对端校验、`<Delay>` 解析），
  写入新配置字段（`krc_timeout_cycles` 等）与联锁说明。
- 新增验证矩阵（场景 × 工具 × 预期 × 判定）。

## 分阶段提交

1. `IpocTracker` + RsiWorker 接线 + `test_ipoc_tracker`
2. `RsiCodec` Delay 解析 + `test_rsi_codec` 扩展
3. 会话安全（对端/积压/写返回/接收缓冲）+ `SessionGuard` + AppConfig 字段 + UI 联锁 + `test_session_guard`
4. 工具（simulator 故障注入 + `pcap_replay.py` + 验证脚本）
5. 文档（缺口清理、联锁说明、验证矩阵）

每阶段以测试守护，可独立验证后提交。

## 已知缺口（保留）

- BASE/TOOL：协议层面不可感知，仅文档约束（KRL 固定 + 真机人工核对）。
- RsiCodec 防御性加固（根元素/Type/报文大小/重复字段/NaN 清洗/senType 白名单）：
  按用户决定不做。
- QoS/CPU 亲和性/线程实时优先级：不做。
- 真机 T1（12ms/4ms）验证：待执行，属验证矩阵的一部分而非本次代码改动。
