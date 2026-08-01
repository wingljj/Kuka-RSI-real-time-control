# 真机部署与首次联机（Real KRC Commissioning）

本文档针对**真实 KUKA 控制器**（KR C4/C5）。OfficeLite 不具备 RSI 以太网所需的
外部通信路径（无 KLI，官方不支持 RSI，社区有 VxWin 绕过方案但未验证），
**真机才是本项目的最终验证点** —— 真机有 X66 接口，网络配置是标准操作。

上位机与 KRC 侧文件均已对照官方格式验证，但**还没有在任何运行中的 RSI 上完成过
一次完整联机**。首次联机请逐节核对，不要跳步。

---

## 0. 部署前：确认真机的 RSI 版本

`.rsix` 的属性规则随版本略有差异。已在 OfficeLite（RSI 4.1.0）验证 `RSI_CREATE`
能加载，真机上按实际版本核对一次：

1. 示教器查版本：主菜单 → 帮助 → 版本信息，记下 **RobotSensorInterface** 版本
   （如 4.1 / 5.0）。
2. 核对 `krc/PoseTrack.rsix` 里 `IsRuntime="false"` 的用法：
   **参数写 `IsRuntime` 当且仅当该版本的 `rsiToolbox.xml` 在该参数上声明了它。**
   全集通常只有：`ETHERNET.ConfigFile`、`PosCorr.RefCorrSys`、
   `MONITOR.IP/Channel`、IIRFILTER 参数、DELAY.DelayTime。
   **`PosCorrMon` 的 `MaxTrans`/`MaxRotAngle` 不写。**
3. 若版本差异导致 `RSI_CREATE` 报参数错，按 [rsi-object-facts.md](rsi-object-facts.md)
   的规则修正（`ParamId` = 该对象参数类型空间内的 0-based 顺序号；枚举参数按数字序列化）。

---

## 1. 网络：X66 接口

真机有 X66 物理口，网络配置是标准操作：

1. 示教器：**投入运行 → 网络配置 → 其他... → 添加接口**
   - 地址类型：**混合型 IP 地址**
   - IP 地址：与宿主同网段，如 `192.168.250.147`
     （**禁止用 `192.168.0.x`**，且不得落在其它 KLI 子网内）
   - 子网掩码：`255.255.255.0`
   - 自动生成：接收任务 → 过滤器 **目标子网**；实时接收任务 → 过滤器 **UDP**
   - **Windows 接口：不勾**
   - 保存 → 冷启动控制器
2. 宿主网卡设静态 IP，与机器人同网段，如 `192.168.250.10`。
3. 宿主 `ping 机器人IP` 通。

---

## 2. 部署文件

| 源 | 目标 |
|---|---|
| `krc/PoseTrack.src` | `KRC:\R1\Program\` |
| `krc/PoseTrack.rsix` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `krc/PoseTrack_ethernet.xml` | 同上 |

途径：U 盘 / SMB / RDP。

**`PoseTrack_ethernet.xml` 里的 `<IP_NUMBER>` 必须改成宿主的实际 IP**
（默认 `192.168.44.1` 是 OfficeLite 环境的）。

---

## 3. 部署前必须对上的四个值

| 项 | 一处 | 另一处 | 不一致的后果 |
|---|---|---|---|
| 上位机 IP | `PoseTrack_ethernet.xml` 的 `<IP_NUMBER>` | `rsi_config.json` 的 `listen_ip` | RSI 连不上 |
| 端口 | `<PORT>` | `listen_port` | 同上 |
| `SENTYPE` | `<SENTYPE>ImFree</SENTYPE>` | `sen_type` | **KRC 静默丢弃每一帧回包，上位机毫无察觉** |
| 小数位 | `PoseTrack_ethernet.xml` 的 `Precision=4`（在 `.rsix` 的 Ethernet 对象） | `RsiCodec::buildSen` 的 `'f', 4` | 数值截断或解析失败 |

第三项最阴险：上位机会显示"已连接、丢包 0"，而 KRC 一帧回包都没收下，最终以
RSI 超时停机告终。这也是为什么 `SEND` 里请求了 `<Delay>`（见 §7）。

---

## 4. 限值梯度（当前 `krc/PoseTrack.rsix` 的值）

KRC 侧限值仍须**自内向外单调放大**，否则内层永不触发：

| 层 | 位置 | 值 |
|---|---|---|
| 1 单周期增量 | 上位机 `kp × 误差`，夹到 `vmax × cycle` | 0.1 × 10mm/s × 12ms = 0.012 mm |
| ~~2 锚定位移~~（**已移除**，仅显示） | 上位机 `accum_limit_pos_mm` / `_rot_deg` | 20 / 20，**不再判限、不触发 Fault** |
| 3 单周期限幅 | `Limit_*` 对象 | ±20 mm / ±20° |
| 4 POSCORR 累积 | `LowerLim/UpperLim`、`MaxRotAngle` | ±25 mm / 25° |
| 5 监控停机 | `PosCorrMon` 的 `MaxTrans`/`MaxRotAngle` | 45 mm / 45° |

主机侧第 2 层（累积位移/命令和越限保护）已按用户决定移除（2026-08-01）：
`accum_limit_pos_mm` / `accum_limit_rot_deg` 保留仅供 UI「累积修正」显示，
不再判限、不再产生 Fault。**KRC 侧层 4/5（POSCORR ±25 / POSCORRMON 45）是
唯一兜底**，基于 KRC 自己累计的修正量，不受 RIst 姿态折返影响。

注意 3 和 4 不是同一个量：**`Limit` 限的是单周期增量**（正常只有 0.012mm，
永不会触发，兜住垃圾帧），**PosCorr 限的是累积修正**（真正的天花板）。调整层 3~5
保持单调 —— 任何一层比它内侧小，内侧那层就永远不会触发。

---

## 5. 首次联机（T1 模式，手放急停上）

1. **确认 AXISCORR 程序没在运行。** `RKorr` 与 `AKorr` 同时供值行为不可预测。
2. **示教器确认当前工具号与基坐标号。** 修正量相对 BASE 解释，基坐标选错 = 机器人
   朝错误方向走。程序不替你改这两个值。
3. 宿主启动 `rsi_host.exe`，状态栏应为 `◐ 监听中（等待 KRC 发帧）`。
4. 上位机参数调最保守：`Kp` 0.1，限速 10 mm/s、2 °/s（累积上限仅作显示，调多少
   都不影响保护——第 2 层已移除）。
5. 示教器切 **T1**，选中 `PoseTrack.src`，启动做 BCO。**机器人不应产生任何位移**
   （`PTP $POS_ACT` 原地 BCO）。
6. 上位机转 `● 已连接`，当前位姿显示实际值，**六个误差全为 0**（首帧已把目标同步
   为实际位姿）。误差不为 0 就停下排查，别往下走。
7. 读界面「周期」实测值，回填 `config/rsi_config.json` 的 `cycle_ms`，重启上位机。
   这一步不能省：第 1 层限值是 `vmax × cycle`，周期填错（实际 4ms 而写 12ms），
   每个增量就大三倍。
8. 勾选「使能跟踪」。机器人仍应静止（误差为 0）。
9. **单轴小量试探**：X 加 2mm，观察沿 BASE X 方向移动约 2mm，误差曲线出峰后衰减到 0。
10. 逐步放开：单轴到 10mm，姿态 A 加 2°，最后六自由度同时给量。
11. 逐步提高 `Kp` 与限速，盯误差曲线是否振荡/超调。
12. 联机通过后，把限值调回应用需要的值，并重跑一遍确认梯度仍单调。

---

## 6. 异常对照

| 现象 | 排查方向 |
|---|---|
| 上位机停在 `◐ 监听中` | KRL 没跑起来；或 `<IP_NUMBER>`/`<PORT>` 与界面不一致 |
| 上位机停在 `○ 未监听` | 本机绑定失败，多半端口被占（模拟器还挂着）。改端口重试 |
| 已连接但机器人不动 | 没勾「使能跟踪」；或误差本来就是 0 |
| `丢包 0` 但 RSI 报错停机 | 极可能 `SENTYPE` 不符 —— KRC 在丢弃回包。见 §3 |
| RSI 报修正量过大 | KRC 侧层 3~5 梯度反了，或 `Kp` 过高撞上第 4 层 |
| 启动瞬间跳动 | 首帧未同步目标；检查第 6 步「误差全为 0」是否被跳过 |
| 误差曲线持续振荡 | `Kp` 过高，减半再试 |
| 机器人方向与预期相反 | 基坐标系不是你以为的那个（见 §5 第 2 步） |

---

## 7. 联锁与已知缺口

上位机现在带启动联锁（硬拦截，无覆盖）：使能跟踪前自动校验，不通过则状态栏
红字列出原因并拒绝勾选。联锁检查项：

- `cycle_ms` > 0
- `session_gap_ms` > `krc_timeout_cycles × cycle_ms`（配置里 `krc_timeout_cycles`
  默认 100，即 100 个 IPO 周期）
- `sen_type` 非空
- 实测周期与 `cycle_ms` 偏差 ≤ 10%

> `accum_limit_*` 已不参与联锁（第 2 层移除，2026-08-01）。累积修正的上限只靠
> KRC 侧层 4/5（POSCORR ±25 / POSCORRMON 45）兜底，见 §4。

运行中保护：Tracking 期间 KRC 回报的 `<Delay>` 连续 3 帧递增 → 自动转 Fault。
这是 SENTYPE 错配（KRC 静默丢弃回包、主机显示丢包 0）的唯一可见征兆。

新增配置字段（均已在 `config/rsi_config.json`）：
`krc_timeout_cycles`、`krc_poscorr_limit_pos_mm/rot_deg`、`rx_buffer_bytes`。

通信健壮性验证工具：`tools/verify_robustness.sh`（故障注入端到端）与
`tools/pcap_replay.py`（真实抓包回放），矩阵见
[docs/verification-matrix.md](verification-matrix.md)。

已知缺口（保留）：
- **BASE/TOOL 无自动校验**：RSI 帧不含该信息，仅能靠 KRL 程序固定编号 +
  人工核对（§5 第 2 步）。
- QoS、CPU 亲和性、线程实时优先级未做（Windows 收益甚微）。

---

## 8. 安全

- 界面「停止跟踪」是**软停止**：误差归零、机器人停原地，但**仍在持续回包**。
  停止回包会让 RSI 判通信故障并报错停机。
- **急停只有示教器物理按钮。** 本应用没有任何东西是急停。
- 首次联机务必 T1、小范围、低增益，手放急停上。
- 所有行为目前只对 `krc_simulator` 验证过，它与上位机 codec 出自同一套假设。
  首次联机优先核对 §3 的四个值和 §7 的 `<Delay>`。
