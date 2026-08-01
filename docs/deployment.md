# 部署与联机

## 0. 先验证 KRC 侧文件，再碰机器人

`krc/` 下的四个文件是依据宿主机上的权威定义写出来的（见
[rsi-object-facts.md](rsi-object-facts.md)），但**没有在真实 RSI 上加载过**。
`ParamID` 的取值规则已用 `ETHERNET` 与 `AXISCORR` 两个对象在
`ref/kuka_rsi_hw_interface/krl/KR_C4/ros_rsi.rsi.xml` 上逐项校对过，但 `POSCORR`
本身没有旧格式实例可比，所以仍有格式出错的可能。

**你机器上装着 RSI Visual，用它验证一遍再部署 —— 这一步几乎零成本：**

1. 打开 `C:\Program Files (x86)\KUKA\RSIVisual` 里的 RSIVisualShell
2. 用它打开 `E:\kuka_rsi_win\krc\PoseTrack.rsi`
3. 能正常加载并在图上看到 `ETHERNET1 → Limit_X..C → POSCORR1` 的连线、
   参数值与 `POSCORRMON1`，就说明格式与本机 RSI 版本匹配
4. 若报错或某个参数显示为空/异常，**以工具为准**：在工具里改对、另存，
   用它生成的文件替换我写的那份

工具能加载 = 格式正确。这比任何人工检查都可靠。

## 1. 宿主侧前置条件

上位机与实时链路已在无机器人、无虚拟机的条件下验证通过。重跑一遍确认环境没变：

```bash
cd /e/kuka_rsi_win
QBIN="/d/Software/QT/content/6.5.3/mingw_64/bin"
PATH="$QBIN:$PATH" ./build/tools/loopback_test/loopback_test.exe --port 59152 --seconds 12 &
sleep 2
PATH="$QBIN:$PATH" ./build/tools/krc_simulator/krc_simulator.exe \
  --host 127.0.0.1 --port 59152 --cycle-ms 12 --cycles 500
wait
```

必须得到 `replies=500 missed=0 ipoc_mismatch=0` 与 `PASS`。**没通过不要联机** ——
一条在环回上就丢包的链路，对着机器人只会更糟。

## 2. 网络

| 端 | 地址 |
|---|---|
| 宿主（上位机） | `192.168.44.1/24`（VMnet1，host-only） |
| guest（OfficeLite） | `192.168.44.128/24` |

`ping 192.168.44.128` 应有 0–1ms 回复。

注意：示教器「网络配置」页显示的 `172.31.1.147` 与 guest 网卡实际地址不符 —— 那
份 KSS 配置从未落到网卡上。**一切以 `192.168.44.128` 为准。**

## 3. 复制文件到 guest

| 源 | 目标 |
|---|---|
| `krc/PoseTrack.src` | `KRC:\R1\Program\` |
| `krc/PoseTrack.rsix` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `krc/PoseTrack_ethernet.xml` | 同上 |

**只有三个文件。** 本机 RSI 是 5.0+，用 `.rsix` 单文件格式。
`krc/legacy-krc4/` 里那两个 `.rsi` + `.rsi.xml` 是旧格式（给 KR C4 用），
不要部署到这台 —— RSIVisual 能打开它们只说明工具向后兼容，不代表运行时认。

`PoseTrack.src` 里 `RSI_CREATE("PoseTrack", ...)` **刻意不写扩展名** ——
RSI 5.0 会自己补 `.rsix`。写成 `"PoseTrack.rsi"` 会让它去找
`PoseTrack.rsi.rsix`，报「未找到文件」。

途径：SMB（`net use Z: \\192.168.44.128\C$ /user:<账号> <密码>`，注意 guest 的
Windows 账号密码与示教器管理员密码 `kuka` 不是一回事）或 RDP
（`mstsc /v:192.168.44.128`，tcp/3389 已确认开放）。

## 4. 部署前必须对上的四个值

| 项 | 一处 | 另一处 | 不一致的后果 |
|---|---|---|---|
| 上位机 IP | `PoseTrack_ethernet.xml` 的 `<IP_NUMBER>` | 界面「监听地址」/ `rsi_config.json` 的 `listen_ip` | RSI 连不上 |
| 端口 | `<PORT>` | 界面端口 / `listen_port` | 同上 |
| `SENTYPE` | `<SENTYPE>ImFree</SENTYPE>` | `rsi_config.json` 的 `sen_type` | **KRC 静默丢弃每一帧回包，而上位机毫无察觉** |
| 小数位 | `PoseTrack.rsi.xml` 的 `Precision=4` | `RsiCodec::buildSen` 的 `'f', 4` | 数值被截断或解析失败 |

第三项最阴险：上位机会显示"已连接、丢包 0"，而 KRC 那边一帧回包都没收下，
最终以 RSI 超时停机告终。这也是为什么要解析 `<Delay>`（见 §8）。

## 5. KRC 侧限值必须自内向外单调放大

| 层 | 位置 | 当前值 |
|---|---|---|
| 1 单周期增量 | 上位机 `vmax × cycle` | 50 mm/s × 12ms = 0.6 mm |
| ~~2 累积位移~~（**已移除**，仅显示） | 上位机 `accum_limit_pos_mm` / `_rot_deg` | 不再判限 |
| 3 分量限幅 | `PoseTrack.rsi.xml` 的 `Limit_*` | ±35 mm / ±35° |
| 4 POSCORR 限值 | `POSCORR1` 的 `LowerLim/UpperLim`、`MaxRotAngle` | ±40 mm / 40° |
| 5 监控停机 | `POSCORRMON1` 的 `MaxTrans`/`MaxRotAngle` | 45 mm / 45° |

主机侧第 2 层（累积位移保护）已按用户决定移除（2026-08-01），`accum_limit_*`
仅供 UI 显示，不再判限。**KRC 侧层 4/5（POSCORR / POSCORRMON）是唯一兜底。**

**POSCORR 的出厂默认只有 ±5mm / 5°。** 若照默认部署，RSI 会先在 5mm 拒绝，
层 3~5 的梯度就无从谈起。上表已经把梯度拉开。

调整时保持单调：任何一层比它内侧的小，内侧那层就永远不会触发。

## 6. 联机步骤

1. **确认 AXISCORR 程序没在运行。** `RKorr` 与 `AKorr` 同时供值行为不可预测。
2. **在示教器上确认当前工具号与基坐标号。** 修正量是相对 BASE 解释的，
   基坐标选错等于机器人朝错误方向走。程序不会替你改这两个值。
3. 宿主启动 `build/rsi_host.exe`，状态栏应为 `◐ 监听中（等待 KRC 发帧）`。
   若显示 `○ 未监听`，是绑定失败 —— 改地址/端口后点「开始监听」重试。
4. 示教器切 **T1**，选中 `KRC:\R1\Program\PoseTrack.src`。
5. **把上位机参数调到最保守**：`Kp` 0.1/0.1，限速 10 mm/s、2 °/s，
   累积上限 10 mm、5°。
6. 启动 KRL 程序做 BCO 运行。**机器人不应产生任何位移**（`PTP $POS_ACT`）。
7. 观察上位机：状态变 `● 已连接`，当前位姿显示实际值，**误差应全为 0**
   （首帧已把目标同步为实际位姿）。误差不为 0 就停下排查，别往下走。
8. 读界面「周期」实测值，回填 `config/rsi_config.json` 的 `cycle_ms`，重启上位机。
   这一步不能省：第 1 层限值是 `vmax × cycle`，周期填错 12ms 而实际 4ms，
   每个增量就是三倍大。
9. 勾选「使能跟踪」。机器人仍应静止（误差为 0）。
10. **单轴小量试探**：`X` 加 2mm，观察机器人沿 BASE 的 X 方向移动约 2mm，
    误差曲线出现峰后衰减到 0。
11. 逐步放开：单轴到 10mm，再试姿态 `A` 加 2°，最后六自由度同时给量。
12. 逐步提高 `Kp` 与限速，同时盯误差曲线是否出现振荡或超调。

## 7. 异常对照

| 现象 | 排查方向 |
|---|---|
| 上位机停在 `◐ 监听中` | KRL 程序没跑起来；或 `<IP_NUMBER>`/`<PORT>` 与界面不一致 |
| 上位机停在 `○ 未监听` | 本机绑定失败，多半端口被占（上次的模拟器还挂着）。改端口重试 |
| 已连接但机器人不动 | 没勾「使能跟踪」（二段式使能）；或误差本来就是 0 |
| 状态栏显示 `丢包 0` 但 RSI 报错停机 | 极可能 `SENTYPE` 不符 —— KRC 在丢弃你的回包。见 §4 |
| RSI 报修正量过大 | 五层梯度反了，或 `Kp` 过高让单周期增量撞上第 3/4 层 |
| 机器人启动瞬间跳动 | 首帧未同步目标；检查第 7 步「误差应全为 0」是否被跳过 |
| 误差曲线持续振荡 | `Kp` 过高，减半再试 |
| 机器人方向与预期相反 | 基坐标系不是你以为的那个（见第 2 步） |
| 故障 `displacement from session anchor ... exceeds limit` | 第 2 层触发，正常保护。按「归零到当前位姿」后可继续，但累积预算已耗尽，要真正复位需重启 KRL 程序 |

## 8. 已知缺口

这些在最终评审里列为 Important，尚未实现，联机前值得权衡：

- **`<Delay>` 没有解析。** `PoseTrack_ethernet.xml` 已经在 `SEND` 里请求了它，
  但上位机没读。它是 KRC 自己统计的丢包数，也是唯一能让你看见「KRC 认为我丢包
  了」的量 —— 上位机自己的丢包计数看不到迟到回包，也看不到因 `SENTYPE` 不符被
  丢弃的回包。这是 §7 里那条最阴险故障的唯一解药。
- **`writeDatagram` 的返回值被丢弃**，发送失败不计数。
- **没有对端校验**：任何发到监听端口的报文都会被当作 KRC 的帧，回包目标也跟着
  改。现场最可能的触发是上次的 `krc_simulator` 还在跑，于是回包被劈成两半。
- **实测周期未与配置周期交叉校验**，第 8 步靠人不忘。
- **界面上的限值输入框范围是 0..100000**，没有与 POSCORR 的实际限值挂钩。
- **`session_gap_ms` 没有范围校验**，部署时若被写成 0，会重新打开那个
  "比 KRC 超时更短的通信间隙会清零安全账本" 的漏洞。

## 9. 安全

- 界面上的「停止跟踪」是**软停止**：误差归零、机器人停在原地，但上位机**仍在
  持续回包**。停止回包会让 RSI 判通信故障并报错停机。
- **急停只有示教器上的物理急停按钮。** 这个应用里没有任何东西是急停。
- 首次联机务必 T1、小范围、低增益，手放在急停上。
- 所有行为目前只对着 `krc_simulator` 验证过，那个模拟器和上位机的 codec 出自
  同一套假设。一个共享的报文格式误解会以构造的方式通过全部现有测试。
  第一次联机时优先确认 §4 的四个值和 §8 的 `<Delay>`。
