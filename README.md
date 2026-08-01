# KUKA RSI POSCORR 实时位姿跟踪

通过 KUKA.RobotSensorInterface (RSI) 的 **POSCORR** 在 **BASE 坐标系**下以
**RELATIVE** 增量模式对机器人位姿做实时闭环跟踪。含一个 C++/Qt 上位机、
KRC 侧的 RSI 对象图与 KRL 程序，以及一套可脱离机器人运行的验证工具。

在 KUKA OfficeLite (KSS 8.6.2 / RSI 5.0+) 上开发与验证。

---

## 这是什么，不是什么

POSCORR 是**叠加在机器人当前位姿上的小幅实时修正**，量级几十毫米，
用于视觉纠偏、力控贴合、轨迹微调。

它**不是**把机器人从 A 点搬到 B 点的工具 —— 那是 KRL 的 `PTP`/`LIN`
或 EthernetKRL 的事。目标位姿应该在当前位姿附近微调。

---

## 快速开始（不需要机器人）

从 `dist/`（由 `tools/package.sh` 生成）：

1. 双击 `rsi_host.exe` —— 状态栏显示 `◐ 监听中`
2. 双击 `启动模拟器.bat` —— 一个假 KUKA 控制器开始按 12ms 节拍发帧
3. 状态栏转 `● 已连接`，读数显示模拟器位姿，六个误差全为 0
4. 勾选顶部「使能跟踪」
5. X 目标加 5mm —— 机器人跟过去，误差曲线弹一个峰再衰减回 0

`dist/` 是自包含的（Qt 运行库已打包），目标机不需要装 Qt。

---

## 架构

```
KRC ──UDP──> 上位机
   <Rob><RIst .../><IPOC>n</IPOC></Rob>
              │ 通信线程（独占，禁止阻塞与堆分配）
              ├─ RsiCodec::parseRob
              ├─ 误差 = 目标 − 实际（姿态取最短角路径）
              ├─ 限幅 P 控制 → ΔRKorr
              ↓
上位机 ──UDP──> KRC
   <Sen><RKorr .../><IPOC>n</IPOC></Sen>
              ↓
   Ethernet → Limit×6 → PosCorr(BASE, RELATIVE) → 运动
```

| 单元 | 职责 | IO |
|---|---|---|
| `core/Pose` | 6 自由度类型、`wrap180` 最短角、`poseSub` | 无 |
| `core/AppConfig` | JSON 配置，缺字段回退默认 | 加载时 |
| `core/RsiCodec` | `<Rob>` 解析 / `<Sen>` 生成，IPOC 原样回显 | 无 |
| `core/PoseController` | 限幅 P 控制、锚定位移安全限值 | 无 |
| `net/RsiWorker` | UDP 收发、必定回包、看门狗 | socket |
| `net/SharedState` | 跨线程快照 + 定容环形缓冲（push 无分配） | 无 |
| `ui/MainWindow` | 目标输入、实时读数、状态机 | GUI |
| `ui/ErrorChart` | QtCharts 滚动误差曲线 | GUI |

前四个不含任何 IO，可完全脱离机器人做单元测试。

---

## 三条不可协商的约束

**1. 任何分支都必须按时回包。** RSI 对迟到零容忍：不回包会被判为通信故障
并让机器人错误停机。**解析失败是正常控制路径**，此时回零增量，而不是不回。

**2. `RKorr` 是每周期位移增量，不是目标坐标。** 发绝对坐标等于命令机器人
在一个插补周期内走完整段距离。

**3. `IPOC` 必须原样回显。** RSI 靠它做时序同步，回错等同丢包。

---

## 安全分层

修正量在 KRC 侧是**累加**的，五层限值必须自内向外单调放大，
否则内层永不触发：

| 层 | 位置 | 值 |
|---|---|---|
| 1 单周期增量 | 上位机 `kp × 误差`，夹到 `vmax × cycle` | 0.1 × 10mm/s × 12ms |
| 2 锚定位移 | 上位机 `‖RIst − RIst₀‖` | 可配 |
| 3 单周期限幅 | `PoseTrack.rsix` 的 `Limit` 对象 | ±20 mm / ±20° |
| 4 POSCORR 累积（分量） | `LowerLim/UpperLim`、`MaxRotAngle` | ±25 mm / 25° |
| 5 POSCORRMON 累积（总量） | `MaxTrans`、`MaxRotAngle` | 45 mm / 45° |

三层里只有 4、5 是**累积**量；`Limit` 限的是**单周期增量**，两者不是同一个量。
上位机单周期增量最大 0.012 mm，所以 `Limit` 正常不会触发，它的作用是兜住一帧垃圾数据。
真正的天花板是 POSCORR 的 ±25 mm（分量）与 POSCORRMON 的 45 mm（总量）。
两者必须保持这个关系：三轴各 25 mm 的欧氏总量是 43.3 mm，仍低于 45 —— 
否则外层监视会先于内层限值触发，梯度就反了。

第 2 层刻意用**实际位姿相对会话锚点的位移**，而非命令增量之和 ——
两者原点不同，只有前者与 POSCORR 共享原点因而可比较。

`resetToActual` 刻意**保留**锚点：主机侧归零不能凭空清掉控制器侧已施加的
修正，否则反复「停止→归零→使能」就能不断领取新预算。

**界面上的「停止跟踪」是软停止，不是急停。** 它让机器人停住但**继续回包**。
真正的急停只有示教器上的物理按钮。

---

## 构建

需要 Qt 6.5.3 (mingw_64)、MinGW 11.2、CMake、Ninja。

```bash
cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=<Qt路径> -DCMAKE_BUILD_TYPE=Debug
cmake --build build
bash tools/package.sh      # 生成自包含的 dist/
```

单元测试（Qt Test）：注意 Qt 6.5.3 的文本日志器在 stdout 非控制台时
**什么都不输出**，必须用文件日志器：

```bash
./build/tests/test_pose_controller.exe -o result.log,txt
```

---

## 验证情况

**已验证（无机器人、无虚拟机）**

- 端到端环回：700/700 应答、零丢包、零 IPOC 不匹配，回包延迟 ~150–200µs / 12ms 周期
- 闭环收敛：目标 +5mm，实际到位，误差 0.000
- 通过真实 GUI：拉出 5mm 梯形误差曲线；软停止后 ~1600 周期仍持续应答；
  超限触发故障并自动取消使能；故障后归零保留了安全账本
- 70 个单元断言（5 个二进制），无编译警告

**未验证**

KRC 侧文件（`krc/`）已验证格式（`RSI_CREATE` 在 OfficeLite 上加载通过，属性
规则对照官方参考确认），但**未在任何运行中的 RSI 上完成一次完整联机**。
所有行为都是对着 `krc_simulator` 验证的，而那个模拟器与上位机 codec 出自
同一套假设 —— 一个共享的报文格式误解会以构造的方式通过全部现有测试。

**OfficeLite 无法跑通 RSI 以太网**（官方限制：无 KLI、无法外接通信；社区存在
VxWin 绕过方案但未验证）。因此真机是最终验证点，首次联机请严格按
[docs/real-machine-deployment.md](docs/real-machine-deployment.md) 走，
优先核对 `SENTYPE` 与实测周期。

---

## 文档

| 文件 | 内容 |
|---|---|
| [docs/real-machine-deployment.md](docs/real-machine-deployment.md) | **真机**部署与首次联机（当前主文档） |
| [docs/verification-matrix.md](docs/verification-matrix.md) | 通信健壮性验证矩阵（故障注入 + 回放 + 真机 T1） |
| [docs/deployment.md](docs/deployment.md) | 早期部署文档（含 OfficeLite 环境的记录） |
| [docs/rsi-object-facts.md](docs/rsi-object-facts.md) | 从本机 RSI Visual 提取的对象定义权威数据 |
| [docs/references.md](docs/references.md) | 参照过的第三方资料与各自解决了什么 |
| `docs/superpowers/specs/` | 设计文档 |
| `docs/superpowers/plans/` | 实施计划 |

---

## 许可

MIT，见 [LICENSE](LICENSE)。

`krc/` 下的 RSI 配置与 KRL 程序针对特定的机器人与 RSI 版本编写，
用于其他配置前请自行核对。**接真机前请通读 `docs/deployment.md`。**
