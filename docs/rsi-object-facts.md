# RSI 对象定义 —— 从本机权威来源提取的事实

来源（全部在宿主机上，无需访问虚拟机）：

- `C:\Program Files (x86)\KUKA\RSIVisual\rsiToolbox.xml` —— RSI Visual 自带的构件工具箱定义
- `C:\Users\LiuJunjie\OneDrive\桌面\kuka\ROS_RSI_CONTEXT.rsix` —— 一份**真实可用的 PosCorr 对象图**，
  并且把 `<BlueprintCollections>` 整份嵌在文件里，所以枚举值可以逐一对号
- `E:\kuka_rsi_win\ref\kuka_rsi_hw_interface\krl\KR_C4\ros_rsi.rsi.xml` —— KR C4 (KSS 8.x) 的旧格式实例
- `E:\kuka_rsi_driver-main\...\krl\KR_C5\ros_rsi.rsix` —— KR C5 的新格式实例

---

## 1. 两种互不兼容的文件格式

| | 旧格式 | 新格式 |
|---|---|---|
| 扩展名 | `.rsi` + `.rsi.xml` | `.rsix`（单文件） |
| 根元素 | `<rSIModel xmlns="http://schemas.microsoft.com/dsltools/RSIVisual">` / `<RSIObjects>` | `<RsiContext SchemaVersion="2.0.0" xmlns="RsiContext">` |
| 对象 | `<RSIObject ObjType="POSCORR" ObjTypeID="27" ObjID="…">` | `<RsiObject ObjTypeId="27" ObjType="PosCorr" ObjId="…">` |
| 类型名 | 全大写 `POSCORR` | 驼峰 `PosCorr` |
| 属性拼写 | `ObjTypeID`、`OutObjID`（大写 ID） | `ObjTypeId`、`OutObjId`（小写 d） |
| 索引基准 | **1-based** | **0-based** |
| 适用 | KR C4 / KSS 8.x | KR C5 / iiQKA |

**OfficeLite 是 KSS 8.6.2 = KR C4 系列，需要旧格式。** 桌面那份 PosCorr 的
`.rsix` 出自 iiQKA.RobotSensorInterface 6.2.0.34，**不能直接部署**。

## 2. 不能机械换算 ParamID —— 这是个陷阱

新格式的 `ParamId` 等于 blueprint 的 `Index`（都 0-based）。但旧格式的 `ParamID`
**不是** `Index + 1`：

- blueprint `AxisCorr`：`LowerLimA1` Index=0 … `UpperLimA1` Index=6
- 旧格式实例：`LowerLimA1` ParamID=1 … `UpperLimA1` ParamID=**13**

旧格式的编号是全局的、给外部轴（E1–E6）预留了 7–12。所以旧格式的 `POSCORR`
`ParamID` 必须从一份真实的旧格式 POSCORR 实例里读，**不能从 blueprint 推**。
我们手上没有这样的实例 —— 这是目前唯一的缺口。

## 3. PosCorr（ObjTypeId 27）权威定义

输入端口（blueprint Index，新格式 InIdx）：

| 端口 | Index |
|---|---|
| `CorrX` `CorrY` `CorrZ` | 0 1 2 |
| `CorrA` `CorrB` `CorrC` | 3 4 5 |

输出：`Stat`(0) `X`(1) `Y`(2) `Z`(3) `A`(4) `B`(5) `C`(6)

参数（blueprint Index = 新格式 ParamId）：

| 参数 | Index | 默认 | 说明 |
|---|---|---|---|
| `LowerLimX/Y/Z` | 0 1 2 | **-5** | Max=0，即必须为负 |
| `UpperLimX/Y/Z` | 3 4 5 | **5** | Min=0，即必须为正 |
| `MaxRotAngle` | 6 | **5** | **单一总角度限值**，不是每轴 |
| `LastCorrStat` | 7 | 0 | |
| `LastCorrX…C` | 8–13 | 0 | |
| `RefCorrSys` | 0 | `Base` | 枚举，非运行时参数，独立索引空间 |

`RefCorrSys` 是 `TrafoCosysType` 枚举，取值顺序决定数值：

```
World=0   Base=1   RobRoot=2   Tool=3   TTS=4
```

真实文件里写的是 **`ParamValue="1"`（数字），不是字符串 `Base`**。

**我们要的 BASE 坐标系就是默认值**，无需改动语义，但序列化时必须写 `1`。

## 4. PosCorrMon（ObjTypeId 81）

| 参数 | Index | 默认 |
|---|---|---|
| `MaxTrans` | 0 | 6 |
| `MaxRotAngle` | 1 | 6 |

两者都 `CanBeSetAtRuntime="false"`。

**`MaxTrans` 是单一总平移量**，不是 AxisCorrMon 那样的每轴 `MaxA1..MaxA6`。这
独立印证了主机侧第 2 层用欧几里得范数（`hypot`）判限是与 RSI 侧语义一致的。

## 5. Limit（ObjTypeId 39）—— 第 3 层的正解

真实文件的做法：在 `Ethernet` 与 `PosCorr` 之间，为**每个分量各串一个** `Limit`：

```
ETHERNET1.Out1 -> Limit_X -> PosCorr.CorrX
ETHERNET1.Out2 -> Limit_Y -> PosCorr.CorrY
...            -> Limit_C -> PosCorr.CorrC
```

| 参数 | Index | 默认 |
|---|---|---|
| `LowerLimit` | 0 | -10 |
| `UpperLimit` | 1 | 10 |

这就是设计里"第 3 层限幅对象，不依赖上位机代码正确"的实现方式。

## 6. Ethernet（ObjTypeId 64）

| 参数 | Index | 默认 | 真实文件取值 |
|---|---|---|---|
| `ConfigFile` | 0 | `RSIEthernet.xml` | 指向 `_ethernet.xml` |
| `Timeout` | 0 | **10** | **100** |
| `Flag` | 3 | -1 | 1 |
| `Precision` | 7 | **1** | **4** |

`ConfigFile` 与 `Timeout` 同为 Index=0 **是正常的** —— 前者
`CanBeSetAtRuntime="false"`，两类参数各有独立索引空间。此前评审把这判为
"ParamID 重复的错误"，那是**误判**，ROS-I 的配置在这点上没错。

`Precision=4` 与主机侧 `RsiCodec::buildSen` 的 `'f', 4` 必须一致 —— 两边互为
契约，改一边就要改另一边。

## 7. Stop（ObjTypeId 18）—— 比软停止更彻底的退出通道

真实文件把 `ETHERNET1.Out7` 接到 `Stop.In`，`Mode=4`：

```
StopType 枚举：InfoMessage=0  PathNormal=1  Velocity=2  PathFast=3  ExitMoveCorr=4
```

即上位机可以通过第 7 个 `RKorr` 通道主动请求 RSI 退出修正。这比界面上的
"软停止"（只是把误差归零、继续回包）强一级，值得纳入。

## 8. 关于 HOLDON 的悬置问题

最终评审担心 `_ethernet.xml` 里 `HOLDON="1"` 在 RELATIVE 语义下会让 KRC 在主机
停顿时每周期重复施加最后一个增量。上面这份真实 PosCorr 配置用的是
`Limit` + `PosCorrMon` + `Stop` 三重防护，说明实际工程做法是**不依赖 HOLDON 的
语义**，而是用对象图本身兜住。这个疑问仍需在部署前用 RSI 手册确认。

---

## 结论：下一步该怎么做

不要手写旧格式的 `.rsi.xml` —— `ParamID` 编号无法从 blueprint 推导，猜错 RSI
会拒绝加载或静默配错坐标系。正确路径二选一：

**A. 用宿主机的 RSI Visual 生成（推荐）**
新建一个 RSI 上下文，按 §5 的拓扑拖出
`Ethernet → Limit×6 → PosCorr` 再加 `PosCorrMon` 与 `Stop`，参数按 §3–§7 填，
`RefCorrSys` 选 `Base`。工具输出的格式必然与本机 RSI 版本匹配。

**B. 先确认 OfficeLite 的 RSI 版本**
若它其实支持 `.rsix`（RSI 5.0+），桌面那份 `ROS_RSI_CONTEXT.rsix` 可以作为直接
基底改用，只需把 `ConfigFile` 指向我们的 `_ethernet.xml`、并按 §9 重定限值。

## 9. 限值该怎么定（KRC 侧三层要一起定，不能各自为政）

POSCORR 的出厂默认只有 ±5mm / 5°。若照默认部署，RSI 会先在 5mm 拒绝，
层 3~5 的梯度就无从谈起。

建议自内向外拉开梯度（联机后按实测调）：

| 层 | 位置 | 建议值 |
|---|---|---|
| 1 单周期增量 | 主机 `vmax × cycle` | 50 mm/s × 12ms = 0.6 mm |
| ~~2 累积位移~~（**已移除**，仅显示） | 主机 `accum_limit_pos_mm` | 不再判限 |
| 3 分量限幅 | `Limit` 对象 | ±35 mm / ±35° |
| 4 PosCorr 限值 | `LowerLim/UpperLim`、`MaxRotAngle` | ±40 mm / 40° |
| 5 监控停机 | `PosCorrMon` `MaxTrans`/`MaxRotAngle` | 45 mm / 45° |

主机侧第 2 层（累积位移保护）已按用户决定移除（2026-08-01），`accum_limit_*`
仅供 UI 显示，不再判限。**KRC 侧层 4/5（PosCorr / PosCorrMon）是唯一兜底。**

关键约束：**层号越大值必须越大**，否则内层永不触发。设计文档里写的
"POSCORR ~50mm 硬限"来自论坛，实际是可配置的 `LowerLim/UpperLim`，
真正不可配的上限需要查手册确认。
