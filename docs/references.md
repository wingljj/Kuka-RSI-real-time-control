# 参照资料

本项目在设计与实现期间参照过以下第三方materials。它们**不包含在本仓库中**
（`ref/` 已加入 .gitignore），此处仅记录出处与它们各自解决了什么问题。

## ROS-Industrial `kuka_rsi_hw_interface`

- 出处：https://github.com/ros-industrial/kuka_experimental
- 许可：BSD
- 用途：一份经过实战检验的 RSI 集成实现（AXISCORR 型关节控制）。
  本项目用它交叉验证了三件事：
  1. `<Rob>` / `<Sen>` 报文的实际字段与结构
  2. 旧格式 `.rsi.xml` 的 `ParamID` 编号规则
     （`AxisCorr` 的 `UpperLimA1` 是 `ParamID=13`，
      印证了 `rsiToolbox.xml` 的 `Index` 即旧格式 `ParamID`）
  3. `_ethernet.xml` 的 `INDX` 是 1-based，且该格式跨 RSI 版本不变

## `kuka_rsi_driver`

- 出处：https://github.com/kroshu/kuka_rsi_driver
- 用途：其中 `krl/KR_C5/ros_rsi.rsix` 是 **RSIVisual 生成的真品**
  （标注 `RobotSensorInterface 5.0 B485`），本项目据此确认了新格式
  `.rsix` 的确切写法 —— 特别是 `ParamId` 直接等于 blueprint 的 `Index`
  （0-based），而不是从旧格式换算：`AxisCorr` 的 `UpperLimA1` 在新格式是
  `ParamId=6`，旧格式是 `13`，因为旧格式给外部轴预留了 7–12，新格式把它们
  拆到了 `AxisCorrExt`。

## 本机 RSI Visual 安装

- 位置：`C:\Program Files (x86)\KUKA\RSIVisual\rsiToolbox.xml`
- 用途：RSI 对象定义的权威来源。本项目从中提取了
  `POSCORR`(27) / `POSCORRMON`(81) / `LIMIT`(39) / `STOP`(18) 的
  端口、参数名与索引，以及 `TrafoCosysType` 的枚举顺序
  （`World=0 Base=1 RobRoot=2 Tool=3 TTS=4`）—— 后者决定了
  `RefCorrSys` 必须序列化为数字 `1` 而不是字符串 `Base`。

详见 [rsi-object-facts.md](rsi-object-facts.md)。
