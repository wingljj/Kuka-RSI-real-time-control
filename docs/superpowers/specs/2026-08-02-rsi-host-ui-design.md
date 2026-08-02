# rsi_host 界面审查与重构设计

日期：2026-08-02
范围：`src/ui/`（MainWindow、StatusBar、CommCards、CumulativeBar、AlarmLog、ErrorChart、style.qss），
连带 `CMakeLists.txt`（移除 OpenGL 依赖）。通信层（`src/net/`、`src/core/`）不改。

## 一、动机

现有界面在真机上跑起来后暴露两类问题：

**正确性缺陷** —— 界面显示的内容与实际数据不符，操作员据此判断会得出错误结论。
**结构性缺陷** —— 三栏固定宽度撑爆窗口、右栏部件过载、内联样式与 QSS 并存两套色板。

用户已确认：修正全部缺陷，并重整布局与样式；同时砍掉四个装饰性部件。

## 二、缺陷清单（全部需修复）

### A. 控制参数标签与数值串行（`MainWindow.cpp:402-418`）

标签按 `(1,0) (2,0) (1,2) (2,2) (1,4) (2,4)` 放置，数值循环却按
`(r+1, c*2+1)`，即 `(1,1) (1,3) (2,1) (2,3) (3,1) (3,3)`。结果：

| 界面显示 | 实际绑定 |
|---|---|
| Kp 位置 | kpPos ✓ |
| 限速位置 | **kpRot** ✗ |
| Kp 姿态 | **vmaxPosMmS** ✗ |
| 上限位置 / 上限姿态 | **空** ✗ |
| 第 3 行两个无标签数值 | accumLimit 位置/姿态 ✗ |

六个参数里只有一个标对。修复：改用 `QFormLayout` 两列，标签与数值在同一次
循环里成对创建，从结构上杜绝错位。

### B. 位姿对比表格轴名不可见（`MainWindow.cpp:341` vs `356`）

`verticalHeader()->setVisible(false)` 之后又用 `setVerticalHeaderItem` 写入
X/Y/Z/A/B/C，轴名永不显示。六行数字无法区分轴。
修复：轴名改为表格第 0 列的普通单元格。

### C. QSS 选择器污染卡片内部（`StatusBar.cpp:18,103`）

`c.frame->setStyleSheet("QFrame { ... }")` 中的 `QFrame` 选择器会匹配后代
部件——`QLabel` 继承自 `QFrame`，于是卡片内每个标签都套上白底、边框、圆角，
视觉上像输入框。修复：卡片状态改用 `setProperty` + QSS 属性选择器，
样式表里用 `QFrame#card` 或 `QFrame[state="ok"]` 精确限定。

### D. 表格固定高度截断数据（`MainWindow.cpp:220,350`）

`setFixedHeight(170)` 装不下表头 + 6 行（实测仅可见 4 行），B/C 轴需滚动
才能看到，而滚动条在右栏边缘不明显。修复：按行高计算高度，
`horizontalHeader()->height() + rowHeight*6 + frame*2`，六行全部常驻可见。

### E. 回包耗时卡片两行同值（`CommCards.cpp:65-67`）

"当前"与"最大"都取 `s.maxReplyUs`。`StatusSnapshot` 没有"当前回包耗时"
字段，且通信层不改动。修复：该卡片只显示一个值，标题写明"最大回包 µs"。

### F. 事件日志会被刷屏（`MainWindow.cpp:780-790`）

`onRefresh` 每 20ms 执行一次，`accumOverLimit` 或 `missedCount > 0` 持续为真时
每帧插入一条，1 秒即写满 50 条、4 秒撑满 200 条上限，把之前的真实事件全部挤掉。
修复：改为**边沿触发**——MainWindow 保存上一帧的告警状态，仅在状态由假转真
时记一条。

### G. 图表暂停逻辑相反（`ErrorChart.cpp:160-165`）

暂停时 `m_placeholder->setVisible(true)` 且 `m_view->setVisible(true)`，
两者在同一 grid cell 重叠，placeholder 盖住冻结的曲线——恰恰看不到想暂停
查看的内容。（此项随"砍掉图表按钮"一并消失，见第三节。）

### H. 停止监听不再取消跟踪（`MainWindow.cpp:479-485`）

主分支的 `onStopListening` 会 `m_trackCheck->setChecked(false)`；重构版删掉了
这一行。socket 关闭后不可能再收发帧，但控制器状态仍停留在 Tracking，
状态卡片会持续显示"跟踪中"——覆盖一台已经断开的机器。
修复：停止监听时一并 `setTracking(false)`。

### I. 撤销后差值预览不刷新（`MainWindow.cpp:607-614`）

`restoreTargetSnapshot` 置 `m_suppressTargetSignal = true`，`onTargetEdited`
不会被调用，随后手工写死文案"已撤销未应用的修改"。但此时目标值确实变了，
预览内容与实际偏差不符。修复：撤销后主动重算一次预览。

### J. `m_targetApplied` 只写不读（`MainWindow.h:79`）

置位三处，从未参与任何判断。目标已改未应用时，界面无任何提示。
修复：用它驱动"应用目标"按钮的高亮状态（未应用时按钮为 primary，
已应用时回落为普通），让"改了没发"一眼可见。

### K. 差值预览判空条件恒假（`MainWindow.cpp:587`）

`delta` 起手就被写入前缀"差值预览（目标 − 当前）："，`delta.isEmpty()`
永不为真，"无偏差"分支是死代码。修复：先收集偏差项，再决定文案。

### L. 内联样式与 QSS 双色板

`MainWindow.cpp` 有 20 余处 `setStyleSheet`，`CommCards`/`CumulativeBar`
用的是 `#28a745 / #ffc107 / #dc3545`（Bootstrap 3 色板），而 QSS 与
`StatusBar` 用 `#16A34A / #D97706 / #DC2626`（Tailwind 色板）。同一含义
（正常/警告/故障）在界面上呈现两种绿、两种黄。
修复：全部内联样式迁入 `style.qss`，统一 Tailwind 一套。运行时才知道的
状态（如进度条颜色）通过 `setProperty` + `style()->polish()` 切换。

## 三、砍掉的部件（用户已确认）

1. **流程步骤条**（StatusBar 下排 5 个圆圈 + 箭头）——与上排四张状态卡片
   信息重复，且 Fault 时全部置灰反而丢失信息。
2. **3D TCP 视图**（`TcpView3D.*`）——整个文件删除，连带
   `CMakeLists.txt` 中的 `Qt6::OpenGLWidgets` 与 `opengl32`。位姿数值在
   对比表里已经完整可读，3D 视图不提供额外判据。
3. **图表的暂停 / 清空 / CSV 按钮**（每张图 3 个，共 6 个）——暂停逻辑本身
   是坏的，CSV 导出硬编码文件名且无成功提示。移除后 `ErrorChart` 只剩
   "画一条曲线"这一件事。
4. **「RKorr 输出 | 跟踪状态」标签**（`MainWindow.cpp:93-98`）——文字写死，
   从不更新。其本意由下一节的 RKorr 列真正实现。

## 四、新增：RKorr 输出列

`StatusSnapshot::lastDelta` 已经装着每帧发给 KRC 的增量，界面从未显示。
位姿对比表由三列扩为四列：

```
轴 │ 当前实际 │ 实时误差 │ 目标位姿 │ RKorr 输出
X  │ 1280.400 │   +0.031 │ 1280.431 │   +0.0031
```

RKorr 列用 4 位小数——与 `RsiCodec::buildSen` 的线上量化位数一致，
低于该量化步长的增量在线上就是 0，显示 3 位会让操作员看到一个"在动"
的数字而机器实际不动。零值显示为灰色，非零为蓝色，方便一眼看出哪个轴在动。

通信层零改动。

## 五、布局重整

### 现状问题

```
leftPanel->setFixedWidth(360);   // 固定
midPanel->setFixedWidth(420);    // 固定
body->addWidget(rightPanel, 1);  // 仅右栏可伸缩
```

固定 780px + 右栏塞了 4 个部件（两张图表 + 通信卡片 + 3D 视图），
在 1400px 窗口下右栏只剩 600px 却要竖排 4 个部件，实测通信卡片与
3D 视图被完全挤出可视区。

### 新布局

顶部与底部不可压缩，中部三栏放进 `QSplitter`：

```
┌─────────────────────────────────────────────────┐
│ 状态卡片 ×4（网络 / RSI通信 / 控制状态 / 跟踪质量）│  固定高
├─────────────────────────────────────────────────┤
│ [开始监听][停止监听] │ [复位故障][使能跟踪][停止跟踪]│  固定高
├─────────────────────────────────────────────────┤
│ 联锁提示条（仅拦截时可见）                        │  按需
├──────────────┬──────────────┬───────────────────┤
│ 监听配置      │ 位姿对比      │ 位置误差图         │
│ 目标位姿编辑  │ (4 列 ×6 轴)  │ 姿态误差图         │  QSplitter
│              │ 累积修正      │ 通信指标 ×4        │  可拖动
│              │ 控制参数      │                   │
├──────────────┴──────────────┴───────────────────┤
│ ▶ 事件日志（默认折叠）                            │  固定高
└─────────────────────────────────────────────────┘
```

- 三栏改用 `QSplitter` + `setStretchFactor(0,3) (1,4) (2,5)`，
  给最小宽度而非固定宽度，窄窗口下用户可自行分配。
- 通信卡片（周期 / 最大回包 / 丢包 / IPOC）从右栏底部移到图表下方，
  与图表同属"通信诊断"语义。
- 安全警告语从 StatusBar 内部移到按钮栏右侧，与"停止跟踪"按钮同行——
  提示的是这个按钮的性质，就该在它旁边。

## 六、视觉系统

沿用现有 Tailwind 色板，但**唯一来源是 `style.qss`**：

- 背景 `#F4F6F8`，卡片 `#FFFFFF`，边框 `#D9E0E7`，主色 `#2563EB`
- 语义色：正常 `#16A34A`，警告 `#D97706`，故障 `#DC2626`
- 字号：正文 12px（现为 10px，工业现场偏小），数值 13px 等宽
- 数值一律 `Consolas` 等宽右对齐——位数变化时不跳动

状态色不再靠内联 `setStyleSheet` 拼字符串，改为：

```cpp
w->setProperty("state", "ok");        // ok / warn / fault / idle
w->style()->unpolish(w);
w->style()->polish(w);
```

QSS 侧：

```css
QFrame#statusCard[state="ok"]    { background:#F0FDF4; border-color:#16A34A; }
QFrame#statusCard[state="warn"]  { background:#FFFBEB; border-color:#D97706; }
QFrame#statusCard[state="fault"] { background:#FEF2F2; border-color:#DC2626; }
```

## 七、交互与状态逻辑（保持并补齐）

- 未监听：仅 IP/端口可编辑，仅「开始监听」可用。
- 监听中：连接配置锁定；「停止监听」可用，且触发 `setTracking(false)`。
- `Fault`：仅「复位故障」可用，「使能跟踪」禁用。
- `Ready`：「使能跟踪」可用。
- `Tracking`：仅「停止跟踪」可用，控制参数对话框内字段锁定。
- 使能前保留现有 `SessionGuard::enableChecks` 与确认对话框；
  被拦截的原因显示在联锁提示条。

## 八、验证

1. `cmake --build build --target rsi_host` 通过，无警告新增。
2. 启动 `rsi_host` + `krc_simulator`，截图核对：
   - 控制参数六个标签与数值一一对应（对照 `rsi_config.json`）；
   - 位姿对比表六行轴名齐全，RKorr 列随模拟器运动变化；
   - 四张状态卡片内部无多余边框；
   - 1280×800 窗口下所有部件可见，无部件被挤出。
3. 触发丢包（`krc_simulator --drop`），确认事件日志只记一条而非每帧一条。
4. 停止监听，确认控制状态不再停留在"跟踪中"。
