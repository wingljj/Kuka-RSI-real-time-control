# rsi_host 界面重构实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修正 `rsi_host` 界面的 12 个缺陷，砍掉 4 个装饰性部件，新增 RKorr 输出列，并把布局与配色统一到单一 QSS 来源。

**Architecture:** 把"会算错"的界面逻辑（告警边沿触发、数值格式化、按钮启用状态）抽成 `src/ui/UiLogic.h/.cpp` 纯函数，进 `rsi_core` 库并用 QtTest 覆盖——这些正是当前出 bug 的地方，也是唯一能脱离窗口测试的部分。纯视觉问题（布局、配色、选择器污染）无法单测，用截图核对验证。`src/net/`、`src/core/` 的通信与控制逻辑一行不改。

**Tech Stack:** Qt 6.5.3 (Widgets, Charts, Test)、CMake + Ninja、MinGW、QtTest、CTest。

## Global Constraints

- Qt 版本 6.5.3，MinGW 64 位；构建目录 `build`（Debug，Ninja 已配置）。
- 通信层 `src/net/RsiWorker.*`、`src/net/SharedState.h`、`src/core/` 下所有文件**不得修改**（`SharedState.h` 只读不写）。
- 配色唯一来源 `src/ui/style.qss`，色值：背景 `#F4F6F8`、卡片 `#FFFFFF`、边框 `#D9E0E7`、主色 `#2563EB`、正常 `#16A34A`、警告 `#D97706`、故障 `#DC2626`。
- 正文字号 12px，数值 13px 且用 `Consolas` 等宽右对齐。
- RKorr 数值固定 4 位小数（与 `RsiCodec::buildSen` 的线上量化位数一致）；位姿/误差 3 位小数。
- 新增测试一律 `QtTest` + `tests/CMakeLists.txt` 注册 `add_test`，风格对齐 `tests/test_session_guard.cpp`。
- 注释用中文，说明「为什么」而非「做了什么」，与现有代码一致。
- 每个任务结束必须 `cmake --build build --target rsi_host` 通过后再提交。

---

### Task 1: 抽出 UI 纯逻辑层（告警边沿触发 + 按钮启用状态）

修的是缺陷 F（事件日志 4 秒刷满 200 条）与状态机的可测性。这两块是纯函数，先建立测试地基。

**Files:**
- Create: `src/ui/UiLogic.h`
- Create: `src/ui/UiLogic.cpp`
- Create: `tests/test_ui_logic.cpp`
- Modify: `CMakeLists.txt:16-27`（把 `UiLogic.cpp` 加进 `rsi_core`）
- Modify: `tests/CMakeLists.txt`（末尾追加 `test_ui_logic`）

**Interfaces:**
- Consumes: `StatusSnapshot`、`ControlState`（`src/net/SharedState.h`，只读）
- Produces:
  - `struct AlarmEdge { bool accumOverLimit = false; bool packetLoss = false; };`
  - `AlarmEdge uilogic::risingEdges(const AlarmEdge &prev, const StatusSnapshot &s);`
  - `AlarmEdge uilogic::currentAlarms(const StatusSnapshot &s);`
  - `struct ButtonStates { bool resetFault, enableTrack, stopTrack, startListen, stopListen, connEditable; };`
  - `ButtonStates uilogic::buttonStates(const StatusSnapshot &s, bool listening);`

- [ ] **Step 1: 写失败的测试**

创建 `tests/test_ui_logic.cpp`：

```cpp
#include <QtTest>
#include "ui/UiLogic.h"

namespace {

// 构造一个处于 Tracking 且累计超限的快照
StatusSnapshot tracking(bool over, int missed)
{
    StatusSnapshot s;
    s.state          = ControlState::Tracking;
    s.connected      = true;
    s.accumOverLimit = over;
    s.missedCount    = missed;
    return s;
}

} // namespace

class TestUiLogic : public QObject
{
    Q_OBJECT
private slots:
    // ── 告警边沿触发（缺陷 F）──

    void alarmFiresOnceOnRisingEdge()
    {
        AlarmEdge prev;                       // 全 false
        const StatusSnapshot s = tracking(true, 0);

        const AlarmEdge first = uilogic::risingEdges(prev, s);
        QVERIFY(first.accumOverLimit);        // 第一次：记一条

        prev = uilogic::currentAlarms(s);
        const AlarmEdge second = uilogic::risingEdges(prev, s);
        QVERIFY(!second.accumOverLimit);      // 持续为真：不再记
    }

    void alarmRefiresAfterClearing()
    {
        AlarmEdge prev = uilogic::currentAlarms(tracking(true, 0));

        // 告警消失
        const StatusSnapshot ok = tracking(false, 0);
        QVERIFY(!uilogic::risingEdges(prev, ok).accumOverLimit);
        prev = uilogic::currentAlarms(ok);

        // 再次出现：应重新记一条
        const StatusSnapshot bad = tracking(true, 0);
        QVERIFY(uilogic::risingEdges(prev, bad).accumOverLimit);
    }

    void accumAlarmOnlyWhenTracking()
    {
        // 非 Tracking 下 accumOverLimit 不该产生告警：控制器没在发增量
        StatusSnapshot s = tracking(true, 0);
        s.state = ControlState::Ready;
        QVERIFY(!uilogic::currentAlarms(s).accumOverLimit);
    }

    void lossAlarmNeedsConnection()
    {
        StatusSnapshot s = tracking(false, 5);
        QVERIFY(uilogic::currentAlarms(s).packetLoss);
        s.connected = false;                  // 断开时的 missedCount 无意义
        QVERIFY(!uilogic::currentAlarms(s).packetLoss);
    }

    // ── 按钮启用状态（缺陷 H 与状态机）──

    void faultEnablesOnlyReset()
    {
        StatusSnapshot s;
        s.state = ControlState::Fault;
        s.connected = true;
        const ButtonStates b = uilogic::buttonStates(s, true);
        QVERIFY(b.resetFault);
        QVERIFY(!b.enableTrack);              // Fault 必须先复位
    }

    void readyEnablesTracking()
    {
        StatusSnapshot s;
        s.state = ControlState::Ready;
        s.connected = true;
        const ButtonStates b = uilogic::buttonStates(s, true);
        QVERIFY(b.enableTrack);
        QVERIFY(!b.resetFault);
        QVERIFY(!b.stopTrack);
    }

    void trackingEnablesOnlyStop()
    {
        StatusSnapshot s;
        s.state = ControlState::Tracking;
        s.connected = true;
        const ButtonStates b = uilogic::buttonStates(s, true);
        QVERIFY(b.stopTrack);
        QVERIFY(!b.enableTrack);
    }

    void notListeningLocksEverything()
    {
        StatusSnapshot s;                     // Disconnected
        const ButtonStates b = uilogic::buttonStates(s, false);
        QVERIFY(b.startListen);
        QVERIFY(!b.stopListen);
        QVERIFY(b.connEditable);              // 仅未监听时可改 IP/端口
        QVERIFY(!b.enableTrack);
    }

    void listeningLocksConnConfig()
    {
        StatusSnapshot s;
        s.state = ControlState::WaitingFirstFrame;
        const ButtonStates b = uilogic::buttonStates(s, true);
        QVERIFY(!b.startListen);
        QVERIFY(b.stopListen);
        QVERIFY(!b.connEditable);
    }
};

QTEST_MAIN(TestUiLogic)
#include "test_ui_logic.moc"
```

- [ ] **Step 2: 注册测试并确认编译失败**

在 `tests/CMakeLists.txt` 末尾追加：

```cmake
add_executable(test_ui_logic test_ui_logic.cpp)
target_link_libraries(test_ui_logic PRIVATE rsi_core Qt6::Test)
add_test(NAME test_ui_logic COMMAND test_ui_logic)
```

Run: `cmake --build build --target test_ui_logic 2>&1 | tail -5`
Expected: FAIL —— `fatal error: ui/UiLogic.h: No such file or directory`

- [ ] **Step 3: 写实现**

创建 `src/ui/UiLogic.h`：

```cpp
#pragma once
#include "net/SharedState.h"

// 界面侧的纯判定逻辑。抽出来单独成文件的理由：这些判断原先散在
// MainWindow::onRefresh 里，而 onRefresh 每 20ms 跑一次、又只能靠肉眼验证，
// 于是「持续为真的告警每帧记一条」这类错误可以长期不被发现。纯函数可单测。

// 一组告警的布尔状态。既用作「当前是否告警」，也用作「上一帧的告警状态」。
struct AlarmEdge
{
    bool accumOverLimit = false;
    bool packetLoss     = false;
};

// 按钮与输入框的启用状态。由控制状态和监听状态共同决定。
struct ButtonStates
{
    bool resetFault   = false;
    bool enableTrack  = false;
    bool stopTrack    = false;
    bool startListen  = false;
    bool stopListen   = false;
    bool connEditable = false;   // IP / 端口是否可编辑
};

namespace uilogic {

// 当前帧的告警状态。
AlarmEdge currentAlarms(const StatusSnapshot &s);

// 上升沿：仅在「上一帧为假、本帧为真」时返回真。
// 事件日志只该在告警「发生」时记一条，而不是在告警「持续」的每一帧都记。
// 后者会在 4 秒内把 200 条上限刷满，把之前的真实事件全部挤掉。
AlarmEdge risingEdges(const AlarmEdge &prev, const StatusSnapshot &s);

// 按钮启用状态。
ButtonStates buttonStates(const StatusSnapshot &s, bool listening);

} // namespace uilogic
```

创建 `src/ui/UiLogic.cpp`：

```cpp
#include "ui/UiLogic.h"

namespace uilogic {

AlarmEdge currentAlarms(const StatusSnapshot &s)
{
    AlarmEdge a;
    // 累计超限只在 Tracking 下有意义：不跟踪时控制器不发增量，
    // 累计值是上一段跟踪留下的历史，不构成「现在出事了」。
    a.accumOverLimit = s.accumOverLimit && s.state == ControlState::Tracking;
    // 丢包计数在断开时是上一次会话的残值，同样不构成当前告警。
    a.packetLoss     = s.missedCount > 0 && s.connected;
    return a;
}

AlarmEdge risingEdges(const AlarmEdge &prev, const StatusSnapshot &s)
{
    const AlarmEdge now = currentAlarms(s);
    AlarmEdge e;
    e.accumOverLimit = now.accumOverLimit && !prev.accumOverLimit;
    e.packetLoss     = now.packetLoss     && !prev.packetLoss;
    return e;
}

ButtonStates buttonStates(const StatusSnapshot &s, bool listening)
{
    ButtonStates b;

    // 地址与端口只在未绑定时可编辑：运行中改它们不会生效，
    // 只会让界面显示的地址与实际绑定的地址不符——那比不给改更糟。
    b.connEditable = !listening;
    b.startListen  = !listening;
    b.stopListen   = listening;

    // Fault 是锁存的：PoseController::setTracking(true) 在 Fault 下不转
    // Tracking，必须先经复位清除。所以 Fault 下「使能跟踪」必须禁用，
    // 否则点了没反应，操作员会以为是程序卡死。
    b.resetFault  = (s.state == ControlState::Fault);
    b.enableTrack = (s.state == ControlState::Ready);
    b.stopTrack   = (s.state == ControlState::Tracking);

    return b;
}

} // namespace uilogic
```

在 `CMakeLists.txt` 的 `rsi_core` 源文件列表里加入 `src/ui/UiLogic.cpp`（放在 `src/core/SessionGuard.cpp` 之后一行）。

- [ ] **Step 4: 运行测试确认通过**

Run: `cmake --build build --target test_ui_logic && ./build/tests/test_ui_logic.exe`
Expected: PASS，`Totals: 9 passed, 0 failed, 0 skipped`

- [ ] **Step 5: 提交**

```bash
git add src/ui/UiLogic.h src/ui/UiLogic.cpp tests/test_ui_logic.cpp tests/CMakeLists.txt CMakeLists.txt
git commit -m "feat(ui): extract alarm edge detection and button state logic

Alarms were logged every 20ms refresh while the condition held, filling
the 200-row log in 4 seconds. Edge triggering plus button state are pure
functions, so they get tests."
```

---

### Task 2: 数值格式化纯函数（RKorr 列 + 差值预览）

覆盖缺陷 K（差值预览判空恒假）并为 Task 5 的 RKorr 列准备格式化函数。

**Files:**
- Modify: `src/ui/UiLogic.h`
- Modify: `src/ui/UiLogic.cpp`
- Modify: `tests/test_ui_logic.cpp`

**Interfaces:**
- Consumes: Task 1 的 `uilogic` 命名空间、`Pose`（`src/core/Pose.h`）
- Produces:
  - `QString uilogic::axisUnit(int axis);`  → `" mm"` (0-2) / `" deg"` (3-5)
  - `QString uilogic::formatValue(double v, int axis);`  → 3 位小数 + 单位
  - `QString uilogic::formatRkorr(double v, int axis);`  → 4 位小数 + 单位，非零带正负号
  - `QString uilogic::deltaPreview(const double target[6], const Pose &actual);`

- [ ] **Step 1: 写失败的测试**

在 `tests/test_ui_logic.cpp` 的 `private slots:` 区末尾追加：

```cpp
    // ── 数值格式化 ──

    void formatValueUsesThreeDecimals()
    {
        QCOMPARE(uilogic::formatValue(1280.4, 0), QString("1280.400 mm"));
        QCOMPARE(uilogic::formatValue(-0.5, 4),   QString("-0.500 deg"));
    }

    void formatRkorrUsesFourDecimals()
    {
        // 4 位小数 = RsiCodec::buildSen 的线上量化位数。显示 3 位会把
        // 0.00005 这种「线上就是 0」的量显示成 0.000，看不出差别。
        QCOMPARE(uilogic::formatRkorr(0.00312, 0), QString("+0.0031 mm"));
        QCOMPARE(uilogic::formatRkorr(-0.00312, 3), QString("-0.0031 deg"));
    }

    void formatRkorrZeroHasNoSign()
    {
        // 零增量是常态（未跟踪时每帧都是零），带个 "+0.0000" 很吵
        QCOMPARE(uilogic::formatRkorr(0.0, 0), QString("0.0000 mm"));
    }

    // ── 差值预览（缺陷 K）──

    void deltaPreviewReportsNoDeviation()
    {
        // 原代码里 delta 起手就写入了前缀，isEmpty() 永不为真，
        // 「无偏差」是死代码
        Pose actual;
        actual.x = 100.0;
        const double target[6] = {100.0, 0, 0, 0, 0, 0};
        QVERIFY(uilogic::deltaPreview(target, actual).contains("无偏差"));
    }

    void deltaPreviewListsDeviatingAxesOnly()
    {
        Pose actual;
        actual.x = 100.0;
        actual.z = 50.0;
        const double target[6] = {105.0, 0, 50.0, 0, 0, 0};
        const QString s = uilogic::deltaPreview(target, actual);
        QVERIFY(s.contains("X"));
        QVERIFY(!s.contains("Z"));       // Z 无偏差，不该列出
        QVERIFY(!s.contains("无偏差"));
    }
```

- [ ] **Step 2: 运行测试确认失败**

Run: `cmake --build build --target test_ui_logic 2>&1 | tail -5`
Expected: FAIL —— `'formatValue' is not a member of 'uilogic'`

- [ ] **Step 3: 写实现**

在 `src/ui/UiLogic.h` 的 `namespace uilogic {` 内，`buttonStates` 声明之后追加：

```cpp
// 轴单位。0-2 为位置（mm），3-5 为姿态（deg）。
QString axisUnit(int axis);

// 位姿 / 误差 / 目标的显示格式：3 位小数 + 单位。
QString formatValue(double v, int axis);

// RKorr 增量的显示格式：4 位小数 + 单位。位数必须与 RsiCodec::buildSen
// 的线上量化一致——幅值小于该量化步长的增量会被格式化成 0，机器人实际
// 不动；用 3 位显示会让操作员看到一个「在动」的数字而机器静止。
// 非零值带正负号，零值不带（未跟踪时每帧都是零，满屏 "+0.0000" 很吵）。
QString formatRkorr(double v, int axis);

// 差值预览文案：列出目标与实际偏差超过 0.005 的轴。全部在容差内时
// 返回「无偏差」。
QString deltaPreview(const double target[6], const Pose &actual);
```

并在文件顶部加入 `#include <QString>` 与 `#include "core/Pose.h"`。

在 `src/ui/UiLogic.cpp` 的 `namespace uilogic {` 内追加：

```cpp
namespace {

const char *kAxisName[6] = {"X", "Y", "Z", "A", "B", "C"};

} // namespace

QString axisUnit(int axis)
{
    return (axis < 3) ? QStringLiteral(" mm") : QStringLiteral(" deg");
}

QString formatValue(double v, int axis)
{
    return QStringLiteral("%1%2").arg(v, 0, 'f', 3).arg(axisUnit(axis));
}

QString formatRkorr(double v, int axis)
{
    // 与 buildSen 的量化步长对齐：小于半个步长的量线上就是 0，
    // 显示成 0 是如实反映，不是精度损失。
    constexpr double kWireQuantum = 5e-5;
    if (std::fabs(v) < kWireQuantum)
        return QStringLiteral("0.0000%1").arg(axisUnit(axis));
    return QStringLiteral("%1%2%3")
        .arg(v > 0 ? "+" : "")
        .arg(v, 0, 'f', 4)
        .arg(axisUnit(axis));
}

QString deltaPreview(const double target[6], const Pose &actual)
{
    const double act[6] = {actual.x, actual.y, actual.z,
                           actual.a, actual.b, actual.c};
    // 先收集，再决定文案。原实现把前缀直接写进结果串，
    // 于是 isEmpty() 永不为真、「无偏差」分支永不可达。
    QStringList parts;
    for (int i = 0; i < 6; ++i) {
        const double d = target[i] - act[i];
        if (std::fabs(d) > 0.005)
            parts << QStringLiteral("%1 %2%3")
                         .arg(kAxisName[i])
                         .arg(d, 0, 'f', (i < 3) ? 2 : 3)
                         .arg(axisUnit(i));
    }
    if (parts.isEmpty())
        return QStringLiteral("目标与当前位姿无偏差");
    return QStringLiteral("目标 − 当前：") + parts.join(QStringLiteral("　"));
}
```

在 `src/ui/UiLogic.cpp` 顶部加入 `#include <QStringList>` 与 `#include <cmath>`。

- [ ] **Step 4: 运行测试确认通过**

Run: `cmake --build build --target test_ui_logic && ./build/tests/test_ui_logic.exe`
Expected: PASS，`Totals: 14 passed, 0 failed, 0 skipped`

- [ ] **Step 5: 提交**

```bash
git add src/ui/UiLogic.h src/ui/UiLogic.cpp tests/test_ui_logic.cpp
git commit -m "feat(ui): value formatting helpers with RKorr wire precision

deltaPreview's empty check was unreachable — the prefix was written into
the string before the check. RKorr needs 4 decimals to match buildSen's
wire quantization; 3 would show motion where the wire sends zero."
```

---

### Task 3: 删除四个装饰性部件

先做删除：后续任务在更小的代码面上进行，减少返工。

**Files:**
- Delete: `src/ui/TcpView3D.h`
- Delete: `src/ui/TcpView3D.cpp`
- Modify: `CMakeLists.txt:14`（移除 `OpenGLWidgets`）、`:53`（移除 `TcpView3D.cpp`）、`:56`（移除 `Qt6::OpenGLWidgets opengl32`）
- Modify: `src/ui/StatusBar.h`、`src/ui/StatusBar.cpp`（移除流程步骤条）
- Modify: `src/ui/ErrorChart.h`、`src/ui/ErrorChart.cpp`（移除按钮与暂停逻辑）
- Modify: `src/ui/MainWindow.h`、`src/ui/MainWindow.cpp`（移除 3D 视图与 RKorr 死标签）

**Interfaces:**
- Produces: `StatusBar` 不再有 `setStepActive`；`ErrorChart` 不再有 `onPauseToggled` / `onClear` / `onExport` / `setSubMode`；`MainWindow` 不再有 `m_tcpView`。

- [ ] **Step 1: 删除 TcpView3D 并解除 OpenGL 依赖**

```bash
git rm src/ui/TcpView3D.h src/ui/TcpView3D.cpp
```

`CMakeLists.txt` 三处改动：

```cmake
# 第 14 行
find_package(Qt6 REQUIRED COMPONENTS Core Network Widgets Charts Test)

# 第 53 行附近：从 rsi_host 源文件列表删除 src/ui/TcpView3D.cpp

# 第 56 行
target_link_libraries(rsi_host PRIVATE
    rsi_net Qt6::Core Qt6::Network Qt6::Widgets Qt6::Charts)
```

`src/ui/MainWindow.h`：删除 `class TcpView3D;` 前向声明与 `TcpView3D *m_tcpView = nullptr;` 成员。
`src/ui/MainWindow.cpp`：删除 `#include "ui/TcpView3D.h"`、`buildRightPanel` 中创建 `m_tcpView` 的三行、`onRefresh` 中 `if (s.connected) m_tcpView->updatePoses(...)` 的两行。

- [ ] **Step 2: 移除流程步骤条**

`src/ui/StatusBar.h`：删除 `struct Step`、`m_steps`、`m_arrows`、`setStepActive` 声明，并删除现在多余的 `#include <array>`（若有）。
`src/ui/StatusBar.cpp`：删除构造函数中「下排：流程步骤条」整段（`auto *flow = ...` 到 `v->addLayout(flow);`）、`setStepActive` 函数定义、`updateFrom` 末尾「流程步骤条」整段（含 `activeStep` lambda 与 Fault 置灰循环）。

- [ ] **Step 3: 移除图表按钮与暂停逻辑**

`src/ui/ErrorChart.h`：删除 `#include <QPushButton>`、三个按钮成员、`m_paused`、`m_pausedTEnd`、`private slots:` 整段、`setSubMode` 声明。
`src/ui/ErrorChart.cpp`：删除三个按钮的创建与 `btnRow`、`onPauseToggled`/`onClear`/`onExport`/`setSubMode` 四个函数定义、`updateFrom` 中的 `if (m_paused) {...}` 分支，以及 `#include <QDateTime>`、`#include <QFile>`、`#include <QTextStream>`。

布局简化为：

```cpp
    auto *lay = new QGridLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->setSpacing(0);
    lay->addWidget(m_view, 0, 0);
    lay->addWidget(m_placeholder, 0, 0);
```

- [ ] **Step 4: 移除 RKorr 死标签**

`src/ui/MainWindow.cpp` 构造函数中删除 `rkorrLabel` 的创建与 `btnBar->addWidget(rkorrLabel);`（约 93-98 行）。

- [ ] **Step 5: 构建并确认无残留引用**

Run: `cmake --build build --target rsi_host 2>&1 | tail -10`
Expected: 构建成功，无 `TcpView3D` / `setStepActive` / `m_paused` 相关错误

Run: `grep -rn "TcpView3D\|setStepActive\|m_paused\|setSubMode\|OpenGLWidgets" --include="*.cpp" --include="*.h" --include="*.txt" src/ CMakeLists.txt`
Expected: 无输出

- [ ] **Step 6: 提交**

```bash
git add -A src/ui CMakeLists.txt
git commit -m "refactor(ui): remove flow steps, 3D view, chart buttons, dead label

The flow step bar duplicated the four status cards and went fully grey on
Fault, losing information. The 3D view was pushed off-screen and adds no
judgement the pose table lacks. Chart pause showed the placeholder over
the frozen curve — inverted. The RKorr label text never updated.

Drops the Qt6::OpenGLWidgets dependency."
```

---

### Task 4: 修控制参数面板串行与轴名隐藏

缺陷 A 与 B——两个「显示与实际不符」的问题，都在中栏。

**Files:**
- Modify: `src/ui/MainWindow.cpp`（`buildMidPanel` 的位姿表与参数网格）
- Modify: `src/ui/MainWindow.h`（`m_paramVal` 数组注释）

**Interfaces:**
- Consumes: Task 2 的 `uilogic::formatValue`
- Produces: `m_paramVal[0..5]` 顺序固定为 kpPos, kpRot, vmaxPos, vmaxRot, accumPos, accumRot

- [ ] **Step 1: 用 QFormLayout 重建参数面板**

把 `buildMidPanel` 中「控制参数只读行」整段（`auto *paramGrid = new QGridLayout;` 到 `safeV->addLayout(paramGrid);`）替换为：

```cpp
    // 标签与数值在同一次循环里成对创建。原实现把标签按
    // (1,0)(2,0)(1,2)(2,2)(1,4)(2,4) 摆放、数值按 (r+1, c*2+1) 摆放，
    // 六个参数里只有第一个对得上——操作员照标签读到的是别的参数。
    // 成对创建让这种错位在结构上不可能发生。
    struct ParamRow { const char *label; };
    const ParamRow kParams[6] = {
        {"Kp 位置"},   {"Kp 姿态"},
        {"限速位置"},  {"限速姿态"},
        {"累积上限位置"}, {"累积上限姿态"},
    };

    auto *paramForm = new QGridLayout;
    paramForm->setHorizontalSpacing(12);
    paramForm->setVerticalSpacing(4);
    for (int i = 0; i < 6; ++i) {
        const int row = i / 2;
        const int col = (i % 2) * 2;
        auto *name = new QLabel(kParams[i].label, safeBox);
        name->setProperty("cssClass", "fieldLabel");
        paramForm->addWidget(name, row, col);

        auto *val = new QLabel("--", safeBox);
        val->setProperty("cssClass", "readout");
        val->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        paramForm->addWidget(val, row, col + 1);
        m_paramVal[i] = val;
    }
    paramForm->setColumnStretch(1, 1);
    paramForm->setColumnStretch(3, 1);
    safeV->addLayout(paramForm);
```

`onRefresh` 中赋值顺序对应改为（原顺序已正确，仅确认）：

```cpp
    m_paramVal[0]->setText(QString::number(m_cfg.kpPos, 'f', 3));
    m_paramVal[1]->setText(QString::number(m_cfg.kpRot, 'f', 3));
    m_paramVal[2]->setText(QStringLiteral("%1 mm/s").arg(m_cfg.vmaxPosMmS, 0, 'f', 1));
    m_paramVal[3]->setText(QStringLiteral("%1 deg/s").arg(m_cfg.vmaxRotDegS, 0, 'f', 1));
    m_paramVal[4]->setText(QStringLiteral("%1 mm").arg(m_cfg.accumLimitPosMm, 0, 'f', 1));
    m_paramVal[5]->setText(QStringLiteral("%1 deg").arg(m_cfg.accumLimitRotDeg, 0, 'f', 1));
```

- [ ] **Step 2: 轴名改为第 0 列普通单元格**

`buildMidPanel` 的位姿表由 3 列改为 5 列（轴名 + 4 个数据列，RKorr 列在 Task 5 填充）：

```cpp
    auto *tbl = new QTableWidget(6, 5, poseBox);
    tbl->setHorizontalHeaderLabels(
        {"轴", "当前实际", "实时误差", "目标位姿", "RKorr 输出"});
    tbl->verticalHeader()->setVisible(false);
```

循环里把原先的 `setVerticalHeaderItem` 改为第 0 列单元格——垂直表头刚被
`setVisible(false)` 隐藏，写进去的轴名永远看不到：

```cpp
        auto *ax = new QTableWidgetItem(kAxisName[i]);
        ax->setFlags(Qt::NoItemFlags);
        ax->setTextAlignment(Qt::AlignCenter);
        auto axF = ax->font(); axF.setBold(true); ax->setFont(axF);
        tbl->setItem(i, 0, ax);
```

其余三列的列索引整体 +1（`setItem(i, 1, act)`、`setItem(i, 2, err)`、`setItem(i, 3, tgt)`）。

- [ ] **Step 3: 构建并截图核对**

Run: `cmake --build build --target rsi_host && ./build/rsi_host.exe &`
Run: `powershell -File tools/snap.ps1 -Title rsi_host -Out _t4.png`
Expected: 控制参数六个标签各自右侧有数值，对照 `config/rsi_config.json`
（kp 0.10/0.10、限速 10.0/2.0、上限 20.0/20.0）逐一相符；
位姿表最左列显示 X/Y/Z/A/B/C。

- [ ] **Step 4: 提交**

```bash
git add src/ui/MainWindow.cpp src/ui/MainWindow.h
git commit -m "fix(ui): param labels matched wrong values, axis names hidden

Labels sat at (1,0)(2,0)(1,2)... while values looped over (r+1, c*2+1) —
only Kp 位置 lined up. Axis names went into the vertical header one line
after it was hidden. Both now build label and value in one pass."
```

---

### Task 5: 表格布局修正 + RKorr 输出列

缺陷 D（固定高度截断）与新增 RKorr 列。

**Files:**
- Modify: `src/ui/MainWindow.h`（新增 `m_rkorrItem`）
- Modify: `src/ui/MainWindow.cpp`（`buildMidPanel`、`buildLeftPanel`、`onRefresh`）

**Interfaces:**
- Consumes: Task 2 的 `uilogic::formatValue` / `formatRkorr`；`StatusSnapshot::lastDelta`
- Produces: `std::array<QTableWidgetItem *, 6> m_rkorrItem{};`

- [ ] **Step 1: 加入 RKorr 列成员**

`src/ui/MainWindow.h` 的中栏表格项区追加：

```cpp
    // RKorr 输出：本帧实际发给 KRC 的增量。数据来自 StatusSnapshot::lastDelta，
    // 通信层早就在填，界面此前从未显示——操作员无从判断「主机到底发了什么」。
    std::array<QTableWidgetItem *, 6> m_rkorrItem{};
```

- [ ] **Step 2: 用行高计算表格高度**

新增私有辅助函数。在 `MainWindow.h` 的 `private:` 区加入声明：

```cpp
    // 按行高撑满 6 行。setFixedHeight(170) 装不下表头 + 6 行（实测仅 4 行
    // 可见），B/C 轴要滚动才看得到，而滚动条在栏边缘并不显眼——
    // 一个只显示四分之三数据的表格比没有表格更危险。
    static void fitTableToRows(QTableWidget *tbl, int rows);
```

在 `MainWindow.cpp` 的匿名 namespace 之后加入定义：

```cpp
void MainWindow::fitTableToRows(QTableWidget *tbl, int rows)
{
    tbl->resizeRowsToContents();
    int h = tbl->horizontalHeader()->height() + 2 * tbl->frameWidth();
    for (int r = 0; r < rows; ++r)
        h += tbl->rowHeight(r);
    tbl->setFixedHeight(h);
    tbl->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
}
```

在中栏表格与左栏表格创建的末尾各调用一次 `fitTableToRows(tbl, 6);`，
并删除原有的两处 `tbl->setFixedHeight(170);`。

`MainWindow.cpp` 顶部加入 `#include <QTableWidget>`（若尚未包含）。

- [ ] **Step 3: 创建并填充 RKorr 列**

中栏循环内，`m_targetItem[i]` 之后追加：

```cpp
        // RKorr 输出
        auto *rk = new QTableWidgetItem("--");
        rk->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        rk->setFlags(Qt::NoItemFlags);
        auto rkF = rk->font(); rkF.setFamily("Consolas"); rkF.setPointSize(10);
        rk->setFont(rkF);
        tbl->setItem(i, 4, rk);
        m_rkorrItem[i] = rk;
```

`onRefresh` 的六轴循环内追加：

```cpp
        // RKorr：零值灰、非零蓝——一眼看出哪个轴在动
        const double rk = (&s.lastDelta.x)[i];
        m_rkorrItem[i]->setText(uilogic::formatRkorr(rk, i));
        m_rkorrItem[i]->setForeground(
            std::fabs(rk) < 5e-5 ? QColor("#9CA3AF") : QColor("#2563EB"));
```

同时把该循环内原有的四处手写 `.arg(act[i], 0, 'f', 3).arg(units[i])` 替换为
`uilogic::formatValue(act[i], i)` 等，并删除现已无用的 `units[6]` 数组。

`MainWindow.cpp` 顶部加入 `#include "ui/UiLogic.h"`。

- [ ] **Step 4: 构建并核对**

Run: `cmake --build build --target rsi_host && ./build/rsi_host.exe &`
Run: `powershell -File tools/snap.ps1 -Title rsi_host -Out _t5.png`
Expected: 中栏表格六行全部可见（X 到 C），第五列显示 `0.0000 mm` 灰色；
左栏目标位姿表六行全部可见，无纵向滚动条。

- [ ] **Step 5: 提交**

```bash
git add src/ui/MainWindow.cpp src/ui/MainWindow.h
git commit -m "feat(ui): RKorr output column; size tables to fit all six axes

setFixedHeight(170) showed only four of six rows — B/C needed scrolling
past a scrollbar at the panel edge. lastDelta was already in the snapshot
but never displayed, so 'what did the host actually send' was unanswerable."
```

---

### Task 6: 接入告警边沿与按钮状态，修停止监听与撤销

缺陷 F、H、I、J 的界面接线。

**Files:**
- Modify: `src/ui/MainWindow.h`（新增 `m_prevAlarms`）
- Modify: `src/ui/MainWindow.cpp`（`onRefresh`、`onStopListening`、`onUndoTarget`、`onTargetEdited`、`onApplyTarget`、`updateConnControls`）

**Interfaces:**
- Consumes: Task 1 的 `uilogic::risingEdges` / `currentAlarms` / `buttonStates`；Task 2 的 `uilogic::deltaPreview`

- [ ] **Step 1: 加入上一帧告警状态成员**

`src/ui/MainWindow.h` 追加：

```cpp
    // 上一帧的告警状态，用于边沿触发。持续为真的告警每帧记一条，
    // 4 秒就能把 200 条上限刷满、挤掉之前的真实事件。
    AlarmEdge m_prevAlarms;
```

并加入 `#include "ui/UiLogic.h"`。

- [ ] **Step 2: onRefresh 改用边沿触发与统一按钮状态**

把 `onRefresh` 末尾「使能按钮状态机」整段替换为：

```cpp
    const ButtonStates btn = uilogic::buttonStates(s, m_listening);
    m_resetFaultBtn->setEnabled(btn.resetFault);
    m_enableBtn->setEnabled(btn.enableTrack);
    m_enableBtn->setText(s.state == ControlState::Tracking ? "已使能跟踪"
                                                           : "使能跟踪");
    m_stopBtn->setEnabled(btn.stopTrack);
    m_listenBtn->setEnabled(btn.startListen);
    m_unlistenBtn->setEnabled(btn.stopListen);
    m_ipEdit->setEnabled(btn.connEditable);
    m_portSpin->setEnabled(btn.connEditable);
```

把「事件日志」整段替换为：

```cpp
    // 边沿触发：只在告警「发生」时记一条，而不是在它「持续」的每一帧
    const AlarmEdge edge = uilogic::risingEdges(m_prevAlarms, s);
    if (edge.accumOverLimit)
        m_alarmLog->addEvent(AlarmLog::Fault,
            QStringLiteral("累计修正超限 位置 %1% 姿态 %2%")
                .arg(int(s.accumPosPct * 100)).arg(int(s.accumRotPct * 100)),
            "请停止跟踪并复位故障");
    if (edge.packetLoss)
        m_alarmLog->addEvent(AlarmLog::Warning,
            QStringLiteral("出现丢包，连续 %1 帧").arg(s.missedCount),
            "检查网络或 KRC 周期");
    m_prevAlarms = uilogic::currentAlarms(s);
```

`updateConnControls()` 现在与 `onRefresh` 中的按钮设置重复，但它在构造期
（尚无快照）与 `bindFailed` 回调里仍被需要。保留，函数体改为调用同一逻辑：

```cpp
void MainWindow::updateConnControls()
{
    const ButtonStates b = uilogic::buttonStates(m_state.snapshot(), m_listening);
    if (m_ipEdit)      m_ipEdit->setEnabled(b.connEditable);
    if (m_portSpin)    m_portSpin->setEnabled(b.connEditable);
    if (m_listenBtn)   m_listenBtn->setEnabled(b.startListen);
    if (m_unlistenBtn) m_unlistenBtn->setEnabled(b.stopListen);
}
```

- [ ] **Step 3: 停止监听时取消跟踪**

`onStopListening` 在 `stop` 之后、`updateConnControls` 之前插入：

```cpp
    // socket 一关就不可能再收发帧，跟踪也就无从谈起。不取消的话控制器
    // 状态会停在 Tracking，状态卡片持续显示「跟踪中」——覆盖一台已经
    // 断开的机器，这是最坏的一类显示错误。
    QMetaObject::invokeMethod(m_worker, "setTracking", Qt::QueuedConnection,
                              Q_ARG(bool, false));
```

- [ ] **Step 4: 撤销后重算预览，应用状态驱动按钮**

`onTargetEdited` 的预览计算替换为：

```cpp
void MainWindow::onTargetEdited()
{
    m_targetApplied = false;
    refreshDeltaPreview();
    updateApplyButton();
}
```

新增两个私有方法（`MainWindow.h` 声明，`MainWindow.cpp` 定义）：

```cpp
void MainWindow::refreshDeltaPreview()
{
    double tgt[6];
    for (int i = 0; i < 6; ++i)
        tgt[i] = m_targetSpin[i]->value();
    m_deltaPreview->setText(
        uilogic::deltaPreview(tgt, m_state.snapshot().actual));
}

void MainWindow::updateApplyButton()
{
    // m_targetApplied 原先只写不读：目标改了没发，界面毫无提示。
    // 用它驱动按钮高亮，「改了没发」一眼可见。
    m_applyBtn->setProperty("cssClass",
                            m_targetApplied ? "secondary" : "primary");
    m_applyBtn->style()->unpolish(m_applyBtn);
    m_applyBtn->style()->polish(m_applyBtn);
}
```

`m_applyBtn` 需提升为成员（`buildLeftPanel` 中原为局部变量 `apply`）。
`onApplyTarget`、`onUndoTarget`、`onReadActualTarget` 末尾各调用一次
`refreshDeltaPreview(); updateApplyButton();`，并删除三处手写的
`m_deltaPreview->setStyleSheet(...)` 与写死文案。

`MainWindow.cpp` 加入 `#include <QStyle>`。

- [ ] **Step 5: 构建并验证丢包只记一条**

Run: `cmake --build build --target rsi_host`
Run: 启动 `rsi_host` 与 `krc_simulator --drop 5`，展开事件日志
Expected: 丢包告警只有一条，而非每帧一条

Run: 点「停止监听」
Expected: 控制状态卡片不再显示「跟踪中」

- [ ] **Step 6: 提交**

```bash
git add src/ui/MainWindow.cpp src/ui/MainWindow.h
git commit -m "fix(ui): edge-triggered alarms, stop-listening clears tracking

Stopping the socket left the controller in Tracking, so the status card
kept claiming '跟踪中' over a disconnected machine. Undo restored values
with signals suppressed, leaving a stale delta preview. m_targetApplied
was written three times and never read — now it highlights Apply."
```

---

### Task 7: 样式统一——内联样式迁入 QSS

缺陷 C（选择器污染）与 L（双色板）。

**Files:**
- Modify: `src/ui/style.qss`
- Modify: `src/ui/StatusBar.cpp`、`src/ui/CommCards.cpp`、`src/ui/CumulativeBar.cpp`、`src/ui/AlarmLog.cpp`、`src/ui/ErrorChart.cpp`、`src/ui/MainWindow.cpp`

**Interfaces:**
- Produces: QSS 属性约定 `cssClass` ∈ {primary, secondary, warning, danger, fieldLabel, readout, cardTitle, cardValue, hint}；`state` ∈ {ok, warn, fault, idle}

- [ ] **Step 1: 扩充 style.qss**

在 `src/ui/style.qss` 末尾追加：

```css
/* ── 字号：工业现场 10px 偏小 ── */
QWidget { font-size: 12px; }

/* ── 状态卡片 ──
   注意用 #objectName 精确限定。原实现对 QFrame 用 "QFrame { ... }"
   作为部件级样式表，而 QLabel 继承自 QFrame，于是卡片里每个标签都
   套上了白底+边框+圆角，看起来像一排输入框。 */
QFrame#statusCard {
    background-color: #FFFFFF;
    border: 1px solid #D9E0E7;
    border-radius: 6px;
}
QFrame#statusCard[state="ok"]    { background-color: #F0FDF4; border-color: #16A34A; }
QFrame#statusCard[state="warn"]  { background-color: #FFFBEB; border-color: #D97706; }
QFrame#statusCard[state="fault"] { background-color: #FEF2F2; border-color: #DC2626; }
QFrame#statusCard[state="idle"]  { background-color: #F9FAFB; border-color: #E5E7EB; }

QFrame#statusCard QLabel { border: none; background: transparent; }

QLabel[cssClass="cardTitle"] { font-size: 11px; color: #64748B; font-weight: bold; }
QLabel[cssClass="cardValue"] { font-size: 15px; font-weight: bold; }
QFrame#statusCard[state="ok"]    QLabel[cssClass="cardValue"] { color: #16A34A; }
QFrame#statusCard[state="warn"]  QLabel[cssClass="cardValue"] { color: #D97706; }
QFrame#statusCard[state="fault"] QLabel[cssClass="cardValue"] { color: #DC2626; }
QFrame#statusCard[state="idle"]  QLabel[cssClass="cardValue"] { color: #9CA3AF; }

/* ── 数值读数：等宽、右对齐，位数变化时不跳动 ── */
QLabel[cssClass="readout"] {
    font-family: Consolas, "Courier New", monospace;
    font-size: 13px;
    color: #1F2937;
}
QLabel[cssClass="fieldLabel"] { color: #64748B; }

/* ── 提示条 ── */
QLabel[cssClass="hint"] {
    color: #64748B;
    background-color: #F3F4F6;
    border-radius: 4px;
    padding: 5px 10px;
}
QLabel[cssClass="hint"][state="warn"]  { color: #92400E; background-color: #FEF3C7; }
QLabel[cssClass="hint"][state="ok"]    { color: #166534; background-color: #F0FDF4; }
QLabel[cssClass="hint"][state="fault"] {
    color: #DC2626; background-color: #FEF2F2;
    border: 1px solid #FECACA; font-weight: bold;
}

/* ── 进度条状态色 ── */
QProgressBar[state="ok"]::chunk    { background-color: #16A34A; }
QProgressBar[state="warn"]::chunk  { background-color: #D97706; }
QProgressBar[state="fault"]::chunk { background-color: #DC2626; }

/* ── 表格 ── */
QTableWidget { border: 1px solid #D9E0E7; border-radius: 4px; }
QTableWidget::item { padding: 2px 8px; }
```

- [ ] **Step 2: StatusBar 改用 objectName + property**

`StatusBar.cpp` 的 `makeCard` 中把 `c.frame->setStyleSheet(...)` 替换为：

```cpp
        c.frame->setObjectName("statusCard");
        c.frame->setProperty("state", "idle");
```

`c.label` 与 `c.status` 的 `setStyleSheet` 替换为：

```cpp
        c.label->setProperty("cssClass", "cardTitle");
        c.status->setProperty("cssClass", "cardValue");
```

`Card::set` 签名由 `(icon, label, status, bg, fg)` 改为 `(icon, status, state)`：

```cpp
void StatusBar::Card::set(const QString &iconText, const QString &statusText,
                          const char *state)
{
    icon->setText(iconText);
    status->setText(statusText);
    frame->setProperty("state", state);
    // 属性变化后必须重新 polish，否则 QSS 属性选择器不会重新求值
    frame->style()->unpolish(frame);
    frame->style()->polish(frame);
}
```

`updateFrom` 中全部调用点相应简化，例如：

```cpp
    if (s.connected)      m_netCard.set("●", "已连接", "ok");
    else if (listening)   m_netCard.set("◐", "监听中", "idle");
    else                  m_netCard.set("○", "未监听", "idle");
```

`setWarning` 改为：

```cpp
void StatusBar::setWarning(const QString &text, bool isFault)
{
    m_warning->setText(text);
    m_warning->setProperty("cssClass", "hint");
    m_warning->setProperty("state", isFault ? "fault" : "");
    m_warning->style()->unpolish(m_warning);
    m_warning->style()->polish(m_warning);
    m_warning->setVisible(!text.isEmpty());
}
```

`StatusBar.h` 同步修改 `Card::set` 声明，并加入 `#include <QStyle>` 到 `.cpp`。

- [ ] **Step 3: CommCards / CumulativeBar 统一到 Tailwind 色板**

`CommCards.cpp`：`Card::setColors(const char*, const char*)` 改为
`Card::setState(const char *state)`，内部设 `frame` 的 `state` 属性并 polish；
调用点 `#d4edda/#28a745` → `"ok"`、`#fff3cd/#ffc107` → `"warn"`、
`#f8d7da/#dc3545` → `"fault"`、`#f5f5f5/#ccc` → `"idle"`。
卡片 frame 加 `setObjectName("statusCard")`，标题/数值加对应 `cssClass`。

回包卡片（缺陷 E）改为单值显示：

```cpp
    // StatusSnapshot 没有「本帧回包耗时」字段，只有会话最大值。
    // 原实现两行都取 maxReplyUs，「当前」是假的。
    m_reply.line1->setText(QStringLiteral("%1").arg(s.maxReplyUs, 0, 'f', 0));
    m_reply.line2->setText("会话最大值");
```

并把 `makeCard(m_reply, "回包 µs", 1);` 的标题改为 `"最大回包 µs"`。

`CumulativeBar.cpp`：删除 `r.bar->setStyleSheet(...)` 与
`r.status->setStyleSheet(...)`，改为设 `state` 属性并 polish；
数值/状态标签加 `cssClass="readout"` 与 `"fieldLabel"`。

- [ ] **Step 4: 清除 MainWindow / AlarmLog / ErrorChart 的内联样式**

`MainWindow.cpp`：删除全部 `setStyleSheet` 调用，改为设 `cssClass`——
目标 spinbox 无需特殊样式（QSS 里 `QDoubleSpinBox` 已统一）；
`m_liveLabel` → `cssClass="readout"`；`m_deltaPreview` → `cssClass="hint"`；
`m_interlockLabel` → `cssClass="hint"` + `state="fault"`；
左栏 `setCur`/`undo` 保留 `cssClass="secondary"`，`m_applyBtn` 由
Task 6 的 `updateApplyButton` 管理。

`AlarmLog.cpp`：`m_toggle`/`m_export`/`m_clear`/`m_table` 的 `setStyleSheet`
全部删除，`m_toggle` 设 `cssClass="secondary"`。
`ErrorChart.cpp`：`m_placeholder->setStyleSheet(...)` 删除，设
`cssClass="fieldLabel"`。

- [ ] **Step 5: 确认无残留内联样式并截图**

Run: `grep -rn "setStyleSheet" src/ui/`
Expected: 无输出（`main.cpp` 的 `app.setStyleSheet` 不在 `src/ui/` 下）

Run: `cmake --build build --target rsi_host && ./build/rsi_host.exe &`
Run: `powershell -File tools/snap.ps1 -Title rsi_host -Out _t7.png`
Expected: 四张状态卡片内部无多余边框（不再像输入框）；绿色只有一种

- [ ] **Step 6: 提交**

```bash
git add src/ui/
git commit -m "style(ui): single QSS source, fix selector leaking into cards

setStyleSheet(\"QFrame{...}\") on a card matched its QLabel children —
QLabel derives from QFrame — so every label inside wore a border and
looked like an input. Cards now use #statusCard plus a state property.
Also merges the Bootstrap and Tailwind palettes into one."
```

---

### Task 8: 布局重整——QSplitter 三栏

**Files:**
- Modify: `src/ui/MainWindow.cpp`（构造函数、`buildRightPanel`）
- Modify: `src/ui/MainWindow.h`

- [ ] **Step 1: 三栏改 QSplitter**

构造函数中把 `body` 整段替换为：

```cpp
    // 三栏用 QSplitter 而非固定宽度。原实现左 360 + 中 420 = 780px 写死，
    // 右栏拿剩下的——1400px 窗口下右栏只剩 600px 却要竖排四个部件，
    // 实测通信卡片被完全挤出可视区。给最小宽度、让用户自己分配。
    auto *body = new QSplitter(Qt::Horizontal, this);
    body->setChildrenCollapsible(false);
    body->setHandleWidth(6);

    auto *leftPanel = buildLeftPanel();
    leftPanel->setMinimumWidth(300);
    body->addWidget(leftPanel);

    auto *midPanel = buildMidPanel();
    midPanel->setMinimumWidth(360);
    body->addWidget(midPanel);

    auto *rightPanel = buildRightPanel();
    rightPanel->setMinimumWidth(320);
    body->addWidget(rightPanel);

    body->setStretchFactor(0, 3);
    body->setStretchFactor(1, 4);
    body->setStretchFactor(2, 5);
    outer->addWidget(body, 1);
```

删除原有的两处 `setFixedWidth`。`MainWindow.cpp` 已包含 `<QSplitter>`。

- [ ] **Step 2: 通信卡片移到图表下方**

`buildRightPanel` 中把 `m_commCards` 的 `addWidget` 移到两张图表之后
（当前已在图表后，仅需确认顺序为：位置图 → 姿态图 → 通信卡片），
并给图表 stretch：

```cpp
    v->addWidget(m_chartPos, 1);
    v->addWidget(m_chartRot, 1);
    v->addWidget(m_commCards);   // 不给 stretch：固定高度
```

- [ ] **Step 3: 安全提示移到按钮栏**

构造函数中，`btnBar->addStretch();` 之后加入：

```cpp
    // 提示说的是「停止跟踪」这个按钮的性质，就该在它旁边，
    // 而不是在顶部状态区里当一条通用横幅。
    m_safetyNote = new QLabel(
        "「停止跟踪」是软停止（RKorr=0，RSI 回包保持），不是急停。"
        "急停只能用示教器上的物理按钮。", this);
    m_safetyNote->setProperty("cssClass", "hint");
    btnBar->addWidget(m_safetyNote);
```

`MainWindow.h` 加入 `QLabel *m_safetyNote = nullptr;`。
删除构造函数中原有的 `m_statusBar->setWarning(...)` 初始调用；
`onRefresh` 中的 `setWarning` 改为只在故障时显示故障信息：

```cpp
    const bool isFault = (s.state == ControlState::Fault) || s.accumOverLimit;
    m_statusBar->setWarning(
        isFault ? QStringLiteral("故障：%1").arg(
                      s.faultReason.isEmpty() ? "跟踪已停止，请检查累计修正与通信"
                                              : s.faultReason)
                : QString(),
        isFault);
```

- [ ] **Step 4: 窄窗口验证**

Run: `cmake --build build --target rsi_host && ./build/rsi_host.exe &`
Run: 手动把窗口拖到约 1280×800，`powershell -File tools/snap.ps1 -Title rsi_host -Out _t8.png`
Expected: 三栏均可见，通信卡片在右栏图表下方可见，无部件被挤出；
拖动分隔条可重新分配宽度。

- [ ] **Step 5: 提交**

```bash
git add src/ui/MainWindow.cpp src/ui/MainWindow.h
git commit -m "layout(ui): splitter columns, comm cards under charts

Left 360 + mid 420 were hard-coded, leaving the right column 600px for
four stacked widgets — the comm cards fell off-screen. Minimum widths
plus stretch factors let the operator allocate. The soft-stop note moves
next to the button it describes."
```

---

### Task 9: 全量回归与真机核对

**Files:**
- Modify: `docs/superpowers/specs/2026-08-02-rsi-host-ui-design.md`（若实现与设计有偏差，据实更新）

- [ ] **Step 1: 全部单测通过**

Run: `cmake --build build && ctest --test-dir build --output-on-failure 2>&1 | tail -20`
Expected: 全部测试 Passed，含新增的 `test_ui_logic`

- [ ] **Step 2: 联机核对**

启动模拟器（注意 PATH 需含 Qt bin，模拟器依赖 Qt6Widgets）：

```bash
export PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH"
./build/rsi_host.exe &
./build/tools/krc_simulator/krc_simulator.exe --host 127.0.0.1 --port 59152 --cycles 0 &
```

在界面把 IP 改为 `127.0.0.1`，点「开始监听」。

Run: `powershell -File tools/snap.ps1 -Title rsi_host -Out _final.png`

逐项核对：
- 控制参数六个标签与 `config/rsi_config.json` 的值一一相符
- 位姿表六行轴名齐全，RKorr 列在使能跟踪后出现非零蓝色数值
- 四张状态卡片内部无多余边框
- 图表两条曲线正常绘制，无残留按钮
- 事件日志展开后条数合理（非每帧一条）

- [ ] **Step 3: 停止监听行为核对**

点「停止监听」。
Expected: 控制状态卡片不再显示「跟踪中」；IP/端口重新可编辑；
「开始监听」重新可用。

- [ ] **Step 4: 提交收尾**

```bash
git add -A
git commit -m "test(ui): full regression after UI refactor"
```

---

## Self-Review

**Spec 覆盖检查：**

| Spec 条目 | 对应任务 |
|---|---|
| A 参数串行 | Task 4 |
| B 轴名隐藏 | Task 4 |
| C 选择器污染 | Task 7 |
| D 表格截断 | Task 5 |
| E 回包卡同值 | Task 7 Step 3 |
| F 日志刷屏 | Task 1 + Task 6 |
| G 暂停逻辑 | Task 3（随按钮移除） |
| H 停止监听不取消跟踪 | Task 6 Step 3 |
| I 撤销后预览不刷新 | Task 6 Step 4 |
| J `m_targetApplied` 只写不读 | Task 6 Step 4 |
| K 判空恒假 | Task 2 |
| L 双色板 | Task 7 |
| 砍四个部件 | Task 3 |
| RKorr 列 | Task 2（格式化）+ Task 5（表格） |
| 布局重整 | Task 8 |
| 视觉系统 | Task 7 |
| 交互状态逻辑 | Task 1 + Task 6 |
| 验证 | Task 9 |

全部覆盖，无遗漏。

**类型一致性：** `AlarmEdge`、`ButtonStates` 在 Task 1 定义，Task 6 使用，字段名一致；
`uilogic::formatValue` / `formatRkorr` / `deltaPreview` 在 Task 2 定义，Task 5、Task 6 使用，签名一致；
`fitTableToRows` 在 Task 5 定义并使用；`m_applyBtn` 在 Task 6 提升为成员，Task 7 引用其 `cssClass`。
