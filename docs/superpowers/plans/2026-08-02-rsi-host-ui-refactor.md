# rsi_host 界面重构实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修正 `rsi_host` 界面的 12 个缺陷，砍掉 4 个装饰性部件，新增 RKorr 输出列，并把主窗口改成菜单栏 + 可停靠面板 + 状态栏的原生 Qt 外观。

**Architecture:** 把"会算错"的界面逻辑（告警边沿触发、数值格式化、按钮启用状态）抽成 `src/ui/UiLogic.h/.cpp` 纯函数，进 `rsi_core` 库并用 QtTest 覆盖——这些正是当前出 bug 的地方，也是唯一能脱离窗口测试的部分。纯视觉问题（布局、选择器污染）无法单测，用截图核对验证。`src/net/`、`src/core/` 的通信与控制逻辑一行不改。

**Tech Stack:** Qt 6.5.3 (Widgets, Charts, Test)、CMake + Ninja、MinGW、QtTest、CTest。

## Global Constraints

- Qt 版本 6.5.3，MinGW 64 位；构建目录 `build`（Debug，Ninja 已配置）。
- 通信层 `src/net/RsiWorker.*`、`src/net/SharedState.h`、`src/core/` 下所有文件**不得修改**（`SharedState.h` 只读不写）。
- **不使用 QSS**（2026-08-02 变更，见 spec 第六节）。界面为原生 Qt 外观，`src/ui/` 下不得出现 `setStyleSheet`。语义色只经 `QPalette` / `QTableWidgetItem::setForeground` 施加，取自 `uilogic::severityColor`：正常 `#16A34A`、警告 `#D97706`、故障 `#DC2626`、空闲 `#6B7280`。
- 等宽字体用 `QFontDatabase::systemFont(QFontDatabase::FixedFont)`，不硬编码 `Consolas`——未装该字体的机器会静默回退到比例字体，小数点从此不对齐。
- 颜色永远伴随文字（"正常"/"警告"/"超限"），不单独依赖颜色传达状态。
- RKorr 数值固定 4 位小数（与 `RsiCodec::buildSen` 的线上量化位数一致）；位姿/误差 3 位小数。
- 新增测试一律 `QtTest` + `tests/CMakeLists.txt` 注册 `add_test`，风格对齐 `tests/test_session_guard.cpp`。
- 注释用中文，说明「为什么」而非「做了什么」，与现有代码一致。
- 每个任务结束必须 `cmake --build build --target rsi_host` 通过后再提交。
- 涉及显示的改动必须实际截图核对（`powershell -File tools/snap.ps1 -Title rsi_host -Out _x.png`，再用 Read 打开看），不得以"代码看起来对"代替——本次要修的缺陷全都是代码看起来对但显示错。

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

缺陷 D（固定高度截断）与新增 RKorr 列。另修 Task 4 审查发现的横向溢出。

**Files:**
- Modify: `src/ui/UiLogic.h` / `src/ui/UiLogic.cpp`（新增 `monospaceFont`）
- Modify: `src/ui/MainWindow.h`（新增 `m_rkorrItem`）
- Modify: `src/ui/MainWindow.cpp`（`buildMidPanel`、`buildLeftPanel`、`onRefresh`）

**Interfaces:**
- Consumes: Task 2 的 `uilogic::formatValue` / `formatRkorr`；`StatusSnapshot::lastDelta`
- Produces:
  - `std::array<QTableWidgetItem *, 6> m_rkorrItem{};`
  - `QFont uilogic::monospaceFont();` —— Task 7、8 也要用

- [ ] **Step 0: 加入 monospaceFont**

`src/ui/UiLogic.h` 的 `namespace uilogic` 内追加：

```cpp
// 数值显示用的系统等宽字体。硬编码 "Consolas" 在没装该字体的机器上会
// 静默回退到比例字体，小数点从此不对齐——而对齐正是读数列存在的理由。
QFont monospaceFont();
```

`src/ui/UiLogic.cpp` 追加：

```cpp
QFont monospaceFont()
{
    return QFontDatabase::systemFont(QFontDatabase::FixedFont);
}
```

头文件加 `#include <QFont>`，实现文件加 `#include <QFontDatabase>`。

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
        rk->setFont(uilogic::monospaceFont());   // 系统等宽，不硬编码 Consolas
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
    // 用它驱动按钮的默认态，「改了没发」一眼可见。
    // 用 setDefault 而不是配色：原生按钮的「默认按钮」有平台自带的
    // 视觉强调（Windows 上是高亮边框），且回车键会触发它——正是
    // 「刚改完值，按回车发出去」这个动作。
    m_applyBtn->setDefault(!m_targetApplied);
    m_applyBtn->setEnabled(!m_targetApplied);
}
```

`m_applyBtn` 需提升为成员（`buildLeftPanel` 中原为局部变量 `apply`），
类型用 `QPushButton *`，且 `setAutoDefault(true)` 才能让 `setDefault`
生效。
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

### Task 7: 移除自定义样式，回到原生 Qt 外观

**背景（2026-08-02 计划变更）**：原 Task 7 是"把内联样式统一进 QSS"。
人类看过 Robotics Library 的 `rlPlanDemo`（`ref/fig/ref.png`、
`ref/rlPlanDemo/`）后决定改为该程序的做法——**它全程没有一处
`setStyleSheet`，完全是原生 Qt 外观**。因此本任务由"统一样式"反转为
"删除样式"。

删除而非统一的理由（写进 commit message）：本次审查已经出现两起样式
缺陷——`QFrame` 选择器污染卡片内的 QLabel（`QLabel` 继承自 `QFrame`）、
内联样式与 QSS 两套色板并存。这两类都不会在原生外观下发生。另外硬编码
的浅色背景会覆盖操作员在系统层面设置的高对比度主题。

**Files:**
- Delete: `src/ui/style.qss`
- Modify: `src/main.cpp`（删除加载 qss 的整段）
- Modify: `src/ui/StatusBar.cpp`、`src/ui/CommCards.cpp`、
  `src/ui/CumulativeBar.cpp`、`src/ui/AlarmLog.cpp`、
  `src/ui/ErrorChart.cpp`、`src/ui/MainWindow.cpp`
- Modify: `tools/package.sh`（若其中拷贝 style.qss）

**Interfaces:**
- Produces: 状态色改由 `uilogic::stateColor(...)` 提供（见 Step 2），
  供 Task 8 的状态栏复用。

- [ ] **Step 1: 删除 qss 与其加载代码**

```bash
git rm src/ui/style.qss
```

`src/main.cpp` 删除这一整段：

```cpp
    // 加载全局样式表
    const QStringList qssPaths{...};
    for (const QString &qss : qssPaths) { ... }
```

检查 `tools/package.sh` 是否拷贝 `style.qss` 或 `config/style.qss`，
有则一并删除该行。

- [ ] **Step 2: 语义色改由纯函数提供，不再用样式表**

在 `src/ui/UiLogic.h` 的 `namespace uilogic` 内追加：

```cpp
// 状态语义等级。与 QSS 时代的 ok/warn/fault/idle 一一对应，
// 但现在只用来选一个 QColor，不再拼样式表字符串。
enum class Severity { Idle, Ok, Warn, Fault };

// 语义色。取值来自原 QSS 色板，但施加方式改为 QPalette /
// QTableWidgetItem::setForeground——样式表会级联到后代部件，
// 本次审查里 "QFrame{...}" 污染卡片内每个 QLabel 就是这么来的
//（QLabel 继承自 QFrame）。直接设颜色没有级联，也就没有这一类错误。
QColor severityColor(Severity s);
```

（`monospaceFont` 已在 Task 5 Step 0 加入，此处不重复。）

`src/ui/UiLogic.cpp` 追加实现：

```cpp
QColor severityColor(Severity s)
{
    switch (s) {
    case Severity::Ok:    return QColor(0x16, 0xA3, 0x4A);
    case Severity::Warn:  return QColor(0xD9, 0x77, 0x06);
    case Severity::Fault: return QColor(0xDC, 0x26, 0x26);
    case Severity::Idle:  break;
    }
    return QColor(0x6B, 0x72, 0x80);
}
```

头文件加 `#include <QColor>`。

- [ ] **Step 3: 清空 src/ui 下所有 setStyleSheet**

逐个文件处理，一律遵循同一原则：**删掉样式表调用，需要保留的语义
只用控件自带 API 表达**。

- `StatusBar.cpp`：`Card::set` 的 `frame->setStyleSheet(...)` 改为
  `frame->setFrameShape(QFrame::StyledPanel)` + 用 `QPalette` 设
  `QPalette::WindowText` 为 `uilogic::severityColor(...)`。
  卡片内数值标签的加粗改用 `QFont::setBold(true)`。
- `CommCards.cpp`：`Card::setColors` 改名 `setSeverity(uilogic::Severity)`，
  内部同样走 QPalette。
- `CumulativeBar.cpp`：进度条颜色删掉样式表——`QProgressBar` 的
  chunk 颜色在原生样式下不可靠地可控。改为进度条保持原生外观，
  由右侧的状态文字（"正常"/"注意"/"警告"/"超限"）+ 文字颜色承载语义。
  这也符合"颜色永远伴随文字"的既定原则。
- `AlarmLog.cpp`、`ErrorChart.cpp`、`MainWindow.cpp`：删除全部
  `setStyleSheet`；等宽数值改用 `setFont(uilogic::monospaceFont())`；
  提示条的背景色不再需要——原生 `QLabel` 加 `QFrame::StyledPanel`
  边框即可区分。

- [ ] **Step 4: 验证无残留并截图**

Run: `grep -rn "setStyleSheet\|cssClass" src/ main.cpp`
Expected: 无输出

Run:
```bash
export PATH="/d/Software/QT/content/6.5.3/mingw_64/bin:$PATH"
cmake --build build --target rsi_host
./build/rsi_host.exe &
sleep 3
powershell -File tools/snap.ps1 -Title rsi_host -Out _t7.png
```
用 Read 工具打开 `_t7.png` 亲眼核对：界面为原生 Windows 外观，
无自定义配色；状态文字仍带语义色；数值列仍等宽对齐。

- [ ] **Step 5: 提交**

```bash
git add -A src/ui src/main.cpp tools/package.sh
git commit -m "style(ui): drop custom stylesheets for native Qt appearance

Two styling defects surfaced in review: a QFrame selector cascading onto
every QLabel inside a card (QLabel derives from QFrame), and inline
styles carrying a second palette alongside the QSS one. Neither can
happen without stylesheets. Hard-coded light backgrounds also override
whatever high-contrast theme the operator set system-wide.

Semantic colours now come from uilogic::severityColor and are applied
through QPalette, which does not cascade."
```

---

### Task 8: 菜单栏 + 可停靠面板 + 状态栏

把主窗口改成 `rlPlanDemo` 的组织方式。参考实现在
`ref/rlPlanDemo/rlPlanDemo/MainWindow.cpp`——菜单构建见其 `init()`
（约 557-760 行），停靠面板见构造函数（约 253-272 行）。

**Files:**
- Modify: `src/ui/MainWindow.h`、`src/ui/MainWindow.cpp`

**Interfaces:**
- Consumes: Task 7 的 `uilogic::severityColor`；既有各面板部件
  （`buildLeftPanel` / `buildMidPanel` / `buildRightPanel` 的产物）
- Produces: 各面板成为独立的 `QDockWidget`，其
  `toggleViewAction()` 进「视图」菜单

- [ ] **Step 1: 拆分面板构建函数**

现有三个 `buildXxxPanel()` 各自返回一个塞了多个 `QGroupBox` 的
`QWidget`。改为每个逻辑区块一个函数，各返回一个可直接放进
`QDockWidget` 的部件：

```cpp
    QWidget *buildListenPanel();    // 监听配置（IP / 端口）
    QWidget *buildTargetPanel();    // 目标位姿编辑
    QWidget *buildComparePanel();   // 位姿对比表（含 RKorr 列）
    QWidget *buildCumulPanel();     // 累积修正
    QWidget *buildParamPanel();     // 控制参数
    QWidget *buildCommPanel();      // 通信指标
```

各面板内部不再套 `QGroupBox`——`QDockWidget` 自带标题栏，
再套一层分组框是重复的边框。

- [ ] **Step 2: 建立停靠面板**

构造函数中央部件改为图表，其余全部入停靠区：

```cpp
    // 中央部件是误差图表：它是唯一需要大面积且持续观察的东西。
    // 其余面板都是"看一眼确认数值"的性质，适合停靠、按需调出。
    auto *charts = new QWidget(this);
    auto *cv = new QVBoxLayout(charts);
    cv->setContentsMargins(0, 0, 0, 0);
    m_chartPos = new ErrorChart(m_cfg.chartWindowS, ErrorChart::Mode::Position, charts);
    m_chartRot = new ErrorChart(m_cfg.chartWindowS, ErrorChart::Mode::Rotation, charts);
    cv->addWidget(m_chartPos, 1);
    cv->addWidget(m_chartRot, 1);
    setCentralWidget(charts);

    auto addDock = [this](const QString &title, QWidget *w,
                          Qt::DockWidgetArea area) -> QDockWidget * {
        auto *d = new QDockWidget(title, this);
        d->setObjectName(title);   // saveState/restoreState 靠它认面板
        d->setWidget(w);
        addDockWidget(area, d);
        return d;
    };

    m_listenDock  = addDock("监听配置", buildListenPanel(),  Qt::LeftDockWidgetArea);
    m_targetDock  = addDock("目标位姿", buildTargetPanel(),  Qt::LeftDockWidgetArea);
    m_compareDock = addDock("位姿对比", buildComparePanel(), Qt::LeftDockWidgetArea);
    m_cumulDock   = addDock("累积修正", buildCumulPanel(),   Qt::RightDockWidgetArea);
    m_paramDock   = addDock("控制参数", buildParamPanel(),   Qt::RightDockWidgetArea);
    m_commDock    = addDock("通信指标", buildCommPanel(),    Qt::RightDockWidgetArea);
    m_alarmDock   = addDock("事件日志", m_alarmLog,          Qt::BottomDockWidgetArea);
    m_alarmDock->hide();   // 默认隐藏，从「视图」菜单调出
```

- [ ] **Step 3: 菜单栏**

```cpp
void MainWindow::buildMenus()
{
    // ── 监听 ──
    QMenu *listenMenu = menuBar()->addMenu("监听(&L)");
    m_startListenAct = listenMenu->addAction("开始监听");
    m_startListenAct->setShortcut(QKeySequence("F5"));
    connect(m_startListenAct, &QAction::triggered, this, &MainWindow::onStartListening);
    m_stopListenAct = listenMenu->addAction("停止监听");
    connect(m_stopListenAct, &QAction::triggered, this, &MainWindow::onStopListening);

    // ── 控制 ──
    QMenu *ctlMenu = menuBar()->addMenu("控制(&C)");
    m_enableAct = ctlMenu->addAction("使能跟踪");
    m_enableAct->setShortcut(QKeySequence("F9"));
    connect(m_enableAct, &QAction::triggered, this, &MainWindow::onPrepareTracking);
    m_stopTrackAct = ctlMenu->addAction("停止跟踪");
    m_stopTrackAct->setShortcut(QKeySequence("Esc"));
    connect(m_stopTrackAct, &QAction::triggered, this, &MainWindow::onStopTracking);
    ctlMenu->addSeparator();
    m_resetFaultAct = ctlMenu->addAction("复位故障");
    connect(m_resetFaultAct, &QAction::triggered, this, &MainWindow::onResetFault);
    ctlMenu->addSeparator();
    QAction *paramsAct = ctlMenu->addAction("编辑控制参数…");
    connect(paramsAct, &QAction::triggered, this, &MainWindow::onEditParams);

    // ── 视图：各面板的显示开关 ──
    // toggleViewAction() 直接给出带勾选状态的 QAction，
    // 显示状态与菜单勾选自动同步，不必自己维护。
    QMenu *viewMenu = menuBar()->addMenu("视图(&V)");
    for (QDockWidget *d : {m_listenDock, m_targetDock, m_compareDock,
                           m_cumulDock, m_paramDock, m_commDock, m_alarmDock})
        viewMenu->addAction(d->toggleViewAction());

    // ── 工具栏：安全关键动作 ──
    // 使能与停止同时留在工具栏。菜单里的动作要两次点击才触发，
    // 停止跟踪不该有这个延迟。
    QToolBar *tb = addToolBar("控制");
    tb->setObjectName("controlToolBar");
    tb->addAction(m_startListenAct);
    tb->addAction(m_stopListenAct);
    tb->addSeparator();
    tb->addAction(m_resetFaultAct);
    tb->addAction(m_enableAct);
    tb->addAction(m_stopTrackAct);
}
```

`MainWindow.h` 相应加入 `QAction *` 成员与 `QDockWidget *` 成员，
并加 `void buildMenus();`。

- [ ] **Step 4: 状态栏取代顶部状态卡片**

`StatusBar` 部件（四张卡片）删除，其信息移入 `QStatusBar`：

```cpp
void MainWindow::buildStatusBar()
{
    // 常驻标签用 addPermanentWidget（右侧），瞬时消息用 showMessage（左侧）。
    // 连接/控制状态是"随时想瞥一眼"的信息，占一整块面板不值得。
    m_connLabel  = new QLabel(this);
    m_stateLabel = new QLabel(this);
    m_ipocLabel  = new QLabel(this);
    m_cycleLabel = new QLabel(this);
    for (QLabel *l : {m_connLabel, m_stateLabel, m_ipocLabel, m_cycleLabel})
        statusBar()->addPermanentWidget(l);
    m_ipocLabel->setFont(uilogic::monospaceFont());
    m_cycleLabel->setFont(uilogic::monospaceFont());
}
```

`onRefresh` 中更新这四个标签，颜色用 `QPalette` +
`uilogic::severityColor`。联锁拦截原因改用
`statusBar()->showMessage(reason, 10000)`。

删除 `src/ui/StatusBar.h`、`src/ui/StatusBar.cpp`，
从 `CMakeLists.txt` 的 `rsi_host` 源列表移除。

- [ ] **Step 5: 布局持久化**

```cpp
MainWindow::~MainWindow()
{
    QSettings st("kuka_rsi_win", "rsi_host");
    st.setValue("geometry", saveGeometry());
    st.setValue("windowState", saveState());
    // ...既有的线程收尾...
}
```

构造函数末尾：

```cpp
    QSettings st("kuka_rsi_win", "rsi_host");
    restoreGeometry(st.value("geometry").toByteArray());
    restoreState(st.value("windowState").toByteArray());
```

注意 `restoreState` 依赖每个 dock 与 toolbar 的 `objectName`——
Step 2、3 里已经设好，漏设会导致该面板位置不被恢复且 Qt 打警告。

- [ ] **Step 6: 验证**

Run:
```bash
cmake --build build --target rsi_host
./build/rsi_host.exe &
sleep 3
powershell -File tools/snap.ps1 -Title rsi_host -Out _t8.png
```
用 Read 工具打开截图核对：
- 菜单栏四项（监听/控制/视图/帮助）齐全
- 中央是两张图表
- 各面板停靠在左右两侧，标题栏可见
- 底部状态栏显示连接状态、IPOC、周期

手工验证（截图看不出来的）：
- 拖动某个面板到另一侧，关闭程序再启动，确认位置被恢复
- 「视图」菜单勾掉某面板，确认它隐藏且勾选状态同步
- 把某面板拖出窗口成为浮动窗口，确认正常

- [ ] **Step 7: 提交**

```bash
git add -A src/ui CMakeLists.txt
git commit -m "layout(ui): menu bar, dockable panels, status bar

Follows Robotics Library's rlPlanDemo (ref/rlPlanDemo): one large
central view plus panels the operator arranges as the job needs, rather
than a fixed three-column split that pushed widgets off-screen at 1400px.

Charts take the centre — the only thing needing sustained attention.
Connection state moves to the status bar. Enable and stop stay on a
toolbar as well as in the menu: a menu action costs two clicks, which
stop-tracking should not.

Layout persists via QSettings."
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
| C 选择器污染 | Task 7（删除 QSS，此类缺陷从根上消失） |
| D 表格截断 | Task 5 |
| E 回包卡同值 | Task 7 Step 3 |
| F 日志刷屏 | Task 1 + Task 6 |
| G 暂停逻辑 | Task 3（随按钮移除） |
| H 停止监听不取消跟踪 | Task 6 Step 3 |
| I 撤销后预览不刷新 | Task 6 Step 4 |
| J `m_targetApplied` 只写不读 | Task 6 Step 4 |
| K 判空恒假 | Task 2 |
| L 双色板 | Task 7（两套色板一并删除） |
| 砍四个部件 | Task 3 |
| RKorr 列 | Task 2（格式化）+ Task 5（表格） |
| 菜单栏 + 停靠面板 + 状态栏 | Task 8 |
| 视觉系统（原生外观） | Task 7 |
| 交互状态逻辑 | Task 1 + Task 6 |
| 验证 | Task 9 |

全部覆盖，无遗漏。

**2026-08-02 计划变更的影响：** Task 7 由「统一进 QSS」反转为「删除 QSS」，
Task 8 由「QSplitter 三栏」改为「菜单栏 + QDockWidget + 状态栏」。
缺陷 C 与 L 的修法随之改变（从「统一样式」变为「不用样式表」），
但两者仍被覆盖，且新修法更彻底——样式表级联导致的缺陷类别整个消失。
Task 1-6 不受影响：它们修的是逻辑与数据绑定，与外观无关。

**类型一致性：** `AlarmEdge`、`ButtonStates` 在 Task 1 定义，Task 6 使用，字段名一致；
`uilogic::formatValue` / `formatRkorr` / `deltaPreview` 在 Task 2 定义，Task 5、Task 6 使用，签名一致；
`monospaceFont` 在 Task 5 Step 0 定义，Task 7、8 使用；
`severityColor` 与 `Severity` 在 Task 7 定义，Task 8 使用；
`fitTableToRows` 在 Task 5 定义并使用；
`m_applyBtn` 在 Task 6 提升为 `QPushButton *` 成员，用 `setDefault` 而非样式表表达"未应用"。
