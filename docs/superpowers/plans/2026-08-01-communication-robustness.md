# 通信健壮性加固 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 加固 RSI 上位机 UDP 会话/IPOC 状态机、增加启动联锁，并建立故障注入与 pcap 回放验证设施。

**Architecture:** 新增两个纯逻辑类 `IpocTracker`（IPOC 序列分类状态机）与 `SessionGuard`（联锁规则，纯函数），`RsiWorker` 只负责接线与会话安全（对端锁定/积压上限/写返回检查/接收缓冲/KRC Delay 保护）。验证靠扩展现有 `krc_simulator` 加故障注入开关 + 新建 `pcap_replay.py`。

**Tech Stack:** C++17 / Qt 6.5.3 (mingw_64, `D:/Software/QT/content/6.5.3/mingw_64`) / CMake / Ninja / Qt Test / Python 3 (标准库)。

## Global Constraints

- **Qt 6.5.3 + MinGW 11.2 + C++17**，构建：`cmake --build build`（改 CMakeLists 后 Ninja 会自动重配；失败则手动 `cmake -S . -B build -G Ninja -DCMAKE_PREFIX_PATH=D:/Software/QT/content/6.5.3/mingw_64 -DCMAKE_BUILD_TYPE=Debug`）。
- **QtTest 文本日志器在 stdout 非 tty 时不输出**：所有单元测试必须 `-o result.log,txt` 再 `cat`。
- **实时路径禁堆分配/日志/文件 IO**：`RsiWorker` 通信线程内不做任何输出与分配。
- **三个 KRC 文件只读**：`krc/PoseTrack.rsix`、`krc/PoseTrack.src`、`krc/PoseTrack_ethernet.xml` 一律不修改。
- **`RsiCodec` 只加 `<Delay>` 解析**，不做任何防御性 XML 加固（根元素/Type/大小/重复字段/NaN/senType 白名单全部不做）。
- **IpocTracker 分类语义**（spec 已确认）：Gap 帧回正常修正、更新 lastGood、丢包 += gapCount；Duplicate/Backward 只回零增量、不 step、不更新 lastGood、丢包 +1；只有 Normal 帧清零丢包；异常帧仍必须回包。
- **写失败连续 5 次 → Fault**（一次成功清零）；**KRC Delay 连续 3 帧递增 → Fault**（持平不算）；**积压上限 8 帧/轮**；**接收缓冲默认 1MB**。
- **联锁硬拦截无覆盖**：使能被拦时不得置勾，红字列出未通过项。
- **对端首帧锁定**：异源帧丢弃且不回包，计数 peerRejected。

---

### Task 1: IpocTracker 纯状态机

**Files:**
- Create: `src/core/IpocTracker.h`
- Create: `src/core/IpocTracker.cpp`
- Create: `tests/test_ipoc_tracker.cpp`
- Modify: `CMakeLists.txt`（rsi_core 源列表加 `src/core/IpocTracker.cpp`）
- Modify: `tests/CMakeLists.txt`（加 test_ipoc_tracker 目标）

**Interfaces:**
- Produces: `struct IpocEvent { enum Kind { First, Normal, Gap, Duplicate, Backward } kind; quint64 gapCount; }; class IpocTracker { IpocEvent classify(quint64 ipoc); quint64 lastGood() const; bool haveFirst() const; void reset(); };` Task 2 依赖。

- [ ] **Step 1: Write the failing test**

`tests/test_ipoc_tracker.cpp`:

```cpp
#include <QtTest>
#include "core/IpocTracker.h"

class TestIpocTracker : public QObject
{
    Q_OBJECT
private slots:
    void firstFrame_returnsFirst()
    {
        IpocTracker t;
        const IpocEvent ev = t.classify(1000);
        QCOMPARE(int(ev.kind), int(IpocEvent::First));
        QVERIFY(t.haveFirst());
        QCOMPARE(t.lastGood(), quint64(1000));
    }

    void normalSequence_returnsNormal()
    {
        IpocTracker t;
        t.classify(1000);
        const IpocEvent ev = t.classify(1001);
        QCOMPARE(int(ev.kind), int(IpocEvent::Normal));
        QCOMPARE(ev.gapCount, quint64(0));
        QCOMPARE(t.lastGood(), quint64(1001));
    }

    void duplicate_doesNotAdvanceLastGood()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1001);
        const IpocEvent ev = t.classify(1001);   // 重复
        QCOMPARE(int(ev.kind), int(IpocEvent::Duplicate));
        QCOMPARE(t.lastGood(), quint64(1001));   // 不推进
    }

    void backward_doesNotAdvanceLastGood()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1001);
        const IpocEvent ev = t.classify(999);    // 回退
        QCOMPARE(int(ev.kind), int(IpocEvent::Backward));
        QCOMPARE(t.lastGood(), quint64(1001));
    }

    void gap_countsMissingCycles()
    {
        IpocTracker t;
        t.classify(1000);
        const IpocEvent ev = t.classify(1004);   // 缺 1001-1003
        QCOMPARE(int(ev.kind), int(IpocEvent::Gap));
        QCOMPARE(ev.gapCount, quint64(3));
        QCOMPARE(t.lastGood(), quint64(1004));   // Gap 帧推进
    }

    void gapOfOne_isNormal()
    {
        IpocTracker t;
        t.classify(1000);
        const IpocEvent ev = t.classify(1001);
        QCOMPARE(int(ev.kind), int(IpocEvent::Normal));
        QCOMPARE(ev.gapCount, quint64(0));
    }

    void gap_thenNormal_stillWorks()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1004);                        // Gap
        const IpocEvent ev = t.classify(1005);
        QCOMPARE(int(ev.kind), int(IpocEvent::Normal));
        QCOMPARE(t.lastGood(), quint64(1005));
    }

    void reset_startsFreshSequence()
    {
        IpocTracker t;
        t.classify(1000);
        t.reset();
        QVERIFY(!t.haveFirst());
        const IpocEvent ev = t.classify(500);    // 重置后首帧，即使更小
        QCOMPARE(int(ev.kind), int(IpocEvent::First));
    }

    void duplicateThenNormal_resumesCorrectly()
    {
        IpocTracker t;
        t.classify(1000);
        t.classify(1001);
        t.classify(1001);                        // Duplicate，lastGood 仍 1001
        const IpocEvent ev = t.classify(1002);   // 正常
        QCOMPARE(int(ev.kind), int(IpocEvent::Normal));
    }
};
QTEST_MAIN(TestIpocTracker)
#include "test_ipoc_tracker.moc"
```

- [ ] **Step 2: Register the test and run to verify it fails**

`CMakeLists.txt` rsi_core 段改为：

```cmake
add_library(rsi_core STATIC
    src/core/AppConfig.cpp
    src/core/IpocTracker.cpp
    src/core/RsiCodec.cpp
    src/core/PoseController.cpp
)
```

`tests/CMakeLists.txt` 末尾追加：

```cmake
add_executable(test_ipoc_tracker test_ipoc_tracker.cpp)
target_link_libraries(test_ipoc_tracker PRIVATE rsi_core Qt6::Test)
add_test(NAME test_ipoc_tracker COMMAND test_ipoc_tracker)
```

```bash
cmake --build build
./build/tests/test_ipoc_tracker.exe -o result.log,txt; cat result.log
```

Expected: FAIL，编译报 `core/IpocTracker.h: No such file`。

- [ ] **Step 3: Write the minimal implementation**

`src/core/IpocTracker.h`:

```cpp
#pragma once
#include <QtGlobal>

// RSI IPOC 序列分类。纯状态机，无 IO、无 Qt 对象，可在通信线程内直接调用。
struct IpocEvent
{
    enum Kind { First, Normal, Gap, Duplicate, Backward };
    Kind    kind = Normal;
    quint64 gapCount = 0;   // 仅 Gap：前向跳号缺失的周期数
};

class IpocTracker
{
public:
    IpocEvent classify(quint64 ipoc);
    quint64  lastGood() const { return m_lastGood; }
    bool     haveFirst() const { return m_haveFirst; }
    void     reset();

private:
    bool    m_haveFirst = false;
    quint64 m_lastGood  = 0;
};
```

`src/core/IpocTracker.cpp`:

```cpp
#include "core/IpocTracker.h"

IpocEvent IpocTracker::classify(quint64 ipoc)
{
    IpocEvent ev;
    if (!m_haveFirst) {
        m_haveFirst = true;
        m_lastGood  = ipoc;
        ev.kind = IpocEvent::First;
        return ev;
    }
    if (ipoc == m_lastGood) {
        ev.kind = IpocEvent::Duplicate;      // 不推进 lastGood
        return ev;
    }
    if (ipoc < m_lastGood) {
        ev.kind = IpocEvent::Backward;       // 不推进 lastGood
        return ev;
    }
    // ipoc > m_lastGood
    if (ipoc == m_lastGood + 1) {
        ev.kind = IpocEvent::Normal;
    } else {
        ev.kind     = IpocEvent::Gap;
        ev.gapCount = ipoc - m_lastGood - 1;
    }
    m_lastGood = ipoc;
    return ev;
}

void IpocTracker::reset()
{
    m_haveFirst = false;
    m_lastGood  = 0;
}
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cmake --build build
./build/tests/test_ipoc_tracker.exe -o result.log,txt; cat result.log
```

Expected: `Totals: 10 passed, 0 failed`。

- [ ] **Step 5: Commit**

```bash
git add CMakeLists.txt tests/CMakeLists.txt src/core/IpocTracker.h src/core/IpocTracker.cpp tests/test_ipoc_tracker.cpp
git commit -m "feat(core): add IPOC sequence tracker"
```

---

### Task 2: RsiWorker 接线 IpocTracker

**Files:**
- Modify: `src/net/RsiWorker.h`
- Modify: `src/net/RsiWorker.cpp`

**Interfaces:**
- Consumes: `IpocTracker`（Task 1）。
- Produces: `RsiWorker` 不再持有 `m_lastIpoc`/`m_haveFirstFrame`，改用 `m_ipocTracker`；丢包计数语义改变（Gap 加缺口、Dup/Back +1、仅 Normal 清零）。Task 6 在此基础上继续加会话安全。

- [ ] **Step 1: Modify `RsiWorker.h`**

把 `#include "core/AppConfig.h"` 之前加一行 `#include "core/IpocTracker.h"`，并把成员区：

```cpp
    bool    m_haveFirstFrame = false;
    quint64 m_lastIpoc       = 0;
    int     m_missed         = 0;
    quint64 m_frameCount     = 0;
    double  m_maxReplyUs     = 0.0;
```

改为：

```cpp
    IpocTracker m_ipocTracker;
    int     m_missed         = 0;
    quint64 m_frameCount     = 0;
    double  m_maxReplyUs     = 0.0;
```

- [ ] **Step 2: Modify `RsiWorker.cpp` — `start()`**

`start()` 里：

```cpp
    m_sessionTimer.start();
    m_haveFirstFrame  = false;
    m_frameCount      = 0;
    m_missed          = 0;
    m_maxReplyUs      = 0.0;
    m_cycleTimerValid = false;
    m_lastIpoc        = 0;
    m_measuredCycleMs = 0.0;
```

改为：

```cpp
    m_sessionTimer.start();
    m_ipocTracker.reset();
    m_frameCount      = 0;
    m_missed          = 0;
    m_maxReplyUs      = 0.0;
    m_cycleTimerValid = false;
    m_measuredCycleMs = 0.0;
```

（`m_sinceLastFrame` 依旧刻意不动，注释保留原样。）

- [ ] **Step 3: Modify `RsiWorker.cpp` — `stop()`**

`stop()` 里：

```cpp
    m_haveFirstFrame = false;
    StatusSnapshot s;
```

改为：

```cpp
    m_ipocTracker.reset();
    StatusSnapshot s;
```

- [ ] **Step 4: Modify `RsiWorker.cpp` — `onWatchdog()`**

`onWatchdog()` 里两处 `m_haveFirstFrame`：

```cpp
    if (!m_haveFirstFrame)
        return;
```

改为：

```cpp
    if (!m_ipocTracker.haveFirst())
        return;
```

以及连接丢失处理处：

```cpp
    m_haveFirstFrame  = false;
    m_cycleTimerValid = false;   // 否则下个会话的首帧会把整段静默当作周期发布
```

改为：

```cpp
    m_ipocTracker.reset();
    m_cycleTimerValid = false;   // 否则下个会话的首帧会把整段静默当作周期发布
```

- [ ] **Step 5: Rewrite `onDatagram()` 的有效帧分支**

把 `onDatagram()` 中从 `const quint64 echoIpoc = ...` 到 `delta = m_ctl.step(f.rist);` 的整段（现为 `RsiWorker.cpp:134-180`），连同 `else { ++m_missed; }` 分支一起替换为：

```cpp
        // ── 无论解析成败，都必须回包 ──
        // codec 独立解析 IPOC，并在其不可信时留在默认值 0（见 RsiCodec 里的
        // hasError 守卫），所以 0 是可靠哨兵。只要 IPOC 本身解析成功就必须
        // 原样回显——哪怕 RIst 损坏导致整帧 invalid。回一个陈旧 IPOC 等同
        // 丢包，而那正是"任何分支都必须回包"这条约束要避免的后果。
        const quint64 echoIpoc = f.ipoc ? f.ipoc : m_ipocTracker.lastGood();
        Pose    delta;   // 默认零增量
        bool    wasFirstFrame = false;

        if (f.valid) {
            const IpocEvent ev = m_ipocTracker.classify(f.ipoc);

            if (ev.kind == IpocEvent::First) {
                // 只有确实静默过至少一个会话间隔，才算真正的 RSI 会话重启，
                // 才可以清零累积量。快速的 stop()→start() 不算：KRC 侧已施加
                // 的修正仍然存在，清零等于凭空多发一份预算，反复几次就能把
                // 总修正推过 POSCORR 的 ~50mm 硬限。
                // 用独立的会话间隔阈值，而不是看门狗间隔。看门狗只负责
                // "连接丢失"的显示，阈值必须小；会话判定则必须大于 KRC 的
                // Timeout，否则会在 KRC 仍认为会话连续时移动安全锚点。
                const bool genuineSessionStart =
                    !m_sinceLastFrame.isValid()
                    || m_sinceLastFrame.elapsed() >= qint64(m_cfg.sessionGapMs);
                if (genuineSessionStart)
                    m_ctl.beginSession(f.rist);
                else
                    m_ctl.resetToActual(f.rist);
                wasFirstFrame = true;
            }

            if (m_cycleTimerValid) {
                m_measuredCycleMs = m_cycleTimer.nsecsElapsed() / 1.0e6;
            }
            m_cycleTimer.start();
            m_cycleTimerValid = true;

            m_sinceLastFrame.restart();
            m_lastActual = f.rist;
            ++m_frameCount;

            // 丢包计数：仅 Normal 清零；Gap 加缺口；Dup/Back 各 +1。
            // Gap 帧带全新 RIst（非旧位姿重放），正常 step；Dup/Back 是旧数据
            // 重放，只回零增量且不推进 lastGood（IpocTracker 已保证），否则会
            // 用一帧旧位姿再产生一次修正。
            switch (ev.kind) {
            case IpocEvent::Normal:
                m_missed = 0;
                break;
            case IpocEvent::Gap:
                m_missed += int(ev.gapCount);
                break;
            case IpocEvent::Duplicate:
            case IpocEvent::Backward:
                ++m_missed;
                break;
            case IpocEvent::First:
                break;
            }

            if (ev.kind == IpocEvent::Normal || ev.kind == IpocEvent::Gap)
                delta = m_ctl.step(f.rist);
        } else {
            ++m_missed;
        }
```

- [ ] **Step 6: Build and run existing test suite**

```bash
cmake --build build
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state; do
  ./build/tests/$t.exe -o result.log,txt || echo "FAIL $t"
  grep "Totals" result.log
done
```

Expected: 每个测试 `passed, 0 failed`。

- [ ] **Step 7: End-to-end loopback regression**

```bash
./build/tools/krc_simulator.exe --host 127.0.0.1 --port 59152 --cycles 700 > /tmp/sim.log 2>&1 &
SIM=$!
./build/tools/loopback_test.exe --seconds 9 > /tmp/lb.log 2>&1
wait $SIM
cat /tmp/sim.log
cat /tmp/lb.log
```

Expected: `sim.log` 含 `replies=700 missed=0 ipoc_mismatch=0` 与 `PASS`；`lb.log` 显示约 700 帧、`missed=0`。确认正常路径无回归。

- [ ] **Step 8: Commit**

```bash
git add src/net/RsiWorker.h src/net/RsiWorker.cpp
git commit -m "refactor(net): route IPOC classification through IpocTracker"
```

---

### Task 3: RsiCodec 解析 `<Delay>`

**Files:**
- Modify: `src/core/RsiCodec.h`
- Modify: `src/core/RsiCodec.cpp`
- Modify: `tests/test_rsi_codec.cpp`

**Interfaces:**
- Produces: `RobFrame` 新增 `quint64 delay = 0;`。Task 6 的 KRC Delay 保护依赖。注意 spec 决定：**只加 Delay 解析，不做任何其它 XML 加固**。

- [ ] **Step 1: Add failing tests**

在 `tests/test_rsi_codec.cpp` 的 `private slots:` 内追加：

```cpp
    void parseRob_readsDelay()
    {
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1\" Y=\"2\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<Delay D=\"42\"/>"
            "<IPOC>5</IPOC>"
            "</Rob>";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.delay, quint64(42));
    }

    void parseRob_missingDelay_isValidDefaultZero()
    {
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1\" Y=\"2\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<IPOC>5</IPOC>"
            "</Rob>";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.delay, quint64(0));
    }

    void parseRob_badDelayKeepsDefault()
    {
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1\" Y=\"2\" Z=\"3\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<Delay D=\"abc\"/>"
            "<IPOC>5</IPOC>"
            "</Rob>";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.delay, quint64(0));
    }
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cmake --build build
./build/tests/test_rsi_codec.exe -o result.log,txt; cat result.log
```

Expected: 3 个新用例 FAIL（`delay` 恒为 0）。

- [ ] **Step 3: Modify `RsiCodec.h`**

`RobFrame` 加字段：

```cpp
struct RobFrame
{
    Pose    rist;           // 实际位姿
    Pose    rsol;           // 额定位姿
    quint64 ipoc  = 0;
    quint64 delay = 0;      // KRC 统计的迟到/丢失回包数（DEF_Delay）
    bool    valid = false;
};
```

- [ ] **Step 4: Modify `RsiCodec.cpp`**

在 `parseRob` 的 `else if (name == QLatin1String("IPOC")) {` 分支之前插入：

```cpp
        } else if (name == QLatin1String("Delay")) {
            // DEF_Delay 是 KRC 自己统计的迟到/丢失回包数，唯一能让主机看见
            // "KRC 认为我丢包了"的量。诊断字段，解析失败不拒绝整帧。
            const QStringView d = xml.attributes().value(QLatin1String("D"));
            bool ok = false;
            const quint64 v = d.toULongLong(&ok);
            if (ok)
                out.delay = v;
        }
```

- [ ] **Step 5: Run test to verify it passes**

```bash
cmake --build build
./build/tests/test_rsi_codec.exe -o result.log,txt; cat result.log
```

Expected: 全部用例 `passed, 0 failed`（含新增 3 个）。

- [ ] **Step 6: Commit**

```bash
git add src/core/RsiCodec.h src/core/RsiCodec.cpp tests/test_rsi_codec.cpp
git commit -m "feat(core): parse KRC <Delay> diagnostic"
```

---

### Task 4: AppConfig 新字段 + SessionGuard 联锁

**Files:**
- Modify: `src/core/AppConfig.h`
- Modify: `src/core/AppConfig.cpp`
- Modify: `config/rsi_config.json`
- Create: `src/core/SessionGuard.h`
- Create: `src/core/SessionGuard.cpp`
- Create: `tests/test_session_guard.cpp`
- Modify: `CMakeLists.txt`（rsi_core 加 `src/core/SessionGuard.cpp`）
- Modify: `tests/CMakeLists.txt`（加 test_session_guard 目标）

**Interfaces:**
- Consumes: `AppConfig`。
- Produces: 四个新配置字段（`krcTimeoutCycles`、`krcPoscorrLimitPosMm`、`krcPoscorrLimitRotDeg`、`rxBufferBytes`）与 `SessionGuard::staticChecks(const AppConfig&)`、`SessionGuard::enableChecks(const AppConfig&, double measuredCycleMs)`（返回 QStringList，空 = 通过）。Task 6/7 依赖。

- [ ] **Step 1: Add AppConfig fields**

`src/core/AppConfig.h`，在 `int watchdogMissLimit = 3;` 之后加：

```cpp
    // 联锁与运行时保护（见 SessionGuard）：
    int     krcTimeoutCycles      = 100;       // KRC ETHERNET Timeout（周期数）
    double  krcPoscorrLimitPosMm  = 25.0;      // KRC POSCORR 位置累积限值
    double  krcPoscorrLimitRotDeg = 25.0;      // KRC POSCORR 姿态累积限值
    int     rxBufferBytes         = 1048576;   // socket 接收缓冲（字节）
```

`src/core/AppConfig.cpp`，`loadFromFile` 里 `readInt(rsi, "watchdog_miss_limit", ...)` 之后加：

```cpp
    readInt(rsi, "krc_timeout_cycles", &out->krcTimeoutCycles);
    readDouble(rsi, "krc_poscorr_limit_pos_mm", &out->krcPoscorrLimitPosMm);
    readDouble(rsi, "krc_poscorr_limit_rot_deg", &out->krcPoscorrLimitRotDeg);
```

`net` 段加：

```cpp
    readInt(net, "rx_buffer_bytes", &out->rxBufferBytes);
```

`config/rsi_config.json`：`network` 对象加 `"rx_buffer_bytes": 1048576`；`rsi` 对象加：

```json
    "krc_timeout_cycles": 100,
    "krc_poscorr_limit_pos_mm": 25.0,
    "krc_poscorr_limit_rot_deg": 25.0
```

- [ ] **Step 2: Write the failing test**

`tests/test_session_guard.cpp`:

```cpp
#include <QtTest>
#include "core/SessionGuard.h"

namespace {

// 通过全部静态联锁的基准配置。注意默认 AppConfig 的 accumLimitPosMm=30 >
// krcPoscorrLimitPosMm=25，会被拦——必须显式调到安全值。
AppConfig good()
{
    AppConfig c = AppConfig::defaults();
    c.cycleMs           = 12.0;
    c.sessionGapMs      = 2000.0;    // > 100 × 12 = 1200
    c.accumLimitPosMm   = 20.0;      // < 25
    c.accumLimitRotDeg  = 20.0;      // < 25
    c.senType           = "ImFree";
    return c;
}

} // namespace

class TestSessionGuard : public QObject
{
    Q_OBJECT
private slots:
    void goodConfig_passes()
    {
        QVERIFY(SessionGuard::staticChecks(good()).isEmpty());
        QVERIFY(SessionGuard::enableChecks(good(), 12.5).isEmpty());
    }

    void cycleZero_fails()
    {
        AppConfig c = good();
        c.cycleMs = 0.0;
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void sessionGapZero_fails()
    {
        AppConfig c = good();
        c.sessionGapMs = 0.0;
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("session_gap_ms"));
    }

    void sessionGapBelowTimeoutProduct_fails()
    {
        AppConfig c = good();
        c.sessionGapMs = 500.0;      // < 100 × 12 = 1200
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void accumLimitOverKrc_fails()
    {
        AppConfig c = good();
        c.accumLimitPosMm = 30.0;    // 默认值 > 25
        const QStringList r = SessionGuard::staticChecks(c);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("accum_limit_pos_mm"));
    }

    void accumRotLimitOverKrc_fails()
    {
        AppConfig c = good();
        c.accumLimitRotDeg = 30.0;
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void senTypeEmpty_fails()
    {
        AppConfig c = good();
        c.senType = "   ";
        QVERIFY(!SessionGuard::staticChecks(c).isEmpty());
    }

    void measuredCycleDeviation_fails()
    {
        AppConfig c = good();
        const QStringList r = SessionGuard::enableChecks(c, 15.0);
        QVERIFY(!r.isEmpty());
        QVERIFY(r.join('\n').contains("cycle"));
    }

    void measuredCycleWithinTolerance_passes()
    {
        AppConfig c = good();
        QVERIFY(SessionGuard::enableChecks(c, 12.6).isEmpty());   // 差 0.6 < 1.2
    }

    void measuredCycleUnknown_doesNotFail()
    {
        AppConfig c = good();
        QVERIFY(SessionGuard::enableChecks(c, -1.0).isEmpty());   // 无实测不拦
    }
};
QTEST_MAIN(TestSessionGuard)
#include "test_session_guard.moc"
```

- [ ] **Step 3: Register test and run to verify it fails**

`CMakeLists.txt` rsi_core 加 `src/core/SessionGuard.cpp`：

```cmake
add_library(rsi_core STATIC
    src/core/AppConfig.cpp
    src/core/IpocTracker.cpp
    src/core/RsiCodec.cpp
    src/core/PoseController.cpp
    src/core/SessionGuard.cpp
)
```

`tests/CMakeLists.txt` 追加：

```cmake
add_executable(test_session_guard test_session_guard.cpp)
target_link_libraries(test_session_guard PRIVATE rsi_core Qt6::Test)
add_test(NAME test_session_guard COMMAND test_session_guard)
```

```bash
cmake --build build
./build/tests/test_session_guard.exe -o result.log,txt; cat result.log
```

Expected: 编译报 `core/SessionGuard.h: No such file`。

- [ ] **Step 4: Write the minimal implementation**

`src/core/SessionGuard.h`:

```cpp
#pragma once
#include <QString>
#include <QStringList>
#include "core/AppConfig.h"

// 联锁规则，纯函数，无 IO、无状态。返回未通过项的说明列表；为空即全部通过。
class SessionGuard
{
public:
    // 静态联锁：仅依据配置评估，绑定后即可查。
    static QStringList staticChecks(const AppConfig &cfg);

    // 动态联锁：使能跟踪时评估。measuredCycleMs < 0 表示尚无实测周期。
    static QStringList enableChecks(const AppConfig &cfg, double measuredCycleMs);
};
```

`src/core/SessionGuard.cpp`:

```cpp
#include "core/SessionGuard.h"

#include <cmath>

QStringList SessionGuard::staticChecks(const AppConfig &cfg)
{
    QStringList out;

    if (!(cfg.cycleMs > 0.0))
        out << QStringLiteral("cycle_ms=%1 must be > 0").arg(cfg.cycleMs);

    if (!(cfg.sessionGapMs > 0.0))
        out << QStringLiteral("session_gap_ms=%1 must be > 0").arg(cfg.sessionGapMs);

    // 会话判定阈值必须大于 KRC 的容忍度，否则存在窗口：KRC 认为会话未断、
    // 仍按原始起始位姿累计修正，而主机已把安全锚点移到当前位置并发放新预算。
    if (!(cfg.sessionGapMs > cfg.krcTimeoutCycles * cfg.cycleMs))
        out << QStringLiteral(
            "session_gap_ms=%1 must exceed krc_timeout_cycles(%2) × cycle_ms(%3) = %4")
                .arg(cfg.sessionGapMs)
                .arg(cfg.krcTimeoutCycles)
                .arg(cfg.cycleMs)
                .arg(cfg.krcTimeoutCycles * cfg.cycleMs);

    // 主机累计限值必须小于 KRC 侧 POSCORR 累积限值，否则第 2 层先于第 4 层触发，
    // 梯度就反了。当前配置 accum_limit_pos_mm=1000 会被此规则主动拦下。
    if (!(cfg.accumLimitPosMm < cfg.krcPoscorrLimitPosMm))
        out << QStringLiteral(
            "accum_limit_pos_mm=%1 must be < KRC poscorr limit %2")
                .arg(cfg.accumLimitPosMm)
                .arg(cfg.krcPoscorrLimitPosMm);
    if (!(cfg.accumLimitRotDeg < cfg.krcPoscorrLimitRotDeg))
        out << QStringLiteral(
            "accum_limit_rot_deg=%1 must be < KRC poscorr limit %2")
                .arg(cfg.accumLimitRotDeg)
                .arg(cfg.krcPoscorrLimitRotDeg);

    if (cfg.senType.trimmed().isEmpty())
        out << QStringLiteral("sen_type must be non-empty");

    return out;
}

QStringList SessionGuard::enableChecks(const AppConfig &cfg, double measuredCycleMs)
{
    QStringList out = staticChecks(cfg);
    // 实测周期与配置周期偏差 > 10% 即拦。首帧后才有实测值；未收到帧时
    // measuredCycleMs 保持 0，调用方传入 -1 表示"尚无实测"，不参与判定。
    if (measuredCycleMs > 0.0 && cfg.cycleMs > 0.0) {
        const double tol = 0.10 * cfg.cycleMs;
        if (std::fabs(measuredCycleMs - cfg.cycleMs) > tol)
            out << QStringLiteral(
                "measured cycle %1 ms deviates from configured %2 ms by more than 10%%")
                    .arg(measuredCycleMs)
                    .arg(cfg.cycleMs);
    }
    return out;
}
```

- [ ] **Step 5: Run test to verify it passes**

```bash
cmake --build build
./build/tests/test_session_guard.exe -o result.log,txt; cat result.log
```

Expected: `Totals: 10 passed, 0 failed`。

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt tests/CMakeLists.txt src/core/AppConfig.h src/core/AppConfig.cpp src/core/SessionGuard.h src/core/SessionGuard.cpp tests/test_session_guard.cpp config/rsi_config.json
git commit -m "feat(core): add session interlock guard and KRC limit config"
```

---

### Task 5: SharedState 新字段 + PoseController::forceFault

**Files:**
- Modify: `src/net/SharedState.h`
- Modify: `src/core/PoseController.h`
- Modify: `src/core/PoseController.cpp`
- Modify: `tests/test_pose_controller.cpp`

**Interfaces:**
- Produces: `StatusSnapshot` 新增 `quint64 krcDelay`、`int peerRejected`、`int sendFails`；`PoseController::forceFault(const QString &reason)`（锁存 Fault 直到 `resetToActual`）。Task 6 填充这些字段并调用 forceFault。

- [ ] **Step 1: Add failing test**

`tests/test_pose_controller.cpp` 的 `private slots:` 内追加：

```cpp
    void forceFault_latchesUntilReset()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.forceFault(QStringLiteral("network write failed"));
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(pc.faultReason().contains("network write failed"));
        QCOMPARE(pc.step(Pose{0, 0, 0, 0, 0, 0}).x, 0.0);
        pc.setTracking(true);   // 不得直接重新使能
        QCOMPARE(pc.state(), TrackState::Fault);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(pc.state(), TrackState::Idle);
    }
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cmake --build build
./build/tests/test_pose_controller.exe -o result.log,txt; cat result.log
```

Expected: 新用例编译失败（`forceFault` 不存在）。

- [ ] **Step 3: Modify `PoseController.h`**

在 `QString faultReason() const { return m_faultReason; }` 之后加：

```cpp
    // 外部（网络层）注入的锁存故障：写失败、KRC Delay 增长等。与内部判定的
    // Fault 一样，必须经 resetToActual 才能清除，绝不能被 setTracking(true) 绕过。
    void forceFault(const QString &reason);
```

- [ ] **Step 4: Modify `PoseController.cpp`**

在 `setTracking` 实现之后加：

```cpp
void PoseController::forceFault(const QString &reason)
{
    m_state = TrackState::Fault;
    m_faultReason = reason;
}
```

- [ ] **Step 5: Modify `SharedState.h`**

`StatusSnapshot` 加：

```cpp
    quint64 krcDelay        = 0;   // KRC 统计的迟到/丢失回包数（<Delay D=...>）
    int     peerRejected    = 0;   // 被对端锁定丢弃的异源帧数
    int     sendFails       = 0;   // writeDatagram 连续失败计数
```

- [ ] **Step 6: Run test to verify it passes**

```bash
cmake --build build
./build/tests/test_pose_controller.exe -o result.log,txt; cat result.log
```

Expected: 全部用例 `passed, 0 failed`。

- [ ] **Step 7: Commit**

```bash
git add src/net/SharedState.h src/core/PoseController.h src/core/PoseController.cpp tests/test_pose_controller.cpp
git commit -m "feat(net): surface krcDelay and peer/send stats; add forceFault"
```

---

### Task 6: RsiWorker 会话安全 + KRC Delay 保护

**Files:**
- Modify: `src/net/RsiWorker.h`
- Modify: `src/net/RsiWorker.cpp`

**Interfaces:**
- Consumes: `IpocTracker`（Task 1）、`<Delay>`（Task 3）、`StatusSnapshot` 新字段与 `forceFault`（Task 5）、`rxBufferBytes`（Task 4）。
- Produces: `RsiWorker` 具备对端锁定/积压上限/写返回检查/接收缓冲/KRC Delay 保护；`publishSnapshot` 填充新字段。

- [ ] **Step 1: Modify `RsiWorker.h`**

成员区 `m_ipocTracker;` 之后加：

```cpp
    // 会话安全
    QHostAddress m_peerAddr;
    quint16      m_peerPort     = 0;
    bool         m_peerLocked   = false;
    int          m_peerRejected = 0;
    int          m_sendFails    = 0;
    quint64      m_lastDelay    = 0;
    int          m_delayRising  = 0;

    static constexpr int kMaxBurst = 8;   // 每轮 onDatagram 最多处理的积压帧数
```

把现有的：

```cpp
    QHostAddress m_peerAddr;
    quint16      m_peerPort = 0;
```

删除（上面的新声明已含）。

- [ ] **Step 2: Modify `start()` — 设置接收缓冲**

`connect(m_sock, &QUdpSocket::readyRead, ...)` 之后加：

```cpp
    // 突发积压时内核缓冲兜底。Windows 上 SO_RCVBUF 是软上限，由 OS 决定实际值。
    m_sock->setSocketOption(
        QAbstractSocket::ReceiveBufferSizeSocketOption, m_cfg.rxBufferBytes);
```

- [ ] **Step 3: Modify `stop()` — 重置会话安全状态**

`stop()` 里 `m_ipocTracker.reset();` 之后加：

```cpp
    m_peerLocked   = false;
    m_peerRejected = 0;
    m_sendFails    = 0;
    m_lastDelay    = 0;
    m_delayRising  = 0;
```

- [ ] **Step 4: Rewrite `onDatagram()` 的外层循环与首部**

把 `while (m_sock && m_sock->hasPendingDatagrams()) {` 起首替换为：

```cpp
    int processed = 0;
    while (m_sock && m_sock->hasPendingDatagrams() && processed < kMaxBurst) {
        ++processed;
        QElapsedTimer replyTimer;
        replyTimer.start();

        const QNetworkDatagram dg = m_sock->receiveDatagram();
        if (!m_peerLocked) {
            // 首帧锁定对端。此后只认同一 (IP, port) 的帧。
            m_peerAddr   = dg.senderAddress();
            m_peerPort   = quint16(dg.senderPort());
            m_peerLocked = true;
        } else if (dg.senderAddress() != m_peerAddr
                   || quint16(dg.senderPort()) != m_peerPort) {
            // 异源帧：假 KRC（残留模拟器之类）。丢弃且不回包——回包会让它
            // 误以为掌控链路；真实 KRC 的源固定，不受影响。
            ++m_peerRejected;
            continue;
        }
```

把 `m_peerAddr = dg.senderAddress(); m_peerPort = quint16(dg.senderPort());` 那两行（在 receiveDatagram 之后原有）删除。

- [ ] **Step 5: KRC Delay 运行中保护**

在 `onDatagram()` 的有效帧分支内、`switch (ev.kind) { ... }`（丢包计数）之后、
`if (ev.kind == IpocEvent::Normal || ev.kind == IpocEvent::Gap)` 之前，插入。
注意必须放在 `f.valid` 分支顶层，而非 `step()` 的 if 块内——Dup/Back/First 帧
的 delay 增长同样必须被跟踪。加：

```cpp
            // KRC Delay 运行中保护：SENTYPE 错配、回复迟到/被丢弃都会让 KRC
            // 自己的 Delay 计数增长，而主机侧的丢包计数看不见这些。连续 3 帧
            // 递增（持平不算）即转 Fault。
            if (f.delay > m_lastDelay) {
                ++m_delayRising;
                if (m_delayRising >= 3 && m_ctl.state() == TrackState::Tracking)
                    m_ctl.forceFault(QStringLiteral(
                        "KRC reports rising delay %1 (lost/late replies)")
                            .arg(f.delay));
            } else {
                m_delayRising = 0;
            }
            m_lastDelay = f.delay;
```

- [ ] **Step 6: 写返回值检查**

把：

```cpp
        m_sock->writeDatagram(sen, m_peerAddr, m_peerPort);
```

替换为：

```cpp
        // 发送失败必须计数：KRC 收不到修正时继续跟踪是危险的。连续 5 次失败
        // 即 Fault；一次成功清零连续计数。
        const qint64 sent = m_sock->writeDatagram(sen, m_peerAddr, m_peerPort);
        if (sent < 0) {
            ++m_sendFails;
            if (m_sendFails >= 5 && m_ctl.state() == TrackState::Tracking)
                m_ctl.forceFault(
                    QStringLiteral("send failed %1 times").arg(m_sendFails));
        } else {
            m_sendFails = 0;
        }
```

- [ ] **Step 7: `publishSnapshot` 填充新字段**

在 `s.maxReplyUs = m_maxReplyUs;` 之后加：

```cpp
    s.krcDelay     = m_lastDelay;
    s.peerRejected = m_peerRejected;
    s.sendFails    = m_sendFails;
```

- [ ] **Step 8: Build, run unit tests, loopback regression**

```bash
cmake --build build
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state test_ipoc_tracker test_session_guard; do
  ./build/tests/$t.exe -o result.log,txt || echo "FAIL $t"
  grep "Totals" result.log
done
```

```bash
./build/tools/krc_simulator.exe --host 127.0.0.1 --port 59152 --cycles 700 > /tmp/sim.log 2>&1 &
SIM=$!
./build/tools/loopback_test.exe --seconds 9 > /tmp/lb.log 2>&1
wait $SIM
grep -q "replies=700 missed=0 ipoc_mismatch=0" /tmp/sim.log && grep -q "PASS" /tmp/sim.log
cat /tmp/sim.log
cat /tmp/lb.log
```

Expected: 单元测试全绿；正常环回 `replies=700 missed=0`。

- [ ] **Step 9: Commit**

```bash
git add src/net/RsiWorker.h src/net/RsiWorker.cpp
git commit -m "feat(net): peer lock, burst cap, send-fail fault, rx buffer, KRC delay protection"
```

---

### Task 7: MainWindow 联锁 UI

**Files:**
- Modify: `src/ui/MainWindow.h`
- Modify: `src/ui/MainWindow.cpp`

**Interfaces:**
- Consumes: `SessionGuard::enableChecks`（Task 4）、`StatusSnapshot` 新字段（Task 5）。

- [ ] **Step 1: Modify `MainWindow.h`**

在 `QLabel *m_safetyNote = nullptr;` 之后加：

```cpp
    // 联锁拦截原因（红字）。硬拦截：使能不通过时置红字，无覆盖入口。
    QLabel *m_interlockLabel = nullptr;
```

- [ ] **Step 2: Modify 构造函数 — 添加联锁标签**

`MainWindow.cpp` 构造函数的 `outer->addLayout(bar);` 之后加：

```cpp
    m_interlockLabel = new QLabel(this);
    m_interlockLabel->setStyleSheet("color: #b00; font-weight: bold;");
    m_interlockLabel->setWordWrap(true);
    m_interlockLabel->hide();
    outer->addWidget(m_interlockLabel);
```

- [ ] **Step 3: Modify `onStartListening` — 清除旧拦截**

`onStartListening()` 里 `QMetaObject::invokeMethod(m_worker, "applyConfig", ...);` 之前加：

```cpp
    if (m_interlockLabel)
        m_interlockLabel->hide();
```

- [ ] **Step 4: Modify `onTrackingToggled` — 硬拦截**

把整个 `onTrackingToggled` 替换为：

```cpp
void MainWindow::onTrackingToggled(bool on)
{
    if (!m_worker)
        return;
    if (on) {
        // 联锁：硬拦截无覆盖。不通过就不置勾，红字列出全部原因。
        const StatusSnapshot s = m_state.snapshot();
        const QStringList blocked =
            SessionGuard::enableChecks(m_cfg, s.measuredCycleMs);
        if (!blocked.isEmpty()) {
            m_trackCheck->blockSignals(true);
            m_trackCheck->setChecked(false);
            m_trackCheck->blockSignals(false);
            m_interlockLabel->setText(QStringLiteral("使能被拦截：\n")
                                      + blocked.join(QLatin1Char('\n')));
            m_interlockLabel->show();
            return;
        }
        m_interlockLabel->hide();
    }
    // 必须排队：直连会在通信线程 step() 读状态的同时改写它。
    QMetaObject::invokeMethod(m_worker, "setTracking",
                              Qt::QueuedConnection, Q_ARG(bool, on));
}
```

- [ ] **Step 5: Modify `onRefresh` — 状态栏追加统计**

把：

```cpp
    st += QStringLiteral("   IPOC %1   周期 %2 ms   最大回包 %3 µs"
                         "   丢包 %4")
              .arg(s.ipoc)
              .arg(s.measuredCycleMs, 0, 'f', 1)
              .arg(s.maxReplyUs, 0, 'f', 0)
              .arg(s.missedCount);
```

替换为：

```cpp
    st += QStringLiteral("   IPOC %1   周期 %2 ms   最大回包 %3 µs"
                         "   丢包 %4   KRC丢包 %5   异源 %6   发送失败 %7")
              .arg(s.ipoc)
              .arg(s.measuredCycleMs, 0, 'f', 1)
              .arg(s.maxReplyUs, 0, 'f', 0)
              .arg(s.missedCount)
              .arg(s.krcDelay)
              .arg(s.peerRejected)
              .arg(s.sendFails);
```

- [ ] **Step 6: Build and run unit tests**

```bash
cmake --build build
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state test_ipoc_tracker test_session_guard; do
  ./build/tests/$t.exe -o result.log,txt || echo "FAIL $t"
  grep "Totals" result.log
done
```

Expected: 全绿（`rsi_host` 链接成功即 UI 改动无编译错误）。

- [ ] **Step 7: Commit**

```bash
git add src/ui/MainWindow.h src/ui/MainWindow.cpp
git commit -m "feat(ui): interlock enable tracking; show KRC delay and peer stats"
```

---

### Task 8: krc_simulator 故障注入

**Files:**
- Modify: `tools/krc_simulator/main.cpp`（整文件替换为下面内容）

**Interfaces:**
- Produces: 新命令行开关 `--ipoc-dup/--ipoc-gap/--ipoc-back/--drop/--reorder/--late-ms/--ignore-replies/--send-delay`，并在 `<Rob>` 里发出 `<Delay D="n"/>`。Task 10 的验证脚本依赖这些开关与输出格式。

- [ ] **Step 1: Replace `tools/krc_simulator/main.cpp` 全文**

```cpp
// 假 KRC：按固定周期发 <Rob>、收 <Sen>，验证上位机的实时行为。
// 把收到的 RKorr 累加到自身位姿，模拟 RELATIVE 修正语义。
// 支持故障注入：--ipoc-dup/--ipoc-gap/--ipoc-back/--drop/--reorder/--late-ms
// /--ignore-replies/--send-delay，用于验证主机的异常处理路径。
#include <QCoreApplication>
#include <QCommandLineParser>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QThread>
#include <QUdpSocket>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include "core/Pose.h"
#include "core/RsiCodec.h"

namespace {

QByteArray buildRob(const Pose &p, quint64 ipoc, quint64 delay)
{
    QByteArray s;
    s.reserve(320);
    s += "<Rob Type=\"KUKA\">\n<RIst";
    const char *k[6] = {" X=\"", " Y=\"", " Z=\"",
                        " A=\"", " B=\"", " C=\""};
    const double v[6] = {p.x, p.y, p.z, p.a, p.b, p.c};
    for (int i = 0; i < 6; ++i) {
        s += k[i];
        s += QByteArray::number(v[i], 'f', 4);
        s += '"';
    }
    s += "/>\n<RSol";
    for (int i = 0; i < 6; ++i) {
        s += k[i];
        s += QByteArray::number(v[i], 'f', 4);
        s += '"';
    }
    s += "/>\n<Delay D=\"";
    s += QByteArray::number(delay);
    s += "\"/>\n<IPOC>";
    s += QByteArray::number(ipoc);
    s += "</IPOC>\n</Rob>";
    return s;
}

// 从 <Sen> 中取出 RKorr 与 IPOC。
// 注意不能用 RsiCodec::parseRob —— <Sen> 与 <Rob> 结构不同，这里定向提取。
bool parseSen(const QByteArray &d, Pose *korr, quint64 *ipoc)
{
    const int rk = d.indexOf("<RKorr");
    const int ip = d.indexOf("<IPOC>");
    if (rk < 0 || ip < 0)
        return false;

    const char *keys[6] = {"X=\"", "Y=\"", "Z=\"",
                           "A=\"", "B=\"", "C=\""};
    double *dst[6] = {&korr->x, &korr->y, &korr->z,
                      &korr->a, &korr->b, &korr->c};
    for (int i = 0; i < 6; ++i) {
        const int at = d.indexOf(keys[i], rk);
        if (at < 0)
            return false;
        const int b = at + int(qstrlen(keys[i]));
        const int e = d.indexOf('"', b);
        if (e < 0)
            return false;
        bool ok = false;
        *dst[i] = d.mid(b, e - b).toDouble(&ok);
        if (!ok)
            return false;
    }

    const int b = ip + 6;
    const int e = d.indexOf("</IPOC>", b);
    if (e < 0)
        return false;
    bool ok = false;
    *ipoc = d.mid(b, e - b).trimmed().toULongLong(&ok);
    return ok;
}

// 注入开关：everyN > 0 且第 i 周期触发（i>0 避开首帧，保证对端锁定正常）。
bool active(int everyN, int i)
{
    return everyN > 0 && i > 0 && (i % everyN == 0);
}

} // namespace

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QCommandLineParser p;
    p.addHelpOption();
    QCommandLineOption oHost("host", "host IP", "ip", "192.168.44.1");
    QCommandLineOption oPort("port", "host port", "n", "59152");
    QCommandLineOption oCycle("cycle-ms", "cycle", "ms", "12.0");
    QCommandLineOption oCount("cycles", "cycle count", "n", "500");
    QCommandLineOption oDup("ipoc-dup", "every Nth frame resend previous IPOC", "n", "0");
    QCommandLineOption oGap("ipoc-gap", "every Nth frame jump N IPOC", "n", "0");
    QCommandLineOption oBack("ipoc-back", "every Nth frame send IPOC-1", "n", "0");
    QCommandLineOption oDrop("drop", "every Nth frame send nothing", "n", "0");
    QCommandLineOption oReorder("reorder", "every Nth frame swap order with next", "n", "0");
    QCommandLineOption oLate("late-ms", "every Nth frame delay N ms", "n", "0");
    QCommandLineOption oIgnore("ignore-replies", "drop replies and raise Delay", "");
    QCommandLineOption oDelay("send-delay", "fixed Delay value in every frame", "n", "0");
    p.addOptions({oHost, oPort, oCycle, oCount, oDup, oGap, oBack,
                  oDrop, oReorder, oLate, oIgnore, oDelay});
    p.process(app);

    const QHostAddress host(p.value(oHost));
    const quint16 port  = quint16(p.value(oPort).toUShort());
    const double cycleMs = p.value(oCycle).toDouble();
    const int cycles     = p.value(oCount).toInt();

    const int dupN     = p.value(oDup).toInt();
    const int gapN     = p.value(oGap).toInt();
    const int backN    = p.value(oBack).toInt();
    const int dropN    = p.value(oDrop).toInt();
    const int reorderN = p.value(oReorder).toInt();
    const int lateN    = p.value(oLate).toInt();
    const bool ignore  = p.isSet(oIgnore);
    const quint64 delayBase = p.value(oDelay).toULongLong();

    const bool injected = dupN > 0 || gapN > 0 || backN > 0 || dropN > 0
                          || reorderN > 0 || lateN > 0 || ignore;

    QUdpSocket sock;
    if (!sock.bind(QHostAddress::AnyIPv4, 0)) {
        std::fprintf(stderr, "simulator bind failed: %s\n",
                     qPrintable(sock.errorString()));
        return 2;
    }

    Pose pose{1250.0, 0.0, 1000.0, 0.0, 90.0, 0.0};
    quint64 ipoc = 1000;
    quint64 lastSent = 0;
    quint64 delay    = delayBase;

    int replies = 0, ipocMismatch = 0, missed = 0;
    double maxRttUs = 0.0, sumRttUs = 0.0;

    // 真实 KRC 按固定节拍发帧。若不设节拍而是收到回复就立刻发下一帧，
    // 面对快速主机会全速空转——那测的是吞吐，不是"能否在周期内回复"，
    // 而且主机侧实测出来的周期也不再是 cycleMs。
    QElapsedTimer pace;
    pace.start();

    QByteArray heldRob;          // --reorder 缓冲的上周期帧
    quint64    heldIpoc = 0;
    bool       heldValid = false;

    // 发出的帧按序入队，收到回包弹队首比对 IPOC 回显。UDP 保序，乱序/重复
    // 注入下仍能精确判定主机是否原样回显。--ignore-replies 时不消费，故不入队。
    std::deque<quint64> sentIpocs;

    for (int i = 0; i < cycles; ++i) {
        // 等到本周期的标称发送时刻
        const qint64 dueNs = qint64(double(i) * cycleMs * 1.0e6);
        while (pace.nsecsElapsed() < dueNs) {
            const qint64 remainMs = (dueNs - pace.nsecsElapsed()) / 1000000;
            if (remainMs > 1)
                QThread::msleep(1);
        }

        const bool dup     = active(dupN, i);
        const bool gap     = active(gapN, i);
        const bool back    = active(backN, i);
        const bool drop    = active(dropN, i);
        const bool reorder = active(reorderN, i);
        const bool late    = active(lateN, i);

        // 决定本帧 IPOC：dup 重发上一帧；back 回退；gap 前向跳号。
        quint64 sendIpoc = ipoc;
        if (dup)         sendIpoc = lastSent;
        else if (back)   sendIpoc = (lastSent > 0) ? lastSent - 1 : 0;
        else if (gap)    sendIpoc = ipoc + gapN;

        const QByteArray rob = buildRob(pose, sendIpoc, delay);

        auto sendFrame = [&](const QByteArray &rob2, quint64 frameIpoc) {
            if (late)
                QThread::msleep(lateN);
            sock.writeDatagram(rob2, host, port);
            if (!ignore)
                sentIpocs.push_back(frameIpoc);
        };

        if (reorder) {
            // 本帧缓冲，下周期先发（乱序到达）
            heldRob   = rob;
            heldIpoc  = sendIpoc;
            heldValid = true;
        } else {
            if (heldValid) {
                sendFrame(heldRob, heldIpoc);
                heldValid = false;
            }
            if (!drop)
                sendFrame(rob, sendIpoc);
        }

        // 推进序列：dup 不推进（下帧还发同一个）；其余推进。
        if (!dup) {
            lastSent = sendIpoc;
            ipoc     = sendIpoc + 1;
        }

        QElapsedTimer rtt;
        rtt.start();
        // 等待本周期内的回包。注意不能用 waitForReadyRead 的返回值当作
        // "收到回复"：端口关闭时 Windows 的 ICMP port-unreachable 也会让它
        // 返回 true，那样该周期既不计 replies 也不计 timeouts，
        // cycles == replies + missed 就不再成立。只认解析成功的 <Sen>。
        if (!ignore) {
            const int budgetMs = std::max(1, int(cycleMs));
            bool got = false;
            if (sock.waitForReadyRead(budgetMs)) {
                while (sock.hasPendingDatagrams()) {
                    const QByteArray d = sock.receiveDatagram().data();
                    Pose korr;
                    quint64 echoed = 0;
                    if (parseSen(d, &korr, &echoed)) {
                        ++replies;
                        got = true;
                        if (!sentIpocs.empty()) {
                            if (echoed != sentIpocs.front())
                                ++ipocMismatch;
                            sentIpocs.pop_front();
                        }
                        // RELATIVE：增量累加到当前位姿
                        pose.x += korr.x; pose.y += korr.y; pose.z += korr.z;
                        pose.a = wrap180(pose.a + korr.a);
                        pose.b = wrap180(pose.b + korr.b);
                        pose.c = wrap180(pose.c + korr.c);
                    }
                }
            }
            if (got) {
                const double us = rtt.nsecsElapsed() / 1000.0;
                maxRttUs = std::max(maxRttUs, us);
                sumRttUs += us;
            } else {
                ++missed;
            }
        } else {
            // 模拟 SENTYPE 错配：KRC 静默丢弃每一帧回包，主机毫无察觉，
            // 只有 KRC 自己的 Delay 计数在涨。这里递增 delay，触发主机
            // 的"KRC Delay 连续 3 帧递增 → Fault"运行中保护。
            ++missed;
            delay += 1;
        }
    }

    std::printf("cycles=%d replies=%d missed=%d ipoc_mismatch=%d delay=%llu\n",
                cycles, replies, missed, ipocMismatch,
                static_cast<unsigned long long>(delay));
    std::printf("rtt_avg_us=%.1f rtt_max_us=%.1f\n",
                replies ? sumRttUs / replies : 0.0, maxRttUs);
    std::printf("final_pose X=%.3f Y=%.3f Z=%.3f A=%.3f B=%.3f C=%.3f\n",
                pose.x, pose.y, pose.z, pose.a, pose.b, pose.c);

    // 主机必须始终原样回显 IPOC（即使对异常帧回零增量）。无注入时还要求
    // 每帧都回包；有注入时丢包是预期行为，只查回显正确。
    const bool pass = ipocMismatch == 0 && (injected || replies == cycles);
    std::printf("%s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
```

- [ ] **Step 2: Build and smoke-test each switch**

```bash
cmake --build build
for sw in "--ipoc-dup 50" "--ipoc-gap 50" "--ipoc-back 50" "--drop 50" "--reorder 50" "--late-ms 5" "--ignore-replies"; do
  echo "== $sw =="
  ./build/tools/krc_simulator.exe --host 127.0.0.1 --port 59152 --cycles 200 $sw > /tmp/smoke.log 2>&1
  cat /tmp/smoke.log
done
```

Expected: 每个开关各自运行不崩溃，打印统计与 `PASS` 或 `FAIL`（有注入时 replies>0 即 PASS）。注意 smoke 时没有主机在监听，`missed` 会很大，无妨。

- [ ] **Step 3: Commit**

```bash
git add tools/krc_simulator/main.cpp
git commit -m "feat(tools): fault injection switches in krc_simulator"
```

---

### Task 9: pcap 回放工具

**Files:**
- Create: `tools/pcap_replay.py`

**Interfaces:**
- Produces: `python tools/pcap_replay.py <file.pcap> --host <ip> --port <n> [--speed S] [--ipoc-shift N] [--drop N] [--loop]`。非闭环回放 `<Rob>` 帧。Task 10 的验证矩阵引用它。

- [ ] **Step 1: Write `tools/pcap_replay.py`**

```python
#!/usr/bin/env python3
"""重放 pcap 中的 <Rob> 帧到指定 IP:端口，对 rsi_host 做真实抓包序列验证。

非闭环：不回包。支持 --speed 加速、--ipoc-shift 改写 IPOC、--drop 按比例
丢帧、--loop 循环回放（每圈 IPOC 整体上移避免重复）。

用法:
  python pcap_replay.py capture.pcap --host 127.0.0.1 --port 59152
"""
import argparse
import socket
import struct
import sys
import time

ETH_P_IP = 0x0800
ETH_P_8021Q = 0x8100
UDP_PROTO = 17


def parse_pcap(path, target_port):
    """返回 [(relative_ts_sec: float, payload: bytes), ...]，payload 为 UDP 载荷。"""
    frames = []
    with open(path, "rb") as f:
        gh = f.read(24)
        if len(gh) < 24:
            raise ValueError("pcap 全局头不足 24 字节")
        raw = gh[:4]
        magic_le = struct.unpack("<I", raw)[0]
        # 小端文件字节为 D4 C3 B2 A1 → <I 读得 0xA1B2C3D4；大端文件字节为
        # A1 B2 C3 D4 → <I 读得 0xD4C3B2A1。据此推断真实字节序。
        if magic_le in (0xA1B2C3D4, 0xA1B23C4D):
            endian = "<"
        else:
            endian = ">"
        magic = struct.unpack(endian + "I", raw)[0]
        if magic not in (0xA1B2C3D4, 0xA1B23C4D):
            raise ValueError("未知 pcap magic %#x" % magic)
        linktype = struct.unpack(endian + "I", gh[20:24])[0]
        if linktype != 1:
            raise ValueError("仅支持 Ethernet(1) 链路层，得到 %d" % linktype)

        t0 = None
        while True:
            rec = f.read(16)
            if not rec:
                break
            ts_sec, ts_usec, incl_len, _ = struct.unpack(endian + "IIII", rec)
            data = f.read(incl_len)
            if len(data) < incl_len:
                break
            t = ts_sec + ts_usec / 1e6
            if t0 is None:
                t0 = t
            payload = extract_udp_payload(data, target_port)
            if payload is not None:
                frames.append((t - t0, payload))
    return frames


def extract_udp_payload(eth, target_port):
    """从以太网帧提取 UDP 载荷；目标端口不匹配或非 UDP 返回 None。"""
    if len(eth) < 14:
        return None
    ethertype = struct.unpack(">H", eth[12:14])[0]
    off = 14
    if ethertype == ETH_P_8021Q:
        if len(eth) < 18:
            return None
        ethertype = struct.unpack(">H", eth[16:18])[0]
        off = 18
    if ethertype != ETH_P_IP:
        return None
    ip = eth[off:]
    if len(ip) < 20:
        return None
    ihl = (ip[0] & 0x0F) * 4
    if ip[9] != UDP_PROTO:
        return None
    udp = ip[ihl:]
    if len(udp) < 8:
        return None
    _, dport, _, _ = struct.unpack(">HHHH", udp[:8])
    if dport != target_port:
        return None
    return udp[8:]


def rewrite_ipoc(payload, shift):
    """把 <IPOC>n</IPOC> 改写为 <IPOC>n+shift</IPOC>；不改写返回原样。"""
    if not shift:
        return payload
    head, sep, tail = payload.partition(b"<IPOC>")
    if not sep:
        return payload
    num, _, rest = tail.partition(b"</IPOC>")
    try:
        v = int(num) + shift
    except ValueError:
        return payload
    return head + sep + str(v).encode() + b"</IPOC>" + rest


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pcap", help="输入 pcap 文件")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=59152)
    ap.add_argument("--speed", type=float, default=1.0, help="回放加速倍率")
    ap.add_argument("--ipoc-shift", type=int, default=0, help="改写 IPOC：加 shift")
    ap.add_argument("--drop", type=int, default=0, help="每 N 帧丢 1 帧")
    ap.add_argument("--loop", action="store_true", help="循环回放")
    args = ap.parse_args()

    frames = parse_pcap(args.pcap, args.port)
    if not frames:
        sys.exit("pcap 中没有发往端口 %d 的 UDP 帧: %s" % (args.port, args.pcap))

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    sent = 0
    loop_shift = 0
    try:
        while True:
            for k, (t, payload) in enumerate(frames):
                if args.drop and k % args.drop == 0:
                    continue
                payload = rewrite_ipoc(payload, args.ipoc_shift + loop_shift)
                sock.sendto(payload, (args.host, args.port))
                sent += 1
                if t > 0:
                    time.sleep(t / args.speed)
            if not args.loop:
                break
            loop_shift += 1000   # 每圈 IPOC 整体上移，避免圈间重复
    except KeyboardInterrupt:
        pass
    print("sent %d frames from %s" % (sent, args.pcap))


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Syntax-check**

```bash
python -m py_compile tools/pcap_replay.py
```

Expected: 无输出（编译通过）。

- [ ] **Step 3: Commit**

```bash
git add tools/pcap_replay.py
git commit -m "feat(tools): add pcap replay tool"
```

---

### Task 10: 端到端验证脚本

**Files:**
- Create: `tools/verify_robustness.sh`

**Interfaces:**
- Consumes: Task 2/3/6 的主机行为、Task 8 的 simulator 开关。
- Produces: `bash tools/verify_robustness.sh` —— 依次跑正常环回与故障注入场景，断言主机行为，全部通过返回 0。

- [ ] **Step 1: Write `tools/verify_robustness.sh`**

```bash
#!/usr/bin/env bash
# 通信健壮性端到端验证：用 krc_simulator 故障注入驱动 loopback_test。
# 用法: bash tools/verify_robustness.sh
set -u
cd "$(dirname "$0")/.."

BUILD=build
HOST=127.0.0.1
PORT=59152
CYCLES=700
SECS=10

pass=0
fail=0

# run <tag> <lb-extra> <sim args...>：后台起 simulator，前台跑 loopback_test。
# lb-extra 是传给 loopback_test 的额外参数（如 "--track 1"）。
run() {
    local tag=$1; shift
    local lb=$1; shift
    "$BUILD/tools/krc_simulator.exe" --host "$HOST" --port "$PORT" \
        --cycles "$CYCLES" "$@" > "/tmp/sim_$tag.log" 2>&1 &
    local sim=$!
    # shellcheck disable=SC2086
    "$BUILD/tools/loopback_test.exe" --seconds "$SECS" $lb > "/tmp/lb_$tag.log" 2>&1
    local rc=$?
    wait "$sim" || rc=1
    return "$rc"
}

check() {   # check <描述> <tag> <grep 表达式>
    local desc=$1 tag=$2 pat=$3
    if grep -q "$pat" "/tmp/sim_$tag.log"; then
        echo "PASS  $desc"
        pass=$((pass + 1))
    else
        echo "FAIL  $desc  (grep '$pat' 于 /tmp/sim_$tag.log)"
        fail=$((fail + 1))
    fi
}

# 1. 正常环回：700/700 应答、零丢包、零 IPOC 不匹配
run normal ""
check "正常环回 700/700 零丢包" normal "replies=$CYCLES missed=0 ipoc_mismatch=0"

# 2. --ipoc-dup：重复帧仍被原样回显，且计入主机丢包
run dup "" --ipoc-dup 50
check "重复帧回显正确" dup "ipoc_mismatch=0"
check "重复帧计入丢包" dup "missed=[1-9][0-9]*"

# 3. --drop 50：主机看到丢包且回显仍正确
run drop "" --drop 50
check "丢包计入" drop "missed=[1-9][0-9]*"
check "丢包后回显正确" drop "ipoc_mismatch=0"

# 4. --ipoc-gap 50：前向跳号计缺失周期
run gap "" --ipoc-gap 50
check "前向跳号计入丢包" gap "missed=[1-9][0-9]*"
check "跳号帧回显正确" gap "ipoc_mismatch=0"

# 5. --ignore-replies：模拟 SENTYPE 错配。必须使能跟踪（--track 1）——
#    KRC Delay 增长是运行中保护，只在 Tracking 状态触发 Fault。
run ignore "--track 1" --ignore-replies
if grep -q "state=Fault" "/tmp/lb_ignore.log"; then
    echo "PASS  主机 KRC Delay 增长 → Fault"
    pass=$((pass + 1))
else
    echo "FAIL  主机未因 KRC Delay 增长转 Fault（见 /tmp/lb_ignore.log）"
    fail=$((fail + 1))
fi

echo "----"
echo "PASS=$pass FAIL=$fail"
[ "$fail" -eq 0 ]
```

- [ ] **Step 2: Run the script**

```bash
bash tools/verify_robustness.sh
```

Expected: 全部 `PASS`，`FAIL=0`。若 `--ignore-replies` 场景未转 Fault，检查
`/tmp/lb_ignore.log` 最终 `state=`——该场景需主机在 Tracking 状态（脚本已用
`--track 1`），KRC Delay 保护只在 Tracking 下触发；若 `state=` 显示 Idle 而
非 Fault，查主机的 Delay 保护是否被正确接线。

- [ ] **Step 3: Commit**

```bash
git add tools/verify_robustness.sh
git commit -m "feat(tools): add robustness verification script"
```

---

### Task 11: 文档更新

**Files:**
- Modify: `docs/real-machine-deployment.md`
- Create: `docs/verification-matrix.md`

- [ ] **Step 1: 更新 `docs/real-machine-deployment.md` §7 已知缺口**

把 §7 整个小节替换为：

```markdown
## 7. 联锁与已知缺口

上位机现在带启动联锁（硬拦截，无覆盖）：使能跟踪前自动校验，不通过则状态栏
红字列出原因并拒绝勾选。联锁检查项：

- `cycle_ms` > 0
- `session_gap_ms` > `krc_timeout_cycles × cycle_ms`（配置里 `krc_timeout_cycles`
  默认 100，即 100 个 IPO 周期）
- 主机 `accum_limit_pos_mm/rot_deg` < `krc_poscorr_limit_pos_mm/rot_deg`
  （默认 25/25，即 KRC POSCORR 累积限值）。**当前 `config/rsi_config.json`
  的 1000/100 会被主动拦下——首次联机前必须按应用需要调回安全值。**
- `sen_type` 非空
- 实测周期与 `cycle_ms` 偏差 ≤ 10%

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
```

- [ ] **Step 2: 新建 `docs/verification-matrix.md`**

```markdown
# 通信健壮性验证矩阵

主机侧行为验证工具：
- `tools/verify_robustness.sh` —— krc_simulator 故障注入 + loopback_test 端到端
- `tools/pcap_replay.py` —— 真实抓包回放（非闭环）

## 主机侧故障注入矩阵（本机可跑）

| 场景 | 工具/开关 | 预期 | 判定 |
|---|---|---|---|
| 正常环回 | `verify_robustness.sh` | 700/700 应答、零丢包 | `replies=700 missed=0` |
| 重复帧 | `--ipoc-dup 50` | 回零增量、不推进 lastGood、丢包+1、回显正确 | `missed>0` 且 `ipoc_mismatch=0` |
| 回退帧 | `--ipoc-back 50` | 同上 | 同上 |
| 前向跳号 | `--ipoc-gap 50` | 正常修正、丢包 += 缺口、回显正确 | `missed>0` 且 `ipoc_mismatch=0` |
| 丢包 | `--drop 50` | 主机计丢包 | `missed>0` |
| 乱序 | `--reorder 50` | 按序回显不崩溃 | 无 `ipoc_mismatch` 暴涨 |
| 迟到 | `--late-ms 5` | 回包延迟可测 | simulator `rtt_max_us` 可见 |
| 错误 SENTYPE | `--ignore-replies` | KRC Delay 增长 → 主机 Fault | `state=Fault` |
| 断网 | 运行中杀 simulator / 停监听 | 看门狗置未连接 | 状态栏回 `◐ 监听中` |
| 长时间运行 | `--cycles 10000` | 无泄漏、丢包不漂移 | 长时间窗口 missed 稳定 |

## 真机 T1 验证（待执行）

分别完成 **12 ms** 与 **4 ms** 周期真机 T1 联机后，再评估自动模式。
每档按 [real-machine-deployment.md](real-machine-deployment.md) §5 执行，
并额外核对：
- 实测周期回填 `cycle_ms`（联锁会校验偏差 ≤ 10%）
- KRC `<Delay>` 全程为 0（非 0 即链路异常，联锁会 Fault）
- 五层限值梯度仍单调
```

- [ ] **Step 3: Commit**

```bash
git add docs/real-machine-deployment.md docs/verification-matrix.md
git commit -m "docs: update gaps, interlock and verification matrix"
```

---

## Self-Review 记录

- **Spec 覆盖**：IpocTracker（Task 1-2）、Delay 解析（Task 3）、对端锁定/积压上限/写返回/接收缓冲/KRC Delay 保护（Task 6）、静态+动态联锁（Task 4/7）、新配置字段（Task 4）、SharedState 字段（Task 5）、simulator 故障注入 + pcap 回放 + 验证矩阵（Task 8-11）、BASE/TOOL 仅文档（Task 11）。无缺口。
- **占位符**：无 TBD/TODO；所有代码块完整。
- **类型一致**：`IpocEvent::Kind`、`classify`、`lastGood`、`haveFirst`、`reset`、`SessionGuard::staticChecks/enableChecks`、`forceFault`、`StatusSnapshot.krcDelay/peerRejected/sendFails`、simulator 开关名在任务间一致。
- **验证脚本注意点**：`--ignore-replies` 场景需主机在 Tracking 状态才能触发 Fault，脚本内已注明用 `--track 1` 单独起 loopback_test（Step 2 有修正路径）。


