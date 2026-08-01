# Review 反馈加固 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 落地 review 反馈：旋转数学缓解（P1-1）、目标平滑器（P1-2）、UI 8 条渐进增强。

**Architecture:** 控制层改动集中在 `PoseController`（commandedSum 姿态兜底 + 目标一阶低通），`AppConfig` 加一个配置字段；UI 在现有 `MainWindow`/`ErrorChart` 上渐进增强（状态卡、两阶段使能、软停止横幅、小步进、参数高级页、双图、三列读数、诊断字段）。不引入 Eigen。

**Tech Stack:** C++17 / Qt 6.5.3 (mingw_64) / CMake / Ninja / Qt Test。在 `feature/communication-robustness` 分支继续（PR #1 迭代）。

## Global Constraints

- **不引入 Eigen**。旋转误差维持逐轴欧拉角（P1-1 只做缓解：commandedSum 兜底 + 文案 + 奇异警告 + 文档）。
- **实时路径（RsiWorker 通信线程）禁堆分配/日志/IO**。周期 P99 统计用定容 `std::array` + `std::nth_element`（拷贝进临时定容数组），无分配。
- **`testCfg()` 必须设 `targetSmoothingMs = 0.0`**——否则现有精确算术断言（`kp×误差`）被平滑破坏。Task 3 改 `testCfg` 后，先前所有依赖精确增量的用例自动保持。
- **KRC 三文件只读**（`PoseTrack.rsix`/`.src`/`_ethernet.xml`）。
- **状态卡颜色分级**：灰=未监听 / 蓝=监听中 / 绿=已连接 / 黄=抖动·丢包·延迟高 / 红=Fault·超时·超限。
- **两阶段使能状态机**：Idle+未连接=禁用「准备跟踪」；Idle+已连接=可用；Tracking=「已使能跟踪」禁用；Fault=「归零并复位」。
- **Tracking 运行中参数锁定**：控制参数对话框在 Tracking 时禁用。
- UI 无自动化测试：验证 = 构建通过 + 脚本驱动 GUI（沿用 `WM_KEYDOWN` 法）。
- 构建/测试环境同上一轮：`export MINGW=/d/Software/QT/content/Tools/mingw1120_64/bin; export NINJA=/d/Software/QT/content/Tools/Ninja; export QTBIN=/d/Software/QT/content/6.5.3/mingw_64/bin; export PATH="$MINGW:$NINJA:$QTBIN:$PATH"`；测试用文件日志器 `-o .superpowers/sdd/<name>.log,txt`。

---

### Task 1: Part A — 第 2 层姿态监控 commandedSum 兜底

**Files:**
- Modify: `src/core/PoseController.cpp`
- Modify: `tests/test_pose_controller.cpp`

**Interfaces:**
- Consumes: `PoseController::commandedSum()`（已有，返回 `m_accum`，不折返累计命令增量）。
- Produces: 第 2 层姿态监控从"仅 RIst 锚点位移逐轴最大"改为"RIst 位移 与 commandedSum 取 max"；`faultReason` 文案改「max per-axis accumulated rotation」。Task 6（状态卡 Fault 显示）依赖文案。

- [ ] **Step 1: Write the failing test**

`tests/test_pose_controller.cpp` 的 `private slots:` 内追加：

```cpp
    void rotatedOverLimit_firesViaCommandedSumEvenIfRistWraps()
    {
        // RIst 姿态角折返：机器人沿 A 轴转整圈后实际位姿报回 0 附近，
        // 主机从 RIst 无法得知真实累计角度。commandedSum（不折返）必须兜底。
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitRotDeg = 200.0;   // 高于单圈 180°，让 RIst 折返不触发
        c.vmaxRotDegS      = 50.0;    // 12ms → 0.6°/cycle，200° 需 ~334 周期
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 220, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 500 && pc.state() == TrackState::Tracking; ++i) {
            actual.a += pc.step(actual).a;
            actual.a  = wrap180(actual.a);   // 模拟 RIst 折返
        }
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(pc.faultReason().contains("rotation"));
    }
```

- [ ] **Step 2: Run test to verify it fails**

```bash
cmake --build build
./build/tests/test_pose_controller.exe -o .superpowers/sdd/rf-t1.log,txt; cat .superpowers/sdd/rf-t1.log
```

Expected: 新用例 FAIL（当前 `rotMax` 只看 `m_displacement`，RIst 折返下 `|wrap180(220)|=40 < 200`，永不触发）。

- [ ] **Step 3: Modify `PoseController.cpp`**

`step()` 里把姿态越限计算：

```cpp
    const double rotMax = std::max({std::fabs(m_displacement.a),
                                    std::fabs(m_displacement.b),
                                    std::fabs(m_displacement.c)});
```

替换为：

```cpp
    // 姿态监控取两源保守值：
    //  (1) RIst 锚点位移逐轴最大 —— RIst 姿态角本身可能折返（±180°），主机
    //      无法得知真实累计圈数，仅靠它会在多圈旋转时漏掉；
    //  (2) 主机未折返累计命令增量（commandedSum）逐轴最大 —— 不折返，反映
    //      "主机以为发出去了多少修正"。
    // 取二者较大。高估是安全方向：宁可因丢包导致的高估提前 Fault，也不漏报。
    const double rotDisp = std::max({std::fabs(m_displacement.a),
                                     std::fabs(m_displacement.b),
                                     std::fabs(m_displacement.c)});
    const double rotCmd  = std::max({std::fabs(m_accum.a),
                                     std::fabs(m_accum.b),
                                     std::fabs(m_accum.c)});
    const double rotMax  = std::max(rotDisp, rotCmd);
```

`faultReason` 姿态文案改为：

```cpp
        m_faultReason = QStringLiteral(
            "max per-axis accumulated rotation %1 deg exceeds limit %2 deg")
```

- [ ] **Step 4: Run test to verify it passes**

```bash
cmake --build build
./build/tests/test_pose_controller.exe -o .superpowers/sdd/rf-t1.log,txt; cat .superpowers/sdd/rf-t1.log
```

Expected: 新用例 PASS + 全部现有用例 PASS。

- [ ] **Step 5: Commit**

```bash
git add src/core/PoseController.cpp tests/test_pose_controller.cpp
git commit -m "fix(core): layer-2 rotation monitor falls back to commanded sum"
```

---

### Task 2: Part A — UI 奇异区警告

**Files:**
- Modify: `src/ui/MainWindow.h`
- Modify: `src/ui/MainWindow.cpp`

**Interfaces:**
- Consumes: 现有目标面板 spinbox 数组 `m_targetSpin[4]`（B 角）。
- Produces: `m_singularWarnLabel`（黄色 QLabel），B 角 |值|≥85° 时显示。

- [ ] **Step 1: Modify `MainWindow.h`**

在 `m_interlockLabel` 声明之后加：

```cpp
    // 欧拉奇异区警告（B≈±90° 时姿态控制退化）。黄色提示，不拦截。
    QLabel *m_singularWarnLabel = nullptr;
```

- [ ] **Step 2: Modify 构造函数**

`buildTargetPanel()` 返回后（`outer->addWidget(buildTargetPanel())` 之后加一行）：

在构造函数 `left->addWidget(buildTargetPanel());` 之后加：

```cpp
    left->addWidget(buildSingularWarn());
```

新增私有方法（`MainWindow.h` 声明 + `MainWindow.cpp` 实现）：

```cpp
QWidget *MainWindow::buildSingularWarn()
{
    m_singularWarnLabel = new QLabel(this);
    m_singularWarnLabel->setStyleSheet(
        "color: #a06000; font-weight: bold;");
    m_singularWarnLabel->setWordWrap(true);
    m_singularWarnLabel->hide();
    return m_singularWarnLabel;
}
```

（`MainWindow.h` 的 private 区加 `QWidget *buildSingularWarn();`。）

- [ ] **Step 3: 在 `onTargetEdited()` 末尾检查 B 角**

在 `onTargetEdited()` 的 `QMetaObject::invokeMethod(...applyTarget...)` 之后加：

```cpp
    // 欧拉奇异区：B≈±90° 时 A/C 耦合，姿态误差计算退化。仅警告，不拦截。
    if (std::fabs(m_targetSpin[4]->value()) >= 85.0) {
        m_singularWarnLabel->setText(
            QStringLiteral("⚠ 接近欧拉奇异区 (B≈±90°)，姿态控制可能退化"));
        m_singularWarnLabel->show();
    } else {
        m_singularWarnLabel->hide();
    }
```

（`MainWindow.cpp` 需 `#include <cmath>`，检查是否已有。）

- [ ] **Step 4: Build**

```bash
cmake --build build
```

Expected: `rsi_host` 链接成功。单测回归：

```bash
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state test_ipoc_tracker test_session_guard; do
  ./build/tests/$t.exe -o .superpowers/sdd/rf-t2-$t.log,txt
done
```

- [ ] **Step 5: Commit**

```bash
git add src/ui/MainWindow.h src/ui/MainWindow.cpp
git commit -m "feat(ui): warn near Euler singularity on target input"
```

---

### Task 3: Part B — 目标一阶低通平滑器

**Files:**
- Modify: `src/core/AppConfig.h`
- Modify: `src/core/AppConfig.cpp`
- Modify: `config/rsi_config.json`
- Modify: `src/core/PoseController.h`
- Modify: `src/core/PoseController.cpp`
- Modify: `tests/test_pose_controller.cpp`

**Interfaces:**
- Consumes: 新字段 `targetSmoothingMs`。
- Produces: `PoseController` 内部平滑目标 `m_smoothTarget` + 系数 `m_alpha`（≤0 时 1=直通）。`target()` 仍返回原始目标；误差改用平滑目标。Task 4+ 不依赖。

- [ ] **Step 1: 加配置字段**

`AppConfig.h`，在 `double sessionGapMs = 2000.0;` 之后加：

```cpp
    double  targetSmoothingMs   = 50.0;    // 目标一阶低通时间常数 ms（≤0 禁用）
```

`AppConfig.cpp`，`readInt(rsi, "watchdog_miss_limit", ...)` 之后加：

```cpp
    readDouble(rsi, "target_smoothing_ms", &out->targetSmoothingMs);
```

`config/rsi_config.json` 的 `rsi` 对象加 `"target_smoothing_ms": 50.0`。

- [ ] **Step 2: 改 `testCfg()` 保持现有算术断言**

`tests/test_pose_controller.cpp` 的 `testCfg()` 里，在 `c.accumLimitRotDeg = 15.0;` 之后加：

```cpp
    c.targetSmoothingMs    = 0.0;    // 保持增量 = kp×误差 的精确算术断言
```

- [ ] **Step 3: Write the failing smoothing tests**

`private slots:` 内追加：

```cpp
    void smoothing_progressivelyApproachesStepTarget()
    {
        // 放开限幅让增量 = kp×误差；无平滑时目标阶跃 100 第一周期误差=100
        // → 增量=50。平滑后平滑目标第一步 = 100×α，α=12/(12+50)=0.1935
        // → 误差≈19.35 → 增量≈9.68，显著削平。
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        c.kpPos             = 0.5;
        c.vmaxPosMmS        = 1000000.0;   // 放开限幅
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QVERIFY(d1.x < 20.0);
        QVERIFY(qAbs(d1.x - 9.68) < 0.5);
    }

    void smoothing_tauZero_isPassthrough()
    {
        PoseController pc;
        AppConfig c = testCfg();            // targetSmoothingMs=0
        c.kpPos      = 0.5;
        c.vmaxPosMmS = 1000000.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d1 = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QVERIFY(qAbs(d1.x - 50.0) < 1e-9);   // α=1 直通
    }

    void resetToActual_syncsSmoothTarget()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        c.kpPos             = 0.5;
        c.vmaxPosMmS        = 1000000.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        pc.step(Pose{0, 0, 0, 0, 0, 0});     // 平滑目标开始逼近（≈19.35）
        pc.resetToActual(Pose{3, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        // 平滑目标必须同步为 actual(3)，否则从 19.35 向 3 逼近 → 假误差 → 非零增量
        const Pose d = pc.step(Pose{3, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);
    }

    void smoothing_doesNotChangeSteadyState()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.targetSmoothingMs = 50.0;
        pc.configure(c);
        pc.beginSession(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{5, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5000; ++i) {
            const Pose d = pc.step(actual);
            actual.x += d.x;
        }
        QVERIFY(qAbs(pc.target().x - actual.x) < 1e-3);
        QVERIFY(qAbs(pc.accumulated().x - 5.0) < 1e-3);
    }
```

- [ ] **Step 4: Run test to verify it fails**

```bash
cmake --build build
./build/tests/test_pose_controller.exe -o .superpowers/sdd/rf-t3.log,txt; cat .superpowers/sdd/rf-t3.log
```

Expected: 4 个新用例 FAIL（平滑未实现，第一周期增量 = 50 而非 9.68；`resetToActual_syncsSmoothTarget` 因无平滑目标语义也偏离）。

- [ ] **Step 5: Modify `PoseController.h`**

在私有区 `Pose m_anchor;` 之前加：

```cpp
    // 目标一阶低通（仅 Tracking 生效）。m_smoothTarget 每周期向 m_target 逼近，
    // 系数 m_alpha = cycleS/(cycleS+tauS)；tauS≤0 时 m_alpha=1 直通。
    Pose    m_smoothTarget;
    double  m_alpha = 1.0;
```

- [ ] **Step 6: Modify `PoseController.cpp`**

`configure()` 里、`m_stepLimitRot` 计算之后加：

```cpp
    // 平滑系数：仅当时间常数 > 0 才启用低通，否则直通（保持旧行为）。
    const double tauS = cfg.targetSmoothingMs > 0.0
                            ? cfg.targetSmoothingMs / 1000.0
                            : 0.0;
    m_alpha = (tauS <= 0.0 || cycleS <= 0.0) ? 1.0 : cycleS / (cycleS + tauS);
```

`resetToActual()` 里 `m_faultReason.clear();` 之后加：

```cpp
    m_smoothTarget = actual;   // 归零/会话开始同步平滑目标，避免假误差
```

`beginSession()` 内部已调用 `resetToActual`，无需重复。

`step()` 里误差计算：

```cpp
    // 误差：位置直接相减，姿态取最短角路径
    const Pose err = poseSub(m_target, actual);
```

替换为：

```cpp
    // 误差源：默认原始目标；平滑启用时用低通后的平滑目标（每周期指数逼近）。
    // m_smoothTarget 只在 resetToActual/beginSession 同步到 actual，此处每周期
    // 累加，绝不直接赋值——否则会丢历史。τ=0 时 m_alpha=1，一步到位等价无平滑。
    Pose errSrc = m_target;
    if (m_alpha < 1.0) {
        m_smoothTarget.x += m_alpha * (m_target.x - m_smoothTarget.x);
        m_smoothTarget.y += m_alpha * (m_target.y - m_smoothTarget.y);
        m_smoothTarget.z += m_alpha * (m_target.z - m_smoothTarget.z);
        m_smoothTarget.a += m_alpha * (m_target.a - m_smoothTarget.a);
        m_smoothTarget.b += m_alpha * (m_target.b - m_smoothTarget.b);
        m_smoothTarget.c += m_alpha * (m_target.c - m_smoothTarget.c);
        errSrc = m_smoothTarget;
    }
    // 误差：位置直接相减，姿态取最短角路径
    const Pose err = poseSub(errSrc, actual);
```

- [ ] **Step 7: Run test to verify it passes**

```bash
cmake --build build
./build/tests/test_pose_controller.exe -o .superpowers/sdd/rf-t3.log,txt; cat .superpowers/sdd/rf-t3.log
./build/tests/test_app_config.exe -o .superpowers/sdd/rf-t3-app.log,txt; cat .superpowers/sdd/rf-t3-app.log
```

Expected: 全部 PASS（含 4 个新平滑用例 + 现有算术用例因 `targetSmoothingMs=0` 保持）。

- [ ] **Step 8: Commit**

```bash
git add src/core/AppConfig.h src/core/AppConfig.cpp config/rsi_config.json src/core/PoseController.h src/core/PoseController.cpp tests/test_pose_controller.cpp
git commit -m "feat(core): first-order low-pass smoothing on control target"
```

---

### Task 4: SharedState 诊断字段 + RsiWorker 填充

**Files:**
- Modify: `src/net/SharedState.h`
- Modify: `src/net/RsiWorker.h`
- Modify: `src/net/RsiWorker.cpp`

**Interfaces:**
- Consumes: `PoseController::commandedSum`（无直接依赖，本任务只用增量）。
- Produces: `StatusSnapshot` 新增 `peerIp4`/`peerPort`/`lifetimeLost`/`lastDelta`/`cycleMeanMs`/`cycleMaxMs`/`cycleP99Ms`。Task 5（状态卡）显示它们。

- [ ] **Step 1: Modify `SharedState.h`**

`StatusSnapshot` 在 `bool connected = false;` 之后加：

```cpp
    quint32 peerIp4      = 0;    // 对端 IPv4（0=未锁定）
    quint16 peerPort     = 0;    // 对端端口（0=未锁定）
    quint64 lifetimeLost = 0;    // 累计丢包（区别于连续 missedCount）
    Pose    lastDelta;           // 最近一帧 RKorr 增量
    double  cycleMeanMs  = 0.0;  // 周期均值（最近 256 样本窗口）
    double  cycleMaxMs   = 0.0;  // 周期最大
    double  cycleP99Ms   = 0.0;  // 周期 P99
```

- [ ] **Step 2: Modify `RsiWorker.h`**

`#include <array>` 加到 include 区。私有成员（`double m_measuredCycleMs = 0.0;` 之后）加：

```cpp
    // 诊断：累计丢包（会话内只增不减）、最近增量、周期直方（定容，实时无分配）
    quint64 m_lifetimeLost = 0;
    Pose    m_lastDelta;
    static constexpr int kCycleHist = 256;
    std::array<double, kCycleHist> m_cycleHist{};
    int m_cycleHead  = 0;
    int m_cycleCount = 0;
```

- [ ] **Step 3: Modify `RsiWorker.cpp` — start() 重置**

`start()` 的 `m_measuredCycleMs = 0.0;` 之后加：

```cpp
    m_lifetimeLost = 0;
    m_lastDelta    = Pose{};
    m_cycleHead    = 0;
    m_cycleCount   = 0;
```

- [ ] **Step 4: Modify `RsiWorker.cpp` — 丢包计数同步 lifetimeLost**

`onDatagram()` 的 `else { ++m_missed; }`（invalid 分支）改为：

```cpp
        } else {
            ++m_missed;
            ++m_lifetimeLost;
        }
```

`switch (ev.kind)` 里：

```cpp
            case IpocEvent::Gap:
                m_missed += int(ev.gapCount);
                break;
            case IpocEvent::Duplicate:
            case IpocEvent::Backward:
                ++m_missed;
                break;
```

改为：

```cpp
            case IpocEvent::Gap:
                m_missed += int(ev.gapCount);
                m_lifetimeLost += quint64(ev.gapCount);
                break;
            case IpocEvent::Duplicate:
            case IpocEvent::Backward:
                ++m_missed;
                ++m_lifetimeLost;
                break;
```

- [ ] **Step 5: Modify `RsiWorker.cpp` — 周期直方 + lastDelta**

有效帧周期测量处（`if (m_cycleTimerValid) { m_measuredCycleMs = ... }`）改为：

```cpp
            if (m_cycleTimerValid) {
                m_measuredCycleMs = m_cycleTimer.nsecsElapsed() / 1.0e6;
                m_cycleHist[m_cycleHead] = m_measuredCycleMs;
                m_cycleHead = (m_cycleHead + 1) % kCycleHist;
                if (m_cycleCount < kCycleHist)
                    ++m_cycleCount;
            }
```

在 `delta = m_ctl.step(f.rist);`（或异常帧处）之后、`buildSen` 之前加：

```cpp
        m_lastDelta = delta;
```

（放在 `const QByteArray sen = ...` 之前，`if (f.valid) {...} else {...}` 之后。）

- [ ] **Step 6: Modify `RsiWorker.cpp` — publishSnapshot 填字段 + 周期统计**

`publishSnapshot()` 里 `s.sendFails = m_sendFails;` 之后加：

```cpp
    s.peerIp4      = m_peerLocked ? m_peerAddr.toIPv4Address() : 0;
    s.peerPort     = m_peerPort;
    s.lifetimeLost = m_lifetimeLost;
    s.lastDelta    = m_lastDelta;
    if (m_cycleCount > 0) {
        // 拷贝到定容临时数组：nth_element 原地改，不能碰历史。无堆分配。
        const int n = m_cycleCount;
        std::array<double, kCycleHist> h{};
        double sum = 0.0, mx = 0.0;
        for (int i = 0; i < n; ++i) {
            const double v =
                m_cycleHist[(m_cycleHead - n + i + kCycleHist) % kCycleHist];
            h[i] = v;
            sum += v;
            mx = std::max(mx, v);
        }
        s.cycleMeanMs = sum / n;
        s.cycleMaxMs  = mx;
        const int p   = std::max(0, (n * 99) / 100 - 1);
        std::nth_element(h.begin(), h.begin() + p, h.begin() + n);
        s.cycleP99Ms  = h[p];
    }
```

（`RsiWorker.cpp` 需确认 `#include <array>` 与 `<algorithm>` 已含；nth_element 来自 `<algorithm>`。）

- [ ] **Step 7: Build + 全单测 + 环回回归**

```bash
cmake --build build
for t in test_pose test_app_config test_rsi_codec test_pose_controller test_shared_state test_ipoc_tracker test_session_guard; do
  ./build/tests/$t.exe -o .superpowers/sdd/rf-t4-$t.log,txt
done
./build/tools/loopback_test/loopback_test.exe --seconds 6 > /tmp/lb.log 2>&1 &
sleep 0.2
./build/tools/krc_simulator/krc_simulator.exe --host 127.0.0.1 --port 59152 --cycles 400 > /tmp/sim.log 2>&1
wait
cat /tmp/sim.log
```

Expected: 单测全绿；`replies=400 missed=0 ipoc_mismatch=0`。

- [ ] **Step 8: Commit**

```bash
git add src/net/SharedState.h src/net/RsiWorker.h src/net/RsiWorker.cpp
git commit -m "feat(net): diagnostics — peer address, lifetime loss, last delta, cycle P99"
```

---

### Task 5: C1 — 状态卡

**Files:**
- Modify: `src/ui/MainWindow.h`
- Modify: `src/ui/MainWindow.cpp`

**Interfaces:**
- Consumes: `StatusSnapshot` 新字段（Task 4）、`TrackState`、`krcDelay`。
- Produces: 状态卡 `m_stateCard`（大字颜色）+ 详情 `m_stateDetail` 取代 `m_statusLabel`。Task 6 的使能按钮状态机在 `onRefresh` 同处刷新。

- [ ] **Step 1: Modify `MainWindow.h`**

把 `QLabel *m_statusLabel = nullptr;` 替换为：

```cpp
    QLabel *m_stateCard   = nullptr;   // 大字状态卡（颜色分级）
    QLabel *m_stateDetail = nullptr;   // 次行诊断详情
```

- [ ] **Step 2: Modify 构造函数**

把 `m_statusLabel = new QLabel("未连接", this); m_statusLabel->setStyleSheet("font-weight: bold;");` 和 `outer->addWidget(m_statusLabel);` 替换为：

```cpp
    m_stateCard = new QLabel("未连接", this);
    m_stateCard->setStyleSheet(
        "font-size: 18px; font-weight: bold; color: #888;");
    m_stateDetail = new QLabel("", this);
    m_stateDetail->setStyleSheet("color: #555;");
    outer->addWidget(m_stateCard);
    outer->addWidget(m_stateDetail);
```

- [ ] **Step 3: 重写 `onRefresh` 的状态段**

把 `onRefresh` 里从 `QString st;` 到 `m_statusLabel->setText(st);` 的整段替换为：

```cpp
    // ── 状态卡：颜色分级，让「机器人是否真的连接/可动」一眼可判 ──
    QString cardText;
    bool red = false, yellow = false;
    if (s.state == TrackState::Fault) {
        red = true;
        cardText = QStringLiteral("● 故障: %1").arg(s.faultReason);
    } else if (s.connected) {
        const bool degraded =
            s.missedCount > 0 || s.krcDelay > 0
            || (s.measuredCycleMs > 0.0 && m_cfg.cycleMs > 0.0
                && std::fabs(s.measuredCycleMs - m_cfg.cycleMs)
                       > 0.10 * m_cfg.cycleMs);
        yellow = degraded;
        cardText = s.state == TrackState::Tracking
                       ? (degraded ? "● 已连接（注意）  跟踪中"
                                   : "● 已连接  跟踪中")
                       : (degraded ? "● 已连接（注意）" : "● 已连接");
    } else if (m_listening) {
        cardText = "◐ 监听中（等待 KRC 发帧）";
    } else {
        cardText = "○ 未监听";
    }
    const char *cardColor = red    ? "#c00"
                            : yellow ? "#a06000"
                            : s.connected ? "#080"
                            : m_listening ? "#069"
                                          : "#888";
    m_stateCard->setText(cardText);
    m_stateCard->setStyleSheet(
        QStringLiteral("font-size: 18px; font-weight: bold; color: %1;")
            .arg(QLatin1String(cardColor)));

    // ── 详情行：诊断字段 ──
    const QString peer = s.peerIp4
                             ? QStringLiteral("%1:%2")
                                   .arg(QHostAddress(s.peerIp4).toString())
                                   .arg(s.peerPort)
                             : QStringLiteral("?");
    m_stateDetail->setText(
        QStringLiteral("KRC %1   IPOC %2   周期 %3 ms（均值 %4 / 最大 %5 / P99 %6）"
                       "   回包 %7 µs   丢包 %8 / 累计 %9   RSI Delay %10")
            .arg(peer)
            .arg(s.ipoc)
            .arg(s.measuredCycleMs, 0, 'f', 1)
            .arg(s.cycleMeanMs, 0, 'f', 2)
            .arg(s.cycleMaxMs, 0, 'f', 2)
            .arg(s.cycleP99Ms, 0, 'f', 2)
            .arg(s.maxReplyUs, 0, 'f', 0)
            .arg(s.missedCount)
            .arg(s.lifetimeLost)
            .arg(s.krcDelay));
```

- [ ] **Step 4: Build + 手动驱动验证**

```bash
cmake --build build
./build/tools/loopback_test/loopback_test.exe --seconds 8 > /tmp/lb.log 2>&1 &
sleep 0.2
./build/tools/krc_simulator/krc_simulator.exe --host 127.0.0.1 --port 59152 --cycles 500 > /tmp/sim.log 2>&1
wait
cat /tmp/sim.log
```

手动确认：启动 GUI 连模拟器，状态卡变绿 + "已连接"，详情显示 KRC 127.0.0.1:端口、周期/回包/丢包/RSI Delay。

- [ ] **Step 5: Commit**

```bash
git add src/ui/MainWindow.h src/ui/MainWindow.cpp
git commit -m "feat(ui): status card with color-coded connection/degradation states"
```

---

### Task 6: C2 — 两阶段使能

**Files:**
- Modify: `src/ui/MainWindow.h`
- Modify: `src/ui/MainWindow.cpp`

**Interfaces:**
- Consumes: `SessionGuard::enableChecks`（已有）、`TrackState`。
- Produces: `m_enableBtn`（QPushButton）取代 `m_trackCheck`；新槽 `onPrepareTracking()`。删除旧 `onTrackingToggled` 及所有 `m_trackCheck` 引用。

- [ ] **Step 1: Modify `MainWindow.h`**

把 `QCheckBox *m_trackCheck = nullptr;` 替换为：

```cpp
    QPushButton *m_enableBtn = nullptr;   // 两阶段使能：准备→确认→已使能
```

`private slots:` 把 `void onTrackingToggled(bool on);` 替换为：

```cpp
    void onPrepareTracking();
```

- [ ] **Step 2: Modify 构造函数 — 按钮替换 checkbox**

把：

```cpp
    m_trackCheck = new QCheckBox("使能跟踪", this);
    connect(m_trackCheck, &QCheckBox::toggled,
            this, &MainWindow::onTrackingToggled);
    bar->addWidget(m_trackCheck);
```

替换为：

```cpp
    m_enableBtn = new QPushButton("准备跟踪", this);
    m_enableBtn->setEnabled(false);   // 首帧前不可用
    connect(m_enableBtn, &QPushButton::clicked,
            this, &MainWindow::onPrepareTracking);
    bar->addWidget(m_enableBtn);
```

- [ ] **Step 3: 重写 `onPrepareTracking`（替换旧 `onTrackingToggled`）**

```cpp
void MainWindow::onPrepareTracking()
{
    if (!m_worker)
        return;
    const StatusSnapshot s = m_state.snapshot();

    if (s.state == TrackState::Fault) {
        // 故障锁存：必须归零并复位才能重新使能
        QMetaObject::invokeMethod(m_worker, "resetToActual",
                                  Qt::QueuedConnection);
        return;   // 按钮文本由 onRefresh 统一刷新
    }

    // 联锁：硬拦截无覆盖
    const QStringList blocked =
        SessionGuard::enableChecks(m_cfg, s.measuredCycleMs);
    if (!blocked.isEmpty()) {
        m_interlockLabel->setText(QStringLiteral("使能被拦截：\n")
                                  + blocked.join(QLatin1Char('\n')));
        m_interlockLabel->show();
        return;
    }
    m_interlockLabel->hide();

    // 两阶段确认：操作员核对 BASE/TOOL、目标位姿、限值余量
    const Pose t = s.target;
    const QMessageBox::StandardButton r = QMessageBox::question(
        this, "确认使能跟踪",
        QStringLiteral(
            "使能前请确认：\n"
            "1. 示教器当前 BASE / TOOL 正确\n"
            "2. 目标位姿符合预期\n"
            "3. 限值余量足够（累积 %1 / 上限 %2）\n\n"
            "当前目标: X %3  Y %4  Z %5  A %6  B %7  C %8\n\n"
            "继续？")
            .arg(s.accum.x, 0, 'f', 1)
            .arg(m_cfg.accumLimitPosMm, 0, 'f', 1)
            .arg(t.x, 0, 'f', 1).arg(t.y, 0, 'f', 1)
            .arg(t.z, 0, 'f', 1).arg(t.a, 0, 'f', 1)
            .arg(t.b, 0, 'f', 1).arg(t.c, 0, 'f', 1),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (r != QMessageBox::Yes)
        return;

    QMetaObject::invokeMethod(m_worker, "setTracking",
                              Qt::QueuedConnection, Q_ARG(bool, true));
}
```

- [ ] **Step 4: `onRefresh` 加使能按钮状态机**

在 `m_stateDetail->setText(...)` 之后加：

```cpp
    // ── 使能按钮状态机 ──
    if (s.state == TrackState::Fault) {
        m_enableBtn->setText("归零并复位");
        m_enableBtn->setEnabled(true);
    } else if (s.state == TrackState::Tracking) {
        m_enableBtn->setText("已使能跟踪");
        m_enableBtn->setEnabled(false);
    } else {
        m_enableBtn->setText("准备跟踪");
        m_enableBtn->setEnabled(s.connected);
    }
```

- [ ] **Step 5: 清理旧 `m_trackCheck` 引用**

搜索并删除/替换所有 `m_trackCheck` 引用：
- `onStopListening()`：删 `if (m_trackCheck) m_trackCheck->setChecked(false);`
- `onRefresh()`：删 `if (s.state == TrackState::Fault && m_trackCheck->isChecked()) m_trackCheck->setChecked(false);`
- `onStopTracking()`：`m_trackCheck->setChecked(false);` 删除（软停止走 `onZeroToActual`，按钮状态由 onRefresh 管）
- `onZeroToActual()`：删 `m_trackCheck->setChecked(false);`

确认 `onStopTracking` 保持调用 `onZeroToActual`（目标归零 → 误差零 → 停）。`MainWindow.cpp` 删除 `#include <QCheckBox>`（若 MainWindow.h 有则从 .h 删）。`MainWindow.h` 的 `#include <QCheckBox>` 删除。

- [ ] **Step 6: Build + 手动驱动**

```bash
cmake --build build
```

手动（脚本驱动 GUI）验证四态：未连接→按钮灰；连上→「准备跟踪」可点→确认框→Yes→「已使能跟踪」灰；使能后状态卡显示"已连接 跟踪中"；Fault（触发超限）→按钮变「归零并复位」→点击→回「准备跟踪」。

- [ ] **Step 7: Commit**

```bash
git add src/ui/MainWindow.h src/ui/MainWindow.cpp
git commit -m "feat(ui): two-stage enable with confirmation; fault requires reset"
```

---

### Task 7: C3 — 软停止安全横幅

**Files:**
- Modify: `src/ui/MainWindow.cpp`

- [ ] **Step 1: 升级 `m_safetyNote`**

构造函数里把：

```cpp
    m_safetyNote = new QLabel(
        "「停止跟踪」是软停止，不是急停。急停只能用示教器上的物理急停按钮。",
        this);
    m_safetyNote->setStyleSheet("color: #b00; font-weight: bold;");
```

替换为：

```cpp
    m_safetyNote = new QLabel(
        "⚠ 软件停止 = 目标归零并继续回包。急停只有示教器上的物理急停按钮。",
        this);
    m_safetyNote->setStyleSheet(
        "background-color: #fdd; color: #900; font-weight: bold; "
        "padding: 4px 8px; border: 1px solid #c00;");
```

（保留原有"刻意不开 setWordWrap"的注释逻辑——横幅单行完整可见。）

- [ ] **Step 2: Build + 手动确认**

```bash
cmake --build build
```

手动确认横幅以醒目红底显示，位于按钮栏固定位置。

- [ ] **Step 3: Commit**

```bash
git add src/ui/MainWindow.cpp
git commit -m "feat(ui): fixed soft-stop safety banner"
```

---

### Task 8: C4 — 目标小步进

**Files:**
- Modify: `src/ui/MainWindow.h`
- Modify: `src/ui/MainWindow.cpp`

**Interfaces:**
- Consumes: 现有 `m_targetSpin[i]`（6 轴数值框）、`m_targetSlider[i]`。
- Produces: 每轴 [-][+] 步进按钮 + 全局步长选择器（0.1/0.5/1/5）。滑块保留粗调。

- [ ] **Step 1: Modify `MainWindow.h`**

成员区加：

```cpp
    // 小步进：步长选择器 + 每轴 [-][+] 按钮
    QComboBox *m_stepSel = nullptr;
    std::array<QPushButton *, 6> m_stepMinus{};
    std::array<QPushButton *, 6> m_stepPlus{};
```

`#include <QComboBox>` 加到头部。

- [ ] **Step 2: Modify `buildTargetPanel`**

面板顶部（grid 之前）加步长选择行：

```cpp
    // 步长选择：位置 mm / 姿态 °
    auto *stepLay = new QHBoxLayout;
    stepLay->addWidget(new QLabel("步长", box));
    m_stepSel = new QComboBox(box);
    m_stepSel->addItems({"0.1", "0.5", "1", "5"});
    stepLay->addWidget(m_stepSel);
    stepLay->addStretch();
    auto *v = new QVBoxLayout(box);
    v->addLayout(stepLay);
    v->addLayout(grid);
    delete box->layout();   // 现有 grid 直接 setLayout(box) 的场景需适配
```

（实际按现有 `buildTargetPanel` 的 `auto *grid = new QGridLayout(box);` 结构调整——把 grid 放进带步长行的 VBox，或改用 `box->setLayout`。）

每行在 `grid->addWidget(live, r, 3);` 之后加：

```cpp
        auto *minus = new QPushButton("−", box);
        minus->setMaximumWidth(28);
        grid->addWidget(minus, r, 4);
        m_stepMinus[i] = minus;
        auto *plus = new QPushButton("＋", box);
        plus->setMaximumWidth(28);
        grid->addWidget(plus, r, 5);
        m_stepPlus[i] = plus;

        const int axis = i;
        connect(minus, &QPushButton::clicked, this, [this, axis] {
            const double step = m_stepSel->currentText().toDouble();
            m_targetSpin[axis]->setValue(m_targetSpin[axis]->value() - step);
        });
        connect(plus, &QPushButton::clicked, this, [this, axis] {
            const double step = m_stepSel->currentText().toDouble();
            m_targetSpin[axis]->setValue(m_targetSpin[axis]->value() + step);
        });
```

（`m_targetSpin[i]->setValue` 触发 `onTargetEdited`，走现有排队通道。）

- [ ] **Step 3: Build + 手动驱动**

```bash
cmake --build build
```

手动：勾选步长 1mm，点 X 轴 [+]，X 目标 +1.00；点 [−] 回退。

- [ ] **Step 4: Commit**

```bash
git add src/ui/MainWindow.h src/ui/MainWindow.cpp
git commit -m "feat(ui): per-axis fine-step buttons with selectable step"
```

---

### Task 9: C5 — 参数高级页 + 运行锁定

**Files:**
- Modify: `src/ui/MainWindow.h`
- Modify: `src/ui/MainWindow.cpp`

**Interfaces:**
- Consumes: `AppConfig`、`applyConfig` 排队通道、`StatusSnapshot::state`。
- Produces: 主界面参数区改为只读 + 「控制参数…」按钮；`QDialog` 内编辑；Tracking 时对话框参数禁用。

- [ ] **Step 1: Modify `MainWindow.h`**

`private slots:` 加：

```cpp
    void onEditParams();
```

成员加：

```cpp
    QPushButton *m_paramsBtn = nullptr;   // 打开控制参数对话框
```

- [ ] **Step 2: 改造 `buildParamPanel` 为只读 + 按钮**

`buildParamPanel` 改为：Kp/限速/累积上限 的**只读**显示（QLabel 或禁用的 QDoubleSpinBox）+ 「控制参数…」按钮。

具体：把现有 3 行 spinbox 改为只读 label 行（每行两个只读值），并在面板底部加按钮：

```cpp
    m_paramsBtn = new QPushButton("控制参数…", box);
    connect(m_paramsBtn, &QPushButton::clicked,
            this, &MainWindow::onEditParams);
    grid->addWidget(m_paramsBtn, rows + 1, 0, 1, 3);
```

只读值在 `onRefresh` 更新（Kp/限速/累积上限显示 `m_cfg` 当前值）。

- [ ] **Step 3: 实现 `onEditParams`**

```cpp
void MainWindow::onEditParams()
{
    QDialog dlg(this);
    dlg.setWindowTitle("控制参数");

    auto *form = new QFormLayout(&dlg);
    auto *kpP  = new QDoubleSpinBox; kpP->setRange(0.0, 100.0);  kpP->setDecimals(3);
    auto *kpR  = new QDoubleSpinBox; kpR->setRange(0.0, 100.0);  kpR->setDecimals(3);
    auto *vP   = new QDoubleSpinBox; vP->setRange(0.0, 10000.0); vP->setSuffix(" mm/s");
    auto *vR   = new QDoubleSpinBox; vR->setRange(0.0, 10000.0); vR->setSuffix(" °/s");
    auto *alP  = new QDoubleSpinBox; alP->setRange(0.0, 10000.0); alP->setSuffix(" mm");
    auto *alR  = new QDoubleSpinBox; alR->setRange(0.0, 10000.0); alR->setSuffix(" °");
    kpP->setValue(m_cfg.kpPos);  kpR->setValue(m_cfg.kpRot);
    vP->setValue(m_cfg.vmaxPosMmS);  vR->setValue(m_cfg.vmaxRotDegS);
    alP->setValue(m_cfg.accumLimitPosMm);  alR->setValue(m_cfg.accumLimitRotDeg);

    form->addRow("Kp 位置", kpP);
    form->addRow("Kp 姿态", kpR);
    form->addRow("限速位置", vP);
    form->addRow("限速姿态", vR);
    form->addRow("累积上限位置", alP);
    form->addRow("累积上限姿态", alR);

    // 运行中锁定 + 显示 KRC 硬限与余量
    const bool locked = m_state.snapshot().state == TrackState::Tracking;
    for (auto *w : {static_cast<QWidget*>(kpP), kpR, vP, vR, alP, alR}) {
        w->setEnabled(!locked);
    }
    if (locked) {
        auto *note = new QLabel("运行中：参数已锁定，停止跟踪后可修改。", &dlg);
        note->setStyleSheet("color: #a06000;");
        form->addRow(note);
    } else {
        form->addRow(new QLabel(QStringLiteral(
            "KRC 硬限: 位置 %1 mm / 姿态 %2 °；当前主机上限 %3 / %4，余量 %5 / %6")
            .arg(m_cfg.krcPoscorrLimitPosMm).arg(m_cfg.krcPoscorrLimitRotDeg)
            .arg(alP->value()).arg(alR->value())
            .arg(m_cfg.krcPoscorrLimitPosMm - alP->value())
            .arg(m_cfg.krcPoscorrLimitRotDeg - alR->value()), &dlg));
    }

    auto *btn = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    connect(btn, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btn, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    form->addRow(btn);

    if (dlg.exec() != QDialog::Accepted)
        return;
    AppConfig c2 = m_cfg;
    c2.kpPos = kpP->value();  c2.kpRot = kpR->value();
    c2.vmaxPosMmS = vP->value();  c2.vmaxRotDegS = vR->value();
    c2.accumLimitPosMm = alP->value();  c2.accumLimitRotDeg = alR->value();
    m_cfg = c2;
    QMetaObject::invokeMethod(m_worker, "applyConfig",
                              Qt::QueuedConnection, Q_ARG(AppConfig, c2));
}
```

（`MainWindow.cpp` 需 `#include <QDialog>`, `<QFormLayout>`, `<QDialogButtonBox>`；`MainWindow.h` 保留原参数 spinbox 成员不再使用则删除，`buildParamPanel` 不再建 spinbox——按 Step 2 只读化后确认无残留引用。）

- [ ] **Step 4: `onRefresh` 更新只读参数值**

在 `m_stateDetail->setText(...)` 之后加（若无专门 label 则跳过，改为状态卡 Fault 时可见）：

```cpp
    // 参数只读区（若 Step 2 建了 label 则在此更新；否则参数值已随 m_cfg 固定）
```

（实现时若只读区用 QLabel 存成员，则在此逐行 `setText`。）

- [ ] **Step 5: Build + 手动驱动**

```bash
cmake --build build
```

手动：点「控制参数…」→ 对话框显示当前值 + KRC 硬限 + 余量；非跟踪可改并生效（状态栏/图表反映）；跟踪中打开对话框参数灰显。

- [ ] **Step 6: Commit**

```bash
git add src/ui/MainWindow.h src/ui/MainWindow.cpp
git commit -m "feat(ui): move control params to dialog; lock while tracking"
```

---

### Task 10: C6 图表双图/空态 + C7 读数三列卡片

**Files:**
- Modify: `src/ui/ErrorChart.h`
- Modify: `src/ui/ErrorChart.cpp`
- Modify: `src/ui/MainWindow.h`
- Modify: `src/ui/MainWindow.cpp`

**Interfaces:**
- Consumes: `SampleRing`（`ChartSample` 含 `posErrNorm`/`rotErrNorm`）。
- Produces: `ErrorChart` 单系列 + 空态占位；`MainWindow` 两个 `ErrorChart`（位置/姿态）；读数面板三列卡片化。

- [ ] **Step 1: `ErrorChart` 改单系列 + 空态**

`ErrorChart.h` 构造函数改为：

```cpp
    enum class Mode { Position, Rotation };
    explicit ErrorChart(int windowSeconds, Mode mode, QWidget *parent = nullptr);
```

成员：删 `m_rotSeries`/`m_axisRot`，加 `Mode m_mode`、空态 `QLabel *m_placeholder`。

`ErrorChart.cpp` 构造函数改为：按 `m_mode` 只建一个系列 + 一个 Y 轴（标题 mm 或 °）；`chart->setTitle("位置误差 mm" / "姿态误差 °")`；`m_placeholder` QLabel 叠放：

```cpp
    m_placeholder = new QLabel(
        m_mode == Mode::Position
            ? "等待 RSI 数据…\n请启动 KRL PoseTrack 程序"
            : "等待姿态误差数据…", this);
    m_placeholder->setAlignment(Qt::AlignCenter);
    m_placeholder->setStyleSheet("color: #888; font-size: 14px;");
```

布局：`m_placeholder` 与 `m_view` 在同一层，`updateFrom` 里 `n==0` 时显示 placeholder、隐藏 view；有数据反之。

`updateFrom` 改为只画所选系列：

```cpp
    QList<QPointF> pts;
    double yMax = 1.0;
    for (...) {
        ...
        const double y = (m_mode == Mode::Position) ? m_buf[i].posErrNorm
                                                    : m_buf[i].rotErrNorm;
        pts.append(QPointF(m_buf[i].tSec, y));
        yMax = std::max(yMax, y);
    }
    m_series->replace(pts);
    m_axisY->setRange(0.0, yMax * 1.2);
    m_placeholder->setVisible(n == 0);
    m_view->setVisible(n > 0);
```

（`#include <QLabel>`、`<QHBoxLayout>` 视需要。）

- [ ] **Step 2: `MainWindow` 两个图**

构造函数 `m_chart = new ErrorChart(m_cfg.chartWindowS, this);` 替换为两个（位置在上、姿态在下）：

```cpp
    m_chartPos = new ErrorChart(m_cfg.chartWindowS, ErrorChart::Mode::Position, this);
    m_chartRot = new ErrorChart(m_cfg.chartWindowS, ErrorChart::Mode::Rotation, this);
    auto *chartCol = new QVBoxLayout;
    chartCol->addWidget(m_chartPos, 1);
    chartCol->addWidget(m_chartRot, 1);
    right->addLayout(chartCol, 2);
```

`onRefresh` 的 `m_chart->updateFrom(m_ring);` 改为两个都更新。

`MainWindow.h`：`ErrorChart *m_chart` → `ErrorChart *m_chartPos, *m_chartRot`。

- [ ] **Step 3: C7 读数三列卡片化**

`buildReadoutPanel` 重构为三个并排 `QGroupBox`：

```cpp
QWidget *MainWindow::buildReadoutPanel()
{
    auto *row = new QWidget(this);
    auto *lay = new QHBoxLayout(row);
    const char *titles[3] = {"当前位姿", "目标误差", "累积修正"};
    for (int col = 0; col < 3; ++col) {
        auto *box = new QGroupBox(titles[col], row);
        auto *g = new QGridLayout(box);
        for (int i = 0; i < 6; ++i) {
            g->addWidget(new QLabel(kAxisName[i], box), i, 0);
            auto *lab = new QLabel("--", box);
            lab->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            lab->setStyleSheet(col == 1 ? "color: #b06;" : col == 2 ? "color: #068;" : "color: #0057b8; font-weight: bold;");
            g->addWidget(lab, i, 1);
            (col == 0 ? m_actualLabel[i] : col == 1 ? m_errorLabel[i] : m_accumLabel[i]) = lab;
        }
        lay->addWidget(box, 1);
    }
    return row;
}
```

（`m_actualLabel`/`m_errorLabel`/`m_accumLabel` 数组复用，`onRefresh` 的填充逻辑不变。）

- [ ] **Step 4: 改 `chart_window_s` 默认 10**

`AppConfig.h` 的 `int chartWindowS = 20;` 改为 `= 10;`；`config/rsi_config.json` 的 `"chart_window_s": 20` 改 `10`。

- [ ] **Step 5: Build + 手动驱动**

```bash
cmake --build build
```

手动：无数据时两图显示"等待 RSI 数据…"；连模拟器后显示位置/姿态两图；读数三列卡片显示实际/误差/累积。

- [ ] **Step 6: Commit**

```bash
git add src/ui/ErrorChart.h src/ui/ErrorChart.cpp src/ui/MainWindow.h src/ui/MainWindow.cpp src/core/AppConfig.h config/rsi_config.json
git commit -m "feat(ui): split charts with placeholders; card-style readouts; 10s window"
```

---

### Task 11: 文档更新

**Files:**
- Modify: `docs/real-machine-deployment.md`
- Modify: `docs/verification-matrix.md`

- [ ] **Step 1: `real-machine-deployment.md` 加姿态折返局限**

在 §4 限值梯度之后加一段：

```markdown
### 姿态监控的折返局限（重要）

KRC 回报的 RIst 姿态角可能本身是折返的（±180°），主机无法从 RIst 得知真实
累计旋转圈数。第 2 层姿态监控因此取「RIst 锚点位移逐轴最大」与「主机未折返
累计命令增量（commandedSum）逐轴最大」的较大者——即使 RIst 折返漏报，命令和
仍能反映主机发出的累计修正（高估是安全方向）。真正兜底的是 KRC 侧
POSCORR MaxRotAngle（25°/45°），它基于 KRC 自己累计的修正量，不受折返影响。
```

- [ ] **Step 2: `verification-matrix.md` 加新验证项**

追加：

```markdown
## 本轮新增验证项

| 场景 | 工具 | 预期 | 判定 |
|---|---|---|---|
| 姿态多圈累计 | `test_pose_controller` rotatedOverLimit_firesViaCommandedSumEvenIfRistWraps | commandedSum 兜底触发 Fault | 单测 PASS |
| 目标平滑 | `test_pose_controller` smoothing_* 4 用例 | 阶跃削平 / τ=0 直通 / reset 同步 / 稳态不变 | 单测 PASS |
| UI 状态卡 | 手动驱动 GUI | 颜色分级 + 诊断字段 | 手动 |
| 两阶段使能 | 手动驱动 GUI | 准备→确认→已使能；Fault→归零并复位 | 手动 |
```

- [ ] **Step 3: Commit**

```bash
git add docs/real-machine-deployment.md docs/verification-matrix.md
git commit -m "docs: rotation wrap limitation, smoothing, UI hardening verification"
```

---

## Self-Review 记录

- **Spec 覆盖**：Part A（Task 1-2）、Part B（Task 3）、Part C C1（Task 5）、C2（Task 6）、C3（Task 7）、C4（Task 8）、C5（Task 9）、C6+C7（Task 10）、C8 字段（Task 4）、文档（Task 11）。无缺口。
- **占位符**：无 TBD/TODO。UI 任务的布局细节（Step 2 的 `buildTargetPanel` 结构适配）留给实现者按现有代码精确落位，已注明适配点。
- **类型一致**：`ErrorChart::Mode`、`m_stateCard`/`m_stateDetail`、`m_enableBtn`、`onPrepareTracking`、`StatusSnapshot` 新字段名在任务间一致。
- **风险注意**：Task 6 删 checkbox 时需清理所有 `m_trackCheck` 引用（Step 5 列出全部）；Task 9 只读化参数面板时确认无残留 spinbox 引用；Task 10 改 `ErrorChart` 构造函数会影响 `MainWindow`（同步改）。


