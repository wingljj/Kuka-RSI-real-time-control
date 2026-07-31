# KUKA RSI POSCORR 实时位姿跟踪上位机 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 构建一套基于 RSI POSCORR（BASE 坐标系 + RELATIVE 模式）的实时位姿跟踪系统，含 KRC 侧配置与 KRL 程序、C++ Qt 上位机、以及可脱机运行的测试链路。

**Architecture:** 上位机以双线程运行 —— 通信线程独占 UDP 收发与闭环计算（每周期解析 `<Rob>`、算限幅增量、回 `<Sen>`），GUI 线程以 33ms 定时拉取共享状态刷新数值与曲线。核心逻辑拆为两个无 IO 的纯计算单元（`RsiCodec`、`PoseController`），使整条实时链路可用一个 KRC 模拟器在不开虚拟机的情况下验证。

**Tech Stack:** C++17 / Qt 6.5.3 (mingw_64) / MinGW-W64 11.2.0 / CMake 3.29.3 + Ninja / Qt Test / Qt Charts

## Global Constraints

以下取值与规则从设计文档逐字继承，**每个任务的要求都隐含包含本节**：

- **RELATIVE 模式下 `RKorr` 是「相对当前位置的修正增量」，不是目标位置。** 发送绝对坐标会被解释为要在一个 IPO 周期内完成的巨量运动。
- **POSCORR 内建安全限值，`RKorr` 超过约 50mm 即拒绝并停机。** 限幅是功能必需项，不是优化项。
- **`IPOC` 必须原样回显**，RSI 依此做时序同步，回错等同丢包。
- **无论出什么错，都要按时回包。** 唯一不可接受的行为是「因出错而不回包」。
- **`RKorr` 与 `AKorr` 不可同时供值**，新 POSCORR 程序与现有 AXISCORR 程序必须互斥运行。
- **通信线程绝不触碰 GUI 对象，绝不执行可能阻塞的操作**（无文件 IO、无日志落盘、无动态内存分配）。
- 位置与姿态量纲不同，全程分两组独立处理：位置 mm / 姿态 °。
- 网络：宿主监听 `192.168.44.1`，guest 为 `192.168.44.128`（host-only VMnet1）。
- 语言标准 C++17；Qt 组件限定 `Core Network Widgets Charts Test`（`Qt6DataVisualization` 与 `Qt6SerialPort` 在本机不存在，不得引用）。
- 默认参数：`Kp_pos`/`Kp_rot` = 0.3/0.3，`vmax_pos` = 50 mm/s，`vmax_rot` = 10 °/s，`accum_limit_pos` = 30 mm，`accum_limit_rot` = 15 °。
- **guest 文件访问尚未打通**，故上位机与前两层测试必须能完全脱离虚拟机独立开发与验证。

## 工具链路径（命令中逐字使用）

```bash
# 被 CMake 消费的路径用 Windows 形式；要放进 PATH 的必须用 POSIX 形式 ——
# MSYS/Git-Bash 按 ':' 切分 PATH，"D:/Software/..." 会被撕成 "D" 和
# "/Software/..." 两个无效项，导致 Ninja 找不到、g++ 误解析到其他安装。
QTDIR="D:/Software/QT/content/6.5.3/mingw_64"
CMAKE="D:/Software/QT/content/Tools/CMake_64/bin/cmake.exe"
MINGW="/d/Software/QT/content/Tools/mingw1120_64/bin"
NINJA="/d/Software/QT/content/Tools/Ninja"
```

配置与构建（后续任务反复使用，记作 **BUILD 命令**）：

```bash
cd /e/kuka_rsi_win
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" -S . -B build -G Ninja \
  -DCMAKE_PREFIX_PATH="$QTDIR" -DCMAKE_BUILD_TYPE=Debug
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build
```

## File Structure

| 文件 | 职责 |
|---|---|
| `CMakeLists.txt` | 顶层构建：主程序 target + 引入 tests/ 与 tools/ |
| `config/rsi_config.json` | 外置配置：网络、周期、增益、四层限值 |
| `src/main.cpp` | 入口：加载配置、建窗口、启动 |
| `src/core/Pose.h` | `Pose` 六自由度值类型 + `wrap180` 自由函数（纯头文件） |
| `src/core/AppConfig.h/.cpp` | 配置结构体与 JSON 加载，无 GUI 依赖 |
| `src/core/RsiCodec.h/.cpp` | `<Rob>` 解析 / `<Sen>` 生成，纯函数无 IO |
| `src/core/PoseController.h/.cpp` | 误差 → 限幅增量 + 累积限值，纯计算无 IO |
| `src/net/RsiWorker.h/.cpp` | UDP socket + 通信线程循环，组合 Codec 与 Controller |
| `src/net/SharedState.h` | 通信线程与 GUI 线程间的互斥保护快照 + 环形缓冲 |
| `src/ui/MainWindow.h/.cpp` | 布局、数值显示、状态机、控制按钮 |
| `src/ui/ErrorChart.h/.cpp` | QtCharts 误差曲线封装 |
| `tests/CMakeLists.txt` | 三个测试 target |
| `tests/test_pose.cpp` | `wrap180` 边界 |
| `tests/test_rsi_codec.cpp` | 解析/生成/畸形输入 |
| `tests/test_pose_controller.cpp` | 限幅、量纲分离、累积限值 |
| `tools/krc_simulator/main.cpp` | 假 KRC：按周期发 `<Rob>` 收 `<Sen>`，统计回包率与延迟 |
| `krc/PoseTrack.rsi` | RSI Visual 模型，由 guest 上的 RSI Visual 生成 |
| `krc/PoseTrack.rsi.xml` | 运行时对象图：POSCORR(27) ← ETHERNET(64) |
| `krc/PoseTrack_ethernet.xml` | 通信配置，被 ETHERNET 的 `ConfigFile` 引用 |
| `krc/PoseTrack.src` | KRL 程序（部署到 guest） |
| `docs/deployment.md` | 部署步骤与联机测试清单 |

---

### Task 1: 仓库骨架与构建系统

**Files:**
- Create: `.gitignore`, `CMakeLists.txt`, `src/main.cpp`

**Interfaces:**
- Consumes: 无
- Produces: 可构建的 CMake 工程；target 名 `rsi_host`

- [ ] **Step 1: 初始化仓库**

```bash
cd /e/kuka_rsi_win
git init
git add docs/superpowers/specs/2026-07-30-kuka-rsi-poscorr-qt-design.md
git commit -m "docs: add approved design spec for RSI POSCORR pose tracking"
```

- [ ] **Step 2: 写 `.gitignore`**

```gitignore
build/
_screen*.png
*.log
setup_vmnet1.ps1
set_vmnet1_native.ps1
vmnet1_backup.txt
CMakeLists.txt.user
```

- [ ] **Step 3: 写顶层 `CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.21)
project(kuka_rsi_win LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Network Widgets Charts Test)

add_library(rsi_core STATIC
    src/core/AppConfig.cpp
    src/core/RsiCodec.cpp
    src/core/PoseController.cpp
)
target_include_directories(rsi_core PUBLIC src)
target_link_libraries(rsi_core PUBLIC Qt6::Core)

# RsiWorker 单独成库：rsi_host 与 loopback_test 都要用它，
# 若各自把 RsiWorker.cpp 列进源文件会重复编译且 AUTOMOC 跨目录易出错。
add_library(rsi_net STATIC src/net/RsiWorker.cpp)
target_link_libraries(rsi_net PUBLIC rsi_core Qt6::Network)

add_executable(rsi_host
    src/main.cpp
    src/ui/MainWindow.cpp
    src/ui/ErrorChart.cpp
)
target_link_libraries(rsi_host PRIVATE
    rsi_net Qt6::Core Qt6::Network Qt6::Widgets Qt6::Charts)

enable_testing()
add_subdirectory(tests)
add_subdirectory(tools/krc_simulator)
```

- [ ] **Step 4: 建占位源文件让工程可配置**

创建以下文件，内容为最小可编译骨架。`src/main.cpp`：

```cpp
#include <QApplication>
#include <QLabel>

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    QLabel label("RSI host skeleton");
    label.resize(320, 120);
    label.show();
    return app.exec();
}
```

创建空实现文件（后续任务填充），每个仅含一行注释：
`src/core/AppConfig.cpp`、`src/core/RsiCodec.cpp`、`src/core/PoseController.cpp`、`src/net/RsiWorker.cpp`、`src/ui/MainWindow.cpp`、`src/ui/ErrorChart.cpp`，内容均为：

```cpp
// filled in by a later task
```

`tests/CMakeLists.txt`：

```cmake
# test targets added by later tasks
```

`tools/krc_simulator/CMakeLists.txt`：

```cmake
# simulator target added by a later task
```

- [ ] **Step 5: 配置并构建**

Run:
```bash
cd /e/kuka_rsi_win
QTDIR="D:/Software/QT/content/6.5.3/mingw_64"
MINGW="D:/Software/QT/content/Tools/mingw1120_64/bin"
NINJA="D:/Software/QT/content/Tools/Ninja"
CMAKE="D:/Software/QT/content/Tools/CMake_64/bin/cmake.exe"
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" -S . -B build -G Ninja \
  -DCMAKE_PREFIX_PATH="$QTDIR" -DCMAKE_BUILD_TYPE=Debug
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build
```

Expected: 配置阶段打印 `-- Configuring done` 与 `-- Generating done`；构建产出 `build/rsi_host.exe`，无 error。

- [ ] **Step 6: 提交**

```bash
git add .gitignore CMakeLists.txt src tests tools
git commit -m "build: scaffold CMake project with Qt 6.5.3 mingw toolchain"
```

---

### Task 2: Pose 值类型与 wrap180

**Files:**
- Create: `src/core/Pose.h`, `tests/test_pose.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: 无
- Produces:
  - `struct Pose { double x, y, z, a, b, c; }`
  - `double wrap180(double deg)` — 把任意角度归一化到 `(-180, 180]`
  - `Pose poseSub(const Pose &lhs, const Pose &rhs)` — 逐分量相减，**姿态分量自动 wrap180**

- [ ] **Step 1: 写失败测试**

`tests/test_pose.cpp`：

```cpp
#include <QtTest>
#include "core/Pose.h"

class TestPose : public QObject
{
    Q_OBJECT
private slots:
    void wrap180_withinRange()
    {
        QCOMPARE(wrap180(0.0), 0.0);
        QCOMPARE(wrap180(90.0), 90.0);
        QCOMPARE(wrap180(-90.0), -90.0);
        QCOMPARE(wrap180(180.0), 180.0);
    }

    void wrap180_crossesBoundary()
    {
        // 关键用例：目标 179，实际 -179，差值 358 应折成 -2
        QCOMPARE(wrap180(358.0), -2.0);
        QCOMPARE(wrap180(-358.0), 2.0);
        QCOMPARE(wrap180(181.0), -179.0);
        QCOMPARE(wrap180(-181.0), 179.0);
    }

    void wrap180_multipleTurns()
    {
        QCOMPARE(wrap180(720.0), 0.0);
        QCOMPARE(wrap180(725.0), 5.0);
        QCOMPARE(wrap180(-725.0), -5.0);
    }

    void poseSub_wrapsRotationOnly()
    {
        Pose target{10.0, 0.0, 0.0, 179.0, 0.0, 0.0};
        Pose actual{0.0, 0.0, 0.0, -179.0, 0.0, 0.0};
        const Pose d = poseSub(target, actual);
        QCOMPARE(d.x, 10.0);        // 位置直接相减，不 wrap
        QCOMPARE(d.a, -2.0);        // 姿态走近路
    }
};

QTEST_MAIN(TestPose)
#include "test_pose.moc"
```

- [ ] **Step 2: 加测试 target 并运行，确认失败**

`tests/CMakeLists.txt`：

```cmake
add_executable(test_pose test_pose.cpp)
target_link_libraries(test_pose PRIVATE rsi_core Qt6::Test)
add_test(NAME test_pose COMMAND test_pose)
```

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target test_pose
```
Expected: FAIL —— 编译错误 `core/Pose.h: No such file or directory`。

- [ ] **Step 3: 实现 `src/core/Pose.h`**

```cpp
#pragma once
#include <QMetaType>
#include <cmath>

struct Pose
{
    double x = 0.0;   // mm
    double y = 0.0;   // mm
    double z = 0.0;   // mm
    double a = 0.0;   // deg
    double b = 0.0;   // deg
    double c = 0.0;   // deg
};

// 归一化到 (-180, 180]
inline double wrap180(double deg)
{
    double r = std::fmod(deg + 180.0, 360.0);
    if (r <= 0.0)
        r += 360.0;
    return r - 180.0;
}

// 逐分量相减；姿态分量取最短角路径
inline Pose poseSub(const Pose &lhs, const Pose &rhs)
{
    return Pose{
        lhs.x - rhs.x,
        lhs.y - rhs.y,
        lhs.z - rhs.z,
        wrap180(lhs.a - rhs.a),
        wrap180(lhs.b - rhs.b),
        wrap180(lhs.c - rhs.c),
    };
}

// 必需：Pose 会通过 Q_ARG 跨线程排队传递（Task 10/12），
// 未注册元类型会导致队列连接在运行时静默失败。
Q_DECLARE_METATYPE(Pose)
```

- [ ] **Step 4: 运行测试，确认通过**

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target test_pose
./build/tests/test_pose.exe
```
Expected: PASS，输出 `Totals: 5 passed, 0 failed`（含 initTestCase/cleanupTestCase）。

- [ ] **Step 5: 提交**

```bash
git add src/core/Pose.h tests/test_pose.cpp tests/CMakeLists.txt
git commit -m "feat(core): add Pose type and wrap180 shortest-angle normalization"
```

---

### Task 3: AppConfig 与 JSON 加载

**Files:**
- Create: `src/core/AppConfig.h`, `config/rsi_config.json`, `tests/test_app_config.cpp`
- Modify: `src/core/AppConfig.cpp`, `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: 无
- Produces:
  - `struct AppConfig` 含字段 `listenIp`(QString), `listenPort`(quint16), `cycleMs`(double), `senType`(QString), `watchdogMissLimit`(int), `kpPos`, `kpRot`, `vmaxPosMmS`, `vmaxRotDegS`, `accumLimitPosMm`, `accumLimitRotDeg`(均 double), `refreshMs`(int), `chartWindowS`(int)
  - `static AppConfig AppConfig::defaults()`
  - `static bool AppConfig::loadFromFile(const QString &path, AppConfig *out, QString *error)`

- [ ] **Step 1: 写配置文件 `config/rsi_config.json`**

```json
{
  "network": {
    "listen_ip":   "192.168.44.1",
    "listen_port": 59152
  },
  "rsi": {
    "cycle_ms":            12.0,
    "sen_type":            "ImFree",
    "watchdog_miss_limit": 3
  },
  "control": {
    "kp_pos":              0.30,
    "kp_rot":              0.30,
    "vmax_pos_mm_s":       50.0,
    "vmax_rot_deg_s":      10.0,
    "accum_limit_pos_mm":  30.0,
    "accum_limit_rot_deg": 15.0
  },
  "ui": {
    "refresh_ms":     33,
    "chart_window_s": 20
  }
}
```

- [ ] **Step 2: 写失败测试 `tests/test_app_config.cpp`**

```cpp
#include <QtTest>
#include <QTemporaryFile>
#include "core/AppConfig.h"

class TestAppConfig : public QObject
{
    Q_OBJECT
private slots:
    void defaults_matchSpec()
    {
        const AppConfig c = AppConfig::defaults();
        QCOMPARE(c.kpPos, 0.30);
        QCOMPARE(c.kpRot, 0.30);
        QCOMPARE(c.vmaxPosMmS, 50.0);
        QCOMPARE(c.vmaxRotDegS, 10.0);
        QCOMPARE(c.accumLimitPosMm, 30.0);
        QCOMPARE(c.accumLimitRotDeg, 15.0);
        QCOMPARE(c.watchdogMissLimit, 3);
    }

    void load_readsAllFields()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({
          "network": { "listen_ip": "10.0.0.5", "listen_port": 12345 },
          "rsi": { "cycle_ms": 4.0, "sen_type": "MyType",
                   "watchdog_miss_limit": 7 },
          "control": { "kp_pos": 0.1, "kp_rot": 0.2,
                       "vmax_pos_mm_s": 11.0, "vmax_rot_deg_s": 22.0,
                       "accum_limit_pos_mm": 33.0,
                       "accum_limit_rot_deg": 44.0 },
          "ui": { "refresh_ms": 50, "chart_window_s": 60 }
        })");
        f.flush();

        AppConfig c;
        QString err;
        QVERIFY2(AppConfig::loadFromFile(f.fileName(), &c, &err),
                 qPrintable(err));
        QCOMPARE(c.listenIp, QString("10.0.0.5"));
        QCOMPARE(c.listenPort, quint16(12345));
        QCOMPARE(c.cycleMs, 4.0);
        QCOMPARE(c.senType, QString("MyType"));
        QCOMPARE(c.watchdogMissLimit, 7);
        QCOMPARE(c.kpPos, 0.1);
        QCOMPARE(c.accumLimitRotDeg, 44.0);
        QCOMPARE(c.refreshMs, 50);
        QCOMPARE(c.chartWindowS, 60);
    }

    void load_missingFieldKeepsDefault()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write(R"({ "control": { "kp_pos": 0.9 } })");
        f.flush();

        AppConfig c;
        QString err;
        QVERIFY(AppConfig::loadFromFile(f.fileName(), &c, &err));
        QCOMPARE(c.kpPos, 0.9);                        // 覆盖
        QCOMPARE(c.vmaxPosMmS, 50.0);                  // 保留默认
        QCOMPARE(c.listenIp, QString("192.168.44.1")); // 保留默认
    }

    void load_malformedJsonReportsError()
    {
        QTemporaryFile f;
        QVERIFY(f.open());
        f.write("{ this is not json ");
        f.flush();

        AppConfig c;
        QString err;
        QVERIFY(!AppConfig::loadFromFile(f.fileName(), &c, &err));
        QVERIFY(!err.isEmpty());
    }

    void load_missingFileReportsError()
    {
        AppConfig c;
        QString err;
        QVERIFY(!AppConfig::loadFromFile("Z:/nonexistent.json", &c, &err));
        QVERIFY(!err.isEmpty());
    }
};

QTEST_MAIN(TestAppConfig)
#include "test_app_config.moc"
```

- [ ] **Step 3: 加 target 并运行，确认失败**

追加到 `tests/CMakeLists.txt`：

```cmake
add_executable(test_app_config test_app_config.cpp)
target_link_libraries(test_app_config PRIVATE rsi_core Qt6::Test)
add_test(NAME test_app_config COMMAND test_app_config)
```

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target test_app_config
```
Expected: FAIL —— `core/AppConfig.h: No such file or directory`。

- [ ] **Step 4: 写 `src/core/AppConfig.h`**

```cpp
#pragma once
#include <QMetaType>
#include <QString>

struct AppConfig
{
    QString listenIp          = "192.168.44.1";
    quint16 listenPort        = 59152;

    double  cycleMs           = 12.0;
    QString senType           = "ImFree";
    int     watchdogMissLimit = 3;

    double  kpPos             = 0.30;
    double  kpRot             = 0.30;
    double  vmaxPosMmS        = 50.0;
    double  vmaxRotDegS       = 10.0;
    double  accumLimitPosMm   = 30.0;
    double  accumLimitRotDeg  = 15.0;

    int     refreshMs         = 33;
    int     chartWindowS      = 20;

    static AppConfig defaults() { return AppConfig{}; }

    // 未出现的字段保留 out 中原有值（即默认值）
    static bool loadFromFile(const QString &path, AppConfig *out,
                             QString *error);
};

// 必需：AppConfig 会通过 Q_ARG 跨线程排队传递（Task 10 的 applyConfig），
// 未注册元类型会导致队列连接在运行时静默失败。
Q_DECLARE_METATYPE(AppConfig)
```

- [ ] **Step 5: 实现 `src/core/AppConfig.cpp`**

```cpp
#include "core/AppConfig.h"

#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>

namespace {

void readDouble(const QJsonObject &o, const char *key, double *dst)
{
    if (o.contains(key) && o.value(key).isDouble())
        *dst = o.value(key).toDouble();
}

void readInt(const QJsonObject &o, const char *key, int *dst)
{
    if (o.contains(key) && o.value(key).isDouble())
        *dst = o.value(key).toInt();
}

void readString(const QJsonObject &o, const char *key, QString *dst)
{
    if (o.contains(key) && o.value(key).isString())
        *dst = o.value(key).toString();
}

} // namespace

bool AppConfig::loadFromFile(const QString &path, AppConfig *out,
                             QString *error)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        if (error)
            *error = QStringLiteral("cannot open %1: %2")
                         .arg(path, f.errorString());
        return false;
    }

    QJsonParseError pe{};
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &pe);
    if (pe.error != QJsonParseError::NoError) {
        if (error)
            *error = QStringLiteral("JSON parse error at offset %1: %2")
                         .arg(pe.offset)
                         .arg(pe.errorString());
        return false;
    }
    if (!doc.isObject()) {
        if (error)
            *error = QStringLiteral("root is not a JSON object");
        return false;
    }

    const QJsonObject root = doc.object();

    const QJsonObject net = root.value("network").toObject();
    readString(net, "listen_ip", &out->listenIp);
    // 与其余字段一致地做类型守卫：无守卫时 "59152"(带引号) 或 null 都会
    // 让 toInt() 返回 0，端口 0 会绑到 OS 分配的临时端口，KRC 永远连不上，
    // 且表现为静默失败而非配置报错。范围检查挡住 quint16 截断。
    if (net.value("listen_port").isDouble()) {
        const int p = net.value("listen_port").toInt(-1);
        if (p > 0 && p <= 65535)
            out->listenPort = quint16(p);
    }

    const QJsonObject rsi = root.value("rsi").toObject();
    readDouble(rsi, "cycle_ms", &out->cycleMs);
    readString(rsi, "sen_type", &out->senType);
    readInt(rsi, "watchdog_miss_limit", &out->watchdogMissLimit);

    const QJsonObject ctl = root.value("control").toObject();
    readDouble(ctl, "kp_pos", &out->kpPos);
    readDouble(ctl, "kp_rot", &out->kpRot);
    readDouble(ctl, "vmax_pos_mm_s", &out->vmaxPosMmS);
    readDouble(ctl, "vmax_rot_deg_s", &out->vmaxRotDegS);
    readDouble(ctl, "accum_limit_pos_mm", &out->accumLimitPosMm);
    readDouble(ctl, "accum_limit_rot_deg", &out->accumLimitRotDeg);

    const QJsonObject ui = root.value("ui").toObject();
    readInt(ui, "refresh_ms", &out->refreshMs);
    readInt(ui, "chart_window_s", &out->chartWindowS);

    if (error)
        error->clear();
    return true;
}
```

- [ ] **Step 6: 运行测试，确认通过**

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target test_app_config
./build/tests/test_app_config.exe
```
Expected: PASS，5 个用例全过。

- [ ] **Step 7: 提交**

```bash
git add src/core/AppConfig.h src/core/AppConfig.cpp config/rsi_config.json \
        tests/test_app_config.cpp tests/CMakeLists.txt
git commit -m "feat(core): add AppConfig with JSON loading and default fallback"
```

---

### Task 4: RsiCodec —— 解析 `<Rob>` 与生成 `<Sen>`

**Files:**
- Create: `src/core/RsiCodec.h`, `tests/test_rsi_codec.cpp`
- Modify: `src/core/RsiCodec.cpp`, `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `Pose`（Task 2）
- Produces:
  - `struct RobFrame { Pose rist; Pose rsol; quint64 ipoc = 0; bool valid = false; }`
  - `static RobFrame RsiCodec::parseRob(const QByteArray &datagram)`
  - `static QByteArray RsiCodec::buildSen(const Pose &korr, quint64 ipoc, const QString &senType)`

`buildSen` 必须原样回显传入的 `ipoc`。解析失败时返回 `valid == false`，调用方仍须回包（见 Global Constraints）。

- [ ] **Step 1: 写失败测试 `tests/test_rsi_codec.cpp`**

```cpp
#include <QtTest>
#include "core/RsiCodec.h"

class TestRsiCodec : public QObject
{
    Q_OBJECT
private slots:
    void parseRob_readsRistAndIpoc()
    {
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"1250.5\" Y=\"-10.25\" Z=\"1000.0\" "
                  "A=\"1.5\" B=\"90.0\" C=\"-45.5\"/>"
            "<RSol X=\"1250.0\" Y=\"0.0\" Z=\"1000.0\" "
                  "A=\"0.0\" B=\"90.0\" C=\"0.0\"/>"
            "<Delay D=\"0\"/>"
            "<IPOC>123456789</IPOC>"
            "</Rob>";

        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.rist.x, 1250.5);
        QCOMPARE(f.rist.y, -10.25);
        QCOMPARE(f.rist.z, 1000.0);
        QCOMPARE(f.rist.a, 1.5);
        QCOMPARE(f.rist.b, 90.0);
        QCOMPARE(f.rist.c, -45.5);
        QCOMPARE(f.rsol.x, 1250.0);
        QCOMPARE(f.ipoc, quint64(123456789));
    }

    void parseRob_handlesLargeIpoc()
    {
        const QByteArray d =
            "<Rob Type=\"KUKA\">"
            "<RIst X=\"0\" Y=\"0\" Z=\"0\" A=\"0\" B=\"0\" C=\"0\"/>"
            "<IPOC>18446744073709551615</IPOC>"
            "</Rob>";
        const RobFrame f = RsiCodec::parseRob(d);
        QVERIFY(f.valid);
        QCOMPARE(f.ipoc, std::numeric_limits<quint64>::max());
    }

    void parseRob_malformedIsInvalidNotCrash()
    {
        QVERIFY(!RsiCodec::parseRob("").valid);
        QVERIFY(!RsiCodec::parseRob("not xml at all").valid);
        QVERIFY(!RsiCodec::parseRob("<Rob><RIst X=\"1\"/></Rob>").valid);
        QVERIFY(!RsiCodec::parseRob("<Rob><IPOC>5</IPOC></Rob>").valid);
        // 截断的 XML
        QVERIFY(!RsiCodec::parseRob("<Rob><RIst X=\"1\" Y=\"2\"").valid);
    }

    void buildSen_echoesIpocVerbatim()
    {
        const Pose k{0.5, -1.25, 2.0, 0.1, -0.2, 0.3};
        const QByteArray s = RsiCodec::buildSen(k, 987654321, "ImFree");
        QVERIFY(s.contains("<Sen Type=\"ImFree\">"));
        QVERIFY(s.contains("<IPOC>987654321</IPOC>"));
        QVERIFY(s.contains("</Sen>"));
    }

    void buildSen_roundTripsThroughParser()
    {
        // 生成的 RKorr 数值须能被重新读回，验证格式与精度
        const Pose k{0.5, -1.25, 2.0, 0.1, -0.2, 0.3};
        const QByteArray s = RsiCodec::buildSen(k, 42, "ImFree");
        QVERIFY(s.contains("X=\"0.5000\""));
        QVERIFY(s.contains("Y=\"-1.2500\""));
        QVERIFY(s.contains("C=\"0.3000\""));
    }

    void buildSen_zeroKorrIsWellFormed()
    {
        const QByteArray s = RsiCodec::buildSen(Pose{}, 1, "ImFree");
        QVERIFY(s.contains("X=\"0.0000\""));
        QVERIFY(s.contains("<IPOC>1</IPOC>"));
    }
};

QTEST_MAIN(TestRsiCodec)
#include "test_rsi_codec.moc"
```

- [ ] **Step 2: 加 target 并运行，确认失败**

追加到 `tests/CMakeLists.txt`：

```cmake
add_executable(test_rsi_codec test_rsi_codec.cpp)
target_link_libraries(test_rsi_codec PRIVATE rsi_core Qt6::Test)
add_test(NAME test_rsi_codec COMMAND test_rsi_codec)
```

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target test_rsi_codec
```
Expected: FAIL —— `core/RsiCodec.h: No such file or directory`。

- [ ] **Step 3: 写 `src/core/RsiCodec.h`**

```cpp
#pragma once
#include <QByteArray>
#include <QString>
#include "core/Pose.h"

struct RobFrame
{
    Pose    rist;           // 实际位姿
    Pose    rsol;           // 额定位姿
    quint64 ipoc  = 0;
    bool    valid = false;
};

class RsiCodec
{
public:
    // 解析 KRC 发来的 <Rob> 报文。失败时 valid == false。
    static RobFrame parseRob(const QByteArray &datagram);

    // 生成回给 KRC 的 <Sen> 报文。ipoc 必须原样回显。
    static QByteArray buildSen(const Pose &korr, quint64 ipoc,
                               const QString &senType);
};
```

- [ ] **Step 4: 实现 `src/core/RsiCodec.cpp`**

```cpp
#include "core/RsiCodec.h"

#include <QXmlStreamReader>
#include <cmath>

namespace {

// 从元素属性读六自由度；六项缺一即失败
bool readPoseAttrs(const QXmlStreamAttributes &at, Pose *p)
{
    static const char *keys[6] = {"X", "Y", "Z", "A", "B", "C"};
    double *dst[6] = {&p->x, &p->y, &p->z, &p->a, &p->b, &p->c};

    for (int i = 0; i < 6; ++i) {
        if (!at.hasAttribute(QLatin1String(keys[i])))
            return false;
        bool ok = false;
        const double v =
            at.value(QLatin1String(keys[i])).toDouble(&ok);
        if (!ok)
            return false;
        *dst[i] = v;
    }
    return true;
}

} // namespace

RobFrame RsiCodec::parseRob(const QByteArray &datagram)
{
    RobFrame out;
    bool haveRist = false;
    bool haveIpoc = false;

    QXmlStreamReader xml(datagram);
    while (!xml.atEnd()) {
        xml.readNext();
        if (!xml.isStartElement())
            continue;

        const QStringView name = xml.name();
        if (name == QLatin1String("RIst")) {
            haveRist = readPoseAttrs(xml.attributes(), &out.rist);
        } else if (name == QLatin1String("RSol")) {
            readPoseAttrs(xml.attributes(), &out.rsol);  // 可选
        } else if (name == QLatin1String("IPOC")) {
            const QString t = xml.readElementText();
            // readElementText() 在文档提前结束时会返回"已累积的部分字符"而非
            // 报废，所以必须在此检查 reader 状态：被截断的 <IPOC>5551 会解析出
            // 数值 5551（真实值可能是 5551234）并被误判为有效，而 IPOC 字节精确
            // 是硬契约——回错等同丢包。现实触发场景是接收缓冲区过小导致
            // readDatagram 静默截断，而 IPOC 恰位于 RSI 报文末尾。
            if (!xml.hasError()) {
                bool ok = false;
                const quint64 v = t.toULongLong(&ok);
                if (ok) {
                    out.ipoc = v;
                    haveIpoc = true;
                }
            }
        }
    }

    // 尾部填充容忍：真实 KRC datagram 可能带尾部 NUL 或空白，
    // QXmlStreamReader 会就此报错；若因任何 reader 错误一律拒绝，将是每帧
    // 都失败的全盘故障而非间歇故障。此处只依据"两个必需元素是否都完整读到"
    // 判定——IPOC 的截断已在上面的 hasError 守卫处挡掉，RIst 的属性在
    // StartElement 时就已完整解析（否则不会 emit），故二者均可信。
    out.valid = haveRist && haveIpoc;
    return out;
}

QByteArray RsiCodec::buildSen(const Pose &korr, quint64 ipoc,
                              const QString &senType)
{
    // 手工拼接而非 QXmlStreamWriter：报文极短且格式固定，
    // 避免在实时路径上引入额外分配与格式化开销。
    QByteArray s;
    s.reserve(256);
    s += "<Sen Type=\"";
    s += senType.toUtf8();
    s += "\">\n<RKorr";

    static const char *keys[6] = {" X=\"", " Y=\"", " Z=\"",
                                  " A=\"", " B=\"", " C=\""};
    const double vals[6] = {korr.x, korr.y, korr.z,
                            korr.a, korr.b, korr.c};
    for (int i = 0; i < 6; ++i) {
        s += keys[i];
        // 非有限值守卫：NaN 会输出 "nan"、Inf 输出 "inf"，都不是 4 位小数，
        // KRC 的 RKorr 解析不了，等同丢包并停机。而上游基于比较的限幅会
        // 传播 NaN 而非限界它，所以这道防线必须在此层——它是 wire 格式的
        // 保证者。替换为 0.0 表示"本周期无修正"，是正确的降级行为。
        const double v = std::isfinite(vals[i]) ? vals[i] : 0.0;
        s += QByteArray::number(v, 'f', 4);
        s += '"';
    }

    s += "/>\n<IPOC>";
    s += QByteArray::number(ipoc);
    s += "</IPOC>\n</Sen>";
    return s;
}
```

- [ ] **Step 5: 运行测试，确认通过**

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target test_rsi_codec
./build/tests/test_rsi_codec.exe
```
Expected: PASS，6 个用例全过。

- [ ] **Step 6: 提交**

```bash
git add src/core/RsiCodec.h src/core/RsiCodec.cpp tests/test_rsi_codec.cpp \
        tests/CMakeLists.txt
git commit -m "feat(core): add RsiCodec for Rob parsing and Sen building with IPOC echo"
```

---

### Task 5: PoseController —— 限幅 P 控制与累积限值

**Files:**
- Create: `src/core/PoseController.h`, `tests/test_pose_controller.cpp`
- Modify: `src/core/PoseController.cpp`, `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `Pose`, `poseSub`（Task 2）；`AppConfig`（Task 3）
- Produces:
  - `enum class TrackState { Idle, Tracking, Fault }`
  - `class PoseController` 含：
    - `void configure(const AppConfig &cfg)`
    - `void setTarget(const Pose &t)` / `Pose target() const`
    - `void resetToActual(const Pose &actual)` — 目标置为实际、累积清零、状态回 Idle
    - `Pose step(const Pose &actual)` — 返回本周期增量；非 Tracking 时返回零增量
    - `void setTracking(bool on)`
    - `TrackState state() const`
    - `Pose accumulated() const`
    - `QString faultReason() const`

- [ ] **Step 1: 写失败测试 `tests/test_pose_controller.cpp`**

```cpp
#include <QtTest>
#include "core/PoseController.h"

namespace {

AppConfig testCfg()
{
    AppConfig c = AppConfig::defaults();
    c.cycleMs            = 12.0;
    c.kpPos              = 1.0;    // 便于算术验证
    c.kpRot              = 1.0;
    c.vmaxPosMmS         = 50.0;   // 12ms → 步长上限 0.6mm
    c.vmaxRotDegS        = 10.0;   // 12ms → 步长上限 0.12°
    c.accumLimitPosMm    = 30.0;
    c.accumLimitRotDeg   = 15.0;
    return c;
}

} // namespace

class TestPoseController : public QObject
{
    Q_OBJECT
private slots:
    void notTracking_returnsZeroDelta()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});   // 巨大误差
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        QCOMPARE(d.x, 0.0);                        // 未使能 → 不动
        QCOMPARE(pc.state(), TrackState::Idle);
    }

    void zeroError_producesZeroDelta()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.resetToActual(Pose{10, 20, 30, 1, 2, 3});
        pc.setTracking(true);
        const Pose d = pc.step(Pose{10, 20, 30, 1, 2, 3});
        QCOMPARE(d.x, 0.0);
        QCOMPARE(d.a, 0.0);
    }

    void largeError_isClampedToStepLimit()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 50mm/s * 0.012s = 0.6mm
        QVERIFY(qAbs(d.x - 0.6) < 1e-9);
    }

    void rotationClampUsesSeparateLimit()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 90, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 10deg/s * 0.012s = 0.12deg，且不受位置限值影响
        QVERIFY(qAbs(d.a - 0.12) < 1e-9);
        QCOMPARE(d.x, 0.0);
    }

    void smallError_notClamped()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.kpPos = 0.5;
        pc.configure(c);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0.4, 0, 0, 0, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, 0, 0, 0});
        // 0.5 * 0.4 = 0.2 < 0.6 → 不限幅
        QVERIFY(qAbs(d.x - 0.2) < 1e-9);
    }

    void rotationTakesShortestPath()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.resetToActual(Pose{0, 0, 0, -179, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 179, 0, 0});
        const Pose d = pc.step(Pose{0, 0, 0, -179, 0, 0});
        // 误差 wrap 成 -2° → 向负方向走，而非 +358°
        QVERIFY(d.a < 0.0);
        QVERIFY(qAbs(d.a + 0.12) < 1e-9);
    }

    void accumulation_tracksSumOfDeltas()
    {
        PoseController pc;
        pc.configure(testCfg());
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 3; ++i) {
            const Pose d = pc.step(actual);
            actual.x += d.x;               // 模拟机器人跟随
        }
        QVERIFY(qAbs(pc.accumulated().x - 1.8) < 1e-9);  // 3 * 0.6
    }

    void accumOverLimit_entersFaultAndStopsMoving()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;     // 两步就越限
        pc.configure(c);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});

        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i) {
            const Pose d = pc.step(actual);
            actual.x += d.x;
        }
        QCOMPARE(pc.state(), TrackState::Fault);
        QVERIFY(!pc.faultReason().isEmpty());
        // Fault 后必须返回零增量
        QCOMPARE(pc.step(actual).x, 0.0);
        QVERIFY(qAbs(pc.accumulated().x) <= 1.0 + 1e-9);
    }

    void rotationAccumHasOwnLimit()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitRotDeg = 0.2;    // 姿态先越限
        c.accumLimitPosMm  = 1000.0; // 位置不越限
        pc.configure(c);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{0, 0, 0, 90, 0, 0});

        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i) {
            const Pose d = pc.step(actual);
            actual.a += d.a;
        }
        QCOMPARE(pc.state(), TrackState::Fault);
    }

    void resetToActual_clearsFaultAndAccum()
    {
        PoseController pc;
        AppConfig c = testCfg();
        c.accumLimitPosMm = 1.0;
        pc.configure(c);
        pc.resetToActual(Pose{0, 0, 0, 0, 0, 0});
        pc.setTracking(true);
        pc.setTarget(Pose{100, 0, 0, 0, 0, 0});
        Pose actual{0, 0, 0, 0, 0, 0};
        for (int i = 0; i < 5; ++i)
            actual.x += pc.step(actual).x;
        QCOMPARE(pc.state(), TrackState::Fault);

        pc.resetToActual(Pose{7, 8, 9, 0, 0, 0});
        QCOMPARE(pc.state(), TrackState::Idle);
        QCOMPARE(pc.accumulated().x, 0.0);
        QCOMPARE(pc.target().x, 7.0);      // 目标 = 实际，误差归零
    }
};

QTEST_MAIN(TestPoseController)
#include "test_pose_controller.moc"
```

- [ ] **Step 2: 加 target 并运行，确认失败**

追加到 `tests/CMakeLists.txt`：

```cmake
add_executable(test_pose_controller test_pose_controller.cpp)
target_link_libraries(test_pose_controller PRIVATE rsi_core Qt6::Test)
add_test(NAME test_pose_controller COMMAND test_pose_controller)
```

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target test_pose_controller
```
Expected: FAIL —— `core/PoseController.h: No such file or directory`。

- [ ] **Step 3: 写 `src/core/PoseController.h`**

```cpp
#pragma once
#include <QString>
#include "core/AppConfig.h"
#include "core/Pose.h"

enum class TrackState { Idle, Tracking, Fault };

// 纯计算，无 IO、无 Qt 信号槽。可在通信线程内直接调用。
class PoseController
{
public:
    void configure(const AppConfig &cfg);

    void setTarget(const Pose &t) { m_target = t; }
    Pose target() const { return m_target; }

    // 目标置为实际、累积清零、状态回 Idle。
    // 用于「收到首帧」与「停止跟踪」两处，保证误差瞬时归零。
    void resetToActual(const Pose &actual);

    void setTracking(bool on);

    // 计算本周期增量。非 Tracking 状态一律返回零增量，
    // 但调用方仍必须把结果回包给 KRC。
    Pose step(const Pose &actual);

    TrackState state() const { return m_state; }
    Pose accumulated() const { return m_accum; }
    QString faultReason() const { return m_faultReason; }

private:
    AppConfig  m_cfg = AppConfig::defaults();
    Pose       m_target;
    Pose       m_accum;
    TrackState m_state = TrackState::Idle;
    QString    m_faultReason;

    double m_stepLimitPos = 0.0;   // mm  / 周期
    double m_stepLimitRot = 0.0;   // deg / 周期
};
```

- [ ] **Step 4: 实现 `src/core/PoseController.cpp`**

```cpp
#include "core/PoseController.h"

#include <algorithm>
#include <cmath>

namespace {

double clampAbs(double v, double limit)
{
    return std::clamp(v, -limit, limit);
}

} // namespace

void PoseController::configure(const AppConfig &cfg)
{
    m_cfg = cfg;
    const double cycleS = cfg.cycleMs / 1000.0;
    m_stepLimitPos = cfg.vmaxPosMmS * cycleS;
    m_stepLimitRot = cfg.vmaxRotDegS * cycleS;
}

void PoseController::resetToActual(const Pose &actual)
{
    m_target = actual;
    m_accum  = Pose{};
    m_state  = TrackState::Idle;
    m_faultReason.clear();
}

void PoseController::setTracking(bool on)
{
    if (on) {
        // Fault 必须先经 resetToActual 清除，不能直接重新使能
        if (m_state == TrackState::Idle)
            m_state = TrackState::Tracking;
    } else if (m_state == TrackState::Tracking) {
        m_state = TrackState::Idle;
    }
}

Pose PoseController::step(const Pose &actual)
{
    if (m_state != TrackState::Tracking)
        return Pose{};

    // 误差：位置直接相减，姿态取最短角路径
    const Pose err = poseSub(m_target, actual);

    // 第 1 层限值：单周期增量
    Pose d;
    d.x = clampAbs(m_cfg.kpPos * err.x, m_stepLimitPos);
    d.y = clampAbs(m_cfg.kpPos * err.y, m_stepLimitPos);
    d.z = clampAbs(m_cfg.kpPos * err.z, m_stepLimitPos);
    d.a = clampAbs(m_cfg.kpRot * err.a, m_stepLimitRot);
    d.b = clampAbs(m_cfg.kpRot * err.b, m_stepLimitRot);
    d.c = clampAbs(m_cfg.kpRot * err.c, m_stepLimitRot);

    // 第 2 层限值：累积修正量。越限则转 Fault 并停止累加。
    const Pose next{
        m_accum.x + d.x, m_accum.y + d.y, m_accum.z + d.z,
        m_accum.a + d.a, m_accum.b + d.b, m_accum.c + d.c,
    };

    const double posMax = std::max({std::fabs(next.x), std::fabs(next.y),
                                    std::fabs(next.z)});
    const double rotMax = std::max({std::fabs(next.a), std::fabs(next.b),
                                    std::fabs(next.c)});

    if (posMax > m_cfg.accumLimitPosMm) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral(
            "accumulated translation %1 mm exceeds limit %2 mm")
            .arg(posMax, 0, 'f', 3)
            .arg(m_cfg.accumLimitPosMm, 0, 'f', 3);
        return Pose{};
    }
    if (rotMax > m_cfg.accumLimitRotDeg) {
        m_state = TrackState::Fault;
        m_faultReason = QStringLiteral(
            "accumulated rotation %1 deg exceeds limit %2 deg")
            .arg(rotMax, 0, 'f', 3)
            .arg(m_cfg.accumLimitRotDeg, 0, 'f', 3);
        return Pose{};
    }

    m_accum = next;
    return d;
}
```

- [ ] **Step 5: 运行测试，确认通过**

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target test_pose_controller
./build/tests/test_pose_controller.exe
```
Expected: PASS，10 个用例全过。

- [ ] **Step 6: 提交**

```bash
git add src/core/PoseController.h src/core/PoseController.cpp \
        tests/test_pose_controller.cpp tests/CMakeLists.txt
git commit -m "feat(core): add PoseController with clamped P control and accumulation limits"
```

---

### Task 6: SharedState —— 线程间快照与环形缓冲

**Files:**
- Create: `src/net/SharedState.h`

**Interfaces:**
- Consumes: `Pose`（Task 2）, `TrackState`（Task 5）
- Produces:
  - `struct StatusSnapshot { Pose actual, target, error, accum; quint64 ipoc; TrackState state; QString faultReason; int missedCount; double measuredCycleMs; double maxReplyUs; quint64 frameCount; bool connected; }`
  - `class SharedState` 含 `void publish(const StatusSnapshot &)` / `StatusSnapshot snapshot() const`
  - `struct ChartSample { double tSec; double posErrNorm; double rotErrNorm; }`
  - `class SampleRing` 含 `void push(const ChartSample &)` / `int copyOut(ChartSample *dst, int maxCount) const` / `void clear()`

`SampleRing` 使用固定容量数组，**push 路径无任何动态分配**（Global Constraints 要求）。

- [ ] **Step 1: 写 `src/net/SharedState.h`**

```cpp
#pragma once
#include <QMutex>
#include <QMutexLocker>
#include <QString>
#include <array>
#include "core/PoseController.h"
#include "core/Pose.h"

struct StatusSnapshot
{
    Pose       actual;
    Pose       target;
    Pose       error;
    Pose       accum;
    quint64    ipoc            = 0;
    TrackState state           = TrackState::Idle;
    QString    faultReason;
    int        missedCount     = 0;
    double     measuredCycleMs = 0.0;
    double     maxReplyUs      = 0.0;
    quint64    frameCount      = 0;
    bool       connected       = false;
};

// 通信线程 publish，GUI 线程 snapshot。锁持有时间仅够一次结构体拷贝。
class SharedState
{
public:
    void publish(const StatusSnapshot &s)
    {
        QMutexLocker lock(&m_mutex);
        m_snap = s;
    }

    StatusSnapshot snapshot() const
    {
        QMutexLocker lock(&m_mutex);
        return m_snap;
    }

private:
    mutable QMutex m_mutex;
    StatusSnapshot m_snap;
};

struct ChartSample
{
    double tSec       = 0.0;
    double posErrNorm = 0.0;   // mm
    double rotErrNorm = 0.0;   // deg
};

// 定容环形缓冲：push 无分配，可在实时路径调用。
class SampleRing
{
public:
    static constexpr int kCapacity = 4096;

    void push(const ChartSample &s)
    {
        QMutexLocker lock(&m_mutex);
        m_buf[m_head] = s;
        m_head = (m_head + 1) % kCapacity;
        if (m_size < kCapacity)
            ++m_size;
    }

    // 按时间先后写入 dst，返回实际写入数量。
    int copyOut(ChartSample *dst, int maxCount) const
    {
        QMutexLocker lock(&m_mutex);
        const int n = std::min(m_size, maxCount);
        const int start = (m_head - n + kCapacity) % kCapacity;
        for (int i = 0; i < n; ++i)
            dst[i] = m_buf[(start + i) % kCapacity];
        return n;
    }

    void clear()
    {
        QMutexLocker lock(&m_mutex);
        m_head = 0;
        m_size = 0;
    }

private:
    mutable QMutex m_mutex;
    std::array<ChartSample, kCapacity> m_buf{};
    int m_head = 0;
    int m_size = 0;
};
```

- [ ] **Step 2: 确认可编译**

把 `src/net/SharedState.h` 纳入 `rsi_core` 的头文件搜索路径（已由 `target_include_directories(rsi_core PUBLIC src)` 覆盖，无需改 CMake）。

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build
```
Expected: 构建成功，无 error。

- [ ] **Step 3: 提交**

```bash
git add src/net/SharedState.h
git commit -m "feat(net): add SharedState snapshot and allocation-free SampleRing"
```

---

### Task 7: RsiWorker —— UDP 收发与通信线程

**Files:**
- Create: `src/net/RsiWorker.h`
- Modify: `src/net/RsiWorker.cpp`, `CMakeLists.txt`

**Interfaces:**
- Consumes: `RsiCodec`（Task 4）, `PoseController`（Task 5）, `SharedState`/`SampleRing`（Task 6）, `AppConfig`（Task 3）
- Produces:
  - `class RsiWorker : public QObject` 含：
    - `RsiWorker(const AppConfig &cfg, SharedState *state, SampleRing *ring, QObject *parent = nullptr)`
    - slots: `void start()`, `void stop()`, `void applyTarget(Pose t)`, `void setTracking(bool on)`, `void resetToActual()`, `void applyConfig(AppConfig cfg)`
    - signals: `void bindFailed(QString reason)`, `void listening()`, `void firstFrameReceived()`
- 关键：`onDatagram()` 内**必须**在所有分支回包，包括解析失败分支。

- [ ] **Step 1: 写 `src/net/RsiWorker.h`**

```cpp
#pragma once
#include <QElapsedTimer>
#include <QHostAddress>
#include <QObject>
#include <QTimer>
#include <QUdpSocket>
#include "core/AppConfig.h"
#include "core/PoseController.h"
#include "net/SharedState.h"

// 运行在独立通信线程。绝不触碰 GUI 对象，绝不做文件 IO。
class RsiWorker : public QObject
{
    Q_OBJECT
public:
    RsiWorker(const AppConfig &cfg, SharedState *state, SampleRing *ring,
              QObject *parent = nullptr);

public slots:
    void start();
    void stop();
    void applyTarget(Pose t);
    void setTracking(bool on);
    void resetToActual();
    void applyConfig(AppConfig cfg);

signals:
    void bindFailed(QString reason);
    void listening();
    void firstFrameReceived();

private slots:
    void onDatagram();
    void onWatchdog();

private:
    void publishSnapshot(const Pose &actual, const Pose &err,
                         quint64 ipoc, bool connected);

    AppConfig      m_cfg;
    SharedState   *m_state = nullptr;
    SampleRing    *m_ring  = nullptr;
    QUdpSocket    *m_sock  = nullptr;
    QTimer        *m_watchdog = nullptr;
    PoseController m_ctl;

    QHostAddress m_peerAddr;
    quint16      m_peerPort = 0;

    bool    m_haveFirstFrame = false;
    quint64 m_lastIpoc       = 0;
    int     m_missed         = 0;
    quint64 m_frameCount     = 0;
    double  m_maxReplyUs     = 0.0;

    QElapsedTimer m_sessionTimer;   // 曲线时间轴
    QElapsedTimer m_cycleTimer;     // 实测周期
    bool          m_cycleTimerValid = false;
    double        m_measuredCycleMs = 0.0;
};
```

- [ ] **Step 2: 实现 `src/net/RsiWorker.cpp`**

```cpp
#include "net/RsiWorker.h"

#include <QNetworkDatagram>
#include <algorithm>
#include <cmath>
#include "core/RsiCodec.h"

RsiWorker::RsiWorker(const AppConfig &cfg, SharedState *state,
                     SampleRing *ring, QObject *parent)
    : QObject(parent), m_cfg(cfg), m_state(state), m_ring(ring)
{
    m_ctl.configure(cfg);
}

void RsiWorker::start()
{
    if (m_sock)
        return;

    m_sock = new QUdpSocket(this);
    const QHostAddress addr(m_cfg.listenIp);
    if (!m_sock->bind(addr, m_cfg.listenPort)) {
        const QString why = QStringLiteral("bind %1:%2 failed: %3")
                                .arg(m_cfg.listenIp)
                                .arg(m_cfg.listenPort)
                                .arg(m_sock->errorString());
        delete m_sock;
        m_sock = nullptr;
        emit bindFailed(why);
        return;
    }
    connect(m_sock, &QUdpSocket::readyRead,
            this, &RsiWorker::onDatagram);

    m_watchdog = new QTimer(this);
    // 看门狗周期取通信周期的 20 倍，最少 200ms
    m_watchdog->setInterval(
        std::max(200, int(m_cfg.cycleMs * 20.0)));
    connect(m_watchdog, &QTimer::timeout,
            this, &RsiWorker::onWatchdog);
    m_watchdog->start();

    m_sessionTimer.start();
    m_haveFirstFrame  = false;
    m_frameCount      = 0;
    m_missed          = 0;
    m_maxReplyUs      = 0.0;
    m_cycleTimerValid = false;

    emit listening();
}

void RsiWorker::stop()
{
    if (m_watchdog) {
        m_watchdog->stop();
        m_watchdog->deleteLater();
        m_watchdog = nullptr;
    }
    if (m_sock) {
        m_sock->close();
        m_sock->deleteLater();
        m_sock = nullptr;
    }
    m_haveFirstFrame = false;
    StatusSnapshot s;
    s.connected = false;
    m_state->publish(s);
}

void RsiWorker::applyTarget(Pose t)      { m_ctl.setTarget(t); }
void RsiWorker::setTracking(bool on)     { m_ctl.setTracking(on); }

void RsiWorker::applyConfig(AppConfig cfg)
{
    m_cfg = cfg;
    m_ctl.configure(cfg);
}

void RsiWorker::resetToActual()
{
    m_ctl.resetToActual(m_state->snapshot().actual);
}

void RsiWorker::onWatchdog()
{
    // 长时间无包：视为 RSI 已停止，退回未连接
    if (!m_haveFirstFrame)
        return;
    m_haveFirstFrame = false;
    m_ring->clear();
    StatusSnapshot s = m_state->snapshot();
    s.connected = false;
    m_state->publish(s);
}

void RsiWorker::onDatagram()
{
    while (m_sock && m_sock->hasPendingDatagrams()) {
        QElapsedTimer replyTimer;
        replyTimer.start();

        const QNetworkDatagram dg = m_sock->receiveDatagram();
        m_peerAddr = dg.senderAddress();
        m_peerPort = quint16(dg.senderPort());

        const RobFrame f = RsiCodec::parseRob(dg.data());

        // ── 无论解析成败，都必须回包 ──
        quint64 echoIpoc = f.valid ? f.ipoc : m_lastIpoc;
        Pose    delta;   // 默认零增量

        if (f.valid) {
            if (m_haveFirstFrame) {
                // IPOC 应单调递增；否则计一次丢包
                if (f.ipoc <= m_lastIpoc)
                    ++m_missed;
            } else {
                // 首帧：目标置为实际，误差归零，机器人原地不动
                m_ctl.resetToActual(f.rist);
                m_haveFirstFrame = true;
                emit firstFrameReceived();
            }

            if (m_cycleTimerValid) {
                m_measuredCycleMs = m_cycleTimer.nsecsElapsed() / 1.0e6;
            }
            m_cycleTimer.start();
            m_cycleTimerValid = true;

            m_lastIpoc = f.ipoc;
            ++m_frameCount;

            delta = m_ctl.step(f.rist);
        } else {
            ++m_missed;
        }

        const QByteArray sen =
            RsiCodec::buildSen(delta, echoIpoc, m_cfg.senType);
        m_sock->writeDatagram(sen, m_peerAddr, m_peerPort);

        const double replyUs = replyTimer.nsecsElapsed() / 1000.0;
        m_maxReplyUs = std::max(m_maxReplyUs, replyUs);

        if (m_missed >= m_cfg.watchdogMissLimit &&
            m_ctl.state() == TrackState::Tracking) {
            m_ctl.setTracking(false);
        }

        if (f.valid) {
            const Pose err = poseSub(m_ctl.target(), f.rist);
            publishSnapshot(f.rist, err, f.ipoc, true);

            ChartSample cs;
            cs.tSec = m_sessionTimer.nsecsElapsed() / 1.0e9;
            cs.posErrNorm = std::sqrt(err.x * err.x + err.y * err.y +
                                      err.z * err.z);
            cs.rotErrNorm = std::max({std::fabs(err.a), std::fabs(err.b),
                                      std::fabs(err.c)});
            m_ring->push(cs);
        }

        m_watchdog->start();   // 收到包就重置看门狗
    }
}

void RsiWorker::publishSnapshot(const Pose &actual, const Pose &err,
                                quint64 ipoc, bool connected)
{
    StatusSnapshot s;
    s.actual          = actual;
    s.target          = m_ctl.target();
    s.error           = err;
    s.accum           = m_ctl.accumulated();
    s.ipoc            = ipoc;
    s.state           = m_ctl.state();
    s.faultReason     = m_ctl.faultReason();
    s.missedCount     = m_missed;
    s.measuredCycleMs = m_measuredCycleMs;
    s.maxReplyUs      = m_maxReplyUs;
    s.frameCount      = m_frameCount;
    s.connected       = connected;
    m_state->publish(s);
}
```

- [ ] **Step 3: 无需改动 CMake**

`rsi_net` 静态库已在 Task 1 的顶层 `CMakeLists.txt` 中建立并包含 `src/net/RsiWorker.cpp`，本任务只是填充其实现。`SharedState.h` 仅依赖 `QMutex`（Qt6::Core），已由 `rsi_core` 覆盖。

- [ ] **Step 4: 构建确认**

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build
```
Expected: 构建成功，`build/rsi_host.exe` 更新，无 error。

- [ ] **Step 5: 提交**

```bash
git add src/net/RsiWorker.h src/net/RsiWorker.cpp
git commit -m "feat(net): add RsiWorker with always-reply UDP loop and watchdog"
```

---

### Task 8: KRC 模拟器

**Files:**
- Create: `tools/krc_simulator/main.cpp`
- Modify: `tools/krc_simulator/CMakeLists.txt`

**Interfaces:**
- Consumes: `RsiCodec`（Task 4）用于生成 `<Rob>` 与校验 `<Sen>`
- Produces: 可执行 `krc_simulator`，命令行参数 `--host <ip> --port <n> --cycle-ms <d> --cycles <n>`；退出码 0 表示回包率 100% 且 IPOC 全部正确

模拟器扮演 KRC：按周期发 `<Rob>`、收 `<Sen>`，把收到的 `RKorr` 累加到自身位姿上（模拟 RELATIVE 语义），最后打印统计。

- [ ] **Step 1: 写 `tools/krc_simulator/main.cpp`**

```cpp
// 假 KRC：按固定周期发 <Rob>、收 <Sen>，验证上位机的实时行为。
// 把收到的 RKorr 累加到自身位姿，模拟 RELATIVE 修正语义。
#include <QCoreApplication>
#include <QCommandLineParser>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QUdpSocket>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "core/Pose.h"
#include "core/RsiCodec.h"

namespace {

QByteArray buildRob(const Pose &p, quint64 ipoc)
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
    s += "/>\n<Delay D=\"0\"/>\n<IPOC>";
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
    p.addOptions({oHost, oPort, oCycle, oCount});
    p.process(app);

    const QHostAddress host(p.value(oHost));
    const quint16 port  = quint16(p.value(oPort).toUShort());
    const double cycleMs = p.value(oCycle).toDouble();
    const int cycles     = p.value(oCount).toInt();

    QUdpSocket sock;
    if (!sock.bind(QHostAddress::AnyIPv4, 0)) {
        std::fprintf(stderr, "simulator bind failed: %s\n",
                     qPrintable(sock.errorString()));
        return 2;
    }

    Pose pose{1250.0, 0.0, 1000.0, 0.0, 90.0, 0.0};
    quint64 ipoc = 1000;

    int replies = 0, ipocMismatch = 0, timeouts = 0;
    double maxRttUs = 0.0, sumRttUs = 0.0;

    for (int i = 0; i < cycles; ++i) {
        QElapsedTimer rtt;
        rtt.start();
        sock.writeDatagram(buildRob(pose, ipoc), host, port);

        // 等待本周期内的回包
        const int budgetMs = std::max(1, int(cycleMs));
        if (!sock.waitForReadyRead(budgetMs)) {
            ++timeouts;
        } else {
            while (sock.hasPendingDatagrams()) {
                const QByteArray d = sock.receiveDatagram().data();
                Pose korr;
                quint64 echoed = 0;
                if (parseSen(d, &korr, &echoed)) {
                    ++replies;
                    if (echoed != ipoc)
                        ++ipocMismatch;
                    // RELATIVE：增量累加到当前位姿
                    pose.x += korr.x; pose.y += korr.y; pose.z += korr.z;
                    pose.a = wrap180(pose.a + korr.a);
                    pose.b = wrap180(pose.b + korr.b);
                    pose.c = wrap180(pose.c + korr.c);
                }
            }
            const double us = rtt.nsecsElapsed() / 1000.0;
            maxRttUs = std::max(maxRttUs, us);
            sumRttUs += us;
        }
        ++ipoc;
    }

    std::printf("cycles=%d replies=%d timeouts=%d ipoc_mismatch=%d\n",
                cycles, replies, timeouts, ipocMismatch);
    std::printf("rtt_avg_us=%.1f rtt_max_us=%.1f\n",
                replies ? sumRttUs / replies : 0.0, maxRttUs);
    std::printf("final_pose X=%.3f Y=%.3f Z=%.3f A=%.3f B=%.3f C=%.3f\n",
                pose.x, pose.y, pose.z, pose.a, pose.b, pose.c);

    const bool pass = (replies == cycles) && (ipocMismatch == 0);
    std::printf("%s\n", pass ? "PASS" : "FAIL");
    return pass ? 0 : 1;
}
```

- [ ] **Step 2: 写 `tools/krc_simulator/CMakeLists.txt`**

```cmake
add_executable(krc_simulator main.cpp)
target_link_libraries(krc_simulator PRIVATE rsi_core Qt6::Core Qt6::Network)
```

- [ ] **Step 3: 构建**

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target krc_simulator
```
Expected: 构建成功，产出 `build/tools/krc_simulator/krc_simulator.exe`。

- [ ] **Step 4: 提交**

```bash
git add tools/krc_simulator/main.cpp tools/krc_simulator/CMakeLists.txt
git commit -m "test(tools): add KRC simulator for offline real-time verification"
```

---

### Task 9: 端到端实时性验证（无需虚拟机）

**Files:**
- Create: `tools/loopback_test/main.cpp`, `tools/loopback_test/CMakeLists.txt`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `RsiWorker`（Task 7）, `SharedState`/`SampleRing`（Task 6）, `AppConfig`（Task 3）
- Produces: 可执行 `loopback_test` —— 在本机 `127.0.0.1` 上把 `RsiWorker` 跑在真实线程里，供 `krc_simulator` 打靶

这一步验证的是**整条实时链路**：线程切换、socket 收发、闭环计算、回包及时性。它不依赖 OfficeLite。

- [ ] **Step 1: 写 `tools/loopback_test/main.cpp`**

```cpp
// 把 RsiWorker 跑在真实通信线程上，监听 127.0.0.1，
// 供 krc_simulator 打靶，用于端到端实时性验证。
#include <QCoreApplication>
#include <QCommandLineParser>
#include <QThread>
#include <QTimer>
#include <cstdio>
#include "core/AppConfig.h"
#include "net/RsiWorker.h"
#include "net/SharedState.h"

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    QCommandLineParser p;
    p.addHelpOption();
    QCommandLineOption oPort("port", "listen port", "n", "59152");
    QCommandLineOption oSecs("seconds", "run duration", "s", "10");
    QCommandLineOption oTrack("track", "enable tracking with X offset",
                              "mm", "0");
    p.addOptions({oPort, oSecs, oTrack});
    p.process(app);

    AppConfig cfg = AppConfig::defaults();
    cfg.listenIp   = "127.0.0.1";
    cfg.listenPort = quint16(p.value(oPort).toUShort());

    SharedState state;
    SampleRing  ring;

    QThread commThread;
    auto *worker = new RsiWorker(cfg, &state, &ring);
    worker->moveToThread(&commThread);

    QObject::connect(&commThread, &QThread::started,
                     worker, &RsiWorker::start);
    QObject::connect(worker, &RsiWorker::bindFailed,
                     [](const QString &why) {
                         std::fprintf(stderr, "bind failed: %s\n",
                                      qPrintable(why));
                         QCoreApplication::exit(2);
                     });
    QObject::connect(worker, &RsiWorker::listening, [] {
        std::printf("listening\n");
        std::fflush(stdout);
    });

    const double offset = p.value(oTrack).toDouble();
    QObject::connect(worker, &RsiWorker::firstFrameReceived,
                     [worker, offset, &state] {
        std::printf("first frame received\n");
        std::fflush(stdout);
        if (offset != 0.0) {
            Pose t = state.snapshot().actual;
            t.x += offset;
            QMetaObject::invokeMethod(worker, "applyTarget",
                                      Qt::QueuedConnection,
                                      Q_ARG(Pose, t));
            QMetaObject::invokeMethod(worker, "setTracking",
                                      Qt::QueuedConnection,
                                      Q_ARG(bool, true));
        }
    });

    commThread.start();

    QTimer::singleShot(p.value(oSecs).toInt() * 1000, [&] {
        const StatusSnapshot s = state.snapshot();
        std::printf("frames=%llu missed=%d cycle_ms=%.2f "
                    "max_reply_us=%.1f\n",
                    static_cast<unsigned long long>(s.frameCount),
                    s.missedCount, s.measuredCycleMs, s.maxReplyUs);
        std::printf("accum X=%.3f  err X=%.3f\n",
                    s.accum.x, s.error.x);
        QMetaObject::invokeMethod(worker, "stop",
                                  Qt::BlockingQueuedConnection);
        commThread.quit();
        commThread.wait(2000);
        QCoreApplication::quit();
    });

    return app.exec();
}
```

- [ ] **Step 2: 写 `tools/loopback_test/CMakeLists.txt` 并注册到顶层**

`tools/loopback_test/CMakeLists.txt`：

```cmake
add_executable(loopback_test main.cpp)
target_link_libraries(loopback_test PRIVATE rsi_net Qt6::Core Qt6::Network)
```

在顶层 `CMakeLists.txt` 末尾追加：

```cmake
add_subdirectory(tools/loopback_test)
```

- [ ] **Step 3: 构建两个工具**

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" -S . -B build -G Ninja \
  -DCMAKE_PREFIX_PATH="$QTDIR" -DCMAKE_BUILD_TYPE=Debug
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build
```
Expected: 构建成功，产出 `build/tools/loopback_test/loopback_test.exe`。

- [ ] **Step 4: 跑基线验证（不使能跟踪）**

启动接收端（后台），再打靶：

```bash
cd /e/kuka_rsi_win
PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH" \
  ./build/tools/loopback_test/loopback_test.exe --port 59152 --seconds 12 &
sleep 2
PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH" \
  ./build/tools/krc_simulator/krc_simulator.exe \
  --host 127.0.0.1 --port 59152 --cycle-ms 12 --cycles 500
wait
```

Expected:
- 模拟器打印 `cycles=500 replies=500 timeouts=0 ipoc_mismatch=0` 与 `PASS`，退出码 0
- `loopback_test` 打印 `missed=0`，`max_reply_us` 应远小于 12000（预期数百微秒量级）
- 未使能跟踪，`final_pose` 与初值一致（增量恒为 0）

若 `replies != cycles` 或 `ipoc_mismatch != 0`，**不要继续后续任务** —— 实时链路不可靠，联机必然失败。

- [ ] **Step 5: 跑闭环收敛验证（使能跟踪，X 偏移 5mm）**

```bash
cd /e/kuka_rsi_win
PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH" \
  ./build/tools/loopback_test/loopback_test.exe \
  --port 59153 --seconds 12 --track 5 &
sleep 2
PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH" \
  ./build/tools/krc_simulator/krc_simulator.exe \
  --host 127.0.0.1 --port 59153 --cycle-ms 12 --cycles 500
wait
```

Expected:
- `PASS`，`ipoc_mismatch=0`
- 模拟器的 `final_pose` 中 `X` 收敛到约 `1255.000`（初值 1250 + 目标偏移 5）
- `loopback_test` 输出 `err X` 接近 0，`accum X` 接近 5.000
- 累积 5mm 未触及 30mm 限值，状态不应进 Fault

- [ ] **Step 6: 提交**

```bash
git add tools/loopback_test CMakeLists.txt
git commit -m "test(tools): add loopback harness verifying reply rate and convergence"
```

---

### Task 10: MainWindow —— 布局与数值显示

**Files:**
- Create: `src/ui/MainWindow.h`
- Modify: `src/ui/MainWindow.cpp`, `src/main.cpp`

**Interfaces:**
- Consumes: `AppConfig`（Task 3）, `SharedState`/`SampleRing`（Task 6）, `RsiWorker`（Task 7）
- Produces: `class MainWindow : public QMainWindow`，构造签名 `MainWindow(const AppConfig &cfg, QWidget *parent = nullptr)`；内部自建通信线程与 worker

本任务只做数值与布局；曲线在 Task 11 接入，状态机按钮在 Task 12 完成。

- [ ] **Step 1: 写 `src/ui/MainWindow.h`**

```cpp
#pragma once
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMainWindow>
#include <QSlider>
#include <QThread>
#include <QTimer>
#include <array>
#include "core/AppConfig.h"
#include "net/RsiWorker.h"
#include "net/SharedState.h"

class ErrorChart;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(const AppConfig &cfg, QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    void onRefresh();
    void onTargetEdited();

private:
    QWidget *buildTargetPanel();
    QWidget *buildReadoutPanel();
    QWidget *buildParamPanel();

    AppConfig    m_cfg;
    SharedState  m_state;
    SampleRing   m_ring;
    QThread     *m_commThread = nullptr;
    RsiWorker   *m_worker     = nullptr;
    QTimer      *m_refresh    = nullptr;

    // 目标位姿输入：6 个滑块 + 6 个数值框联动
    std::array<QSlider *, 6>        m_targetSlider{};
    std::array<QDoubleSpinBox *, 6> m_targetSpin{};

    // 读数：当前位姿 / 误差 / 累积
    std::array<QLabel *, 6> m_actualLabel{};
    std::array<QLabel *, 6> m_errorLabel{};
    std::array<QLabel *, 6> m_accumLabel{};

    QLabel *m_statusLabel = nullptr;
    ErrorChart *m_chart   = nullptr;

    bool m_suppressTargetSignal = false;
};
```

- [ ] **Step 2: 实现 `src/ui/MainWindow.cpp`（布局 + 刷新，暂不接曲线）**

```cpp
#include "ui/MainWindow.h"

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <cmath>

namespace {

const char *kAxisName[6] = {"X", "Y", "Z", "A", "B", "C"};

// 位置 ±4000mm，姿态 ±180°
double axisMin(int i) { return i < 3 ? -4000.0 : -180.0; }
double axisMax(int i) { return i < 3 ?  4000.0 :  180.0; }
const char *axisUnit(int i) { return i < 3 ? " mm" : " °"; }

} // namespace

MainWindow::MainWindow(const AppConfig &cfg, QWidget *parent)
    : QMainWindow(parent), m_cfg(cfg)
{
    setWindowTitle("KUKA RSI POSCORR 位姿跟踪");

    m_statusLabel = new QLabel("未连接", this);
    m_statusLabel->setStyleSheet("font-weight: bold;");

    auto *central = new QWidget(this);
    auto *outer = new QVBoxLayout(central);
    outer->addWidget(m_statusLabel);

    auto *row = new QHBoxLayout;
    auto *left = new QVBoxLayout;
    left->addWidget(buildTargetPanel());
    left->addWidget(buildParamPanel());
    left->addStretch();
    row->addLayout(left, 1);
    row->addWidget(buildReadoutPanel(), 1);
    outer->addLayout(row);

    setCentralWidget(central);

    // 通信线程
    m_commThread = new QThread(this);
    m_worker = new RsiWorker(m_cfg, &m_state, &m_ring);
    m_worker->moveToThread(m_commThread);
    connect(m_commThread, &QThread::started,
            m_worker, &RsiWorker::start);
    connect(m_worker, &RsiWorker::bindFailed,
            this, [this](const QString &why) {
                QMessageBox::critical(this, "监听失败", why);
            });
    connect(m_worker, &RsiWorker::firstFrameReceived,
            this, [this] {
                // 首帧到达：把界面目标同步为机器人实际位姿
                const Pose a = m_state.snapshot().actual;
                m_suppressTargetSignal = true;
                const double v[6] = {a.x, a.y, a.z, a.a, a.b, a.c};
                for (int i = 0; i < 6; ++i) {
                    m_targetSpin[i]->setValue(v[i]);
                    m_targetSlider[i]->setValue(int(v[i] * 10.0));
                }
                m_suppressTargetSignal = false;
            });
    m_commThread->start();

    m_refresh = new QTimer(this);
    m_refresh->setInterval(m_cfg.refreshMs);
    connect(m_refresh, &QTimer::timeout, this, &MainWindow::onRefresh);
    m_refresh->start();

    resize(980, 620);
}

MainWindow::~MainWindow()
{
    if (m_commThread) {
        QMetaObject::invokeMethod(m_worker, "stop",
                                  Qt::BlockingQueuedConnection);
        m_commThread->quit();
        m_commThread->wait(2000);
    }
}

QWidget *MainWindow::buildTargetPanel()
{
    auto *box = new QGroupBox("目标位姿 (BASE)", this);
    auto *grid = new QGridLayout(box);

    for (int i = 0; i < 6; ++i) {
        grid->addWidget(new QLabel(kAxisName[i], box), i, 0);

        auto *sl = new QSlider(Qt::Horizontal, box);
        sl->setRange(int(axisMin(i) * 10.0), int(axisMax(i) * 10.0));
        grid->addWidget(sl, i, 1);
        m_targetSlider[i] = sl;

        auto *sp = new QDoubleSpinBox(box);
        sp->setRange(axisMin(i), axisMax(i));
        sp->setDecimals(2);
        sp->setSingleStep(0.5);
        sp->setSuffix(axisUnit(i));
        sp->setKeyboardTracking(false);
        grid->addWidget(sp, i, 2);
        m_targetSpin[i] = sp;

        // 滑块与数值框联动（滑块用 0.1 单位整数）
        connect(sl, &QSlider::valueChanged, this, [this, i](int v) {
            if (m_suppressTargetSignal)
                return;
            m_suppressTargetSignal = true;
            m_targetSpin[i]->setValue(v / 10.0);
            m_suppressTargetSignal = false;
            onTargetEdited();
        });
        connect(sp, &QDoubleSpinBox::valueChanged,
                this, [this, i](double v) {
            if (m_suppressTargetSignal)
                return;
            m_suppressTargetSignal = true;
            m_targetSlider[i]->setValue(int(v * 10.0));
            m_suppressTargetSignal = false;
            onTargetEdited();
        });
    }
    return box;
}

QWidget *MainWindow::buildParamPanel()
{
    auto *box = new QGroupBox("控制参数", this);
    auto *grid = new QGridLayout(box);
    grid->addWidget(new QLabel("位置", box), 0, 1);
    grid->addWidget(new QLabel("姿态", box), 0, 2);

    struct Row { const char *name; double pos, rot, step; int dec; };
    const Row rows[3] = {
        {"Kp",       m_cfg.kpPos,           m_cfg.kpRot,            0.01, 2},
        {"限速",     m_cfg.vmaxPosMmS,      m_cfg.vmaxRotDegS,      1.0,  1},
        {"累积上限", m_cfg.accumLimitPosMm, m_cfg.accumLimitRotDeg, 1.0,  1},
    };

    for (int r = 0; r < 3; ++r) {
        grid->addWidget(new QLabel(rows[r].name, box), r + 1, 0);
        for (int c = 0; c < 2; ++c) {
            auto *sp = new QDoubleSpinBox(box);
            sp->setRange(0.0, 100000.0);
            sp->setDecimals(rows[r].dec);
            sp->setSingleStep(rows[r].step);
            sp->setValue(c == 0 ? rows[r].pos : rows[r].rot);
            sp->setKeyboardTracking(false);
            grid->addWidget(sp, r + 1, c + 1);

            connect(sp, &QDoubleSpinBox::valueChanged,
                    this, [this, r, c](double v) {
                AppConfig c2 = m_cfg;
                if (r == 0) (c == 0 ? c2.kpPos : c2.kpRot) = v;
                if (r == 1) (c == 0 ? c2.vmaxPosMmS : c2.vmaxRotDegS) = v;
                if (r == 2) (c == 0 ? c2.accumLimitPosMm
                                    : c2.accumLimitRotDeg) = v;
                m_cfg = c2;
                QMetaObject::invokeMethod(m_worker, "applyConfig",
                                          Qt::QueuedConnection,
                                          Q_ARG(AppConfig, c2));
            });
        }
    }
    return box;
}

QWidget *MainWindow::buildReadoutPanel()
{
    auto *box = new QGroupBox("读数", this);
    auto *grid = new QGridLayout(box);
    grid->addWidget(new QLabel("当前位姿", box), 0, 1);
    grid->addWidget(new QLabel("误差",     box), 0, 2);
    grid->addWidget(new QLabel("累积修正", box), 0, 3);

    for (int i = 0; i < 6; ++i) {
        grid->addWidget(new QLabel(kAxisName[i], box), i + 1, 0);
        m_actualLabel[i] = new QLabel("--", box);
        m_errorLabel[i]  = new QLabel("--", box);
        m_accumLabel[i]  = new QLabel("--", box);
        grid->addWidget(m_actualLabel[i], i + 1, 1);
        grid->addWidget(m_errorLabel[i],  i + 1, 2);
        grid->addWidget(m_accumLabel[i],  i + 1, 3);
    }
    return box;
}

void MainWindow::onTargetEdited()
{
    Pose t;
    t.x = m_targetSpin[0]->value();
    t.y = m_targetSpin[1]->value();
    t.z = m_targetSpin[2]->value();
    t.a = m_targetSpin[3]->value();
    t.b = m_targetSpin[4]->value();
    t.c = m_targetSpin[5]->value();
    QMetaObject::invokeMethod(m_worker, "applyTarget",
                              Qt::QueuedConnection, Q_ARG(Pose, t));
}

void MainWindow::onRefresh()
{
    const StatusSnapshot s = m_state.snapshot();

    const double act[6] = {s.actual.x, s.actual.y, s.actual.z,
                           s.actual.a, s.actual.b, s.actual.c};
    const double err[6] = {s.error.x, s.error.y, s.error.z,
                           s.error.a, s.error.b, s.error.c};
    const double acc[6] = {s.accum.x, s.accum.y, s.accum.z,
                           s.accum.a, s.accum.b, s.accum.c};
    for (int i = 0; i < 6; ++i) {
        m_actualLabel[i]->setText(QString::number(act[i], 'f', 3));
        m_errorLabel[i]->setText(QString::number(err[i], 'f', 3));
        m_accumLabel[i]->setText(QString::number(acc[i], 'f', 3));
    }

    QString st = s.connected ? "● 已连接" : "○ 未连接";
    if (s.state == TrackState::Tracking)
        st += "  跟踪中";
    else if (s.state == TrackState::Fault)
        st += "  故障: " + s.faultReason;
    st += QStringLiteral("   IPOC %1   周期 %2 ms   最大回包 %3 µs"
                         "   丢包 %4")
              .arg(s.ipoc)
              .arg(s.measuredCycleMs, 0, 'f', 1)
              .arg(s.maxReplyUs, 0, 'f', 0)
              .arg(s.missedCount);
    m_statusLabel->setText(st);
}
```

- [ ] **Step 3: 改 `src/main.cpp` 加载配置并开窗**

```cpp
#include <QApplication>
#include <QDir>
#include <QMessageBox>
#include "core/AppConfig.h"
#include "ui/MainWindow.h"

int main(int argc, char **argv)
{
    QApplication app(argc, argv);

    AppConfig cfg = AppConfig::defaults();
    const QString path =
        QDir(QCoreApplication::applicationDirPath())
            .filePath("../config/rsi_config.json");
    QString err;
    if (!AppConfig::loadFromFile(path, &cfg, &err)) {
        QMessageBox::warning(nullptr, "配置",
            QStringLiteral("未能加载 %1\n%2\n\n将使用内置默认值。")
                .arg(path, err));
    }

    MainWindow w(cfg);
    w.show();
    return app.exec();
}
```

- [ ] **Step 4: 加入 ErrorChart 占位以便链接**

`src/ui/ErrorChart.cpp` 暂时保持 `// filled in by a later task`。`MainWindow.cpp` 中 `m_chart` 尚未使用，不会产生链接错误。

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target rsi_host
```
Expected: 构建成功，无 error。

- [ ] **Step 5: 手动验证界面**

```bash
cd /e/kuka_rsi_win
PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH" ./build/rsi_host.exe &
sleep 3
PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH" \
  ./build/tools/krc_simulator/krc_simulator.exe \
  --host 192.168.44.1 --port 59152 --cycle-ms 12 --cycles 300
```

Expected: 窗口显示，状态栏从「○ 未连接」变为「● 已连接」，当前位姿显示 `1250.000 / 0.000 / 1000.000 / 0.000 / 90.000 / 0.000`，误差全为 `0.000`（首帧已把目标同步为实际），模拟器打印 `PASS`。

- [ ] **Step 6: 提交**

```bash
git add src/ui/MainWindow.h src/ui/MainWindow.cpp src/main.cpp
git commit -m "feat(ui): add MainWindow with target inputs and numeric readouts"
```

---

### Task 11: 误差曲线（QtCharts）

**Files:**
- Create: `src/ui/ErrorChart.h`
- Modify: `src/ui/ErrorChart.cpp`, `src/ui/MainWindow.cpp`, `src/ui/MainWindow.h`

**Interfaces:**
- Consumes: `SampleRing`/`ChartSample`（Task 6）, `AppConfig`（Task 3）
- Produces: `class ErrorChart : public QWidget`，构造 `ErrorChart(int windowSeconds, QWidget *parent = nullptr)`，方法 `void updateFrom(const SampleRing &ring)`

- [ ] **Step 1: 写 `src/ui/ErrorChart.h`**

```cpp
#pragma once
#include <QChartView>
#include <QLineSeries>
#include <QValueAxis>
#include <QWidget>
#include "net/SharedState.h"

class ErrorChart : public QWidget
{
    Q_OBJECT
public:
    explicit ErrorChart(int windowSeconds, QWidget *parent = nullptr);

    // 从环形缓冲拉取样本重画。由 GUI 线程调用。
    void updateFrom(const SampleRing &ring);

private:
    int m_windowSeconds = 20;
    QLineSeries *m_posSeries = nullptr;
    QLineSeries *m_rotSeries = nullptr;
    QValueAxis  *m_axisX = nullptr;
    QValueAxis  *m_axisPos = nullptr;
    QValueAxis  *m_axisRot = nullptr;
    QChartView  *m_view = nullptr;
};
```

- [ ] **Step 2: 实现 `src/ui/ErrorChart.cpp`**

```cpp
#include "ui/ErrorChart.h"

#include <QChart>
#include <QVBoxLayout>
#include <algorithm>
#include <vector>

ErrorChart::ErrorChart(int windowSeconds, QWidget *parent)
    : QWidget(parent), m_windowSeconds(std::max(1, windowSeconds))
{
    auto *chart = new QChart;
    chart->setTitle("跟踪误差");
    chart->legend()->setAlignment(Qt::AlignBottom);

    m_posSeries = new QLineSeries;
    m_posSeries->setName("位置误差 mm");
    m_rotSeries = new QLineSeries;
    m_rotSeries->setName("姿态误差 °");

    chart->addSeries(m_posSeries);
    chart->addSeries(m_rotSeries);

    m_axisX = new QValueAxis;
    m_axisX->setTitleText("时间 s");
    chart->addAxis(m_axisX, Qt::AlignBottom);
    m_posSeries->attachAxis(m_axisX);
    m_rotSeries->attachAxis(m_axisX);

    m_axisPos = new QValueAxis;
    m_axisPos->setTitleText("mm");
    chart->addAxis(m_axisPos, Qt::AlignLeft);
    m_posSeries->attachAxis(m_axisPos);

    m_axisRot = new QValueAxis;
    m_axisRot->setTitleText("°");
    chart->addAxis(m_axisRot, Qt::AlignRight);
    m_rotSeries->attachAxis(m_axisRot);

    m_view = new QChartView(chart, this);
    m_view->setRenderHint(QPainter::Antialiasing);

    auto *lay = new QVBoxLayout(this);
    lay->setContentsMargins(0, 0, 0, 0);
    lay->addWidget(m_view);
}

void ErrorChart::updateFrom(const SampleRing &ring)
{
    static std::vector<ChartSample> buf(SampleRing::kCapacity);
    const int n = ring.copyOut(buf.data(), SampleRing::kCapacity);
    if (n == 0) {
        m_posSeries->clear();
        m_rotSeries->clear();
        return;
    }

    const double tEnd = buf[n - 1].tSec;
    const double tStart = std::max(0.0, tEnd - double(m_windowSeconds));

    QList<QPointF> pos, rot;
    pos.reserve(n);
    rot.reserve(n);
    double posMax = 1.0, rotMax = 1.0;
    for (int i = 0; i < n; ++i) {
        if (buf[i].tSec < tStart)
            continue;
        pos.append(QPointF(buf[i].tSec, buf[i].posErrNorm));
        rot.append(QPointF(buf[i].tSec, buf[i].rotErrNorm));
        posMax = std::max(posMax, buf[i].posErrNorm);
        rotMax = std::max(rotMax, buf[i].rotErrNorm);
    }

    m_posSeries->replace(pos);
    m_rotSeries->replace(rot);
    m_axisX->setRange(tStart, std::max(tStart + 1.0, tEnd));
    m_axisPos->setRange(0.0, posMax * 1.2);
    m_axisRot->setRange(0.0, rotMax * 1.2);
}
```

- [ ] **Step 3: 接进 MainWindow**

在 `src/ui/MainWindow.cpp` 顶部增加 include：

```cpp
#include "ui/ErrorChart.h"
```

把 `buildReadoutPanel()` 那一行替换为右侧竖排「曲线 + 读数」。将构造函数中：

```cpp
    row->addWidget(buildReadoutPanel(), 1);
```

改为：

```cpp
    auto *right = new QVBoxLayout;
    m_chart = new ErrorChart(m_cfg.chartWindowS, this);
    right->addWidget(m_chart, 2);
    right->addWidget(buildReadoutPanel(), 1);
    row->addLayout(right, 1);
```

在 `onRefresh()` 末尾追加：

```cpp
    m_chart->updateFrom(m_ring);
```

- [ ] **Step 4: 构建并验证曲线**

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target rsi_host
cd /e/kuka_rsi_win
PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH" ./build/rsi_host.exe &
sleep 3
PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH" \
  ./build/tools/krc_simulator/krc_simulator.exe \
  --host 192.168.44.1 --port 59152 --cycle-ms 12 --cycles 600
```

Expected: 曲线区出现两条曲线；因目标已同步为实际、且未使能跟踪，两条误差曲线应贴近 0。

- [ ] **Step 5: 提交**

```bash
git add src/ui/ErrorChart.h src/ui/ErrorChart.cpp src/ui/MainWindow.cpp \
        src/ui/MainWindow.h
git commit -m "feat(ui): add QtCharts error chart with rolling time window"
```

---

### Task 12: 状态机与控制按钮

**Files:**
- Modify: `src/ui/MainWindow.h`, `src/ui/MainWindow.cpp`

**Interfaces:**
- Consumes: `RsiWorker` slots `setTracking`/`resetToActual`（Task 7）
- Produces: 界面按钮 `归零到当前位姿`、`使能跟踪`（可勾选）、`停止跟踪`；以及 TOOL/BASE 提示标签

**「停止跟踪」不是急停** —— 它把目标拉回实际使误差归零、机器人停在原地，但仍持续回包。此说明必须出现在界面上（Global Constraints 与设计 §7.4）。

- [ ] **Step 1: 在 `MainWindow.h` 增加成员与槽**

在 `private slots:` 区加入：

```cpp
    void onZeroToActual();
    void onTrackingToggled(bool on);
    void onStopTracking();
```

在文件顶部的 include 区加入：

```cpp
#include <QCheckBox>
#include <QPushButton>
```

在私有成员区加入：

```cpp
    QCheckBox *m_trackCheck = nullptr;
    QLabel    *m_safetyNote = nullptr;
```

- [ ] **Step 2: 在 `MainWindow.cpp` 构造函数中加按钮栏**

在 `outer->addLayout(row);` 之后、`setCentralWidget(central);` 之前插入：

```cpp
    auto *bar = new QHBoxLayout;

    auto *zeroBtn = new QPushButton("归零到当前位姿", this);
    connect(zeroBtn, &QPushButton::clicked,
            this, &MainWindow::onZeroToActual);
    bar->addWidget(zeroBtn);

    m_trackCheck = new QCheckBox("使能跟踪", this);
    connect(m_trackCheck, &QCheckBox::toggled,
            this, &MainWindow::onTrackingToggled);
    bar->addWidget(m_trackCheck);

    auto *stopBtn = new QPushButton("停止跟踪", this);
    connect(stopBtn, &QPushButton::clicked,
            this, &MainWindow::onStopTracking);
    bar->addWidget(stopBtn);

    bar->addStretch();
    m_safetyNote = new QLabel(
        "「停止跟踪」是软停止，不是急停。急停只能用示教器上的物理急停按钮。",
        this);
    m_safetyNote->setStyleSheet("color: #b00; font-weight: bold;");
    bar->addWidget(m_safetyNote);

    outer->addLayout(bar);
```

在文件顶部增加 include：

```cpp
#include <QPushButton>
```

- [ ] **Step 3: 实现三个槽**

追加到 `src/ui/MainWindow.cpp` 末尾：

```cpp
void MainWindow::onZeroToActual()
{
    // 目标置为实际、累积清零、状态回 Idle
    QMetaObject::invokeMethod(m_worker, "resetToActual",
                              Qt::QueuedConnection);
    const Pose a = m_state.snapshot().actual;
    m_suppressTargetSignal = true;
    const double v[6] = {a.x, a.y, a.z, a.a, a.b, a.c};
    for (int i = 0; i < 6; ++i) {
        m_targetSpin[i]->setValue(v[i]);
        m_targetSlider[i]->setValue(int(v[i] * 10.0));
    }
    m_suppressTargetSignal = false;
    m_trackCheck->setChecked(false);
}

void MainWindow::onTrackingToggled(bool on)
{
    QMetaObject::invokeMethod(m_worker, "setTracking",
                              Qt::QueuedConnection, Q_ARG(bool, on));
}

void MainWindow::onStopTracking()
{
    // 软停止：误差归零、原地停住，但通信线程继续回包
    m_trackCheck->setChecked(false);
    onZeroToActual();
}
```

- [ ] **Step 4: 在 `onRefresh()` 中反映 Fault 状态**

在 `onRefresh()` 内 `m_statusLabel->setText(st);` 之前插入：

```cpp
    // Fault 必须经「归零到当前位姿」清除后才能重新使能
    if (s.state == TrackState::Fault && m_trackCheck->isChecked())
        m_trackCheck->setChecked(false);
```

- [ ] **Step 5: 构建并做闭环手动验证**

Run:
```bash
PATH="$MINGW:$NINJA:$PATH" "$CMAKE" --build build --target rsi_host
cd /e/kuka_rsi_win
PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH" ./build/rsi_host.exe &
sleep 3
PATH="D:/Software/QT/content/6.5.3/mingw_64/bin:$PATH" \
  ./build/tools/krc_simulator/krc_simulator.exe \
  --host 192.168.44.1 --port 59152 --cycle-ms 12 --cycles 2000
```

操作与预期：
1. 连接后勾选「使能跟踪」，把 `X` 数值框加 5mm
2. 误差曲线出现一个峰随后衰减到 0；模拟器 `final_pose` 的 `X` 约为 `1255.000`
3. 把 `X` 再加 100mm → 累积超过 30mm 限值 → 状态栏出现「故障: accumulated translation ... exceeds limit」，「使能跟踪」自动取消勾选，模拟器仍 `PASS`（证明 Fault 下依然按时回包）
4. 点「归零到当前位姿」→ 故障清除，可重新使能

- [ ] **Step 6: 提交**

```bash
git add src/ui/MainWindow.h src/ui/MainWindow.cpp
git commit -m "feat(ui): add two-stage tracking enable, soft stop and fault latch"
```

---

### Task 13: KRC 侧 RSI 对象图与 KRL 程序

> **依赖 guest 访问，未打通前不要开始本任务。** Task 1–12 与本任务无依赖关系，可先全部完成。

**Files:**
- Create: `krc/PoseTrack.rsi`, `krc/PoseTrack.rsi.xml`, `krc/PoseTrack_ethernet.xml`, `krc/PoseTrack.src`

**RSI 需要四个文件，缺一不可**（据 `ref/kuka_rsi_hw_interface/krl/KR_C4/` 实证）：

| 文件 | 作用 |
|---|---|
| `PoseTrack.rsi` | RSI Visual 模型（`<rSIModel>`），含端口连接 |
| `PoseTrack.rsi.xml` | 运行时对象图（`<RSIObjects>`），KRC 实际执行 |
| `PoseTrack_ethernet.xml` | 通信配置，被 ETHERNET 对象的 `ConfigFile` 参数引用 |
| `PoseTrack.src` | KRL 程序 |

**已确认的对象定义：**
- `POSCORR` ObjTypeID = **27**，输入 InIdx 1–6 ← `ETHERNET1` OutIdx 1–6
- `POSCORR` 参数：`LowerLimX/Y/Z` = ParamID 1/2/3，`UpperLimX/Y/Z` = ParamID 4/5/6，**限值相对 RSI 启动位置**
- `ETHERNET` ObjTypeID = **64**，参数 `ConfigFile`/`Timeout`(默认 100)/`Flag`/`Precision`(4)
- `Precision=4` 与 `RsiCodec::buildSen` 的 `'f', 4` 一致

**必须先从 guest 确认的三项**（猜错会导致 RSI 拒绝加载或坐标系错误）：
1. 选择参考坐标系（BASE）的参数名
2. 姿态 A/B/C 是否有独立限值参数
3. `POSCORRMON` 的 ObjTypeID 与参数名

- [ ] **Step 1: 从 guest 取得 POSCORR 的权威定义**

三选一，优先第一种：

1. **用 RSI Visual 生成**（KUKA 官方做法，结果必然正确）：guest 内最小化 smartHMI，打开 RSI Visual，新建上下文，拖入 `POSCORR` 与 `ETHERNET` 对象并连接 Out1–6 → In1–6，设置参考坐标系为 BASE，另存为 `PoseTrack.rsi`。同目录会生成 `.rsi.xml` 与 `.rsi.diagram`。
2. **读对象类型定义**：`C:\KRC\Roboter\Config\System\Common\Schemes\RSIContext.xsd` 以及 RSI 对象库，查 POSCORR/POSCORRMON 的参数表。
3. **参照现有 AXISCORR 程序**：读 guest 上你现有 RSI 程序的 `.rsi.xml`，确认本机 RSI 版本实际使用的对象写法。

把取得的定义记录下来，再进行 Step 2。

- [ ] **Step 2: 写 `krc/PoseTrack_ethernet.xml`**

```xml
<ROOT>
   <CONFIG>
      <IP_NUMBER>192.168.44.1</IP_NUMBER>
      <PORT>59152</PORT>
      <SENTYPE>ImFree</SENTYPE>
      <ONLYSEND>FALSE</ONLYSEND>
   </CONFIG>
   <SEND>
      <ELEMENTS>
         <ELEMENT TAG="DEF_RIst"  TYPE="DOUBLE" INDX="INTERNAL" />
         <ELEMENT TAG="DEF_RSol"  TYPE="DOUBLE" INDX="INTERNAL" />
         <ELEMENT TAG="DEF_AIPos" TYPE="DOUBLE" INDX="INTERNAL" />
         <ELEMENT TAG="DEF_ASPos" TYPE="DOUBLE" INDX="INTERNAL" />
         <ELEMENT TAG="DEF_Delay" TYPE="LONG"   INDX="INTERNAL" />
      </ELEMENTS>
   </SEND>
   <RECEIVE>
      <ELEMENTS>
         <ELEMENT TAG="RKorr.X" TYPE="DOUBLE" INDX="1" HOLDON="1" />
         <ELEMENT TAG="RKorr.Y" TYPE="DOUBLE" INDX="2" HOLDON="1" />
         <ELEMENT TAG="RKorr.Z" TYPE="DOUBLE" INDX="3" HOLDON="1" />
         <ELEMENT TAG="RKorr.A" TYPE="DOUBLE" INDX="4" HOLDON="1" />
         <ELEMENT TAG="RKorr.B" TYPE="DOUBLE" INDX="5" HOLDON="1" />
         <ELEMENT TAG="RKorr.C" TYPE="DOUBLE" INDX="6" HOLDON="1" />
      </ELEMENTS>
   </RECEIVE>
</ROOT>
```

`RECEIVE` 的 TAG 用 `RKorr.X` 形式，对应上位机回包中的 `<RKorr X=".."/>`（参照 ROS-I 用 `AK.A1` 对应 `<AK A1=".."/>`）。`IPOC` 无需在此声明，RSI 自动处理。

- [ ] **Step 3: 写 `krc/PoseTrack.rsi.xml`**

以 Step 1 取得的定义填入参考坐标系参数与 POSCORRMON。骨架如下（`<!-- 待确认 -->` 处按 Step 1 结果替换）：

```xml
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<RSIObjects xsi:noNamespaceSchemaLocation="/Roboter/Config/System/Common/Schemes/RSIContext.xsd" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <RSIObject ObjType="POSCORR" ObjTypeID="27" ObjID="POSCORR1">
    <Inputs>
      <Input InIdx="1" OutObjID="ETHERNET1" OutIdx="1" />
      <Input InIdx="2" OutObjID="ETHERNET1" OutIdx="2" />
      <Input InIdx="3" OutObjID="ETHERNET1" OutIdx="3" />
      <Input InIdx="4" OutObjID="ETHERNET1" OutIdx="4" />
      <Input InIdx="5" OutObjID="ETHERNET1" OutIdx="5" />
      <Input InIdx="6" OutObjID="ETHERNET1" OutIdx="6" />
    </Inputs>
    <Parameters>
      <Parameter Name="LowerLimX" ParamID="1" ParamValue="-100.0" />
      <Parameter Name="LowerLimY" ParamID="2" ParamValue="-100.0" />
      <Parameter Name="LowerLimZ" ParamID="3" ParamValue="-100.0" />
      <Parameter Name="UpperLimX" ParamID="4" ParamValue="100.0" />
      <Parameter Name="UpperLimY" ParamID="5" ParamValue="100.0" />
      <Parameter Name="UpperLimZ" ParamID="6" ParamValue="100.0" />
      <!-- 待确认：参考坐标系参数（BASE），姿态 A/B/C 限值 -->
    </Parameters>
  </RSIObject>
  <RSIObject ObjType="ETHERNET" ObjTypeID="64" ObjID="ETHERNET1">
    <Parameters>
      <Parameter Name="ConfigFile" ParamID="1" ParamValue="PoseTrack_ethernet.xml" IsRuntime="false" />
      <Parameter Name="Timeout"    ParamID="1" ParamValue="100" />
      <Parameter Name="Flag"       ParamID="4" ParamValue="1" />
      <Parameter Name="Precision"  ParamID="8" ParamValue="4" />
    </Parameters>
  </RSIObject>
  <!-- 待确认：POSCORRMON 对象，构成设计 §8 的第 3 层限值 -->
</RSIObjects>
```

`PoseTrack.rsi`（RSI Visual 模型）由 Step 1 的工具生成，不手写。

- [ ] **Step 4: 写 `krc/PoseTrack.src`**

```krl
&ACCESS RVP
&REL 4
&PARAM TEMPLATE = C:\KRC\Roboter\Template\vorgabe
&PARAM EDITMASK = *
DEF PoseTrack( )

; =============================================
; RSI POSCORR 实时位姿跟踪
;   参考坐标系：BASE    修正模式：RELATIVE
;
; 约束：
;  1) 与现有 AXISCORR 程序互斥。RKorr 与 AKorr 同时供值行为不可预测。
;  2) 不修改 $TOOL / $BASE，沿用执行本程序时激活的坐标系。
;  3) 首条运动指令为 PTP $POS_ACT，原地完成 BCO，零位移。
;  4) 上位机收到首帧后把目标置为实际位姿，故启动后机器人原地待命。
;  5) RSI_MOVECORR() 阻塞执行，直到 RSI 结束才返回，无需自建循环。
; =============================================

DECL INT ret
DECL INT CONTID

;FOLD INI
  ;FOLD BASISTECH INI
    GLOBAL INTERRUPT DECL 3 WHEN $STOPMESS==TRUE DO IR_STOPM ( )
    INTERRUPT ON 3
    BAS (#INITMOV,0 )
  ;ENDFOLD (BASISTECH INI)
;ENDFOLD (INI)

; 原地 BCO：以当前位置为起点，不产生位移
PTP $POS_ACT

; 建立 RSI 上下文（加载对象图 .rsi，它再引用 _ethernet.xml）
ret = RSI_CREATE("PoseTrack.rsi", CONTID, TRUE)
IF (ret <> RSIOK) THEN
  HALT
ENDIF

; 启动 RSI，RELATIVE 修正模式
ret = RSI_ON(#RELATIVE)
IF (ret <> RSIOK) THEN
  HALT
ENDIF

; 传感器引导运动。阻塞至 RSI 结束。
RSI_MOVECORR()

; 收尾
ret = RSI_OFF()
IF (ret <> RSIOK) THEN
  HALT
ENDIF

END
```

- [ ] **Step 5: 提交**

```bash
git add krc/
git commit -m "feat(krc): add POSCORR RSI object graph, ethernet config and KRL program"
```


---

### Task 14: 部署说明与联机测试清单

**Files:**
- Create: `docs/deployment.md`

**Interfaces:**
- Consumes: Task 13 的两个部署产物；Task 9 的验证结论
- Produces: 可执行的部署与联机步骤文档

- [ ] **Step 1: 写 `docs/deployment.md`**

```markdown
# 部署与联机测试

## 前置条件

宿主侧三层验证的前两层必须已通过（见实施计划 Task 9）：

```bash
./build/tools/krc_simulator/krc_simulator.exe \
  --host 127.0.0.1 --port 59152 --cycle-ms 12 --cycles 500
```

必须输出 `replies=500 timeouts=0 ipoc_mismatch=0` 与 `PASS`。**未通过不要联机。**

## 网络

| 端 | 地址 |
|---|---|
| 宿主（上位机） | `192.168.44.1/24`（VMnet1，host-only） |
| guest（OfficeLite） | `192.168.44.128/24` |

验证连通：`ping 192.168.44.128` 应有 0–1ms 回复。

注意：示教器「网络配置」页显示的 `172.31.1.147` 与实际网卡地址不符，**以 `192.168.44.128` 为准**。

## 部署文件到 guest

把两个文件复制到虚拟机：

| 源 | 目标 |
|---|---|
| `krc/PoseTrack.src` | `KRC:\R1\Program\` |
| `krc/PoseTrack.rsi` | `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\` |
| `krc/PoseTrack.rsi.xml` | 同上 |
| `krc/PoseTrack_ethernet.xml` | 同上 |

传输途径任选：SMB（`net use Z: \\192.168.44.128\C$ /user:<账号> <密码>`，注意 guest Windows 账号密码与示教器管理员密码 `kuka` 不是同一个）或 RDP（`mstsc /v:192.168.44.128`，tcp/3389 已确认开放）。

## 部署前核对参数

| 参数 | 核对方式 | 两处必须一致 |
|---|---|---|
| UDP 端口 | XML 的 `<PORT>` ↔ `config/rsi_config.json` 的 `listen_port` | 否则收不到包 |
| 上位机 IP | XML 的 `<IP_NUMBER>` = `192.168.44.1` | 否则 RSI 连不上 |
| `SENTYPE` | XML 的 `<SENTYPE>` ↔ JSON 的 `sen_type` | 回包被拒 |
| 实际 IPO 周期 | 联机后读界面「周期」实测值，回填 JSON 的 `cycle_ms` | 步长上限算错 |

## 联机测试步骤

1. **确认 AXISCORR 程序未在运行** —— `RKorr` 与 `AKorr` 不可同时供值。
2. 宿主启动上位机：`./build/rsi_host.exe`，状态栏应为「○ 未连接」。
3. 示教器切 **T1** 模式，选中 `PoseTrack.src`。
4. 把界面参数调到最保守：`Kp` 0.1 / 0.1，限速 10 mm/s / 2 °/s，累积上限 10 mm / 5 °。
5. 启动 KRL 程序，做 BCO 运行。**机器人不应产生位移**（`PTP $POS_ACT` 原地 BCO）。
6. 观察上位机：状态栏变「● 已连接」，当前位姿显示实际值，**误差应全为 0**（首帧已同步目标）。
7. 读界面「周期」实测值，回填 `config/rsi_config.json` 的 `cycle_ms`，重启上位机。
8. 勾选「使能跟踪」。机器人仍应静止（误差为 0）。
9. **单轴小量试探**：把 `X` 加 2mm，观察机器人沿 BASE 的 X 方向移动约 2mm，误差曲线出现峰后衰减到 0。
10. 逐步放开：先单轴增大到 10mm，再试姿态 `A` 加 2°，最后六自由度同时给量。
11. 逐步提高 `Kp` 与限速，同时盯住误差曲线是否出现振荡或超调。

## 异常处理

| 现象 | 排查方向 |
|---|---|
| 上位机始终「未连接」 | 端口不一致；guest 防火墙；`ping` 不通 |
| RSI 报错停机、提示修正量过大 | 累积上限设得过大，或 `Kp` 过高导致单周期增量撞上 POSCORR ~50mm 硬限 |
| 机器人启动瞬间跳动 | 首帧未同步目标；检查「误差应全为 0」这一步是否被跳过 |
| 误差曲线持续振荡 | `Kp` 过高，减半再试 |
| 界面显示丢包递增 | 宿主负载过高，或 `max_reply_us` 接近周期；关掉曲线以外的负载重试 |
| 机器人行为与预期方向相反 | 确认 `$BASE` 就是你以为的那个基坐标系 |

## 安全

- 界面上的「停止跟踪」是**软停止**：误差归零、机器人停在原地，但上位机仍持续回包（停止回包会导致 RSI 报错停机）。
- **急停只能用示教器上的物理急停按钮**，界面上任何按钮都不能替代。
- 首次联机务必在 T1 模式、小范围、低增益下进行。
```

- [ ] **Step 2: 提交**

```bash
git add docs/deployment.md
git commit -m "docs: add deployment steps and commissioning checklist"
```

---

## Self-Review

**1. 规格覆盖检查**

| 设计文档章节 | 对应任务 |
|---|---|
| §2 已确认决策（双线程、原地 BCO、沿用 TOOL/BASE） | Task 7, 13 |
| §3 运行环境（工具链） | Task 1 |
| §4 RSI 约束（RELATIVE/50mm/IPOC/不混用） | Task 4, 5, 7, 13 |
| §5 数据流 | Task 7 |
| §6.1 RSI 配置 XML | Task 13 |
| §6.2 KRL 程序与启动序列 | Task 13 |
| §7.1 四个单元划分 | Task 4, 5, 7, 10 |
| §7.2 线程模型与通信 | Task 6, 7 |
| §7.3 界面布局 | Task 10, 11, 12 |
| §7.4 状态机与软停止 | Task 12 |
| §8 控制律与四层限值 | Task 5（第 1、2 层）、Task 13（第 3 层）、RSI 内建（第 4 层） |
| §9 错误处理 | Task 5, 7 |
| §10 三层测试 | Task 2–5（第 1 层）、Task 8、9（第 2 层）、Task 14（第 3 层） |
| §11 配置文件 | Task 3 |
| §12 待确认参数 | Task 14 核对表 |

无遗漏章节。

**2. 占位符扫描**

`src/*.cpp` 中 `// filled in by a later task` 是 Task 1 刻意留下的可编译骨架，均在后续任务被实际实现覆盖（`AppConfig.cpp`→Task 3，`RsiCodec.cpp`→Task 4，`PoseController.cpp`→Task 5，`RsiWorker.cpp`→Task 7，`MainWindow.cpp`→Task 10，`ErrorChart.cpp`→Task 11）。不存在无归属的占位符。

**3. 类型一致性核对**

- `Pose` 字段 `x,y,z,a,b,c`：Task 2 定义，Task 4/5/7/8/10 一致引用
- `wrap180` / `poseSub`：Task 2 定义，Task 5 用于误差、Task 8 用于模拟器累加
- `RobFrame.rist/.rsol/.ipoc/.valid`：Task 4 定义，Task 7 一致引用
- `RsiCodec::parseRob` / `buildSen` 签名：Task 4 定义，Task 7/8 一致调用
- `TrackState::Idle/Tracking/Fault`：Task 5 定义，Task 6 快照、Task 7 判定、Task 10/12 显示一致
- `PoseController` 方法名 `configure/setTarget/target/resetToActual/setTracking/step/state/accumulated/faultReason`：Task 5 定义，Task 7 全部按此调用
- `StatusSnapshot` 字段：Task 6 定义，Task 7 `publishSnapshot` 逐字段填充，Task 10 `onRefresh` 逐字段读取，名称一致
- `SampleRing::push/copyOut/clear/kCapacity`：Task 6 定义，Task 7 push、Task 11 copyOut、Task 7 clear 一致
- `RsiWorker` slots `start/stop/applyTarget/setTracking/resetToActual/applyConfig`：Task 7 定义，Task 9/10/12 通过 `invokeMethod` 按名调用，字符串与声明一致
- `ErrorChart::updateFrom(const SampleRing&)`：Task 11 定义并在同任务接入

**已就地修正**：`Pose` 与 `AppConfig` 会通过 `Q_ARG` 跨线程排队传递（Task 9/10/12 的 `invokeMethod`），必须注册元类型，否则队列连接在运行时静默失败。已在 Task 2 Step 3 的 `Pose.h` 与 Task 3 Step 4 的 `AppConfig.h` 中分别加入 `Q_DECLARE_METATYPE(Pose)` 与 `Q_DECLARE_METATYPE(AppConfig)`，并补上 `#include <QMetaType>`。
