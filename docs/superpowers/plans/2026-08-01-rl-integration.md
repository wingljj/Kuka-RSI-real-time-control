# RL 库集成 Implementation Plan（阶段 1）

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用 Robotics Library（RL）的现成运动学替代自写 kr210：Eigen + RL 核心构建、rlmdl 模型加载（Comau Racer 7-1.4）、正解 + 解析逆解、模拟器接入；RL 静态链接，打包 exe 零 Eigen 运行时依赖。

**Architecture:** 新封装 `rl_kinematics`（rl::mdl 加载 robot.rlmdl.xml，接口对齐 kr210）；模拟器 main.cpp 调用点替换；kr210 保留不删。RL 核心（math/mdl/kin/xml）静态构建，禁 plan/sg/hal。

**Tech Stack:** C++11（RL）/ C++17（项目）/ CMake / MinGW。分支 `feature/communication-robustness`。

## Global Constraints

- **Eigen**：`E:\download\eigen-3.3.8`（header-only，编译期依赖）。**最终 exe 运行时不需要 Eigen 文件**——离线安装包不带 Eigen。
- **RL 静态链接**（`librl*.a` 进 exe，不产生 RL DLL）；若出现 RL DLL 必须打包。
- **rlmdl 模型**：`D:\QTproj\rl\rl-master\3dmodel\robot.rlmdl.xml`（Comau Racer 7-1.4，6 轴）。
- **kr210 保留不删**；模拟器不再使用（调用点换 rlk）。
- 位姿转 KUKA A/B/C（ZYX）用现有 `poseops`（`abcFromQuat` 等）。
- RL 只构建 `rl::math/mdl/kin/xml`；`plan/sg/hal`（Boost/Qt）禁用。
- 构建产物放工具目录（如 `D:/Software/rl-build`），不进项目仓库。
- 测试用文件日志器；构建 PATH：MINGW/NINJA/QTBIN。

---

### Task 1: Eigen 确认 + RL 核心构建 + 验证程序

**Files:**
- Create: `tools/rl_verify/main.cpp`（临时验证程序：加载 robot.rlmdl + 正解 + 逆解）
- Create: `tools/rl_verify/CMakeLists.txt`
- Create: `docs/rl-build-notes.md`（构建步骤记录）

**Interfaces:**
- Produces: 构建好的 RL 静态库（工具目录）+ 验证程序跑通（rlmdl 加载/正解/逆解）。Task 2 依赖。

- [ ] **Step 1: 确认 Eigen**

`E:\download\eigen-3.3.8\Eigen\` 存在（header-only）。RL CMake 用 `-DEIGEN3_INCLUDE_DIR=E:/download/eigen-3.3.8`（RL 自带 `cmake/FindEigen3.cmake`）。

- [ ] **Step 2: 配置并构建 RL 核心**

```bash
cd /d/QTproj/rl/rl-master/rl-master
cmake -S . -B D:/Software/rl-build -G "MinGW Makefiles" \
  -DCMAKE_CXX_COMPILER=D:/Software/QT/content/Tools/mingw1120_64/bin/g++.exe \
  -DEIGEN3_INCLUDE_DIR=E:/download/eigen-3.3.8 \
  -DBUILD_SHARED_LIBS=OFF \
  -DRL_BUILD_PLAN=OFF -DRL_BUILD_SG=OFF -DRL_BUILD_HAL=OFF \
  -DRL_BUILD_CORE=ON -DRL_BUILD_KIN=ON -DRL_BUILD_MDL=ON -DRL_BUILD_XML=ON
cmake --build D:/Software/rl-build --target rl rlmdl rlkin 2>&1 | tail -5
```

（CMake 选项名以实际为准——探索 `rl-master/CMakeLists.txt` 的 `option()`/`RL_*` 变量，只开 math/mdl/kin/xml。若 `BUILD_SHARED_LIBS=OFF` 不生效，找 RL 的静态/共享开关。）

- [ ] **Step 3: 验证程序（加载 + 正解 + 逆解）**

`tools/rl_verify/main.cpp`（RL API 按头文件探索填写；关键用法）：

```cpp
#include <rl/mdl/Model.h>
#include <rl/mdl/Kinematic.h>
#include <rl/mdl/AnalyticalInverseKinematics.h>
#include <rl/xml/Document.h>

int main(int argc, char **argv)
{
    const char *path = argc > 1 ? argv[1]
        : "D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml";
    // rlmdl 加载：rl::xml::Document 或 SAXHandler（按 examples/demos 探索标准方式）
    ...
    // q → 正解：kinematic->setPosition(q); forwardPosition(); getFrame(tcp)
    // 逆解：AnalyticalInverseKinematics(kinematic); solve 目标位姿 → q
    // 打印：q=0 的位姿、逆解往返误差（验证 IK 精度）
    return 0;
}
```

- [ ] **Step 4: 编译 + 运行验证**

```bash
cmake -S tools/rl_verify -B build/rl_verify -G Ninja \
  -DCMAKE_PREFIX_PATH=D:/Software/QT/content/6.5.3/mingw_64 \
  -DEIGEN3_INCLUDE_DIR=E:/download/eigen-3.3.8 \
  -DRL_DIR=D:/Software/rl-build
cmake --build build/rl_verify
./build/rl_verify/rl_verify.exe
```

Expected: 打印 q=0 正解位姿（与 robot.rlmdl 初始位形一致）、逆解往返误差（常规位形 < 1e-3 rad）。若 RL 需要 Boost（某模块），禁用该模块或用 RL 的纯核心。

- [ ] **Step 5: 记录构建步骤到 `docs/rl-build-notes.md`**（Eigen 路径、CMake 命令、选项、验证结果）

- [ ] **Step 6: Commit**

```bash
git add tools/rl_verify/ docs/rl-build-notes.md
git commit -m "feat(kin): RL core build verified — rlmdl load, forward, inverse (Comau)"
```

---

### Task 2: `rl_kinematics` 封装 + 单测

**Files:**
- Create: `tools/krc_simulator/rl_kinematics.h/.cpp`
- Create: `tests/test_rl_kinematics.cpp`
- Modify: `tests/CMakeLists.txt`
- Modify: 根 `CMakeLists.txt`（rsi_core 或模拟器链接 RL + rl_kinematics）

**Interfaces:**
- Produces: `namespace rlk { bool loadModel(const std::string &path); Pose forward(const double qRad[6]); bool inverse(const Pose &target, double qRad[6]); const kr210::JointLimits &limits(); }`。Task 3 依赖。

- [ ] **Step 1: `rl_kinematics.h/.cpp`**

```cpp
namespace rlk {
// 加载 robot.rlmdl.xml；失败返回 false（路径可配，默认 D:/QTproj/.../robot.rlmdl.xml）
bool loadModel(const std::string &rlmdlPath);

// 正解：q（rad）→ Pose（mm；A/B/C 度，KUKA ZYX——用 poseops::abcFromQuat 从
// RL 的四元数位姿转出）。位置 RL 是米 → 毫米。
Pose forward(const double qRad[6]);

// 逆解：目标 Pose → q（rad）。RL AnalyticalInverseKinematics（解析）。
// 成功返回 true；奇异/不可达返回 false（调用方回零）。
bool inverse(const Pose &target, double qRad[6]);

// 关节限位（rlmdl 的 joint 范围，rad）
const kr210::JointLimits &limits();
}
```

实现要点（按 RL API 探索结果填写）：
- 成员：`rl::mdl::Model* m_model`、`rl::mdl::Kinematic* m_kinematic`（从 Model 取）
- `loadModel`：rlmdl XML 解析 → Model → Kinematic；限位从 joint 读
- `forward`：`setPosition(q)` → `forwardPosition()` → `getFrame(tcp)`（Transform）→
  位置 m→mm，四元数→`poseops::abcFromQuat`
- `inverse`：目标 Transform（mm→m + 四元数）→ `AnalyticalInverseKinematics::solve`
  （或 `calculateInverseKinematics`）→ q；失败返回 false
- 静态单例（RL 对象生命周期）；线程安全：仅模拟器单线程调用

- [ ] **Step 2: `test_rl_kinematics.cpp`**

```cpp
class TestRlKin : public QObject {
    Q_OBJECT
private slots:
    void loadModel_ok() { QVERIFY(rlk::loadModel(kModelPath)); }
    void forward_zeroPose() {
        // q=0 → 期望位姿（与验证程序/Task 1 输出一致）
        const Pose p = rlk::forward(q0);
        QVERIFY(qAbs(p.x - expectX) < 1e-3); ...
    }
    void inverse_roundTrip() {
        // 常规位形：forward(q0) → inverse → q ≈ q0
        double q[6]; QVERIFY(rlk::inverse(target, q));
        // 逐关节差 < 1e-3 rad（或位姿差 < 1e-3 mm）
    }
    void inverse_singularOrUnreachable_returnsFalse() {
        // 奇异姿态（如 B=±90 组合）或工作空间外 → false 或近似解
    }
    void limits_positive() { QVERIFY(rlk::limits().min[0] < rlk::limits().max[0]); }
};
```

（`kModelPath` 默认 `D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml`，可环境变量覆盖。）

- [ ] **Step 3: Build + run**

```bash
cmake --build build
./build/tests/test_rl_kinematics.exe -o .superpowers/sdd/rl-t2.log,txt; cat .superpowers/sdd/rl-t2.log
```

Expected: 全绿（含加载/正解/逆解往返/限位）。

- [ ] **Step 4: Commit**

```bash
git add tools/krc_simulator/rl_kinematics.h tools/krc_simulator/rl_kinematics.cpp tests/test_rl_kinematics.cpp tests/CMakeLists.txt CMakeLists.txt
git commit -m "feat(kin): rl_kinematics wrapper — rlmdl load, forward, analytical IK, limits"
```

---

### Task 3: 模拟器接入（kr210 → rlk）+ 闭环回归

**Files:**
- Modify: `tools/krc_simulator/main.cpp`

**Interfaces:**
- Consumes: `rlk`（Task 2）。
- Produces: 模拟器用 RL 运动学；kr210 不再调用（文件保留）。

- [ ] **Step 1: main.cpp 调用点替换**

- `kr210::forward(q)` → `rlk::forward(q)`；`kr210::solveDelta(q, d, dq)` → 目标位姿法：
  当前位姿 forward(q) + 增量 d → 新目标 Pose → `rlk::inverse(目标, qNew)`（一次逆解，
  替代逐周期雅可比伪逆）；失败回零。
- `kr210::limits()` → `rlk::limits()`；初始化 `q` → `rlk::inverse(初始 Pose)` 或默认 0
- `--self-test`：用 rlk（加载模型 + 正解验证）
- 启动时 `rlk::loadModel(...)`（模型路径可 `--model` 覆盖，默认 D:/QTproj/.../robot.rlmdl.xml）

- [ ] **Step 2: 闭环回归**

```bash
cmake --build build
./build/tools/krc_simulator/krc_simulator.exe --self-test
./build/tools/loopback_test/loopback_test.exe --seconds 6 > /tmp/lb.log 2>&1 &
sleep 0.2
./build/tools/krc_simulator/krc_simulator.exe --host 127.0.0.1 --port 59152 --cycles 400 > /tmp/sim.log 2>&1
wait
cat /tmp/sim.log   # replies=400 missed=0 ipoc_mismatch=0
bash tools/verify_kinematics.sh
bash tools/verify_robustness.sh
```

Expected: 全 PASS（模拟器行为不退化；注：verify_kinematics 的关节限位场景
`--joint-limits` 现在作用于 rlk 限位——数值可能变，脚本断言若失效按新行为调整）。

- [ ] **Step 3: Commit**

```bash
git add tools/krc_simulator/main.cpp
git commit -m "feat(tools): simulator uses RL kinematics (Comau rlmdl, analytical IK)"
```

---

### Task 4: 打包验证 + 文档

**Files:**
- Modify: `docs/real-machine-deployment.md` 或 `docs/rl-build-notes.md`
- Modify: `tools/package.sh`（如需）

- [ ] **Step 1: 打包验证**

`bash tools/package.sh`（生成 dist）后，在**干净环境**（无 Eigen/无 RL 路径、无 Qt PATH）
运行模拟器 `--self-test` + 与 rsi_host 闭环：确认 exe 零 Eigen 运行时依赖（RL 静态
链接生效）。若有 RL DLL 出现，纳入 dist 打包。

- [ ] **Step 2: 文档**

`docs/rl-build-notes.md` 补：打包验证结果、模型来源（robot.rlmdl.xml）、
`--model` 参数、Eigen 编译期依赖说明（运行时不需要）。

- [ ] **Step 3: Commit**

```bash
git add docs/ tools/package.sh
git commit -m "docs(kin): RL build/packaging notes; model path parameter"
```

---

## Self-Review 记录

- **Spec 覆盖**：Eigen（Task 1）、RL 构建（Task 1）、rl_kinematics（Task 2）、模拟器替换（Task 3）、打包/文档（Task 4）。无缺口。
- **占位符**：Task 1/2 的 RL API 具体调用（Model 加载、IK 方法名）标注"按 RL 头文件探索填写"——RL 0.7 的 API 需实现时确认，验证程序与单测是断言标准（非占位）。
- **类型一致**：`rlk::loadModel/forward/inverse/limits`、`qRad[6]` 在任务间一致。
- **风险注意**：
  - RL CMake 选项名（`RL_BUILD_*`/`BUILD_SHARED_LIBS`）以实际为准，Task 1 探索。
  - RL 的 q 索引/单位（rad）与 rlmdl 模型定义一致；位姿 KUKA 转换用 poseops。
  - 逆解失败（奇异/不可达）→ 模拟器回零（安全）。
  - verify_kinematics 的关节限位场景数值可能随模型变化，脚本断言按新行为调整。
  - 打包：RL 静态链接是关键（Eigen header-only 编译期，运行时零依赖）。
