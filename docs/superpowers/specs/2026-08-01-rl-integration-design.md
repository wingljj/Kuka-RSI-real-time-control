# RL 库集成设计（Robotics Library Integration）

日期：2026-08-01
状态：设计已获用户批准（阶段 1；阶段 2 可视化后续）

## 背景

用户决定用 Robotics Library（RL, github.com/roboticslibrary/rl，本机 `D:\QTproj\rl\rl-master`）
的现成算法与工具替代自写运动学，并在实机前做模拟验证。模型采用
`D:\QTproj\rl\rl-master\3dmodel\robot.rlmdl.xml`（**Comau Racer 7-1.4，6 轴**，
不需要 KR210）。

## 范围

**阶段 1（本次）**：
- Eigen 依赖（本机 `E:\download\eigen-3.3.8`，header-only）
- RL 核心构建（`rl::math/mdl/kin/xml`，纯 C++ + Eigen）
- 模拟器运动学替换：`rl_kinematics` 封装（rlmdl 加载 + 正解 + **解析逆解**）
  替代自写 `kr210`（保留不删）
- 测试与回归

**阶段 2（后续，独立一轮）**：RL 可视化（Qt6 适配渲染，`robot.rlmdl.xml` + VRML 几何）。

## 关键技术决策

| # | 决策 | 内容 |
|---|---|---|
| 1 | 模型 | Comau Racer 7-1.4（robot.rlmdl.xml），非 KR210 |
| 2 | 阶段 | 先核心运动学替换，可视化随后 |
| 3 | 逆解 | RL `AnalyticalInverseKinematics`（解析，替代雅可比伪逆） |
| 4 | Eigen | 本机 E:\download\eigen-3.3.8（header-only），不进项目仓库 |
| 5 | kr210 | 保留不删（历史/对照），模拟器不再使用 |

## 实现设计

### 1. Eigen 依赖

- 目录：`E:\download\eigen-3.3.8`（header-only，`Eigen/` 子目录）
- 使用方式：CMake `-DEIGEN3_INCLUDE_DIR=E:/download/eigen-3.3.8`（RL 的
  `FindEigen3.cmake` 用）或 `-DEigen3_DIR`。不复制进项目。

### 2. RL 核心构建

- 源码：`D:\QTproj\rl\rl-master\rl-master`（CMake 3.1+，C++11）
- 构建：CMake 配置，**禁用 plan/sg/hal**（Boost/Qt 依赖模块）——探索
  `RL_BUILD_*`/`RL_*_LIBRARY` 选项；只构建 `rl::math/mdl/kin/xml`
- 工具链：MinGW（项目一致，`D:/Software/QT/content/Tools/mingw1120_64`）
- 产出：静态库 + `rlmdl` 解析器（`rl::xml` 加载 XML 模型）
- 安装：构建到工具目录（如 `D:/Software/rl-build`），不进项目仓库；
  通过 CMake target/接口引入项目构建

### 3. 模拟器运动学替换（`tools/krc_simulator/rl_kinematics.h/.cpp`）

接口对齐现有 `kr210`（`tools/krc_simulator/kinematics.h`），模拟器 main.cpp 最小改动：

```cpp
namespace rlk {
// 加载 robot.rlmdl.xml，失败返回 false
bool loadModel(const std::string &rlmdlPath);

// 正解：q（rad）→ Pose（mm/度，KUKA ZYX 欧拉——RL 模型输出转 KUKA 约定）
Pose forward(const double qRad[6]);

// 逆解：目标笛卡尔 Pose → 关节角（解析逆解，返回是否成功）
bool inverse(const Pose &target, double qRad[6]);

// 关节限位（rlmdl 中定义）
const JointLimits &limits();
}
```

- 注意 RL 的位姿是 `rl::math::Transform`（位置 m + 旋转四元数）；需转 KUKA
  A/B/C（ZYX）——用现有 `poseops` 辅助（`abcFromQuat` 等）
- 逆解：`rl::mdl::Kinematic::calculateInverseKinematics`（解析 `AnalyticalInverseKinematics`）
  或迭代 IK；目标位姿含位置+姿态
- 模拟器 main.cpp：`kr210::` 调用点换 `rlk::`（forward/solveDelta→inverse/限位）
- **保留** `tools/krc_simulator/kinematics.h/.cpp`（kr210）不删

### 4. 测试

- 新增 `tests/test_rl_kinematics.cpp`：
  - rlmdl 加载成功（robot.rlmdl.xml 路径可配置）
  - 正解已知位形（q=0 → 期望位姿，从 rlmdl 的初始位形验证）
  - 逆解往返（forward(q0) → inverse → q ≈ q0，含常规与奇异姿态）
  - 限位读取
- 模拟器闭环回归：loopback_test + verify_kinematics/verify_robustness（替换后行为不退化）
- kr210 测试保留（对照）

### 5. 风险与验证点

- RL CMake 模块选项（实现时探索 `RL_BUILD_*`）；rlmdl 解析（Comau 模型）；
  RL API 用法（`rl::mdl::Model/Kinematic/AnalyticalInverseKinematics`）
- 验证顺序：① 命令行程序加载 robot.rlmdl + 正/逆解跑通 → ② 单测 → ③ 模拟器接入 → ④ 闭环回归

## 文件变更（阶段 1）

- Create: `tools/krc_simulator/rl_kinematics.h/.cpp`
- Create: `tests/test_rl_kinematics.cpp`
- Modify: `tools/krc_simulator/main.cpp`（kr210 → rlk 调用点）
- Modify: `tools/krc_simulator/CMakeLists.txt` 或根 CMake（引入 RL 构建/链接）
- Modify: `tests/CMakeLists.txt`
- 构建脚本/文档：构建 RL 的步骤记录（tools/ 或 docs）

## 分阶段（实现）

1. Eigen 路径确认 + RL 核心构建（命令行验证程序：加载 robot.rlmdl + 正解 + 逆解）
2. `rl_kinematics` 封装（forward/inverse/limits，KUKA 约定转换）+ 单测
3. 模拟器接入（kr210 → rlk）+ 闭环回归
4. 文档（构建步骤 + 模型说明）

## 阶段 2（可视化，后续）

RL 的 Qt6 适配渲染：`robot.rlmdl.xml` + `3dmodel/` 的 VRML 几何（frame.wrl/link*.wrl），
rlPlanDemo 的渲染技术探索 + Qt6 迁移，独立一轮（验收：模拟器运行时模型动起来）。
