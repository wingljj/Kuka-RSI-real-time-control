# RL 0.7.0 核心构建笔记（MinGW + Eigen 静态库）

日期：2026-08-01。任务 RL-T1：Eigen 确认 + RL 核心构建 + 验证程序（Comau Racer 7-1.4 rlmdl 加载 / 正解 / 逆解）。

## 依赖确认

| 依赖 | 路径 | 说明 |
|---|---|---|
| Eigen 3.3.8 | `E:/download/eigen-3.3.8/eigen-3.3.8` | header-only；注意压缩包解压后是**嵌套目录**（`eigen-3.3.8/eigen-3.3.8/`），`EIGEN3_INCLUDE_DIR` 必须指向内层 |
| Boost 1.85.0 | `D:/Software/boost_1_85_0/boost_1_85_0` | 同样嵌套目录。RL 只用作 `Boost::headers`（纯头文件），无需编译 lib |
| libxml2 2.12.8 | `C:/msys64/ucrt64` | msys2 ucrt64 提供（include + `libxml2.dll.a` 导入库），运行时需 `ucrt64/bin` 在 PATH |
| libxslt | **无**（见下文 workaround） | msys2 镜像不可达，pacman 装不了 |

### 重要发现：机器上装过 RL 0.7.0 官方包（MSVC 版）

`C:/Program Files/Robotics Library/0.7.0/MSVC/14.1/x64/` 存在一个官方安装包（含 boost 头、NLopt、RL 头文件与 MSVC 静态库）。
它对本次 MinGW 构建产生了两次干扰，**必须显式屏蔽**：

1. `find_package(Boost)` 会先找到它自带的 boost（而不是 `BOOST_ROOT` 指定的本地 boost）。
   对策：`-DBoost_INCLUDE_DIR=<本地 boost> -DBoost_NO_SYSTEM_PATHS=ON`。
2. `find_package(NLopt)` 会找到它自带的 **MSVC 版 `nlopt.lib`**（MinGW 无法链接），导致
   `RL_BUILD_MDL_NLOPT=ON`。对策：`-DRL_BUILD_MDL_NLOPT=OFF`（我们用 `JacobianInverseKinematics`，不需要 NLopt）。
3. 验证程序 `find_package(rl CONFIG)` 会通过**注册表**找到 MSVC 版包配置（`rl_DIR` 被缓存为
   `C:/Program Files/.../rl-0.7.0`）。对策：显式传 `-Drl_DIR=<构建树>/lib/cmake/rl`（小写！）加 `-DCMAKE_FIND_USE_REGISTRY=OFF`。

## RL 0.7.0 CMake 选项（探索结果）

顶层 `CMakeLists.txt` 的开关（`cmake_dependent_option`，同名 `-D` 关闭即可）：

- `RL_BUILD_MATH` / `RL_BUILD_XML` / `RL_BUILD_KIN` / `RL_BUILD_MDL` — 核心四件（全部打开）
- `RL_BUILD_UTIL` — util（头文件库），math/mdl/kin/xml 均不引用，关闭
- `RL_BUILD_HAL` / `RL_BUILD_SG` / `RL_BUILD_PLAN` — Boost/Qt 依赖模块，关闭
- `RL_BUILD_DEMOS` / `RL_BUILD_TESTS` / `RL_BUILD_EXTRAS` — 关闭
- `RL_BUILD_MDL_NLOPT` — 依赖 NLopt，关闭（见上）
- `BUILD_SHARED_LIBS=OFF` — 静态库（非 MSVC 平台默认是 ON，必须显式关）

目标名（`add_library` 名，`OUTPUT_NAME` 为 `rl*`）：`math`(INTERFACE) `std`(INTERFACE) `xml`(INTERFACE) `mdl` `kin`。
产物：`D:/Software/rl-build/lib/librlmdl.a`、`librlkin.a`（math/std/xml 为 header-only INTERFACE 库，无 .a）。
构建树里还生成了 `lib/cmake/rl/rl-config.cmake` + `rl-export.cmake`，下游可直接 `find_package(rl CONFIG)`。

## libxslt workaround（fake libxslt）

RL 的 `rl/xml/Stylesheet.h` 直接 `#include <libxslt/transform.h>`，`XmlFactory.cpp`/`Kinematics.cpp`/`UrdfFactory.cpp`
都编译它（XSLT 分支，运行时永不触发）。本机无 libxslt 且 msys2 镜像 404/超时，故提供：

- `tools/rl_verify/cmake/FindLibXslt.cmake` — 让 `find_package(LibXslt REQUIRED)` 通过，导出 `LibXslt::LibXslt`
  INTERFACE 目标（include 指向 fakexslt）。
- `tools/rl_verify/cmake/fakexslt/libxslt/{transform.h,xsltInternals.h}` — 最小 xslt 类型
  （`xsltStylesheet` 含 `doc` 成员）与 `static inline` 空实现（`xsltNewStylesheet` 等 6 个函数），
  满足编译/链接、不产生任何运行时代价。

`CMAKE_MODULE_PATH` 必须**排在 RL 自己的 cmake 目录之前**（RL 是 `list(APPEND ...)`，我们 `-D` 传入的排前面）。

## 构建命令（RL 核心）

```bash
cmake -S D:/QTproj/rl/rl-master/rl-master -B D:/Software/rl-build -G "MinGW Makefiles" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=D:/Software/QT/content/Tools/mingw1120_64/bin/gcc.exe \
  -DCMAKE_CXX_COMPILER=D:/Software/QT/content/Tools/mingw1120_64/bin/g++.exe \
  -DCMAKE_RC_COMPILER=D:/Software/QT/content/Tools/mingw1120_64/bin/windres.exe \
  -DBUILD_SHARED_LIBS=OFF \
  -DEIGEN3_INCLUDE_DIR=E:/download/eigen-3.3.8/eigen-3.3.8 \
  -DBOOST_ROOT=D:/Software/boost_1_85_0/boost_1_85_0 \
  -DBoost_INCLUDE_DIR=D:/Software/boost_1_85_0/boost_1_85_0 \
  -DBoost_NO_SYSTEM_PATHS=ON \
  -DRL_BUILD_MDL_NLOPT=OFF \
  -DCMAKE_PREFIX_PATH=C:/msys64/ucrt64 \
  -DCMAKE_MODULE_PATH=<repo>/tools/rl_verify/cmake \
  -DRL_BUILD_PLAN=OFF -DRL_BUILD_SG=OFF -DRL_BUILD_HAL=OFF -DRL_BUILD_UTIL=OFF \
  -DRL_BUILD_DEMOS=OFF -DRL_BUILD_TESTS=OFF -DRL_BUILD_EXTRAS=OFF

cmake --build D:/Software/rl-build --target math std xml mdl kin -j 8
```

注意：`-DCMAKE_PREFIX_PATH=C:/msys64/ucrt64` 会让 CMake 把 **ucrt64 的 windres** 选为 RC 编译器
（preprocessing failed），必须显式 `-DCMAKE_RC_COMPILER` 指回 Qt 的 mingw 工具链。

## 构建命令（验证程序）

```bash
cmake -S tools/rl_verify -B build/rl_verify -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=D:/Software/QT/content/Tools/mingw1120_64/bin/gcc.exe \
  -DCMAKE_CXX_COMPILER=D:/Software/QT/content/Tools/mingw1120_64/bin/g++.exe \
  -Drl_DIR=D:/Software/rl-build/lib/cmake/rl \
  -DCMAKE_FIND_USE_REGISTRY=OFF \
  -DEIGEN3_INCLUDE_DIR=E:/download/eigen-3.3.8/eigen-3.3.8 \
  -DBOOST_ROOT=D:/Software/boost_1_85_0/boost_1_85_0 \
  -DBoost_INCLUDE_DIR=D:/Software/boost_1_85_0/boost_1_85_0 \
  -DBoost_NO_SYSTEM_PATHS=ON \
  -DCMAKE_PREFIX_PATH=C:/msys64/ucrt64 \
  -DCMAKE_MODULE_PATH=<repo>/tools/rl_verify/cmake

cmake --build build/rl_verify
PATH=/c/msys64/ucrt64/bin:$PATH ./build/rl_verify/rl_verify.exe   # libxml2-2.dll 运行时依赖
```

坑：`tools/rl_verify/CMakeLists.txt` 必须 `project(... LANGUAGES C CXX)` — RL 的 `FindIconv.cmake` 用
`check_c_source_runs()`，只开 CXX 会报 "C: needs to be enabled before use"。

## 验证输出（Comau Racer 7-1.4）

```
model: Comau Racer 7-1.4
dof: 6, joints: 6, operational dof: 1 (TCP frame 'frame8')
joint limits [deg]:
  joint0: [-165, 165]   joint1: [-85, 155]   joint2: [-170, 0]
  joint3: [-210, 210]   joint4: [-120, 130]  joint5: [-2700, 2700]
home q [deg]:   0   0 -90   0   0   0
home TCP position [mm]: 934.000  0.000  1150.000  |  orientation (w x y z): 0.70711  0.00000  0.70711  0.00000
q=0  TCP position [mm]: 20.000  -0.000  1804.000  |  orientation (w x y z): 1.00000  0.00000  0.00000  0.00000
IK start q [deg]:   5.72958 -11.45916 -81.40563   0.00000  -5.72958   2.86479
IK solve: true
IK solved q [deg]:   0.00000  -0.00000 -90.00000 -10.92360  -0.00000  10.92360
round-trip error per joint [rad]:
  joint 0: 2.381e-11   joint 1: 5.560e-11   joint 2: 2.585e-10
  joint 3: 1.907e-01   joint 4: 1.951e-10   joint 5: 1.907e-01
pose residual: position [mm] 0.000000, rotation [deg] 0.000000
```

结论：rlmdl 加载、正逆解全部工作。IK 位姿残差为 0（<0.5e-6 mm / deg）。关节 3/5 的 0.19 rad
偏差是**腕部零空间自运动**：q3+q5 不变（0+0 = -10.92+10.92），雅可比迭代 IK 收敛到与 home 位姿
等价的另一组关节角（Comau 腕部类似 Puma 结构），位姿完全相同。若需要"回到原关节角"需解析 IK 或
加位形偏好（如从 q 出发的最近解）；RL 0.7.0 的 `rl::mdl::AnalyticalInverseKinematics` 是抽象基类，
**核心库内没有具体实现**（无 6 自由度解析 IK 子类），因此本任务用 `rl::mdl::JacobianInverseKinematics`
（迭代）验证。

## 验证程序用到的 RL API（0.7.0）

- 加载：`rl::mdl::XmlFactory factory; auto model = factory.create(path)`（返回 `shared_ptr<rl::mdl::Model>`）
- 正解：`kinematic->setPosition(q); kinematic->forwardPosition(); kinematic->getOperationalPosition(0)`
- 操作点：`getOperationalDof()` = 树中叶节点帧数（本模型只有 frame8 一个叶 = TCP），`getOperationalFrame(0)->getName()`
- 关节限位：`kinematic->getJoint(i)->getMinimum()(0)` / `getMaximum()(0)`（内部弧度，XML 是度）
- 迭代逆解：`rl::mdl::JacobianInverseKinematics ik(kinematic.get()); ik.addGoal(goal, 0); ik.setDuration(...); ik.solve()`，
  solve 后 `kinematic->getPosition()` 取结果（需再 `forwardPosition()` 刷新位姿）
- 角度单位：模型内部一律弧度；XML 的 `<min>/<max>/<home>` 由 XmlFactory 自动 deg→rad

## 打包与分发 (RL-T4)

日期：2026-08-02。

### 运行时依赖

| 依赖 | 来源 | 说明 |
|---|---|---|
| robot.rlmdl.xml | `D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml` | Comau Racer 7-1.4 运动学模型（`--model` 指定路径） |
| libxml2-2.dll | `C:/msys64/ucrt64/bin` | RL xml 模块的运行时依赖（动态链接） |
| libiconv-2.dll, liblzma-5.dll, zlib1.dll | `C:/msys64/ucrt64/bin` | libxml2 的传递依赖（ldd 实测；三者只依赖系统 DLL，无需再带） |
| Qt6*.dll | Qt 6.5.3 mingw_64 | Qt Core/Network/Gui/Widgets/Charts |
| libgcc_s_seh-1.dll, libstdc++-6.dll, libwinpthread-1.dll | MinGW 11.2.0 | C++ 运行时 |

### 编译期依赖（运行时不需要）

- **Eigen 3.3.8**：header-only，编译后零运行时依赖
- **Boost 1.85.0**：仅 `Boost::headers`（RL 头文件引用），零运行时依赖
- **RL 核心库**（math/mdl/kin/xml）：**静态链接**进 exe（`librlmdl.a`、`librlkin.a`；math/std/xml 为 INTERFACE 头文件库）
- **libxslt**：fake headers 绕过（RL 的 XSLT 分支编译期满足，运行时永不触发）

### 打包验证（干净环境，2026-08-02）

`bash tools/package.sh` 产物 dist/ 33M。验证方式：清空 QTDIR/CMAKE_PREFIX_PATH/
EIGEN3_INCLUDE_DIR/rl_DIR/BOOST_ROOT 等环境变量，PATH 只留 `/usr/bin:/bin:C:/Windows/system32`
（不带 Qt、不带 ucrt64），cd dist 后直接运行——exe 目录缺任何 DLL 都会当场起不来。

1. **正解自检**：`dist/krc_simulator.exe --model robot.rlmdl.xml --self-test`
   → `self-test OK`（rc=0）。证明 Qt6*.dll、libxml2 链、MinGW 运行库全部从
   dist/ 自身目录解析成功，RL 静态链接生效。
2. **闭环**：宿主 `loopback_test.exe --track 50`（与 rsi_host 同一 RsiWorker
   宿主代码）+ 模拟器 `--model robot.rlmdl.xml --cycles 200`：
   - 模拟器：`cycles=200 replies=200 missed=0 ipoc_mismatch=0 delay=0` → `PASS`
   - 宿主：`frames=200 missed=0 cycle_ms=12.00 max_reply_us=168.8`，
     `state=Tracking`，`accum X=50.000 err X=0.000 actual X=70.000 target X=70.000`
   - 结论：RL 逆解在干净环境精确跟踪 50mm 目标，零 Eigen 运行时依赖。
3. **rsi_host.exe**：干净环境启动 GUI 存活（rsi_host 纯 Qt，不含 RL）。

### --model 参数

模拟器支持 `--model <path>` 覆盖默认模型路径。main.cpp 的默认值仍是开发机
绝对路径 `D:/QTproj/rl/rl-master/3dmodel/robot.rlmdl.xml`（开发树双击
tools/启动模拟器.bat 时可用，不破坏开发流程）。分发包里模型放在 exe 同目录：
`dist/启动模拟器.bat` 先 `cd /d %~dp0` 再以相对路径传 `--model robot.rlmdl.xml`
（包内脚本由 tools/启动模拟器-dist.bat 复制而来）。`tools/package.sh` 的
Release 构建带 RL 配置钉（rl_DIR / FIND_USE_REGISTRY=OFF / Eigen / Boost /
编译器显式指回 Qt 的 mingw 工具链），详见脚本内注释。
