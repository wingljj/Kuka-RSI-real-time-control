cmake_minimum_required(VERSION 3.21)
# C 必须启用：RL 的 FindIconv.cmake 用 check_c_source_runs()（见 docs/rl-build-notes.md）。
project(kuka_rsi_win LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# 屏蔽注册表里的 MSVC 版 RL/Qt 包：本机装了官方 RL 0.7.0 MSVC 安装包，
# find_package 默认会先命中注册表（rl_DIR 被缓存成 MSVC 版），导致 MinGW 链接
# 失败。Qt 走 CMAKE_PREFIX_PATH，不受影响（见 docs/rl-build-notes.md）。
set(CMAKE_FIND_USE_REGISTRY OFF)

find_package(Qt6 REQUIRED COMPONENTS Core Network Widgets Charts Test)

add_library(rsi_core STATIC
    src/core/AppConfig.cpp
    src/core/ForceController.cpp
    src/core/ForceController.h
    src/core/ForceFilter.cpp
    src/core/ForceFilter.h
    src/core/Wrench.h
    src/core/IpocTracker.cpp
    src/core/PoseOps.cpp
    src/core/RsiCodec.cpp
    src/core/PoseController.cpp
    src/core/SessionGuard.cpp
    src/core/SriDriver.cpp
    src/core/SriDriver.h
    src/core/SriProtocol.cpp
    src/core/SriProtocol.h
    src/core/TargetTrajectory.cpp
    tools/krc_simulator/kinematics.cpp
)
target_include_directories(rsi_core PUBLIC src tools/krc_simulator ${CMAKE_CURRENT_SOURCE_DIR})
# Qt6::Network：SriDriver.h 公开头直接 include <QTcpSocket>，须 PUBLIC 传递。
# 注意 Network 不引入 QT_GUI_LIB，纯算法单测仍是控制台程序。
target_link_libraries(rsi_core PUBLIC Qt6::Core Qt6::Network)

# UiLogic 单独成库而不是留在 rsi_core 里：它需要 QFontDatabase（QtGui），
# 而 QTEST_MAIN 按 QT_GUI_LIB 是否定义在 QCoreApplication 与 QGuiApplication
# 之间二选一。把 Qt6::Gui 挂到 rsi_core 上，十三个纯算法单测会一起变成需要
# 窗口站的 GUI 程序——为一个字体函数付这个代价不划算。
add_library(rsi_uilogic STATIC src/ui/UiLogic.cpp)
target_link_libraries(rsi_uilogic PUBLIC rsi_core Qt6::Gui)

# Robotics Library (RL) 0.7.0 封装（rl_kinematics，模拟器正逆解）。
# 配置钉：-Drl_DIR=D:/Software/rl-build/lib/cmake/rl（小写！）、
# -DCMAKE_FIND_USE_REGISTRY=OFF、-DBoost_NO_SYSTEM_PATHS=ON、
# -DEIGEN3_INCLUDE_DIR=<eigen>/eigen-3.3.8、-DCMAKE_PREFIX_PATH 含 C:/msys64/ucrt64
# （libxml2）。fake-libxslt 模块路径必须排在 RL 自己的 cmake 目录前（RL 会 APPEND）。
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/tools/rl_verify/cmake)
find_package(rl CONFIG REQUIRED)

add_library(rl_kin STATIC tools/krc_simulator/rl_kinematics.cpp)
target_link_libraries(rl_kin PUBLIC rsi_core rl::mdl)

# RsiWorker 单独成库：rsi_host 与 loopback_test 都要用它，
# 若各自把 RsiWorker.cpp 列进源文件会重复编译且 AUTOMOC 跨目录易出错。
add_library(rsi_net STATIC src/net/RsiWorker.cpp)
target_link_libraries(rsi_net PUBLIC rsi_core Qt6::Network)

# 窗口与部件单独成库而不是直接列进 rsi_host 的源文件：单测要构造真实的
# MainWindow 才能验证「控件初始启用态」这类只存在于部件实例上的事实
#（判定函数返回什么都不管用——缺陷恰恰是判定结果没有被施加到控件上）。
# 可执行文件里的源文件是链接不到测试目标的。
add_library(rsi_ui STATIC
    src/ui/MainWindow.cpp
    src/ui/ErrorChart.cpp
    src/ui/ForceChart.cpp
    src/ui/CommCards.cpp
    src/ui/CumulativeBar.cpp
    src/ui/AlarmLog.cpp
)
target_link_libraries(rsi_ui PUBLIC
    rsi_net rsi_uilogic Qt6::Widgets Qt6::Charts)

add_executable(rsi_host src/main.cpp)
target_link_libraries(rsi_host PRIVATE rsi_ui Qt6::Core Qt6::Network)

enable_testing()
add_subdirectory(tests)
add_subdirectory(tools/krc_simulator)
add_subdirectory(tools/loopback_test)
