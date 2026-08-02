# OpenGLWidgets 只有模拟器的 RobotView 用到，顶层 rsi_host 已不再依赖 OpenGL，
# 所以组件在这里按需查找，避免没装 OpenGL 模块的环境连上位机都配不起来。
find_package(Qt6 REQUIRED COMPONENTS OpenGLWidgets)

add_executable(krc_simulator main.cpp RobotView.cpp RobotView.h MeshLoader.cpp MeshLoader.h)
# rl_kin 已 PUBLIC 链接 rsi_core（含 kinematics.cpp 的 limitCartDelta）与 rl::mdl。
# MinGW 下需显式链接 opengl32（Qt6::OpenGLWidgets 不自带）。
target_link_libraries(krc_simulator PRIVATE rl_kin Qt6::Core Qt6::Network Qt6::OpenGLWidgets opengl32)
