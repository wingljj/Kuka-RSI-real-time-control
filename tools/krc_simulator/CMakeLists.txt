add_executable(krc_simulator main.cpp RobotView.cpp RobotView.h)
# rl_kin 已 PUBLIC 链接 rsi_core（含 kinematics.cpp 的 limitCartDelta）与 rl::mdl。
# MinGW 下需显式链接 opengl32（Qt6::OpenGLWidgets 不自带）。
target_link_libraries(krc_simulator PRIVATE rl_kin Qt6::Core Qt6::Network Qt6::OpenGLWidgets opengl32)
