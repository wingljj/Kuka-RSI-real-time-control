# OpenGLWidgets 只有模拟器的 RobotView 用到，顶层 rsi_host 已不再依赖 OpenGL，
# 所以组件在这里按需查找，避免没装 OpenGL 模块的环境连上位机都配不起来。
find_package(Qt6 REQUIRED COMPONENTS OpenGLWidgets)

add_executable(krc_simulator main.cpp RobotView.cpp RobotView.h MeshLoader.cpp MeshLoader.h)
# rl_kin 已 PUBLIC 链接 rsi_core（含 kinematics.cpp 的 limitCartDelta）与 rl::mdl。
# MinGW 下需显式链接 opengl32（Qt6::OpenGLWidgets 不自带）。
target_link_libraries(krc_simulator PRIVATE rl_kin Qt6::Core Qt6::Network Qt6::OpenGLWidgets opengl32)

# 把 qwindows.dll 部署到 exe 同级的 platforms/ 下。
#
# 本机有一个 User 级的 QT_QPA_PLATFORM_PLUGIN_PATH 指向另一套 Qt（5.15 MSVC），
# 于是模拟器启动即报 "no Qt platform plugin could be initialized"。Qt 查找平台
# 插件时 exe 同级的 platforms/ 优先于该环境变量，所以放一份在这里就免疫了——
# 这也正是 rsi_host 一直没撞上这个问题的原因（它的 platforms/ 早就在）。
#
# 不能用 windeployqt：该 Qt 安装的 plugins/platforms/ 里混进了一个 rlPlanDemo.exe，
# windeployqt 扫描时会因找不到它的依赖而中断（tools/package.sh 开头有同一段说明）。
add_custom_command(TARGET krc_simulator POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:Qt6::QWindowsIntegrationPlugin>"
            "$<TARGET_FILE_DIR:krc_simulator>/platforms/$<TARGET_FILE_NAME:Qt6::QWindowsIntegrationPlugin>"
    COMMENT "部署 qwindows.dll 到模拟器的 platforms/"
    VERBATIM)
