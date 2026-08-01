add_executable(krc_simulator main.cpp)
# rl_kin 已 PUBLIC 链接 rsi_core（含 kinematics.cpp 的 limitCartDelta）与 rl::mdl。
target_link_libraries(krc_simulator PRIVATE rl_kin Qt6::Core Qt6::Network)
