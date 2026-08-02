add_executable(test_pose test_pose.cpp)
target_link_libraries(test_pose PRIVATE rsi_core Qt6::Test)
add_test(NAME test_pose COMMAND test_pose)

add_executable(test_app_config test_app_config.cpp)
target_link_libraries(test_app_config PRIVATE rsi_core Qt6::Test)
add_test(NAME test_app_config COMMAND test_app_config)

add_executable(test_rsi_codec test_rsi_codec.cpp)
target_link_libraries(test_rsi_codec PRIVATE rsi_core Qt6::Test)
add_test(NAME test_rsi_codec COMMAND test_rsi_codec)

add_executable(test_pose_controller test_pose_controller.cpp)
target_link_libraries(test_pose_controller PRIVATE rsi_core Qt6::Test)
add_test(NAME test_pose_controller COMMAND test_pose_controller)

add_executable(test_shared_state test_shared_state.cpp)
target_link_libraries(test_shared_state PRIVATE rsi_core Qt6::Test)
add_test(NAME test_shared_state COMMAND test_shared_state)

add_executable(test_ipoc_tracker test_ipoc_tracker.cpp)
target_link_libraries(test_ipoc_tracker PRIVATE rsi_core Qt6::Test)
add_test(NAME test_ipoc_tracker COMMAND test_ipoc_tracker)

add_executable(test_session_guard test_session_guard.cpp)
target_link_libraries(test_session_guard PRIVATE rsi_core Qt6::Test)
add_test(NAME test_session_guard COMMAND test_session_guard)

add_executable(test_kr210_kinematics test_kr210_kinematics.cpp)
target_link_libraries(test_kr210_kinematics PRIVATE rsi_core Qt6::Test)
add_test(NAME test_kr210_kinematics COMMAND test_kr210_kinematics)

add_executable(test_pose_ops test_pose_ops.cpp)
target_link_libraries(test_pose_ops PRIVATE rsi_core Qt6::Test)
add_test(NAME test_pose_ops COMMAND test_pose_ops)

add_executable(test_target_trajectory test_target_trajectory.cpp)
target_link_libraries(test_target_trajectory PRIVATE rsi_core Qt6::Test)
add_test(NAME test_target_trajectory COMMAND test_target_trajectory)

# RL 封装单测：链接 rl_kin（RL 静态库 + Eigen 经 rl::mdl 传递）。
add_executable(test_rl_kinematics test_rl_kinematics.cpp)
target_link_libraries(test_rl_kinematics PRIVATE rl_kin Qt6::Test)
add_test(NAME test_rl_kinematics COMMAND test_rl_kinematics)

add_executable(test_mesh_loader test_mesh_loader.cpp ../tools/krc_simulator/MeshLoader.cpp)
target_link_libraries(test_mesh_loader PRIVATE rl_kin Qt6::Test Qt6::Widgets)
add_test(NAME test_mesh_loader COMMAND test_mesh_loader)

add_executable(test_ui_logic test_ui_logic.cpp)
target_link_libraries(test_ui_logic PRIVATE rsi_uilogic Qt6::Test)
add_test(NAME test_ui_logic COMMAND test_ui_logic)

# RsiWorker 的会话/间隙路由单测：链接 rsi_net（内含 RsiWorker），
# 用真实 UDP 环回驱动，因为被测的判据依赖 QElapsedTimer 与看门狗定时器。
add_executable(test_rsi_worker test_rsi_worker.cpp)
target_link_libraries(test_rsi_worker PRIVATE rsi_net Qt6::Test Qt6::Network)
add_test(NAME test_rsi_worker COMMAND test_rsi_worker)
