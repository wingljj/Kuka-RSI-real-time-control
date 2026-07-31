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
