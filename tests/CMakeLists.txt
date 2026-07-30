add_executable(test_pose test_pose.cpp)
target_link_libraries(test_pose PRIVATE rsi_core Qt6::Test)
add_test(NAME test_pose COMMAND test_pose)

add_executable(test_app_config test_app_config.cpp)
target_link_libraries(test_app_config PRIVATE rsi_core Qt6::Test)
add_test(NAME test_app_config COMMAND test_app_config)
