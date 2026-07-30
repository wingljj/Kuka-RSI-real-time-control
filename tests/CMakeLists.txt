add_executable(test_pose test_pose.cpp)
target_link_libraries(test_pose PRIVATE rsi_core Qt6::Test)
add_test(NAME test_pose COMMAND test_pose)
