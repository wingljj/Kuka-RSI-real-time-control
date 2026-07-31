add_executable(loopback_test main.cpp)
target_link_libraries(loopback_test PRIVATE rsi_net Qt6::Core Qt6::Network)
