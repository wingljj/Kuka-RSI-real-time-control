add_executable(krc_simulator main.cpp)
target_link_libraries(krc_simulator PRIVATE rsi_core Qt6::Core Qt6::Network)
