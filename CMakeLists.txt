cmake_minimum_required(VERSION 3.21)
project(kuka_rsi_win LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Network Widgets Charts Test)

add_library(rsi_core STATIC
    src/core/AppConfig.cpp
    src/core/IpocTracker.cpp
    src/core/RsiCodec.cpp
    src/core/PoseController.cpp
    src/core/SessionGuard.cpp
)
target_include_directories(rsi_core PUBLIC src)
target_link_libraries(rsi_core PUBLIC Qt6::Core)

# RsiWorker 单独成库：rsi_host 与 loopback_test 都要用它，
# 若各自把 RsiWorker.cpp 列进源文件会重复编译且 AUTOMOC 跨目录易出错。
add_library(rsi_net STATIC src/net/RsiWorker.cpp)
target_link_libraries(rsi_net PUBLIC rsi_core Qt6::Network)

add_executable(rsi_host
    src/main.cpp
    src/ui/MainWindow.cpp
    src/ui/ErrorChart.cpp
)
target_link_libraries(rsi_host PRIVATE
    rsi_net Qt6::Core Qt6::Network Qt6::Widgets Qt6::Charts)

enable_testing()
add_subdirectory(tests)
add_subdirectory(tools/krc_simulator)
add_subdirectory(tools/loopback_test)
