# Standalone verification program for the Robotics Library (RL) core build.
# Not part of the main app build: it links the external static RL libs
# produced by the RL build (see docs/rl-build-notes.md).
#
# Configure (example):
#   cmake -S tools/rl_verify -B build/rl_verify -G Ninja \
#     -DCMAKE_PREFIX_PATH=D:/Software/QT/content/6.5.3/mingw_64 \
#     -DEIGEN3_INCLUDE_DIR=E:/download/eigen-3.3.8/eigen-3.3.8 \
#     -DBOOST_ROOT=D:/Software/boost_1_85_0/boost_1_85_0 \
#     -DRL_DIR=D:/Software/rl-build/lib/cmake/rl \
#     -DCMAKE_PREFIX_PATH=C:/msys64/ucrt64 \
#     -DCMAKE_MODULE_PATH=<this>/cmake

cmake_minimum_required(VERSION 3.21)
# C is required: RL's FindIconv.cmake uses check_c_source_runs().
project(rl_verify LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rl CONFIG REQUIRED)

add_executable(rl_verify main.cpp)
target_link_libraries(rl_verify PRIVATE rl::mdl rl::kin)
