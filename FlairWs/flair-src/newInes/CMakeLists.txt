PROJECT(NewInesSim)
cmake_minimum_required(VERSION 2.8)
include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/AeroInes/)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/PD_control/)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/PID_test/)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Recompile_CircleFollower/)
