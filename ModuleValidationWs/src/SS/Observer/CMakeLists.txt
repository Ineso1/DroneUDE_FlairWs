PROJECT(Observer)
cmake_minimum_required(VERSION 2.8)

include_directories("/usr/include/eigen3")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/UDE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Luenberger)