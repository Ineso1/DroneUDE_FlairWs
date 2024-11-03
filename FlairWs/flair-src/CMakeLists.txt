PROJECT(Flair)
cmake_minimum_required(VERSION 2.8)
include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)
include(${CMAKE_ROOT}/Modules/ExternalProject.cmake)

#we use external project to force lib install before compiling demos and tools
ExternalProject_add(FlairLibs
    SOURCE_DIR $ENV{FLAIR_ROOT}/flair-src/lib
    BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/FlairLibs
    CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE} -DCMAKE_CODELITE_USE_TARGETS=ON
    CMAKE_GENERATOR "CodeLite - Unix Makefiles"
)

if (DEFINED FOUND_HDS_DIR)
    add_subdirectory($ENV{FLAIR_ROOT}/flair-hds/src flair-hds)
endif()

add_subdirectory(${CMAKE_SOURCE_DIR}/newInes)
