include_directories(${EIGEN3_INCLUDE_DIRS})
include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/SS/)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/Place/)
