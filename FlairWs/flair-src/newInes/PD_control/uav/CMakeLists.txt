PROJECT(PD_control)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)


include_directories("/usr/include/eigen3")
include_directories($ENV{FLAIR_ROOT}/flair-src/include)
include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)
add_compile_options(-Wno-poison-system-directories)

add_subdirectory(src/Control)
add_subdirectory(src/Observer)

SET(SRC_FILES
    src/main.cpp
    src/Control/Drone.cpp
    src/Control/DroneBase.cpp
    src/Control/MyLaw.cpp
    src/Observer/StateSpace.cpp
    src/Observer/ObserverBase.cpp
    src/Observer/UDE/UDE.cpp
    src/Observer/Luenberger/Luenberger.cpp
)

FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR PD_control
           COPY_RESOURCES
)
