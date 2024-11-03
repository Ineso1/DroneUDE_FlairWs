PROJECT(Porfavor)
cmake_minimum_required(VERSION 2.8)

SET(FLAIR_USE_FILTER TRUE)
SET(FLAIR_USE_SENSOR_ACTUATOR TRUE)
SET(FLAIR_USE_META TRUE)
SET(FLAIR_USE_VRPN TRUE)


include_directories("/usr/include/eigen3")
include($ENV{FLAIR_ROOT}/flair-src/cmake-modules/GlobalCmakeFlair.cmake)
add_compile_options(-Wno-poison-system-directories)


SET(SRC_FILES
    src/main.cpp
    src/UDEdrone.cpp
    src/MyLaw.cpp
    src/StateSpace.cpp
)

FLAIR_DEMO(${PROJECT_NAME} "${SRC_FILES}"
           DEST_DIR Porfavor
           COPY_RESOURCES
)
