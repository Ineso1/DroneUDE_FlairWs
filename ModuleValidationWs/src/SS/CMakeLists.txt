project(SS_project)

set(CMAKE_CXX_STANDARD 17) 

add_executable(
    ss_executable 
    main.cpp 
    StateSpace.cpp
    MyLaw.cpp
    Drone.cpp
)

target_link_libraries(
    ss_executable 
    Eigen3::Eigen
    stdc++fs
)

add_definitions(-DSOURCE_DIR="${CMAKE_SOURCE_DIR}")

add_custom_command(
    TARGET ss_executable POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_SOURCE_DIR}/SimData"
)

add_custom_command(
    OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/SimData/RealStateSpace_trans.csv"
           "${CMAKE_CURRENT_SOURCE_DIR}/SimData/RealStateSpace_rot.csv"
           "${CMAKE_CURRENT_SOURCE_DIR}/SimData/ControlData.csv"
           "${CMAKE_CURRENT_SOURCE_DIR}/SimData/TranslationalEstimation.csv"
           "${CMAKE_CURRENT_SOURCE_DIR}/SimData/RotationalEstimation.csv"
    COMMAND ${CMAKE_COMMAND} -E touch   "${CMAKE_CURRENT_SOURCE_DIR}/SimData/RealStateSpace_trans.csv"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/SimData/RealStateSpace_rot.csv"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/SimData/ControlData.csv"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/SimData/TranslationalEstimation.csv"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/SimData/RotationalEstimation.csv"
)

add_custom_target(create_csv_files
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/SimData/RealStateSpace_trans.csv"
            "${CMAKE_CURRENT_SOURCE_DIR}/SimData/RealStateSpace_rot.csv"
            "${CMAKE_CURRENT_SOURCE_DIR}/SimData/ControlData.csv"
            "${CMAKE_CURRENT_SOURCE_DIR}/SimData/TranslationalEstimation.csv"
            "${CMAKE_CURRENT_SOURCE_DIR}/SimData/RotationalEstimation.csv"
)

add_dependencies(ss_executable create_csv_files)