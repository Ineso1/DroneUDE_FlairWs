project(Place_project)

add_executable(
    place_executable 
    main.cpp 
    place_function.cpp
)

target_link_libraries(
    place_executable 
    Eigen3::Eigen
)
