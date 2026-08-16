cmake_minimum_required(VERSION 3.16)
project(uav_motion_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(uav_planner
    src/csv_writer.cpp
    src/kinodynamic_planner.cpp
    src/planner_config.cpp
    src/uav_planner.cpp
)
target_include_directories(uav_planner PUBLIC include)

add_executable(uav_planner_demo src/main.cpp)
target_link_libraries(uav_planner_demo PRIVATE uav_planner)

enable_testing()
add_executable(uav_planner_tests tests/test_planner.cpp)
target_link_libraries(uav_planner_tests PRIVATE uav_planner)
add_test(NAME planner_tests COMMAND uav_planner_tests)
