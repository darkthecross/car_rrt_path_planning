cmake_minimum_required (VERSION 2.8.11)
project (car_rrt_path_planning)

add_library(librrt RRT.cpp)

add_executable(rrt_test RRT_test.cpp)
target_link_libraries(rrt_test librrt)
