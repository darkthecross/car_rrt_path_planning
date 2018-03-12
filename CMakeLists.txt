cmake_minimum_required (VERSION 2.8.11)
project (car_rrt_path_planning)

set (CMAKE_CXX_STANDARD 11)

add_library(librrt RRT.cpp)
add_library(libkdtree kdTree.cpp)

set(OpenCV_DIR /usr/local/share/OpenCV)

add_executable(rrt_test RRT_test.cpp)
add_executable(kdTree_test kdTree_test.cpp)
find_package( OpenCV REQUIRED )
target_link_libraries(kdTree_test libkdtree ${OpenCV_LIBS})
target_link_libraries(rrt_test librrt libkdtree ${OpenCV_LIBS})
