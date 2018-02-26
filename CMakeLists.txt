cmake_minimum_required (VERSION 2.8.11)
project (car_rrt_path_planning)

add_library(librrt RRT.cpp)

set(OpenCV_DIR /usr/local/share/OpenCV)

add_executable(rrt_test RRT_test.cpp)
find_package( OpenCV REQUIRED )
target_link_libraries(rrt_test librrt ${OpenCV_LIBS})
