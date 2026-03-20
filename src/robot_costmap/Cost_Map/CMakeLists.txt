cmake_minimum_required(VERSION 3.0)

project(COSTMAP)

# 用于查找代码中是否有警告
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -g  -O2  -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -Wall")
set (CMAKE_BUILD_TYPE Debug)

# 注意include 前面/ 空格
include_directories(${CMAKE_SOURCE_DIR}/ include)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(cost_map main.cpp src/cost_map.cpp)
target_link_libraries(cost_map ${OpenCV_LIBS})
