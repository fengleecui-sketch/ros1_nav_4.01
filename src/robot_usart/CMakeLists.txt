cmake_minimum_required(VERSION 3.0.2)
project(robot_usart)

#cmake 编译配置
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

#第三方库配置
set(THIRD_PART_LIBRARIES "")
include(cmake/eigen.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  serial
  std_msgs
  robot_communication
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()

#自定义库设置
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})           #当有多个node 文件的时候使用这句话

#
add_executable(usart_node src/usart_node.cpp src/crc.cpp src/usart_config.cpp)
target_link_libraries(usart_node ${catkin_LIBRARIES} )

add_library(easylog SHARED src/easylog/easylogging++.cc)
target_link_libraries( usart_node  easylog)

