cmake_minimum_required(VERSION 3.0.2)
project(robot_costmap)

#cmake 编译配置
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

#第三方库配置
set(THIRD_PART_LIBRARIES "")

include(cmake/eigen.cmake)

set(ROS_CXX_DEPENDENCIES
  amcl
  geometry_msgs
  gmapping
  laser_geometry
  map_server
  move_base
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  robot_communication #添加自定义通讯包
)

find_package(catkin  REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )
find_package( OpenCV REQUIRED ) 

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_costmap
  CATKIN_DEPENDS roscpp std_msgs
)

#自定义库设置
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/nodes/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})           #当有多个node 文件的时候使用这句话

add_library( robot_costmap
  src/map_deal/edt_environment.cpp
  src/map_deal/grid_map_2d.cpp 
  src/map_deal/signed_distance_field_2d.cpp
  src/map_deal/global_map_deal.cpp
  src/map_deal/local_map_deal.cpp
  )

## 对局部地图进行简单处理
add_executable(map_deal_node src/nodes/map_deal_node.cpp 
                                    src/map_deal/map_deal.cpp 
                                    src/map_deal/grid_map_2d.cpp 
                                    src/map_deal/signed_distance_field_2d.cpp
                                    src/map_deal/global_map_deal.cpp
                                    src/map_deal/local_map_deal.cpp
                                    src/map_deal/edt_environment.cpp
                                    )
target_link_libraries(map_deal_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES} ${OpenCV_LIBS})


