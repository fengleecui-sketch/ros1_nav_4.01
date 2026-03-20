cmake_minimum_required(VERSION 3.0.2)
project(scan_to_cloud)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  roscpp
  sensor_msgs
  tf2_ros
  tf2_sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs laser_geometry tf2_ros tf2_sensor_msgs
)

add_executable(scan_to_cloud_node src/scan_to_cloud_node.cpp)
target_link_libraries(scan_to_cloud_node ${catkin_LIBRARIES})
add_dependencies(scan_to_cloud_node ${catkin_EXPORTED_TARGETS})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)