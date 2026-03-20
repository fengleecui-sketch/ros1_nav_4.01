cmake_minimum_required(VERSION 3.0.2)
project(livox_cloud_to_scan)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  tf2_ros
  tf2_eigen
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros pcl_conversions tf2_ros tf2_eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(cloud_to_scan_node
  src/cloud_to_scan_node.cpp
)

target_link_libraries(cloud_to_scan_node
  ${catkin_LIBRARIES}
)

add_dependencies(cloud_to_scan_node
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS cloud_to_scan_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
