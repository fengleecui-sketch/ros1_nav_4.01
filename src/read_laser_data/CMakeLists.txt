cmake_minimum_required(VERSION 3.0.2)
project(read_laser_data)

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
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf
  laser_geometry
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()

#自定义库设置
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})           #当有多个node 文件的时候使用这句话


add_executable(laser_scan_node src/laser_scan_node.cpp ${ALL_SRCS} )
target_link_libraries(laser_scan_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES} )

add_executable(feature_detection_node src/feature_detection_node.cpp ${ALL_SRCS} )
target_link_libraries(feature_detection_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES} )

add_executable(laser_carto_node src/laser_carto_node.cpp ${ALL_SRCS} )
target_link_libraries(laser_carto_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES} )

add_executable(laser1_to_cloud_node src/laser1_to_cloud_node.cpp ${ALL_SRCS} )
target_link_libraries(laser1_to_cloud_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES} )

add_executable(laser2_to_cloud_node src/laser2_to_cloud_node.cpp ${ALL_SRCS} )
target_link_libraries(laser2_to_cloud_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES} )

add_executable(laser_static_pub_node src/laser_static_pub_node.cpp ${ALL_SRCS} )
target_link_libraries(laser_static_pub_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES} )
