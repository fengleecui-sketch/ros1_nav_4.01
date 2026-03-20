cmake_minimum_required(VERSION 3.0.2)
project(robot_gmapping)
###########################################
# 该CMakeLists.txt与别的文件略有不同，不要修改会出问题
###########################################
#cmake 编译配置
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

#第三方库配置
set(THIRD_PART_LIBRARIES "")
include(cmake/eigen.cmake)

find_package(catkin REQUIRED COMPONENTS
  laser_geometry
  nav_msgs
  roscpp
  sensor_msgs
  tf
  tf_conversions
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()


## occupancy_grid
add_executable(occupancy_grid_node src/occupancy_grid/occupancy_grid.cpp)
target_link_libraries(occupancy_grid_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES})

# GMapping map
add_executable(gmapping_node src/gmapping/gmapping.cpp)
target_link_libraries(gmapping_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${THIRD_PART_LIBRARIES})

## Hector map
add_executable(hector_mapping_node src/hector_mapping/hector_mapping.cpp)
target_link_libraries(hector_mapping_node ${catkin_LIBRARIES} ${THIRD_PART_LIBRARIES})

## Hector SLAM 
add_executable(hector_slam_node src/hector_slam/hector_slam.cpp)
target_link_libraries(hector_slam_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

