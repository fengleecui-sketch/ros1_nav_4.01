cmake_minimum_required(VERSION 3.0.2)
project(robot_locatization)

#cmake 编译配置
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

#第三方库配置
set(THIRD_PART_LIBRARIES "")
include(cmake/csm.cmake)
include(cmake/eigen.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  tf
  robot_communication #添加自定义通讯包
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )
include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()

#自定义库设置
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/nodes/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})           #当有多个node 文件的时候使用这句话

# plicp_odometry_node
add_executable(plicp_odometry_node src/nodes/plicp_odometry_node.cpp ${ALL_SRCS})
target_link_libraries(plicp_odometry_node ${catkin_LIBRARIES} ${csm_LIBRARIES} ${THIRD_PART_LIBRARIES})

# truth_pose_odom_node
add_executable(truth_pose_odom_node src/nodes/truth_pose_odom_node.cpp ${ALL_SRCS})
target_link_libraries( truth_pose_odom_node ${catkin_LIBRARIES} ${csm_LIBRARIES} ${THIRD_PART_LIBRARIES})

# chassisControl_node
add_executable(chassisControl_node src/nodes/chassisControl_node.cpp ${ALL_SRCS})
target_link_libraries( chassisControl_node ${catkin_LIBRARIES} ${csm_LIBRARIES} ${THIRD_PART_LIBRARIES})

# cartographer_locatization_node
add_executable(carto_locatization_node src/nodes/carto_locatization_node.cpp ${ALL_SRCS})
target_link_libraries( carto_locatization_node ${catkin_LIBRARIES} ${csm_LIBRARIES} ${THIRD_PART_LIBRARIES})