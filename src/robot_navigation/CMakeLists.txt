cmake_minimum_required(VERSION 3.0.2)
project(robot_navigation)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

set(THIRD_PART_LIBRARIES "")
include(cmake/eigen.cmake)

# ROS配置
set(ROS_CXX_DEPENDENCIES
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
  pcl_ros
  visualization_msgs
  robot_communication
  message_generation
  robot_costmap
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})
find_package(OpenCV REQUIRED)
# 仅一次查找Boost，删除错误的geometry组件
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
# 查找OSQP求解器
find_package(OsqpEigen REQUIRED)

# 合并所有头文件路径（仅一次）
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/local/include
)

# 库文件路径
link_directories(/usr/local/lib)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS robot_costmap
)

# 自动扫描源码
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/nodes/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})

# ===================== 所有节点编译配置 =====================
add_executable(nav_node src/nodes/nav_node.cpp ${ALL_SRCS})
target_link_libraries(nav_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_executable(Astar_node src/nodes/Astar_node.cpp ${ALL_SRCS})
target_link_libraries(Astar_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_executable(rrt_node src/nodes/rrt_node.cpp ${ALL_SRCS})
target_link_libraries(rrt_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_executable(Hybrid_node src/nodes/Hybrid_node.cpp ${ALL_SRCS})
target_link_libraries(Hybrid_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_executable(Differential_DWA_node src/nodes/differential_DWA_node.cpp ${ALL_SRCS})
target_link_libraries(Differential_DWA_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_executable(Omnidirectional_DWA_node src/nodes/omnidirectional_DWA_node.cpp ${ALL_SRCS})
target_link_libraries(Omnidirectional_DWA_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

# 核心规划节点
add_executable(motionPlan_node src/nodes/motionPlan_node.cpp ${ALL_SRCS})
target_link_libraries(motionPlan_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_executable(pure_pursuit_node src/nodes/pure_pursuit_node.cpp ${ALL_SRCS})
target_link_libraries(pure_pursuit_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_executable(Hybrid_local_node src/nodes/Hybrid_local_node.cpp ${ALL_SRCS})
target_link_libraries(Hybrid_local_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_executable(test_edt_node src/nodes/test_edt_node.cpp ${ALL_SRCS})
target_link_libraries(test_edt_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_executable(dyn_planner_node src/nodes/dyn_planner_node.cpp ${ALL_SRCS})
target_link_libraries(dyn_planner_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_executable(traj_server src/nodes/traj_server_node.cpp ${ALL_SRCS})
target_link_libraries(traj_server
  ${catkin_LIBRARIES}
  /usr/local/lib/libnlopt.so
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  OsqpEigen::OsqpEigen
)