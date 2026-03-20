cmake_minimum_required(VERSION 3.0.2)
project(robot_pid_local_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
  robot_communication
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(omnidirectional_pid_local_planner_node
  src/omnidirectional_pid_local_planner_node.cpp
)

target_link_libraries(omnidirectional_pid_local_planner_node
  ${catkin_LIBRARIES}
)

install(TARGETS omnidirectional_pid_local_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
