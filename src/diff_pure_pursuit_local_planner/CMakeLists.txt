cmake_minimum_required(VERSION 3.0.2)
project(diff_pure_pursuit_local_planner)

## 使用 C++14（Noetic 推荐）
add_compile_options(-std=c++14)

## 找 catkin + 依赖包（robot_communication 是你的自定义 msg 包）
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  robot_communication
)

###################################
## catkin package
###################################
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs tf2 tf2_ros robot_communication
)

###################################
## include dirs
###################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

###################################
## build target
###################################
add_executable(diff_pure_pursuit_node
  src/diff_pure_pursuit_node.cpp
)

target_link_libraries(diff_pure_pursuit_node
  ${catkin_LIBRARIES}
)

## 确保消息/头文件生成顺序正确（尤其是自定义 msg）
add_dependencies(diff_pure_pursuit_node
  ${catkin_EXPORTED_TARGETS}
)

###################################
## install (可选，但建议保留)
###################################
install(TARGETS diff_pure_pursuit_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
