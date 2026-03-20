cmake_minimum_required(VERSION 3.0.2)   #定义cmake需求版本和工程名称
project(robot_navigation)

#cmake 编译配置
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

#第三方库配置
set(THIRD_PART_LIBRARIES "")

include(cmake/eigen.cmake)

# ROS配置
set(  ROS_CXX_DEPENDENCIES
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
  robot_communication #添加自定义通讯包
  message_generation
  robot_costmap
)

find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES} )
find_package( OpenCV REQUIRED ) 

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS robot_costmap
#  DEPENDS system_lib
)  

#自定义库设置
file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/nodes/*_node.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})           #当有多个node 文件的时候使用这句话

# 目前AStar JPS  RRT三种算法测试的demo文件
add_executable(nav_node src/nodes/nav_node.cpp ${ALL_SRCS} )
target_link_libraries(nav_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES} 
  ${OpenCV_LIBS}
  )

add_executable(Astar_node src/nodes/Astar_node.cpp ${ALL_SRCS} )
target_link_libraries(Astar_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES} 
  ${OpenCV_LIBS}
  )

add_executable(rrt_node src/nodes/rrt_node.cpp ${ALL_SRCS} )
target_link_libraries(rrt_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES} 
  ${OpenCV_LIBS}
  )

# hybrid astar 测试节点
add_executable(Hybrid_node src/nodes/Hybrid_node.cpp ${ALL_SRCS} )
target_link_libraries(Hybrid_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  )

# 差速DWA 测试节点
add_executable(Differential_DWA_node src/nodes/differential_DWA_node.cpp ${ALL_SRCS} )
target_link_libraries(Differential_DWA_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  )

# 全向DWA 测试节点
add_executable(Omnidirectional_DWA_node src/nodes/omnidirectional_DWA_node.cpp ${ALL_SRCS} )
target_link_libraries(Omnidirectional_DWA_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  )

# 规划节点
add_executable(motionPlan_node src/nodes/motionPlan_node.cpp ${ALL_SRCS} )
target_link_libraries(motionPlan_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  )

# 跟随节点
add_executable(pure_pursuit_node src/nodes/pure_pursuit_node.cpp ${ALL_SRCS} )
target_link_libraries(pure_pursuit_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  )

# 跟随节点
add_executable(Hybrid_local_node src/nodes/Hybrid_local_node.cpp ${ALL_SRCS} )
target_link_libraries(Hybrid_local_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  )

# 测试人工势场的节点
add_executable(test_edt_node src/nodes/test_edt_node.cpp ${ALL_SRCS} )
target_link_libraries(test_edt_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  )

# fast planner 的规划节点但是还没成功
add_executable(dyn_planner_node src/nodes/dyn_planner_node.cpp ${ALL_SRCS})
target_link_libraries(dyn_planner_node 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  )

# fast planner 的跟随节点但是还没成功
add_executable(traj_server src/nodes/traj_server_node.cpp ${ALL_SRCS})
target_link_libraries(traj_server 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libnlopt.so 
  ${THIRD_PART_LIBRARIES}
  ${OpenCV_LIBS}
  )
