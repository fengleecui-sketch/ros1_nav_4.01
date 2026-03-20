cmake_minimum_required(VERSION 3.0.2)
project(robot_communication)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  # 消息文件写在这下面

  # 定位消息广播
  localizationInfoBroadcast.msg
  # 运动规划部分发布底盘控制消息
  chassisControl.msg
  # 接收下位机控制器的各项传感器消息
  sensorData.msg
  # B样条消息
  Bspline.msg
  # esdf人工势场消息
  ESDFmap.msg
  # 目标点信息
  goal.msg
)

add_action_files(
  DIRECTORY action
  FILES DoDishes.action
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib_msgs
  actionlib
)

catkin_package()
