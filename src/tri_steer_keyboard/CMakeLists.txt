cmake_minimum_required(VERSION 3.0.2)
project(tri_steer_keyboard)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/tri_steer_keyboard.py
  scripts/tri_steer_kinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
