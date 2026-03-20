#!/usr/bin/env bash
set -e

# 1) 清理旧进程（允许失败）
rosnode kill -a 2>/dev/null || true
killall -9 gzserver gzclient 2>/dev/null || true
killall -9 roscore rosmaster 2>/dev/null || true

yes | rosclean purge

sleep 2


# 2) 启动 roscore
gnome-terminal -t "roscore" -- bash -lc "roscore"

# 等待 roscore 就绪
until rostopic list >/dev/null 2>&1; do
  sleep 0.2
done

# 3) 启动 Gazebo + robot
gnome-terminal -t "gazebo_rviz_start" -- bash -lc "source ~/Gitkraken/ros1_noetic_nav/devel/setup.bash; roslaunch tri_steer_gazebo sim_gazebo_rviz.launch"

# 等待 Gazebo 节点起来（/gazebo/model_states 出现再继续）
until rostopic list 2>/dev/null | grep -q "/gazebo/model_states"; do
  sleep 0.5
done

# 4) 启动 teleop + robot
gnome-terminal -t "teleop_start" -- bash -lc "source ~/Gitkraken/ros1_noetic_nav/devel/setup.bash; roslaunch tri_steer_keyboard keyboard_tri_steer.launch"

# 5) 启动  robot gmapping
gnome-terminal -t "gmapping_start" -- bash -lc "source ~/Gitkraken/ros1_noetic_nav/devel/setup.bash; roslaunch robot_gmapping ros_gmapping.launch;exec bash "

