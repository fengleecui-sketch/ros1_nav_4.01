#!/usr/bin/env bash
source ~/.bashrc

set -e

# 1) 清理旧进程（允许失败）
rosnode kill -a 2>/dev/null || true
killall -9 gzserver gzclient 2>/dev/null || true
killall -9 roscore rosmaster 2>/dev/null || true

yes | rosclean purge

sleep 2

# 2) 启动 roscore
#ros_term "roscore" "roscore"
gnome-terminal --title="roscore" -- bash -c "roscore"

# 等待 roscore 就绪
until rostopic list >/dev/null 2>&1; do
  sleep 0.2
done


# 3) 启动 激光雷达
gnome-terminal --title="lider_start" -- bash -c \
"source ~/Gitkraken/ws_livox/devel/setup.bash;

roslaunch livox_ros_driver2 rviz_msg_MID360.launch rviz_enable:=false"
# roslaunch livox_ros_driver2 msg_MID360.launch"rviz_MID360

sleep 2

# gnome-terminal --title="mapping_start" -- bash -c \
# "source ~/Gitkraken/ws_livox/devel/setup.bash; 
# roslaunch fast_lio mapping_mid360.launch

# 3) 激光雷达数据转换成2d
gnome-terminal --title="lider_msg_convert" -- bash -c \
"source ~/Gitkraken/ros1_slam/devel/setup.bash; 
roslaunch livox_cloud_to_scan cloud_to_scan.launch"

