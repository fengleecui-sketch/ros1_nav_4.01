#!/usr/bin/env bash
source ~/.bashrc
set -e
# 1) 启动 roscore
gnome-terminal --title="roscore" -- bash -c "roscore"
# 等待 roscore 就绪
until rostopic list >/dev/null 2>&1; do
  sleep 0.2
done
# 2) 启动 Gazebo + robot
gnome-terminal --title="rviz_start" -- bash -c "source ~/Gitdesktop/ros1_nav2026/devel/setup.bash; roslaunch tri_steer_gazebo real_slamRun.launch; exec bash"
# 3) 启动雷达
gnome-terminal --title="lider_start" -- bash -c "source ~/Gitkraken/ws_livox/devel/setup.bash; roslaunch livox_ros_driver2 rviz_msg_MID360.launch rviz_enable:=false"
# sleep 2

# 3) 激光雷达数据转换成2d
gnome-terminal --title="lider_msg_convert" -- bash -c "source ~/Gitdesktop/ros1_nav2026/devel/setup.bash; roslaunch livox_cloud_to_scan cloud_to_scan.launch"

# 4）算法重定位
gnome-terminal -t "carto_relocalize" -- bash -lc "cd /home/action/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash; roslaunch cartographer_ros my_robot_2d_mid360_localization.launch; exec bash"
# 解析carto算法发布的tf变换到机器人
gnome-terminal -t "carto_odom_start" -- bash -lc "source ~/Gitdesktop/ros1_nav2026/devel/setup.bash; roslaunch carto_tf_to_odom carto_tf_to_odom_acl.launch; exec bash"
# 等待 Cartographer 发布 /map（或你也可以等 TF map->odom）
 until rostopic list 2>/dev/null | grep -q "^/map$"; do
    sleep 0.5
 done
#订阅雷达的消息转化为cloud
 gnome-terminal -t "cloud start" -- bash -lc "source ~/Gitdesktop/ros1_nav2026/devel/setup.bash; roslaunch scan_to_cloud scan_to_cloud.launch; exec bash"
# 5）启动地图节点
 gnome-terminal -t "map start" -- bash -lc "t; exec bash"
# 6) 运动规划节点
 gnome-terminal -t "motion Plan start" -- bash -lc "source ~/Gitdesktop/ros1_nav2026/devel/setup.bash; roslaunch robot_navigation motionPlan_acl.launch"
# 7）局部规划pid算法
 gnome-terminal -t "PID_control" -- bash -c "source ~/Gitdesktop/ros1_nav2026/devel/setup.bash; roslaunch robot_pid_local_planner Omnidirectional_PID.launch; exec bash"


# 8）订阅控制消息
#gnome-terminal -t "control start" -- bash -lc "source ~/Gitdesktop/ros1_nav2026/devel/setup.bash; roslaunch tri_steer_drive bringup.launch use_teleop:=true;exec bash"
# 9）通过串口下发上位机控制指令
gnome-terminal -t "usart msgPub" -- bash -lc "source ~/Gitdesktop/ros1_nav2026/devel/setup.bash; roslaunch robot_usart usart_node.launch;exec bash"