#!/usr/bin/env bash
source ~/.bashrc

sleep 2

# 2) 启动 roscore
#ros_term "roscore" "roscore"
gnome-terminal --title="roscore" -- bash -c "roscore"

# 等待 roscore 就绪
until rostopic list >/dev/null 2>&1; do
  sleep 0.2
done

#  # 3) 启动 Gazebo + robot
# gnome-terminal --title="gazebo_start" -- bash -c "source ~/ros1_slam/devel/setup.bash; roslaunch tri_steer_gazebo carto_test.launch"

# # # 7) 雷达点云反转成水平车体下的point-cloud2
# gnome-terminal -t "laser to cloud" -- bash -lc "source ~/ros1_slam/devel/setup.bash; roslaunch read_laser_data laser_to_cloud.launch; exec bash"

# # 10）订阅控制消息
# gnome-terminal -t "control  start" -- bash -lc "source ~/ros1_slam/devel/setup.bash; roslaunch tri_steer_drive bringup.launch use_teleop:=true;exec bash"

# # 3) 启动 激光雷达
gnome-terminal --title="lider_start" -- bash -c \
"source ~/ws_livox/devel/setup.bash;roslaunch livox_ros_driver2 rviz_msg_MID360.launch rviz_enable:=false"
# # roslaunch livox_ros_driver2 msg_MID360.launch"rviz_MID360

# sleep 2

# # 3) 激光雷达数据转换成2d
gnome-terminal --title="lider_msg_convert" -- bash -c \
"source ~/ros1_slam/devel/setup.bash; 
roslaunch livox_cloud_to_scan cloud_to_scan.launch"

# 启动cartographer建图节点
gnome-terminal -t "carto gmapping" -- bash -c "cd /home/action/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash;roslaunch cartographer_ros my_robot_2d.launch;exec bash"

echo "Gmapping Starting Success!"
