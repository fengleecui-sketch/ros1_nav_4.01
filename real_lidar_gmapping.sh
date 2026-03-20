# <!-- 使用cartograher算法进行建图 -->

# 1) 启动 roscore
gnome-terminal --title="roscore" -- bash -c "roscore"
# 等待 roscore 就绪
until rostopic list >/dev/null 2>&1; do
  sleep 0.2
done
# 2) 启动雷达
gnome-terminal --title="lider_start" -- bash -c "source ~/Gitkraken/ws_livox/devel/setup.bash; roslaunch livox_ros_driver2 rviz_msg_MID360.launch rviz_enable:=false"
sleep 2
# 3) 激光雷达数据转换成2d
gnome-terminal --title="lider_msg_convert" -- bash -c "source ~/Gitdesktop/ros1_slam/devel/setup.bash; roslaunch livox_cloud_to_scan cloud_to_scan.launch"
# 4）启动cartographer建图节点
gnome-terminal -t "carto gmapping" -- bash -c "cd /home/action/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash;roslaunch cartographer_ros my_robot_2d_mid360.launch;exec bash"

# rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/livox/lidar scan:=/scan