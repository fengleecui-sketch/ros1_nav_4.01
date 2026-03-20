# gnome-terminal -t "roskill" -x bash -c "rosnode kill --all"
# # 延时2s确保节点注册成功
# sleep 2s

# #启动roscore
gnome-terminal -t "roscore" -x bash -c "roscore;exec bash;"
# # 延迟5s确保节点注册成功
# sleep 5s

# 激光雷达参数配置节点
gnome-terminal -t "laser set" -- bash -c "source ~/Gitkraken/ws_livox/devel/setup.bash;roslaunch read_laser_data laser1_change_param.launch;exec bash"

# 启动cartographer建图节点
gnome-terminal -t "carto gmapping" -- bash -c \
"cd /home/action/Cartographer/Cartographer_Locatization;
source install_isolated/setup.bash;
roslaunch cartographer_ros my_robot_2d.launch scan_topic:=/not_true_scan;exec bash"

echo "Gmapping Starting Success!"

wait

exit 0

# 保存地图
cd /home/action/Cartographer/Cartographer_Locatization
source install_isolated/setup.bash
rosservice call /write_state "{filename: '/home/action/carto_map.pbstream'}"

#将carto_map.pbstream文件转化为pgm和yaml文件
