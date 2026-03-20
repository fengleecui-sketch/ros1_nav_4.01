
# 1) 保存地图
gnome-terminal -t "carto gmapping save" -- bash -c "cd /home/action/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash;rosservice call /write_state "{filename: '/home/action/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mapc.pbstream'}"; exec bash"
sleep 10
# 2）转换 pbstream → 栅格地图 保存到 Cartographer_Locatization 文件夹下
gnome-terminal -t "pbstream to pgm" -- bash -c "cd /home/action/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash;rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/action/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mapc.pbstream -map_filestem=/home/action/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mapc -resolution=0.1; exec bash"
sleep 10
# 2）转换 pbstream → 栅格地图 保存 ros1_slam 文件夹下
gnome-terminal -t "pbstream to pgm2" -- bash -c "cd /home/action/Cartographer/Cartographer_Locatization;source install_isolated/setup.bash;rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=/home/action/Cartographer/Cartographer_Locatization/src/cartographer_ros/cartographer_ros/map/carto_mapc.pbstream -map_filestem=/home/action/Gitkraken/ros1_slam/src/tri_steer_gazebo/map/carto_mapc -resolution=0.1; exec bash"