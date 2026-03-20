
# 1）获取时间
#export MAP_TIME=$(date +"%Y-%m-%d_%H-%M-%S")

# 2）获取备注
export MAP_TIME="map_gmapping"

# 3) 启动 保存地图
gnome-terminal -t "map_save" -- bash -lc "source ~/Gitkraken/ros1_noetic_nav/devel/setup.bash; roslaunch robot_gmapping start_map_server.launch"


rosservice call /write_state "{filename: '/home/action/carto_map.pbstream'}"
