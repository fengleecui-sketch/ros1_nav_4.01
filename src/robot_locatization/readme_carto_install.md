# 主要是写一下cartographer在ubuntu20.04中的安装
1. 使用李太白的教程<Cartographer从入门到精通_ 原理深剖+源码逐行讲解【课件】>
2. 下载安装包，安装包下载之后，**需要修改auto-carto-build.sh中的：python-wstool、python-rosdep、python-sphinx改为python3-wstool、python3-rosdep、python3-sphinx**，这是因为Ubuntu20.04里面没有python了用的都是python3，并打开终端输入sudo chmod +x auto-carto-build.sh，赋予.sh文件权限
3. 在执行完以上步骤之后运行.sh文件，这里面说几个坑的地方，如果是新的系统，建议事先安装Cartographer，因为Carto已经停止维护，所以
## 介绍一下Cartographer源码和实际激光雷达的连接:
1. 修改cartographer_ros/configuration_files/revo_lds.lua文件,该文件的形式相当于yaml参数文件:
```c++
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  // 这个是carto原来的配置,网上是修改成了base_link,但是这里我们使用了一个节点,将激光雷达的link转换成了"laser_link"就无需修改了
  tracking_frame = "laser_link",
  published_frame = "laser_link",
  // tracking_frame = "base_link",
  // published_frame = "base_link",  
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 8.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
// 如果使用imu需要改为true 这里先只考虑激光雷达
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65

return options
```
2. 修改cartographer_ros/launch/demo_revo_lds.launch文件参数
```c++
<launch>
<!-- 如果接入自己的雷达这里要设定成false 仿真用true -->
  <param name="/use_sim_time" value="false" />
  <!------------------------------------------->
  <!这段相当于加载实际的机器人模型,这部分是carto中原来没有的,这里我们用carto中自带的urdf文件>
  <param name="robot_description"
    textfile="$(find cartographer_ros)/urdf/backpack_2d.urdf" />
  <node name="robot_state_publisher" pkg="robot_state_publisher"
    type="robot_state_publisher" />
  <!------------------------------------------->
  
  <node name="cartographer_node" pkg="cartographer_ros"
      type="cartographer_node" args="
          -configuration_directory $(find cartographer_ros)/configuration_files
          -configuration_basename revo_lds.lua"
      output="screen">
    <remap from="scan" to="scan" />
  </node>

  <node name="cartographer_occupancy_grid_node" pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node" args="-resolution 0.05" />

  <node name="rviz" pkg="rviz" type="rviz" required="true"
      args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz" />
  
  // 不跑数据集必须注释掉
  <!-- clocl前面是两个- -->
  <!-- 
  <node name="playbag" pkg="rosbag" type="play"
      args="-clock $(arg bag_filename)" />
  -->

</launch>
```
3. 需要自己写一个节点,因为激光雷达发布的话题是"scan",并且link或者说frame_id是"laser",对于carto来说无法进行坐标变换会出现问题,这个功能节点写到了**read_laser_data**功能包下,laser_carto_node.cpp
```c++
// 主要是接收原始雷达的话题简单修改以下并发布
// 话题发布
cartoLaserTopic = node_handle_.advertise<sensor_msgs::LaserScan>("scan",1);

// 回调函数
void LaserScan::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  cartoLaserData = *scan_msg;
  cartoLaserData.header.frame_id = "laser_link";
  cartoLaserTopic.publish(cartoLaserData);
}
```
做完以上工作之后,按照顺序启动以下节点:
a.roslaunch read_laser_data laser_change_param.launch //用来启动激光雷达和更改相应的雷达参数
b.roslaunch read_laser_data laser_carto.launch        //用来转换雷达的消息
c.在carto的工作空间下: roslaunch cartographer_ros demo_revo_lds.launch  //启动carto
成功的话![链接成功](carto单独激光雷达使用.png)

**至于后续更多的使用，请参考Robot_Demo下的Cartographer_Locatization中的readme_carto_use.md**


