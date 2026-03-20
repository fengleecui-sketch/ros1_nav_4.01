/*
 * @Author: your name
 * @Date: 2023-07-25 11:52:57
 * @LastEditTime: 2023-07-25 12:21:57
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/src/nodes/b_traj_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>




// 接收点云数据回调函数
void rcyPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // 获取点云数据
  pcl::fromROSMsg(pointcloud_map,cloud);

  if ((int)cloud.points.size() == 0)
  {
    return;
  }
  
  // 删除碰撞地图
  // delete collision_map_local;

  ros::Time time_1 = ros::Time::now();

  // 碰撞地图复位
  // collision_map->RestMap();
  
  
}


