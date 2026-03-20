/*
 * @Author: your name
 * @Date: 2023-10-16 21:29:38
 * @LastEditTime: 2023-10-18 19:49:28
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/read_laser_data/src/laser2_to_cloud_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include"ros/ros.h"
#include "iostream"
#include "sensor_msgs/LaserScan.h"
#include"sensor_msgs/PointCloud.h"
#include"laser_geometry/laser_geometry.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

using namespace std;

laser_geometry::LaserProjection projector;
ros::Publisher scan_pub;

// 雷达安装的x y z方向的位置
double laser_x,laser_y,laser_z;
// 雷达安装的欧拉角 数值范围 -pi到pi
double laser_roll,laser_pitch,laser_yaw;

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
  sensor_msgs::PointCloud2 cloud;
  projector.projectLaser(*scan_in,cloud);
  // scan_pub.publish(cloud);

  // 局部点云数据
  pcl::PointCloud<pcl::PointXYZ> laser2cloud;
  // 将pointCloud转换为pcl::PointCloud
  pcl::fromROSMsg(cloud,laser2cloud);

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud_out;
  pcl_cloud_out.header.frame_id = "base_link";

  // cout<<"laser x"<<laser_x<<endl;
  // cout<<"laser y"<<laser_y<<endl;

  // cout<<"laser yaw"<<laser_yaw<<endl;

  Eigen::Affine3f transformation_matrix = Eigen::Affine3f::Identity();
  // 旋转操作（绕 Z 轴旋转 theta 弧度）
  transformation_matrix.rotate(Eigen::AngleAxisf(laser_yaw,Eigen::Vector3f::UnitZ()));
  // 平移操作（x, y, z 表示位移向量）
  transformation_matrix.translation() << laser_x, laser_y, laser_z;

  for (const pcl::PointXYZ& point : laser2cloud) {
    // 执行坐标变换
    pcl::PointXYZ transformed_point;
    Eigen::Vector4f input_point(point.x, point.y, point.z, 1.0);
    Eigen::Vector4f output_point = transformation_matrix * input_point;
    transformed_point.x = output_point[0];
    transformed_point.y = output_point[1];
    transformed_point.z = output_point[2];

    pcl_cloud_out.points.push_back(transformed_point);
  }  
  
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(pcl_cloud_out,ros_cloud);
  ros_cloud.header.frame_id = "base_link";

  scan_pub.publish(ros_cloud);
}
 
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_scan_to_pointcloud");
  ros::NodeHandle nh("~");

  nh.param("laser_x",laser_x,0.0);
  nh.param("laser_y",laser_y,0.0);
  nh.param("laser_z",laser_z,0.25);

  nh.param("laser_roll",laser_roll,0.0);
  nh.param("laser_pitch",laser_pitch,0.0);
  nh.param("laser_yaw",laser_yaw,0.0);  

  // cout<<"laser x:"<<laser_x<<endl;
  // cout<<"laser y:"<<laser_y<<endl;

  // cout<<"laser yaw:"<<laser_yaw<<endl;

  scan_pub=nh.advertise<sensor_msgs::PointCloud2>("/cloud_map2",1);

  ros::Subscriber sub = nh.subscribe("/scan2", 10, ScanCallback);

  ros::spin();
  return 0;
}
 