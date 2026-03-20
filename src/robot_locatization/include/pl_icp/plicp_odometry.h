/*
 * Copyright 2020 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __SCAN_MATCH_PLICP
#define __SCAN_MATCH_PLICP

#include <cmath>
#include <vector>
#include <chrono>

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

// tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

// csm
#include <csm/csm_all.h>
#undef min
#undef max

#include "vel_transform/vel_transform.h"

class ScanMatchPLICP
{
private:
  ros::NodeHandle node_handle_;           // ros中的句柄
  ros::NodeHandle private_node_;          // ros中的私有句柄
  ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
  ros::Subscriber odom_subscriber;

  ros::Publisher odom_publisher_;         // 声明一个Publisher

  nav_msgs::Odometry odom_sensors;        //里程计数据
  std::string laserType;      //雷达类型：是仿真雷达还是实际雷达
  std::string odomType;      //里程计类型：是实际的里程计还是仿真中的里程计

  ros::Time last_icp_time_;
  ros::Time current_time_;

  geometry_msgs::Twist latest_velocity_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  tf2::Transform base_to_laser_;    
  tf2::Transform laser_to_base_; 

  tf2::Transform base_in_odom_;           // base_link在odom坐标系下的坐标
  tf2::Transform base_in_odom_keyframe_;  // base_link在odom坐标系下的keyframe的坐标

  // parameters
  bool initialized_;

  std::string odom_frame_;
  std::string base_frame_;

  double kf_dist_linear_;
  double kf_dist_linear_sq_;
  double kf_dist_angular_;

  double linear_x,linear_y,angular;   //定义线速度角速度   

  int kf_scan_count_;
  int scan_count_;

  std::vector<double> a_cos_;
  std::vector<double> a_sin_;

  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point end_time_;
  std::chrono::duration<double> time_used_;

  // csm
  sm_params input_;
  sm_result output_;
  LDP prev_ldp_scan_;

  // pl_icp参数初始化
  void InitParams();
  void CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  // 获取基坐标系到雷达坐标系的坐标变换
  bool GetBaseToLaserTf(const std::string &frame_id);
  void LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp);
  // 使用PLICP进行解算
  void ScanMatchWithPLICP(LDP &curr_ldp_scan, const ros::Time &time);
  // 获取预测位置
  void GetPrediction(double &prediction_change_x, double &prediction_change_y, double &prediction_change_angle, double dt);
  // 从XY 和 theta角创建Tf坐标变换
  void CreateTfFromXYTheta(double x, double y, double theta, tf2::Transform& t);
  // 发布坐标变换和里程计节点
  void PublishTFAndOdometry();
  bool NewKeyframeNeeded(const tf2::Transform &d);
public:
  Odom_data_define local;
  Odom_data_define global;

  ScanMatchPLICP();
  ~ScanMatchPLICP();

  // 激光雷达扫描点云数据回调函数
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
  // odom里程计数据回调函数
  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_sensor_msg);
};



#endif //

