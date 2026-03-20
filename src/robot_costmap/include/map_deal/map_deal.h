/*
 * @Author: your name
 * @Date: 2023-04-24 17:40:45
 * @LastEditTime: 2023-12-12 15:11:51
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_costmap/include/map_deal/map_deal.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __MAP_DEAL_H
#define __MAP_DEAL_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"

// tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include "Eigen/Eigen"
#include "iostream"

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "map_deal/grid_map_2d.h"
#include "map_deal/signed_distance_field_2d.h"
#include "map_deal/global_map_deal.h"
#include "map_deal/local_map_deal.h"
#include "map_deal/edt_environment.h"

// 包含自定义消息包
#include "robot_communication/localizationInfoBroadcast.h"
// 包含自定义地图包
#include "robot_communication/ESDFmap.h"

using namespace std;
using namespace Eigen;


namespace map_deal
{
class deal_all_map
{
private:
  /* data */
  ros::NodeHandle node_handle_;           // ros中的句柄
  ros::NodeHandle private_node_;          // ros中的私有句柄

  ros::Publisher local_map_publiser;      //  局部地图发布
  ros::Publisher local_map_inflate_publisher; //局部膨胀地图发布
  ros::Publisher local_map_esdf_publisher;    //局部esdf地图发布
  ros::Publisher local_esdf_map_publisher;    //局部esdf地图发布
  ros::Publisher local_map_cost_publisher;    //局部代价地图发布

  ros::Publisher test_cloud_map_publisher;    //局部代价地图发布

  ros::Publisher global_inflate_map_publiser;           //  全局地图发布
  ros::Publisher global_esdf_map_publisher;            //全局距离场地图发布
  ros::Publisher global_esdf_map_display_publisher;   //可以显示出来的esdf地图发布
  ros::Publisher global_cost_map_publisher;           //发布全局代价地图

  void LocalMapubliser(void);

  ros::Subscriber map_subscriber;  // hector发布的地图订阅
  void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map);
  ros::Subscriber localizationSub;        // 订阅当前车的姿态消息
  void localizationCallback(const nav_msgs::OdometryConstPtr &msg); //定位信息回调函数
  ros::Subscriber pointCloud2Sub;          //订阅点云消息
  void pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  ros::Subscriber laser2CloudSub;          //订阅点云消息
  void laser2Cloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &msg);
  // 订阅鼠标点击的位置
  ros::Subscriber clickSub;
  void clickCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  ros::Subscriber laser_scan_subscriber_; // 订阅雷达消息
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

  // 用于定时发布地图消息
  ros::Timer update_timer_;
  // 更新局部地图消息
  void updateLocalMapCallback(const ros::TimerEvent &e); // 0.1s定时器, 更新局部地图

  // 定时器发布全局距离场地图消息
  ros::Timer esdf_update_timer_;
  // 更新全局距离场地图消息
  void updateEsdfMapCallback(const ros::TimerEvent &e); // 0.1s定时器，更新地图

  // 定义
  nav_msgs::Odometry odom_;
  // 定义局部地图
  nav_msgs::OccupancyGrid localmap;
  // 定义局部膨胀地图
  nav_msgs::OccupancyGrid localinflatemap;  
  // 定义局部esdf地图
  nav_msgs::OccupancyGrid localesdfmap;  
  // 定义局部代价地图
  nav_msgs::OccupancyGrid localcostmap;  
  // 定义全局膨胀地图
  nav_msgs::OccupancyGrid globalinflatemap;
  // 定义全局esdf地图
  nav_msgs::OccupancyGrid globalesdfmap;
  // 定义全局cost地图
  nav_msgs::OccupancyGrid globalcostmap;
  // 定义全局原始地图
  nav_msgs::OccupancyGrid globalmap;

  // 定义自定义ESDF消息地图
  robot_communication::ESDFmap globalesdftest;
  // 定义自定义局部ESDF消息地图
  robot_communication::ESDFmap localesdftest;

private:
	double origin_x;
	double origin_y;
  // 需要的局部地图最大最小尺寸
  double map_x_min;
  double map_x_max;
  double map_y_min;
  double map_y_max;
  // 地图分辨率
  double resolution;
  // 分辨率的倒数
  double resolution_inv;
  // 激光雷达的扫描范围
  double laser_min_theta;
  double laser_max_theta;
  bool assemDirect;     //雷达安装方向
  // 膨胀半径
  double inflate;

  // 虚拟地图是实际地图倍数大小
  double virtual_map;

  double diffx;   // 局部地图x方向上的偏置
  double diffy;   // 局部地图y方向上的偏置

  // 定义膨胀地图层 该地图只保留了地图的有效部分
  std::vector<int8_t> occupancy_buffer_; // 0 is free, 1 is occupied
  // 膨胀地图层
  std::vector<int8_t> inflate_map_; // 0 is free, 1 is occupied
  // // 定义esdf地图层
  std::vector<double> esdf_buffer_;
  // // 有效地图的起点 x y
  int actual_map_startx,actual_map_starty;
  // 定义整数esdf地图
  std::vector<int32_t> esdf_map_buffer_;

  // 定义ROS可以发布的esdf地图
  std::vector<int8_t> esdf_map_display_;

  // 机器人姿态
  Eigen::Vector3d robotPose;
  // 定义障碍物容器
  // 局部地图中的障碍物集合
  vector<Vector2i> obstacleLocal;

  // 局部膨胀地图
  std::vector<int8_t> localinflate;
  
  // 局部点云数据
  pcl::PointCloud<pcl::PointXYZ> localcloud;

  // 激光雷达2的局部点云数据
  pcl::PointCloud<pcl::PointXYZ> laser2cloud;

  // 定义机器人的偏航角
  double robotYaw;

  bool is_use_sim = true;       //是否使用仿真，否就是真实情况下运行

  bool has_global_map = false;  //是否获取全局地图
  bool has_been_inflate = false;  //是否被膨胀
  bool has_been_esdf = false;   //是否是人工势场地图
  bool has_rece_cloud = false;  //判断是否接收到
  bool has_odom_flag = false;   //判断是否有里程计数据

  // 定义全局地图的x 和 y
  int global_map_x,global_map_y;
  // 有效地图部分 因为ros里面地图格式固定的，这里把地图值-1的部分去掉，只取有效部分
  int actual_map_x,actual_map_y;
  // // 有效地图的起点 x y
  // int actual_map_startx,actual_map_starty;

  // 参数初始化
  void InitParams(void);
  // 填充ESDF地图
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
  void setUpdateRange(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos);
  // 更新二维势场地图
  void updatedESDF2d(bool neg);   
  
  /**
   * @brief 栅格地图坐标系转世界坐标系
   * @param mx   地图坐标x
   * @param my   地图坐标y
   * @param wx   世界坐标x
   * @param wy   世界坐标y
   * @return
   * @attention
   * @todo
   * */
  Vector2d mapToWorld(Vector2i mapt) const
  {
    double wx,wy;
    wx = origin_x + (mapt[0] + 0.5) * resolution;
    wy = origin_y + (mapt[1] + 0.5) * resolution;

    return Vector2d(wx,wy);
  }

  /**
   * @brief 世界坐标系转栅格地图坐标系
   * @param wx   世界坐标x
   * @param wy   世界坐标y
   * @param mx   地图坐标x
   * @param my   地图坐标y
   * @return
   * @attention
   * @todo
   * */
  Vector2i worldToMap(Vector2d worldpt) const
  {
    int mx,my;

    mx = (int)(1.0 * (worldpt[0] - origin_x) / resolution);
    my = (int)(1.0 * (worldpt[1] - origin_y) / resolution);

    return Vector2i(mx,my);
  }

  // 全局地图处理智能指针
  global_map_deal::Ptr global_map_;
  // 局部地图处理只能指针
  local_map_deal::Ptr local_map_;
  // 梯度
  // EDTEnvironment * edt_environment = new EDTEnvironment();
  EDTEnvironment::Ptr edt_environment;

public:
  deal_all_map(/* args */);
  ~deal_all_map();
};
}

#endif  //
