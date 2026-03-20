/*
 * @Author: your name
 * @Date: 2023-07-03 11:54:03
 * @LastEditTime: 2023-09-07 22:21:27
 * @LastEditors: your name
 * @Description: 用来接收carto发布的base_link和map之间的坐标变换，即定位信息
 * @FilePath: /MyFormProject/src/robot_locatization/include/cartographer_local/carto_locatization.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// 使用自定义消息类型
#include "robot_communication/localizationInfoBroadcast.h"
#include "robot_communication/sensorData.h"

using namespace std;

class carto_locatization
{
private:
  /* data */
  ros::NodeHandle cartoLocatization;       //carto定位节点
  // 如果没有私有节点，launch文件中的参数加载不进来，目前还不知道为什么，但是一定要像这样使用
  ros::NodeHandle private_node;  // ros中的私有句柄
  
  // 发布carto的定位消息
  ros::Publisher cartoLocalPub;
  // 发布carto的定位消息
  ros::Publisher cartOdomPub;
  // 发布里程计消息
  ros::Publisher OdomPub;
  // 订阅底盘速度消息
  ros::Subscriber chassDataSub;


  // 发布自定义位置消息以及相关消息
  robot_communication::localizationInfoBroadcast cartOdom;
  // 传感器消息
  robot_communication::sensorData chassisData;
  // 定义里程计消息用来给ros包生成局部代价地图
  nav_msgs::Odometry odom;

  /* data */
  double x,y,z, qx,qy,qz, qw;
  double theta;
  geometry_msgs::Pose2D pos_now;

  // 坐标变换
  tf::StampedTransform transform;
  // tf接收
  tf::TransformListener listener;

  // 订阅传感器消息
  void SensorDataCallback(const robot_communication::sensorDataConstPtr &sensor);
public:
  carto_locatization(/* args */);
  ~carto_locatization();
};



