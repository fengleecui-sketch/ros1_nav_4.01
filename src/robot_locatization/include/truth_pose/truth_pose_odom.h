/*
 * @Author: your name
 * @Date: 2023-04-27 10:50:35
 * @LastEditTime: 2023-05-18 14:02:57
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /Robot_Formation/src/robot_locatization/include/truth_pose/truth_pose_odom.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __TRUTH_POSE_ODOM_H
#define __TRUTH_POSE_ODOM_H

#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "chrono"
#include "iostream"

// tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

// 使用自定义消息类型
#include "robot_communication/localizationInfoBroadcast.h"
// 需要使用局部速度向全局速度转变
#include "vel_transform/vel_transform.h"

using namespace std;
using namespace chrono;

class truth_pose_odom
{
private:
  ros::NodeHandle truth_pose_node;           // ros中的句柄

  ros::Subscriber truth_pose_subscriber;    // 真值里程计消息订阅
  ros::Publisher truth_pose_publisher;      // 真值里程计消息发布

  ros::Publisher carto_odomDataPub;          //真值里程计自定义消息发布
  // 发布自定义位置消息以及相关消息
  robot_communication::localizationInfoBroadcast odomDataMsg;

  nav_msgs::Odometry truth_odom_msg;              // 里程计话题发布

public:
  truth_pose_odom(/* args */);
  ~truth_pose_odom();

  // 模型位置真值获取回调函数
  void modelcarto_odomCallback(const nav_msgs::Odometry::ConstPtr& gazebomsg);
};



#endif //__TRUTH_POSE_ODOM_H
