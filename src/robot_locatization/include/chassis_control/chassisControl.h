/*
 * @Author: your name
 * @Date: 2023-05-06 21:01:31
 * @LastEditTime: 2023-05-20 17:20:45
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /Robot_Formation/src/robot_locatization/include/chassis_control/chassisControl.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __CHASSIS_CONTROL_H
#define __CHASSIS_CONTROL_H

#include "iostream"
using namespace std;
#include "chrono"
using namespace chrono;

#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include "chrono"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/convert.h"
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include "tf2_ros/transform_broadcaster.h"

#include "vel_transform/vel_transform.h"
#include "robot_communication/chassisControl.h"
#include "robot_communication/localizationInfoBroadcast.h"

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

typedef struct PIDParameterStruct
{
  float P;   //比例常数
  float I;   //积分常数
  float D;   //微分常数

  float SetPoint;  //设定数值
  float feedPoint; //反馈数值
  float OutPoint;  //输出结果

  float OutMax;    //输出限幅
  float OutMin;    //输出限幅

  float DiffError; //微分
  float IntError;  //积分
  float IntLimt;   //积分限幅
  float NowError[3]; //误差值
  float LastfeedPoint; //最终输出数值
}PIDFloatDefine;

void SetPIDParameter(PIDFloatDefine *p_pid,float *A1);
void PIDFloatPositionCal(PIDFloatDefine *p_pid);
// 过圈处理
void Deal_Super_Circle(float *setangle,float *feedangle);

class chassisControl
{
private:
  /* data */
  ros::NodeHandle chassisCtl;       //底盘速度控制节点
  // 如果没有私有节点，launch文件中的参数加载不进来，目前还不知道为什么，但是一定要像这样使用
  ros::NodeHandle private_node;  // ros中的私有句柄
  
  // 全局速度消息订阅
  ros::Subscriber globalVelSub;  

  /* data */
  ros::Publisher chassCtlPub;       //底盘控制
  geometry_msgs::Twist my_twist;    //速度控制节点

  // 车体局部速度和全局速度
  Odom_data_define local;
  Odom_data_define global;

  // 理想状态下的里程计数据
  robot_communication::localizationInfoBroadcast truthOdom;   //定位信息
  // 获取速度数据
  nav_msgs::Odometry odoMsg;

  // 当前yaw;
  double nowYaw;
  // 全局速度回调函数用于接收全局速度消息并处理
  void GlobalVelocityCallback(const robot_communication::chassisControlConstPtr &msg);
  // 一般状态下里程计消息订阅
  ros::Subscriber odom_sub;
  // 正常里程计消息回调函数
  void odom_callback(const nav_msgs::OdometryConstPtr &msg);
  ros::Subscriber truth_odom_sub;  //订阅定位信息
  void truthOdomCallback(const robot_communication::localizationInfoBroadcastConstPtr &msg); //定位信息回调函数
public:
  chassisControl(/* args */);
  ~chassisControl();
};




#endif //
