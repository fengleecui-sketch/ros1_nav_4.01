/*
 * @Author: your name
 * @Date: 2023-04-27 10:39:55
 * @LastEditTime: 2024-02-01 16:11:09
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_locatization/src/truth_pose/truth_pose_odom.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "truth_pose/truth_pose_odom.h"


truth_pose_odom::truth_pose_odom(/* args */)
{
  // 订阅真值消息
  truth_pose_subscriber = truth_pose_node.subscribe("carto_odom",10,&truth_pose_odom::modelcarto_odomCallback,this);
  // 消息发布
  carto_odomDataPub = truth_pose_node.advertise<robot_communication::localizationInfoBroadcast>("truth_pose_odom",50);


  ros::Rate loop_rate(50);
  while(ros::ok())
  {
    auto timeSatrt = steady_clock::now();
    
    loop_rate.sleep();
    ros::spinOnce();
    
    //--运行速度测量
    auto timeEnd = steady_clock::now();
    auto timeDuration = duration_cast<microseconds>(timeEnd - timeSatrt);
  }
}

truth_pose_odom::~truth_pose_odom()
{
}

void truth_pose_odom::modelcarto_odomCallback(const nav_msgs::Odometry::ConstPtr& gazebomsg)
{
  // gazebo发布的世界真值里程计消息是把当前模型当做一个质点的理想状态，发布的速度加速度均是世界坐标系下的速度和
  // 加速度

  double roll,pitch,yaw;
  // 四元数向欧拉角转换，当前车的角度
  tf2::Quaternion quat(
                        gazebomsg->pose.pose.orientation.x,
                        gazebomsg->pose.pose.orientation.y,
                        gazebomsg->pose.pose.orientation.z,
                        gazebomsg->pose.pose.orientation.w
                       );
  // 四元数向欧拉角进行转换 对local的角度
  tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);

  // 得出全局位置
  odomDataMsg.xPosition = gazebomsg->pose.pose.position.x;
  odomDataMsg.yPosition = gazebomsg->pose.pose.position.y;
  // 得出全局速度
  odomDataMsg.xSpeed = gazebomsg->twist.twist.linear.x;
  odomDataMsg.ySpeed = gazebomsg->twist.twist.linear.y;
  // 得出角度和角速度
  odomDataMsg.chassisAngle = yaw;
  odomDataMsg.chassisGyro = gazebomsg->twist.twist.angular.z;
  // 得出全局加速度
  odomDataMsg.xAccel = 0;
  odomDataMsg.yAccel = 0;
  // 发布消息
  carto_odomDataPub.publish(odomDataMsg);
}

