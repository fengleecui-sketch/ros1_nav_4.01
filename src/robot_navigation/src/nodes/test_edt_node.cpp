/*
 * @Author: your name
 * @Date: 2023-08-18 11:39:00
 * @LastEditTime: 2023-08-19 17:54:58
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/src/nodes/test_edt_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

#include "map_deal/edt_environment.h"

using namespace map_deal;

global_map_deal::Ptr global_map_;
EDTEnvironment::Ptr edt_environment;

// 订阅鼠标点击的位置
ros::Subscriber clickSub;
void clickCallback(const geometry_msgs::PoseStampedConstPtr &msg);

// 订阅鼠标点击点
void clickCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  double dist;
  dist = edt_environment->evaluateCoarseEDT(Vector2d(msg->pose.position.x,msg->pose.position.y),-1);
  cout<<"position: "<<msg->pose.position.x<<"  "<<msg->pose.position.y<<"   "<<"esdf "<<dist<<endl;
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "test_edt_node");
  ros::NodeHandle nh("~");

  global_map_.reset(new global_map_deal);
  global_map_->init(nh);

  edt_environment.reset(new EDTEnvironment);
  edt_environment->setMap(global_map_);

  // 订阅鼠标点击目标点
  clickSub = nh.subscribe("/move_base_simple/goal", 10,clickCallback);

  ros::spin();
  ros::shutdown();
  return 0;
}

