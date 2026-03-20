/*
 * @Author: your name
 * @Date: 2023-10-20 04:53:55
 * @LastEditTime: 2024-01-26 16:36:29
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/src/nodes/omnidirectional_DWA_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "ros/ros.h"
#include "local_plan/omnidirectional_dwa.h"


int main(int argc, char** argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "dwa_planner");
  ros::NodeHandle nh("~");

  Omnidirectional_DWAPlanner dwa_local_planner;  
  dwa_local_planner.process();

  return 0;
}

