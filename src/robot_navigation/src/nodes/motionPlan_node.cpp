/*
 * @Author: your name
 * @Date: 2023-04-28 10:50:02
 * @LastEditTime: 2023-07-22 11:45:18
 * @LastEditors: your name
 * @Description:
 * @FilePath: /MyFormProject/src/robot_navigation/src/nodes/motionPlan_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "motionPlan/motionPlan.h"

/// @brief
/// @param argc
/// @param argv
/// @return
int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "motionPlan_node");

  motionPlan motionTest; // 规划测试

  return 0;
}
