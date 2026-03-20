/*
 * @Author: your name
 * @Date: 2023-04-28 10:50:02
 * @LastEditTime: 2024-04-19 14:09:42
 * @LastEditors: your name
 * @Description:
 * @FilePath: /MyFormProject/src/robot_navigation/src/nodes/Hybrid_local_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "local_plan/Hybrid_astar_local.h"

/// @brief
/// @param argc
/// @param argv
/// @return
int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "localPlan_node");

  Hybrid_astar_local localplan; // 规划测试

  return 0;
}

