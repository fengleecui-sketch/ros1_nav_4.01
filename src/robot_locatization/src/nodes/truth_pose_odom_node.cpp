/*
 * @Author: your name
 * @Date: 2023-04-27 19:57:46
 * @LastEditTime: 2023-05-06 12:52:46
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /Robot_Formation/src/robot_locatization/src/nodes/truth_pose_odom_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "truth_pose/truth_pose_odom.h"

int main(int argc,char** argv)
{
  // 设置编码方式为UTF-8
  setlocale(LC_ALL, "");

  // 初始化ROS
  ros::init(argc,argv,"truth_pose_odom");

  // 定义节点
  truth_pose_odom truth_pose_inform;

  return 0;
}
