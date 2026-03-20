/*
 * @Author: your name
 * @Date: 2023-05-06 12:08:37
 * @LastEditTime: 2024-04-21 12:36:21
 * @LastEditors: your name
 * @Description:
 * @FilePath: /MyFormProject/src/robot_locatization/include/vel_transform/vel_transform.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __VEL_TRANSFORM_H
#define __VEL_TRANSFORM_H

#include "iostream"
using namespace std;

#include <cmath>
#include <vector>
#include <chrono>


typedef struct Odom_data
{
  double yaw;   // 偏航角
  double pitch; // 俯仰角
  double roll;  // 横滚角度

  double gyroYaw; // 角速度

  double Vx; // x轴方向上的速度
  double Vy; // y轴方向上的速度

  double Accx; // x轴方向上的加速度
  double Accy; // y轴方向上的加速度

  double theta; // arctan(Vy/Vx) 速度矢量合成角度
} Odom_data_define;

namespace vel_transform
{
  // 换成麦轮模型之后的飘逸问题是因为获得的数据是车的局部速度数据，现在将数据转换为全局速度数据
  void LocalVelocityToGlobal(Odom_data_define *localVelocity, Odom_data_define *globalVelocity);
  // 换成麦轮模型之后的飘逸问题是因为获得的数据是车的局部速度数据，现在将数据转换为全局速度数据
  void LocalAcceleraToGlobal(Odom_data_define *localVelocity, Odom_data_define *globalVelocity);
  // 全局速度转局部
  void GlobalVelocityToLocal(Odom_data_define *localVelocity, Odom_data_define *globalVelocity);
  // 全局速度转局部
  void GlobalVelocityToLocalVelocity(Odom_data_define *localVelocity, Odom_data_define *globalVelocity);
  // 采用向量方法将全局速度转成局部速度
  void GlobalVelocityToLocalVector(Odom_data_define *localVelocity, Odom_data_define *globalVelocity);
} // namespace name

#endif //__VEL_TRANSFORM_H
