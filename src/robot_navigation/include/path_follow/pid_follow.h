/*
 * @Author: your name
 * @Date: 2023-05-06 13:27:29
 * @LastEditTime: 2023-09-22 09:54:35
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/include/path_follow/pid_follow.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __PID_FOLLOW_H
#define __PID_FOLLOW_H

#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
#include "utility/math_function.h"

using namespace std;

#define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

typedef struct PIDParameterStruct
{
  double P;   //比例常数
  double I;   //积分常数
  double D;   //微分常数

  double SetPoint;  //设定数值
  double feedPoint; //反馈数值
  double OutPoint;  //输出结果

  double OutMax;    //输出限幅
  double OutMin;    //输出限幅

  double DiffError; //微分
  double IntError;  //积分
  double IntLimt;   //积分限幅
  double Intdt;     // 积分时间
  double NowError[3]; //误差值
  double LastfeedPoint; //最终输出数值
}PIDFloatDefine;


// 创建全向移动底盘跟踪控制器
class pid_follow
{
private:
  /* data */
  double speedLimit;

  PIDFloatDefine x_vel_pid;   // x 方向上速度的pid计算
  PIDFloatDefine y_vel_pid;   // y 方向上速度的pid计算

public:
  PIDFloatDefine gyro_pid;    //角速度PID

  pid_follow(double _kp, double _ki, double _kd,double _speedLimit, double _forwardDistance);
  pid_follow();
  ~pid_follow();

  double forwardDistance; //10cm
  double pid_params[7];   //pid参数数组

  double gyro;

  // PID控制器初始化
  void Init();

  void SetPIDParameter(PIDFloatDefine *p_pid,double *A1);
  void PIDFloatPositionCal(PIDFloatDefine *p_pid);
  
  Eigen::Vector2d speedOutput(const Eigen::Vector2d _startTrace,
                                        const Eigen::Vector2d _endTrace,
                                        const vector<Eigen::Vector2d> _pathTrace);

  int navStatusFlag = 1; //默认到达 
};

#endif  // __PID_FOLLOW_H

