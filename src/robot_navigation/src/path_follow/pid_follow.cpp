/*
 * @Author: your name
 * @Date: 2023-06-26 09:08:31
 * @LastEditTime: 2023-11-04 18:20:21
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/src/path_follow/pid_follow.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "path_follow/pid_follow.h"

// 速度PID参数设定
double velPID[7] = {16.0 , 0, 1, 2.0, -2.0, 3, 0.01};

double anglePID[7] = {50.0 , 0.01, 0, 0.5, -0.5, 3, 0.01};

pid_follow::pid_follow(double _kp, double _ki,double _kd, double _speedLimit, double _forwardDistance)
{
  this->speedLimit = _speedLimit;
  this->forwardDistance = _forwardDistance;
}

pid_follow::pid_follow()
{
  SetPIDParameter(&x_vel_pid,velPID);
  SetPIDParameter(&y_vel_pid,velPID);
  SetPIDParameter(&gyro_pid,anglePID);
  forwardDistance = 0.5;
}


pid_follow::~pid_follow()
{
}

// PID控制器初始化
void pid_follow::Init(void)
{
  SetPIDParameter(&x_vel_pid,pid_params);
  SetPIDParameter(&y_vel_pid,pid_params);
  SetPIDParameter(&gyro_pid,anglePID);
}

void pid_follow::SetPIDParameter(PIDFloatDefine *p_pid,double *A1)
{
  p_pid->NowError[0] = 0.0;

  p_pid->P = A1[0];
  p_pid->I = A1[1];
  p_pid->D = A1[2];

  p_pid->DiffError = 0.0;
  p_pid->IntError = 0.0;
  p_pid->feedPoint = 0;
  p_pid->OutPoint = 0.0;
  p_pid->SetPoint = 0.0;
  p_pid->OutMax = A1[3];
  p_pid->OutMin = A1[4];
  p_pid->LastfeedPoint = 0.0;
  p_pid->IntLimt = A1[5]; 
  p_pid->Intdt = A1[6]; 
}

// 位置PID控制
void pid_follow::PIDFloatPositionCal(PIDFloatDefine *p_pid)
{
  p_pid->NowError[0] = p_pid->SetPoint -  p_pid->feedPoint;//当前误差
  p_pid->DiffError = p_pid->feedPoint - p_pid->LastfeedPoint;//微分先行

  p_pid->IntError += ((double)p_pid->NowError[0] * p_pid->Intdt); //积分/p_pid->T_fru
  p_pid->IntError = LIMIT(p_pid->IntError, -p_pid->IntLimt, p_pid->IntLimt); //积分限幅
  p_pid->OutPoint =
      (p_pid->P * p_pid->NowError[0])//比例
      + (p_pid->I * p_pid->IntError)//积分项
      + (p_pid->D * p_pid->DiffError);//微分

  p_pid->OutPoint = LIMIT(p_pid->OutPoint, p_pid->OutMin, p_pid->OutMax); //输出限幅
  p_pid->LastfeedPoint = p_pid->feedPoint;
}

// 纯路径跟随
Eigen::Vector2d pid_follow::speedOutput(const Eigen::Vector2d _startTrace,
                                      const Eigen::Vector2d _endTrace,
                                      const vector<Eigen::Vector2d> _pathTrace)
{
  /*###参数初始化###*/
  Eigen::Vector2d errorDistance = Eigen::Vector2d(0.0f, 0.0f);
  Eigen::Vector2d speedOutput = Eigen::Vector2d(0.0f, 0.0f);

  if (_startTrace[0] == -1.0 || _startTrace[1] == -1.0 || _endTrace[0] == -1.0 || _endTrace[1] == -1.0)
  {
    return Eigen::Vector2d(0.0f, 0.0f);
  }
  /*###已导航状态###*/
  double currentDistance = sqrt((_startTrace[0] - _endTrace[0]) * (_startTrace[0] - _endTrace[0]) 
                        + (_startTrace[1] - _endTrace[1]) * (_startTrace[1] - _endTrace[1]));

  // 当车的位置与终点距离为0.05 代表车已经到达导航点
  if (currentDistance <= forwardDistance)
  {
    errorDistance[0] = 0.0f;
    errorDistance[1] = 0.0f;

    x_vel_pid.OutPoint = 0.0;
    y_vel_pid.OutPoint = 0.0;
    gyro_pid.OutPoint = 0.0;
    this->navStatusFlag = 1; //到达导航点
  }
  else
  {
    if((_pathTrace.size() > 0)){
      for (auto it : _pathTrace)
      {
        // 计算当前点到前一个点之间的距离
        double dis = sqrt(
          (it[0] - _pathTrace.front()[0]) * (it[0] - _pathTrace.front()[0]) + (it[1] - _pathTrace.front()[1]) * (it[1] - _pathTrace.front()[1]));
        if (dis >= forwardDistance)
        {
          //误差计算
          errorDistance[0] = it[0] - _pathTrace.front()[0];
          errorDistance[1] = it[1] - _pathTrace.front()[1];

          x_vel_pid.SetPoint = it[0];
          x_vel_pid.feedPoint = _pathTrace.front()[0];
          PIDFloatPositionCal(&x_vel_pid);
          
          y_vel_pid.SetPoint = it[1];
          y_vel_pid.feedPoint = _pathTrace.front()[1];
          PIDFloatPositionCal(&y_vel_pid);

          this->navStatusFlag = 0; //未能到达导航点
          break;
        }
      }
    }
    else
    {
      this->navStatusFlag = 0; //未能到达导航点
    }
  }

  /*###速度输出计算###*/
  speedOutput[0] = x_vel_pid.OutPoint;
  speedOutput[1] = y_vel_pid.OutPoint;
  gyro = gyro_pid.OutPoint;
  

  return speedOutput;  
}



