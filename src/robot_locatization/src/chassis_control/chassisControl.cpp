/*
 * @Author: your name
 * @Date: 2023-05-06 21:01:43
 * @LastEditTime: 2024-04-18 10:42:38
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_locatization/src/chassis_control/chassisControl.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "chassis_control/chassisControl.h"

PIDFloatDefine chassAnglePID;
float chassAnglePIDParams[6] = {15.0,0,1,3,-3,3};

void SetPIDParameter(PIDFloatDefine *p_pid,float *A1)
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
}

// 位置PID控制
void PIDFloatPositionCal(PIDFloatDefine *p_pid)
{
  p_pid->NowError[0] = p_pid->SetPoint -  p_pid->feedPoint;//当前误差
  p_pid->DiffError = p_pid->feedPoint - p_pid->LastfeedPoint;//微分先行

  p_pid->IntError += ((double)p_pid->NowError[0]) * p_pid->I  ; //积分/p_pid->T_fru
  p_pid->IntError = LIMIT(p_pid->IntError, -p_pid->IntLimt, p_pid->IntLimt); //积分限幅
  p_pid->OutPoint =
      (p_pid->P * p_pid->NowError[0])//比例
      + (p_pid->IntError)//积分项
      + (p_pid->D * p_pid->DiffError);//微分

  p_pid->OutPoint = LIMIT(p_pid->OutPoint, p_pid->OutMin, p_pid->OutMax); //输出限幅
  p_pid->LastfeedPoint = p_pid->feedPoint;
}

// 过圈处理
void Deal_Super_Circle(float *setangle,float *feedangle)
{
  /* 
		因为当前的车的临界值是±PI，所以在达到这个角度的时候车只会按照
		正常的累积误差计算
	*/
  float delta = *feedangle - *setangle;
  if(delta >= -M_PI && delta <= M_PI)
  {
    *feedangle = 0.0f;
    *setangle = delta;
  }
  else if(delta > M_PI)
  {
    *feedangle = 0.0f;
    *setangle = -M_PI*2 + delta;
  }
  else if(delta < -M_PI)
  {
    *feedangle = 0.0f;
    *setangle = M_PI*2+delta;
  }

  *setangle = -*setangle;
  *feedangle = 0.0f; 
}

chassisControl::chassisControl(/* args */)
{
  // PID参数初始化
  SetPIDParameter(&chassAnglePID,chassAnglePIDParams);

  // 订阅全局消息
  globalVelSub = chassisCtl.subscribe("/acl_velocity",1,&chassisControl::GlobalVelocityCallback,this);
  chassCtlPub = chassisCtl.advertise<geometry_msgs::Twist>("chassis_control", 1, true);

  // gazebo发送的里程计真正数值消息订阅
  truth_odom_sub = chassisCtl.subscribe("/truth_pose_odom",1,&chassisControl::truthOdomCallback,this);
}

chassisControl::~chassisControl()
{
}

// 订阅真值里程计数值消息订阅
void chassisControl::truthOdomCallback(const robot_communication::localizationInfoBroadcastConstPtr &msg)
{
  truthOdom = *msg;
}

void chassisControl::GlobalVelocityCallback(const robot_communication::chassisControlConstPtr &msg)
{
  if(msg->globalOrlocal == true)
  {
    // 全向移动
    global.Vx = msg->xSpeed;
    global.Vy = msg->ySpeed;

    // 必须要有当前车底盘的姿态位置才能计算
    local.yaw = msg->chassisAngle;

    // 速度转换
    vel_transform::GlobalVelocityToLocal(&local,&global);
    
    // 全局 
    my_twist.linear.x = local.Vx;
    my_twist.linear.y = local.Vy;
    // 输出值
    // my_twist.angular.z = msg->chassisGyro;
  }
  else
  {
    // 局部 
    my_twist.linear.x = msg->xSpeed;
    my_twist.linear.y = msg->ySpeed;

    // 设定角度
    chassAnglePID.SetPoint = msg->chassisAngle;
    // 反馈角度
    chassAnglePID.feedPoint = nowYaw;
    Deal_Super_Circle(&chassAnglePID.SetPoint,&chassAnglePID.feedPoint);
    // 需要写一个简单的PID控制
    PIDFloatPositionCal(&chassAnglePID);
    // 输出值
    my_twist.angular.z = chassAnglePID.OutPoint;

    // my_twist.angular.z = msg->chassisGyro;    
  }
  // 局部或者说车体的速度发布
  chassCtlPub.publish(my_twist);
}
