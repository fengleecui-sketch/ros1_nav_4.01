#ifndef __PURE_PURSUIT_H
#define __PURE_PURSUIT_H


#include "ros/ros.h"
#include "iostream"
#include <ros/console.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include "tf/tf.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "Eigen/Eigen"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "pid_follow.h"

// 调用自己写的数学库，用来求解导数和多项式系数
#include "math_tools/curve_fitting.h"
#include "math_tools/filter.h"
#include "vel_transform/vel_transform.h"

// 包含自定义消息包
#include "robot_communication/localizationInfoBroadcast.h"
#include "robot_communication/chassisControl.h"

using namespace std;
using namespace Eigen;
using namespace math_tools;

#define PI 3.1415926

class pure_pursuit
{
private:
  /* data */
  ros::NodeHandle purePur;       //运动规划节点
  // 如果没有私有节点，launch文件中的参数加载不进来，目前还不知道为什么，但是一定要像这样使用
  ros::NodeHandle private_node;  // ros中的私有句柄

  ros::Publisher chassCtlPub;       //底盘控制
  geometry_msgs::Twist vel_control;    //速度控制节点
  ros::Publisher recePathPub;
  ros::Publisher globalVelPub;
  ros::Publisher directionPub;    //机器人行进方向发布 以odom形式进行发布

  ros::Publisher sampleNodesPub;    //采样点初始化
  // 定义里程计消息用来给ros包生成局部代价地图
  nav_msgs::Odometry directOdom;

  robot_communication::localizationInfoBroadcast localData;   //定位信息
  robot_communication::chassisControl motionData;             //运动消息

  // 用于对多项式求解求导
  curve_fitting curve;
  // 用于滤波
  filter  dealdata;   //处理数据
  //参数配置
  // paramSolution paramSolver;
  pid_follow pidFollow;

  double anglePID[7] = {15.0 , 0, 0, 3.0, -3.0, 3.0, 0.01};

  // yaw轴跟随路径
  vector<Vector2d> yawtrajPath;    //跟随路径
  // 速度跟随路径
  vector<Vector2d> veltrajPath;    //速度跟随路径

  vector<Vector2d> lastpath;    //上一次的路径
  // 全部的路径
  vector<Vector2d> globalPath;

  /* data */
  Vector2d nowPoint;      //当前点
  Vector2d lastPoint;     //下一个点
  Vector2d endPoint;      //

  // 系统控制频率
  int control_hz;

  double alpha;   //车的转角
  double car_length;    //车的长度
  double pre_distance;  //预瞄距离 取下一个点-当前点距离的长度
  double fro_distance;  //前馈距离
  double fix_distance;  //固定距离

  double velocity_x;    //当前车的速度
  double velocity_y;    
  double setVeloc_x;    //设定车速度
  double setVeloc_y;

  double max_velocity;  //最大速度
  double max_acce;      //最大加速度

  double nowangle;      //车的角度
  double setangle;      //设定角度   
  double lastangle;     //上一个角度

  double time;          //求导时间

  int order;            //多项式子阶数

  // 用来在路径点上截取相应的位置坐标
  // 记录当前的点
  int first_yaw_num = 0;
  int first_vel_num = 0;
  int last_yaw_num = 0;    //
  int last_vel_num = 0;

  // 当前路径点数目
  int now_path_nodes_num = 0;
  // 上一个路径点数目
  int last_path_nodes_num = 0;

  bool has_receive_path = false;
  bool first_path_flag = false;
  bool is_use_sim = false;    //是否仿真
  bool has_arrived_end;     //是否到达终点
  bool update_path;         //更新路径标志位

  bool get_end_flag;        //获得终点标志位

  double pathlength;        // 剩余的路径
  double alllength;         // 总路径
  double complelength;      // 已经完成的路径

  double run_time;          // 运行时间
  double set_acce;          // 设定加速度
  double integral_t;        // 积分时间

  double map_resolution;    // 地图分辨率
  double set_vel_k;         // 按照地图分辨率划分路径点的系数用来*map_resolution,用来规划速度
  double set_yaw_k;         // 设定yaw跟随划分路径点的系数用来*map_resolution
  double set_vel_sum;       // 设定车的速度，合速度标量没有方向
  double feed_vel_sum;      // 反馈的合成速度，同样是矢量

  double length_x_path;     //对路径点上每个点求解x差值累加
  double length_y_path;     //对路径点上每个点求解y插值累加

  // 速度合成，局部转全局
  // 车体局部速度和全局速度
  Odom_data_define carVel;
  Odom_data_define worldVel;

  void visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
float a_set,float r_set,float g_set,float b_set,float length);

  // 计算两个点之间的长度欧氏距离
  double calPointLength(Vector2d vector1,Vector2d vector2);

public:

  void PublishPath(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> path)
  {
    nav_msgs::Path pathTopic; // Astar路径的话题名
    pathTopic.header.frame_id = "map";
    for (unsigned int i = 0; i < path.size(); i++)
    {
      geometry_msgs::PoseStamped pathPose;
      pathPose.pose.position.x = path[i][0];
      pathPose.pose.position.y = path[i][1];
      pathPose.pose.position.z = 0;

      pathTopic.header.stamp = ros::Time::now();
      pathTopic.poses.push_back(pathPose);
    }
    pathPublish.publish(pathTopic);
  }

  // 传递参数
  void Init_Params(ros::NodeHandle &nh);

  // 获取车的速度
  void Get_CarParams(double vx,double vy,double angle);

  // 过圈处理
  void Deal_Super_Circle(double *setangle,double *feedangle);

  // nowpos 当前位置点
  // path 路径
  // 初步拟定使用5个点，即4次多项式进行轨迹跟随，输出的是速度
  void control(Vector2d nowpos,vector<Vector2d> path);

  // 五次多项式路径跟随
  // nowpos  当前位置点
  // path  路径
  void Pure_Pursuit_Control(void);

  // 规划yaw轴
  // 当前点 nowpos
  // 路径   path
  // 输出角速度 yaw_speed
  // 输出角度 yaw
  void planYaw(Vector2d nowpos,vector<Vector2d> path,float &yaw_speed,float &set_yaw);

  // 规划速度
  // 当前点 nowpos
  // 路径 path
  // 输出x轴方向速度 x_velocity
  // 输出y轴方向速度 y_velocity
  void planVelocity(Vector2d nowpos,vector<Vector2d> path,float &x_velocity,float &y_velocity);

  // 用来求解当前位置点以一定半径相交的路径点
  // path 输入路径
  // radius 搜索半径
  // return 局部终点
  Vector2d caLocalGoalPosition(vector<Vector2d> path,Vector2d nowpoint,double radius,int &point_num);

  // 用来对速度进行限制,因为机器人是全向移动的,如果直接通过数值对x y速度限制会产生畸变
  // 所以需要先合成,再进行限制
  // vel_x 输入的x方向上的速度
  // vel_y 输入的y方向上的速度
  // limit_velocity 限制的速度
  void LIMIT_VECTOR_Velocity(double &vel_x,double &vel_y,double limit_velocity);

  // 动态五次多项式规划速度
  // 当前点 nowpos
  // 终点 endpos
  // 当前速度 nowvel
  // 终点速度 endvel
  // 当前点加速度 nowacc
  // 终点加速度   endacc
  // 路径 nowpath
  void Dynamic_Quintic_Polynomial(Vector2d nowpos,Vector2d endpos,
  Vector2d nowvel,Vector2d endvel,
  Vector2d nowacc,Vector2d endacc,vector<Vector2d> nowpath);

  ros::Subscriber pathSub;
  void pathCallback(const nav_msgs::PathConstPtr &points);    // 路径回调函数
  ros::Subscriber localizationSub;  //订阅定位信息
  void localizationCallback(const robot_communication::localizationInfoBroadcastConstPtr &msg); //定位信息回调函数
  ros::Subscriber clickSub;
  void clickCallback(const geometry_msgs::PoseStampedConstPtr &msg); //鼠标点击回调函数

  pure_pursuit(/* args */);
  ~pure_pursuit();
};


#endif  //

