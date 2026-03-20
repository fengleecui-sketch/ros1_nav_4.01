/*
 * @Author: your name
 * @Date: 2023-04-28 10:46:59
 * @LastEditTime: 2024-04-22 14:55:50
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/include/motionPlan/motionPlan.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __MOTION_PLAN_H
#define __MOTION_PLAN_H

#include "iostream"
using namespace std;
#include "chrono"
using namespace chrono;

#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "chrono"

// 包含全局规划包
#include "path_searcher/Astar_searcher.h"
#include "path_searcher/JPS_searcher.h"
#include "path_searcher/JPS_utils.h"
#include "path_searcher/node.h"
#include "path_searcher/two_way_rrt.h"
#include "path_searcher/nav_app.h"
#include "path_searcher/Hybrid_astar.h"

// 包含自定义速度转换包
#include "vel_transform/vel_transform.h"

// 包含路径优化包
#include "path_optimization/minimum_snap.h"
#include "path_optimization/bezier_curve.h"
#include "path_optimization/BSpline.h"

#include "path_optimization/non_uniform_bspline.h"
#include "path_optimization/bspline_optimizer.h"
#include "path_optimization/fast_security.h"

// 包含路径跟随包
#include "path_follow/pid_follow.h"

// 包含自定义消息包
#include "robot_communication/localizationInfoBroadcast.h"
#include "robot_communication/chassisControl.h"
#include "robot_communication/ESDFmap.h"
#include "robot_communication/goal.h"


using namespace dyn_planner;

class motionPlan
{
private:
  /* data */
  ros::NodeHandle motPlan;       //运动规划节点
  // 如果没有私有节点，launch文件中的参数加载不进来，目前还不知道为什么，但是一定要像这样使用
  ros::NodeHandle private_node;  // ros中的私有句柄

  // 话题发布
  ros::Publisher navmapPub;      //导航地图
  ros::Publisher oriPathPub;     //导航原始路径
  ros::Publisher optPathPub;     //优化路径
  ros::Publisher optPath2Pub;     //优化路径

  ros::Publisher esdfPathPub;   //发布astar算法在esdf地图情况下搜索到的路径

  ros::Publisher localPathPub;   // 局部地图发布
  ros::Publisher localGoalPub;      // 发布局部终点

  ros::Publisher optpathNodePub;    //优化路径节点消息发布

  ros::Publisher goalPointPub;      //目标点发布 
  ros::Publisher startPointPub;      //起点发布 

  // 第一个点发布，第二个点发布
  ros::Publisher firstPointPub;
  ros::Publisher secondPointPub;

  ros::Timer safety_timer_;       // 定义路径安全检测每s10次，频率10hz，如果有障碍物进行重新规划
  bool Replan_Path_Flag;          // 重规划路径标志位 

  // 安全检测回调函数
  void safetyCallback(const ros::TimerEvent& e);

  robot_communication::goal local_goal;    //发布局部终点
  geometry_msgs::PoseStamped global_goal;   //获取全局终点

  ros::Publisher chassConPub;    //底盘控制
  ros::Publisher StatusPub;      //状态发布

  ros::Publisher visitNodesPub;  //访问节点发布节点
  ros::Publisher globalEndPub;  //发布底盘控制消息
  ros::Publisher chassVelPub;  //发布底盘控制消息

  mapDefine pathNav;          //导航路径
  AstarPathFinder::Ptr astar_path_finder;
  // 用作对照实验
  AstarPathFinder::Ptr astar_esdf_path_finder;
  JPSPathFinder::Ptr jps_path_finder;
  Hybrid_astar::Ptr hybrid_astar_finder;
  RRTstarPlanner::Ptr rrt_path_finder;

  MinimumSnap MinimumSnapFlow;      //使用minimum_snap优化路径
  BEZIER Bezier_Flow;               //使用贝塞尔曲线优化路径

  //使用Fast_Secutity 优化路径
  Fast_Security::Ptr fast_security_opt;  

  Eigen::Vector2d startPoint;
  Eigen::Vector2d endPoint;

  robot_communication::localizationInfoBroadcast localData;   //定位信息
  robot_communication::chassisControl motionData;             //运动消息
  robot_communication::chassisControl motionLocal;            //局部运动消息

  //参数配置
  // paramSolution paramSolver;
  pid_follow pidFollow;

  // 话题接收
  ros::Subscriber staticMap;        //订阅静态代价地图
  void staticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map); //全局地图订阅回调函数
  // void staticMapCallback(const robot_communication::ESDFmap::ConstPtr &map); //全局地图订阅回调函数
  ros::Subscriber staticinflateMap;    //订阅静态膨胀地图
  void staticinflateMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map); //全局地图订阅回调函数
  ros::Subscriber clickSub;     //订阅鼠标点击信息(单位是m)
  void clickCallback(const geometry_msgs::PoseStampedConstPtr &msg); //鼠标点击回调函数
  ros::Subscriber localizationSub;  //订阅定位信息
  void localizationCallback(const robot_communication::localizationInfoBroadcastConstPtr &msg); //定位信息回调函数
  void localizationCallback_mid360(const nav_msgs::OdometryConstPtr& msg); //定位信息回调函数
  ros::Subscriber dynamicMap;   //订阅局部动态地图
  void dynamicMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map); //动态地图订阅回调函数
  // void dynamicMapCallback(const robot_communication::ESDFmap::ConstPtr &map); //动态地图订阅回调函数
  ros::Subscriber localPath;  //订阅局部规划出来的路径
  void localPathCallback(const nav_msgs::PathConstPtr &localpath);
  ros::Subscriber localVelocitySub;   //订阅局部速度消息
  void localVelocityCallback(const robot_communication::chassisControlConstPtr &velocity);
  ros::Subscriber initalposeSub;      //初始位置订阅
  void initalPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

  // 用来传递
  vector<Vector2d> local_path;
  // 车体局部速度和全局速度
  Odom_data_define localVel;
  Odom_data_define globalVel;
  // 定义临时路径
  vector<Vector2d> tempath;
  double length;
  // 用来传递局部终点
  Vector2d localGoal;

  bool mapInitFlag = false;   //地图是否初始化成功标志位
  bool localMapFlag = false;    //是否获得到局部地图
  // 这个在有定位的情况下设置成false
  bool getStartFlag = false;    //是否获取到起点标志位
  bool getEndFlag = false;      //是否获取到终点标志位
  bool has_arrived_end = false; //是否到达终点

  int control_method = 0;       //选用何种规划方式

  int cruise_num;
  // 定义5个巡航点
  double first_point_x;
  double first_point_y;
  double second_point_x;
  double second_point_y;
  double third_point_x;
  double third_point_y;
  double forth_point_x;
  double forth_point_y;
  double fifth_point_x;
  double fifth_point_y;
  
  vector<Vector2d> cruise_points;

  // 定义是否仿真标志位
  bool is_sim;
  // 定义是否巡航标志位
  bool is_cruise;
  // 定义停车范围
  double stop_margin;
  // frame_id名称
  string frame_id_name;       //

  // 运动规划功能函数
  // 系统初始化
  void motionInit(void);
  // 路径规划 A* JPS RRT
  void pathPlanning(Eigen::Vector2d startMapPoint,Eigen::Vector2d goalMapPoint);
  // 路径跟踪 PID MPC
  void pathFollow(std::vector<Eigen::Vector2d> path,Vector2d start,Vector2d goal);
  // 生成路径用于测试跟随算法效果
  void BuildPathTest_Follow(void);

  // 
  void visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
float a_set,float r_set,float g_set,float b_set,float length);
  // 发布可视化的点，起点终点这类的点
  void visualPoints(ros::Publisher pointPublish,Vector2d visitnodes,float a_set,float r_set,float g_set,float b_set,float length); 

  /*** 
   * @description: 
   * @param {Publisher} pathPublish   要发布的话题
   * @param {vector<Vector2i>} path   要发布的路径
   * @return {*}
   */
  void PublishPath(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> path);

  /*** 
   * @description: 
   * @param 1 读取出来的地图的参数  一维数组
   * @param 2 转换为一维算法能用的普通一维数组
   * @param 3 地图的x方向尺寸
   * @param 4 地图的y方向尺寸
   * @return {*}
   */
  void NavtfGrid(const nav_msgs::OccupancyGrid::ConstPtr &data,std::vector<int> &map,int x_size,int y_size);

  void mapInit(void); //地图参数初始化

  vector<Eigen::Vector2d> fixedpath;
  // 生成一个固定路线，用来测试车的跟随效果
  void BuildFixPath(vector<Eigen::Vector2d> &fixpath);
  
  // 系统控制频率
  int control_hz;

public:
  motionPlan(/* args */);
  ~motionPlan();
};


#endif  // __MOTION_PLAN_H
