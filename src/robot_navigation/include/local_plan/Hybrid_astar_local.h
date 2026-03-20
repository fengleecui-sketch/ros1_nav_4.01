#ifndef __HYBRID_ASTAR_LOCAL_H
#define __HYBRID_ASTAR_LOCAL_H

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf2/convert.h"
#include <tf2/utils.h>

#include <Eigen/Dense>

#include "path_searcher/nav_app.h"
#include "path_searcher/Hybrid_astar.h"
#include "vel_transform/vel_transform.h"
#include "path_follow/pid_follow.h"
// 包含自定义消息包
#include "robot_communication/localizationInfoBroadcast.h"
#include "robot_communication/chassisControl.h"
#include "robot_communication/goal.h"

#include "iostream"
using namespace std;
#include "chrono"
using namespace chrono;


class Hybrid_astar_local
{
private:
  // 定义是否仿真标志位
  bool is_sim = true;
  // 系统控制频率
  int control_hz;
  // 定义规划路径半径，用于在先验路径中找点
  double TEMP_GOAL_RADIUS = 4.0;

  // 用于判断路径点数目
  int path_nodes_num;
  int last_path_nodes_num;

  bool local_goal_update;
  bool scan_updated;
  bool local_map_updated;
  bool odom_updated;
  bool target_velocity_update;    //目标速度是否更新
  bool update_path;
  bool updateEndFlag;           // 终点是否更新

  bool mapInitFlag = false;   //地图是否初始化成功标志位
  bool localMapFlag = false;    //是否获得到局部地图
  // 这个在有定位的情况下设置成false
  bool getStartFlag = false;    //是否获取到起点标志位
  bool getEndFlag = false;      //是否获取到终点标志位
  bool has_arrived_end = false; //是否到达终点

  // 定义一个代标志位的路径
  vector<Vector2d> trajpath;
  // 定义当前位置点
  Vector3d nowposition;
  // 定义局部终点
  Vector2d localgoalpoint;
  // 全局路径
  vector<Vector2d> globalpath;
  // 定义访问路径用于查看是否访问过
  vector<pair<int,Vector2d>> visitPath;

  mapDefine pathNav;          //导航路径


  Eigen::Vector2d startPoint;
private:
  // 如果没有私有节点，launch文件中的参数加载不进来，目前还不知道为什么，但是一定要像这样使用
  ros::NodeHandle private_node;  // ros中的私有句柄
  ros::NodeHandle local_plan;   // ros中的私有句柄

  /* data */
  Hybrid_astar::Ptr hybrid_astar_finder;

  void LocalPlanInit(void);
  /*** 
   * @description: 
   * @param {Publisher} pathPublish   要发布的话题
   * @param {vector<Vector2i>} path   要发布的路径
   * @return {*}
   */
  void PublishPath(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> path);
  void visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
  float a_set,float r_set,float g_set,float b_set,float length);

  robot_communication::localizationInfoBroadcast localData;   //定位信息
  robot_communication::chassisControl motionData;             //运动消息

  ros::Subscriber dynamicMap;   //订阅局部动态地图
  void dynamicMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map); //动态地图订阅回调函数
  ros::Subscriber pathSub;   //订阅局部动态地图
  void pathCallback(const nav_msgs::PathConstPtr &path);
  ros::Subscriber localizationSub;  //订阅定位信息
  void localizationCallback(const robot_communication::localizationInfoBroadcastConstPtr &msg); //定位信息回调函数

  ros::Publisher localgoalPub;      //局部终点查看
  ros::Publisher localPathPub;     //优化路径

  // 用来求解当前位置点以一定半径相交的路径点
  // path 输入路径
  // radius 搜索半径
  // updatepath 路径是否更新
  // return 局部终点
  Vector2d caLocalGoalPosition(vector<Vector2d> path,Vector2d nowpoint,double radius);

  // 路径规划
  void pathPlanning(Vector2d startMapPoint,Vector2d goalMapPoint);
  // 计算单位向量
  Vector2d calUnitvector(Vector2d unitv);
  // 计算向量之间的夹角
  double calVectorAngle(Vector2d vector1,Vector2d vector2);
  // 计算两个点之间的长度欧氏距离
  double calPointLength(Vector2d vector1,Vector2d vector2);

public:
  Hybrid_astar_local(/* args */);
  ~Hybrid_astar_local();
};




#endif
