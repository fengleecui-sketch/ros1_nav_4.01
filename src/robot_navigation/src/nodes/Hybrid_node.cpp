#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "Eigen/Eigen"
#include "path_searcher/Hybrid_astar.h"

using namespace std;
using namespace Eigen;

double origin_x = 0.0;   // 栅格坐标系与世界坐标系的原点X的差值，单位为m
double origin_y = 0.0;   // 栅格坐标系与世界坐标系的原点Y的差值，单位为m
double width = 0.0;      // 栅格地图的宽
double height = 0.0;     // 栅格地图的高
double resolution = 0.0; // 栅格地图的分辨率

std::vector<int> mapData;
vector<Vector2d> astarPath; // 最终路径表   //Astar算法的路径点
vector<Vector2i> visitNode; // 访问节点数目

ros::Publisher astarPathPub;    // 发布Astar路径
ros::Publisher astarStartPoint; // 发布起点位置
ros::Publisher astarGoalPoint;  // 发布终点位置
ros::Subscriber mapSub;         // 订阅栅格地图
ros::Subscriber initPoseSub;    // 订阅起点
ros::Subscriber goalPosePub;    // 订阅终点

Eigen::Vector2d startMapPoint; // 算法的起点
Eigen::Vector2d goalMapPoint;  // 算法的目标点
std::string distance;          // Astar的距离函数

void PublishPath(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> path);

Hybrid_astar *_astar_path_finder = new Hybrid_astar();

void StartFindPath(Eigen::Vector2d startMapPoint, Eigen::Vector2d goalMapPoint)
{
  // startMapPoint[0] = 0.5;
  // startMapPoint[1] = 0.5;

  ROS_INFO("Start find path with Astar");

  _astar_path_finder->reset();
  // 记录路径搜索需要的时间
  ros::Time time_1 = ros::Time::now();
  int status = _astar_path_finder->search(startMapPoint, Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0),
                                          goalMapPoint, Eigen::Vector2d(0, 0), true);
  ros::Time time_2 = ros::Time::now();

  // 如果没有找到
  if (status == Hybrid_astar::NO_PATH)
  {
    cout << "[kino replan]: kinodynamic search fail!" << endl;

    // 再次复位
    _astar_path_finder->reset();
    // 重新搜索
    status = _astar_path_finder->search(startMapPoint, Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0),
                                        goalMapPoint, Eigen::Vector2d(0, 0), false);

    // 两次搜索还是没有找到 寄了
    if (status == Hybrid_astar::NO_PATH)
    {
      cout << "[kino replan]: Can't find path." << endl;
    }
    else
    {
      cout << "[kino replan]: retry search success." << endl;
    }
  }

  astarPath = _astar_path_finder->getKinoTraj(0.01); // 获取路径
  ROS_INFO("time is %fms,path size is %ld", (time_2 - time_1).toSec() * 1000, astarPath.size());

  PublishPath(astarPathPub, astarPath);
}

// 栅格地图的回调函数
void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  origin_x = msg->info.origin.position.x; // 获得栅格地图的原点x值(相对世界坐标系),单位为m
  origin_y = msg->info.origin.position.y; // 获得栅格地图的原点y值(相对世界坐标系),单位为m
  resolution = msg->info.resolution;      // 获得栅格地图的分辨率
  width = msg->info.width;                // 获得栅格地图的宽
  height = msg->info.height;              // 获得栅格地图的高

  std::cout << "***********map message**********" << std::endl;
  std::cout << "origin_x:" << origin_x << std::endl;
  std::cout << "origin_y:" << origin_y << std::endl;
  std::cout << "resolution:" << resolution << std::endl;
  std::cout << "width:" << width << std::endl;
  std::cout << "height:" << height << std::endl;
  std::cout << "*********************************" << std::endl;
  mapData.resize(width * height);
  // 获得地图数据 0自由通行区域，100障碍物
  for (int i = 0; i < width; i++)
  {
    for (int j = 0; j < height; j++)
    {
      mapData[i * height + j] = int(msg->data[j * width + i]);
    }
  }
  _astar_path_finder->InitMap(resolution, origin_x, origin_y, width, height, mapData);
}

// 初始位置的回调函数
void InitPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();
  node_vis.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis.action = visualization_msgs::Marker::ADD;
  node_vis.id = 0;

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.color.a = 1.0;
  node_vis.color.r = 1.0;
  node_vis.color.g = 0.0;
  node_vis.color.b = 0.0;

  node_vis.scale.x = resolution * 2;
  node_vis.scale.y = resolution * 2;
  node_vis.scale.z = resolution * 2;

  geometry_msgs::Point pt;
  pt.x = msg->pose.pose.position.x;
  pt.y = msg->pose.pose.position.y;
  pt.z = 0.0;

  ROS_INFO("Start x:%f y:%f", pt.x, pt.y);
  node_vis.points.push_back(pt);
  astarStartPoint.publish(node_vis);

  startMapPoint[0] = msg->pose.pose.position.x;
  startMapPoint[1] = msg->pose.pose.position.y;
}

// 终点位置的回调函数
void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();
  node_vis.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis.action = visualization_msgs::Marker::ADD;
  node_vis.id = 0;

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.color.a = 1.0;
  node_vis.color.r = 0.0;
  node_vis.color.g = 0.0;
  node_vis.color.b = 1.0;
  node_vis.scale.x = resolution * 2;
  node_vis.scale.y = resolution * 2;
  node_vis.scale.z = resolution * 2;

  geometry_msgs::Point pt;
  pt.x = msg->pose.position.x;
  pt.y = msg->pose.position.y;
  pt.z = 0.0;
  ROS_INFO("Goal x:%f y:%f", pt.x, pt.y);
  node_vis.points.push_back(pt);
  astarGoalPoint.publish(node_vis);

  goalMapPoint[0] = msg->pose.position.x;
  goalMapPoint[1] = msg->pose.position.y;

  StartFindPath(startMapPoint, goalMapPoint);
}

// 发布Astar路径轨迹
void PublishPath(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> path)
{
  nav_msgs::Path astarPathTopic; // Astar路径的话题名
  astarPathTopic.poses.clear();  // 上一次搜索路径清空
  for (unsigned int i = 0; i < path.size(); i++)
  {
    geometry_msgs::PoseStamped pathPose;
    pathPose.pose.position.x = path[i][0];
    pathPose.pose.position.y = path[i][1];

    // cout<<path[i][0]<<"  "<<path[i][1]<<endl;

    pathPose.pose.position.z = 0;

    pathPose.pose.orientation.x = 0.0;
    pathPose.pose.orientation.y = 0.0;
    pathPose.pose.orientation.z = 0.0;
    pathPose.pose.orientation.w = 1.0;
    astarPathTopic.header.stamp = ros::Time::now();
    astarPathTopic.header.frame_id = "odom";
    astarPathTopic.poses.push_back(pathPose);
  }
  pathPublish.publish(astarPathTopic);
}

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "hybrid_astar_node");
  ros::NodeHandle nh("~");

  astarPathPub = nh.advertise<nav_msgs::Path>("/hybrid_Astar", 15);              // 发布Astar路径
  mapSub = nh.subscribe("/map", 1, MapCallback);                                 // 订阅栅格地图
  initPoseSub = nh.subscribe("/initialpose", 1, InitPoseCallback);               // 订阅起点
  goalPosePub = nh.subscribe("/move_base_simple/goal", 1, GoalPoseCallback);     // 订阅终点
  astarStartPoint = nh.advertise<visualization_msgs::Marker>("/start_point", 1); // 可视化起点位置
  astarGoalPoint = nh.advertise<visualization_msgs::Marker>("/goal_point", 1);   // 可视化终点位置

  _astar_path_finder->InitParams(nh);

  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete _astar_path_finder;

  return 0;
}
