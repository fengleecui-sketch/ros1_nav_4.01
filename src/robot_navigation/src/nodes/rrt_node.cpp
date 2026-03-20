#include "path_searcher/two_way_rrt.h"

using namespace Eigen;
using namespace std;

RRTstarPlanner *rrtPlanner = new RRTstarPlanner();

// simulation param from launch file
double _resolution, _inv_resolution, _cloud_margin;
double _x_size, _y_size, _z_size;

Vector2d _map_lower, _map_upper;
int _max_x_id, _max_y_id, _max_z_id;

void PublishPath(void);
// 查看访问树
void visTree(std::vector<geometry_msgs::Point> rrtree);

void initializeMarkers(visualization_msgs::Marker &sourcePoint,
                       visualization_msgs::Marker &goalPoint)
{
  // init headers
  sourcePoint.header.frame_id = goalPoint.header.frame_id = "world";
  sourcePoint.header.stamp = goalPoint.header.stamp = ros::Time::now();
  sourcePoint.ns = goalPoint.ns = "world";
  sourcePoint.action = goalPoint.action = visualization_msgs::Marker::ADD;
  sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = 1.0;

  // setting id for each marker
  sourcePoint.id = 0;
  goalPoint.id = 1;

  // defining types
  sourcePoint.type = goalPoint.type = visualization_msgs::Marker::SPHERE;

  // setting scale
  sourcePoint.scale.x = goalPoint.scale.x = 0.3;
  sourcePoint.scale.y = goalPoint.scale.y = 0.3;
  sourcePoint.scale.z = goalPoint.scale.z = 0.3;

  // colors
  sourcePoint.color.r = 1.0f;
  goalPoint.color.g = 1.0f;

  sourcePoint.color.a = goalPoint.color.a = 1.0f;
}

std::vector<geometry_msgs::Point> rrtTree; // 定义搜索树木
std::vector<Eigen::Vector2d> finalpath;    // 定义最终的搜索到终点的路径

ros::Publisher rrtPathPub; // 发布rrt路径
void PublishPath(std::vector<Eigen::Vector2d> path)
{
  nav_msgs::Path rrtPathTopic; // Astar路径的话题名
  rrtPathTopic.poses.clear();  // 上一次搜索路径清空
  for (unsigned int i = 0; i < path.size(); i++)
  {
    geometry_msgs::PoseStamped pathPose;

    pathPose.pose.position.x = path[i][0];
    pathPose.pose.position.y = path[i][1];
    pathPose.pose.position.z = 0;

    pathPose.pose.orientation.x = 0.0;
    pathPose.pose.orientation.y = 0.0;
    pathPose.pose.orientation.z = 0.0;
    pathPose.pose.orientation.w = 1.0;
    rrtPathTopic.header.stamp = ros::Time::now();
    rrtPathTopic.header.frame_id = "odom"; // 这部分可能报错，注意看基坐标系名称
    rrtPathTopic.poses.push_back(pathPose);
  }
  rrtPathPub.publish(rrtPathTopic);
}

ros::Publisher rrt_Tree_pub; // rrt树发布
// 查看访问树
void visTree(std::vector<geometry_msgs::Point> rrtree)
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

  node_vis.color.a = 0.25;
  node_vis.color.r = 0.0;
  node_vis.color.g = 0.0;
  node_vis.color.b = 1.0;
  node_vis.scale.x = _resolution * 2;
  node_vis.scale.y = _resolution * 2;
  node_vis.scale.z = _resolution * 2;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(rrtree.size()); i++)
  {
    pt.x = rrtree[i].x;
    pt.y = rrtree[i].y;
    node_vis.points.push_back(pt);
  }
  rrt_Tree_pub.publish(node_vis);
}

double origin_x = 0.0;   // 栅格坐标系与世界坐标系的原点X的差值，单位为m
double origin_y = 0.0;   // 栅格坐标系与世界坐标系的原点Y的差值，单位为m
double width = 0.0;      // 栅格地图的宽
double height = 0.0;     // 栅格地图的高
double resolution = 0.0; // 栅格地图的分辨率
std::vector<int> mapData;

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
  rrtPlanner = new RRTstarPlanner();
  rrtPlanner->initMap(resolution, origin_x, origin_y, width, height, mapData);
}

ros::Publisher rrtStartPoint; // 发布起点位置
ros::Subscriber initPoseSub;  // 订阅起点
Vector2d _start_pt;           // 定义起点

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
  node_vis.points.push_back(pt);
  rrtStartPoint.publish(node_vis);

  _start_pt[0] = pt.x;
  _start_pt[1] = pt.y;
}

ros::Publisher rrtGoalPoint; // 发布终点位置
// 终点位置的回调函数
void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
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
  node_vis.points.push_back(pt);
  rrtGoalPoint.publish(node_vis);

  Eigen::Vector2d targetpoint;
  targetpoint[0] = pt.x;
  targetpoint[1] = pt.y;

  ROS_INFO("Goal x:%f y:%f", _start_pt[0], _start_pt[1]);
  ROS_INFO("Goal x:%f y:%f", pt.x, pt.y);

  rrtPlanner->FindPath(_start_pt, targetpoint);
  PublishPath(rrtPlanner->path);
}

ros::Subscriber mapSub;      // 订阅栅格地图
ros::Subscriber goalPosePub; // 订阅终点
int main(int argc, char **argv)
{
  // initializing ROS
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "rrt_node");
  ros::NodeHandle n;

  // defining Publisher
  rrtPathPub = n.advertise<nav_msgs::Path>("/rrt_planner_path", 15);
  mapSub = n.subscribe("/map", 1, MapCallback);                               // 订阅栅格地图
  initPoseSub = n.subscribe("/initialpose", 1, InitPoseCallback);             // 订阅起点
  goalPosePub = n.subscribe("/move_base_simple/goal", 1, targetPoseCallback); // 订阅终点
  rrtStartPoint = n.advertise<visualization_msgs::Marker>("/start_point", 1); // 可视化起点位置
  rrtGoalPoint = n.advertise<visualization_msgs::Marker>("/goal_point", 1);   // 可视化终点位置

  rrt_Tree_pub = n.advertise<visualization_msgs::Marker>("visited_nodes_vis", 1);

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  delete rrtPlanner; // 养成习惯有几个new 删几个new 只要写了就在这删
  return 0;
}
