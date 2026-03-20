/***
 * @                       _oo0oo_
 * @                      o8888888o
 * @                      88" . "88
 * @                      (| -_- |)
 * @                      0\  =  /0
 * @                    ___/`---'\___
 * @                  .' \\|     |// '.
 * @                 / \\|||  :  |||// \
 * @                / _||||| -:- |||||- \
 * @               |   | \\\  - /// |   |
 * @               | \_|  ''\---/''  |_/ |
 * @               \  .-\__  '-'  ___/-. /
 * @             ___'. .'  /--.--\  `. .'___
 * @          ."" '<  `.___\_<|>_/___.' >' "".
 * @         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 * @         \  \ `_.   \_ __\ /__ _/   .-` /  /
 * @     =====`-.____`.___ \_____/___.-`___.-'=====
 * @                       `=---='
 * @
 * @
 * @     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * @
 * @           佛祖保佑     永不宕机     永无BUG
 * @
 * @Author: your name
 * @Date: 2022-12-06 15:06:40
 * @LastEditTime: 2023-02-24 15:51:32
 * @LastEditors: your name
 * @Description:
 * @FilePath: /tianbot_mini/src/astar_super/src/path_searcher/nav_app.cpp
 * @可以输入预定的版权声明、个性签名、空行等
 */

#include "path_searcher/nav_app.h"

navSolution::navSolution(int flag)
{
  this->choose = flag;
  this->All();

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  delete jps_path_finder;
  delete astar_path_finder;
  delete rrt_path_finder;
  delete hybrid_astar_finder;
}

// A*搜索全局路径
// 优化并发布
void navSolution::AstarFindPath(Eigen::Vector2d startMapPoint, Eigen::Vector2d goalMapPoint)
{
  ROS_INFO("\033[1;32m A* find path! \033[0m");
  is_use_jps = false;
  ros::Time time_3 = ros::Time::now();
  astar_path_finder->AstarWorldSearch(startMapPoint, goalMapPoint);
  ros::Time time_4 = ros::Time::now();
  pathNav.worldpath = astar_path_finder->getWorldPath();               // 获取路径
  pathNav.visitWorldNodes = astar_path_finder->getVisitedWorldNodes(); // 获取访问节点
  astar_path_finder->resetUsedGrids();                                 // 复位已经访问的地图
  visual_VisitedNode(visited_nodes_pub, pathNav.visitWorldNodes);      // 可视化访问节点
  PublishPath(PathPub_Astar, pathNav.worldpath);                       // 发布路径
  ROS_INFO("\033[1;32m time is %f ms,path size is %ld \033[0m",
           (time_4 - time_3).toSec() * 1000, pathNav.worldpath.size());

  ros::Time start_time = ros::Time::now();
  // pathNav.optpath = MinimumSnapFlow.Minimum_Snap(pathNav.worldpath);
  // pathNav.optpath = MinimumSnapFlow.SubsectionPath_Minimum_Snap(pathNav.worldpath);
  pathNav.optpath = Bezier_Flow.SubsectionPath_Bezier(pathNav.worldpath);
  ros::Time end_time = ros::Time::now();
  ROS_INFO("\033[1;32m --> Time in Minimum Snap is %f ms  \033[0m",
           (end_time - start_time).toSec() * 1000.0);
  PublishPath(trajectory_pub_, pathNav.optpath);
}

// rrt搜索全局路径
// 优化并发布
void navSolution::rrtFindPath(Eigen::Vector2d startMapPoint, Eigen::Vector2d goalMapPoint)
{
  ROS_INFO("\033[1;32m RRT* find path! \033[0m");
  ros::Time time_1 = ros::Time::now();
  rrt_path_finder->FindPath(startMapPoint, goalMapPoint);
  ros::Time time_2 = ros::Time::now();
  pathNav.worldpath = rrt_path_finder->path; // 获取路径

  ROS_INFO("\033[1;32m time is %f ms,path size is %ld \033[0m",
           (time_2 - time_1).toSec() * 1000, pathNav.worldpath.size());

  PublishPath(PathPub_RRT, pathNav.worldpath); // 发布路径

#if 0
  ros::Time start_time = ros::Time::now();
  pathNav.optpath = MinimumSnapFlow.Minimum_Snap(pathNav.worldpath);
  ros::Time end_time = ros::Time::now();
  ROS_INFO("\033[1;32m --> Time in Minimum Snap is %f ms  \033[0m",
           (end_time - start_time).toSec() * 1000.0);
  PublishPath(trajectory_pub_, pathNav.optpath);
#endif
}

// JPS搜索全局路径
// 优化并发布
void navSolution::JPSFindPath(Eigen::Vector2d startMapPoint, Eigen::Vector2d goalMapPoint)
{
  ROS_INFO("\033[1;32m JPS finding path! \033[0m");
  is_use_jps = true;
  ros::Time time_1 = ros::Time::now();
  jps_path_finder->JPSWorldSearch(startMapPoint, goalMapPoint);
  ros::Time time_2 = ros::Time::now();
  pathNav.worldpath = jps_path_finder->getJPSWorldPath();            // 获取路径
  pathNav.visitWorldNodes = jps_path_finder->getVisitedWorldNodes(); // 获取访问节点
  jps_path_finder->resetUsedGrids();                                 // 复位已经访问的地图

  ROS_INFO("\033[1;32m time is %f ms,path size is %ld \033[0m",
           (time_2 - time_1).toSec() * 1000, pathNav.worldpath.size());

  visual_VisitedNode(visited_nodes_pub, pathNav.visitWorldNodes); // 可视化访问节点
  PublishPath(PathPub_JPS, pathNav.worldpath);                    // 发布路径

  ros::Time start_time = ros::Time::now();
  pathNav.optpath = MinimumSnapFlow.Minimum_Snap(pathNav.worldpath);
  ros::Time end_time = ros::Time::now();
  ROS_INFO("\033[1;32m --> Time in Minimum Snap is %f ms  \033[0m",
           (end_time - start_time).toSec() * 1000.0);
  PublishPath(trajectory_pub_, pathNav.optpath);
}

void navSolution::HybridAstarFindPath(Eigen::Vector2d startMapPoint, Eigen::Vector2d goalMapPoint)
{
  ROS_INFO("Start find path with HybridAstar");

  hybrid_astar_finder->reset();
  // 记录路径搜索需要的时间
  ros::Time time_1 = ros::Time::now();
  int status = hybrid_astar_finder->search(startMapPoint,Eigen::Vector2d(0.0,0.0),Eigen::Vector2d(1.0,1.0),
                          goalMapPoint,Eigen::Vector2d(0,0),true);
  ros::Time time_2 = ros::Time::now();

// 如果没有找到
if (status == Hybrid_astar::NO_PATH) {
  cout << "[kino replan]: kinodynamic search fail!" << endl;

  // 再次复位
  hybrid_astar_finder->reset();
  // 重新搜索
  status = hybrid_astar_finder->search(startMapPoint,Eigen::Vector2d(0.0,0.0),Eigen::Vector2d(1.0,1.0),
                          goalMapPoint,Eigen::Vector2d(0,0),false);

  // 两次搜索还是没有找到 寄了
  if (status == Hybrid_astar::NO_PATH) {
    cout << "[kino replan]: Can't find path." << endl;
  } else {
    cout << "[kino replan]: retry search success." << endl;
  }
  }

  pathNav.worldpath = hybrid_astar_finder->getKinoTraj(0.01); //获取路径
  ROS_INFO("time is %fms,path size is %ld",(time_2-time_1).toSec()*1000,pathNav.worldpath.size());

  PublishPath(PathPub_Hybrid,pathNav.worldpath);  
}

/***
 * @description:
 * @return {*}
 */
void navSolution::StartFindPath(Eigen::Vector2d startMapPoint, Eigen::Vector2d goalMapPoint)
{
  ROS_INFO("\033[1;32m start: %f %f \033[0m",
           startMapPoint[0], startMapPoint[1]);
  ROS_INFO("\033[1;32m goal: %f %f \033[0m",
           goalMapPoint[0], goalMapPoint[1]);
  // 记录路径搜索需要的时间
  if (this->choose == 1)
  {
    JPSFindPath(startMapPoint, goalMapPoint);
  }
  else if (this->choose == 2)
  {
    AstarFindPath(startMapPoint, goalMapPoint);
  }
  else if (this->choose == 3)
  {
    rrtFindPath(startMapPoint, goalMapPoint);
  }
  else if(this->choose == 4)
  {
    HybridAstarFindPath(startMapPoint, goalMapPoint);
  }
  else if (this->choose == 5)
  {
    JPSFindPath(startMapPoint, goalMapPoint);
    AstarFindPath(startMapPoint, goalMapPoint);
    rrtFindPath(startMapPoint, goalMapPoint);
    HybridAstarFindPath(startMapPoint, goalMapPoint);
  }
}

/***
 * @description:
 * @param {ConstPtr} &msg
 * @return {*}
 */
void navSolution::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  pathNav.origin_x = msg->info.origin.position.x; // 获得栅格地图的原点x值(相对世界坐标系),单位为m
  pathNav.origin_y = msg->info.origin.position.y; // 获得栅格地图的原点y值(相对世界坐标系),单位为m
  pathNav.resolution = msg->info.resolution;      // 获得栅格地图的分辨率
  pathNav.width = msg->info.width;                // 获得栅格地图的宽
  pathNav.height = msg->info.height;              // 获得栅格地图的高
  ROS_INFO("\033[1;32m ***********map message**********\033[0m");
  ROS_INFO("\033[1;32m origin_x: %f  \033[0m",
           pathNav.origin_x);
  ROS_INFO("\033[1;32m origin_y: %f  \033[0m",
           pathNav.origin_y);
  ROS_INFO("\033[1;32m resolution: %f  \033[0m",
           pathNav.resolution);
  ROS_INFO("\033[1;32m width: %d  \033[0m",
           pathNav.width);
  ROS_INFO("\033[1;32m height: %d  \033[0m",
           pathNav.height);
  ROS_INFO("\033[1;32m *********************************\033[0m");

  NavtfGrid(msg, pathNav.mapData, pathNav.width, pathNav.height);
  mapInit(); // 地图初始化
}

/***
 * @description: 将ros解析的地图转变为正常的先行后列的顺序，非常非常非常重要
 * @param 1 读取出来的地图的参数  一维数组
 * @param 2 转换为一维算法能用的普通一维数组
 * @param 3 地图的x方向尺寸
 * @param 4 地图的y方向尺寸
 * @return {*}
 */
void navSolution::NavtfGrid(const nav_msgs::OccupancyGrid::ConstPtr &data, std::vector<int> &map, int x_size, int y_size)
{
  map.resize(x_size * y_size);
  for (int i = 0; i < x_size; i++)
  {
    for (int j = 0; j < y_size; j++)
    { /* 这行代码非常非常非常重要，将ros解析的地图转变为正常的先行后列的顺序 */
      map[i * y_size + j] = int(data->data[j * x_size + i]);
    }
  }
}

void navSolution::mapInit(void)
{
  astar_path_finder = new AstarPathFinder(); // 重置 防止之前的处理对于现在的产生影响
  jps_path_finder = new JPSPathFinder();     //
  rrt_path_finder = new RRTstarPlanner();

  if (this->choose == 1)
  {
    jps_path_finder->InitMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                             pathNav.width, pathNav.height, pathNav.mapData);
  }
  else if (this->choose == 2)
  {
    astar_path_finder->InitMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                               pathNav.width, pathNav.height, pathNav.mapData);
  }
  else if (this->choose == 3)
  {
    rrt_path_finder->initMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                             pathNav.width, pathNav.height, pathNav.mapData);
  }
  else if(this->choose == 4)
  {
    hybrid_astar_finder->InitMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                             pathNav.width, pathNav.height, pathNav.mapData);
  }

  else if (this->choose == 5)
  {
    jps_path_finder->InitMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                             pathNav.width, pathNav.height, pathNav.mapData);
    astar_path_finder->InitMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                               pathNav.width, pathNav.height, pathNav.mapData);
    rrt_path_finder->initMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                             pathNav.width, pathNav.height, pathNav.mapData);
    hybrid_astar_finder->InitMap(pathNav.resolution, pathNav.origin_x, pathNav.origin_y,
                             pathNav.width, pathNav.height, pathNav.mapData);
  }
}

/***
 * @description:
 * @param {ConstPtr} &msg
 * @return {*}
 */
void navSolution::StartPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
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

  node_vis.scale.x = pathNav.resolution * 2;
  node_vis.scale.y = pathNav.resolution * 2;
  node_vis.scale.z = pathNav.resolution * 2;
  geometry_msgs::Point pt;
  pt.x = msg->pose.pose.position.x;
  pt.y = msg->pose.pose.position.y;
  pt.z = 0.0;
  ROS_INFO("Start x:%f y:%f", pt.x, pt.y);
  node_vis.points.push_back(pt);

  StartPoint.publish(node_vis);
}

void navSolution::GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
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
  node_vis.scale.x = pathNav.resolution * 2;
  node_vis.scale.y = pathNav.resolution * 2;
  node_vis.scale.z = pathNav.resolution * 2;

  geometry_msgs::Point pt;
  pt.x = msg->pose.position.x;
  pt.y = msg->pose.position.y;
  pt.z = 0.0;
  node_vis.points.push_back(pt);
  GoalPoint.publish(node_vis);

  pathNav.goalMapPoint[0] = pt.x;
  pathNav.goalMapPoint[1] = pt.y;

  pathNav.startMapPoint[0] = pathNav.start_x;
  pathNav.startMapPoint[1] = pathNav.start_y;

  // 搜索路径
  StartFindPath(pathNav.startMapPoint, pathNav.goalMapPoint);
}

void navSolution::PublishPath(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> path)
{
  nav_msgs::Path pathTopic; // Astar路径的话题名
  pathTopic.poses.clear();  // 上一次搜索路径清空
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
    pathTopic.header.stamp = ros::Time::now();
    pathTopic.header.frame_id = "odom";
    pathTopic.poses.push_back(pathPose);
  }
  pathPublish.publish(pathTopic);
}

void navSolution::visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes)
{
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  if (is_use_jps)
  {
    node_vis.ns = "demo_node/jps_path";
    node_vis.color.a = 0.25;
    node_vis.color.r = 1.0;
    node_vis.color.g = 0.0;
    node_vis.color.b = 0.0;
  }
  else
  {
    node_vis.color.a = 0.25;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;
    node_vis.ns = "demo_node/astar_path";
  }

  node_vis.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis.action = visualization_msgs::Marker::ADD;
  node_vis.id = 0;

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.scale.x = pathNav.resolution * 2;
  node_vis.scale.y = pathNav.resolution * 2;
  node_vis.scale.z = pathNav.resolution * 2;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(visitnodes.size()); i++)
  {
    pt.x = visitnodes[i][0];
    pt.y = visitnodes[i][1];
    node_vis.points.push_back(pt);
  }

  pathPublish.publish(node_vis);
}

void navSolution::All(void)
{
  ros::NodeHandle nh("~");

  PathPub_JPS = nh.advertise<nav_msgs::Path>("/pathJPS", 15);     // 发布路径
  PathPub_Astar = nh.advertise<nav_msgs::Path>("/pathAstar", 15); // 发布路径
  PathPub_RRT = nh.advertise<nav_msgs::Path>("/pathRRTstar", 15); // 发布路径
  PathPub_Hybrid = nh.advertise<nav_msgs::Path>("/pathHybrid", 15); // 发布路径

  trajectory_pub_ = nh.advertise<nav_msgs::Path>("generated_trajectory", 10);        // 使用minimum_snap优化之后的路径
  mapSub = nh.subscribe("/map", 1, &navSolution::MapCallback, this);                 // 订阅栅格地图
  startSub = nh.subscribe("/initialpose", 1, &navSolution::StartPoseCallback, this); // 订阅起点
  // 固定起点
  goalSub = nh.subscribe("/move_base_simple/goal", 1, &navSolution::GoalPoseCallback, this); // 订阅终点
  StartPoint = nh.advertise<visualization_msgs::Marker>("/start_point", 1);                  // 可视化起点位置
  GoalPoint = nh.advertise<visualization_msgs::Marker>("/goal_point", 1);                    // 可视化终点位置

  visited_nodes_pub = nh.advertise<visualization_msgs::Marker>("visited_nodes_vis", 1);

  double astar_weight_g, astar_weight_h, jps_weight_g, jps_weight_h;
  int astar_heuristic, astar_glength, jps_heuristic, jps_glength;

  nh.param("astar_weight/g", astar_weight_g, 1.0);          // 权重a值
  nh.param("astar_weight/h", astar_weight_h, 1.0);          // 权重b值
  nh.param("astar_heuristic/distance", astar_heuristic, 0); // 0 1 2 3方法 欧氏距离 曼哈顿 切比学夫 对角线
  nh.param("astar_glength/distance", astar_glength, 0);

  nh.param("jps_weight/g", jps_weight_g, 1.0);          // 权重a值
  nh.param("jps_weight/h", jps_weight_h, 1.0);          // 权重b值
  nh.param("jps_heuristic/distance", jps_heuristic, 0); // 0 1 2 3方法 欧氏距离 曼哈顿 切比学夫 对角线
  nh.param("jps_glength/distance", jps_glength, 0);
  astar_path_finder->setParams(astar_weight_g, astar_weight_h, astar_glength, astar_heuristic);
  jps_path_finder->setParams(jps_weight_g, jps_weight_h, jps_glength, jps_heuristic);
  hybrid_astar_finder->InitParams(nh);


  double max_vel, max_acce;
  int order;

  nh.param("minimum_snap/max_vel", max_vel, 1.0);
  nh.param("minimum_snap/max_acce", max_acce, 1.0);
  nh.param("minimum_snap/order", order, 3);
  MinimumSnapFlow.setParams(order, max_vel, max_acce);

  // 固定起点
  nh.param("planning/start_x", pathNav.start_x, 2.0);
  nh.param("planning/start_y", pathNav.start_y, 0.0);
}
