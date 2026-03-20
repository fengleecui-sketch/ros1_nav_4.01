/*
 * @Author: your name
 * @Date: 2024-04-19 12:20:11
 * @LastEditTime: 2024-04-19 15:00:57
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/src/local_plan/Hybrid_astar_local.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "local_plan/Hybrid_astar_local.h"

// 如果没有私有节点，launch文件中的参数加载不进来，目前还不知道为什么，但是一定要像这样使用
Hybrid_astar_local::Hybrid_astar_local(/* args */) : private_node("~")
{
  // 初始化
  LocalPlanInit();

  ros::Rate LoopRate(control_hz);
  while (ros::ok())
  {
    auto timeSatrt = steady_clock::now();
    // 回调函数数据刷新
    ros::spinOnce();
    pathPlanning(startPoint,localgoalpoint);
    //--运行速度测量
    auto timeEnd = steady_clock::now();
    auto timeDuration = duration_cast<microseconds>(timeEnd - timeSatrt);
    // cout << "运行时间 " << timeDuration.count()/1000 << endl;
    LoopRate.sleep();
  }
}

Hybrid_astar_local::~Hybrid_astar_local()
{
}

void Hybrid_astar_local::LocalPlanInit(void)
{
  // 获取是否仿真
  private_node.param("Hybird_local/is_sim", is_sim, false); // 权重a值
  // 获取控制频
  private_node.param("Hybird_local/control_hz",control_hz,10);

  // 初始化混合astar
  hybrid_astar_finder.reset(new Hybrid_astar);
  // 初始化参数
  hybrid_astar_finder->InitParams(private_node);

  // 订阅局部地图消息
  dynamicMap = local_plan.subscribe("/local_map_esdf", 10, &Hybrid_astar_local::dynamicMapCallback, this);
  // 订阅路径
  pathSub = local_plan.subscribe("/opt_path",10,&Hybrid_astar_local::pathCallback,this);  
  // 查看局部终点
  localgoalPub = local_plan.advertise<visualization_msgs::Marker>("/local_goal",10);

  // 优化路径发布
  localPathPub = local_plan.advertise<nav_msgs::Path>("/local_path", 10);

  // 仿真和实际的定位消息是不一样的
  // if (is_sim == false)
  // {
  //   // 订阅定位消息
  //   localizationSub = local_plan.subscribe("/odom_carto", 10, &Hybrid_astar_local::localizationCallback, this);
  // }
  // else
  // {
    // 订阅真值里程计进行仿真
    localizationSub = local_plan.subscribe("/truth_pose_odom", 10, &Hybrid_astar_local::localizationCallback, this);
  // }
}

void Hybrid_astar_local::localizationCallback(const robot_communication::localizationInfoBroadcastConstPtr &msg)
{
  // 获取起点等规划消息
  localData.xPosition = msg->xPosition;
  localData.yPosition = msg->yPosition;
  localData.xSpeed = msg->xSpeed;
  localData.ySpeed = msg->ySpeed;
  localData.xAccel = msg->xAccel;
  localData.yAccel = msg->yAccel;
  localData.chassisGyro = msg->chassisGyro;
  localData.chassisAngle = msg->chassisAngle;

  startPoint[0] = msg->xPosition;
  startPoint[1] = msg->yPosition;

  if(fabs((startPoint-localgoalpoint).norm()) <= 0.1)
  {
    has_arrived_end = true;
  }
  else{
    has_arrived_end = false;
  }

  getStartFlag = true; // 获取到起点标志位
}

void Hybrid_astar_local::dynamicMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
  vector<int> tempmap;
  tempmap.resize(map->info.width * map->info.height);
  for (int i = 0; i < map->info.width; i++)
  {
    for (int j = 0; j < map->info.height; j++)
    { /* 这行代码非常非常非常重要，将ros解析的地图转变为正常的先行后列的顺序 */
      tempmap[i * map->info.height + j] = int(map->data[j * map->info.width + i]);
    }
  }

  hybrid_astar_finder->InitMap(map->info.resolution, 
                              map->info.origin.position.x, 
                              map->info.origin.position.y,
                              map->info.width, 
                              map->info.height,tempmap);

  // cout<<"map size:"<<map->info.width<<"  "<<map->info.height<<endl;
  localMapFlag = true;
}

// 路径回调函数
void Hybrid_astar_local::pathCallback(const nav_msgs::PathConstPtr &path)
{
  nav_msgs::Path tempath = *path;

  // 获取当前路径点数目
  path_nodes_num = tempath.poses.size();
  // 判断路径是不是空的
  if(path_nodes_num <= 0)
  {
    return;
  }

  // 当两次路径点数目没有发生改变，认为当前的终点没有发生变化，还是原来的路径
  if(path_nodes_num == last_path_nodes_num)
  {
    // 路径没有更新
    update_path = false;
  }
  if(path_nodes_num != last_path_nodes_num)
  {
    // 路径更新了
    update_path = true;
  }

  // 路径更新了就把新的路径放进来
  if(update_path == true)
  {
    // 跟随路径清空
    trajpath.clear();
    // 更新
    visitPath.clear();
    // 对跟随路径进行赋值
    for (int i = 0; i < path_nodes_num; i++)
    {
      // 临时路径点用于数据类型转换
      // 设定为0代表新的路径所有点没有被访问过
      Vector2d tempoint =Vector2d (tempath.poses[i].pose.position.x,
                                   tempath.poses[i].pose.position.y);
      // 添加到跟随路径中
      trajpath.push_back(tempoint);
      visitPath.push_back(make_pair(0,tempoint));
    }
  }

  // 当没更新的时候
  if(update_path == false)
  {
    // 定义临时终点
    Vector2d tempgoal;
    // 得出位置角度
    Vector2d tempnow = startPoint;
    tempgoal = caLocalGoalPosition(trajpath,tempnow,TEMP_GOAL_RADIUS);
    // 设定水平向量
    Vector2d horizontal = Vector2d(1.0,0);
    double angle =  calVectorAngle(horizontal,tempgoal-tempnow);
    // 使用templocal中最后一个担任临时终点
    localgoalpoint = tempgoal;
  }
  vector<Vector2d> tempvec;
  tempvec.push_back(localgoalpoint);
  visual_VisitedNode(localgoalPub,tempvec,1,0,1,0.5,5);

  // 获取当前路径点的数目用作比较
  last_path_nodes_num = path_nodes_num;
}

// 计算两个点之间的长度欧氏距离
double Hybrid_astar_local::calPointLength(Vector2d vector1,Vector2d vector2)
{
  return (sqrt((vector1[0]-vector2[0])*(vector1[0]-vector2[0])+(vector1[1]-vector2[1])*(vector1[1]-vector2[1])));
}

// 用来求解当前位置点以一定半径相交的路径点
// path 输入路径
// radius 搜索半径
// updatepath 路径是否更新
// return 局部终点
Vector2d Hybrid_astar_local::caLocalGoalPosition(vector<Vector2d> path,Vector2d nowpoint,double radius)
{
  Vector2d localgoal;
  static int last_point_num;
  int path_length_num = path.size();

  // 计算当前位置到路径点上的最小距离
  double min_distance = 1000;
  int now_num;    //定位当前点再路径上的位置
  for (int i = 0; i < path_length_num; i++)
  {
    // 计算距离
    double distance = calPointLength(path[i],nowpoint);
    if(distance < min_distance)
    {
      min_distance = distance;
      now_num = i;
    }
  }

  // 跟随路径点
  int traj_num = 0;
  for (int i = now_num+1; i < path_length_num; i++)
  {
    // 计算距离
    double distance = calPointLength(visitPath[now_num].second,visitPath[i].second);
    // 距离约束并且一定不能访问过
    if(distance >= radius && distance < radius + 0.3 && visitPath[i].first == 0)
    {
      traj_num = i;
      break;
    }
  }

  // 路径更新
  if(update_path == true)
  {
    last_point_num = 0;
  }

  // 等于上一个
  if(traj_num == 0)
  {
    // 获取上一个点
    traj_num = last_point_num;
  }

  // 之前的点设定为1
  for (int i = 0; i < traj_num; i++)
  {
    visitPath[i].first = 1;
  }

  // 确定下一个点的采样位置
  if(traj_num < path_length_num-1)
  {
    localgoal = path[traj_num];
  }
  else
  {
    localgoal = path[path_length_num-1];
  }

  // 判断得出的局部终点到最后终点的位置
  double lastdis = calPointLength(localgoal,path[path_length_num-1]);
  if(lastdis <= radius+0.3)
  {
    localgoal = path[path_length_num-1]; 
  } 

  // 获取点
  last_point_num = traj_num;
  return localgoal;
}

// 计算单位向量
Vector2d Hybrid_astar_local::calUnitvector(Vector2d unitv)
{
  // 计算单位向量
  unitv = unitv * 1.0f/(sqrt(pow(abs(unitv[0]),2)+pow(abs(unitv[1]),2)));
  return unitv;
}

// 计算向量之间的夹角
double Hybrid_astar_local::calVectorAngle(Vector2d vector1,Vector2d vector2)
{
  // 先单位化
  Vector2d vectorFirst = calUnitvector(vector1);
  // 
  Vector2d vectorSecond = calUnitvector(vector2);

  // 向量乘积
  double vector_angle = vectorFirst[0]*vectorSecond[0] + vectorFirst[1]*vectorSecond[1];
  // 计算夹角
  return acos(vector_angle);
}

// 路径规划
void Hybrid_astar_local::pathPlanning(Vector2d startMapPoint,Vector2d goalMapPoint)
{
  cout<<"has_arrived_end:"<<has_arrived_end<<endl;
  cout<<"localMapFlag:"<<localMapFlag<<endl;
  cout<<"getStartFlag:"<<getStartFlag<<endl;

  if(has_arrived_end){
    cout<<"Arrived the end!"<<endl;
    return;
  }

  // 开始进行规划
  if (localMapFlag == true && getStartFlag == true )
  {
    // 记录路径搜索需要的时间
    ros::Time time_1 = ros::Time::now();

    // 再次复位
    hybrid_astar_finder->reset();
    // 重新搜索
    int status = hybrid_astar_finder->search(startMapPoint,
                                  Eigen::Vector2d(0, 0),
                                  Eigen::Vector2d(0, 0),
                                  goalMapPoint, 
                                  Eigen::Vector2d(0, 0),true,false,-1.0);
    if (status == Hybrid_astar::NO_PATH)
    {
      cout << "[planner]: init search fail!" << endl;
      hybrid_astar_finder->reset();
      status = hybrid_astar_finder->search(startMapPoint, 
                                            Eigen::Vector2d(0, 0), 
                                            Eigen::Vector2d(0, 0), 
                                            goalMapPoint, 
                                            Eigen::Vector2d(0, 0), false, false,-1.0);
      if (status == Hybrid_astar::NO_PATH)
      {
        cout << "[planner]: Can't find path." << endl;
      }
      else
      {
        cout << "[planner]: retry search success." << endl;
      }
    }
    ros::Time time_2 = ros::Time::now(); 

    pathNav.worldpath = hybrid_astar_finder->getKinoTraj(0.02);
    ROS_WARN("hybrid a star search time is:%f",(time_2-time_1).toSec() * 1000.0);

    PublishPath(localPathPub, pathNav.worldpath); // 发布路径    
  }
}

void Hybrid_astar_local::PublishPath(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> path)
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

// 发布可视化节点
void Hybrid_astar_local::visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
float a_set,float r_set,float g_set,float b_set,float length)
{
  visualization_msgs::Marker node_vis;
  node_vis.header.frame_id = "map";
  node_vis.header.stamp = ros::Time::now();

  node_vis.color.a = a_set;
  node_vis.color.r = r_set;
  node_vis.color.g = g_set;
  node_vis.color.b = b_set;
  node_vis.ns = "omnidirectional_dwa";

  node_vis.type = visualization_msgs::Marker::CUBE_LIST;
  node_vis.action = visualization_msgs::Marker::ADD;
  node_vis.id = 0;

  node_vis.pose.orientation.x = 0.0;
  node_vis.pose.orientation.y = 0.0;
  node_vis.pose.orientation.z = 0.0;
  node_vis.pose.orientation.w = 1.0;

  node_vis.scale.x = 0.05*length;
  node_vis.scale.y = 0.05*length;
  node_vis.scale.z = 0.05*length;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(visitnodes.size()); i++)
  {
    pt.x = visitnodes[i][0];
    pt.y = visitnodes[i][1];
    node_vis.points.push_back(pt);
  }

  pathPublish.publish(node_vis);
}


