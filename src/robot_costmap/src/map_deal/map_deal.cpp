/*
 * @Author: your name
 * @Date: 2023-04-24 17:41:38
 * @LastEditTime: 2024-04-18 23:33:43
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_costmap/src/map_deal/map_deal.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "map_deal/map_deal.h"

namespace map_deal
{
// 后面的这个用来传参
deal_all_map::deal_all_map(/* args */) : private_node_("~")
{
  // 初始化各项参数
  InitParams();

  // 订阅话题消息
  map_subscriber = node_handle_.subscribe("/map",1,&deal_all_map::MapCallback,this);
  // 订阅里程计消息
  if(is_use_sim)  // 仿真情况下使用
  {
    localizationSub = node_handle_.subscribe("/carto_odom",1,&deal_all_map::localizationCallback,this);
    // 订阅point cloud2点云消息用来发布局部地图
    pointCloud2Sub = node_handle_.subscribe("/cloud_map",10,&deal_all_map::pointCloud2Callback,this);
    // 订阅鼠标点击目标点
    clickSub = node_handle_.subscribe("/move_base_simple/goal", 10, &deal_all_map::clickCallback, this);
  }
  else
  {
    localizationSub = node_handle_.subscribe("/carto_odom",1,&deal_all_map::localizationCallback,this);
    // 订阅point cloud2点云消息用来发布局部地图
    pointCloud2Sub = node_handle_.subscribe("/cloud_map",10,&deal_all_map::pointCloud2Callback,this);

    clickSub = node_handle_.subscribe("/move_base_simple/goal", 10, &deal_all_map::clickCallback, this);
    // 订阅point cloud2点云消息用来发布局部地图
    //laser2CloudSub = node_handle_.subscribe("/cloud_map2",10,&deal_all_map::laser2Cloud2Callback,this);
  }

  // 发布全局地图消息
  global_inflate_map_publiser = node_handle_.advertise<nav_msgs::OccupancyGrid>("/global_map_inflate",10,true);
  // 发布全局距离场地图消息
  global_esdf_map_publisher = node_handle_.advertise<robot_communication::ESDFmap>("/global_esdf_map",10,true);
  // 发布可以在rviz中显示出来的势场地图
  global_esdf_map_display_publisher = node_handle_.advertise<nav_msgs::OccupancyGrid>("/global_map_esdf_display",10,true);
  // 发布全局代价地图消息
  global_cost_map_publisher = node_handle_.advertise<nav_msgs::OccupancyGrid>("/global_cost_map",10,true);

  // 发布局部地图消息
  local_map_publiser = node_handle_.advertise<nav_msgs::OccupancyGrid>("/local_map",10,true);
  // 发布局部膨胀地图消息
  local_map_inflate_publisher = node_handle_.advertise<nav_msgs::OccupancyGrid>("/local_map_inflate",10,true);
  // 发布局部esdf地图消息
  local_map_esdf_publisher = node_handle_.advertise<nav_msgs::OccupancyGrid>("/local_map_esdf",10,true);
  // 发布局部esdf地图消息
  local_esdf_map_publisher = node_handle_.advertise<robot_communication::ESDFmap>("/local_esdf_map",10,true);
  // 发布局部cost地图消息
  local_map_cost_publisher = node_handle_.advertise<nav_msgs::OccupancyGrid>("/local_map_cost",10,true);

  // 发布局部地图消息
  update_timer_ = node_handle_.createTimer(ros::Duration(0.05), &deal_all_map::updateLocalMapCallback, this);
  // 发布全局距离场地图消息
  // esdf_update_timer_ = node_handle_.createTimer(ros::Duration(0.1),&deal_all_map::updateEsdfMapCallback,this);

  // 订阅雷达话题消息
  laser_scan_subscriber_ = node_handle_.subscribe("/scan", 1, &deal_all_map::ScanCallback, this);

}

deal_all_map::~deal_all_map()
{

}

void deal_all_map::InitParams(void)
{
  // 获取launch文件中的参数
  private_node_.param("local_map/xmin", map_x_min,-10.0);
  private_node_.param("local_map/ymin", map_y_min,-10.0);
  private_node_.param("local_map/xmax", map_x_max,10.0);
  private_node_.param("local_map/ymax", map_y_max,10.0);
  private_node_.param("local_map/laser_min_theta", laser_min_theta,-3.14);
  private_node_.param("local_map/laser_max_theta", laser_max_theta,3.14);
  private_node_.param("local_map/resolution", resolution,0.05);
  resolution_inv = 1.0/resolution;
  // 获取膨胀半径
  private_node_.param("local_map/inflate",inflate,0.1);
  // 获取虚拟地图是实际地图大小倍数
  private_node_.param("local_map/virtual_map",virtual_map,1.5);
  // 获取是否仿真标志位
  private_node_.param("local_map/is_use_sim",is_use_sim,true);

  // 全局地图智能指针复位
  global_map_.reset(new global_map_deal);
  // 局部地图智能指针复位
  local_map_.reset(new local_map_deal);
  // ESDF地图的梯度处理
  edt_environment.reset(new EDTEnvironment);

  // 设定全局代价地图参数
  global_map_->Set_CostMaparams(private_node_);
  // 设定局部代价地图参数
  local_map_->Set_CostMaparams(private_node_);

  // 设定局部地图参数
  local_map_->Set_Maparams(resolution,inflate,true);
  // 获取地图大小
  localmap.info.width = (map_x_max-map_x_min)*resolution_inv;
  localmap.info.height = (map_y_max-map_y_min)*resolution_inv;
  // 这个一定得有要不然无法生成地图
  localmap.info.resolution = resolution;
  localmap.data.resize(localmap.info.width*localmap.info.height);
}

// 订阅鼠标点击点
void deal_all_map::clickCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  Vector2i gridpoint = worldToMap(Vector2d(msg->pose.position.x,msg->pose.position.y));
  cout<<"grid value is"<<(int)esdf_map_display_[gridpoint(0) + gridpoint(1)*global_map_x]<<endl;
}

// 雷达数据回调函数
void deal_all_map::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
  assemDirect = false;    //反向安装

  // 构建局部地图的时候认为雷达扫描范围之外是盲区
  if(assemDirect)
  {
    laser_max_theta = scan_msg->angle_max;
    laser_min_theta = scan_msg->angle_min;
  }
  else
  {
    laser_max_theta = scan_msg->angle_min;
    laser_min_theta = scan_msg->angle_max;
  }
}

// 订阅里程计消息
void deal_all_map::localizationCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_ = *msg;
  odom_.header.frame_id = "base_link";

  double roll,pitch,yaw;
  // 四元数向欧拉角转换，当前车的角度
  tf2::Quaternion quat(
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w
                       );
  // 四元数向欧拉角进行转换 对local的角度
  tf2::Matrix3x3(quat).getRPY(roll,pitch,yaw);

  robotYaw = yaw; 
  if(!is_use_sim)
  {
    static int time = 0;
    time++;
    if(has_odom_flag == false)
    {
      diffx = odom_.pose.pose.position.x;
      diffy = odom_.pose.pose.position.y;
    }

    if(time >= 200)
    {
      time = 0;
      has_odom_flag = true;
    }
  }
  else{
    has_odom_flag = true;
  }

  //cout<<"robotYaw:"<<robotYaw<<endl;
}

// point cloud2点云回调函数
void deal_all_map::pointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // 将pointCloud转换为pcl::PointCloud
  pcl::fromROSMsg(*msg,localcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg,"x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg,"y");

  // 初始化最小和最大角度
  float min_angle = std::numeric_limits<float>::max();
  float max_angle = std::numeric_limits<float>::min();

  // 遍历点云数据
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
    float x = *iter_x;
    float y = *iter_y;

    // 计算点(x, y, z)的角度
    float angle = atan2(y, x);

    // 更新最小和最大角度
    min_angle = std::min(min_angle, angle);
    max_angle = std::max(max_angle, angle);
  }

  local_map_->laser_max_theta = max_angle;
  local_map_->laser_min_theta = min_angle;

  // 输出最小和最大角度
  // ROS_INFO("Min Angle: %.2f degrees", min_angle * 180.0 / M_PI);
  // ROS_INFO("Max Angle: %.2f degrees", max_angle * 180.0 / M_PI);

  has_rece_cloud = true;
}

// laser2 point cloud2点云回调函数 
void deal_all_map::laser2Cloud2Callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // 将pointCloud转换为pcl::PointCloud
  pcl::fromROSMsg(*msg,laser2cloud);
}

void deal_all_map::MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
  ROS_INFO("\033[1;32m ***********map message**********\033[0m");
  ROS_INFO("\033[1;32m origin_x: %f  \033[0m",
           map->info.origin.position.x);
  ROS_INFO("\033[1;32m origin_y: %f  \033[0m",
           map->info.origin.position.y);
  ROS_INFO("\033[1;32m resolution: %f  \033[0m",
           map->info.resolution);
  ROS_INFO("\033[1;32m width: %d  \033[0m",
           map->info.width);
  ROS_INFO("\033[1;32m height: %d  \033[0m",
           map->info.height);
  ROS_INFO("\033[1;32m *********************************\033[0m");

  origin_x = map->info.origin.position.x;
  origin_y = map->info.origin.position.y;

  // 传递全局静态地图参数
  globalmap = *map;
  // 设定膨胀地图数据
  globalinflatemap = *map;

  global_map_x = globalinflatemap.info.width;
  global_map_y = globalinflatemap.info.height;

  // 设定全局地图参数
  global_map_->Set_Maparams(resolution,inflate,origin_x,origin_y,global_map_x,global_map_y);

  Vector2i actual_map_size,actual_map_start;
  // 膨胀地图
  global_map_->Infalte_Global_Map(
              Vector2i(global_map_x,global_map_y),
              &actual_map_size,
              &actual_map_start,
              globalinflatemap.data,
              inflate_map_);
              
  actual_map_x = actual_map_size[0];
  actual_map_y = actual_map_size[1];
  actual_map_startx = actual_map_start[0];
  actual_map_starty = actual_map_start[1];

  // 传递膨胀地图参数
  globalinflatemap.data = inflate_map_;
  // 发布膨胀地图
  global_inflate_map_publiser.publish(globalinflatemap);

  // 提取有效地图
  global_map_->Get_ActualMap
                (Vector2i(global_map_x,global_map_y),
                Vector2i(actual_map_x,actual_map_y),
                Vector2i(actual_map_startx,actual_map_starty),
                globalinflatemap.data,
                occupancy_buffer_);

  // 获取全局ESDF地图 该地图能够以ROS的消息格式进行发布
  global_map_->Get_ESDFMap
              (Vector2i(global_map_x,global_map_y),
              Vector2i(actual_map_x,actual_map_y),
              Vector2i(actual_map_startx,actual_map_starty),
              occupancy_buffer_,
              esdf_map_display_);

  // 传参
  globalesdfmap = globalinflatemap;
  // 对esdf地图赋值
  globalesdfmap.data = esdf_map_display_;

  // 发布可以在rviz中显示出来的势场地图
  global_esdf_map_display_publisher.publish(globalesdfmap);

  globalesdftest.origin_x = globalinflatemap.info.origin.position.x;
  globalesdftest.origin_y = globalinflatemap.info.origin.position.y;
  globalesdftest.width = globalinflatemap.info.width;
  globalesdftest.height = globalinflatemap.info.height;
  globalesdftest.resolution = globalinflatemap.info.resolution;

  // 准备发布势场地图
  global_map_->ESDF_Map(globalinflatemap.data,esdf_map_buffer_);
  globalesdftest.data = esdf_map_buffer_;
  global_esdf_map_publisher.publish(globalesdftest);

  
  // edt_environment->setMap(global_map_);
  // edt_environment->init();

  // 对代价地图赋值
//   globalcostmap = globalinflatemap;
//   global_map_->Set_Cost_Map(Vector2i(globalinflatemap.info.width,globalinflatemap.info.height),
//                             globalinflatemap.data,
//                             globalcostmap.data);
//   // 发布代价地图
//   global_cost_map_publisher.publish(globalcostmap);

  has_global_map = true;
  has_been_inflate = true;

}

// 更新距离场地图
void deal_all_map::updateEsdfMapCallback(const ros::TimerEvent &e)
{
  if(!has_been_inflate) return; //没有膨胀拉倒

  // 更新代价地图
  // 当前机器人位置作为local map原点
  Eigen::Vector2d center(odom_.pose.pose.position.x, odom_.pose.pose.position.y);
  // 代价地图感知范围
  Eigen::Vector2d disp(map_x_max-map_x_min,map_y_max-map_y_min);
}

// 更新局部地图消息
void deal_all_map::updateLocalMapCallback(const ros::TimerEvent &e)
{
  // 没有点云信息拉到
  if(!has_rece_cloud){return;}
  if(!has_odom_flag) return;    //没有里程计拉到


  // 当前机器人位置作为local map原点
  Eigen::Vector3d center(odom_.pose.pose.position.x, odom_.pose.pose.position.y,0);
  
  if (isnan(center(0)) || isnan(center(1)) || isnan(center(2)))
    return;

  // 代价地图感知范围
  Eigen::Vector3d disp(map_x_max-map_x_min,map_y_max-map_y_min,0);

  // 定义膨胀多少格
  const int ifn = ceil(inflate * resolution_inv);
  
  // 获取x y轴上下限
  Vector2d lower_x_y_world = Vector2d((center[0]-disp[0]/2),(center[1]-disp[1]/2));
  Vector2d upper_x_y_world = Vector2d((center[0]+disp[0]/2),(center[1]+disp[1]/2));

  // 求在栅格地图中的整数坐标映射
  Vector2i lower_x_y_map = worldToMap(lower_x_y_world);
  Vector2i upper_x_y_map = worldToMap(upper_x_y_world);
  // 获取局部地图x y轴相对于栅格地图的整数数值
  int x_length = disp[0] * resolution_inv;
  int y_length = disp[1] * resolution_inv;

  localmap.header.frame_id = "odom";
  // 获取当前的位置
  localmap.info.origin.position.x = lower_x_y_world[0];
  localmap.info.origin.position.y = lower_x_y_world[1];
  localmap.info.origin.position.z = 0;

  localmap.info.origin.orientation.x = 0;
  localmap.info.origin.orientation.y = 0;
  localmap.info.origin.orientation.z = 0;
  localmap.info.origin.orientation.w = 1;

  // 将地图清空
  localmap.data.assign(localmap.info.width*localmap.info.height,0);

  // 转换获得当前车的姿态角
  // 定义当前车的姿态 全局位置姿态和yaw姿态
  robotPose = Vector3d(odom_.pose.pose.position.x,odom_.pose.pose.position.y,robotYaw);

  obstacleLocal.clear();
  localmap.data = local_map_->Local_Tran_Global_Map(lower_x_y_world,
                                        Vector2i(localmap.info.width,localmap.info.height),
                                        robotPose,
                                        (localcloud+laser2cloud),
                                        obstacleLocal,
                                        Vector2d(diffx,diffy));

  localmap.header.stamp = ros::Time::now();
  // 发布局部地图 最普通的局部地图
  local_map_publiser.publish(localmap);

  localinflatemap = localmap;
  // 膨胀局部地图
  local_map_->Infalte_Local_Map(Vector2i(localmap.info.width,localmap.info.height),
                    obstacleLocal,
                    localinflatemap.data);

  // 发布局部膨胀地图
  local_map_inflate_publisher.publish(localinflatemap);

  // 发布代价地图
  localcostmap = localinflatemap;
  
  local_map_->Set_Cost_Map(Vector2i(localinflatemap.info.width,localinflatemap.info.height),
                          localinflatemap.data,
                          localcostmap.data);

  local_map_cost_publisher.publish(localcostmap);

  ros::Time start = ros::Time::now();

  vector<int8_t> tempesdf;
  
  // 
  localesdfmap = localinflatemap;
  // 获取欧几里德距离场地图
  local_map_->Get_ESDF_Local_Map(Vector2i(localmap.info.width,localmap.info.height),
                                  localinflatemap.data,
                                  tempesdf);
  localesdfmap.info.width = virtual_map*localmap.info.width;
  localesdfmap.info.height = virtual_map*localmap.info.height;

  localesdfmap.info.origin.position.x = localinflatemap.info.origin.position.x - (virtual_map-1)*map_x_max;
  localesdfmap.info.origin.position.y = localinflatemap.info.origin.position.y - (virtual_map-1)*map_y_max;
  
  // 重构容器大小
  localesdfmap.data.resize(localesdfmap.info.width*localesdfmap.info.height);
  localesdftest.data.resize(localesdfmap.info.width*localesdfmap.info.height);

  fill(localesdfmap.data.begin(),localesdfmap.data.end(),0);

  // 构造虚拟地图
  for (int x=(localesdfmap.info.width-localmap.info.width)/2;
  x<(localesdfmap.info.width-localmap.info.width)/2+localmap.info.width;
  x++)
  {
    for (int y=(localesdfmap.info.width-localmap.info.height)/2;
    y<(localesdfmap.info.width-localmap.info.height)/2+localmap.info.height;
    y++)
    {
      localesdfmap.data[x+y*localesdfmap.info.width] = 
      tempesdf[(x-(localesdfmap.info.width-localmap.info.height)/2)+
      (y-(localesdfmap.info.width-localmap.info.height)/2)*localmap.info.width];

      localesdftest.data[x+y*localesdfmap.info.width] = 
      tempesdf[(x-(localesdfmap.info.width-localmap.info.height)/2)
      +(y-(localesdfmap.info.width-localmap.info.height)/2)*localmap.info.width];
    }
  }
  ros::Time end = ros::Time::now();

  // 发布带有虚拟地图的欧几里德距离场地图
  local_map_esdf_publisher.publish(localesdfmap);

  // 获取地图起点
  localesdftest.origin_x = localesdfmap.info.origin.position.x;
  localesdftest.origin_y = localesdfmap.info.origin.position.y;
  // 获取地图x y大小
  localesdftest.width = localesdfmap.info.width;
  localesdftest.height = localesdfmap.info.height;
  // 
  localesdftest.resolution = localesdfmap.info.resolution;

  // 发布自定义消息格式下的欧几里德距离场地图
  local_esdf_map_publisher.publish(localesdftest);
}

}


