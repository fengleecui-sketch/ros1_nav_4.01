#include "map_deal/local_map_deal.h"

namespace map_deal
{

local_map_deal::local_map_deal(/* args */)
{
  resolution = 0.05;
  resolution_inv = 1.0/resolution;

  inflate = 0.5;
}

local_map_deal::~local_map_deal()
{
}

void local_map_deal::Set_Maparams(double resolution_,double inflate_,bool _use_sim)
{
  resolution = resolution_;
  resolution_inv = 1.0/resolution;

  is_use_sim = _use_sim;

  inflate = inflate_;
}

void local_map_deal::Set_CostMaparams(ros::NodeHandle &nh)
{
  // 设定全局代价地图上下阈值
  nh.param("local_cost_map/costdownvalue",costdownvalue,40);
  nh.param("local_cost_map/costupvalue",costupvalue,54);

  // 设定全局代价地图上下阈值系数 , 这两个系数与上下阈值的乘积不能超过100
  nh.param("local_cost_map/costdownk",costdownk,0.0);
  nh.param("local_cost_map/costupk",costupk,1.5);

  cout<<"up k"<<costupk<<endl;
  cout<<"down k"<<costdownk<<endl;

  cout<<"up value"<<costupvalue<<endl;
  cout<<"down value"<<costdownvalue<<endl;
}

// 局部地图转全局地图
// localPosition 局部地图的位置
// mapSize 地图尺寸参数
// robotPose 机器人位置姿态
// pclocal 局部点云数据
// obs 获得障碍物集合
vector<int8_t> local_map_deal::Local_Tran_Global_Map(Vector2d localPosition,Vector2i mapSize,
Vector3d robotPose,pcl::PointCloud<pcl::PointXYZ> pclocal,vector<Vector2i> &obs,Vector2d diffxy)
{
  int x_size = mapSize[0];
  int y_size = mapSize[1];

  // 设定返回的局部地图数据
  vector<int8_t> mapdata;
  // 重构地图容器大小
  mapdata.resize(x_size*y_size);
  mapdata.assign(x_size*y_size,0);

  // 车尾部的角度为车头的角度-PI
  double robotTailtheta = robotPose[2]-M_PI;

  if(robotTailtheta <= -M_PI)
  {
    robotTailtheta += 2.0*M_PI;
  }
  else if(robotTailtheta > M_PI)
  {
    robotTailtheta -= 2.0*M_PI;
  }

  // cout<<"robotTailtheta:"<<robotTailtheta<<endl;

  for(const auto&point : pclocal){
    double world_x;
    double world_y;

    if(is_use_sim)
    {
      // 将局部点位转换为全局点位
      world_x = cos(robotPose[2]) * point.x - sin(robotPose[2])*point.y + robotPose[0];
      world_y = sin(robotPose[2]) * point.x + cos(robotPose[2])*point.y + robotPose[1];
    }
    else
    {
      // 将局部点位转换为全局点位
      world_x = cos(robotPose[2]) * point.x - sin(robotPose[2])*point.y + robotPose[0]-diffxy(0);
      world_y = sin(robotPose[2]) * point.x + cos(robotPose[2])*point.y + robotPose[1]-diffxy(1);
    }

    int x = (world_x - localPosition[0])*resolution_inv;
    int y = (world_y - localPosition[1])*resolution_inv;

    // 检测point是否在栅格地图中
    if( x>=0 && x < x_size && 
        y>=0 && y < y_size )
    {
      mapdata[x+x_size*y] = 100;
      obs.push_back(Vector2i(x,y));
    }
  }

  return mapdata;  
}

// 局部地图膨胀
// mapSize地图尺寸参数
// obstacle获取障碍物集合
// oldmap原本的地图参数
// inflatemap膨胀地图
void local_map_deal::Infalte_Local_Map(Vector2i mapSize,vector<Vector2i> obstacle,vector<int8_t> &inflatemap)
{
  int x_size = mapSize[0];
  int y_size = mapSize[1];

  // 重构膨胀地图大小
  inflatemap.resize(x_size*y_size);
  inflatemap.assign(x_size*y_size,0);

  // 定义膨胀多少格
  const int ifnlocal = ceil(inflate * resolution_inv);

  // cout<<"inflocal:"<<ifnlocal<<endl;

  for(int i=0;i<obstacle.size();i++)
  {
    /* inflate the point */
    for (int x = -ifnlocal; x <= ifnlocal; ++x)
    {
      for (int y = -ifnlocal; y <= ifnlocal; ++y)
      {
        if((obstacle[i][0]+x)+(obstacle[i][1]+y)*x_size < 0 ||
            (obstacle[i][0]+x) < 0 ||
            (obstacle[i][1]+y) < 0 ||
            (obstacle[i][0]+x) > x_size ||
            (obstacle[i][1]+y) > y_size ||
            (obstacle[i][0]+x)+(obstacle[i][1]+y)*x_size >= x_size*y_size)
            {
              continue;
            }
        else
        {
          inflatemap[(obstacle[i][0]+x)+(obstacle[i][1]+y)*x_size] = 100;
        }
      }
    }
  }    
}

// 局部代价地图
// mapSize地图尺寸参数
// inflatemap膨胀地图
// costmap代价地图
void local_map_deal::Set_Cost_Map(Vector2i mapSize,vector<int8_t> inflatemap,vector<int8_t> &costmap)
{
  int x_size = mapSize[0];
  int y_size = mapSize[1];

  // 重构膨胀地图大小
  costmap.resize(x_size*y_size);
  costmap.assign(x_size*y_size,0);

  // 获取膨胀后的障碍物数组
  vector<Vector2i> obstacle;
  
  // 按照局部地图位置和规格遍历当前位置的全局地图，获得当前的全局地图障碍物信息
  for(int x=0;x<x_size;x++){
    for(int y=0;y<y_size;y++)
    {
      // 当膨胀地图数值为100 则为障碍物
      if(inflatemap[x+y*x_size] == 100)
      {
        // 将障碍物添加
        obstacle.push_back(Vector2i(x,y));
      }
    }
  }

  // 对代价地图初始化
  costmap = inflatemap;

  //三格代价 90 70 50
  const int costlocal = 3;  //三个格子

  for(int i=0;i<obstacle.size();i++)
  {
    /* inflate the point */
    for (int x = -1; x <= 1; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        if((obstacle[i][0]+x)+(obstacle[i][1]+y)*x_size < 0 ||
            (obstacle[i][0]+x) < 0 ||
            (obstacle[i][1]+y) < 0 ||
            (obstacle[i][0]+x) > x_size ||
            (obstacle[i][1]+y) > y_size ||
            (obstacle[i][0]+x)+(obstacle[i][1]+y)*x_size >= x_size*y_size ||
            costmap[(obstacle[i][0]+x)+(obstacle[i][1]+y)*x_size] == 100 )
            {
              continue;
            }
        else
        {
          costmap[(obstacle[i][0]+x)+(obstacle[i][1]+y)*x_size] = 90;
        }
      }
    }
  }
  // 获取膨胀后的障碍物数组
  vector<Vector2i> cost1;
  for(int x=0;x<x_size;x++){
    for(int y=0;y<y_size;y++)
    {
      // 当膨胀地图数值为100 则为障碍物
      if(costmap[x+y*x_size] == 90)
      {
        // 将障碍物添加
        cost1.push_back(Vector2i(x,y));
      }
    }
  }
  
  // 再次膨胀
  for(int i=0;i<cost1.size();i++)
  {
    /* inflate the point */
    for (int x = -1; x <= 1; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        if((cost1[i][0]+x)+(cost1[i][1]+y)*x_size < 0 ||
            (cost1[i][0]+x) < 0 ||
            (cost1[i][1]+y) < 0 ||
            (cost1[i][0]+x) > x_size ||
            (cost1[i][1]+y) > y_size ||
            (cost1[i][0]+x)+(cost1[i][1]+y)*x_size >= x_size*y_size ||
            costmap[(cost1[i][0]+x)+(cost1[i][1]+y)*x_size] == 100 ||
            costmap[(cost1[i][0]+x)+(cost1[i][1]+y)*x_size] == 90)
            {
              continue;
            }
        else
        {
          costmap[(cost1[i][0]+x)+(cost1[i][1]+y)*x_size] = 70;
        }
      }
    }
  }

  // 获取膨胀后的障碍物数组
  vector<Vector2i> cost2;
  for(int x=0;x<x_size;x++){
    for(int y=0;y<y_size;y++)
    {
      // 当膨胀地图数值为100 则为障碍物
      if(costmap[x+y*x_size] == 70)
      {
        // 将障碍物添加
        cost2.push_back(Vector2i(x,y));
      }
    }
  }

  // 再次膨胀
  for(int i=0;i<cost2.size();i++)
  {
    /* inflate the point */
    for (int x = -1; x <= 1; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        if((cost2[i][0]+x)+(cost2[i][1]+y)*x_size < 0 ||
            (cost2[i][0]+x) < 0 ||
            (cost2[i][1]+y) < 0 ||
            (cost2[i][0]+x) > x_size ||
            (cost2[i][1]+y) > y_size ||
            (cost2[i][0]+x)+(cost2[i][1]+y)*x_size >= x_size*y_size ||
            costmap[(cost2[i][0]+x)+(cost2[i][1]+y)*x_size] == 100 ||
            costmap[(cost2[i][0]+x)+(cost2[i][1]+y)*x_size] == 90  ||
            costmap[(cost2[i][0]+x)+(cost2[i][1]+y)*x_size] == 70)
            {
              continue;
            }
        else
        {
          costmap[(cost2[i][0]+x)+(cost2[i][1]+y)*x_size] = 50;
        }
      }
    }
  }

  // 获取膨胀后的障碍物数组
  // vector<Vector2i> cost3;
  // for(int x=0;x<x_size;x++){
  //   for(int y=0;y<y_size;y++)
  //   {
  //     // 当膨胀地图数值为100 则为障碍物
  //     if(costmap[x+y*x_size] == 50)
  //     {
  //       // 将障碍物添加
  //       cost3.push_back(Vector2i(x,y));
  //     }
  //   }
  // }
  // // 再次膨胀
  // for(int i=0;i<cost3.size();i++)
  // {
  //   /* inflate the point */
  //   for (int x = -1; x <= 1; ++x)
  //   {
  //     for (int y = -1; y <= 1; ++y)
  //     {
  //       if((cost3[i][0]+x)+(cost3[i][1]+y)*x_size < 0 ||
  //           (cost3[i][0]+x) < 0 ||
  //           (cost3[i][1]+y) < 0 ||
  //           (cost3[i][0]+x) > x_size ||
  //           (cost3[i][1]+y) > y_size ||
  //           (cost3[i][0]+x)+(cost3[i][1]+y)*x_size >= x_size*y_size ||
  //           costmap[(cost3[i][0]+x)+(cost3[i][1]+y)*x_size] == 100 ||
  //           costmap[(cost3[i][0]+x)+(cost3[i][1]+y)*x_size] == 90  ||
  //           costmap[(cost3[i][0]+x)+(cost3[i][1]+y)*x_size] == 70  ||
  //           costmap[(cost3[i][0]+x)+(cost3[i][1]+y)*x_size] == 50)
  //           {
  //             continue;
  //           }
  //       else
  //       {
  //         costmap[(cost3[i][0]+x)+(cost3[i][1]+y)*x_size] = 30;
  //       }
  //     }
  //   }
  // }

  // // 获取膨胀后的障碍物数组
  // vector<Vector2i> cost4;
  // for(int x=0;x<x_size;x++){
  //   for(int y=0;y<y_size;y++)
  //   {
  //     // 当膨胀地图数值为100 则为障碍物
  //     if(costmap[x+y*x_size] == 30)
  //     {
  //       // 将障碍物添加
  //       cost4.push_back(Vector2i(x,y));
  //     }
  //   }
  // }
  // // 再次膨胀
  // for(int i=0;i<cost4.size();i++)
  // {
  //   /* inflate the point */
  //   for (int x = -1; x <= 1; ++x)
  //   {
  //     for (int y = -1; y <= 1; ++y)
  //     {
  //       if((cost4[i][0]+x)+(cost4[i][1]+y)*x_size < 0 ||
  //           (cost4[i][0]+x) < 0 ||
  //           (cost4[i][1]+y) < 0 ||
  //           (cost4[i][0]+x) > x_size ||
  //           (cost4[i][1]+y) > y_size ||
  //           (cost4[i][0]+x)+(cost4[i][1]+y)*x_size >= x_size*y_size ||
  //           costmap[(cost4[i][0]+x)+(cost4[i][1]+y)*x_size] == 100 ||
  //           costmap[(cost4[i][0]+x)+(cost4[i][1]+y)*x_size] == 90  ||
  //           costmap[(cost4[i][0]+x)+(cost4[i][1]+y)*x_size] == 70  ||
  //           costmap[(cost4[i][0]+x)+(cost4[i][1]+y)*x_size] == 50  ||
  //           costmap[(cost4[i][0]+x)+(cost4[i][1]+y)*x_size] == 30
  //           )
  //           {
  //             continue;
  //           }
  //       else
  //       {
  //         costmap[(cost4[i][0]+x)+(cost4[i][1]+y)*x_size] = 10;
  //       }
  //     }
  //   }
  // }

}

// 获取局部ESDF地图
// mapSize 地图尺寸大小
// occupancy_map 需要进行esdf的地图
// esdfmap 人工势场地图
void local_map_deal::Get_ESDF_Local_Map(Vector2i mapSize,vector<int8_t> occupancy_map,vector<int8_t> &esdfmap)
{
  int x_size = mapSize[0];
  int y_size = mapSize[1];

  // 获得膨胀地图
  std::array<int, 2> size{x_size,y_size};
  GridMap2D<uint8_t> grid_map2;

  // 设定地图图片大小
  grid_map2.set_cell_number(size);
  // 设定图片起点
  grid_map2.set_origin(std::array<double, 2>{0.0, 0.0});
  // 设定分辨率
  grid_map2.set_resolution(std::array<double, 2>{resolution, resolution});

  for(int x=0;x<x_size;x++)
  {
    for(int y=0;y<y_size;y++)
    {
      // 对地图取反赋值，在opencv中白色是255 黑色是0，但是rviz中习惯0是白色可通过，黑色是不可通过
      if(occupancy_map[x+y*x_size] == 100)
      {
        grid_map2.SetValue(x,y,0);
      }
      else
      {
        grid_map2.SetValue(x,y,1);
      }
    }
  }

  // 构建带符号人工距离场
  SignedDistanceField2D sdf2(std::move(grid_map2));
  auto t0 = chrono::high_resolution_clock::now();

  // 更新人工距离场
  sdf2.UpdateSDF();

  auto t1 = chrono::high_resolution_clock::now();
  double total_ms =
      chrono::duration_cast<chrono::microseconds>(t1 - t0).count() / 1000.0;

  // cout << "time for sdf: " << total_ms << " ms" << endl;

  cv::Mat temp;
  temp = sdf2.esdf().ImageSec();

  // 获取势场的极大值极小值
  Vector2d min_max = sdf2.esdf().getESDFMax_and_Min();
  // 求取分辨率
  double scale = 100.0/abs(min_max[1]-min_max[0]);

  vector<double> local_esdf_buffer;
  // 获取人工势场地图
  // 重构esdf的大小
  local_esdf_buffer.resize(x_size*y_size);
  // 获取地图
  local_esdf_buffer = sdf2.esdf().map_data_;

  // 定义rviz可以显示发布的人工势场地图
  esdfmap.resize(x_size*y_size);
  // 填充esdf地图
  fill(esdfmap.begin(),esdfmap.end(),-1);

  // 赋值
  for(int x=0;x<x_size;x++)
  {
    for(int y=0;y<y_size;y++)
    {
      if(local_esdf_buffer[x+y*x_size] > 0)
      {
        // rviz中障碍物是100
        esdfmap[x+y*x_size] = 100;
      }
      else
      {
        // esdfmap[x+y*x_size] = 100+(int)local_esdf_buffer[x+y*x_size];
        esdfmap[x+y*x_size] = int((local_esdf_buffer[x+y*x_size]-min_max[0])*scale);
      }
    }
  }  
}

// 从全局地图中提取局部数据
// globalSize 全局地图尺寸大小
// localSize 局部地图尺寸大小
// localmapStart 局部地图起点
// globalmap  全局地图数据
// obstacle 障碍物集合
void local_map_deal::Extract_Local_From_GlobalMap(Vector2i globalSize,Vector2i localSize,Vector2i localmapStart,
vector<int8_t> globalmap,vector<Vector2i> &obstacleLocal)
{
  obstacleLocal.clear();

  // localmap.resize(localSize(0)*localSize(1));
  // 按照局部地图位置和规格遍历当前位置的全局地图，获得当前的全局地图障碍物信息
  for(int x=0;x<localSize(0);x++)
  {
    for(int y=0;y<localSize(1);y++)
    {
      if((localmapStart(0)+x) >= 0 && 
         (localmapStart(1)+y) >=0 &&
          (localmapStart(0)+x) < globalSize(0) &&
          (localmapStart(1)+y) < globalSize(1))
        {
          if(globalmap[(localmapStart(0)+x)+(localmapStart(1)+y)*globalSize(0)] == 100)
          {
            obstacleLocal.push_back(Vector2i(x,y));
          }
          // localmap[x+y*localSize(0)] = globalmap[(localmapStart(0)+x)+(localmapStart(1)+y)*globalSize(0)];
        }
    }
  }
}

// 在ESDF地图的基础上获取代价地图，即忽略代价地图某一代价值以下的地图
// mapSize 地图尺寸大小
// 必须在获取代价地图之后使用
vector<int8_t> local_map_deal::Get_CostMap(Vector2i mapSize,vector<int8_t> esdf_map)
{
  // 获取局部地图尺寸
  int x_size = mapSize(0);
  int y_size = mapSize(1);

  vector<int8_t> map;

  map.resize(x_size*y_size);
  // 填充代价地图
  for (int x = 0; x < x_size; x++)
  {
    for(int y=0;y < y_size;y++)
    {
      //  70以下全按照70处理
      if(esdf_map[x+y*x_size] >= 0 && esdf_map[x+y*x_size] <= costdownvalue)
      {
        map[x+y*x_size] = 0;
      }
      else if(esdf_map[x+y*x_size] > costdownvalue && esdf_map[x+y*x_size] <= costupvalue)
      {
        map[x+y*x_size] = (int)(costdownk*esdf_map[x+y*x_size]);
      }
      else if(esdf_map[x+y*x_size] > costupvalue && esdf_map[x+y*x_size] < 100)
      {
        map[x+y*x_size] = (int)(costupk*esdf_map[x+y*x_size]);
      }
      else
      {
        map[x+y*x_size] = (int)esdf_map[x+y*x_size];
      }
    }
  }
  
  return map;
}

}

