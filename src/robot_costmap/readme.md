# 该功能包主要是对全局地图和局部地图数据进行处理
## launch文件功能介绍
map_acl.launch 用来启动实际场景下的地图
map_deal.launch 地图处理节点启动，全局地图膨胀 全局地图欧几里德距离场 全局地图代价 局部地图膨胀 局部地图欧几里德距离场 局部地图代价
## 什么是欧几里德距离场
1. 欧几里德距离场分为两种：距离场(斥力场)、目标场(引力场)
2. 距离场：顾名思义，假设当前有一张地图(不管二维还是三维)，经过栅格化之后，计算每一个格子到障碍物格子的距离的累加，距离障碍物越远，则距离的累加数值越大，距离障碍物越近，则距离的累加数值越小。当然只计算距离的累加数值只是一部分，通过数学语言进行介绍如下[势场地图的讲解](https://blog.csdn.net/qq_44339029/article/details/128510395)：
与文章内容不同的是，如果我们使用的只是距离场或者说斥力场地图，这里的设机器人位置为(x,y)，我们可以看作是设空白栅格位置为(x,y)，计算步骤和计算过程同文中一样，从实际应用场景进行考虑，文中带入的是机器人的位置计算势场然后是一个动态的过程，而我们地图处理中是一个动态的过程，势场地图在这里主要作用是用作安全约束，且是软约束。文中所说的最终求得梯度是一个矢量值，其可以用来指引机器人前进的方向，而如果静态使用的时候，因为我们算的是每一个栅格到障碍物的距离的累加数值，我们是在一个势场地图中进行规划，所以我们可以直接将梯度矢量标量化，代表该格子综合考虑情况下距离障碍物远近的数值，这就是欧几里德距离场地图
3. 目标场和距离场考虑思路是相同的，只不过我们目前(2023年11月05日)只是使用了距离场并没有使用引力场或者说目标场，后续可以使用，目标场的使用方法有两种，一种是按照刚才说的距离场的静态的方法使用，即计算每一个空白栅格到目标点的距离值，然后按照之前提到的方法进行计算；或者说动态使用，动态使用的时候，那么此时欧几里德距离场就可以看作是一种局部规划算法
4. **重点：我们目前将势场地图应用到局部搜索中，即在搜索算法的启发式函数中考虑当前势场数值去进行局部规划，虽然使用的是距离场，但是启发式函数中会计算当前点到终点的距离作为权重，确定下一步应该走哪，结合目标场的动态规划思想，我们现在用的就是一种基于欧几里德距离场(斥力场)地图的局部规划算法，优点就是使用启发式函数的启发式搜索特性充当了引力场的作用**
5. 目前欧几里德距离场的实现是直接从gitee[势场代码](https://gitee.com/kin_zhang/sdf_test)上找来的开源代码，因为之前对欧几里德距离场的理解只是停留在具象化的层面上，并不能以数学的方式进行理解，所以后续可能会对代码进行优化，这里也可以看看原代码作者对于欧几里德距离场的[讲解](https://blog.csdn.net/qq_39537898/article/details/124964419)，这里目前有一种简单欧几里德距离场的实现方式，没有求梯度，只是对距离取的倒数：
```c++
// 膨胀之后的障碍物集合
vector<Vector2i> obstacleInflate;
// 计算到障碍物的距离场
double calculateRepulsiveForce(const int x,const int y)
{
  double repulsiveForce = 0.0;

  // for (const auto& obstacle : obstacleInflate)
  for(int i=0;i<obstacleInflate.size();i++) 
  {      
    double distance = std::sqrt(std::pow(x - obstacleInflate[i][0], 2) + std::pow(y - obstacleInflate[i][1], 2))*resolution;
    if (distance > 0.001) {  // Avoid division by zero
      repulsiveForce += 1.0 / distance;
    }
  }

  if(repulsiveForce >= 100000)
  {
    repulsiveForce = 100000;
  }

  return repulsiveForce;
}
```
```c++
// ChatGPT给出了一种实现的方法如下所是：
#include <iostream>
#include <vector>
#include <cmath>

struct Point {
  double x;
  double y;
};

class ArtificialPotentialField {
public:
  ArtificialPotentialField(int width, int height) : width_(width), height_(height) {
      // Initialize the grid
      grid_.resize(width_, std::vector<double>(height_, 0.0));

      // Set goal and obstacle positions
      setGoal(Point{5.0, 5.0});
      addObstacle(Point{3.0, 3.0});
      addObstacle(Point{1.0, 1.0});
      addObstacle(Point{2.0, 2.0});
      addObstacle(Point{4.0, 4.0});
  }

  void setGoal(const Point& goal) {
      goal_ = goal;
  }

  void addObstacle(const Point& obstacle) {
      obstacles_.push_back(obstacle);
  }

  void calculatePotentialField() {
      for (int x = 0; x < width_; ++x) {
          for (int y = 0; y < height_; ++y) {
              grid_[x][y] = calculateRepulsiveForce(Point{static_cast<double>(x), static_cast<double>(y)});
              // grid_[x][y] += calculateAttractiveForce(Point{static_cast<double>(x), static_cast<double>(y)});
          }
      }
  }

  void printPotentialField() 
  {
    double min = 0,max = 0;

    for (int y = height_ - 1; y >= 0; --y) {
      for (int x = 0; x < width_; ++x) {
        std::cout << grid_[x][y] << " ";
        if(min >= grid_[x][y])
        {
          min = grid_[x][y];
        }
        if(max < grid_[x][y])
        {
          max = grid_[x][y];
        }
      }
      std::cout << std::endl;
    }

    std::cout << std::endl;

    double scale = 10/(max-min);
    for (int y = height_ - 1; y >= 0; --y) {
      for (int x = 0; x < width_; ++x) {
        std::cout << (grid_[x][y]-min)*scale << " ";
      }
      std::cout << std::endl;
    }
  }

private:
  int width_;
  int height_;
  std::vector<std::vector<double>> grid_;
  Point goal_;
  std::vector<Point> obstacles_;

  double calculateRepulsiveForce(const Point& point) {
      double repulsiveForce = 0.0;
      for (const auto& obstacle : obstacles_) {
          double distance = std::sqrt(std::pow(point.x - obstacle.x, 2) + std::pow(point.y - obstacle.y, 2));
          if (distance > 0.01) {  // Avoid division by zero
              repulsiveForce += 1.0 / distance;
          }
      }
      return repulsiveForce;
  }

  double calculateAttractiveForce(const Point& point) {
      double attractiveForce = std::sqrt(std::pow(point.x - goal_.x, 2) + std::pow(point.y - goal_.y, 2));
      return attractiveForce;
  }
};

int main(void) {
  // Define grid dimensions
  const int width = 10;
  const int height = 10;

  ArtificialPotentialField field(width, height);
  field.calculatePotentialField();
  field.printPotentialField();

  return 0;
}
```

## 地图接口适用于ROS Gmapping建图和Cartographer建图生成的.pgm地图文件
1. 使用ROS自带的map_server发布相应的地图数据就行
```c++
// 每次更换地图的时候一定要更改相应的yaml文件
  <node pkg="map_server" type="map_server" name="map_server" args="$(find robot_costmap)/maps/ICRA.yaml"/>
```
2. map_deal.cpp中的代码分为两部分，一个是对应仿真Gazebo环境中的地图数据，另一个是对应实际应用的环境数据，这里简单说一下，**重点：仿真和实际不一样主要是因为仿真环境中大部分地图是没有用的，所以通过简单的算法把有效的地图提取出来进行欧几里德距离场化，实际建图出来就是有效的地图，不需要进行有效数据提取，这里唯一需要弄清楚的问题就是mapserver发布的地图格式是一个一维容器或者说数组，知道这个容器对应二维地图的数据顺序即可**：
```c++
// 地图扫描顺序，这个是用来提取地图中的障碍物数据
// 从栅格地图中获取障碍物点的信息
for(int x = 0;x<grid_map_x;x++)
{
  for(int y = 0;y<grid_map_y;y++)
  {
    // 当其中为障碍物的时候
    if(mapdata[x+grid_map_x*y] == 100)
    {
      obs.push_back(Vector2i(x,y));
    }
  }
}
```
## 全局地图处理：地图膨胀、地图势场化、代价地图
1. 膨胀地图就是按照机器人最大半径，因为目前用的自动步兵，按照对角线长度的一半设定地图膨胀层，详细的实现可以看函数：
**Infalte_Map、Infalte_Global_Map**，前者对应的是实际地图的膨胀、后者对应的是仿真环境中地图的膨胀
```c++
// 顺序就是：
// 1.地图获取
// 2.地图膨胀
// 3.势场化
if(!is_use_sim)
{
  // 膨胀全局地图
  global_map_->Infalte_Map(map->data,globalinflatemap.data);
  global_inflate_map_publiser.publish(globalinflatemap);

  // 准备发布势场地图
  globalesdfmap = globalinflatemap;
  global_map_->ESDF_Map(globalinflatemap.data,esdf_map_buffer_);
  globalesdfmap.data = esdf_map_buffer_;
  global_esdf_map_publisher.publish(globalesdfmap);

  edt_environment->setMap(global_map_);
  edt_environment->init();

  // 对代价地图赋值
  globalcostmap = globalesdfmap;
  globalcostmap.data = global_map_->Get_CostMap(esdf_map_buffer_);
  // 发布代价地图
  global_cost_map_publisher.publish(globalcostmap);
}
// 仿真环境下地图尺寸是gazebo仿真环境可预见的地图尺寸
else
{
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
  global_map_->Get_ActualMap(Vector2i(global_map_x,global_map_y),
                Vector2i(actual_map_x,actual_map_y),
                Vector2i(actual_map_startx,actual_map_starty),
                globalinflatemap.data,
                occupancy_buffer_);
  
  // 获取全局ESDF地图
  global_map_->Get_ESDFMap(Vector2i(global_map_x,global_map_y),
              Vector2i(actual_map_x,actual_map_y),
              Vector2i(actual_map_startx,actual_map_starty),
              occupancy_buffer_,
              esdf_map_buffer_);

  globalesdfmap = globalinflatemap;
  // 对esdf地图赋值
  globalesdfmap.data = esdf_map_buffer_;

  edt_environment->setMap(global_map_);
  edt_environment->init();

  // cout<<"esdf_map_buffer_size  "<<esdf_map_buffer_.size()<<endl;
  // 发布势场地图
  global_esdf_map_publisher.publish(globalesdfmap);

  // 对代价地图赋值
  globalcostmap = globalesdfmap;
  globalcostmap.data = global_map_->Get_CostMap(esdf_map_buffer_);
  // 发布代价地图
  global_cost_map_publisher.publish(globalcostmap);
}
```
2. 地图势场化，这里用到了github上的一个开源代码，但是是独立于ROS的基于opencv图像处理的二维欧几里德距离场化，详细的实现过程可见"signed_distance_field_2d.cpp"：
```c++
// SDF构造函数 输入参数 origin 起点
// dim 维度
// map_resolution 是地图的分辨率
SignedDistanceField2D::SignedDistanceField2D(std::array<double, 2> origin,
                                             std::array<int, 2> dim,
                                             const double map_resolution)
    : map_resolution_(map_resolution) {
  // 设定地图的起点
  occupancy_map_.set_origin(origin);
  // 设定地图的维度，这里的occupancy_map_是普通的栅格地图
  occupancy_map_.set_cell_number(dim);
  // 设定分辨率
  occupancy_map_.set_resolution(
      std::array<double, 2>{map_resolution, map_resolution});
  esdf_.ResizeFrom(occupancy_map_);
}

SignedDistanceField2D::SignedDistanceField2D(OccupancyMap&& occupancy_map)
    : occupancy_map_(std::move(occupancy_map)) {
  esdf_.ResizeFrom(occupancy_map_);
  map_resolution_ = occupancy_map_.resolution()[0];
}

// 欧氏距离欧几里德距离场转换
// 维度
// 判断是否被占据
// 输出的势场地图
void SignedDistanceField2D::EuclideanDistanceTransform(
    std::array<int, 2> dim,
    std::function<bool(const int x, const int y)> is_occupied,
    DistanceMap* output_map) {
  int inf = dim[0] + dim[1] + 10;

  std::vector<std::vector<int>> g(dim[0], std::vector<int>(dim[1], 0));
//   omp_set_num_threads(4);
//   {
// #pragma omp parallel for
    // column scan
    for (int x = 0; x < dim[0]; ++x) {
      g[x][0] = is_occupied(x, 0) ? 0 : inf;

      for (int y = 1; y < dim[1]; ++y) {
        g[x][y] = is_occupied(x, y) ? 0 : 1 + g[x][y - 1];
      }

      for (int y = dim[1] - 2; y >= 0; --y) {
        if (g[x][y + 1] < g[x][y]) g[x][y] = 1 + g[x][y + 1];
      }
    }
  // }

  // row scan
//   omp_set_num_threads(4);
//   {
// #pragma omp parallel for
    for (int y = 0; y < dim[1]; ++y) {
      int q = 0, w;
      std::vector<int> s(dim[0], 0);
      std::vector<int> t(dim[0], 0);

      auto f = [&g, &y](int x, int i) -> double {
        return (x - i) * (x - i) + g[i][y] * g[i][y];
      };

      for (int u = 1; u < dim[0]; ++u) {
        while (q >= 0 && f(t[q], s[q]) > f(t[q], u)) {
          --q;
        }

        if (q < 0) {
          q = 0;
          s[0] = u;
        } else {
          w = 1 + std::floor((u * u - s[q] * s[q] + g[u][y] * g[u][y] -
                              g[s[q]][y] * g[s[q]][y]) /
                             (2 * (u - s[q])));
          if (w < dim[0]) {
            ++q;
            s[q] = u;
            t[q] = w;
          }
        }
      }

      for (int u = dim[0] - 1; u >= 0; --u) {
        output_map->SetValue(u, y, map_resolution_ * std::sqrt(f(u, s[q])));
        if (u == t[q]) --q;
      }
    }
  // }
}

// 更新距离场
void SignedDistanceField2D::UpdateSDF() {
  DistanceMap distance_map, inv_distance_map;
  distance_map.ResizeFrom(occupancy_map_);
  inv_distance_map.ResizeFrom(occupancy_map_);
  OccupancyMap& map = occupancy_map_;

  auto dim = occupancy_map_.cell_num();
  EuclideanDistanceTransform(
      dim,
      [&map](const int x, const int y) -> bool { return map.IsOccupied(x, y); },
      &distance_map);
  EuclideanDistanceTransform(
      dim,
      [&map](const int x, const int y) -> bool {
        return !map.IsOccupied(x, y);
      },
      &inv_distance_map);

  const auto& dis_map_data = distance_map.data();
  const auto& inv_dis_map_data = inv_distance_map.data();
  auto& esdf_data = *esdf_.mutable_data();

//   omp_set_num_threads(4);
//   {
// #pragma omp parallel for
    for (int x = 0; x < dim[0]; ++x) {
      for (int y = 0; y < dim[1]; ++y) {
        int address = occupancy_map_.Index2Address(x, y);
        esdf_data[address] = dis_map_data[address];
        // 这里可以*系数k调整势场强度
        // esdf_data[address] = dis_map_data[address]*1.5f;
        if (inv_dis_map_data[address] > 0) {
          esdf_data[address] += (-inv_dis_map_data[address] + map_resolution_);
        }
      }
    }
  // }
}
```
3. 代价地图主要是在势场地图的基础上对势场地图进行了简单处理：
```c++
// 以下部分代码后续将会以launch文件传参的方式进行设定
vector<int8_t> map;

map.resize(grid_map_x*grid_map_y);
// 填充代价地图
for (int x = 0; x < grid_map_x; x++)
{
  for(int y=0;y < grid_map_y;y++)
  {
    //  70以下全按照70处理
    if(esdf_map[x+y*grid_map_x] >= 0 && esdf_map[x+y*grid_map_x] <= 45)
    {
      map[x+y*grid_map_x] = 0;
    }
    else if(esdf_map[x+y*grid_map_x] > 45 && esdf_map[x+y*grid_map_x] <= 54)
    {
      map[x+y*grid_map_x] = 0;
    }
    else if(esdf_map[x+y*grid_map_x] > 54 && esdf_map[x+y*grid_map_x] < 100)
    {
      map[x+y*grid_map_x] = (int)(1.5*esdf_map[x+y*grid_map_x]);
    }
    else
    {
      map[x+y*grid_map_x] = (int)esdf_map[x+y*grid_map_x];
    }
  }
}
```
## 局部地图处理：根据雷达点云获得局部地图数据、从全局地图中提取当前车位置一定范围内的局部数据、地图膨胀、地图势场化
1. 点云数据回调函数。
```c++
// 订阅point cloud2点云消息用来发布局部地图 这是仿真环境的点云地图，需要对点云进行坐标变换
pointCloud2Sub = node_handle_.subscribe("/cloud_map",10,&deal_all_map::pointCloud2Callback,this);

// scan_matched_points2是Cartographer发布的全局点云数据，直接调用即可生成局部代价地图，不需要进行坐标变换
pointCloud2Sub = node_handle_.subscribe("/scan_matched_points2",10,&deal_all_map::pointCloud2Callback,this);
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
```
2. 仿真中的点云是机器人局部坐标系下的，需要简单处理把点云数据转换为全局数据，**这其中的diffxy是因为在实际场景中Cartographer重定位需要获取定位补偿，效果就是在实际情况下，启动全部节点之后，需要将对车原地旋转几次，从rviz中看到重定位成功后，停的时候一定要停正，之后等几s就可以看到局部地图了**：
```c++
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
      world_x = point.x - diffxy(0);
      world_y = point.y - diffxy(1);
    }

    int x = (world_x - localPosition[0])*20;
    int y = (world_y - localPosition[1])*20;

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
```
3. 从全局地图中提取局部地图数据：
```c++
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
```
4. 地图膨胀:
```c++
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
```
5. 势场部分和全局部分逻辑一样，这就不写了
## 还有一个用于给fast planner使用的edt_environment，主要用途是可以获取指定地图点位的障碍物势场数值，用于给fast planner配置地图环境，目前fast planner还没完全调试成功(2023-09-03)

# 对于欧几里德距离场地图的新认识
因为欧几里德距离场的变化是基于距离产生变化的，所以对于局部地图来说，随着机器人的移动，对点云的结算最终的势场是在改变的，最初对欧几里德距离场地图的认识是基于障碍物距离，是以障碍物为中心不会产生变化，但是实际情况不是这样，所以根据这个又重新构建了**局部代价地图**，逻辑很简单，可以理解成按照逐渐递减的膨胀层，代码在local_map_deal.cpp中：
```c++
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

}
```




