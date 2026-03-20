/*
 * @Author: your name
 * @Date: 2023-08-30 20:33:55
 * @LastEditTime: 2023-09-21 17:24:51
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_costmap/include/map_deal/local_map_deal.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __LOCAL_MAP_DEAL_H
#define __LOCAL_MAP_DEAL_H

#include "map_deal/grid_map_2d.h"
#include "map_deal/signed_distance_field_2d.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>

#include <Eigen/Eigen>
#include <iostream>

using namespace std;
using namespace Eigen;

using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace map_deal
{

class local_map_deal
{
private:
  /* data */
  // 地图分辨率
  double resolution;
  // 分辨率的倒数
  double resolution_inv;
  // 膨胀半径
  double inflate;

  bool is_use_sim;    //是否进行仿真

public:
  typedef shared_ptr<local_map_deal> Ptr;

  // 激光雷达的扫描范围
  double laser_min_theta;
  double laser_max_theta;

  // costmap 分阶阈值设置
  int costupvalue;
  int costdownvalue;
  // 系数设定
  double costupk;
  double costdownk;

  // 局部地图转全局地图
  // localPosition 局部地图的位置
  // mapSize 地图尺寸参数
  // robotPose 机器人位置姿态
  // pclocal 局部点云数据
  // obs 获得障碍物集合
  // diffxy 获得xy方向的偏置是定值
  vector<int8_t> Local_Tran_Global_Map(Vector2d localPosition,Vector2i mapSize,
  Vector3d robotPose,pcl::PointCloud<pcl::PointXYZ> pclocal,vector<Vector2i> &obs,Vector2d diffxy = Vector2d(0.00,0.00));

  // 局部地图膨胀
  // mapSize地图尺寸参数
  // obstacle获取障碍物集合
  // oldmap原本的地图参数
  // inflatemap膨胀地图
  void Infalte_Local_Map(Vector2i mapSize,vector<Vector2i> obstacle,vector<int8_t> &inflatemap);

  // 局部代价地图
  // mapSize地图尺寸参数
  // inflatemap膨胀地图
  // costmap代价地图
  void Set_Cost_Map(Vector2i mapSize,vector<int8_t> inflatemap,vector<int8_t> &costmap);

  // 获取局部ESDF地图
  // mapSize 地图尺寸大小
  // occupancy_map 需要进行esdf的地图
  // esdfmap 人工势场地图
  void Get_ESDF_Local_Map(Vector2i mapSize,vector<int8_t> occupancy_map,vector<int8_t> &esdfmap);

  // 从全局地图中提取局部数据
  // globalSize 全局地图尺寸大小
  // localmapStart 局部地图起点
  // globalmap  全局地图数据
  // obstacle 障碍物集合
  void Extract_Local_From_GlobalMap(Vector2i globalSize,Vector2i localSize,Vector2i localmapStart,vector<int8_t> globalmap,
  vector<Vector2i> &obstacleLocal);
  
  // 设定全局地图处理需要的参数
  void Set_Maparams(double resolution_,double inflate_,bool _use_sim = true);

  // 设定代价地图参数
  void Set_CostMaparams(ros::NodeHandle &nh);

  // 在ESDF地图的基础上获取代价地图，即忽略代价地图某一代价值以下的地图
  // 必须在获取代价地图之后使用 
  vector<int8_t> Get_CostMap(Vector2i mapSize,vector<int8_t> esdf_map);

  vector<int32_t> localesdfmap;

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

  local_map_deal(/* args */);
  ~local_map_deal();
};

}



#endif //

