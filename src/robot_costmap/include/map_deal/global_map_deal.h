/*
 * @Author: your name
 * @Date: 2023-08-11 13:17:38
 * @LastEditTime: 2023-09-20 17:23:34
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_costmap/include/map_deal/global_map_deal.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __GLOBAL_MAP_DEAL_H
#define __GLOBAL_MAP_DEAL_H
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Eigen>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "map_deal/grid_map_2d.h"
#include "map_deal/signed_distance_field_2d.h"

using namespace std;
using namespace Eigen;

using std::shared_ptr;
using std::unique_ptr;
using std::vector;

// 定义地图处理相关数组
typedef struct map_array
{
	int8_t status;		//当前状态，0未处理，1是已经处理,-1不需要处理
	int8_t num;		//赋值
	uint16_t radius;	//半径
	bool     infla_flag;	//膨胀标志位

  map_array(){};
  ~map_array(){};
}map_array_define;

namespace map_deal
{
class global_map_deal
{
private:

  int actual_map_x,actual_map_y;
  // // 有效地图的起点 x y
  int actual_map_startx,actual_map_starty;
  // 人工势场宽度 太宽了会影响车路径的规划
  int esdfwidth;    // 人工势场宽度

  bool has_been_inflate = false;  //是否被膨胀
  bool has_been_esdf = false;   //是否是人工势场地图

  bool have_odom_;  //是否接收到里程计
  bool map_valid_;  //地图是否有效

  ros::Publisher global_inflate_map_publiser;      //  全局地图发布
  ros::Publisher global_esdf_map_publisher; //全局距离场地图发布

  ros::Subscriber map_subscriber;  // 全局地图地图订阅
  void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map);
  ros::Subscriber localizationSub;        // 订阅当前车的姿态消息
  void localizationCallback(const nav_msgs::OdometryConstPtr &msg); //定位信息回调函数

  // 定义膨胀地图层 该地图只保留了地图的有效部分
  std::vector<int8_t> occupancy_buffer_; // 0 is free, 1 is occupied
  // 膨胀地图层
  std::vector<int8_t> inflate_map_; // 0 is free, 1 is occupied
  // 定义整数esdf地图
  std::vector<int8_t> esdf_map_buffer_;
  // 定义代价地图
  std::vector<int8_t> cost_map;   //从esdf地图的基础上获取代价地图

  // 定义
  nav_msgs::Odometry odom_;

  // 定义全局膨胀地图
  nav_msgs::OccupancyGrid globalinflatemap;
  // 定义全局esdf地图
  nav_msgs::OccupancyGrid globalesdfmap;
public:
  /* data */
  // 地图分辨率
  double resolution;
  // 分辨率的倒数
  double resolution_inv;
  // 膨胀半径
  double inflate;
  // 地图的原点
	double origin_x;
	double origin_y;
  // 地图的终点
  double mapover_x;
  double mapover_y;
  // 地图的x轴方向的栅格地图尺寸
  int grid_map_x;
  int grid_map_y;
  // costmap 分阶阈值设置
  int costupvalue;
  int costdownvalue;
  // 系数设定
  double costupk;
  double costdownk;

  std::vector<double> esdf_buffer_;

  // 定义势场地图
  std::vector<int32_t> esdf_map_test;

  /**
   * @brief 栅格地图坐标系转世界坐标系
   * @param mx   地图坐标x
   * @param my   地图坐标y
   * @param wx   世界坐标x
   * @param wy   世界坐标y
   * @return
   * @attention
   * @todo
   * */
  Vector2d mapToWorld(Vector2i mapt) const
  {
    double wx,wy;
    wx = origin_x + (mapt[0] + 0.5) * resolution;
    wy = origin_y + (mapt[1] + 0.5) * resolution;

    return Vector2d(wx,wy);
  }

  /**
   * @brief 世界坐标系转栅格地图坐标系
   * @param wx   世界坐标x
   * @param wy   世界坐标y
   * @param mx   地图坐标x
   * @param my   地图坐标y
   * @return
   * @attention
   * @todo
   * */
  Vector2i worldToMap(Vector2d worldpt) const
  {
    int mx,my;

    mx = (int)(1.0 * (worldpt[0] - origin_x) / resolution);
    my = (int)(1.0 * (worldpt[1] - origin_y) / resolution);

    return Vector2i(mx,my);
  }

  // 判断是否在地图中
  bool isInMap(Eigen::Vector2d worldpt)
  {
    if (worldpt(0) < origin_x || worldpt(1) < origin_y)
    {
      // cout << "less than min range!" << endl;
      // cout << "world point: "<<worldpt(0)<<"  "<<worldpt(1)<<endl;
      return false;
    }

    if (worldpt(0) > mapover_x || worldpt(1) > mapover_y)
    {
      // cout << "larger than max range!" << endl;
      // cout << "world point: "<<worldpt(0)<<"  "<<worldpt(1)<<endl;
      return false;
    }

    return true;
  }

public:
  typedef shared_ptr<global_map_deal> Ptr;

  vector<double> global_esdf_buffer_;
  // 有效地图的起点 x y
  // 获取ESDF地图是可以从rviz中显示的地图
  // actualmapSize真实地图尺寸
  // occupancy_map膨胀地图
  // esdfmap最终的人工势场地图可以用rviz显示
  void Get_ESDFMap(Vector2i mapSize,Vector2i actualmapSize,Vector2i actualmapStart,
  vector<int8_t> occupancy_map,vector<int8_t> &esdfmap);

  // 全局地图膨胀
  // mapSize地图尺寸参数
  // actualmapSize 实际地图的尺寸
  // actualmapStart 实际地图的起点和终点
  // oldmap原本的地图参数
  // inflatemap膨胀地图
  void Infalte_Global_Map(Vector2i mapSize,Vector2i *actualmapSize,
  Vector2i *actualmapStart,vector<int8_t> oldmap,vector<int8_t> &inflatemap);

  // 获取实际地图并把地图参数转换为0 1方便后续生成人工势场
  // 必须在Infalte_Global_Map之后使用
  // mapSize原来地图尺寸
  // actualmapSize真实地图尺寸
  // actualStart真实地图的起点
  // oldmap原本的地图参数
  // actualmap真正的地图
  void Get_ActualMap(Vector2i mapSize,Vector2i actualmapSize,Vector2i actualStart,vector<int8_t> oldmap,vector<int8_t> &actualmap);

  // 设定全局地图处理需要的参数
  // resolution_ 分辨率
  // inflate_ 膨胀半径
  // origin_x_ 起点x
  // origin_y_ 起点y
  void Set_Maparams(double resolution_,double inflate_,double origin_x_,double origin_y_,double map_x_size,double map_y_size);

  // 全局代价地图
  // mapSize地图尺寸参数
  // inflatemap膨胀地图
  // costmap代价地图
  void Set_Cost_Map(Vector2i mapSize,vector<int8_t> inflatemap,vector<int8_t> &costmap);

  // 在ESDF地图的基础上获取代价地图，即忽略代价地图某一代价值以下的地图
  // 必须在获取代价地图之后使用
  vector<int8_t> Get_CostMap(vector<int8_t> esdf_map);

  // 
  void getInterpolationData(const Eigen::Vector2d &pos, vector<Eigen::Vector2d> &pos_vec, Eigen::Vector2d &diff);

  // 全局地图膨胀参数初始化
  void init(ros::NodeHandle &nh);

  // 代价地图参数设置
  void Set_CostMaparams(ros::NodeHandle &nh);

  // mapdata 地图数据
  // inflatemap 膨胀地图    
  void Infalte_Map(vector<int8_t> mapdata,vector<int8_t> &inflatemap);
  // occupancy_map膨胀地图
  // esdfmap 人工势场地图
  void ESDF_Map(vector<int8_t> occupancy_map,vector<int32_t> &esdfmap);

  // 获得当前点的距离
  double getDistance(Eigen::Vector2d pos);
  double getDistance(Eigen::Vector2i id);

  // 获取odom状态
  bool odomValid() { return have_odom_; }
  // 获取map状态
  bool mapValid() { return map_valid_; }
  // 返回当前里程计消息
  nav_msgs::Odometry getOdom() { return odom_; }

  global_map_deal(/* args */);
  ~global_map_deal();
};



}


#endif    //
