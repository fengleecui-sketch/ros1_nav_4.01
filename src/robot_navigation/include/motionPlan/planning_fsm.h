#ifndef _PLANNING_FSM_H_
#define _PLANNING_FSM_H_

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <vector>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Empty.h>

#include "map_deal/edt_environment.h"
#include "map_deal/global_map_deal.h"

#include "path_searcher/Hybrid_astar.h"
#include "path_optimization/bspline_optimizer.h"
#include "path_optimization/non_uniform_bspline.h"
#include "motionPlan/dyn_planner_manager.h"

#include "robot_communication/Bspline.h"

using std::vector;

namespace dyn_planner
{
class PlanningFSM
{
private:
  /* ---------- flag ---------- */
  bool trigger_, have_goal_; // 默认值为true
  enum EXEC_STATE
  {
    INIT,
    WAIT_GOAL,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ
  };
  EXEC_STATE exec_state_;

  enum FLIGHT_TYPE
  {
    MANUAL_GOAL = 1,
    PRESET_GOAL = 2
  };

  void changeExecState(EXEC_STATE new_state, string pos_call);
  void printExecState();

  /* ---------- planning utils ---------- */
  // SDFMap::Ptr sdf_map_;

  EDTEnvironment::Ptr edt_env_;
  global_map_deal::Ptr global_map_;

  Hybrid_astar::Ptr path_finder_;
  BsplineOptimizer::Ptr bspline_optimizer_;

  DynPlannerManager::Ptr planner_manager_;

  /* ---------- parameter ---------- */
  int flight_type_;  // 1 mannual select, 2 hard code
  double thresh_no_replan_, thresh_replan_;
  double waypoints_[10][2];
  int wp_num_;

  /* ---------- planning api ---------- */
  Eigen::Vector2d start_pt_, start_vel_, start_acc_, end_pt_, end_vel_;
  int current_wp_;

  bool planSearchOpt();  // front-end and back-end method

  /* ---------- sub and pub ---------- */
  ros::NodeHandle node_;

  ros::Timer exec_timer_, safety_timer_;
  ros::Timer vis_timer_, query_timer_;

  ros::Subscriber waypoint_sub_;

  ros::Publisher replan_pub_, bspline_pub_;

  ros::Publisher oriPathPub;        //导航原始路径
  ros::Publisher optPathPub;        //优化路径
  ros::Publisher optprePathPub;     //优化路径
  ros::Publisher controlPathPub;    //控制路径发布
  void PublishPath(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> path);
  
  void execFSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);

  // 话题接收
  ros::Subscriber staticMap;    //订阅静态膨胀地图
  void staticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map); //局部地图订阅回调函数
  ros::Subscriber clickSub;     //订阅鼠标点击信息(单位是m)
  void clickCallback(const geometry_msgs::PoseStampedConstPtr &msg); //鼠标点击回调函数

  std::vector<int> mapData;   //用二维数组存储数据
  /***
   * @description: 将ros解析的地图转变为正常的先行后列的顺序，非常非常非常重要
   * @param 1 读取出来的地图的参数  一维数组
   * @param 2 转换为一维算法能用的普通一维数组
   * @param 3 地图的x方向尺寸
   * @param 4 地图的y方向尺寸
   * @return {*}
   */
  void NavtfGrid(const nav_msgs::OccupancyGrid::ConstPtr &data, std::vector<int> &map, int x_size, int y_size)
  {
    map.resize(x_size * y_size);
    for (int i = 0; i < x_size; i++)
    {
      for (int j = 0; j < y_size; j++)
      { /* 这行代码非常非常非常重要，将ros解析的地图转变为正常的先行后列的顺序 */
        map[i * y_size + j] = int(data->data[j * x_size + i]);
        // cout<<map[i*y_size+j];
      }
      // cout<<endl;
    }
  }
public:
  PlanningFSM(/* args */)
  {
  }
  ~PlanningFSM()
  {
  }

  void init(ros::NodeHandle& nh);
};

}  // namespace dyn_planner

#endif