/*
 * @Author: your name
 * @Date: 2023-08-17 10:44:39
 * @LastEditTime: 2023-08-20 15:49:10
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/include/motionPlan/dyn_planner_manager.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef _KGB_TRAJECTORY_GENERATOR_H_
#define _KGB_TRAJECTORY_GENERATOR_H_

#include "iostream"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "path_searcher/Hybrid_astar.h"
#include "path_optimization/non_uniform_bspline.h"
#include "path_optimization/bspline_optimizer.h"

#include "map_deal/edt_environment.h"
#include "map_deal/global_map_deal.h"

using namespace Eigen;
using namespace std;

namespace dyn_planner
{
class DynPlannerManager
{
private:

  ros::Publisher optPathPub;    // 发布Astar路径
  
  /* algorithm */
  // shared_ptr<KinodynamicAstar> path_finder;

  EDTEnvironment::Ptr edt_env_;

  Hybrid_astar::Ptr path_finder_;
  BsplineOptimizer::Ptr bspline_optimizer_;

  double time_sample_;
  double max_vel_;

  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
  double time_adjust_ = 0.0;

  /* helper function */
  Eigen::Vector2d getFarPoint(const vector<Eigen::Vector2d>& path, Eigen::Vector2d x1, Eigen::Vector2d x2);

public:
  DynPlannerManager()
  {
  }
  ~DynPlannerManager();

  /* ---------- main API ---------- */
  /* generated traj */
  int traj_id_, dynamic_;
  double traj_duration_, t_start_, t_end_, margin_, time_start_;
  ros::Time time_traj_start_;
  Eigen::Vector2d pos_traj_start_;
  NonUniformBspline traj_pos_, traj_vel_, traj_acc_;

  // 原始路径
  vector<Vector2d> oriworldPath;
  // 优化路径
  vector<Vector2d> optworldPath;

  /* guided optimization */
  NonUniformBspline traj_init_;
  vector<vector<Eigen::Vector2d>> guide_paths_;
  vector<Eigen::Vector2d> guide_pts_;

  bool generateTrajectory(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d start_acc,
                          Eigen::Vector2d end_pt, Eigen::Vector2d end_vel);  // front-end && back-end

  bool orthoGradReplan(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d end_pt,
                       Eigen::Vector2d end_vel);  // gradient-based replan using orthogonal gradient

  bool guidedGradReplan(Eigen::Vector2d start_pt, Eigen::Vector2d start_vel, Eigen::Vector2d end_pt,
                        Eigen::Vector2d end_vel);  // gradient-based replan using guiding points

  void retrieveTrajectory();

  void setParam(ros::NodeHandle& nh);
  void setPathFinder(const Hybrid_astar::Ptr& finder);
  void setOptimizer(const BsplineOptimizer::Ptr& optimizer);
  void setEnvironment(const EDTEnvironment::Ptr& env);

  bool checkTrajCollision();

  /* ---------- evaluation ---------- */
  void getSolvingTime(double& ts, double& to, double& ta);
  void getCostCurve(vector<double>& cost, vector<double>& time)
  {
    bspline_optimizer_->getCostCurve(cost, time);
  }

  typedef shared_ptr<DynPlannerManager> Ptr;
};
}  // namespace dyn_planner

#endif