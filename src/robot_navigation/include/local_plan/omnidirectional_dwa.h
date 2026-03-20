/*
 * @Author: your name
 * @Date: 2023-05-15 18:18:37
 * @LastEditTime: 2024-04-22 10:41:37
 * @LastEditors: your name
 * @Description:
 * @FilePath: /MyFormProject/src/robot_navigation/include/local_plan/omnidirectional_dwa.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
//
// Created by chh3213 on 2022/11/26.
//omnidirectional_dwa
#ifndef __OMNIDIRECTIONAL_DWA_H
#define __OMNIDIRECTIONAL_DWA_H

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf2/convert.h"
#include <tf2/utils.h>

#include <Eigen/Dense>

#include "vel_transform/vel_transform.h"
#include "path_follow/pid_follow.h"
// 包含自定义消息包
#include "robot_communication/localizationInfoBroadcast.h"
#include "robot_communication/chassisControl.h"
#include "robot_communication/goal.h"

using namespace std;
using namespace Eigen;

class Omnidirectional_DWAPlanner
{
public:
  Omnidirectional_DWAPlanner(void);

  class State
  {
  public:
    // 状态方程，因为是全向移动模型，所以我们需要x方向上的速度和y方向上的速度
    State(double, double, double, double, double, double);

    // 机器人的当前位置
    double x; // robot position x
    double y; 
    // 机器人的偏航角读
    double yaw; // robot orientation yaw
    // 机器人速度
    // x方向上的速度
    double velocity_x; // robot linear velocity
    // y方向上的速度
    double velocity_y; // robot linear velocity
    // 机器人角速度
    double yawrate; // robot angular velocity
  private:
  };

  class Window
  {
  public:
    // 移动窗口
    Window(void);
    //
    Window(const double, const double,const double,const double,const double, const double);
    // 最小速度x
    double min_velocity_x;
    // 最大速度x
    double max_velocity_x;
    // 最小速度y
    double min_velocity_y;
    // 最大速度y
    double max_velocity_y;
    // 最小yaw轴角速度
    double min_yawrate;
    // 最大yaw轴角速度
    double max_yawrate;

  private:
  };

  class CoordinateTransformer {
    public:
      CoordinateTransformer(double x_robot, double y_robot, double theta_robot)
          : x_robot_(x_robot), y_robot_(y_robot), theta_robot_(theta_robot) {}

      void transformToRobotFrame(double vx_global, double vy_global, double &vx_local, double &vy_local) {
        // 旋转矩阵
        double rotation_matrix[2][2] = {{cos(theta_robot_), -sin(theta_robot_)},
                                        {sin(theta_robot_), cos(theta_robot_)}};

        // 将全局速度应用旋转
        vx_local = rotation_matrix[0][0] * vx_global + rotation_matrix[0][1] * vy_global;
        vy_local = rotation_matrix[1][0] * vx_global + rotation_matrix[1][1] * vy_global;
      }

    private:
      double x_robot_;
      double y_robot_;
      double theta_robot_;
  };

  // 算法实现
  void process(void);
  // DWA算法实现用于放置在其它节点中
  void local_planner(void);
  // 激光雷达数据回调函数
  void scan_callback(const sensor_msgs::LaserScanConstPtr &msg);
  // 局部目标回调函数
  void local_goal_callback(const robot_communication::goalConstPtr &msg);
  // 局部地图回调函数
  void local_map_callback(const nav_msgs::OccupancyGridConstPtr &msg);
  // 目标速度回调函数
  void target_velocity_callback(const geometry_msgs::TwistConstPtr &global);
  // 正常里程计消息回调函数
  void odom_callback(const robot_communication::localizationInfoBroadcastConstPtr &msg);
  // 用于接收终点信息
  void clickCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  // 路径回调函数
  void pathCallback(const nav_msgs::PathConstPtr &path);    

  // 计算动态窗口
  Window calc_dynamic_window(const geometry_msgs::Twist &cur_velocity);
  // 计算目标代价
  float calc_to_goal_cost(const std::vector<State> &traj, const Eigen::Vector3d &goal);
  // 计算速度代价
  float calc_speed_cost(const std::vector<State> &traj, const Vector3d target_velocity);
  // 判断当前点位置是否有障碍物
  bool isObstacle(Vector2d position,const vector<vector<float>> &obs_list);
  // 判断当前点位置是否有障碍物
  bool isObstacle(Vector2d position);
  // 计算到障碍物代价
  float calc_obstacle_cost(std::vector<State> &traj,const std::vector<std::vector<float>> &obs_list);
  // 获取当前点的势场数值
  int8_t getESDFvalue(Vector2i gridpt);
  // 获取当前点的势场数值
  int8_t getESDFvalue(Vector2d worldpt);
  // 计算到障碍物的距离数值
  float calc_obstacle_cost(vector<State> &traj,int model = 1);
  // 机器人运动模型 源代码中为差速模型，现在转换为全向移动模型
  void motion(State &state, const double vx,const double vy, const double yawrate);
  // 获取障碍物
  std::vector<std::vector<float>> raycast();
  // 获取地图
  void getMapdata(void);
  // 从激光雷达的扫描数据获取障碍物
  std::vector<std::vector<float>> scan_to_obs();

  // 计算两个点之间的长度欧氏距离
  double calPointLength(Vector2d vector1,Vector2d vector2);
  // 计算向量之间的夹角
  double calVectorAngle(Vector2d vector1,Vector2d vector2);
  // 计算向量的模长
  double calVectorModLen(Vector2d vector1);
  // 计算单位向量
  Vector2d calUnitvector(Vector2d unitv);

  // 用来求解当前位置点以一定半径相交的路径点
  // path 输入路径
  // radius 搜索半径
  // updatepath 路径是否更新
  // return 局部终点
  Vector2d caLocalGoalPosition(vector<Vector2d> path,Vector2d nowpoint,double radius);

  // 用来对速度进行限制,因为机器人是全向移动的,如果直接通过数值对x y速度限制会产生畸变
  // 所以需要先合成,再进行限制
  // vel_x 输入的x方向上的速度
  // vel_y 输入的y方向上的速度
  // limit_velocity 限制的速度
  void LIMIT_VECTOR_Velocity(double &vel_x,double &vel_y,double limit_velocity);
  
  // 获取局部终点的函数
  void GetLocalGoal(Vector2d &position,vector<Vector2d> path);

  // 获取机器人的速度
  // nowpoint  当前位置
  // endpoint  终点位置
  // localvelocity 局部速度
  void GetRobotVelocity(Vector3d nowpoint,Vector3d endpoint,Vector3d &localvelocity);
  void Deal_Super_Circle(double *setangle,double *feedangle);
  // 查找跟随轨迹上的点有没有障碍物
  // trajs 轨迹
  float Find_Traj_Obstacle(vector<State> trajs,Vector2d tempgoal);

  Vector2d mapToWorld(Vector2i mapt) const;
  Vector2i worldToMap(Vector2d worldpt) const;

  void visualize_trajectories(const std::vector<std::vector<State>> &, const double, const double, const double, const int, const ros::Publisher &);
  void visualize_trajectory(const std::vector<State> &, const double, const double, const double, const ros::Publisher &);
  std::vector<Omnidirectional_DWAPlanner::State> dwa_planning(
      Eigen::Vector3d goal,
      std::vector<std::vector<float>> obs_list = {{0.0,0.0},{0.0,0.0}});

  void visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes,
  float a_set,float r_set,float g_set,float b_set,float length);
  ros::Publisher chassCtlPub;       //底盘控制
  ros::Publisher localgoalPub;      //局部终点查看
  geometry_msgs::Twist my_twist;    //速度控制节点
  geometry_msgs::PoseStamped global_goal;   //获取全局终点

  ros::Time end_point_time; //终点更新的时间记录
  ros::Time end_process_time; //找到终点的时间

protected:
  double HZ;
  // 是否使用ESDF地图
  bool IS_USE_ESDF;
  // 是否进行仿真
  bool IS_SIM;
  // 定义规划路径半径，用于在先验路径中找点
  double TEMP_GOAL_RADIUS;
  // 机器人base_link
  std::string ROBOT_FRAME;
  // 目标速度
  // 因为是x y方向上的速度
  Vector3d TARGET_VELOCITY;
  // X方向最大速度
  double MAX_VELOCITY_X;
  // X方向最小速度
  double MIN_VELOCITY_X;
  // Y方向最大速度
  double MAX_VELOCITY_Y;
  // Y方向最小速度
  double MIN_VELOCITY_Y;
  // 最大yaw角度范围
  double MAX_YAWRATE;
  // 最大加速度
  double MAX_ACCELERATION;
  // 最大yaw轴角加速度
  double MAX_D_YAWRATE;
  // 最大距离
  double MAX_DIST;
  // 速度分辨率
  double VELOCITY_RESOLUTION;
  // 角速度分辨率
  double YAWRATE_RESOLUTION;
  // 角度分辨率
  double ANGLE_RESOLUTION;
  // 预测时间
  double PREDICT_TIME;
  // 到目标点的代价系数
  double TO_GOAL_COST_GAIN;
  // 速度代价系数
  double SPEED_COST_GAIN;
  // 障碍物代价系数
  double OBSTACLE_COST_GAIN;
  // 积分时间
  double DT;
  // 判断到目标的范围
  double GOAL_THRESHOLD;
  // 判断转向系数TURN_DIRECTION_THRESHOLD
  double TURN_DIRECTION_THRESHOLD;

  // 当前的势场地图
	int8_t *mapdata;

  // 地图分辨率
  double resolution;
  // 地图分辨率的倒数
  double resolution_inv;
  // 地图的(0,0)的位置
  double origin_x;
  double origin_y;

  int grid_map_x;
  int grid_map_y;
  // 
  // 定义一个代标志位的路径
  vector<Vector2d> trajpath;
  // 定义当前位置点
  Vector3d nowposition;
  // 定义局部终点
  Vector3d localposition;
  // 全局路径
  vector<Vector2d> globalpath;

  // 最终终点
  Vector2d lastendPoint;
  // 最开始的起点
  Vector2d firststartPoint;

  // 定义访问路径用于查看是否访问过
  vector<pair<int,Vector2d>> visitPath;

  double anglePID[7] = {10.0,0,0,3.0,-3.0,0.05};
  // pid参数初始化
  pid_follow pidFollow;

  // 用于判断路径点数目
  int path_nodes_num;
  int last_path_nodes_num;

  // 用于记录算法执行次数
  int run_numbers;
  // 记录算法运行的总时间
  double run_times;
  // 获取最好的轨迹长度
  int best_traj_size;
  // 一小段距离
  double min_dis;
  double once_time;

  double obstacle_time;
  bool obstacle_flag;

  // 定义上一次的点
  Vector2d lastpoint;

  // 速度合成，局部转全局
  // 车体局部速度和全局速度
  Odom_data_define carVel;
  Odom_data_define worldVel;

  // 获得最佳路径
  std::vector<State> best_traj_to_use;
  robot_communication::chassisControl motionData;             //运动消息

  ros::NodeHandle nh;
  ros::NodeHandle local_nh;

  ros::Publisher velocity_pub;
  ros::Publisher candidate_trajectories_pub;
  ros::Publisher selected_trajectory_pub;

  ros::Publisher localPathPub;   // 局部路径发布

  ros::Subscriber local_map_sub;
  ros::Subscriber scan_sub;
  ros::Subscriber local_goal_sub;
  // 理想状态下里程计消息订阅
  ros::Subscriber global_velocity_sub;
  // 一般状态下里程计消息订阅
  ros::Subscriber odom_sub;
  ros::Subscriber target_velocity_sub;
  ros::Subscriber pathSub;
  ros::Subscriber clickSub;     //订阅鼠标点击信息(单位是m)

  // 使用tf变换求解速度
  tf2_ros::Buffer tf_buffer;

  // 定位信息
  robot_communication::localizationInfoBroadcast current_state;   //定位信息
  robot_communication::localizationInfoBroadcast target_state;   //定位信息

  robot_communication::goal local_goal;
  sensor_msgs::LaserScan scan;
  nav_msgs::OccupancyGrid local_map;
  geometry_msgs::Twist current_velocity;
  // 速度矢量消息
  double vector_Velocity;
  // 总的里程
  double all_length;
  bool local_goal_subscribed;
  bool scan_updated;
  bool local_map_updated;
  bool odom_updated;
  bool target_velocity_update;    //目标速度是否更新
  bool update_path;
  bool updateEndFlag;           // 终点是否更新
};

#endif //__DWA_PLANNER_H


