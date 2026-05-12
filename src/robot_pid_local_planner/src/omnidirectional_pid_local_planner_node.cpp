#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <robot_communication/localizationInfoBroadcast.h>

#include <cmath>
#include <algorithm>
#include <nav_msgs/Odometry.h>

#include "iostream"

using namespace std;

// 订阅全局/上层给的路径 /opt_path，在路径上选一个前瞻点（lookahead point），
// 把“当前位置 → 前瞻点”的误差变成速度指令 cmd_vel（全向：vx、vy、wz）。

//把角度差归一化到 [-pi, pi]，避免“359° - 1° = 358°”这种跳变
static inline double wrap_to_pi(double a)
{
  while (a > M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

//把值夹在范围内，速度限幅用
static inline double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

struct PID
{
  double kp{0}, ki{0}, kd{0};
  double i{0}, prev_e{0};  // 上一次误差
  double i_limit{1.0};     // 积分限幅

  //给误差 e、时间间隔 dt，输出控制量u: kp*e + ki*i + kd*de
  double step(double e, double dt)
  {
    if (dt <= 1e-6) return kp*e;

    // I
    i += e*dt;
    i = clamp(i, -i_limit, i_limit);

    // D
    const double de = (e - prev_e)/dt;
    prev_e = e;

    return kp*e + ki*i + kd*de;
  }

  // 把积分和 prev_e 清零
  void reset()
  {
    i = 0;
    prev_e = 0;
  }
};

class OmnidirectionalPIDLocalPlanner
{
public:
  OmnidirectionalPIDLocalPlanner()
  : nh_(), pnh_("~")
  {
    // params（用全局参数加载也可以；这里两边都兼容）
    nh_.param("IS_SIM", is_sim_, false);

    nh_.param("LOOKAHEAD_DIST", lookahead_dist_, 0.5);  //前瞻距离
    nh_.param("GOAL_TOL", goal_tol_, 0.25);

    nh_.param("MAX_VX", max_vx_, 0.4);
    nh_.param("MAX_VY", max_vy_, 0.4);
    nh_.param("MAX_WZ", max_wz_, 0.8);

    nh_.param("KP_X", pid_x_.kp, 0.9);
    nh_.param("KI_X", pid_x_.ki, 0.0);
    nh_.param("KD_X", pid_x_.kd, 0.0);

    nh_.param("KP_Y", pid_y_.kp, 0.9);
    nh_.param("KI_Y", pid_y_.ki, 0.0);
    nh_.param("KD_Y", pid_y_.kd, 0.0);

    nh_.param("KP_YAW", pid_yaw_.kp, 1.2);
    nh_.param("KI_YAW", pid_yaw_.ki, 0.0);
    nh_.param("KD_YAW", pid_yaw_.kd, 0.0);

    nh_.param("I_LIMIT_X", pid_x_.i_limit, 0.6);
    nh_.param("I_LIMIT_Y", pid_y_.i_limit, 0.6);
    nh_.param("I_LIMIT_YAW", pid_yaw_.i_limit, 1.0);

    nh_.param("CMD_TIMEOUT", cmd_timeout_, 0.5);
    nh_.param("PUB_HZ", pub_hz_, 30.0);

    // ===== 到达终点后的行为（关键） =====
    // 你希望：到达终点后，键盘可以随意挪车，局部算法不会把车“拉回终点”。
    // 做法：到达后进入 IDLE（锁存 reached 状态），持续输出 0（或不再输出），
    //      直到收到“新路径/新目标”再解除 IDLE。
    nh_.param("IDLE_LATCH_ON_REACHED", idle_latch_on_reached_, true);
    nh_.param("PUBLISH_ZERO_WHEN_IDLE", publish_zero_when_idle_, true);
    nh_.param("STOP_V_TOL", stop_v_tol_, 0.03);   // m/s，用于到达判定（可选）
    nh_.param("STOP_W_TOL", stop_w_tol_, 0.05);   // rad/s
    nh_.param("USE_SPEED_CHECK", use_speed_check_, true);
    nh_.param("NEW_PATH_EPS", new_path_eps_, 0.02); // m，判断“新路径”的末点变化阈值

    nh_.param("LOOKAHEAD_DIST", lookahead_dist_, 0.5); 
    nh_.param("GOAL_TOL", goal_tol_, 0.25);
    nh_.param("YAW_TOL", yaw_tol_, 0.1);  // 【新增】读取偏航角容忍度参数

    // TF 相关参数
    nh_.param("GLOBAL_FRAME", global_frame_, std::string("map"));
    nh_.param("ROBOT_FRAME", robot_base_frame_, std::string("base_link"));

    // topics（保持与你的 DWA 一致）
    path_sub_ = nh_.subscribe("/opt_path", 1, &OmnidirectionalPIDLocalPlanner::pathCb, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &OmnidirectionalPIDLocalPlanner::goalCb, this);

    if (is_sim_)
    {
      // odom_sim_sub_ = nh_.subscribe("/truth_pose_odom", 1, &OmnidirectionalPIDLocalPlanner::simOdomCb, this);
      odom_sim_sub_ = nh_.subscribe("/truth_pose_odom", 1, &OmnidirectionalPIDLocalPlanner::simOdomCb, this);
    }
    else
    {
      //odom_carto_sub_ = nh_.subscribe("/carto_odom", 1, &OmnidirectionalPIDLocalPlanner::odomCb, this);
      //odom_carto_sub_ = nh_.subscribe("/truthPose", 1, &OmnidirectionalPIDLocalPlanner::odomCb, this);
    }

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_auto", 1);

    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/local_goal", 1, true);

    last_cmd_time_ = ros::Time(0);

    timer_ = nh_.createTimer(ros::Duration(1.0/std::max(1.0, pub_hz_)),
                             &OmnidirectionalPIDLocalPlanner::onTimer, this);

    ROS_INFO("[robot_pid_local_planner] started. IS_SIM=%s, lookahead=%.2f, pub_hz=%.1f",
             is_sim_ ? "true":"false", lookahead_dist_, pub_hz_);
    ROS_INFO("[robot_pid_local_planner] idle_latch_on_reached=%s publish_zero_when_idle=%s use_speed_check=%s",
             idle_latch_on_reached_?"true":"false",
             publish_zero_when_idle_?"true":"false",
             use_speed_check_?"true":"false");
  }

private:
  //收到 /opt_path 存起来
  void pathCb(const nav_msgs::PathConstPtr& msg)
  {
    path_ = *msg;
    has_path_ = !path_.poses.empty();

    // ========= 新路径判定：不要用 stamp！！=========
    // 只在“末点变化明显”或“size变化明显”时，才认为是新路径
    bool new_path = false;

    const size_t sz = path_.poses.size();

    if (sz != last_path_size_) {
      // 允许小抖动：比如 sz 只是重复发布同样size，不算新
      // 但如果差异很大，必然是新路径
      if (std::abs((int)sz - (int)last_path_size_) >= 2) new_path = true;
    }

    if (sz > 0) {
      const auto& last = path_.poses.back().pose.position;
      const double dlast = std::hypot(last.x - last_path_last_x_, last.y - last_path_last_y_);
      if (dlast > new_path_eps_) new_path = true;

      last_path_last_x_ = last.x;
      last_path_last_y_ = last.y;
    }

    last_path_size_ = sz;

    // ========= 只有真正新路径/新目标才解除 IDLE =========
    if (has_path_ && new_path) {
      reached_latched_ = false;
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      ROS_INFO("[robot_pid_local_planner] new path detected -> exit IDLE");
    }
  }

  void goalCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    goal_ = *msg;
    has_goal_ = true;
    
    // 【新增】保存目标方向（从RViz点击确定）
    goal_yaw_ = tf::getYaw(goal_.pose.orientation);
    ROS_INFO("[robot_pid_local_planner] received goal at (%.2f, %.2f) with yaw=%.2f(%.1f°)",
             goal_.pose.position.x, goal_.pose.position.y, goal_yaw_, goal_yaw_*180/M_PI);

    // 收到新目标：解除 reached 锁存
    reached_latched_ = false;
    pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
  }

  // 取当前位姿 (x,y,yaw)
  void simOdomCb(const robot_communication::localizationInfoBroadcastConstPtr& msg)
  {
    x_ = msg->xPosition;
    y_ = msg->yPosition;
    yaw_ = msg->chassisAngle; // 【修复】使用 chassisAngle 而不是 chassisGyro（角速度）

    vx_fb_ = msg->xSpeed;
    vy_fb_ = msg->ySpeed;
    wz_fb_ = msg->chassisGyro;

    has_odom_ = true;
  }

  // 取当前位姿 (x,y,yaw)
  void odomCb(const nav_msgs::OdometryConstPtr& msg)
  {
x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    yaw_ = tf::getYaw(msg->pose.pose.orientation);
    vx_fb_ = msg->twist.twist.linear.x;
    vy_fb_ = msg->twist.twist.linear.y;
    wz_fb_ = msg->twist.twist.angular.z;
    has_odom_ = true;
  }

  // 选“前瞻点”
  // 选“前瞻点”并计算目标朝向
  bool computeLookaheadPoint(double& gx, double& gy, double& g_yaw_target)
  {
    if (!has_path_ || path_.poses.empty()) return false;

    // 1) 找最近点
    int nearest = 0;
    double best_d2 = 1e100;
    for (int i = 0; i < (int)path_.poses.size(); ++i)
    {
      const double px = path_.poses[i].pose.position.x;
      const double py = path_.poses[i].pose.position.y;
      const double dx = px - x_;
      const double dy = py - y_;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; nearest = i; }
    }

    // 2) 从 nearest 开始累计到 lookahead_dist
    double acc = 0.0;
    int idx = nearest;
    for (int i = nearest; i+1 < (int)path_.poses.size(); ++i)
    {
      const double x0 = path_.poses[i].pose.position.x;
      const double y0 = path_.poses[i].pose.position.y;
      const double x1 = path_.poses[i+1].pose.position.x;
      const double y1 = path_.poses[i+1].pose.position.y;
      const double seg = std::hypot(x1-x0, y1-y0);
      acc += seg;
      if (acc >= lookahead_dist_) { idx = i+1; break; }
      idx = i+1;
    }

    gx = path_.poses[idx].pose.position.x; // 【修复】应该是 .x 而不是 .y
    gy = path_.poses[idx].pose.position.y; // 【修复】添加 gy 赋值

    // ================== 新增：朝向控制逻辑 ==================

    // A. 计算规划路径在当前前瞻点处的切线方向
    double path_tangent_yaw = yaw_; // 默认保持当前方向
    
    // 计算从最近点到前瞻点的方向（确保沿路径正确方向行驶）
    if (idx > 0) {
      // 用前一个点到当前前瞻点的向量作为路径切线
      const double dx = path_.poses[idx].pose.position.x - path_.poses[idx-1].pose.position.x;
      const double dy = path_.poses[idx].pose.position.y - path_.poses[idx-1].pose.position.y;
      const double dist = std::hypot(dx, dy);
      if (dist > 1e-6) {
        path_tangent_yaw = std::atan2(dy, dx);
      }
    }
    
    // 备选：如果前瞻点后面还有点，也可以用前瞻点和下一个点
    if (idx + 1 < (int)path_.poses.size()) {
      const double dx = path_.poses[idx+1].pose.position.x - path_.poses[idx].pose.position.x;
      const double dy = path_.poses[idx+1].pose.position.y - path_.poses[idx].pose.position.y;
      const double dist = std::hypot(dx, dy);
      if (dist > 1e-6) {
        // 使用前瞻：更提前地调整方向
        path_tangent_yaw = std::atan2(dy, dx);
      }
    }

    // B. 获取 RViz 设定的真实最终目标朝向（从 goal_ 中取，绝不能从 path 取）
    double final_goal_yaw = 0.0;
    if (has_goal_) {
      final_goal_yaw = tf::getYaw(goal_.pose.orientation);
    }

    // C. 距离终点较近时，平滑切换到最终姿态；否则车头对准路径切线
    const auto& last_pos = path_.poses.back().pose.position;
    double dist_to_last = std::hypot(last_pos.x - x_, last_pos.y - y_);

    // 【改进】使用平滑的过渡而不是硬切换
    // 过渡距离：至少 1.5 倍前瞻距离，但不少于 0.5m
    double transition_dist = std::max(lookahead_dist_ * 1.5, 0.5);
    
    if (dist_to_last < transition_dist && has_goal_) {
      // 【平滑过渡】：使用线性插值在路径切线和目标朝向之间过渡
      double blend_ratio = dist_to_last / transition_dist;  // 0 ~ 1
      double angle_diff = wrap_to_pi(final_goal_yaw - path_tangent_yaw);
      g_yaw_target = path_tangent_yaw + (1.0 - blend_ratio) * angle_diff;  // 平滑过渡
    } else {
      g_yaw_target = path_tangent_yaw; // 车头顺着路径切线方向
    }

    // ========================================================

    // 发布局部路径（可视化：nearest 到 idx）
    nav_msgs::Path local;
    local.header = path_.header;
    for (int i = nearest; i <= idx; ++i) local.poses.push_back(path_.poses[i]);
    local_path_pub_.publish(local);

    // marker (保持不变)
    visualization_msgs::Marker mk;
    mk.header = path_.header;
    mk.ns = "pid_local_goal";
    mk.id = 1;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = gx;
    mk.pose.position.y = gy;
    mk.pose.position.z = 0.1;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.18;
    mk.scale.y = 0.18;
    mk.scale.z = 0.18;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.8;
    mk.color.b = 0.0;
    marker_pub_.publish(mk);

    return true;
  }

  // ===== 到达终点判定（使用 RViz 设定的目标方向） =====
  bool isReachedGoal() const
  {
    if (!has_path_ || path_.poses.empty()) return false;
    const auto& last_pos = path_.poses.back().pose.position;
    
    // 1. 判断位置误差
    const double dist_goal = std::hypot(last_pos.x - x_, last_pos.y - y_);
    if (dist_goal > goal_tol_) return false;

    // 2. 判断角度误差（必须使用真实的 goal_ 的姿态）
    if (has_goal_) {
      double final_yaw = tf::getYaw(goal_.pose.orientation);
      double yaw_error = std::abs(wrap_to_pi(final_yaw - yaw_));
      // yaw_tol_ 如果之前没在私有变量定义，记得在类的末尾加一句 double yaw_tol_{0.1};
      if (yaw_error > yaw_tol_) return false; 
    }

    // 3. 判断速度是否彻底停稳
    if (use_speed_check_)
    {
      const double v = std::hypot(vx_fb_, vy_fb_);
      if (v > stop_v_tol_) return false;
      if (std::fabs(wz_fb_) > stop_w_tol_) return false;
    }

    return true;
  }

  // 刹停输出
  void publishZero()
  {
    geometry_msgs::Twist z;
    cmd_pub_.publish(z);
  }

 void onTimer(const ros::TimerEvent& ev)
  {
    // 【新增】通过 TF 获取机器人当前的全局位姿
    tf::StampedTransform transform;
    try {
      // 查询最新的全局坐标系到机器人底盘的变换
      tf_listener_.lookupTransform(global_frame_, robot_base_frame_, ros::Time(0), transform);
      
      x_ = transform.getOrigin().x();
      y_ = transform.getOrigin().y();
      yaw_ = tf::getYaw(transform.getRotation());
      
      // 只要成功获取到了 TF，就认为有了有效位置
      has_odom_ = true; 
    }
    catch (tf::TransformException &ex) {
      ROS_WARN_THROTTLE(1.0, "[robot_pid_local_planner] Cannot get TF: %s", ex.what());
      publishZero();
      return; // 拿不到当前位置，直接停车并返回
    }

    if (!has_odom_)
    {
      publishZero();
      cout<<"publishZero();"<<endl;
      return;
    }

    // 关键：如果到达后锁存为 IDLE，则不再恢复跟踪（直到收到新路径/新目标）
    if (idle_latch_on_reached_ && reached_latched_)
    {
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      if (publish_zero_when_idle_) publishZero();
      cout<<"idle_latch_on_reached_ && reached_latched_"<<endl;
      return;
    }

    // 如果满足到达终点：进入 IDLE（锁存 reached）
    if (idle_latch_on_reached_ && isReachedGoal())
    {
      reached_latched_ = true;
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      publishZero();
      ROS_INFO_THROTTLE(1.0, "[robot_pid_local_planner] reached goal -> IDLE (latched). manual can take over.");
      return;
    }

    // 正常跟踪
    double gx, gy, yaw_target;
    if (!computeLookaheadPoint(gx, gy, yaw_target))
    {
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      publishZero();
      return;
    }

    const double dt = (last_cmd_time_.isZero()) ? (1.0/std::max(1.0, pub_hz_))
                                                : (ros::Time::now() - last_cmd_time_).toSec();
    last_cmd_time_ = ros::Time::now();

    // 世界误差
    const double dx = gx - x_;
    const double dy = gy - y_;

    // 转到机器人坐标系（机器人前方为 +x，左为 +y）
    const double cy = std::cos(yaw_);
    const double sy = std::sin(yaw_);
    const double ex =  cy*dx + sy*dy;
    const double ey = -sy*dx + cy*dy;

    const double e_yaw = wrap_to_pi(yaw_target - yaw_);

    double vx = pid_x_.step(ex, dt);
    double vy = pid_y_.step(ey, dt);
    double wz = pid_yaw_.step(e_yaw, dt);

    // 限幅
    vx = clamp(vx, -max_vx_, max_vx_);
    vy = clamp(vy, -max_vy_, max_vy_);
    wz = clamp(wz, -max_wz_, max_wz_);

    geometry_msgs::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;

    cmd_pub_.publish(cmd);

    if (idle_latch_on_reached_ && isReachedGoal())
    {
      reached_latched_ = true;
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      publishZero();

      const auto& last = path_.poses.back().pose.position;
      const double dist_goal = std::hypot(last.x - x_, last.y - y_);
      ROS_WARN_THROTTLE(1.0, "[robot_pid_local_planner] REACHED & LATCHED IDLE. dist=%.3f", dist_goal);
      return;
    }

  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber path_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sim_sub_;
  ros::Subscriber odom_carto_sub_;

  ros::Publisher cmd_pub_;
  ros::Publisher local_path_pub_;
  ros::Publisher marker_pub_;

  ros::Timer timer_;
  ros::Time last_cmd_time_;

  nav_msgs::Path path_;
  geometry_msgs::PoseStamped goal_;

  bool is_sim_{true};
  bool has_path_{false};
  bool has_goal_{false};
  bool has_odom_{false};

  // ===== reached latch =====
  bool idle_latch_on_reached_{true};
  bool publish_zero_when_idle_{true};
  bool reached_latched_{false};
  bool use_speed_check_{true};
  double stop_v_tol_{0.03};
  double stop_w_tol_{0.05};
  double new_path_eps_{0.02};
  ros::Time last_path_stamp_{0};
  size_t last_path_size_{0};
  double last_path_last_x_{0.0}, last_path_last_y_{0.0};

  double lookahead_dist_{0.6};
  double goal_tol_{0.25};

  double yaw_tol_{0.1}; // 【新增】角度到达容忍度，0.1弧度约等于5.7度
  double max_vx_{0.4}, max_vy_{0.4}, max_wz_{0.8};
  double cmd_timeout_{0.5};
  double pub_hz_{30.0};

  // state
  double x_{0}, y_{0}, yaw_{0};
  double vx_fb_{0}, vy_fb_{0}, wz_fb_{0};
  double goal_yaw_{0.0}; // 【新增】从 RViz 设定的目标方向

  PID pid_x_, pid_y_, pid_yaw_;

  tf::TransformListener tf_listener_; // 【新增】TF 监听器
  std::string global_frame_{"map"};       // 【新增】全局坐标系名
  std::string robot_base_frame_{"base_link"}; // 【新增】机器人坐标系名
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "omnidirectional_pid_local_planner");
  OmnidirectionalPIDLocalPlanner node;
  ros::spin();
  return 0;
}
