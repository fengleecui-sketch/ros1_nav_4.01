#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <robot_communication/localizationInfoBroadcast.h>

#include <cmath>
#include <mutex>
#include <vector>
#include <algorithm>
#include <utility>
#include <string>

static double wrapToPi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

static double clampd(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}

class DiffPurePursuit {
public:
  DiffPurePursuit(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {

    // ---- topics ----
    pnh_.param<std::string>("topic_path", topic_path_, "/opt_path");
    pnh_.param<std::string>("topic_odom", topic_odom_, "/truth_pose_odom");
    pnh_.param<std::string>("topic_map",  topic_map_,  "/local_map_inflate");
    pnh_.param<std::string>("topic_cmd",  topic_cmd_,  "/chassis_control");

    // ---- pure pursuit params ----
    pnh_.param<double>("lookahead_dist", lookahead_dist_, 0.8);
    pnh_.param<double>("goal_tolerance", goal_tolerance_, 0.25);
    pnh_.param<double>("max_v", max_v_, 0.6);
    pnh_.param<double>("max_w", max_w_, 1.2);
    pnh_.param<double>("min_v", min_v_, 0.05);
    pnh_.param<double>("k_heading", k_heading_, 1.8);
    pnh_.param<double>("v_scale", v_scale_, 0.8);

    // ---- timing ----
    pnh_.param<double>("cmd_rate", cmd_rate_, 30.0);
    pnh_.param<double>("timeout_path", timeout_path_, 1.0);
    pnh_.param<double>("timeout_odom", timeout_odom_, 0.5);
    pnh_.param<double>("timeout_map",  timeout_map_,  1.0);

    // ---- local avoidance ----
    pnh_.param<bool>("enable_avoid", enable_avoid_, true);

    // 说明：
    // - 这里 occ_threshold 表示 “>=threshold 判为障碍”
    // - 你的 local_map_esdf 通常会有大量 -1（unknown），现在不会再把 -1 当障碍
    pnh_.param<int>("occ_threshold", occ_threshold_, 98);
    pnh_.param<double>("robot_radius", robot_radius_, 0.25);

    // 预测时域/步长：轨迹长度约 = v * sim_horizon；点数约 = sim_horizon/sim_dt
    pnh_.param<double>("sim_horizon", sim_horizon_, 2.5);
    pnh_.param<double>("sim_dt", sim_dt_, 0.03);

    pnh_.param<int>("v_samples", v_samples_, 8);
    pnh_.param<int>("w_samples", w_samples_, 15);

    // cost weights
    pnh_.param<double>("w_track",  w_track_,  2.0);
    pnh_.param<double>("w_obs",    w_obs_,    1.0);
    pnh_.param<double>("w_smooth", w_smooth_, 0.2);

    // ---- visualization ----
    pnh_.param<bool>("publish_rollouts", publish_rollouts_, true);
    pnh_.param<int>("max_rollout_markers", max_rollout_markers_, 140);
    pnh_.param<std::string>("viz_frame", viz_frame_, ""); // 空：自动跟随地图frame
    pnh_.param<double>("rollout_line_width", rollout_line_width_, 0.03);
    pnh_.param<double>("rollout_z", rollout_z_, 0.05);

    // ---- pubs/subs ----
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_cmd_, 1);

    rollout_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/diff_pp/rollouts", 1);
    best_path_pub_       = nh_.advertise<nav_msgs::Path>("/diff_pp/best_path", 1);

    path_sub_ = nh_.subscribe(topic_path_, 1, &DiffPurePursuit::pathCb, this);
    odom_sub_ = nh_.subscribe(topic_odom_, 1, &DiffPurePursuit::odomCb, this);
    map_sub_  = nh_.subscribe(topic_map_,  1, &DiffPurePursuit::mapCb,  this);

    timer_ = nh_.createTimer(ros::Duration(1.0 / cmd_rate_), &DiffPurePursuit::onTimer, this);

    ROS_INFO("[DiffPurePursuit] start. path=%s odom=%s map=%s cmd=%s avoid=%s rollouts=%s",
             topic_path_.c_str(), topic_odom_.c_str(), topic_map_.c_str(), topic_cmd_.c_str(),
             enable_avoid_ ? "true" : "false",
             publish_rollouts_ ? "true" : "false");
  }

private:
  // ================== callbacks ==================
  void pathCb(const nav_msgs::PathConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    path_ = *msg;
    last_path_time_ = ros::Time::now();
  }

  void odomCb(const robot_communication::localizationInfoBroadcastConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);

    // ⚠️ 若你的 msg 字段名不同，就改这几行
    x_ = msg->xPosition;
    y_ = msg->yPosition;
    yaw_ = msg->chassisAngle;      // 假设是弧度
    vx_meas_ = msg->xSpeed;
    wz_meas_ = msg->chassisGyro;

    last_odom_time_ = ros::Time::now();
  }

  void mapCb(const nav_msgs::OccupancyGridConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    map_ = *msg;
    last_map_time_ = ros::Time::now();
  }

  // ================== helpers ==================
  bool pickLookaheadPoint(const nav_msgs::Path& path, double x, double y, double& tx, double& ty) const {
    if (path.poses.empty()) return false;

    int nearest_i = 0;
    double best_d2 = 1e100;
    for (int i = 0; i < (int)path.poses.size(); ++i) {
      double px = path.poses[i].pose.position.x;
      double py = path.poses[i].pose.position.y;
      double d2 = (px - x) * (px - x) + (py - y) * (py - y);
      if (d2 < best_d2) {
        best_d2 = d2;
        nearest_i = i;
      }
    }

    double accum = 0.0;
    double prev_x = path.poses[nearest_i].pose.position.x;
    double prev_y = path.poses[nearest_i].pose.position.y;

    for (int i = nearest_i + 1; i < (int)path.poses.size(); ++i) {
      double px = path.poses[i].pose.position.x;
      double py = path.poses[i].pose.position.y;
      double ds = std::hypot(px - prev_x, py - prev_y);
      accum += ds;
      prev_x = px; prev_y = py;

      if (accum >= lookahead_dist_) {
        tx = px; ty = py;
        return true;
      }
    }

    tx = path.poses.back().pose.position.x;
    ty = path.poses.back().pose.position.y;
    return true;
  }

  void stop() {
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.linear.y = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_.publish(cmd);
  }

  bool worldToMap(const nav_msgs::OccupancyGrid& map, double wx, double wy, int& mx, int& my) const {
    const double ox = map.info.origin.position.x;
    const double oy = map.info.origin.position.y;
    const double res = map.info.resolution;

    mx = (int)std::floor((wx - ox) / res);
    my = (int)std::floor((wy - oy) / res);

    if (mx < 0 || my < 0) return false;
    if (mx >= (int)map.info.width || my >= (int)map.info.height) return false;
    return true;
  }

  int8_t getCell(const nav_msgs::OccupancyGrid& map, int mx, int my) const {
    const int idx = mx + my * (int)map.info.width;
    if (idx < 0 || idx >= (int)map.data.size()) return -1;
    return map.data[idx];
  }

  // ✅ 修复点1：unknown(-1) 与 地图出界 不再直接当“障碍”
  bool isOccupied(const nav_msgs::OccupancyGrid& map, double wx, double wy) const {
    int mx, my;
    if (!worldToMap(map, wx, wy, mx, my)) {
      // 地图外：不要直接当障碍（否则 horizon 变长时全死）
      return false;
    }
    int8_t v = getCell(map, mx, my);
    if (v < 0) {
      // unknown：不要直接当障碍（local map 未铺满会全死）
      return false;
    }
    return (int)v >= occ_threshold_;
  }

  bool footprintCollision(const nav_msgs::OccupancyGrid& map, double x, double y, double yaw) const {
    (void)yaw;
    const double r = robot_radius_;
    const double pts[5][2] = {
      {0.0, 0.0},
      { r, 0.0},
      {-r, 0.0},
      {0.0,  r},
      {0.0, -r}
    };

    for (int i = 0; i < 5; ++i) {
      if (isOccupied(map, x + pts[i][0], y + pts[i][1])) return true;
    }
    return false;
  }

  // simulate and score a candidate (v,w)
  bool simulateAndScore(const nav_msgs::OccupancyGrid& map,
                        double x0, double y0, double yaw0,
                        double v, double w,
                        double tx, double ty,
                        double& track_cost,
                        double& obs_cost) const {
    double x = x0, y = y0, yaw = yaw0;

    obs_cost = 0.0;

    const int steps = std::max(1, (int)std::ceil(sim_horizon_ / sim_dt_));
    for (int k = 0; k < steps; ++k) {
      x += v * std::cos(yaw) * sim_dt_;
      y += v * std::sin(yaw) * sim_dt_;
      yaw = wrapToPi(yaw + w * sim_dt_);

      // 仅对“明确障碍”判碰撞
      if (footprintCollision(map, x, y, yaw)) {
        return false;
      }

      // ✅ 修复点2：unknown/出界只惩罚，不判死
      int mx, my;
      if (worldToMap(map, x, y, mx, my)) {
        int8_t val = getCell(map, mx, my);
        if (val >= 0) {
          obs_cost += (double)val / 100.0;  // 正常代价
        } else {
          obs_cost += 0.2;                  // unknown：轻微惩罚
        }
      } else {
        obs_cost += 0.5;                    // 出界：惩罚但不判死
      }
    }

    // 末端跟踪代价：末端到追踪点距离 + 航向误差
    const double dx = tx - x;
    const double dy = ty - y;
    const double dist = std::hypot(dx, dy);
    const double desired_heading = std::atan2(dy, dx);
    const double heading_err = std::fabs(wrapToPi(desired_heading - yaw));

    track_cost = dist + 0.5 * heading_err;
    return true;
  }

  // rollout point list for visualization / best path
  std::vector<std::pair<double,double>> rolloutPoints(double x0, double y0, double yaw0,
                                                      double v, double w) const {
    std::vector<std::pair<double,double>> pts;
    double x = x0, y = y0, yaw = yaw0;

    const int steps = std::max(1, (int)std::ceil(sim_horizon_ / sim_dt_));
    pts.reserve(steps + 1);
    pts.push_back({x, y});

    for (int k = 0; k < steps; ++k) {
      x += v * std::cos(yaw) * sim_dt_;
      y += v * std::sin(yaw) * sim_dt_;
      yaw = wrapToPi(yaw + w * sim_dt_);
      pts.push_back({x, y});
    }
    return pts;
  }

  std::string resolveVizFrame(const nav_msgs::OccupancyGrid& map_copy) const {
    if (!viz_frame_.empty()) return viz_frame_;
    if (!map_copy.header.frame_id.empty()) return map_copy.header.frame_id;
    return std::string("map");
  }

  void publishBestPath(const ros::Time& stamp,
                       const std::string& frame,
                       const std::vector<std::pair<double,double>>& best_pts) {
    if (best_pts.empty()) return;

    nav_msgs::Path best;
    best.header.stamp = stamp;
    best.header.frame_id = frame;

    best.poses.reserve(best_pts.size());
    for (size_t k = 0; k < best_pts.size(); ++k) {
      geometry_msgs::PoseStamped ps;
      ps.header = best.header;
      ps.pose.position.x = best_pts[k].first;
      ps.pose.position.y = best_pts[k].second;
      ps.pose.position.z = 0.0;
      ps.pose.orientation.w = 1.0;
      best.poses.push_back(ps);
    }
    best_path_pub_.publish(best);
  }

  void publishRolloutMarkers(const ros::Time& stamp,
                             const std::string& frame,
                             const std::vector<std::vector<std::pair<double,double>>>& all_pts,
                             const std::vector<double>& dangers) {
    if (!publish_rollouts_) return;

    visualization_msgs::MarkerArray marray;

    // 清理上一帧
    visualization_msgs::Marker del;
    del.action = visualization_msgs::Marker::DELETEALL;
    marray.markers.push_back(del);

    const int N = (int)all_pts.size();
    for (int i = 0; i < N; ++i) {
      visualization_msgs::Marker mk;
      mk.header.frame_id = frame;
      mk.header.stamp = stamp;
      mk.ns = "rollouts";
      mk.id = i;
      mk.type = visualization_msgs::Marker::LINE_STRIP;
      mk.action = visualization_msgs::Marker::ADD;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = rollout_line_width_;

      const double danger = clampd(dangers[i], 0.0, 1.0);
      mk.color.r = danger;
      mk.color.g = 1.0 - danger;
      mk.color.b = 0.0;
      mk.color.a = 0.8;

      mk.points.reserve(all_pts[i].size());
      for (auto& p : all_pts[i]) {
        geometry_msgs::Point pt;
        pt.x = p.first;
        pt.y = p.second;
        pt.z = rollout_z_;
        mk.points.push_back(pt);
      }

      mk.lifetime = ros::Duration(0.25);
      marray.markers.push_back(mk);
    }

    rollout_markers_pub_.publish(marray);
  }

  // ================== main loop ==================
  void onTimer(const ros::TimerEvent&) {
    nav_msgs::Path path_copy;
    nav_msgs::OccupancyGrid map_copy;
    ros::Time t_path, t_odom, t_map;
    double x, y, yaw;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      path_copy = path_;
      map_copy  = map_;
      t_path = last_path_time_;
      t_odom = last_odom_time_;
      t_map  = last_map_time_;
      x = x_; y = y_; yaw = yaw_;
    }

    const ros::Time now = ros::Time::now();

    if ((now - t_odom).toSec() > timeout_odom_) {
      ROS_WARN_THROTTLE(1.0, "[DiffPurePursuit] odom timeout -> stop");
      stop();
      return;
    }
    if ((now - t_path).toSec() > timeout_path_ || path_copy.poses.empty()) {
      ROS_WARN_THROTTLE(1.0, "[DiffPurePursuit] path timeout/empty -> stop");
      stop();
      return;
    }

    if (enable_avoid_) {
      if ((now - t_map).toSec() > timeout_map_ || map_copy.data.empty()) {
        ROS_WARN_THROTTLE(1.0, "[DiffPurePursuit] map timeout/empty (avoid enabled) -> stop");
        stop();
        return;
      }
    }

    // 选追踪点
    double tx, ty;
    if (!pickLookaheadPoint(path_copy, x, y, tx, ty)) {
      stop();
      return;
    }

    // 是否到达目标
    const double gx = path_copy.poses.back().pose.position.x;
    const double gy = path_copy.poses.back().pose.position.y;
    const double dist_to_goal = std::hypot(gx - x, gy - y);
    if (dist_to_goal < goal_tolerance_) {
      ROS_INFO_THROTTLE(1.0, "[DiffPurePursuit] goal reached -> stop");
      stop();
      return;
    }

    // ===== Pure Pursuit reference (v_ref, w_ref) =====
    const double dx = tx - x;
    const double dy = ty - y;
    const double c = std::cos(-yaw);
    const double s = std::sin(-yaw);
    double lx = c * dx - s * dy;
    double ly = s * dx + c * dy;

    if (lx < 0.05) lx = 0.05;

    const double heading_err = wrapToPi(std::atan2(ly, lx));
    const double w_ref = clampd(k_heading_ * heading_err, -max_w_, max_w_);

    double v_ref = max_v_ * v_scale_ * std::cos(heading_err);
    v_ref = clampd(v_ref, 0.0, max_v_);
    if (v_ref > 0.0 && v_ref < min_v_) v_ref = min_v_;

    double v_cmd = v_ref;
    double w_cmd = w_ref;

    std::vector<std::pair<double,double>> best_pts;

    // ===== Avoidance sampling =====
    std::vector<std::vector<std::pair<double,double>>> all_rollouts;
    std::vector<double> all_dangers;

    if (enable_avoid_) {
      const int vs = std::max(1, v_samples_);
      const int ws = std::max(3, w_samples_);

      double best_cost = 1e100;
      bool found = false;
      int safe_cnt = 0;
      int total_cnt = 0;

      if (publish_rollouts_) {
        all_rollouts.reserve(std::min(max_rollout_markers_, vs * ws));
        all_dangers.reserve(std::min(max_rollout_markers_, vs * ws));
      }

      const double steps = std::max(1.0, std::ceil(sim_horizon_ / sim_dt_));

      for (int i = 0; i < vs; ++i) {
        double v = (vs == 1) ? v_ref : (double)i / (double)(vs - 1) * max_v_;
        if (v > 0.0 && v < min_v_) v = min_v_;

        for (int j = 0; j < ws; ++j) {
          ++total_cnt;
          const double alpha = (ws == 1) ? 0.0 : (double)j / (double)(ws - 1);
          const double w = -max_w_ + alpha * (2.0 * max_w_);

          double track_cost = 0.0, obs_cost = 0.0;
          const bool safe = simulateAndScore(map_copy, x, y, yaw, v, w, tx, ty, track_cost, obs_cost);
          if (!safe) continue;

          ++safe_cnt;

          // 绘制 safe 的轨迹
          if (publish_rollouts_ && (int)all_rollouts.size() < max_rollout_markers_) {
            all_rollouts.push_back(rolloutPoints(x, y, yaw, v, w));

            // ✅ 修复点3：危险度按步数平均，避免 horizon 变长导致“全红”
            const double danger = clampd((obs_cost / steps), 0.0, 1.0);
            all_dangers.push_back(danger);
          }

          const double smooth = std::fabs(v - v_ref) + 0.3 * std::fabs(w - w_ref);
          const double total_cost = w_track_ * track_cost + w_obs_ * obs_cost + w_smooth_ * smooth;

          if (total_cost < best_cost) {
            best_cost = total_cost;
            v_cmd = v;
            w_cmd = w;
            found = true;
            best_pts = rolloutPoints(x, y, yaw, v, w);
          }
        }
      }

      ROS_INFO_THROTTLE(0.5, "[DiffPurePursuit] sample safe=%d/%d  v_ref=%.3f w_ref=%.3f  v_cmd=%.3f w_cmd=%.3f",
                        safe_cnt, total_cnt, v_ref, w_ref, v_cmd, w_cmd);

      const std::string frame = resolveVizFrame(map_copy);
      if (publish_rollouts_) publishRolloutMarkers(now, frame, all_rollouts, all_dangers);
      publishBestPath(now, frame, best_pts);

      if (!found) {
        ROS_WARN_THROTTLE(1.0, "[DiffPurePursuit] no safe (v,w) found -> stop");
        stop();
        return;
      }
    } else {
      // 避障关闭：仍发布轨迹方便调参
      const std::string frame = resolveVizFrame(map_copy);
      best_pts = rolloutPoints(x, y, yaw, v_cmd, w_cmd);
      publishBestPath(now, frame, best_pts);

      if (publish_rollouts_) {
        std::vector<std::vector<std::pair<double,double>>> one{best_pts};
        std::vector<double> danger{0.0};
        publishRolloutMarkers(now, frame, one, danger);
      }
    }

    // publish command
    geometry_msgs::Twist cmd;
    cmd.linear.x = v_cmd;
    cmd.linear.y = 0.0;
    cmd.angular.z = w_cmd;
    cmd_pub_.publish(cmd);
  }

private:
  ros::NodeHandle nh_, pnh_;

  ros::Subscriber path_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber map_sub_;

  ros::Publisher  cmd_pub_;
  ros::Publisher  rollout_markers_pub_;
  ros::Publisher  best_path_pub_;

  ros::Timer timer_;

  std::mutex mtx_;

  nav_msgs::Path path_;
  nav_msgs::OccupancyGrid map_;
  ros::Time last_path_time_{0};
  ros::Time last_odom_time_{0};
  ros::Time last_map_time_{0};

  // pose
  double x_{0}, y_{0}, yaw_{0};
  double vx_meas_{0}, wz_meas_{0};

  // topics
  std::string topic_path_, topic_odom_, topic_map_, topic_cmd_;

  // pure pursuit params
  double lookahead_dist_{0.8};
  double goal_tolerance_{0.25};
  double max_v_{0.6}, max_w_{1.2}, min_v_{0.05};
  double k_heading_{1.8};
  double v_scale_{0.8};

  // timing
  double cmd_rate_{30.0};
  double timeout_path_{1.0}, timeout_odom_{0.5}, timeout_map_{1.0};

  // avoid params
  bool enable_avoid_{true};
  int occ_threshold_{98};
  double robot_radius_{0.25};
  double sim_horizon_{2.5};
  double sim_dt_{0.03};
  int v_samples_{8};
  int w_samples_{15};
  double w_track_{2.0};
  double w_obs_{1.0};
  double w_smooth_{0.2};

  // visualization params
  bool publish_rollouts_{true};
  int  max_rollout_markers_{140};
  std::string viz_frame_;
  double rollout_line_width_{0.03};
  double rollout_z_{0.05};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "diff_pure_pursuit_local_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  DiffPurePursuit node(nh, pnh);
  ros::spin();
  return 0;
}
