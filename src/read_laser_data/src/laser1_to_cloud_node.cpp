#include <iostream>
#include <limits>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>          // ✅ for tf::Matrix3x3 setRPY
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

class Laser_to_Cloud {
public:
  Laser_to_Cloud();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

private:
  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  laser_geometry::LaserProjection projector;
  tf::TransformListener tfListener;

  std::string laserFrame;
  std::string target_frame;
  std::string input_scan_topic;
  std::string output_cloud_topic;
  std::string output_scan_topic;

  ros::Publisher point_cloud_publisher_;
  ros::Publisher rotated_scan_publisher_;
  ros::Subscriber scan_sub_;

  // ======= ✅ 你自己写死的旋转（单位：弧度）=======
  // 常见翻转：
  //  - 绕 X 翻 180°： roll = M_PI
  //  - 绕 Y 翻 180°： pitch = M_PI
  //  - 绕 Z 旋转：    yaw = ...
  const double MY_ROLL  = 0;   // <<< 你改这里
  const double MY_PITCH = 0;    // <<< 你改这里
  const double MY_YAW   = 0;    // <<< 你改这里
};

Laser_to_Cloud::Laser_to_Cloud() : private_node_("~")
{
  // 参数（你原来的逻辑保留）
  private_node_.param<std::string>("laserFrame", laserFrame, "laser");
  private_node_.param<std::string>("target_frame", target_frame, "base_footprint");
  private_node_.param<std::string>("input_scan_topic", input_scan_topic, "/not_true_scan");
  private_node_.param<std::string>("output_cloud_topic", output_cloud_topic, "/cloud_map");
  private_node_.param<std::string>("output_scan_topic", output_scan_topic, "/scan");

  // 订阅原始 scan（未翻转）
  scan_sub_ = node_.subscribe<sensor_msgs::LaserScan>(input_scan_topic, 10,
                                                     &Laser_to_Cloud::scanCallback, this);

  // 发布：翻转后的点云（保持：TF + laser_geometry）
  point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic, 10, false);

  // 发布：你自己旋转后的 LaserScan（不依赖TF）
  rotated_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan>(output_scan_topic, 10, false);

  tfListener.setExtrapolationLimit(ros::Duration(0.1));

  ROS_INFO("[Laser_to_Cloud] input_scan_topic=%s, output_scan_topic=%s, output_cloud_topic=%s, target_frame=%s",
           input_scan_topic.c_str(), output_scan_topic.c_str(), output_cloud_topic.c_str(), target_frame.c_str());

  ROS_INFO("[Laser_to_Cloud] /scan hard-coded RPY(rad): roll=%.6f pitch=%.6f yaw=%.6f",
           MY_ROLL, MY_PITCH, MY_YAW);
}

void Laser_to_Cloud::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  // =========================
  // 1) 点云：保持你现在的写法（TF + laser_geometry），用于局部建图
  // =========================
  sensor_msgs::PointCloud2 cloud_out;

  try {
    tfListener.waitForTransform(
        target_frame,
        scan->header.frame_id,
        scan->header.stamp,
        ros::Duration(0.05));

    projector.transformLaserScanToPointCloud(
        target_frame,
        *scan,
        cloud_out,
        tfListener);

    cloud_out.header.stamp = scan->header.stamp;
    cloud_out.header.frame_id = target_frame;
    point_cloud_publisher_.publish(cloud_out);
  }
  catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "Laser->Cloud TF failed (%s -> %s): %s",
                      scan->header.frame_id.c_str(), target_frame.c_str(), ex.what());
    // TF 都失败了，点云都发不出来，你这里选择直接 return（保持原逻辑）
    return;
  }

  // =========================
  // 2) /scan：单独出来——不使用 TF 的姿态，完全按你写死的 RPY 旋转
  // =========================

  // 2.1 复制一份 scan，设置输出 frame
  sensor_msgs::LaserScan out_scan = *scan;
  out_scan.header.stamp = scan->header.stamp;
  out_scan.header.frame_id = target_frame;   // 你希望Carto看到的frame（比如 base_footprint）

  const size_t N = scan->ranges.size();
  out_scan.ranges.assign(N, std::numeric_limits<float>::infinity());
  if (!scan->intensities.empty()) {
    out_scan.intensities.assign(N, 0.0f);
  }

  // 2.2 构造你自己的旋转矩阵（固定的）
  tf::Matrix3x3 R;
  R.setRPY(MY_ROLL, MY_PITCH, MY_YAW);

  // 2.3 逐束激光：scan平面点 -> 旋转 -> 投影回 2D -> bin回 LaserScan
  for (size_t i = 0; i < N; ++i) {
    const float r = scan->ranges[i];
    if (!std::isfinite(r)) continue;
    if (r < scan->range_min || r > scan->range_max) continue;

    const double ang = scan->angle_min + static_cast<double>(i) * scan->angle_increment;

    // scan frame 下的点（LaserScan 天然在 z=0 平面）
    tf::Vector3 p(r * std::cos(ang), r * std::sin(ang), 0.0);

    // ✅ 只用你自己的旋转（不读TF）
    tf::Vector3 q = R * p;

    const double x = q.x();
    const double y = q.y();

    const double rr = std::hypot(x, y);
    if (rr < scan->range_min || rr > scan->range_max) continue;

    const double a2 = std::atan2(y, x);

    // bin 到 out_scan 的角度数组中（沿用原 scan 的 angle_min / increment）
    const int idx = static_cast<int>(std::lround((a2 - scan->angle_min) / scan->angle_increment));
    if (idx < 0 || idx >= static_cast<int>(N)) continue;

    // 取更近的
    if (!std::isfinite(out_scan.ranges[idx]) || rr < out_scan.ranges[idx]) {
      out_scan.ranges[idx] = static_cast<float>(rr);
      if (!out_scan.intensities.empty() && i < scan->intensities.size()) {
        out_scan.intensities[idx] = scan->intensities[i];
      }
    }
  }

  rotated_scan_publisher_.publish(out_scan);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_to_cloud_and_scan_node");
  Laser_to_Cloud node;
  ros::spin();
  return 0;
}
