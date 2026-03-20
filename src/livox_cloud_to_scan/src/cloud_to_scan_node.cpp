#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Geometry>

#include <limits>
#include <cmath>
#include <string>

class CloudToScan
{
public:
  CloudToScan() : pnh_("~"), tf_buffer_(), tf_listener_(tf_buffer_)
  {
    pnh_.param<std::string>("cloud_topic", cloud_topic_, std::string("/livox/lidar"));
    pnh_.param<std::string>("scan_topic",  scan_topic_,  std::string("/scan"));
    pnh_.param<std::string>("target_frame", target_frame_, std::string("base_link"));
    pnh_.param<std::string>("output_frame", output_frame_, std::string("base_link")); // LaserScan header.frame_id

    pnh_.param<double>("angle_min", angle_min_, -M_PI);
    pnh_.param<double>("angle_max", angle_max_,  M_PI);
    pnh_.param<double>("angle_increment", angle_inc_, 0.005); // ~0.29deg
    pnh_.param<double>("range_min", range_min_, 0.10);
    pnh_.param<double>("range_max", range_max_, 30.0);

    // 过滤Z（把“地面/天花板”排除在2D之外）
    pnh_.param<double>("z_min", z_min_, -0.20);
    pnh_.param<double>("z_max", z_max_,  0.20);

    // 可选：只保留前方扇区，比如 [-90°, +90°]
    pnh_.param<bool>("use_angle_limits", use_angle_limits_, true);

    // 可选：点云太密时可简单降采样（按步长跳点）
    pnh_.param<int>("stride", stride_, 1);
    if (stride_ < 1) stride_ = 1;

    scan_pub_  = nh_.advertise<sensor_msgs::LaserScan>(scan_topic_, 10);
    cloud_sub_ = nh_.subscribe(cloud_topic_, 5, &CloudToScan::cloudCb, this);

    ROS_INFO("[livox_cloud_to_scan] cloud_topic=%s -> scan_topic=%s, target_frame=%s, z=[%.2f, %.2f]",
             cloud_topic_.c_str(), scan_topic_.c_str(), target_frame_.c_str(), z_min_, z_max_);
  }

private:
  void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // 1) PointCloud2 -> PCL
    pcl::PointCloud<pcl::PointXYZI> cloud_in;
    pcl::fromROSMsg(*msg, cloud_in);

    // 2) 获取变换：msg->header.frame_id  -> target_frame_
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const std::string& src_frame = msg->header.frame_id;

    try {
      geometry_msgs::TransformStamped tf_stamped =
          tf_buffer_.lookupTransform(target_frame_, src_frame, msg->header.stamp, ros::Duration(0.05));
      T = tf2::transformToEigen(tf_stamped.transform);
    } catch (const std::exception& e) {
      ROS_WARN_THROTTLE(1.0, "[livox_cloud_to_scan] TF lookup failed %s -> %s : %s",
                        src_frame.c_str(), target_frame_.c_str(), e.what());
      return;
    }

    // 3) 准备 LaserScan
    sensor_msgs::LaserScan scan;
    scan.header.stamp = msg->header.stamp;
    scan.header.frame_id = output_frame_;     // 通常设 base_link 或 laser
    scan.angle_min = angle_min_;
    scan.angle_max = angle_max_;
    scan.angle_increment = angle_inc_;
    scan.time_increment = 0.0;
    scan.scan_time = 0.0;
    scan.range_min = range_min_;
    scan.range_max = range_max_;

    const int beam_count = static_cast<int>(std::floor((scan.angle_max - scan.angle_min) / scan.angle_increment)) + 1;
    scan.ranges.assign(beam_count, std::numeric_limits<float>::infinity());

    // 4) 遍历点云，投影到2D并按角度取最近点
    const int N = static_cast<int>(cloud_in.points.size());
    for (int i = 0; i < N; i += stride_) {
      const auto& p = cloud_in.points[i];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

      // 变换到 target_frame_
      Eigen::Vector3d pt_src(p.x, p.y, p.z);
      Eigen::Vector3d pt = T * pt_src;

      const double x = pt.x();
      const double y = pt.y();
      const double z = pt.z();

      // Z过滤（压扁成2D常用）
      if (z < z_min_ || z > z_max_) continue;

      const double r = std::hypot(x, y);
      if (r < range_min_ || r > range_max_) continue;

      const double ang = std::atan2(y, x);

      if (use_angle_limits_) {
        if (ang < scan.angle_min || ang > scan.angle_max) continue;
      }

      int idx = static_cast<int>(std::floor((ang - scan.angle_min) / scan.angle_increment));
      if (idx < 0 || idx >= beam_count) continue;

      // 取最近点（更像真实2D雷达）
      if (r < scan.ranges[idx]) {
        scan.ranges[idx] = static_cast<float>(r);
      }
    }

    // 5) 无回波填充：按LaserScan习惯用 inf（RViz也能正常显示）
    scan_pub_.publish(scan);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Publisher scan_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string cloud_topic_, scan_topic_;
  std::string target_frame_, output_frame_;

  double angle_min_, angle_max_, angle_inc_;
  double range_min_, range_max_;
  double z_min_, z_max_;
  bool use_angle_limits_;
  int stride_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_to_scan_node");
  CloudToScan node;
  ros::spin();
  return 0;
}
