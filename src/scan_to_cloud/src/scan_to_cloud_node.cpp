#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <laser_geometry/laser_geometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class ScanToCloudNode
{
public:
  ScanToCloudNode()
  : nh_("~"),
    tf_listener_(tf_buffer_)
  {
    nh_.param<std::string>("scan_topic",  scan_topic_,  "/scan");
    nh_.param<std::string>("cloud_topic", cloud_topic_, "/cloud_in");

    // 输出点云的目标坐标系：一般给 costmap 用 base_link 或 odom
    nh_.param<std::string>("target_frame", target_frame_, "base_link");

    // TF 查不到时，是否用最新 TF（Time(0)）兜底
    nh_.param<bool>("use_latest_tf", use_latest_tf_, true);

    sub_ = nh_.subscribe(scan_topic_, 10, &ScanToCloudNode::cb, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>(cloud_topic_, 1);

    ROS_INFO("[scan_to_cloud] scan=%s -> cloud=%s target_frame=%s",
             scan_topic_.c_str(), cloud_topic_.c_str(), target_frame_.c_str());
  }

private:
  void cb(const sensor_msgs::LaserScanConstPtr& scan)
  {
    sensor_msgs::PointCloud2 cloud;

    try {
      // 优先用 scan 的时间戳（时间同步更好）
      if (!use_latest_tf_) {
        projector_.transformLaserScanToPointCloud(target_frame_, *scan, cloud, tf_buffer_);
      } else {
        // 兜底：用最新 TF（对 TF 不够稳定的系统更友好）
        sensor_msgs::LaserScan scan2 = *scan;
        scan2.header.stamp = ros::Time(0);
        projector_.transformLaserScanToPointCloud(target_frame_, scan2, cloud, tf_buffer_);
        cloud.header.stamp = scan->header.stamp; // 发布时仍保留原时间
      }

      cloud.header.frame_id = target_frame_;
      pub_.publish(cloud);

    } catch (const std::exception& e) {
      ROS_WARN_THROTTLE(1.0, "[scan_to_cloud] transform failed: %s", e.what());
    }
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;

  laser_geometry::LaserProjection projector_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string scan_topic_, cloud_topic_, target_frame_;
  bool use_latest_tf_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_to_cloud_node");
  ScanToCloudNode node;
  ros::spin();
  return 0;
}
