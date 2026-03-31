#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

class OdomFromTruth
{
public:
  OdomFromTruth()
  {
    ros::NodeHandle nh;

    // 订阅 Gazebo 真值位姿
    truth_sub_ = nh.subscribe("/truthPose", 10,
                              &OdomFromTruth::truthCallback, this);

    // 发布 odom
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
  }

  void truthCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    ros::Time current_time = msg->header.stamp;
    if (current_time.isZero())
      current_time = ros::Time::now();

    /* ===== 发布 TF: odom -> base_footprint ===== */
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";

    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = 0.0;  // 平面机器人

    tf_msg.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_.sendTransform(tf_msg);

    /* ===== 发布 /odom ===== */
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose = msg->pose.pose;
    odom.twist.twist = msg->twist.twist;

    odom_pub_.publish(odom);
  }

private:
  ros::Subscriber truth_sub_;
  ros::Publisher  odom_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_from_gazebo");
  OdomFromTruth node;
  ros::spin();
  return 0;
}