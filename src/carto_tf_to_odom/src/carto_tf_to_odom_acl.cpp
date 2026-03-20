#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "carto_odom_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 参数：默认读 odom->base_footprint，发布 /carto_odom
  std::string odom_frame, base_frame, topic_odom;
  double pub_rate;

  pnh.param<std::string>("odom_frame",  odom_frame,  std::string("odom"));
  // 实际中雷达的坐标系名称
  pnh.param<std::string>("base_frame",  base_frame,  std::string("livox_frame"));
  pnh.param<std::string>("topic_odom",  topic_odom,  std::string("/carto_odom"));
  pnh.param<double>("pub_rate", pub_rate, 30.0);

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(topic_odom, 10);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ros::Rate rate(pub_rate);
  while (ros::ok())
  {
    geometry_msgs::TransformStamped tf;
    try
    {
      tf = tf_buffer.lookupTransform(odom_frame, base_frame, ros::Time(0), ros::Duration(0.05));
    }
    catch (const tf2::TransformException& ex)
    {
      ROS_WARN_THROTTLE(1.0, "[carto_odom_publisher] TF lookup failed: %s", ex.what());
      ros::spinOnce();
      rate.sleep();
      continue;
    }

    nav_msgs::Odometry odom;
    odom.header.stamp = tf.header.stamp;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_frame;

    // pose：完全原样照抄 TF 的平移和旋转（包括 z）
    odom.pose.pose.position.x = tf.transform.translation.x;
    odom.pose.pose.position.y = tf.transform.translation.y;
    odom.pose.pose.position.z = tf.transform.translation.z;
    odom.pose.pose.orientation = tf.transform.rotation;

    // twist：TF 不提供速度，这里保持 0（最基础版本）
    odom.twist.twist.linear.x  = 0.0;
    odom.twist.twist.linear.y  = 0.0;
    odom.twist.twist.linear.z  = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    odom_pub.publish(odom);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
