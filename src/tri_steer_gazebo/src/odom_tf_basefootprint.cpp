#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "iostream"

using namespace std;

class OdomToBaseFootprint
{
public:
  OdomToBaseFootprint()
  {
    sub_ = nh_.subscribe("/carto_odom", 10,
                         &OdomToBaseFootprint::odomCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    geometry_msgs::TransformStamped tf_msg;

    // 时间戳：优先用 odom 的
    tf_msg.header.stamp =
        msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id  = "base_footprint";

    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;
    tf_msg.transform.rotation      = msg->pose.pose.orientation;

    br_.sendTransform(tf_msg);

    cout<<"发布 odom tf base_footprint"<<endl;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  tf2_ros::TransformBroadcaster br_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_to_basefootprint");
  OdomToBaseFootprint node;
  ros::spin();
  return 0;
}
