/*
 * @Author: your name
 * @Date: 2023-07-03 11:54:03
 * @LastEditTime: 2023-09-20 17:08:20
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_locatization/src/cartographer_local/carto_locatization.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "cartographer_local/carto_locatization.h"

carto_locatization::carto_locatization(/* args */) : private_node("~")
{
  // 发布里程计
  cartOdomPub = cartoLocatization.advertise<robot_communication::localizationInfoBroadcast>("/odom_carto",10);
  // 发布里程计 给move base 构建代价地图
  OdomPub = cartoLocatization.advertise<nav_msgs::Odometry>("/aclodom",10);
  // 发布位置消息
  cartoLocalPub = cartoLocatization.advertise<geometry_msgs::Pose2D>("/pose_data",10);

  // 订阅底盘的速度消息
  chassDataSub = cartoLocatization.subscribe("/chassis_sensor_data",10,&carto_locatization::SensorDataCallback,this);

  // carto的tf变换分布的最快频率是200hz
  ros::Rate LoopRate(200);

  while (ros::ok())
  {
    // 回调函数数据刷新
    ros::spinOnce();


    try{
        //得到坐map和坐标base_link之间的关系
      listener.waitForTransform("map","footprint",  ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("map", "footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    x=transform.getOrigin().x();
    y=transform.getOrigin().y();
    z=transform.getOrigin().z();

    // 获取位置信息
    cartOdom.xPosition = transform.getOrigin().x();
    cartOdom.yPosition = transform.getOrigin().y();

	  tf::Quaternion q = transform.getRotation();
    qx = q.x();
    qy = q.y();
    qz = q.z();
    qw = q.w();

    // 获取位置信息
    pos_now.x = transform.getOrigin().x();
    pos_now.y =transform.getOrigin().y();
    pos_now.theta = tf::getYaw(q);

    cartOdom.chassisAngle = tf::getYaw(q);

    cartOdom.xSpeed = chassisData.local_x_Veloc;
    cartOdom.ySpeed = chassisData.local_y_Veloc;

    cartOdom.xAccel = chassisData.local_x_Accel;
    cartOdom.yAccel = chassisData.local_y_Accel;

    cout<<"xPosition:"<<cartOdom.xPosition<<"  "<<"yPosition:"<<cartOdom.yPosition<<endl;

    // carto定位消息发布
    cartoLocalPub.publish(pos_now);
    // carto里程计消息发布，自定义消息类型
    cartOdomPub.publish(cartOdom);
    
    // 发布里程计消息 用于生成局部代价地图
    odom.child_frame_id = "footprint";
    odom.header.frame_id = "odom";

    odom.pose.pose.position.x = cartOdom.xPosition;
    odom.pose.pose.position.y = cartOdom.yPosition;
    odom.pose.pose.position.z = 0;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // 还没读取出来线速度消息
    odom.twist.twist.linear.x = chassisData.local_x_Veloc;
    odom.twist.twist.linear.y = chassisData.local_y_Veloc;
    odom.twist.twist.linear.z = 0;

    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    // carto里程计消息发布，用于ros包构建局部地图
    OdomPub.publish(odom);

    // printf("x: %f, y: %f, z: %f, qx: %f,qy: %f,qz: %f, qw: %f, theta: %f\n",x,y,z,qx,qy,qz,qw,pos_now.theta);

    LoopRate.sleep();
  }
}

carto_locatization::~carto_locatization()
{
}

void carto_locatization::SensorDataCallback(const robot_communication::sensorDataConstPtr &sensor)
{
  chassisData = *sensor;
  // cout<<"receive!"<<endl;
}

