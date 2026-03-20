/*
 * @Author: your name
 * @Date: 2023-03-22 17:18:22
 * @LastEditTime: 2023-09-12 10:59:09
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/read_laser_data/src/laser_carto_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
/*
 * Copyright 2020 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// 声明一个类
class LaserScan
{
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher cartoLaserTopic;         // 发布的carto的雷达话题
    sensor_msgs::LaserScan cartoLaserData;
    std::string laserType;     //激光雷达话题

public:
    LaserScan();
    ~LaserScan();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};

// 构造函数
LaserScan::LaserScan() : private_node_("~")
{
  ROS_INFO_STREAM("LaserScan initial.");

  // 获取激光雷达话题 默认是gazebo仿真中的雷达话题消息
  private_node_.param<std::string>("laserType",laserType,"laser_scan");

  // <!-- #设置激光数据topic名称 -->
  // <!-- <param name="scan_topic" value="scan"/>   -->
  // <!-- #激光坐标          -->
  // <!-- <param name="frame_id" value="laser_link"/>       -->
  // 话题发布
  cartoLaserTopic = node_handle_.advertise<sensor_msgs::LaserScan>("scan",1);

  // 将雷达的回调函数与订阅的topic进行绑定
  laser_scan_subscriber_ = node_handle_.subscribe(laserType, 1, &LaserScan::ScanCallback, this);
}

LaserScan::~LaserScan()
{
}


// 回调函数
void LaserScan::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{  
  cartoLaserData = *scan_msg;
  cartoLaserData.header.frame_id = "laser_link";
  cartoLaserTopic.publish(cartoLaserData);
  // 输出标志为，雷达一切正常
  ROS_INFO("laser OK!");


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_carto_node"); // 节点的名字
    LaserScan laser_scan;

    ros::spin();    // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}