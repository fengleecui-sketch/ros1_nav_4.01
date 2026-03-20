/*
 * @Author: your name
 * @Date: 2023-04-04 16:17:21
 * @LastEditTime: 2023-06-13 15:00:10
 * @LastEditors: your name
 * @Description:
 * @FilePath: /Robot_Formation/src/robot_gmapping/include/hector_mapping/hector_mapping.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
/*
 * Copyright 2021 The Project Author: lixiang
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

#ifndef __HECTOR_MAPPING_H_
#define __HECTOR_MAPPING_H_

#include <iostream>
#include <chrono>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "map/GridMap.h"

namespace hector_mapping
{

  class HectorMapping
  {
  private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher map_publisher_;          // 声明一个Publisher
    ros::Publisher map_publisher_metadata_; // 声明一个Publisher
    nav_msgs::OccupancyGrid map_;           // 用来发布map的实体对象

    std::string laserType;      //雷达类型：是仿真雷达还是实际雷达

    bool got_first_scan_;              // 是否获得第一帧scan标志
    double xmin_, xmax_, ymin_, ymax_; // 地图的边界

    bool map_type;                      //地图类型，用来生成局部地图还是另有别的用处

    double resolution_;
    std::string map_frame_;

    Eigen::Vector2i map_size_;
    Eigen::Vector2f offset_;

    std::mutex ros_map_mutex_;
    std::mutex hector_map_mutex_;

    hectorslam::DataContainer laserScan_container_; // hector中保存雷达数据的格式
    hectorslam::GridMap *hector_map_;               // hector格式的地图

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

    void InitParams();
    void ROSLaserScanToDataContainer(const sensor_msgs::LaserScan &scan,
                                     hectorslam::DataContainer &dataContainer,
                                     float resolution);
    void ComputeMap();
    void PublishMap();

  public:
    HectorMapping();
    ~HectorMapping();
    void ScanCallback(const sensor_msgs::LaserScan &scan);
  };

} // end namespace

#endif // LESSON4_HECTOR_MAPPING_H_