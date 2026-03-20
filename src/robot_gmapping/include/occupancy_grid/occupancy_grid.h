/*
 * @Author: your name
 * @Date: 2023-05-13 21:36:04
 * @LastEditTime: 2023-05-13 21:36:52
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /Robot_Formation/src/robot_gmapping/include/robot_gmapping/occupancy_grid/occupancy_grid.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __OCCUPANCY_GRID_H
#define __OCCUPANCY_GRID_H

#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class OccupancyGrid
{
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::Publisher map_publisher_;          // 声明一个Publisher
    ros::Publisher map_publisher_metadata_; // 声明一个Publisher
    nav_msgs::OccupancyGrid map_;           //用来发布map的实体对象

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

public:
    OccupancyGrid();
    void PublishMap();
};

#endif // __OCCUPANCY_GRID_H

