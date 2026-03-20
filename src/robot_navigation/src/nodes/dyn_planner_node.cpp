/*
 * @Author: your name
 * @Date: 2023-08-18 16:40:41
 * @LastEditTime: 2023-08-18 16:47:22
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/src/nodes/dyn_planner_node.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "motionPlan/planning_fsm.h"

using namespace dyn_planner;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dyn_planner_node");
    ros::NodeHandle node;
    ros::NodeHandle nh("~");

    PlanningFSM fsm;
    fsm.init(nh);

    ros::spin();

    return 0;
}
