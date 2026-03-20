#ifndef __NAV_APP_H
#define __NAV_APP_H

#include "iostream"
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "Astar_searcher.h"
#include "JPS_searcher.h"
#include "JPS_utils.h"
#include "two_way_rrt.h"
#include "Hybrid_astar.h"
#include "node.h"
#include "chrono"

#include "path_optimization/minimum_snap.h"
#include "path_optimization/bezier_curve.h"

typedef struct maParams{        //定义地图参数类
    double origin_x=0.0;  //栅格坐标系与世界坐标系的原点X的差值，单位为m
    double origin_y=0.0;  //栅格坐标系与世界坐标系的原点Y的差值，单位为m
    int width=0;     //栅格地图的宽
    int height=0;    //栅格地图的高
    double resolution=0.0;//栅格地图的分辨率
    double start_x;       //定义起点坐标
    double start_y;

    /* 以下参数主要针对图搜索算法进行使用 */
    std::vector<Eigen::Vector2i>  gridpath;  //图搜索算法的路径，栅格地图中的路径
    std::vector<Eigen::Vector2d>  worldpath;      //世界坐标系下的路径
    std::vector<Eigen::Vector2i>  visitNodes;    //最终访问节点数目
    std::vector<Eigen::Vector2d>  visitWorldNodes;    //最终访问节点在世界坐标系下的数目

    std::vector<Eigen::Vector2d>  optpath;  //优化后的路径

    Eigen::Vector2d startMapPoint; // 算法的起点
    Eigen::Vector2d goalMapPoint;  // 算法的目标点

    std::vector<int> mapData;   //用二维数组存储数据
}mapDefine;

// 导航应用类
class navSolution{
    public:
        navSolution(){};
        ~navSolution(){};

        // 重载构造函数
        navSolution(int flag);
    public:
        ros::Publisher PathPub_JPS;      //发布规划路径
        ros::Publisher PathPub_Astar;    //A*路径发布
        ros::Publisher PathPub_RRT;      //RRT路径发布
        ros::Publisher PathPub_Hybrid;   //Hybrid路径发布

        ros::Publisher StartPoint;   //发布起点位置
        ros::Publisher GoalPoint;    //发布终点位置
        ros::Publisher visited_nodes_pub;  //访问节点发布节点
        
        
        ros::Subscriber mapSub;     //订阅栅格地图
        ros::Subscriber startSub;   //订阅起点
        ros::Subscriber goalSub;    //订阅终点

        mapDefine pathNav;          //导航路径

        JPSPathFinder * jps_path_finder = new JPSPathFinder();
        AstarPathFinder * astar_path_finder = new AstarPathFinder();
        RRTstarPlanner * rrt_path_finder = new RRTstarPlanner();
        Hybrid_astar * hybrid_astar_finder = new Hybrid_astar();

        MinimumSnap MinimumSnapFlow;      //使用minimum_snap优化路径
        BEZIER Bezier_Flow;               //使用贝塞尔曲线优化路径
        ros::Publisher trajectory_pub_;     //发布最终优化后的轨迹        
    private:
        int choose;        //true 使用JPS寻路  false使用A*寻路  1是JPS 2是A* 3是全部
        bool is_use_jps;        //是否使用jps
    private:
        /*** 
         * @description: 
         * @param {double} wx
         * @param {double} wy
         * @return {*}
         */
        Eigen::Vector2i WorldToMap(double wx, double wy);      //世界地图向栅格地图转换
        /*** 
         * @description: 
         * @param {double} my
         * @param {double} mx
         * @return {*}
         */
        Eigen::Vector2d MapToWorld(double my, double mx);   //栅格地图向世界地图转换
        /*** 
         * @description: 
         * @param {ConstPtr} &msg   地图消息数据
         * @return {*}
         */
        void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg); //回调函数

        // A*搜索全局路径
        // 优化并发布
        void AstarFindPath(Eigen::Vector2d startMapPoint,Eigen::Vector2d goalMapPoint);

        // rrt搜索全局路径
        // 优化并发布
        void rrtFindPath(Eigen::Vector2d startMapPoint,Eigen::Vector2d goalMapPoint);

        // JPS搜索全局路径
        // 优化并发布
        void JPSFindPath(Eigen::Vector2d startMapPoint,Eigen::Vector2d goalMapPoint);

        // Hybrid搜索全局路径
        void HybridAstarFindPath(Eigen::Vector2d startMapPoint,Eigen::Vector2d goalMapPoint);
        /*** 
         * @description: 
         * @param {ConstPtr} &msg
         * @return {*}
         */
        void StartPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
        /*** 
         * @description: 
         * @param {ConstPtr} &msg
         * @return {*}
         */
        void GoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        /*** 
         * @description: 
         * @param {Publisher} pathPublish   要发布的话题
         * @param {vector<Vector2i>} path   要发布的路径
         * @return {*}
         */
        void PublishPath(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> path);

        /*** 
         * @description: 
         * @param {vector<Vector2i>} visitnodes
         * @param {Publisher} pub
         * @return {*}
         */
        void visual_VisitedNode(ros::Publisher pathPublish, std::vector<Eigen::Vector2d> visitnodes);
        /*** 
         * @description: 
         * @param {Vector2i} start_pt
         * @param {Vector2i} target_pt
         * @return {*}
         */
        void StartFindPath(Eigen::Vector2d startMapPoint,Eigen::Vector2d goalMapPoint);

        /*** 
         * @description: 
         * @param 1 读取出来的地图的参数  一维数组
         * @param 2 转换为一维算法能用的普通一维数组
         * @param 3 地图的x方向尺寸
         * @param 4 地图的y方向尺寸
         * @return {*}
         */
        void NavtfGrid(const nav_msgs::OccupancyGrid::ConstPtr &data,std::vector<int> &map,int x_size,int y_size);

        void mapInit(void); //地图参数初始化

        void testMinimum_Snap(ros::Publisher pathPublish,std::vector<Eigen::Vector2d> path);
        
    public:
        void All(void);
};


#endif


