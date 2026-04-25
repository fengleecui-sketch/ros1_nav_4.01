#ifndef __CORRIDOR_OPTIMIZER_H
#define __CORRIDOR_OPTIMIZER_H

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <Eigen/Eigen>

// Boost Geometry
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/linestring.hpp>

// OSQP-Eigen
#include <OsqpEigen/OsqpEigen.h>

// ROS 可视化
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;
namespace bg = boost::geometry;
using Point_bg = bg::model::d2::point_xy<double>;
using Polygon_bg = bg::model::polygon<Point_bg>;
using Linestring_bg = bg::model::linestring<Point_bg>;

class Corridor_Optimizer
{
private:
    // 地图参数
    double resolution_;
    double origin_x_;
    double origin_y_;
    int width_;
    int height_;
    std::vector<int> map_data_;

    // 优化参数
    double max_corridor_width_;
    double extend_length_;
    double safe_margin_;

    // 发布器 (用于RViz可视化安全走廊)
    ros::Publisher corridor_pub_;

    // 内部功能函数 (私有)
    vector<Vector2d> simplifyPath(const vector<Vector2d>& path, double corner_deg, int corner_dilate);
    vector<array<double, 8>> convexCorridor(const vector<Vector2d>& path_xy);
    vector<Polygon_bg> generateCorridorsOptimized(const vector<Vector2d>& path_xy, vector<array<double, 8>>& rects);
    vector<Vector2d> minimumSnapSolver(const vector<Polygon_bg>& corridor, const vector<Vector2d>& path, int N, int dim, double lambda_center);
    
    // 几何工具函数
    bool pointInPolygon(const Polygon_bg &poly, double x, double y);
    vector<Polygon_bg> splitPolygonByLine(const Polygon_bg &poly, const pair<double,double>& p1, const pair<double,double>& p2);

    // 坐标转换
    Vector2i worldToMap(const Vector2d& world_pt);
    Vector2d mapToWorld(const Vector2i& grid_pt);
    bool isOccupied(int x, int y);
    private:
  // 地图成员变量（修复 grid_ 未定义）
  std::vector<std::vector<int>> grid_;

  // 缺失的成员函数声明（修复所有 no declaration matches 错误）
  void world2Grid(const Eigen::Vector2d& world, int& gx, int& gy);
  Eigen::Vector2d grid2World(int gx, int gy);
  std::vector<double> computeAngles(const std::vector<Eigen::Vector2d>& points);
  Polygon_bg rectToPolygon(const std::array<double, 8>& rect);
  std::pair<double, double> polygonCentroid(const Polygon_bg& poly);
  std::vector<std::pair<int, int>> polygonObstaclesInSubgrid(const Polygon_bg& poly, int min_x, int min_y);
  Eigen::MatrixXd makeDiffMatrix(int order, int N);
  void polygonToInequalities(const Polygon_bg& poly, Eigen::MatrixXd& A, Eigen::VectorXd& b);

public:
    typedef shared_ptr<Corridor_Optimizer> Ptr;

    Corridor_Optimizer();
    ~Corridor_Optimizer();

    // 初始化 ROS 参数和 Publisher
    void InitParams(ros::NodeHandle &nh);

    // 每次规划前更新地图数据
    void updateMap(double res, double ox, double oy, int w, int h, const std::vector<int>& map);

    // 核心接口：输入 A* 原始路径（世界坐标），输出优化后的 Minimum Snap 路径（世界坐标）
    vector<Vector2d> optimizePathWithCorridor(const vector<Vector2d>& raw_world_path);

    // 发布安全走廊用于 RViz 显示
    void visualCorridorsPolygons(const vector<Polygon_bg>& corridors);
};

#endif