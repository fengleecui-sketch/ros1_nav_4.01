#include "path_optimization/corridor_optimizer.h"
#include <cmath>
#include <algorithm>
#include <set>
#include <unordered_map>
#include <queue>

// 构造/析构函数
Corridor_Optimizer::Corridor_Optimizer()
    : resolution_(0.05), origin_x_(0), origin_y_(0), width_(0), height_(0),
      max_corridor_width_(7.0), extend_length_(8.0) {}

Corridor_Optimizer::~Corridor_Optimizer() {}

// 初始化ROS参数
void Corridor_Optimizer::InitParams(ros::NodeHandle &nh)
{
    // 从参数服务器读取配置
    nh.param("corridor/max_width", max_corridor_width_, 7.0);
    nh.param("corridor/extend_length", extend_length_, 8.0);

    // 初始化走廊可视化发布器
    corridor_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/Safety_Corridor_Polygons", 10);
    ROS_INFO("安全走廊优化器初始化完成");
}

// 更新地图数据
void Corridor_Optimizer::updateMap(double res, double ox, double oy, int w, int h, const std::vector<int>& map)
{
    resolution_ = res;
    origin_x_ = ox;
    origin_y_ = oy;
    width_ = w;
    height_ = h;
    map_data_ = map;

    // 一维ROS地图 → 二维算法栅格
    grid_.resize(height_, std::vector<int>(width_, 0));
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            int idx = y * width_ + x;
            if (map_data_[idx] > 50) {
                grid_[y][x] = 50; // 障碍物标记
            } else {
                grid_[y][x] = 0;
            }
        }
    }
    ROS_INFO("地图更新完成：%d x %d", width_, height_);
}

// 坐标转换：世界→栅格
void Corridor_Optimizer::world2Grid(const Vector2d& world, int& gx, int& gy)
{
    gx = (int)((world.x() - origin_x_) / resolution_);
    gy = (int)((world.y() - origin_y_) / resolution_);
    gx = std::max(0, std::min(width_ - 1, gx));
    gy = std::max(0, std::min(height_ - 1, gy));
}

// 坐标转换：栅格→世界
Vector2d Corridor_Optimizer::grid2World(int gx, int gy)
{
    double x = gx * resolution_ + origin_x_;
    double y = gy * resolution_ + origin_y_;
    return Vector2d(x, y);
}

/**********************************************************************
 * 对外核心接口：路径优化（安全走廊 + 最小Snap）
 *********************************************************************/
std::vector<Vector2d> Corridor_Optimizer::optimizePathWithCorridor(const std::vector<Vector2d>& raw_world_path)
{
    if (raw_world_path.size() < 3) {
        ROS_WARN("Path too short (< 3 points), skipping corridor generation!");
        return raw_world_path;
    }

    // 1. 统一战线：将世界坐标 (米) 转换为栅格坐标 (像素)
    std::vector<Vector2d> grid_path;
    for (const auto& pt : raw_world_path) {
        double gx = (pt.x() - origin_x_) / resolution_;
        double gy = (pt.y() - origin_y_) / resolution_;
        grid_path.push_back(Vector2d(gx, gy));
    }

    // 2. 路径简化 (在栅格坐标系下)
    std::vector<Vector2d> simplified_path = simplifyPath(grid_path, 30.0, 1);

    // 3. 生成凸走廊并切割障碍物 (此时坐标系终于对齐了)
    std::vector<std::array<double, 8>> rects = convexCorridor(simplified_path);
    std::vector<Polygon_bg> corridors = generateCorridorsOptimized(simplified_path, rects);

    if (corridors.empty()) {
        ROS_WARN("安全走廊生成失败（空间太小），退回 A* 原始路径！");
        return raw_world_path;
    }

    // 4. 可视化 (函数内部会自动把栅格再转回米，显示在 RViz 上)
    visualCorridorsPolygons(corridors);

    // 5. 最小Snap优化 (在栅格坐标下做纯数学优化)
    std::vector<Vector2d> traj_grid = minimumSnapSolver(corridors, simplified_path, simplified_path.size(), 2, 2.);

    if (traj_grid.empty()) {
        ROS_WARN("OSQP 轨迹平滑优化失败，退回 A* 原始路径！");
        return raw_world_path;
    }

    // 6. 大功告成：将优化好的轨迹转回世界坐标 (米) 交给机器人执行
    std::vector<Vector2d> traj_world;
    for(const auto& pt : traj_grid) {
        double wx = pt.x() * resolution_ + origin_x_;
        double wy = pt.y() * resolution_ + origin_y_;
        traj_world.push_back(Vector2d(wx, wy));
    }

    ROS_INFO("Optimization complete! Trajectory has %zu points.", traj_world.size());
    return traj_world;
}


/**********************************************************************
 * 以下为完整算法实现（从原.cpp迁移，封装为类成员函数）
 *********************************************************************/
// 路径简化
std::vector<Vector2d> Corridor_Optimizer::simplifyPath(const std::vector<Vector2d>& points, double corner_deg, int corner_dilate)
{
    int n = points.size();
    if (n <= 2) return points;

    std::vector<double> angles = computeAngles(points);
    double theta = corner_deg * M_PI / 180.0;
    std::vector<int> corner_idx;

    for (int i = 0; i < n; ++i) {
        if (i == 0 || i == n - 1 || angles[i] >= theta) {
            corner_idx.push_back(i);
        }
    }

    if (corner_dilate > 0) {
        std::set<int> expanded;
        for (int idx : corner_idx) {
            for (int j = idx - corner_dilate; j <= idx + corner_dilate; ++j) {
                if (j >= 0 && j < n) expanded.insert(j);
            }
        }
        corner_idx.assign(expanded.begin(), expanded.end());
    }

    std::vector<Vector2d> out;
    for (int idx : corner_idx) out.push_back(points[idx]);
    return out;
}

// 计算夹角
std::vector<double> Corridor_Optimizer::computeAngles(const std::vector<Vector2d>& points)
{
    int n = points.size();
    std::vector<double> angles(n, M_PI);
    if (n < 3) return angles;

    for (int i = 1; i < n - 1; ++i) {
        Vector2d v1 = points[i] - points[i-1];
        Vector2d v2 = points[i+1] - points[i];
        double n1 = v1.norm(), n2 = v2.norm();
        if (n1 < 1e-12 || n2 < 1e-12) { angles[i] = 0; continue; }
        double cos_angle = v1.dot(v2)/(n1*n2);
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        angles[i] = acos(cos_angle);
    }
    return angles;
}

// 生成凸走廊矩形
std::vector<std::array<double, 8>> Corridor_Optimizer::convexCorridor(const std::vector<Vector2d>& path_xy)
{
    std::vector<std::array<double, 8>> rects;
    for (size_t i=0; i+1 < path_xy.size(); ++i) {
        Vector2d p1 = path_xy[i];
        Vector2d p2 = path_xy[i+1];
        Vector2d seg = p2 - p1;
        double seg_len = seg.norm();
        std::array<double, 8> corners;

        if (seg_len < 1e-6) {
            double half = max_corridor_width_*0.5;
            corners = {p1.x()-half, p1.y()-half, p1.x()+half, p1.y()-half, p1.x()+half, p1.y()+half, p1.x()-half, p1.y()+half};
        } else {
            Vector2d unit = seg/seg_len;
            Vector2d orth_unit(-unit.y(), unit.x());
            double half_w = max_corridor_width_;
            Vector2d c1 = p1 + orth_unit*half_w - unit*extend_length_;
            Vector2d c2 = p1 - orth_unit*half_w - unit*extend_length_;
            Vector2d c3 = p2 - orth_unit*half_w + unit*extend_length_;
            Vector2d c4 = p2 + orth_unit*half_w + unit*extend_length_;

            auto clamp = [&](double v, double min, double max) {
                return std::min(std::max(v, min), max);
            };
            corners = {clamp(c1.x(),0,width_-1), clamp(c1.y(),0,height_-1),
                       clamp(c2.x(),0,width_-1), clamp(c2.y(),0,height_-1),
                       clamp(c3.x(),0,width_-1), clamp(c3.y(),0,height_-1),
                       clamp(c4.x(),0,width_-1), clamp(c4.y(),0,height_-1)};
        }
        rects.push_back(corners);
    }
    return rects;
}

// 矩形转多边形
Polygon_bg Corridor_Optimizer::rectToPolygon(const std::array<double, 8>& rect)
{
    Polygon_bg poly;
    std::vector<Point_bg> pts;
    pts.emplace_back(rect[0], rect[1]);
    pts.emplace_back(rect[2], rect[3]);
    pts.emplace_back(rect[4], rect[5]);
    pts.emplace_back(rect[6], rect[7]);
    pts.emplace_back(pts.front());
    bg::assign_points(poly, pts);
    bg::correct(poly);
    return poly;
}

// 点在多边形内
bool Corridor_Optimizer::pointInPolygon(const Polygon_bg& poly, double x, double y)
{
    Point_bg p(x,y);
    return bg::covered_by(p, poly);
}

// 多边形质心
std::pair<double, double> Corridor_Optimizer::polygonCentroid(const Polygon_bg& poly)
{
    if (bg::area(poly) == 0) {
        double sx=0,sy=0; int cnt=0;
        for (auto& pt : poly.outer()) { sx+=bg::get<0>(pt); sy+=bg::get<1>(pt); cnt++; }
        return cnt==0 ? std::make_pair(0.0, 0.0) : std::make_pair(sx/cnt, sy/cnt);
    }
    Point_bg c; bg::centroid(poly, c);
    return {bg::get<0>(c), bg::get<1>(c)};
}

// 切割多边形
std::vector<Polygon_bg> Corridor_Optimizer::splitPolygonByLine(const Polygon_bg& poly, const std::pair<double, double>& p1, const std::pair<double, double>& p2)
{
    Linestring_bg cut;
    double dx = p2.first-p1.first, dy=p2.second-p1.second;
    double len = hypot(dx,dy);
    if (len < 1e-9) return {poly};

    double ux = dx/len, uy=dy/len;
    cut.push_back(Point_bg(p1.first-ux*1e4, p1.second-uy*1e4));
    cut.push_back(Point_bg(p1.first+ux*1e4, p1.second+uy*1e4));

    double half_w = 1e-3;
    double ox = -uy, oy=ux;
    Polygon_bg band;
    std::vector<Point_bg> outer;
    outer.emplace_back(p1.first-ux*1e4 + ox*half_w, p1.second-uy*1e4 + oy*half_w);
    outer.emplace_back(p1.first+ux*1e4 + ox*half_w, p1.second+uy*1e4 + oy*half_w);
    outer.emplace_back(p1.first+ux*1e4 - ox*half_w, p1.second+uy*1e4 - oy*half_w);
    outer.emplace_back(p1.first-ux*1e4 - ox*half_w, p1.second-uy*1e4 - oy*half_w);
    outer.emplace_back(outer.front());
    bg::assign_points(band, outer);
    bg::correct(band);

    std::vector<Polygon_bg> res;
    try { bg::difference(poly, band, res); }
    catch(...) { return {poly}; }

    std::vector<Polygon_bg> out;
    for (auto& p : res) if (fabs(bg::area(p))>1e-9) out.push_back(p);
    return out.empty() ? std::vector<Polygon_bg>{poly} : out;
}

// 获取多边形内障碍物
std::vector<std::pair<int, int>> Corridor_Optimizer::polygonObstaclesInSubgrid(const Polygon_bg& poly, int min_x, int min_y)
{
    std::vector<std::pair<int, int>> obs;
    for (int r=0; r<height_; r++) {
        for (int c=0; c<width_; c++) {
            if (grid_[r][c] != 50) continue;
            double x = c+0.5, y = r+0.5;
            if (pointInPolygon(poly, x, y)) {
                obs.emplace_back(r, c);
            }
        }
    }
    return obs;
}

// 优化生成走廊 (修复死循环与精度Bug版)
std::vector<Polygon_bg> Corridor_Optimizer::generateCorridorsOptimized(const std::vector<Vector2d>& path_xy, std::vector<std::array<double, 8>>& corridor_rects)
{
    std::vector<Polygon_bg> corridor;
    for (auto& r : corridor_rects) corridor.push_back(rectToPolygon(r));

    for (size_t i = 0; i < corridor.size(); i++) {
        Polygon_bg cur = corridor[i];
        auto obs = polygonObstaclesInSubgrid(cur, 0, 0);
        if (obs.empty()) continue;

        std::vector<std::pair<int,int>> obs_cells = obs;
        int safe_counter = 0; 

        while (!obs_cells.empty()) {
            safe_counter++;
            // 降低容忍度， fail-fast (快速失败)，不要卡住主线程！
            if (safe_counter > 5) { 
                ROS_DEBUG("走廊切割次数过多，跳出当前走廊处理...");
                break; 
            }

            auto rc = obs_cells.front();
            int r = rc.first, c = rc.second;

            double mx = (path_xy[i].x() + path_xy[i+1].x()) / 2.0;
            double my = (path_xy[i].y() + path_xy[i+1].y()) / 2.0;
            
            auto pieces = splitPolygonByLine(cur, {mx, my}, {(double)c, (double)r});

            // ================= 【核心修复：几何精度崩溃拦截】 =================
            // 如果切完之后，多边形没有变成 2 块（切割失败，可能是切线没碰到多边形）
            if (pieces.size() <= 1) {
                // 这个障碍物因为精度问题根本切不掉，继续切就会死循环！
                // 解决办法：强行把它从待处理名单里踢出去，去切下一个障碍物。
                obs_cells.erase(obs_cells.begin()); 
                continue; 
            }
            // =================================================================

            Polygon_bg keep = cur;
            for (auto& p : pieces) {
                if (pointInPolygon(p, mx, my)) { keep = p; break; }
            }
            cur = keep;
            corridor[i] = cur;
            obs_cells = polygonObstaclesInSubgrid(cur, 0, 0);
        }
    }
    return corridor;
}

// 差分矩阵
MatrixXd Corridor_Optimizer::makeDiffMatrix(int order, int N)
{
    if (order >= N) return MatrixXd::Zero(0,N);
    MatrixXd S(N-order, N);
    std::vector<int> coeff = {1};
    for (int k=0; k<order; k++) {
        std::vector<int> nc(coeff.size()+1);
        for (size_t i=0; i<coeff.size(); i++) { nc[i]+=coeff[i]; nc[i+1]-=coeff[i]; }
        coeff.swap(nc);
    }
    for (int i=0; i<N-order; i++) {
        for (size_t j=0; j<coeff.size(); j++) S(i,i+j) = coeff[j];
    }
    return S;
}

// 多边形转不等式约束
void Corridor_Optimizer::polygonToInequalities(const Polygon_bg& poly, MatrixXd& A, VectorXd& b)
{
    auto& pts = poly.outer();
    int n = pts.size()-1;
    A.resize(n,2); b.resize(n);
    auto cen = polygonCentroid(poly);

    for (int i=0; i<n; i++) {
        double x0 = bg::get<0>(pts[i]), y0=bg::get<1>(pts[i]);
        double x1 = bg::get<0>(pts[i+1]), y1=bg::get<1>(pts[i+1]);
        Vector2d normal(y1-y0, x0-x1);
        Vector2d cv(cen.first-x0, cen.second-y0);
        if (normal.dot(cv) > 0) normal = -normal;
        A(i,0)=normal.x(); A(i,1)=normal.y();
        b(i) = normal.x()*x0 + normal.y()*y0;
    }
}

// 最小Snap求解器
std::vector<Vector2d> Corridor_Optimizer::minimumSnapSolver(const std::vector<Polygon_bg>& corridor, const std::vector<Vector2d>& path, int N, int dim, double lambda_center)
{
    MatrixXd S = makeDiffMatrix(4, N);
    int varN = N*dim;
    MatrixXd Qt = MatrixXd::Zero(varN, varN);
    VectorXd c = VectorXd::Zero(varN);
    MatrixXd STS = S.transpose() * S;

    // 构建目标函数
    for (int i=0; i<N; i++) {
        int xi = i*2, yi=i*2+1;
        for (int j=0; j<N; j++) {
            Qt(xi, j*2) += STS(i,j);
            Qt(yi, j*2+1) += STS(i,j);
        }
        Qt(xi,xi) += lambda_center;
        Qt(yi,yi) += lambda_center;
        c(xi) += -2*lambda_center*path[i].x();
        c(yi) += -2*lambda_center*path[i].y();
    }

    MatrixXd P = 2 * Qt;
    VectorXd q = c;

    // 约束构建
    std::vector<VectorXd> Arows;
    std::vector<double> brows;

    // 起点终点约束
    VectorXd row0 = VectorXd::Zero(varN); row0(0)=1; Arows.push_back(row0); brows.push_back(path[0].x());
    VectorXd row1 = VectorXd::Zero(varN); row1(1)=1; Arows.push_back(row1); brows.push_back(path[0].y());
    VectorXd row2 = VectorXd::Zero(varN); row2((N-1)*2)=1; Arows.push_back(row2); brows.push_back(path.back().x());
    VectorXd row3 = VectorXd::Zero(varN); row3((N-1)*2+1)=1; Arows.push_back(row3); brows.push_back(path.back().y());

    // 走廊约束
    for (int i=0; i<(int)corridor.size(); i++) {
        MatrixXd A; VectorXd b;
        polygonToInequalities(corridor[i], A, b);
        for (int r=0; r<A.rows(); r++) {
            VectorXd ar = VectorXd::Zero(varN);
            ar(i*2) = A(r,0); ar(i*2+1)=A(r,1);
            Arows.push_back(ar); brows.push_back(b(r)+1e-6);
        }
    }

    // OSQP求解
    int m = Arows.size();
    MatrixXd Aineq(m, varN);
    VectorXd lb(m), ub(m);
    for (int i=0; i<m; i++) {
        Aineq.row(i) = Arows[i];
        ub(i) = brows[i];
        lb(i) = -1e20;
    }
    for (int i=0; i<4; i++) lb(i)=ub(i);

    OsqpEigen::Solver solver;
    // 转为 OSQP 支持的稀疏矩阵格式
    Eigen::SparseMatrix<double> P_sparse = P.sparseView();
    Eigen::SparseMatrix<double> A_sparse = Aineq.sparseView();

    // 1. 先设置变量和约束的数量（顺序极其重要）
    solver.data()->setNumberOfVariables(varN);
    solver.data()->setNumberOfConstraints(m);
    
    // 2. 再塞入矩阵和边界
    solver.data()->setHessianMatrix(P_sparse);
    solver.data()->setLinearConstraintsMatrix(A_sparse);
    solver.data()->setGradient(q);
    solver.data()->setLowerBound(lb);
    solver.data()->setUpperBound(ub);
    
    solver.settings()->setVerbosity(false);

    // 3. 增加求解失败时的保护，直接返回空路径，阻止闪退
    if (!solver.initSolver()) {
        ROS_ERROR("OSQP init failed! 走廊约束矩阵无效（可能太贴墙）！");
        return {}; // 返回空数组
    }

    if (static_cast<int>(solver.solve()) != 0) {
        ROS_ERROR("OSQP solve failed! 无法在当前走廊内解出平滑曲线！");
        return {}; // 返回空数组
    }

    // 4. 成功求解，提取轨迹
    VectorXd x = solver.getSolution();
    std::vector<Vector2d> traj;
    for (int i=0; i<N; i++) {
        traj.emplace_back(x(i*2), x(i*2+1));
    }
    
    return traj; // 干净利落地返回
} 

// 走廊可视化
void Corridor_Optimizer::visualCorridorsPolygons(const std::vector<Polygon_bg>& corridors)
{
    visualization_msgs::MarkerArray msg;
    for (size_t i=0; i<corridors.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "safe_corridor";
        marker.id = i;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;

        //设置初始位姿
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05; // 线宽
        marker.color.r = 1.0; marker.color.g = 0.6; marker.color.b = 0.0; marker.color.a = 0.9;

        for (auto& pt : corridors[i].outer()) {
            geometry_msgs::Point p;
            p.x = bg::get<0>(pt) * resolution_ + origin_x_;
            p.y = bg::get<1>(pt) * resolution_ + origin_y_;
            p.z = 0.1;
            marker.points.push_back(p);
        }
        msg.markers.push_back(marker);
    }
    corridor_pub_.publish(msg);
}