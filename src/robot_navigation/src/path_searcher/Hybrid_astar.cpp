#include "path_searcher/Hybrid_astar.h"
#include <sstream>

using namespace std;
using namespace Eigen;

// 构造函数
Hybrid_astar::Hybrid_astar()
{
}

Hybrid_astar::~Hybrid_astar()
{
  for (int i = 0; i < allocate_num_; i++)
  {
    delete path_node_pool_[i];
  }
}

// 参数设定
void Hybrid_astar::InitParams(ros::NodeHandle &nh)
{
  nh.param("search/max_tau", max_tau_, -1.0);
  nh.param("search/init_max_tau", init_max_tau_, -1.0);
  nh.param("search/max_vel", max_vel_, -1.0);
  nh.param("search/max_acc", max_acc_, -1.0);
  nh.param("search/w_time", w_time_, -1.0);
  nh.param("search/horizon", horizon_, 10.0);
  nh.param("search/resolution_astar", resolution_, -1.0);
  nh.param("search/time_resolution", time_resolution_, -1.0);
  nh.param("search/lambda_heu", lambda_heu_, -1.0);
  nh.param("search/allocate_num", allocate_num_, 100000);
  nh.param("search/check_num", check_num_, -1);
  nh.param("search/optimistic", optimistic_, true);

  nh.param("search/is_use_esdf",is_use_esdf,true);

  tie_breaker_ = 1.0 + 1.0 / 10000;

  double vel_margin;
  nh.param("search/vel_margin", vel_margin, 0.0);
  max_vel_ += vel_margin;

  /* 对于路径池范围进行初始化，最多路径点数为allocate_num_ */
  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++)
  {
    path_node_pool_[i] = new HybridGridNode;
  }

  phi_ = Eigen::MatrixXd::Identity(4, 4);
  use_node_num_ = 0;
  iter_num_ = 0;
}

// 判断点是否被占据的具体实现形式
// 输入参数栅栏坐标系
inline bool Hybrid_astar::isOccupied(const int &idx, const int &idy) const
{
  return (idx < X_SIZE && idy < Y_SIZE && (data[idx * Y_SIZE + idy] == OBSTACLE));
}

// 判断点是否被占据的具体实现形式
// 输入参数世界坐标系
inline bool Hybrid_astar::isOccupied(const double &dx, const double &dy) const
{
  int idx, idy;
  idx = worldToMap(Vector2d(dx, dy))[0];
  idy = worldToMap(Vector2d(dx, dy))[1];
  return (idx < X_SIZE && idy < Y_SIZE && (data[idx * Y_SIZE + idy] == OBSTACLE));
}

// 检查路径安全性
bool Hybrid_astar::CheckPathSafe(vector<Eigen::Vector2d> path)
{
  // 检测路径上是否存在障碍物
  for (int i = 0; i < path.size(); i++)
  {
    // 判断是否碰撞，以及规划路径最后一个点是否是终点
    if(isOccupied(path[i](0),path[i](1)))
    {
      return true;
    }
  }
  return false;
}

/**
 * @brief 栅格地图坐标系转世界坐标系
 * @param mx   地图坐标x
 * @param my   地图坐标y
 * @param wx   世界坐标x
 * @param wy   世界坐标y
 * @return
 * @attention
 * @todo
 * */
Vector2d Hybrid_astar::mapToWorld(Vector2i mapt) const
{
  double wx, wy;
  wx = origin_x + (mapt[0] + 0.5) * resolution_;
  wy = origin_y + (mapt[1] + 0.5) * resolution_;

  return Vector2d(wx, wy);
}

/**
 * @brief 世界坐标系转栅格地图坐标系
 * @param wx   世界坐标x
 * @param wy   世界坐标y
 * @param mx   地图坐标x
 * @param my   地图坐标y
 * @return
 * @attention
 * @todo
 * */
Vector2i Hybrid_astar::worldToMap(Vector2d worldpt) const
{
  int mx, my;

  mx = (int)(1.0 * (worldpt[0] - origin_x) / resolution_);
  my = (int)(1.0 * (worldpt[1] - origin_y) / resolution_);

  return Vector2i(mx, my);
}

/**
 * @brief Astar初始化(面向用户)
 * @param _resolution 地图分辨率
 * @param _originx   地图实际的起点
 * @param _originy   地图实际的终点
 * @param 地图x方向上的尺寸
 * @param 地图y方向上的尺寸
 * @param 一维的地图数据
 * @return
 * @attention
 * @todo
 * */
void Hybrid_astar::InitMap(double _resolution, double _originx, double _originy,
                           int _x_size, int _y_size, std::vector<int32_t> _mapData)
{
  resolution_ = _resolution;

  Y_SIZE = _y_size;
  X_SIZE = _x_size;

  origin_x = _originx;
  origin_y = _originy;

  // 初始化一个数组,按照XYZ的大小去初始化数组
  data = new int32_t[Y_SIZE * X_SIZE]; // 为将地图转化为8进制栅格地图作准备
  // 内存处理,清空数组
  memset(data, 0, Y_SIZE * X_SIZE * sizeof(int32_t));
  for (int i = 0; i < X_SIZE; i++)
  {
    for (int j = 0; j < Y_SIZE; j++)
    {
      // 设定障碍物
      if (_mapData[i * Y_SIZE + j] == 100)
      {
        data[i * Y_SIZE + j] = OBSTACLE; // data用来储存对应栅格点有没有障碍物
      }
      else{
        data[i * Y_SIZE + j] = _mapData[i * Y_SIZE + j];
      }
      // data[i * Y_SIZE + j] = _mapData[i * Y_SIZE + j];
    }
  }

  // 对地图信息进行赋值
  origin_ = Vector2d(origin_x, origin_y);
  map_size_2d_ = mapToWorld(Vector2i(X_SIZE, Y_SIZE));

  /* ---------- map params ---------- */
  inv_resolution_ = 1.0 / resolution_;
  inv_time_resolution_ = 1.0 / time_resolution_;
}

// 回溯路径
void Hybrid_astar::retrievePath(HybridGridNodePtr end_node)
{
  HybridGridNodePtr cur_node = end_node; // 定义当前节点为路径节点
  // 将结束点添加进来
  path_nodes_.push_back(cur_node);

  // 当父节点不为空
  while (cur_node->cameFrom != NULL)
  {
    // 当前节点
    cur_node = cur_node->cameFrom;
    // 添加父节点
    path_nodes_.push_back(cur_node);
  }

  // 回溯整条路径
  reverse(path_nodes_.begin(), path_nodes_.end());
}

// 获取运动学轨迹
std::vector<Eigen::Vector2d> Hybrid_astar::getKinoTraj(double delta_t)
{
  vector<Vector2d> state_list; // 状态列表

  // 获取搜索到的路径
  HybridGridNodePtr node = path_nodes_.back();

  Matrix<double, 4, 1> x0, xt;

  // 当节点的父节点不为空
  while (node->cameFrom != NULL)
  {
    Vector2d ut = node->input;
    double duration = node->duration;
    // 状态传递
    x0 = node->cameFrom->state;

    for (double t = duration; t >= -1e-5; t -= delta_t)
    {
      // 状态转移
      stateTransit(x0, xt, ut, t);
      // 加入状态列表
      state_list.push_back(xt.head(2));
    }
    node = node->cameFrom;
  }

  // 回溯路径
  reverse(state_list.begin(), state_list.end());

  // 从射线或者说直线中获取路径
  if (is_shot_succ_)
  {
    Vector2d coord;
    VectorXd poly1d, time(4);

    // 进行转换拟合
    for (double t = delta_t; t <= t_shot_; t += delta_t)
    {
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j); // 高次多项式t的幂计算

      // 二维
      for (int dim = 0; dim < 2; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time);
      }
      state_list.push_back(coord);
    }
  }

  // 取最后一个搜索点
  searchEndpoint = state_list.back();
  // state_list.push_back(end_point_);
  return state_list;
}

// 作用从hybrid a star规划的路径中提取关键点,用来进行优化,因为如果使用整条路径
// 回导致运算时间变得很长
// 最后一个3行2列的矩阵是用来输出start_vel_ 起点速度
//                          end_vel_   终点速度
//                          node->input 第一段轨迹的加速度
Eigen::MatrixXd Hybrid_astar::getSamples(double &ts, int &K)
{
  /* ---------- final trajectory time ---------- */
  double T_sum = 0.0;
  if (is_shot_succ_)
    T_sum += t_shot_;

  HybridGridNodePtr node = path_nodes_.back();
  while (node->cameFrom != NULL)
  {
    T_sum += node->duration; // node->duration记录的是以当前节点为终点的前面一小段轨迹的总时长
    node = node->cameFrom;
  }
  // cout << "final time:" << T_sum << endl;

  /* ---------- init for sampling ---------- */
  K = floor(T_sum / ts);
  ts = T_sum / (K + 1); // 将T_sum平分K+1段, 每段为ts, 默认max_vel情况下, ts大约等于0.25s
  // cout << "K:" << K << ", ts:" << ts << endl;

  bool sample_shot_traj = is_shot_succ_;

  Eigen::VectorXd sx(K + 2), sy(K + 2); // 用来存储x, y, z轴的采样点, K+1段, 共K+2个点
  int sample_num = 0;
  node = path_nodes_.back();

  double t;
  if (sample_shot_traj)
    t = t_shot_;
  else
  {
    t = node->duration;
    end_vel_ = node->state.tail(2);
  }

  for (double ti = T_sum; ti > -1e-5; ti -= ts)
  {
    /* ---------- sample shot traj---------- */
    if (sample_shot_traj)
    {
      Vector2d coord;
      VectorXd poly1d, time(4);
      for (int j = 0; j < 4; j++)
        time(j) = pow(t, j);

      for (int dim = 0; dim < 2; dim++)
      {
        poly1d = coef_shot_.row(dim);
        coord(dim) = poly1d.dot(time); // 计算time处的1d坐标
      }

      sx(sample_num) = coord(0), sy(sample_num) = coord(1);
      ++sample_num;
      t -= ts; // t为ShotTraj的时间

      /* end of segment */
      if (t < -1e-5)
      {
        sample_shot_traj = false;
        if (node->cameFrom != NULL)
          t += node->duration;
      }
    }
    /* ---------- sample search traj ---------- */
    else
    {
      Eigen::Matrix<double, 4, 1> x0 = node->cameFrom->state;
      Eigen::Matrix<double, 4, 1> xt;
      Vector2d ut = node->input;

      stateTransit(x0, xt, ut, t);
      sx(sample_num) = xt(0), sy(sample_num) = xt(1);
      ++sample_num;

      t -= ts;

      if (t < -1e-5 && node->cameFrom->cameFrom != NULL) // 当前一小段轨迹已经采样完, 继续采样前一段轨迹
      {
        node = node->cameFrom; // 因为已经预先将轨迹用ts均匀地平分, 所以最后不会出现起点不对齐的情况
        t += node->duration;   // 递归地计算t是为了保持时间轴统一
      }
    }
  }
  /* ---------- return samples ---------- */
  Eigen::MatrixXd samples(2, K + 5);
  samples.block(0, 0, 1, K + 2) = sx.reverse().transpose(); // 计算sx所用的时间轴是逆序, 这里将时间轴调整为正序
  samples.block(1, 0, 1, K + 2) = sy.reverse().transpose();
  samples.col(K + 2) = start_vel_;
  samples.col(K + 3) = end_vel_;
  samples.col(K + 4) = node->input; // 第一段轨迹的输入加速度

  return samples;
}

// 启发式函数
double Hybrid_astar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double &optimal_time)
{
  // 在 x y坐标上的位置差值
  const Vector2d dp = x2.head(2) - x1.head(2);
  // 在 x1处的速度向量
  const Vector2d v0 = x1.segment(2, 2);
  // 在 x2处的速度向量
  const Vector2d v1 = x2.segment(2, 2);

  Vector2i gridMap = worldToMap(x1.head(2));

  //   c1表示四次多项式的x^4项的系数。
  // c2表示四次多项式的x^3项的系数。
  // c3表示四次多项式的x^2项的系数。
  // c4表示四次多项式的x项的系数，为0。
  // c5表示四次多项式的常数项系数，其中w_time_是一个权重，用于调节时间代价的重要性。
  double c1 = -36 * dp.dot(dp);
  double c2 = 24 * (v0 + v1).dot(dp);
  double c3 = -4 * (v0.dot(v0) + v1.dot(v1));
  double c4 = (double)data[gridMap(0)*Y_SIZE + gridMap(1)];
  double c5 = w_time_;

  // 求解四次方程的根
  std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

  double v_max = max_vel_ * 0.5;
  double t_bar = (x1.head(2) - x2.head(2)).lpNorm<Infinity>() / v_max;
  ts.push_back(t_bar);

  double cost = 100000000;
  double t_d = t_bar;

  for (auto t : ts)
  {
    if (t < t_bar)
      continue;
    double c;
    // 在使用人工势场的时候
    if(is_use_esdf == true)
    {
      c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t + c4;
    }
    // 在不使用的时候
    else
    {
      c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
    }
    if (c < cost)
    {
      cost = c;
      t_d = t;
    }
  }

  optimal_time = t_d;

  return 1.0 * (1 + tie_breaker_) * cost;
}

// 用于求解四次多项式方程的解
vector<double> Hybrid_astar::quartic(double a, double b, double c, double d, double e)
{
  vector<double> dts;

  double a3 = b / a;
  double a2 = c / a;
  double a1 = d / a;
  double a0 = e / a;

  vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
  double y1 = ys.front();
  double r = a3 * a3 / 4 - a2 + y1;
  if (r < 0)
    return dts;

  double R = sqrt(r);
  double D, E;
  if (R != 0)
  {
    D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
  }
  else
  {
    D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
    E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
  }

  if (!std::isnan(D))
  {
    dts.push_back(-a3 / 4 + R / 2 + D / 2);
    dts.push_back(-a3 / 4 + R / 2 - D / 2);
  }
  if (!std::isnan(E))
  {
    dts.push_back(-a3 / 4 - R / 2 + E / 2);
    dts.push_back(-a3 / 4 - R / 2 - E / 2);
  }

  return dts;
}

// 用于求解三次方程的函数
vector<double> Hybrid_astar::cubic(double a, double b, double c, double d)
{
  vector<double> dts;

  double a2 = b / a;
  double a1 = c / a;
  double a0 = d / a;

  double Q = (3 * a1 - a2 * a2) / 9;
  double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
  double D = Q * Q * Q + R * R;
  if (D > 0)
  {
    double S = std::cbrt(R + sqrt(D));
    double T = std::cbrt(R - sqrt(D));
    dts.push_back(-a2 / 3 + (S + T));
    return dts;
  }
  else if (D == 0)
  {
    double S = std::cbrt(R);
    dts.push_back(-a2 / 3 + S + S);
    dts.push_back(-a2 / 3 - S);
    return dts;
  }
  else
  {
    double theta = acos(R / sqrt(-Q * Q * Q));
    dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
    dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
    return dts;
  }
}

void Hybrid_astar::reset(void)
{
  // 扩展节点清空
  expanded_nodes_.clear();
  path_nodes_.clear();

  // 设定空的容器
  std::priority_queue<HybridGridNodePtr, std::vector<HybridGridNodePtr>, NodeComparator> empty_queue;
  open_set_.swap(empty_queue); // 可搜索列表清空

  // 小于已经使用节点数目
  for (int i = 0; i < use_node_num_; i++)
  {
    HybridGridNodePtr node = path_node_pool_[i];
    node->cameFrom = NULL;         // 父节点清空
    node->node_state = NOT_EXPAND; // 状态未扩展
  }

  use_node_num_ = 0;
  iter_num_ = 0;
  is_shot_succ_ = false;
  has_path_ = false;
}
// 将时间转换为整形
int Hybrid_astar::timeToIndex(double time)
{
  int idx = floor((time - time_origin_) * inv_time_resolution_);
  return idx;
}

// 搜索函数
// start_pt 起点位置
// start_v 起点速度
// start_a 起点加速度
// end_pt  终点位置
// end_v   终点速度
// init    初始化是否
// dynamic 是否考虑运动学模型
// time_start 开始时间
int Hybrid_astar::search(Eigen::Vector2d start_pt, Eigen::Vector2d start_v, Eigen::Vector2d start_a,
                         Eigen::Vector2d end_pt, Eigen::Vector2d end_v,
                         bool init, bool dynamic, double time_start)
{
  end_point_ = end_pt;
  // 赋值 起点速度赋值
  start_vel_ = start_v;
  start_acc_ = start_a;

  // 检测起点终点是否在障碍物里面
  if (isOccupied(start_pt(0), start_pt(1)))
  {
    ROS_ERROR("start in the obstacle: %.2f %.2f", start_pt[0], start_pt[1]);
    return IN_OCCUPIED;
  }
  if (isOccupied(end_pt(0), end_pt(1)))
  {
    ROS_ERROR("end in the obstacle: %.2f %.2f", end_pt[0], end_pt[1]);
    return IN_OCCUPIED;
  }

  // 当前节点清0
  HybridGridNodePtr cur_node = path_node_pool_[0]; // 定义当前节点为第一节点

  cur_node->cameFrom = NULL;              // 父节点为空
  cur_node->state.head(2) = start_pt;     // 状态开始两位是起点位置
  cur_node->state.tail(2) = start_v;      // 状态开始后两位是速度
  cur_node->index = worldToMap(start_pt); // 将世界坐标系转换成栅格坐标系
  cur_node->gScore = 0.0;                 // 起点已经走过路径是0

  // 定义结束状态为一个4的向量
  VectorXd end_state(4);
  Vector2i end_index;  // 终点栅格坐标
  double time_to_goal; // 到终点的时间

  end_state.head(2) = end_pt;     // 结束状态头两位是位置
  end_state.tail(2) = end_v;      // 结束状态后两位是速度
  end_index = worldToMap(end_pt); // 世界坐标系转栅格坐标系
  cur_node->fScore = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
  cur_node->node_state = IN_OPEN_SET; // 放到开集列表中
  open_set_.push(cur_node);           // 开集添加进来
  use_node_num_ += 1;                 // 已经使用节点数目+1

  // 是否考虑动力学模型，用于重规划，在规划的时候使用动力学模型有的终点可能无法达到，如果在不约束的情况下还是没有找到终点
  // 那就没有终点了
  if (dynamic)
  {
    time_origin_ = time_start;                    // 起始时间等于开始时间
    cur_node->time = time_start;                  // 当前时间就是开始时间
    cur_node->time_idx = timeToIndex(time_start); // 转换成整形
    // 添加到扩展节点中 当前节点的栅格坐标点 和 时间信息
    expanded_nodes_.insert(cur_node->index, cur_node->time_idx, cur_node);
  }
  else
  {
    // 如果不适用运动学模型，那么不添加时间信息
    expanded_nodes_.insert(cur_node->index, cur_node);
  }

  HybridGridNodePtr terminate_node = NULL; // 定义终止节点
  bool init_search = init;                 // 是否初始化搜索
  const int tolerance = ceil(1 / resolution_);

  // 当可搜索集合不为空open不为空
  while (!open_set_.empty())
  {
    cur_node = open_set_.top(); // 当前节点信息等于open列表中第一个信息

    // 判断是否在规划区域之内  是一个圆的大小
    bool reach_horizon = (cur_node->state.head(2) - start_pt).norm() >= horizon_;
    // 接近终点
    bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                    abs(cur_node->index(1) - end_index(1)) <= tolerance;
    // 达到一个范围 或者接近终点
    if (reach_horizon || near_end)
    {
      // 终止节点等于当前节点，代表搜索完毕
      terminate_node = cur_node;
      // 回溯路径
      retrievePath(terminate_node);
      // 接近终点
      if (near_end)
      {
        // 检查当前点是否在轨迹上
        estimateHeuristic(cur_node->state, end_state, time_to_goal);
        // 计算当前点是否在路径上
        computeShotTraj(cur_node->state, end_state, time_to_goal);
        // 如果初始化
        if (init_search)
          ROS_ERROR("Shot in first search loop!");
      }
    }

    // 如果到达范围
    if (reach_horizon)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else
      {
        // 到达同一平面
        std::cout << "reach horizon" << std::endl;
        return REACH_HORIZON;
      }
    }

    // 如果接近终点
    if (near_end)
    {
      if (is_shot_succ_)
      {
        std::cout << "reach end" << std::endl;
        return REACH_END;
      }
      else if (cur_node->cameFrom != NULL)
      {
        std::cout << "near end" << std::endl;
        return NEAR_END;
      }
      else
      {
        std::cout << "no path" << std::endl;
        return NO_PATH;
      }
    }

    // 开集合顶点
    open_set_.pop();
    // 发送当前终点状态
    cur_node->node_state = IN_CLOSE_SET; // 当前节点符合要求
    iter_num_ += 1;

    // 设定分辨率
    double res = 1 / 2.0, time_res = 1 / 1.0, time_res_init = 1 / 20.0;
    // 传递当前节点状态
    Matrix<double, 4, 1> cur_state = cur_node->state;
    // 前一个节点的状态
    Matrix<double, 4, 1> pro_state;
    // 临时扩展节点
    vector<HybridGridNodePtr> tmp_expand_nodes;
    // 车的运动模型
    Vector2d um;
    // 前一个时刻
    double pro_t;
    // 输入值
    vector<Vector2d> inputs;
    vector<double> durations; // 时间间隔

    // 如果搜索初始化
    if (init_search)
    {
      // 添加加速度状态
      inputs.push_back(start_acc_);
      // 时间分隔
      for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3;
           tau += time_res_init * init_max_tau_)
      {
        durations.push_back(tau);
      }
      init_search = false;
    }
    else
    {
      // 这里的最大加速度取三个坐标系下相同
      for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
      {
        for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
        {
          um << ax, ay;
          inputs.push_back(um);
        }
      }

      for (double tau = time_res * max_tau_; tau <= max_tau_; tau += time_res * max_tau_)
      {
        durations.push_back(tau);
      }
    }

    // 进行状态赋值
    for (long unsigned int i = 0; i < inputs.size(); ++i)
    {
      for (long unsigned int j = 0; j < durations.size(); ++j)
      {
        um = inputs[i];
        double tau = durations[j];
        // 状态转移
        stateTransit(cur_state, pro_state, um, tau);
        pro_t = cur_node->time + tau;

        // 前一个点的位置
        Vector2d pro_pos = pro_state.head(2);

        // 检查是否在close中
        Vector2i pro_id = worldToMap(pro_pos);
        // 时间取整
        int pro_t_id = timeToIndex(pro_t);
        // 当前节点是考虑动力学还是普通的扩展节点
        HybridGridNodePtr pro_node = dynamic ? expanded_nodes_.find(pro_id, pro_t_id) : expanded_nodes_.find(pro_id);
        // 如果前一个节点不为空或者前一个节点在close中
        if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
        {
          if (init_search)
            std::cout << "close" << std::endl;
          continue;
        }

        // 传递上一个节点的速度信息
        Vector2d pro_v = pro_state.tail(2); // 获取速度信息
        if (fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_)
        {
          if (init_search)
            std::cout << "vel" << std::endl;
          continue;
        }

        // 确保起点和终点不是同一个点
        Vector2i diff = pro_id - cur_node->index;
        // 时间差
        int diff_time = pro_t_id - cur_node->time_idx;
        // 判断
        if (diff.norm() == 0 && ((!dynamic) || diff_time == 0))
        {
          if (init_search)
            std::cout << "same" << std::endl;
          continue;
        }

        // 检测是否安全
        Vector2d pos;
        Matrix<double, 4, 1> xt;
        bool is_occ = false;
        for (int k = 1; k <= check_num_; ++k)
        {
          // 计算时间
          double dt = tau * double(k) / double(check_num_);
          // 状态转移
          stateTransit(cur_state, xt, um, dt);
          pos = xt.head(2);
          Vector2i posIndex = worldToMap(pos);
          // 是否碰到膨胀地图
          if (isOccupied(posIndex[0], posIndex[1]))
          {
            is_occ = true;
            break;
          }
        }
        if (is_occ) // 如果被占据
        {
          if (init_search)
            std::cout << "safe" << std::endl;
          continue;
        }

        // 定义到终点的时间 临时节点g代价 临时节点f代价
        double time_to_goal, tmp_g_score, tmp_f_score;
        // 计算g代价
        tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->gScore;
        // 计算f代价
        tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

        // 比较同一个父节点扩展出来的节点
        bool prune = false;
        // 扫描扩展节点
        for (long unsigned int j = 0; j < tmp_expand_nodes.size(); ++j)
        {
          // 定义临时扩展节点
          HybridGridNodePtr expanded_nodes = tmp_expand_nodes[j];
          // 符合要求进行扩展
          // 首先不是同一个节点，并且在不考虑动力学，且时间符合扩展节点的时间范围
          if ((pro_id - expanded_nodes->index).norm() == 0 &&
              ((!dynamic) || pro_t_id == expanded_nodes->time_idx))
          {
            // 可以扩展
            prune = true;
            // 如果被扩展的f数值 小于 扩展节点的 f数值
            if (tmp_f_score < expanded_nodes->fScore)
            {
              // 代价数值
              expanded_nodes->fScore = tmp_f_score;
              expanded_nodes->gScore = tmp_g_score;
              // 状态传递
              expanded_nodes->state = pro_state;
              // 输入
              expanded_nodes->input = um;
              // 延时
              expanded_nodes->duration = tau;
              if (dynamic)
                expanded_nodes->time = cur_node->time + tau;
            }
            break;
          }
        }

        // 如果没扩展成功
        if (!prune)
        {
          // 上一个节点不为空
          if (pro_node == NULL)
          {
            // 对当前节点赋值 = 路径节点池中当前节点
            pro_node = path_node_pool_[use_node_num_];
            // 获取坐标
            pro_node->index = pro_id;
            // 传递状态
            pro_node->state = pro_state;
            // 传递代价值
            pro_node->fScore = tmp_f_score;
            pro_node->gScore = tmp_g_score;
            // 输入赋值
            pro_node->input = um;
            // 延时
            pro_node->duration = tau;
            // 父节点
            pro_node->cameFrom = cur_node;
            // 开集
            pro_node->node_state = IN_OPEN_SET;
            // 在考虑动力学模型的时候考虑以下代码
            if (dynamic)
            {
              // 时间传递
              pro_node->time = cur_node->time + tau;
              pro_node->time_idx = timeToIndex(pro_node->time);
            }
            // 放入开集合中
            open_set_.push(pro_node);

            // 在考虑动力学模型的时候考虑时间
            if (dynamic)
            {
              expanded_nodes_.insert(pro_id, pro_node->time, pro_node);
            }
            else
            {
              expanded_nodes_.insert(pro_id, pro_node);
            }

            // 添加
            tmp_expand_nodes.push_back(pro_node);

            // 使用过的节点累加
            use_node_num_ += 1;
            // 如果使用节点数目 == 允许节点数目 那么范围超出寄了
            if (use_node_num_ == allocate_num_)
            {
              cout << "run out of memory." << endl;
              return NO_PATH;
            }
          }
          // 如果当前节点在开集中
          else if (pro_node->node_state == IN_OPEN_SET)
          {
            // 当此时已经走过路径值比上一个节点路静止小
            if (tmp_g_score < pro_node->gScore)
            {
              // 参数传递
              pro_node->state = pro_state;
              // 代价值传递
              pro_node->fScore = tmp_f_score;
              pro_node->gScore = tmp_g_score;
              // 输入值传递
              pro_node->input = um;
              // 延时传递
              pro_node->duration = tau;
              // 父亲节点
              pro_node->cameFrom = cur_node;
              if (dynamic)
                pro_node->time = cur_node->time + tau;
            }
          }
          else
          {
            cout << "error type in searching: " << pro_node->node_state << endl;
          }
        }
      }
    }
  }

  // 开集合为空 没有路径
  cout << "open set empty, no path!" << endl;
  // 已经使用过的节点数目
  cout << "use node num: " << use_node_num_ << endl;
  cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

std::vector<HybridGridNodePtr> Hybrid_astar::getVisitedNodes(void)
{
  vector<HybridGridNodePtr> visited;
  visited.assign(path_node_pool_.begin(), path_node_pool_.begin() + use_node_num_ - 1);
  return visited;
}

// 计算从状态1到状态2的运动轨迹
// 参照时间是到终点的时间
bool Hybrid_astar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
  // 起始位置
  const Vector2d p0 = state1.head(2);      // 位置信息
  const Vector2d dp = state2.head(2) - p0; // 位置信息作差
  // 起始位置速度向量
  const Vector2d v0 = state1.segment(2, 2);
  // 终点位置速度向量
  const Vector2d v1 = state2.segment(2, 2);
  // 速度作差
  const Vector2d dv = v1 - v0;

  double t_d = time_to_goal;

  // 构造一个3行 四列的矩阵
  MatrixXd coef(2, 4);

  // 获取终点速度
  end_vel_ = v1;

  // 用于计算三次多项式曲线的系数
  // a、b、c、d 分别表示多项式的四个系数，代表物体的加速度、速度、初速度和初位移。
  // 这些系数是通过对物体的初速度、末速度、初位移和末位移进行求解得到的
  // a求解加速度
  Vector2d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
  // b求解速度
  Vector2d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv); //
  // c是当前速度
  Vector2d c = v0; // 速度
  // d是位置信息
  Vector2d d = p0; // 位置

  // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
  // a*t^3 + b*t^2 + v0*t + p0
  // 求解三次多项式插值函数的系数
  coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

  //
  Vector2d coord, vel, acc;
  VectorXd poly1d, t, polyv, polya;
  Vector2i index; // 栅格坐标

  // 计算多项式的导数
  Eigen::MatrixXd Tm(4, 4);
  Tm << 0, 1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0;

  // 判断生成的轨迹有效性
  double t_delta = t_d / 10;
  for (double time = t_delta; time < t_d; time += t_delta)
  {
    // 先对时间归0
    t = VectorXd::Zero(4); // 清0
    for (int j = 0; j < 4; j++)
    {
      t(j) = pow(time, j); // 三次多项式时间系数
    }

    // 维度 二维
    for (int dim = 0; dim < 2; dim++)
    {
      // 两个维度进行计算
      poly1d = coef.row(dim);
      // 向t进行点乘 得到一个坐标向量
      coord(dim) = poly1d.dot(t);
      // 速度点乘 求解速度随时间的变化多项式
      vel(dim) = (Tm * poly1d).dot(t);
      // 加速度点乘 求解加速度随时间的变化多项式
      acc(dim) = (Tm * Tm * poly1d).dot(t);

      // 最大加速度限制
      if (fabs(vel(dim)) > max_vel_ || fabs(acc(dim)) > max_acc_)
      {
        // cout << "vel:" << vel(dim) << ", acc:" << acc(dim) << endl;
        // return false;
      }
    }

    // 如果索引位置超出地图边界跳出
    if (coord(0) < origin_(0) || coord(0) >= map_size_2d_(0) ||
        coord(1) < origin_(1) || coord(1) >= map_size_2d_(1))
    {
      return false;
    }

    Vector2i coordindex = worldToMap(coord);
    // 是否碰撞到障碍物
    if (isOccupied(coordindex(0), coordindex(1)))
    {
      return false;
    }
  }

  // 方程传递
  coef_shot_ = coef;
  //
  t_shot_ = t_d;
  // 符合要求 当前节点有效
  is_shot_succ_ = true;
  return true;
}

// 状态转移
void Hybrid_astar::stateTransit(Eigen::Matrix<double, 4, 1> &state0,
                                Eigen::Matrix<double, 4, 1> &state1,
                                Eigen::Vector2d um, double tau)
{
  // 状态传递
  for (int i = 0; i < 2; ++i)
    phi_(i, i + 2) = tau;

  // 整形矩阵
  Matrix<double, 4, 1> integral;
  integral.head(2) = 0.5 * pow(tau, 2) * um;
  integral.tail(2) = tau * um;

  // 状态转移
  state1 = phi_ * state0 + integral;
}
