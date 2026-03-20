//
// Created by Zhang Zhimeng on 2021/11/29.
//

#include "path_optimization/minimum_snap.h"
#include "ros/ros.h"

MinimumSnap::MinimumSnap()
{
  // 默认参数
  order_ = 3;       // 多项式阶数3阶
  max_vel_ = 1.0;   // 最大速度1.0
  max_accel_ = 1.0; // 最大加速度1.0

  // std::cout<<"order="<<order_<<std::endl;
  // std::cout<<"max_vel="<<max_vel_<<std::endl;
  // std::cout<<"max_accel="<<max_accel_<<std::endl;
}

// MinimumSnap初始化函数
// 多项式阶数 1阶是速度  2阶是加速度  3阶是jerk  4阶是snap
// 最大速度约束
// 最大加速度约束
void MinimumSnap::setParams(unsigned int order, double max_vel, double max_accel)
{
  order_ = order;
  max_vel_ = max_vel;
  max_accel_ = max_accel;

  std::cout << "order=" << order_ << std::endl;
  std::cout << "max_vel=" << max_vel_ << std::endl;
  std::cout << "max_accel=" << max_accel_ << std::endl;
}

// 根据路径点去合理分配时间间隔，最终实现最初规划路径的分段
VecXd MinimumSnap::AllocateTime(const MatXd &waypoint) const
{
  VecXd times = VecXd::Zero(waypoint.rows() - 1);
  const double t = max_vel_ / max_accel_;             // t=速度/加速度
  const double dist_threshold_1 = max_accel_ * t * t; // 定义距离届？？？

  double segment_t;
  for (unsigned int i = 1; i < waypoint.rows(); ++i)
  {
    // 计算路径点之间的距离差值
    double delta_dist = (waypoint.row(i) - waypoint.row(i - 1)).norm();
    if (delta_dist > dist_threshold_1)
    {                                                                 // 当距离差>距离届
      segment_t = t * 2 + (delta_dist - dist_threshold_1) / max_vel_; // 时间段=2t+(距离差)/最大速度
    }
    else
    {
      segment_t = std::sqrt(delta_dist / max_accel_); // 时间段=sqrt(距离/最大加速度)
    }

    times[i - 1] = segment_t;
  }

  return times;
}

unsigned int MinimumSnap::GetPolyCoeffNum() const
{
  return 2u * order_;
}

MatXd MinimumSnap::SolveQPClosedForm(const VecXd &waypoints_pos, const VecXd &waypoints_vel,
                                     const VecXd &waypoints_acc, const VecXd &segments_time) const
{
  if(waypoints_pos.size() != waypoints_vel.size())
  {
    std::cout<<"waypoints_pos.size() "<<waypoints_pos.size()<<std::endl;
  }

  if(waypoints_vel.size() != waypoints_acc.size())
  {
    std::cout<<"waypoints_vel.size() "<<waypoints_vel.size()<<std::endl;
  }
  
  if(waypoints_pos.size() != segments_time.size() + 1u)
  {
    std::cout<<"segments_time.size() "<<segments_time.size()<<std::endl;
  }

  // 多项式的阶数为 2*order-1阶，如果是snap优化就是7阶，jerk优化就是5阶段，acce优化就是3阶
  const unsigned int poly_order = 2 * order_ - 1;
  const unsigned int num_poly_coeff = poly_order + 1; // 参数的个数为阶数+1 2*order
  // 按照所分时间段的数目确定分多所段 这里的段与已经规划的路径点的关系是=路径点-1 即k-1段
  const unsigned int num_segments = segments_time.size();

  // k个路径点，每段n+1个未知数  那么一共有(k-1)(n+1)
  const unsigned int num_all_poly_coeff = num_poly_coeff * num_segments;

  // 6个等式对应6个未知数 即 p0 p1 p2 p3 p4 p5
  const unsigned int A_block_rows = 2u * order_;                                   // 行对应x坐标
  const unsigned int A_block_cols = num_poly_coeff;                                // 列对应y坐标
  MatXd A = MatXd::Zero(num_segments * A_block_rows, num_segments * A_block_cols); // 对矩阵初始化

  // 对每个路径段进行求解
  for (unsigned int i = 0; i < num_segments; ++i)
  {
    unsigned int row = i * A_block_rows;
    unsigned int col = i * A_block_cols;

    // 约束初始化
    MatXd sub_A = MatXd::Zero(A_block_rows, A_block_cols);

    for (unsigned int j = 0; j < order_; ++j)
    {
      for (unsigned int k = 0; k < num_poly_coeff; ++k)
      {
        if (k < j)
        {
          continue;
        }

        sub_A(j, num_poly_coeff - 1 - k) = Factorial(k) / Factorial(k - j) * std::pow(0, k - j);
        sub_A(j + order_, num_poly_coeff - 1 - k) =
            Factorial(k) / Factorial(k - j) * pow(segments_time[i], k - j);
      }
    }

    A.block(row, col, A_block_rows, A_block_cols) = sub_A;
  }

  // order*(k+1) 即对应所有路径点的数目*order 中间点约束的变量个数
  const unsigned int num_valid_variables = (num_segments + 1) * order_;
  // 固定变量数目 = 2*order + k-2 = k+4个 固定约束数目
  const unsigned int num_fixed_variables = 2u * order_ + (num_segments - 1);
  // C_T
  MatXd C_T = MatXd::Zero(num_all_poly_coeff, num_valid_variables);
  for (unsigned int i = 0; i < num_all_poly_coeff; ++i)
  {
    if (i < order_)
    {
      C_T(i, i) = 1.0;
      continue;
    }

    if (i >= num_all_poly_coeff - order_)
    {
      const unsigned int delta_index = i - (num_all_poly_coeff - order_);
      C_T(i, num_fixed_variables - order_ + delta_index) = 1.0;
      continue;
    }

    if ((i % order_ == 0u) && (i / order_ % 2u == 1u))
    {
      const unsigned int index = i / (2u * order_) + order_;
      C_T(i, index) = 1.0;
      continue;
    }

    if ((i % order_ == 0u) && (i / order_ % 2u == 0u))
    {
      const unsigned int index = i / (2u * order_) + order_ - 1u;
      C_T(i, index) = 1.0;
      continue;
    }

    if ((i % order_ != 0u) && (i / order_ % 2u == 1u))
    {
      const unsigned int temp_index_0 = i / (2 * order_) * (2 * order_) + order_;
      const unsigned int temp_index_1 = i / (2 * order_) * (order_ - 1) + i - temp_index_0 - 1;
      C_T(i, num_fixed_variables + temp_index_1) = 1.0;
      continue;
    }

    if ((i % order_ != 0u) && (i / order_ % 2u == 0u))
    {
      const unsigned int temp_index_0 = (i - order_) / (2 * order_) * (2 * order_) + order_;
      const unsigned int temp_index_1 =
          (i - order_) / (2 * order_) * (order_ - 1) + (i - order_) - temp_index_0 - 1;
      C_T(i, num_fixed_variables + temp_index_1) = 1.0;
      continue;
    }
  }

  // 构造Q矩阵，这里Q的阶数为 (k-1)*(n+1) 6*(k-1)
  MatXd Q = MatXd::Zero(num_all_poly_coeff, num_all_poly_coeff);
  for (unsigned int k = 0u; k < num_segments; ++k)
  {
    MatXd sub_Q = MatXd::Zero(num_poly_coeff, num_poly_coeff);
    for (unsigned int i = 0u; i <= poly_order; ++i)
    {
      for (unsigned int l = 0u; l <= poly_order; ++l)
      {
        if (num_poly_coeff - i <= order_ || num_poly_coeff - l <= order_)
        {
          continue;
        }

        sub_Q(i, l) = (Factorial(poly_order - i) / Factorial(poly_order - order_ - i)) *
                      (Factorial(poly_order - l) / Factorial(poly_order - order_ - l)) /
                      (poly_order - i + poly_order - l - (2 * order_ - 1)) *
                      std::pow(segments_time[k], poly_order - i + poly_order - l - (2 * order_ - 1));
      }
    }

    const unsigned int row = k * num_poly_coeff;
    Q.block(row, row, num_poly_coeff, num_poly_coeff) = sub_Q;
  }

  // 构造最终的求解等式
  MatXd R = C_T.transpose() * A.transpose().inverse() * Q * A.inverse() * C_T;
  // 连续项约束
  VecXd d_selected = VecXd::Zero(num_valid_variables);
  for (unsigned int i = 0; i < num_all_poly_coeff; ++i)
  {
    if (i == 0u)
    {
      d_selected[i] = waypoints_pos[0]; // 位置约束
      continue;
    }

    if (i == 1u && order_ >= 2u)
    {
      d_selected[i] = waypoints_vel[0]; // 速度约束
      continue;
    }

    if (i == 2u && order_ >= 3u)
    {
      d_selected[i] = waypoints_acc[0]; // 加速度约束
      continue;
    }

    if (i == num_all_poly_coeff - order_ + 2u && order_ >= 3u)
    {
      d_selected(num_fixed_variables - order_ + 2) = waypoints_acc[1];
      continue;
    }

    if (i == num_all_poly_coeff - order_ + 1u && order_ >= 2u)
    {
      d_selected(num_fixed_variables - order_ + 1) = waypoints_vel[1];
      continue;
    }

    if (i == num_all_poly_coeff - order_)
    {
      d_selected(num_fixed_variables - order_) = waypoints_pos[num_segments];
      continue;
    }

    if ((i % order_ == 0u) && (i / order_ % 2u == 0u))
    {
      const unsigned int index = i / (2 * order_) + order_ - 1;
      d_selected(index) = waypoints_pos[i / (2 * order_)];
      continue;
    }
  }

  MatXd R_PP = R.block(num_fixed_variables, num_fixed_variables,
                       num_valid_variables - num_fixed_variables,
                       num_valid_variables - num_fixed_variables);

  VecXd d_F = d_selected.head(num_fixed_variables);
  MatXd R_FP = R.block(0, num_fixed_variables, num_fixed_variables,
                       num_valid_variables - num_fixed_variables);

  // 导数 = -R_PP的逆矩阵 * R_FP的转秩矩阵*
  MatXd d_optimal = -R_PP.inverse() * R_FP.transpose() * d_F;

  d_selected.tail(num_valid_variables - num_fixed_variables) = d_optimal;
  VecXd d = C_T * d_selected;

  VecXd P = A.inverse() * d; // 矩阵反转

  MatXd poly_coeff_mat = MatXd::Zero(num_segments, num_poly_coeff);
  for (unsigned int i = 0; i < num_segments; ++i)
  {
    poly_coeff_mat.block(i, 0, 1, num_poly_coeff) =
        P.block(num_poly_coeff * i, 0, num_poly_coeff, 1).transpose();
  }

  return poly_coeff_mat;
}

// 装载路径节点
MatXd MinimumSnap::PickWaypoints(const std::vector<Eigen::Vector2d> &path_ptr) const
{
  // CHECK_GE(path_ptr.size(), 2u);
  if(path_ptr.size() < 2u)
  {
    std::cout<<"path size is too short"<<std::endl;
  }

  const unsigned int num_waypoint = path_ptr.size();
  MatXd waypoints_mat = MatXd::Zero(num_waypoint, 2u);

  // 进行转秩
  for (unsigned int i = 0; i < path_ptr.size(); ++i)
  {
    waypoints_mat.block(i, 0, 1, 2) =
        Vec2d(path_ptr[i][0],
              path_ptr[i][1])
            .transpose();
  }

  return waypoints_mat;
}

// 对获取的位置在极坐标系下进行标准化
Vec2d MinimumSnap::GetPositionPolynomial(const MatXd &poly_coeff_mat, unsigned int k, double t)
{
  Vec2d position;

  const unsigned int num_poly_coeff = GetPolyCoeffNum();

  for (unsigned int dim = 0; dim < 2u; ++dim)
  {
    VecXd coeff = (poly_coeff_mat.row(k)).segment(num_poly_coeff * dim, num_poly_coeff);
    VecXd time = VecXd::Zero(num_poly_coeff);

    for (unsigned int i = 0; i < num_poly_coeff; ++i)
    {
      if (i == 0)
      {
        time(i) = 1.0;
      }
      else
      {
        time(i) = pow(t, i);
      }
    }

    double temp_position = 0.0;
    for (unsigned int i = 0u; i < time.rows(); ++i)
    {
      temp_position = temp_position + coeff(i) * time(time.rows() - i - 1u);
    }

    position(dim) = temp_position;
  }

  return position;
}

std::vector<Eigen::Vector2d> MinimumSnap::Minimum_Snap(std::vector<Eigen::Vector2d> path)
{
  MatXd waypoint_posi = PickWaypoints(path);                // 获取当前路径
  VecXd waypoint_vel = VecXd::Zero(waypoint_posi.rows());   // 初始化速度矩阵
  VecXd waypoint_accel = VecXd::Zero(waypoint_posi.rows()); // 初始化加速度矩阵

  VecXd time = AllocateTime(waypoint_posi);

  // 将速度加速度参数放近来，通过minimum_snap进行约束
  MatXd polynomial_coeff_x = SolveQPClosedForm(waypoint_posi.col(0),
                                               waypoint_vel, waypoint_accel, time);
  MatXd polynomial_coeff_y = SolveQPClosedForm(waypoint_posi.col(1),
                                               waypoint_vel, waypoint_accel, time);

  MatXd polynomial_coeff = MatXd::Zero(polynomial_coeff_x.rows(), polynomial_coeff_x.cols() * 2u);
  polynomial_coeff.leftCols(polynomial_coeff_x.cols()) = polynomial_coeff_x;
  polynomial_coeff.rightCols(polynomial_coeff_x.cols()) = polynomial_coeff_y;

  std::vector<Eigen::Vector2d> trajectoryPath;
  Vec2d temp_position;
  Eigen::Vector2d point;

  for (unsigned int i = 0; i < time.size(); ++i)
  {
    for (double t = 0.0; t < time(i);)
    {
      temp_position = GetPositionPolynomial(polynomial_coeff, i, t);
      point[0] = temp_position.x();
      point[1] = temp_position.y();
      trajectoryPath.push_back(point);
      t += 0.01;
    }
  }

  return trajectoryPath;
}

// 根据路径点数目进行分段minimum snap
std::vector<Eigen::Vector2d> MinimumSnap::SubsectionPath_Minimum_Snap(std::vector<Eigen::Vector2d> path)
{
  unsigned int pathPoints = path.size();   // 获取路径点数目
  std::vector<Eigen::Vector2d> optSubPath; // 优化路径段
  unsigned int sub = 20;
#if 0
    if(pathPoints > 30)
    {
        std::vector<Eigen::Vector2d> firstPath;     //第一段优化的路径
        firstPath.resize(20);
        for(unsigned int i=0;i<20;i++)              //将路径的前20个点放到里面
        {
            firstPath[i] = path[i];
        }
        std::vector<Eigen::Vector2d> firstOpt;      //第一段优化路径
        firstOpt = Minimum_Snap(firstPath);         //获取到第一次优化的路径

        for(unsigned int i=0;i<firstOpt.size()-5;i++)
        {
            optSubPath.push_back(firstOpt[i]);
        }

        std::vector<Eigen::Vector2d> tempPath;      //临时路段  5个上次优化的  15个新的
        tempPath.resize(20);
        
        std::vector<Eigen::Vector2d> lastOptPath;   //上次优化的路段，5个
        lastOptPath = firstOpt;                     //第一次传参上次优化路径直接等于第一次优化的路径

        for(unsigned int j=1;j<pathPoints/15-1;j++)
        {
            for(unsigned int i=0;i<20;i++)
            {
                if(i<5)
                {
                    tempPath[i] = lastOptPath[lastOptPath.size()-5+i];    //临时路径的前五个是上一次优化的后五个
                }
                else
                {
                    tempPath[i] = path[i+5+j*15];  //获取路径点
                }
            }
            lastOptPath = Minimum_Snap(tempPath); //进行优化
            for(unsigned int i=0;i<lastOptPath.size()-5;i++)        //将优化好的路径点放入优化路径中
            {
                optSubPath.push_back(lastOptPath[i+5]);
            }
        }
        // 最后一段路径优化
        std::vector<Eigen::Vector2d> lastpath;  //最后一段路径
        std::vector<Eigen::Vector2d> lastOpt;   //最后一段优化的路径
        unsigned int lastpoint = pathPoints - 15*(pathPoints/15-1);

        if(lastpoint == pathPoints)
        {
            for(unsigned int i=0;i<pathPoints - 20;i++)
            {
                lastpath[i] = path[20+i];
            }
            lastOpt = Minimum_Snap(lastpath);
            for(unsigned int i=0;i<lastOpt.size();i++)
            {
                optSubPath.push_back(lastOpt[i]);
            }
        }
        else{
            lastpath.resize(lastpoint);
            for(unsigned int i=0;i<lastpoint;i++)
            {
                lastpath[i] = path[15*(pathPoints/15-1)+i];
            }
            lastOpt = Minimum_Snap(lastpath);
            for(unsigned int i=0;i<lastOpt.size();i++)
            {
                optSubPath.push_back(lastOpt[i]);
            }
        }

        return optSubPath;
    }
    else
    {
        return Minimum_Snap(path);
    }
#endif
  if (pathPoints > 25)
  {
    std::vector<Eigen::Vector2d> tempath; // 用于进行优化的路段
    std::vector<Eigen::Vector2d> optpath;
    tempath.resize(sub);
    for (unsigned int j = 0; j < pathPoints / sub; j++) // 求解分了多少段
    {
      for (unsigned int i = 0; i < sub; i++)
      {
        tempath[i] = path[j * sub + i];
      }
      optpath = Minimum_Snap(tempath); // 优化
      for (unsigned int i = 0; i < optpath.size(); i++)
      {
        optSubPath.push_back(optpath[i]);
      }
      optpath.clear();
    }
    std::vector<Eigen::Vector2d> lastpath; // 用于进行优化的路段
    unsigned int lastpoints = pathPoints - sub * (pathPoints / sub);
    lastpath.resize(lastpoints);
    if (lastpoints >= 3)
    {
      // 最后一段进行优化
      for (unsigned int i = 0; i < lastpoints; i++)
      {
        lastpath[i] = path[sub * (pathPoints / sub) + i];
      }
      optpath = Minimum_Snap(lastpath);
      for (unsigned int i = 0; i < optpath.size(); i++)
      {
        optSubPath.push_back(optpath[i]);
      }
    }
    else
    {
      for (unsigned int i = 0; i < lastpoints; i++)
      {
        optSubPath.push_back(path[sub * (pathPoints / sub) + i]);
      }
    }
    return optSubPath;
  }
  else
  {
    return Minimum_Snap(path);
  }
}
