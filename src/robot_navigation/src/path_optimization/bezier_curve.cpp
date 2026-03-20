#include "path_optimization/bezier_curve.h"

// 构造函数中初始化默认参数
BEZIER::BEZIER()
{
  max_vel_ = 1.0;         // 最大速度 1.0
  time_resolution = 0.01; // 时间分辨率0.01
  sub = 3;
}

/**
 * 贝塞尔公式
 * @param path
 * @return
 */
Eigen::Vector2d BEZIER::bezierCommon(std::vector<Eigen::Vector2d> path, double t)
{
  if (path.size() == 1)
    return path[0];

  Eigen::Vector2d p_t(0., 0.);
  int n = path.size() - 1;
  for (unsigned int i = 0; i < path.size(); i++)
  {
    double C_n_i = Factorial(n) / (Factorial(i) * Factorial(n - i));
    p_t += C_n_i * pow((1 - t), (n - i)) * pow(t, i) * path[i];
  }
  return p_t;
}

// 最终的贝塞尔曲线的函数
std::vector<Eigen::Vector2d> BEZIER::BezierCurve(std::vector<Eigen::Vector2d> path)
{
  std::vector<Eigen::Vector2d> optpath;

  for (double t = 0.0; t < 1.0; t += time_resolution)
  {
    Eigen::Vector2d point;
    point = bezierCommon(path, t);
    optpath.push_back(point);
  }

  return optpath;
}

// 用于设定参数
void BEZIER::setParams(double _max_vel, double _time_resolution,unsigned int _sub)
{
  max_vel_ = _max_vel;
  time_resolution = _time_resolution;
  sub = _sub;
}

// 贝塞尔曲线的函数
// 该函数为最高阶四阶的贝塞尔曲线，即将路径进行四点一分段
/*
贝赛尔曲线:
B(t)=(1-t)*P0+t*P1 一次
B(t)=(1-t)^2*P0+2*t*(t-1)*P1+t^2*P2 二次
*/
std::vector<Eigen::Vector2d> BEZIER::BezierPath(std::vector<Eigen::Vector2d> path)
{
  float timeDuration = time_resolution;
  std::vector<Eigen::Vector2d> result;
  if (path.size() >= 2) // 路径规划有效则开始平滑
  {
    for (int index = path.size(); index > 0;)
    {
      // 四点三次贝赛尔曲线
      if (index >= 4)
      {
        for (float t = 0.0; t < 1.00; t += timeDuration)
        {
          float x0 = path[path.size() - index][0];
          float y0 = path[path.size() - index][1];
          float x1 = path[path.size() - index + 1][0];
          float y1 = path[path.size() - index + 1][1];
          float x2 = path[path.size() - index + 2][0];
          float y2 = path[path.size() - index + 2][1];
          float x3 = path[path.size() - index + 3][0];
          float y3 = path[path.size() - index + 3][1];
          float Bx = (1 - t) * (1 - t) * (1 - t) * x0 + 3 * t * (1 - t) * (1 - t) * x1 + 3 * t * t * (1 - t) * x2 + t * t * t * x3;
          float By = (1 - t) * (1 - t) * (1 - t) * y0 + 3 * t * (1 - t) * (1 - t) * y1 + 3 * t * t * (1 - t) * y2 + t * t * t * y3;
          result.push_back(Eigen::Vector2d(Bx, By));
        }
        index -= 3;
        continue;
      }

      // 三点二次贝塞尔
      if (index >= 3)
      {
        for (float t = 0.0; t < 1.00; t += timeDuration)
        {
          float x0 = path[path.size() - index][0];
          float y0 = path[path.size() - index][1];
          float x1 = path[path.size() - index + 1][0];
          float y1 = path[path.size() - index + 1][1];
          float x2 = path[path.size() - index + 2][0];
          float y2 = path[path.size() - index + 2][1];
          float Bx = (1 - t) * (1 - t) * x0 + 2 * t * (1 - t) * x1 + t * t * x2;
          float By = (1 - t) * (1 - t) * y0 + 2 * t * (1 - t) * y1 + t * t * y2;
          result.push_back(Eigen::Vector2d(Bx, By));
        }
        index -= 2;
        continue;
      }

      if (index >= 2)
      {
        // 两点一次贝赛尔
        for (float t = 0.0; t < 1.00; t += timeDuration)
        {
          float x0 = path[path.size() - index][0];
          float y0 = path[path.size() - index][1];
          float x1 = path[path.size() - index + 1][0];
          float y1 = path[path.size() - index + 1][1];
          float Bx = (1 - t) * x0 + t * x1;
          float By = (1 - t) * y0 + t * y1;
          result.push_back(Eigen::Vector2d(Bx, By));
        }
        index -= 1;
        continue;
      }
      // 终点
      if (index == 1)
      {
        result.push_back(path[path.size() - 1]);
        index = 0;
        continue;
      }
    }
  }

  return result;
}

// 分段优化的贝塞尔曲线
std::vector<Eigen::Vector2d> BEZIER::SubsectionPath_Bezier(std::vector<Eigen::Vector2d> path)
{
  unsigned int pathPoints = path.size();   // 获取路径点数目
  std::vector<Eigen::Vector2d> optSubPath; // 优化路径段

  if (pathPoints > sub)
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
      optpath = BezierCurve(tempath); // 优化
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
      optpath = BezierCurve(lastpath);
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
    return BezierCurve(path);
  }
}
