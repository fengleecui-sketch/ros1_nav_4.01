/*
 * @Author: your name
 * @Date: 2023-08-17 20:24:09
 * @LastEditTime: 2023-08-21 13:58:55
 * @LastEditors: your name
 * @Description:
 * @FilePath: /MyFormProject/src/robot_costmap/src/map_deal/edt_environment.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "map_deal/edt_environment.h"

namespace map_deal
{
/* ============================== edt_environment ============================== */
void EDTEnvironment::init()
{
  cout<<"global_map_ size is "<<endl;
  cout<<global_map_->global_esdf_buffer_.size()<<endl;
  cout<<"edt_environment has been init"<<endl;
}

void EDTEnvironment::setMap(shared_ptr<global_map_deal> map)
{
  this->global_map_ = map;
  resolution_inv_ = global_map_->resolution_inv;

  // cout<<resolution_inv_<<endl;
}

void EDTEnvironment::evaluateEDTWithGrad(const Eigen::Vector2d &pos, const double &time,
                                         double &dist, Eigen::Vector2d &grad)
{
  vector<Eigen::Vector2d> pos_vec;
  Eigen::Vector2d diff;
  
  global_map_->getInterpolationData(pos, pos_vec, diff);

  /* ---------- value from surrounding position ---------- */
  double values[2][2]; // 存储周围8个点的距离值
  Eigen::Vector2d pt;
  for (int x = 0; x < 2; x++)
    for (int y = 0; y < 2; y++)
    {
      pt = pos_vec[x*2+y]; 
      // cout<<"pt is:"<<pt(0)<<"       "<<pt(1)<<endl;
      double d1 = global_map_->getDistance(pt);
      values[x][y] = d1;
    }

  /* ---------- use trilinear interpolation ---------- */
  // https://en.wikipedia.org/wiki/Trilinear_interpolation
  // double v00 = (1 - diff(0)) * values[0][0] + diff(0) * values[1][0];
  double v01 = (1 - diff(0)) * values[0][0] + diff(0) * values[1][0];
  double v10 = (1 - diff(0)) * values[0][1] + diff(0) * values[1][1];
  // double v11 = (1 - diff(0)) * values[0][1] + diff(0) * values[1][1];

  double v0 = (1 - diff(1)) * v01 + diff(1) * v10;
  // double v1 = (1 - diff(1)) * v01 + diff(1) * v10;

  dist = v0 ;

  grad[1] = v10-v01;
  grad[0] = (1 - diff[1]) * (values[1][0] - values[0][0]);
  
  grad[0] += diff[1] * (values[1][1] - values[0][1]);
  // grad[0] += (1 - diff[1])*  diff[2] * (values[1][0] - values[0][0]);
  // grad[0] += diff[2] * diff[1] * (values[1][1] - values[0][1]);

  grad[0] *= resolution_inv_;
}

// 用于safetyCallback中的安全检测, 不需要精度太高
double EDTEnvironment::evaluateCoarseEDT(const Eigen::Vector2d &pos, const double &time)
{
  double d1 = global_map_->getDistance(pos);
  return d1;
}
}

