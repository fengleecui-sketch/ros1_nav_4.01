/*
 * @Author: your name
 * @Date: 2023-08-24 15:29:00
 * @LastEditTime: 2023-08-24 17:06:35
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/include/math_tools/curve_fitting.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __CURVE_FITTING_H
#define __CURVE_FITTING_H

#include "iostream"
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

namespace math_tools
{
class curve_fitting
{
private:
  /* data */

public:
  curve_fitting(/* args */);
  ~curve_fitting();

  // 求解一元n次方程系数
  // 输入x y的矩阵
  VectorXd calCubicCoefficient(MatrixXd xy);

  // 求导数
  double calDerivative(MatrixXd xy,double x);
};
}


#endif  //