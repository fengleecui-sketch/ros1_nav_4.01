/*
 * @Author: your name
 * @Date: 2023-08-24 15:28:51
 * @LastEditTime: 2023-08-24 23:36:55
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/src/math_tools/curve_fitting.cpp
 * 可以输入预定的版权声明、个性签名、空行等
 */
#include "math_tools/curve_fitting.h"

namespace math_tools
{
curve_fitting::curve_fitting(/* args */)
{
}

curve_fitting::~curve_fitting()
{
}

VectorXd curve_fitting::calCubicCoefficient(MatrixXd xy)
{
  // 初始化x y向量
  VectorXd x(xy.rows());
  VectorXd y(xy.rows());

  // 进行赋值
  for(int i=0;i<xy.rows();i++)
  {
    x(i) = xy(i,0);
    y(i) = xy(i,1);
  }

  // 解算一元方程系数 c0 c1x c2x^2 c3x^3 ......
  // 构建A矩阵
  MatrixXd A = MatrixXd::Zero(xy.rows(),xy.rows()); 
  // 对A赋值
  for(int i=0;i<xy.rows();i++)
  {
    for(int j=0;j<xy.rows();j++)
    {
      A(i,j) = pow(x(i),j);
    }
  }

  // 定义k向量，即c0 c1 c2 c3的向量
  VectorXd k(xy.rows());
  k = A.inverse()*y;

  return k;

}

double curve_fitting::calDerivative(MatrixXd xy,double x)
{
  VectorXd k = calCubicCoefficient(xy);

  // 初始化x y向量
  VectorXd x_v(xy.rows());

  // 进行赋值
  for(int i=0;i<xy.rows();i++)
  {
    x_v(i) = xy(i,0);
  }  

  // 求导数 derivative
  VectorXd d_k = VectorXd::Zero(k.size()-1);
  VectorXd d_x = VectorXd::Zero(x_v.size()-1);

  // // 获取deriv的系数
  for(int i = k.size()-2;i>=0;i--)
  {
    d_k(i) = (i+1)*k(i+1);
    // 获取导数的x向量
    d_x(i) = pow(x,i);
  }

  double max_num = 1000.0;
  double min_num = -1000.0;
  double derivatives = d_k.transpose()*d_x;

  // cout<<"derivatives"<<derivatives<<endl;

  double last;
  if(!isnan(derivatives))
  {
    max_num = max(derivatives,max_num);
    min_num = min(derivatives,min_num);
    last = derivatives;
  }

  if(isnan(derivatives))
  {
    derivatives = last;
    if(derivatives < 0)
    {
      derivatives = min_num;
    }
    else{
      derivatives = max_num;
    }
  }

  // cout<<"max_num"<<max_num<<endl;
  
  return derivatives;
}  
}
