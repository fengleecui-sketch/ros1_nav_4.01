/*
 * @Author: your name
 * @Date: 2023-08-16 10:37:04
 * @LastEditTime: 2023-08-16 11:05:09
 * @LastEditors: your name
 * @Description: 
 * @FilePath: /MyFormProject/src/robot_navigation/include/path_optimization/BSpline.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
//
// Created by chh3213 on 2022/11/25.
//

#ifndef __BSPLINE_H
#define __BSPLINE_H
#include <iostream>
#include <Eigen/Dense>
#include<vector>
#include<cmath>
#include<algorithm>
using namespace std;
using namespace Eigen;

class BSpline
{
private:
  /* data */
public:
  double baseFunction(int i, int k, double u, vector<double>node_vector);

  vector<double> u_quasi_uniform(int n,int k);
  vector<double> u_piecewise_B_Spline(int n,int k);

  BSpline(/* args */);
  ~BSpline();
};


#endif //CHHROBOTICS_CPP_BSPLINE_H

