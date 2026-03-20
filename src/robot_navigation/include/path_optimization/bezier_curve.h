/*
 * @Author: your name
 * @Date: 2023-03-21 09:57:25
 * @LastEditTime: 2024-01-09 21:18:37
 * @LastEditors: your name
 * @Description:
 * @FilePath: /MyFormProject/src/robot_navigation/include/path_optimization/bezier_curve.h
 * 可以输入预定的版权声明、个性签名、空行等
 */
#ifndef __BEZIERCURVE_H
#define __BEZIERCURVE_H

#include <iostream>
#include "ros/ros.h"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include "utility/math_function.h"

using namespace std;
using namespace Eigen;

class BEZIER
{
public:
	BEZIER();		 // 构造函数
	~BEZIER(){}; // 析构函数

	// 这里的速度取标量没有取矢量
	void setParams(double _max_vel, double _time_resolution, unsigned int _sub);
	std::vector<Eigen::Vector2d> BezierPath(std::vector<Eigen::Vector2d> path); // 贝塞尔最终优化的曲线
	// 分段优化的贝塞尔曲线
	std::vector<Eigen::Vector2d> SubsectionPath_Bezier(std::vector<Eigen::Vector2d> path);
	/**
	 * 贝塞尔公式
	 * @param path
	 * @return
	 */
	Eigen::Vector2d bezierCommon(std::vector<Eigen::Vector2d> path, double t);
	// 最终的贝塞尔曲线的函数
	std::vector<Eigen::Vector2d> BezierCurve(std::vector<Eigen::Vector2d> path);

	// 智能指针
  typedef shared_ptr<BEZIER> Ptr;

private:
	double max_vel_;				// 最大速度限制  通过最大速度限制求出整段路径所用时间是多少
	double time_resolution; // 时间分辨率，时间分辨率越高，求出的曲线越丝滑
	double inv_resolution;	// 时间分辨率的倒数
	double t;								// 运行完全部的路程所需要的时间
	unsigned int sub = 3;		// 贝塞尔阶数    不设定使用默认参数3
};

#endif //__BEZIERCURVE_H
