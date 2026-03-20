//
// Created by Zhang Zhimeng on 2021/11/29.
//

#ifndef TRAJECTORY_GENERATOR_MINIMUM_SNAP_H
#define TRAJECTORY_GENERATOR_MINIMUM_SNAP_H

#include "utility/type.h"
#include "utility/math_function.h"
#include "iostream"
#include <Eigen/Dense>

// refer to: 《Polynomial Trajectory Planning for Aggressive
//             Quadrotor Flight in Dense Indoor Environments》
class MinimumSnap {
public:
    // 放置默认参数
    MinimumSnap();
    ~MinimumSnap(){};

    // MinimumSnap参数设定
    // 多项式阶数 1阶是速度  2阶是加速度  3阶是jerk  4阶是snap
    // 最大速度约束
    // 最大加速度约束
    void setParams(unsigned int order, double max_vel, double max_accel);

    VecXd AllocateTime(const MatXd &waypoint) const;

    MatXd SolveQPClosedForm(const VecXd &waypoints, const VecXd &vel,
                            const VecXd &acc, const VecXd &time) const;
    Vec2d GetPositionPolynomial(const MatXd &poly_coeff_mat, unsigned int k, double t);
    MatXd PickWaypoints(const std::vector<Eigen::Vector2d> &path_ptr) const;
    // 获取最终优化后的路径
    std::vector<Eigen::Vector2d> Minimum_Snap(std::vector<Eigen::Vector2d> path);
    // 根据路径点数目进行分段minimum snap
    std::vector<Eigen::Vector2d> SubsectionPath_Minimum_Snap(std::vector<Eigen::Vector2d> path);

    unsigned int GetPolyCoeffNum() const;

private:
    unsigned int order_;// =2: accel | =3: jerk | =4: snap
    double max_vel_;
    double max_accel_;

    std::vector<Eigen::Vector2d> current_path;          //当前的路径
};

#endif //TRAJECTORY_GENERATOR_MINIMUM_SNAP_H