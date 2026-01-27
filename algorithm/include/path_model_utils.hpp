#pragma once
#ifndef __PATH_MODEL_UTILS_HPP
#define __PATH_MODEL_UTILS_HPP

#include "types.hpp"
#include <Eigen/Core>
#include <tuple>
#include <chrono>

namespace cilqr {

// 简单的计时器类
class TicToc {
  public:
    TicToc(void) { tic(); }
    void tic(void) { start = std::chrono::system_clock::now(); }
    double toc(void) {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count();
    }
  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

namespace path_model {

// 路径模型传播函数（5维状态空间）
// 状态: [x, y, v, heading, kappa]
// 控制: [acceleration, dkappa]
Vector5d propagate(const Vector5d& cur_x, const Eigen::Vector2d& cur_u, double dt);

// 路径模型线性化（计算A和B矩阵）
std::tuple<MatrixX5d, Eigen::MatrixX2d> get_derivatives(
    const MatrixX5d& x, const Eigen::MatrixX2d& u, double dt, uint32_t steps);

// 从参考线计算曲率
double calc_kappa_from_reference(const ReferenceLine& ref_line, double s);

// 辅助函数
template <typename T>
int sign(T num) {
    return (num < 0) ? -1 : 1;
}

// 障碍物相关函数
Eigen::Vector2d get_ellipsoid_obstacle_scales(const Eigen::Vector3d& obs_attr,
                                              double ego_pnt_radius = 0);
double ellipsoid_safety_margin(const Eigen::Vector2d& pnt, const Eigen::Vector3d& obs_state,
                               const Eigen::Vector2d& ellipse_ab);
Eigen::Vector2d ellipsoid_safety_margin_derivatives(const Eigen::Vector2d& pnt,
                                                    const Eigen::Vector3d& obs_state,
                                                    const Eigen::Vector2d& ellipse_ab);

}  // namespace path_model

}  // namespace cilqr

#endif
