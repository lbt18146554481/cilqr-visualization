/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2025-01-XX XX:XX:XX
 * @LastEditTime: 2025-01-XX XX:XX:XX
 * @FilePath: /algorithm/src/path_model_utils.cpp
 * Copyright 2025 puyu, All Rights Reserved.
 * 
 * 路径模型工具函数实现
 */

#include "path_model_utils.hpp"
#include <cmath>
#include <algorithm>

namespace cilqr {

namespace path_model {

Vector5d propagate(const Vector5d& cur_x, const Eigen::Vector2d& cur_u, double dt) {
    // 状态: [x, y, v, heading, kappa]
    // 控制: [acceleration, dkappa]
    double v = cur_x[2];
    double heading = cur_x[3];
    double kappa = cur_x[4];
    double a = cur_u[0];
    double dkappa = cur_u[1];

    Vector5d next_x;
    next_x << cur_x[0] + v * cos(heading) * dt,      // x
              cur_x[1] + v * sin(heading) * dt,      // y
              cur_x[2] + a * dt,                      // v
              cur_x[3] + v * kappa * dt,              // heading
              cur_x[4] + dkappa * dt;                 // kappa

    return next_x;
}

std::tuple<MatrixX5d, Eigen::MatrixX2d> get_derivatives(
    const MatrixX5d& x, const Eigen::MatrixX2d& u, double dt, uint32_t steps) {
    // A矩阵：∂f/∂x (5x5)
    // B矩阵：∂f/∂u (5x2)
    MatrixX5d df_dx(steps * 5, 5);
    Eigen::MatrixX2d df_du(steps * 5, 2);

    for (uint32_t i = 0; i < steps; ++i) {
        double v = x(i, 2);
        double heading = x(i, 3);
        double kappa = x(i, 4);

        // 初始化为单位矩阵
        df_dx.block<5, 5>(i * 5, 0).setIdentity();
        df_du.block<5, 2>(i * 5, 0).setZero();

        // A矩阵：∂f/∂x
        // ∂x/∂v = cos(heading) * dt
        df_dx(i * 5, 2) = cos(heading) * dt;
        // ∂x/∂heading = -v * sin(heading) * dt
        df_dx(i * 5, 3) = -v * sin(heading) * dt;

        // ∂y/∂v = sin(heading) * dt
        df_dx(i * 5 + 1, 2) = sin(heading) * dt;
        // ∂y/∂heading = v * cos(heading) * dt
        df_dx(i * 5 + 1, 3) = v * cos(heading) * dt;

        // ∂heading/∂v = kappa * dt
        df_dx(i * 5 + 3, 2) = kappa * dt;
        // ∂heading/∂kappa = v * dt
        df_dx(i * 5 + 3, 4) = v * dt;

        // B矩阵：∂f/∂u
        // ∂v/∂a = dt
        df_du(i * 5 + 2, 0) = dt;
        // ∂kappa/∂dkappa = dt
        df_du(i * 5 + 4, 1) = dt;
    }

    return std::make_tuple(df_dx, df_du);
}

double calc_kappa_from_reference(const ReferenceLine& ref_line, double s) {
    // 简化实现：使用数值微分计算曲率
    // 注意：算法层的ReferenceLine需要提供spline接口
    // 这里先使用简化版本，实际应该调用spline的calc_curvature方法
    
    if (s < 0) s = 0;
    if (s > ref_line.length()) s = ref_line.length();
    
    // 使用数值微分计算曲率
    double ds = 0.01;
    double s1 = std::max(0.0, s - ds);
    double s2 = std::min(ref_line.length(), s + ds);
    
    // 从longitude数组中找到对应的索引
    size_t idx1 = 0, idx2 = 0, idx = 0;
    for (size_t i = 0; i < ref_line.longitude.size(); ++i) {
        if (ref_line.longitude[i] <= s1) idx1 = i;
        if (ref_line.longitude[i] <= s) idx = i;
        if (ref_line.longitude[i] <= s2) idx2 = i;
        else break;
    }
    
    if (idx1 >= ref_line.x.size() || idx2 >= ref_line.x.size() || idx >= ref_line.x.size()) {
        return 0.0;
    }
    
    // 简化的曲率计算（使用三点法）
    double dx_ds = (ref_line.x[idx2] - ref_line.x[idx1]) / (2 * ds);
    double dy_ds = (ref_line.y[idx2] - ref_line.y[idx1]) / (2 * ds);
    double d2x_ds2 = (ref_line.x[idx2] - 2 * ref_line.x[idx] + ref_line.x[idx1]) / (ds * ds);
    double d2y_ds2 = (ref_line.y[idx2] - 2 * ref_line.y[idx] + ref_line.y[idx1]) / (ds * ds);
    
    double denominator = pow(dx_ds * dx_ds + dy_ds * dy_ds, 1.5);
    if (denominator < 1e-6) {
        return 0.0;
    }
    
    double kappa = (dx_ds * d2y_ds2 - dy_ds * d2x_ds2) / denominator;
    return kappa;
}

Eigen::Vector2d get_ellipsoid_obstacle_scales(const Eigen::Vector3d& obs_attr,
                                              double ego_pnt_radius) {
    double a = 0.5 * obs_attr[1] + obs_attr[2] * 6 + ego_pnt_radius;
    double b = 0.5 * obs_attr[0] + obs_attr[2] + ego_pnt_radius;
    return Eigen::Vector2d{a, b};
}

double ellipsoid_safety_margin(const Eigen::Vector2d& pnt, const Eigen::Vector3d& obs_state,
                               const Eigen::Vector2d& ellipse_ab) {
    Eigen::Vector2d elp_center = obs_state.head(2);
    double theta = obs_state[2];
    Eigen::Vector2d diff = pnt - elp_center;
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(theta), sin(theta), -sin(theta), cos(theta);
    Eigen::Vector2d pnt_std = rotation_matrix * diff;
    double result = 1 - (pow(pnt_std[0], 2) / pow(ellipse_ab[0], 2) +
                         pow(pnt_std[1], 2) / pow(ellipse_ab[1], 2));
    return result;
}

Eigen::Vector2d ellipsoid_safety_margin_derivatives(const Eigen::Vector2d& pnt,
                                                    const Eigen::Vector3d& obs_state,
                                                    const Eigen::Vector2d& ellipse_ab) {
    Eigen::Vector2d elp_center = obs_state.head(2);
    Eigen::Vector2d diff = pnt - elp_center;
    double theta = obs_state[2];
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(theta), sin(theta), -sin(theta), cos(theta);
    Eigen::Vector2d pnt_std = rotation_matrix * diff;

    Eigen::Vector2d res_over_pnt_std = {-2 * pnt_std[0] / pow(ellipse_ab[0], 2),
                                        -2 * pnt_std[1] / pow(ellipse_ab[1], 2)};
    Eigen::Matrix2d pnt_std_over_diff = rotation_matrix.transpose();
    Eigen::Matrix2d diff_over_pnt = Eigen::Matrix2d::Identity();
    Eigen::Vector2d res_over_pnt = diff_over_pnt * pnt_std_over_diff * res_over_pnt_std;

    return res_over_pnt;
}

}  // namespace path_model

}  // namespace cilqr
