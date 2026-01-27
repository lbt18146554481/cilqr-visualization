/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2025-01-XX XX:XX:XX
 * @LastEditTime: 2025-01-XX XX:XX:XX
 * @FilePath: /algorithm/include/types.hpp
 * Copyright 2025 puyu, All Rights Reserved.
 * 
 * 算法模块的基础类型定义
 */

#pragma once
#ifndef __TYPES_HPP
#define __TYPES_HPP

#include <Eigen/Core>
#include <vector>

// 5维状态空间类型别名（用于路径模型）
using Vector5d = Eigen::Matrix<double, 5, 1>;
using Matrix5d = Eigen::Matrix<double, 5, 5>;
using MatrixX5d = Eigen::Matrix<double, Eigen::Dynamic, 5>;

// 枚举类型定义
enum class BoundType { UPPER, LOWER };
enum class SolveType { BARRIER, ALM };

enum class LQRSolveStatus {
    RUNNING,
    CONVERGED,
    BACKWARD_PASS_FAIL,
    FORWARD_PASS_FAIL,
    FORWARD_PASS_SMALL_STEP,
};

namespace cilqr {

// 参考线结构（算法层使用）
struct ReferenceLine {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    std::vector<double> longitude;
    
    double length() const {
        if (longitude.empty()) return 0.0;
        return longitude.back();
    }
    
    size_t size() const {
        return x.size();
    }
};

// 路由线结构（障碍物预测轨迹）
struct RoutingLine {
    size_t size;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    
    Eigen::Vector3d operator[](size_t index) const {
        if (index >= x.size() || index >= y.size() || index >= yaw.size()) {
            throw std::out_of_range("Index out of range");
        }
        return Eigen::Vector3d{x[index], y[index], yaw[index]};
    }
};

}  // namespace cilqr

#endif
