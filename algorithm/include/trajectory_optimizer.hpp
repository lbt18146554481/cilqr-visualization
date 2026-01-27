/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2025-01-XX XX:XX:XX
 * @LastEditTime: 2025-01-XX XX:XX:XX
 * @FilePath: /algorithm/include/trajectory_optimizer.hpp
 * Copyright 2025 puyu, All Rights Reserved.
 * 
 * 轨迹优化器基类接口（参考 trajectory_smoother.cpp 的设计风格）
 */

#pragma once
#ifndef __TRAJECTORY_OPTIMIZER_HPP
#define __TRAJECTORY_OPTIMIZER_HPP

#include "types.hpp"
#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>

namespace trajectory_optimizer {

// 优化结果结构
struct OptimizeResult {
    Eigen::MatrixX2d control_sequence;  // 控制序列 [acceleration, dkappa]
    MatrixX5d state_sequence;           // 状态序列 [x, y, v, heading, kappa]
    bool success = false;               // 是否成功
    std::string status_message;         // 状态信息
    int iterations = 0;                 // 迭代次数
    double final_cost = 0.0;            // 最终代价
};

// 轨迹点结构（用于结果输出）
struct TrajectoryPoint {
    double x = 0.0;
    double y = 0.0;
    double v = 0.0;
    double heading = 0.0;
    double kappa = 0.0;
    double acceleration = 0.0;
    double dkappa = 0.0;
    double s = 0.0;  // 累积弧长
};

// 优化器基类（参考 trajectory_smoother.cpp 的设计风格）
class TrajectoryOptimizer {
public:
    explicit TrajectoryOptimizer(const std::string& name) : name_(name) {}
    virtual ~TrajectoryOptimizer() = default;

    // 初始化（类似 Reset，但用于首次设置）
    virtual bool Init() = 0;

    // 执行优化（类似 Execute，统一接口）
    virtual OptimizeResult Execute(
        const Vector5d& initial_state,
        const cilqr::ReferenceLine& reference_line,
        double reference_velocity,
        const std::vector<cilqr::RoutingLine>& obstacles,
        const Eigen::Vector2d& road_borders) = 0;

    // 获取状态序列（类似 get_xystate_seqs）
    virtual void GetStateSequence(std::vector<TrajectoryPoint>& trajectory_points) const = 0;

    // 获取名称
    const std::string& GetName() const { return name_; }

    // 获取类型
    virtual std::string GetType() const = 0;

    // 重置状态
    virtual void Reset() {}

    // 检查输入是否有效
    virtual bool ValidateInput(
        const Vector5d& initial_state,
        const cilqr::ReferenceLine& reference_line,
        double reference_velocity,
        const std::vector<cilqr::RoutingLine>& obstacles,
        const Eigen::Vector2d& road_borders) const {
        if (reference_line.size() == 0) {
            return false;
        }
        if (reference_velocity < 0) {
            return false;
        }
        return true;
    }

protected:
    std::string name_;
    OptimizeResult last_result_;
    
    // 禁止拷贝构造和赋值
    TrajectoryOptimizer(const TrajectoryOptimizer&) = delete;
    TrajectoryOptimizer& operator=(const TrajectoryOptimizer&) = delete;
};

} // namespace trajectory_optimizer

#endif
