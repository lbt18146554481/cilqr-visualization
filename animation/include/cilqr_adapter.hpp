/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2025-01-XX XX:XX:XX
 * @LastEditTime: 2025-01-XX XX:XX:XX
 * @FilePath: /animation/include/cilqr_adapter.hpp
 * Copyright 2025 puyu, All Rights Reserved.
 * 
 * CILQR算法适配器层：连接算法层和可视化层
 */

#pragma once
#ifndef __CILQR_ADAPTER_HPP
#define __CILQR_ADAPTER_HPP

#include "utils.hpp"  // 可视化层的类型定义
#include "global_config.hpp"
#include "../../algorithm/include/trajectory_optimizer.hpp"
#include "../../algorithm/include/optimizer_factory.hpp"
#include "../../algorithm/include/algorithm_config.hpp"
#include "../../algorithm/include/types.hpp"

#include <Eigen/Core>
#include <tuple>
#include <memory>

class CILQRAdapter {
  public:
    CILQRAdapter() = delete;
    explicit CILQRAdapter(const GlobalConfig* const config);
    ~CILQRAdapter() {}

    // 适配器接口：使用可视化层的类型，内部转换为算法层类型
    std::tuple<Eigen::MatrixX2d, MatrixX5d> solve(
        const Vector5d& x0,  // [x, y, v, heading, kappa]
        const ReferenceLine& ref_waypoints,  // 可视化层的ReferenceLine
        double ref_velo,
        const std::vector<RoutingLine>& obs_preds,  // 可视化层的RoutingLine
        const Eigen::Vector2d& road_boaders);

  private:
    // 将可视化层的ReferenceLine转换为算法层的ReferenceLine
    cilqr::ReferenceLine convert_reference_line(const ReferenceLine& vis_ref);
    
    // 将可视化层的RoutingLine转换为算法层的RoutingLine
    std::vector<cilqr::RoutingLine> convert_routing_lines(const std::vector<RoutingLine>& vis_routing);
    
    // 从GlobalConfig创建AlgorithmConfig
    cilqr::AlgorithmConfig create_algorithm_config(const GlobalConfig* const config);

    // 使用优化器基类接口（参考 trajectory_smoother.cpp 的设计）
    std::unique_ptr<trajectory_optimizer::TrajectoryOptimizer> optimizer_;
    cilqr::AlgorithmConfig algo_config_;
};

#endif
