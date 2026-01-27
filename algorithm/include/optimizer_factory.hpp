#pragma once
#ifndef __OPTIMIZER_FACTORY_HPP
#define __OPTIMIZER_FACTORY_HPP

#include "trajectory_optimizer.hpp"
#include "cilqr_optimizer.hpp"
#include "algorithm_config.hpp"
#include <memory>
#include <string>

namespace trajectory_optimizer {

class OptimizerFactory {
public:
    // 创建优化器
    static std::unique_ptr<TrajectoryOptimizer> Create(
        const std::string& optimizer_type,
        const std::string& name,
        const cilqr::AlgorithmConfig& config) {
        
        if (optimizer_type == "cilqr" || optimizer_type == "CILQR") {
            auto optimizer = std::make_unique<CILQROptimizer>(name, config);
            if (optimizer->Init()) {
                return optimizer;
            }
        }
        // 未来可以添加其他优化器类型
        // else if (optimizer_type == "mpc" || optimizer_type == "MPC") {
        //     auto optimizer = std::make_unique<MPCOptimizer>(name, mpc_config);
        //     if (optimizer->Init()) {
        //         return optimizer;
        //     }
        // }
        
        return nullptr;
    }

    // 创建默认名称的优化器
    static std::unique_ptr<TrajectoryOptimizer> Create(
        const std::string& optimizer_type,
        const cilqr::AlgorithmConfig& config) {
        return Create(optimizer_type, optimizer_type + " optimizer", config);
    }
};

} // namespace trajectory_optimizer

#endif
