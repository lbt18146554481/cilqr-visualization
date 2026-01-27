#pragma once
#ifndef __CILQR_OPTIMIZER_HPP
#define __CILQR_OPTIMIZER_HPP

#include "trajectory_optimizer.hpp"
#include "algorithm_config.hpp"
#include "cilqr_solver.hpp"
#include <memory>

namespace trajectory_optimizer {

class CILQROptimizer : public TrajectoryOptimizer {
public:
    explicit CILQROptimizer(const std::string& name, const cilqr::AlgorithmConfig& config)
        : TrajectoryOptimizer(name), config_(config) {}
    
    ~CILQROptimizer() override = default;

    bool Init() override {
        try {
            solver_ = std::make_unique<cilqr::CILQRSolver>(config_);
            return true;
        } catch (const std::exception& e) {
            return false;
        }
    }

    OptimizeResult Execute(
        const Vector5d& initial_state,
        const cilqr::ReferenceLine& reference_line,
        double reference_velocity,
        const std::vector<cilqr::RoutingLine>& obstacles,
        const Eigen::Vector2d& road_borders) override {
        
        OptimizeResult result;
        result.success = false;

        // 验证输入
        if (!ValidateInput(initial_state, reference_line, reference_velocity, obstacles, road_borders)) {
            result.status_message = "Invalid input parameters";
            return result;
        }

        try {
            // 调用底层求解器（保持原有接口）
            auto [u, x] = solver_->solve(
                initial_state,
                reference_line,
                reference_velocity,
                obstacles,
                road_borders
            );

            result.control_sequence = u;
            result.state_sequence = x;
            result.success = true;
            result.status_message = "CILQR optimization completed";
            result.iterations = config_.max_iter;  // 实际迭代次数可以从solver获取
            
            // 保存结果供 GetStateSequence 使用
            last_result_ = result;
            
        } catch (const std::exception& e) {
            result.status_message = std::string("CILQR failed: ") + e.what();
        }

        return result;
    }

    void GetStateSequence(std::vector<TrajectoryPoint>& trajectory_points) const override {
        trajectory_points.clear();
        
        if (!last_result_.success) {
            return;
        }

        const auto& x = last_result_.state_sequence;
        const auto& u = last_result_.control_sequence;
        
        double s_accum = 0.0;
        for (int i = 0; i < x.rows(); ++i) {
            TrajectoryPoint pt;
            pt.x = x(i, 0);
            pt.y = x(i, 1);
            pt.v = x(i, 2);
            pt.heading = x(i, 3);
            pt.kappa = x(i, 4);
            
            if (i < u.rows()) {
                pt.acceleration = u(i, 0);
                pt.dkappa = u(i, 1);
            }
            
            // 计算累积弧长
            if (i > 0) {
                double dx = pt.x - trajectory_points.back().x;
                double dy = pt.y - trajectory_points.back().y;
                s_accum += std::sqrt(dx * dx + dy * dy);
            }
            pt.s = s_accum;
            
            trajectory_points.push_back(pt);
        }
    }

    std::string GetType() const override {
        return "CILQR";
    }

    void Reset() override {
        TrajectoryOptimizer::Reset();
        last_result_ = OptimizeResult();
    }

    // CILQR特有接口：获取求解状态
    LQRSolveStatus GetSolveStatus() const {
        // 可以从solver获取状态，这里简化处理
        return LQRSolveStatus::RUNNING;
    }

private:
    cilqr::AlgorithmConfig config_;
    std::unique_ptr<cilqr::CILQRSolver> solver_;
};

} // namespace trajectory_optimizer

#endif
