/*
 * 简化的CILQR适配器实现
 * 使用 algorithm 目录下的算法（替代原本的 trajectory_smoother 算法）
 */

#include "cilqr_adapter_simple.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <memory>

// 包含 algorithm 目录下的算法头文件
// 使用 Decider 层（对齐 trajectory_smoother 架构）
// 注意：算法实现已移动到 trajectory_smoother
// #include "../trajectory_smoother/cilqr_lane_change/cilqr_iter_decider.h"
#include "../animation/include/common/algorithm_config.hpp"
#include "../animation/include/common/types.hpp"

// 包含neolix类型定义（使用mock headers）
#include "mock_headers/src/common/trajectory/trajectory_point.h"
#include "mock_headers/src/common/path/path_corridor.h"
#include "mock_headers/src/common/path/path_data.h"
#include "mock_headers/src/common/path/path_point.h"
#include "mock_headers/config/planning_config_manager.h"

using namespace cilqr;

// 使用 single_frame 的数据结构（用于结果存储）
namespace {
    struct PathModelState {
        double state_x{0.};
        double state_y{0.};
        double state_v{0.};
        double state_heading{0.};
        double state_kappa{0.};
    };
    
    struct PathModelControl {
        double control_a{0.};
        double control_dkappa{0.};
    };
}

// Impl类实现
class CilqrTrajectoryOptimizer::Impl {
private:
    cilqr::AlgorithmConfig config_;
    // 使用 Decider（对齐 trajectory_smoother 架构）
    std::unique_ptr<ceshi::planning::PathConstrainedIterLqrDeciderAML> cilqr_decider_;
    std::vector<PathModelState> optimized_states_;
    std::vector<PathModelControl> optimized_controls_;

public:
    Impl(const neolix::planning::ReferenceLine& /* reference_line */) {
        // 创建算法配置
        config_.delta_t = 0.2;
        // N 初始值，将从配置文件读取或根据 planned_time_length 计算
        config_.N = 20;
        config_.w_pos = 10.0;  // 增加位置权重，使轨迹更贴近参考轨迹
        config_.w_vel = 1.0;   // 恢复原始值，避免过度优化导致震荡
        config_.w_heading = 20.0;
        config_.w_kappa = 10.0;
        config_.w_acc = 0.5;    // 恢复原始值
        config_.w_dkappa = 25.0; // 恢复原始值
        config_.solve_type = "alm";
        config_.ref_x_weight = 50.0;  // 增加参考轨迹权重，确保转弯时能跟踪启发式轨迹
        config_.final_x_weight = 100.0;
        config_.uki_init = 20.0;
        config_.alm_rho_init = 20.0;
        config_.alm_gamma = 0.1;
        config_.max_rho = 100.0;
        config_.max_mu = 200.0;
        config_.obstacle_exp_q1 = 8.0;
        config_.obstacle_exp_q2 = 6.0;
        config_.state_exp_q1 = 3.0;
        config_.state_exp_q2 = 3.5;
        config_.use_last_solution = true;
        
        config_.max_iter = 15;
        config_.init_lamb = 5.0;
        config_.lamb_decay = 0.7;
        config_.lamb_amplify = 5.0;
        config_.max_lamb = 600.0;
        config_.convergence_threshold = 0.05;
        config_.accept_step_threshold = 0.5;  // 增加步长接受阈值，减少震荡
        
        // 尝试从配置读取参数
        try {
            auto* planning_config = config::PlanningConfigManager::Instance()->mutable_planning_config();
            const auto& cilqr_conf = planning_config->cilqr_trajectory_conf;
            const auto& heuristic_conf = planning_config->cilqr_heuristic_trajectory_conf;
            
            config_.delta_t = cilqr_conf.dt;
            
            // 从配置文件读取 horizon（预测时域长度）
            // 如果没有设置，则根据 planned_time_length 计算
            if (cilqr_conf.horizon > 0) {
                config_.N = static_cast<uint32_t>(cilqr_conf.horizon);
            } else {
                // 根据 planned_time_length 和 delta_t 计算 N
                double planned_time_length = heuristic_conf.planned_time_length;
                config_.N = static_cast<uint32_t>(std::ceil(planned_time_length / config_.delta_t));
            }
            
            // 从配置文件读取权重，但确保最小值，避免跟踪不足
            // 位置权重：至少10.0，确保转弯时能跟踪参考轨迹
            config_.w_pos = std::max(cilqr_conf.position_weight, 10.0);
            // 速度权重：使用配置值，不强制最小值，避免过度优化导致震荡
            config_.w_vel = cilqr_conf.speed_weight;
            // 航向权重：至少20.0，转弯时航向跟踪很重要
            config_.w_heading = std::max(cilqr_conf.heading_weight, 20.0);
            // 曲率权重：至少10.0，确保曲率跟踪
            config_.w_kappa = std::max(cilqr_conf.kappa_weight, 10.0);
            // 加速度权重：使用配置值，不强制最小值
            config_.w_acc = cilqr_conf.acc_weight;
            // 曲率变化率权重：使用配置值，不强制最小值
            config_.w_dkappa = cilqr_conf.dkappa_weight;
            // ref_x_weight：增加参考轨迹权重，确保转弯时能跟踪启发式轨迹
            // 如果配置值太小，使用更大的默认值（至少50.0）
            config_.ref_x_weight = std::max(config_.ref_x_weight, 50.0);
            
            std::cout << "CILQR Config: N=" << config_.N 
                      << ", delta_t=" << config_.delta_t 
                      << ", trajectory_length=" << (config_.N * config_.delta_t) << "s" << std::endl;
            std::cerr << "CILQR Config: N=" << config_.N 
                      << ", delta_t=" << config_.delta_t 
                      << ", trajectory_length=" << (config_.N * config_.delta_t) << "s" << std::endl;
        } catch (...) {
            // 使用默认值，但确保 ref_x_weight 足够大
            config_.ref_x_weight = std::max(config_.ref_x_weight, 50.0);
            // 如果读取配置失败，尝试根据 planned_time_length 计算 N
            try {
                auto* planning_config = config::PlanningConfigManager::Instance()->mutable_planning_config();
                const auto& heuristic_conf = planning_config->cilqr_heuristic_trajectory_conf;
                double planned_time_length = heuristic_conf.planned_time_length;
                config_.N = static_cast<uint32_t>(std::ceil(planned_time_length / config_.delta_t));
                std::cerr << "Fallback: Calculated N=" << config_.N 
                          << " from planned_time_length=" << planned_time_length << "s" << std::endl;
            } catch (...) {
                // 使用默认值
            }
        }
        
        config_.width = 2.0;
        config_.length = 4.5;
        config_.velo_max = 10.0;
        config_.velo_min = 0.0;
        config_.acc_max = 3.0;
        config_.acc_min = -3.0;
        config_.kappa_max = 0.3;
        config_.kappa_min = -0.3;
        config_.dkappa_max = 0.1;
        config_.dkappa_min = -0.1;
        config_.d_safe = 0.8;
        
       
        cilqr_decider_ = std::make_unique<ceshi::planning::PathConstrainedIterLqrDeciderAML>(
            "single frame cilqr optimizer", config_);

    }
    
    bool Init() {
     
        return cilqr_decider_ != nullptr;
    }
    
    bool Optimize(const neolix::planning::TrajectoryPoint& init_point,
                  const neolix::planning::PathCorridor& path_corridor) {
        try {
            // 1. 转换初始状态为5维向量
            Vector5d x0;
            x0 << init_point.x(),
                  init_point.y(),
                  init_point.velocity(),
                  init_point.theta(),
                  init_point.kappa();
            
            // 2. 提取参考轨迹并转换为 algorithm 的 ReferenceLine
            const auto& heuristic_traj = path_corridor.heuristic_trajectory();
            const auto& traj_points = heuristic_traj.trajectory_points();
            
            cilqr::ReferenceLine ref_line;
            double s_accum = 0.0;
            for (size_t i = 0; i < traj_points.size(); ++i) {
                const auto& pt = traj_points[i];
                ref_line.x.push_back(pt.x());
                ref_line.y.push_back(pt.y());
                ref_line.yaw.push_back(pt.theta());
                
                if (i > 0) {
                    double dx = pt.x() - traj_points[i-1].x();
                    double dy = pt.y() - traj_points[i-1].y();
                    s_accum += std::hypot(dx, dy);
                }
                ref_line.longitude.push_back(s_accum);
            }
            
            // 3. 计算参考速度（使用轨迹点的平均速度或初始速度）
            double ref_velo = init_point.velocity();
            if (!traj_points.empty()) {
                double sum_velo = 0.0;
                for (const auto& pt : traj_points) {
                    sum_velo += pt.velocity();
                }
                ref_velo = sum_velo / traj_points.size();
            }
            
            // 4. 提取道路边界
            const auto& sl_corridor = path_corridor.path_sl_corridor();
            const auto& sl_bound_pts = sl_corridor.sl_bound_points();
            Eigen::Vector2d road_borders;
            if (!sl_bound_pts.empty()) {
                // 找到最大和最小的边界值
                double max_bound = sl_bound_pts[0].hard_ub;
                double min_bound = sl_bound_pts[0].hard_lb;
                for (const auto& sl_pt : sl_bound_pts) {
                    if (sl_pt.hard_ub > max_bound) max_bound = sl_pt.hard_ub;
                    if (sl_pt.hard_lb < min_bound) min_bound = sl_pt.hard_lb;
                }
                // road_borders[0] 是上边界（左边界，正数）
                // road_borders[1] 是下边界（右边界，负数）
                road_borders << max_bound, min_bound;
            } else {
                // 默认边界
                road_borders << 3.6, -3.6;
            }
            
            // 5. 障碍物预测轨迹（single_frame 没有，传入空）
            std::vector<cilqr::RoutingLine> obs_preds;  // 空列表
            
            // 6. 直接调用 Decider 的 Execute（对齐 trajectory_smoother.cpp 第48行）
            if (!cilqr_decider_) {
                std::cerr << "CILQR Decider not initialized" << std::endl;
                return false;
            }
            
            auto [u, x] = cilqr_decider_->Execute(
                x0, ref_line, ref_velo, obs_preds, road_borders);
            
            if (x.rows() == 0 || u.rows() == 0) {
                std::cerr << "Optimization failed: empty result" << std::endl;
                return false;
            }
            
            // 输出轨迹信息用于调试
            std::cout << "DEBUG: Optimized trajectory: " << x.rows() << " points, " 
                      << "heuristic trajectory: " << traj_points.size() << " points" << std::endl;
            std::cout << "DEBUG: Optimized trajectory length: " << (x.rows() * config_.delta_t) << "s" << std::endl;
            std::cerr << "Optimized trajectory: " << x.rows() << " points, " 
                      << "heuristic trajectory: " << traj_points.size() << " points" << std::endl;
            std::cerr << "Optimized trajectory length: " << (x.rows() * config_.delta_t) << "s" << std::endl;
            
            // 7. 转换结果
            optimized_states_.clear();
            optimized_controls_.clear();
            
            for (int i = 0; i < x.rows(); ++i) {
                PathModelState state;
                state.state_x = x(i, 0);
                state.state_y = x(i, 1);
                state.state_v = x(i, 2);
                state.state_heading = x(i, 3);
                state.state_kappa = x(i, 4);
                optimized_states_.push_back(state);
            }
            
            for (int i = 0; i < u.rows(); ++i) {
                PathModelControl control;
                control.control_a = u(i, 0);
                control.control_dkappa = u(i, 1);
                optimized_controls_.push_back(control);
            }
            
            std::cerr << "Algorithm optimize: states=" << optimized_states_.size() 
                      << ", controls=" << optimized_controls_.size() << std::endl;
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "Exception in Optimize: " << e.what() << std::endl;
            return false;
        }
    }
    
public:
    bool FillPathData(neolix::planning::PathData* path_data) {
        if (!path_data) return false;
        
        auto* path_points = path_data->mutable_discretized_path()->mutable_path_points();
        path_points->clear();
        
        double s_accum = 0.0;
        std::cout << "DEBUG: FillPathData: optimized_states_.size()=" << optimized_states_.size() 
                  << ", expected N+1=" << (config_.N + 1) << std::endl;
        std::cerr << "FillPathData: optimized_states_.size()=" << optimized_states_.size() 
                  << ", expected N+1=" << (config_.N + 1) << std::endl;
        for (size_t i = 0; i < optimized_states_.size(); ++i) {
            neolix::planning::PathPoint pt;
            pt.set_x(optimized_states_[i].state_x);
            pt.set_y(optimized_states_[i].state_y);
            pt.set_theta(optimized_states_[i].state_heading);
            pt.set_kappa(optimized_states_[i].state_kappa);
            
            if (i == 0 || i == optimized_states_.size() - 1) {
                std::cerr << "FillPathData[" << i << "]: kappa=" << optimized_states_[i].state_kappa << std::endl;
            }
            
            if (i > 0) {
                double dx = optimized_states_[i].state_x - optimized_states_[i-1].state_x;
                double dy = optimized_states_[i].state_y - optimized_states_[i-1].state_y;
                s_accum += std::hypot(dx, dy);
            }
            pt.set_s(s_accum);
            
            if (i == 0 || i == optimized_states_.size() - 1) {
                std::cerr << "FillPathData[" << i << "]: x=" << pt.x() 
                          << ", y=" << pt.y() 
                          << ", theta=" << pt.theta() 
                          << ", kappa=" << pt.kappa() << std::endl;
            }
            
            path_points->push_back(pt);
        }
        
        std::cerr << "FillPathData: path_points->size()=" << path_points->size() << std::endl;
        return true;
    }
    
public:
    void ExtractDebug(neolix::planning::CilqrPathOptimizerDebug* debug) {
        if (!debug) return;
        
        auto& solution_info = debug->solution_info;
        solution_info.final_x_traj.clear();
        solution_info.final_u_traj.clear();
        
        // 直接从配置读取 dt（与 planning_test_execute.cpp 保持一致）
        double dt = 0.2;  // 默认值
        try {
            auto* planning_config = config::PlanningConfigManager::Instance()->mutable_planning_config();
            dt = planning_config->cilqr_trajectory_conf.dt;
        } catch (...) {
            dt = 0.2;  // 如果读取失败，使用默认值
        }
        
        // 确保 dt 有效
        if (dt <= 1e-6) {
            dt = 0.2;
        }
        
        // 提取状态轨迹和控制轨迹
        for (size_t i = 0; i < optimized_states_.size(); ++i) {
            Eigen::Matrix<double, 6, 1> x_vec;
            x_vec(0) = optimized_states_[i].state_x;
            x_vec(1) = optimized_states_[i].state_y;
            x_vec(2) = optimized_states_[i].state_v;
            x_vec(3) = optimized_states_[i].state_heading;
            x_vec(4) = optimized_states_[i].state_kappa;
            x_vec(5) = (i < optimized_controls_.size()) ? optimized_controls_[i].control_a : 0.0;
            solution_info.final_x_traj.push_back(x_vec);
            
            if (i < optimized_controls_.size()) {
                Eigen::Matrix<double, 2, 1> u_vec;
                u_vec(0) = optimized_controls_[i].control_dkappa;
                
                // 计算jerk（与优化算法保持一致：使用前向差分）
                double jerk = 0.0;
                if (i < optimized_controls_.size() - 1 && dt > 1e-6) {
                    // 前向差分：jerk = (a[i+1] - a[i]) / dt
                    // 与优化算法中的计算方式一致
                    jerk = (optimized_controls_[i+1].control_a - optimized_controls_[i].control_a) / dt;
                } else if (i > 0 && dt > 1e-6) {
                    // 如果已经是最后一步，使用后向差分作为备选
                    jerk = (optimized_controls_[i].control_a - optimized_controls_[i-1].control_a) / dt;
                }
                // 否则 jerk = 0.0（保持默认值）
                
                u_vec(1) = jerk;
                solution_info.final_u_traj.push_back(u_vec);
            }
        }
    }

};

// CilqrTrajectoryOptimizer公共方法实现
CilqrTrajectoryOptimizer::CilqrTrajectoryOptimizer(
    const neolix::planning::ReferenceLine& reference_line)
    : pimpl_(std::make_unique<Impl>(reference_line)) {
}

CilqrTrajectoryOptimizer::~CilqrTrajectoryOptimizer() = default;

bool CilqrTrajectoryOptimizer::Init() {
    return pimpl_->Init();
}

bool CilqrTrajectoryOptimizer::Optimize(
    const neolix::planning::TrajectoryPoint& init_point,
    const neolix::planning::PathCorridor& path_corridor) {
    return pimpl_->Optimize(init_point, path_corridor);
}

bool CilqrTrajectoryOptimizer::FillPathData(neolix::planning::PathData* path_data) {
    return pimpl_->FillPathData(path_data);
}

void CilqrTrajectoryOptimizer::ExtractDebug(neolix::planning::CilqrPathOptimizerDebug* debug) {
    pimpl_->ExtractDebug(debug);
}
