/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2025-01-XX XX:XX:XX
 * @LastEditTime: 2025-01-XX XX:XX:XX
 * @FilePath: /algorithm/src/cilqr_solver.cpp
 * Copyright 2025 puyu, All Rights Reserved.
 * 
 * 5维路径模型版本的CILQR求解器实现（算法层）
 */

#include "cilqr_solver.hpp"
#include "path_model_utils.hpp"

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <limits>
#include <cmath>

// 在命名空间外包含 spdlog，避免命名空间冲突
#include <spdlog/spdlog.h>

namespace cilqr {

CILQRSolver::CILQRSolver(const AlgorithmConfig& config) : config_(config), is_first_solve(true) {
    SPDLOG_DEBUG("CILQRSolver constructor: start");
    dt = config.delta_t;
    N = config.N;
    nx = 5;  // 5维状态空间
    nu = 2;  // 2维控制输入

    SPDLOG_DEBUG("CILQRSolver constructor: initializing state_weight");
    state_weight = Matrix5d::Zero();
    state_weight(0, 0) = config.w_pos;
    state_weight(1, 1) = config.w_pos;
    state_weight(2, 2) = config.w_vel;
    state_weight(3, 3) = config.w_heading;
    state_weight(4, 4) = config.w_kappa;
    
    ctrl_weight = Eigen::Matrix2d::Zero();
    ctrl_weight(0, 0) = config.w_acc;
    ctrl_weight(1, 1) = config.w_dkappa;

    use_last_solution = config.use_last_solution;
    if (config.solve_type == "alm") {
        solve_type = SolveType::ALM;
    } else if (config.solve_type == "barrier") {
        solve_type = SolveType::BARRIER;
    } else {
        SPDLOG_ERROR("The solve_type can only be barrier or alm, default set to barrier!");
        solve_type = SolveType::BARRIER;
    }

    if (solve_type == SolveType::ALM) {
        alm_rho_init = config.alm_rho_init;
        alm_gamma = config.alm_gamma;
        max_rho = config.max_rho;
        max_mu = config.max_mu;
        uki_init = config.uki_init;
        uki = uki_init;
    }

    max_iter = config.max_iter;
    init_lamb = config.init_lamb;
    lamb_decay = config.lamb_decay;
    lamb_amplify = config.lamb_amplify;
    max_lamb = config.max_lamb;
    convergence_threshold = config.convergence_threshold;
    accept_step_threshold = config.accept_step_threshold;

    width = config.width;
    length = config.length;
    velo_max = config.velo_max;
    velo_min = config.velo_min;
    acc_max = config.acc_max;
    acc_min = config.acc_min;
    kappa_max = config.kappa_max;
    kappa_min = config.kappa_min;
    dkappa_max = config.dkappa_max;
    dkappa_min = config.dkappa_min;
    
    obs_attr = {width, length, config.d_safe};

    ref_x_weight = config.ref_x_weight;
    final_x_weight = config.final_x_weight;
    control_a_cost = config.w_acc;
    control_dkappa_cost = config.w_dkappa;

    last_solve_J = std::numeric_limits<double>::max();
    SPDLOG_DEBUG("CILQRSolver constructor: completed successfully");
}

std::tuple<Eigen::MatrixX2d, MatrixX5d> CILQRSolver::solve(
    const Vector5d& x0, const ReferenceLine& ref_waypoints, double ref_velo,
    const std::vector<RoutingLine>& obs_preds, const Eigen::Vector2d& road_boaders) {
    
    // 初始化ALM参数（必须在第一次调用时初始化）
    size_t num_obstacles = obs_preds.size();
    size_t num_constraints_per_step = 8 + 2 * num_obstacles;  // 8个基本约束 + 2*障碍物数
    
    if (solve_type == SolveType::ALM) {
        if (!use_last_solution || (use_last_solution && is_first_solve)) {
            uki = uki_init;  // 重置uki为初始值
            alm_mu = Eigen::MatrixXd::Zero(N, num_constraints_per_step);
            alm_mu_next = Eigen::MatrixXd::Zero(N, num_constraints_per_step);
        }
    } else {
        // 非ALM方法也需要初始化（防止未初始化访问）
        alm_mu = Eigen::MatrixXd::Zero(N, num_constraints_per_step);
        alm_mu_next = Eigen::MatrixXd::Zero(N, num_constraints_per_step);
    }
    
    current_solve_status = LQRSolveStatus::RUNNING;
    Eigen::MatrixX2d u;
    MatrixX5d x;
    
    if (!is_first_solve && use_last_solution) {
        std::tie(u, x) = get_init_traj_increment(x0);
    } else {
        std::tie(u, x) = get_init_traj(x0);
        is_first_solve = false;
    }

    double J = get_total_cost(u, x, ref_waypoints, ref_velo, obs_preds, road_boaders);
    SPDLOG_DEBUG("Initial cost: {:.6f}", J);
    double lamb = init_lamb;
    cilqr::TicToc solve_time;
    bool is_exceed_max_itr = true;
    bool iter_effective_flag = false;
    
    for (uint32_t itr = 0; itr < max_iter; ++itr) {
        auto [new_u, new_x, new_J] = iter_step(u, x, lamb, ref_waypoints, ref_velo, obs_preds,
                                               road_boaders, iter_effective_flag);
        if (iter_effective_flag) {
            x = new_x;
            u = new_u;
            J = new_J;
        }

        if (current_solve_status == LQRSolveStatus::BACKWARD_PASS_FAIL ||
            current_solve_status == LQRSolveStatus::FORWARD_PASS_FAIL) {
            lamb = std::max(lamb_amplify, lamb * lamb_amplify);
        } else if (current_solve_status == LQRSolveStatus::RUNNING) {
            lamb *= lamb_decay;
        }

        if (lamb > max_lamb) {
            SPDLOG_WARN("Regularization reached the maximum. itr: {}, final cost: {:.3f}, "
                       "cost time {:.2f} ms",
                       itr, J, solve_time.toc() * 1000);
            is_exceed_max_itr = false;
            break;
        } else if (current_solve_status == LQRSolveStatus::CONVERGED) {
            SPDLOG_INFO("Optimization has converged. itr: {}, final cost: {:.3f}, cost time "
                       "{:.2f} ms",
                       itr, J, solve_time.toc() * 1000);
            is_exceed_max_itr = false;
            break;
        }
        
        // ALM参数更新
        if (solve_type == SolveType::ALM && iter_effective_flag && 
            current_solve_status != LQRSolveStatus::CONVERGED) {
            alm_mu = alm_mu_next;
            uki = std::min(uki * 2.0, max_rho);
            SPDLOG_DEBUG("ALM update: uki={:.2f}", uki);
        }
    }

    last_solve_u = u;
    last_solve_x = x;
    last_solve_J = J;

    if (is_exceed_max_itr) {
        SPDLOG_WARN("Iteration reached the maximum {}. Final cost: {:.3f}", max_iter, J);
    }

    return std::make_tuple(u, x);
}

std::tuple<Eigen::MatrixX2d, MatrixX5d> CILQRSolver::get_init_traj(
    const Vector5d& x0) {
    Eigen::MatrixX2d init_u = Eigen::MatrixX2d::Zero(N, 2);
    MatrixX5d init_x = const_velo_prediction(x0, N, dt);
    return std::make_tuple(init_u, init_x);
}

std::tuple<Eigen::MatrixX2d, MatrixX5d> CILQRSolver::get_init_traj_increment(
    const Vector5d& x0) {
    Eigen::MatrixX2d init_u = Eigen::MatrixX2d::Zero(N, 2);
    init_u.block(0, 0, N - 1, 2) = last_solve_u.block(1, 0, N - 1, 2);
    init_u.block(N - 1, 0, 1, 2) = last_solve_u.block(N - 1, 0, 1, 2);

    MatrixX5d init_x = MatrixX5d::Zero(N + 1, 5);
    init_x.row(0) = x0;
    Vector5d cur_x = x0;
    for (size_t i = 0; i < N; ++i) {
        Vector5d next_x = path_model::propagate(cur_x, init_u.row(i).transpose(), dt);
        cur_x = next_x;
        init_x.row(i + 1) = next_x;
    }

    return std::make_tuple(init_u, init_x);
}

MatrixX5d CILQRSolver::const_velo_prediction(const Vector5d& x0, size_t steps,
                                                         double dt) {
    Eigen::Vector2d cur_u = Eigen::Vector2d::Zero();
    MatrixX5d predicted_states = MatrixX5d::Zero(steps + 1, 5);
    predicted_states.row(0) = x0;
    Vector5d cur_x = x0;
    for (size_t i = 0; i < steps; ++i) {
        Vector5d next_x = path_model::propagate(cur_x, cur_u, dt);
        cur_x = next_x;
        predicted_states.row(i + 1) = next_x;
    }
    return predicted_states;
}

// 约束计算函数（6个约束：加速度上下界、曲率上下界、曲率变化率上下界）
Eigen::Matrix<double, 6, 1> CILQRSolver::get_constraints(
    const Vector5d& state, const Eigen::Vector2d& control, size_t index,
    const ReferenceLine& ref_waypoints, const std::vector<RoutingLine>& obs_preds,
    const Eigen::Vector2d& road_boaders) {
    Eigen::Matrix<double, 6, 1> ck;
    ck.setZero();
    
    double a = control[0];
    double dkappa = control[1];
    double kappa = state[4];
    
    // 约束定义：ck > 0 表示违反约束
    double amax = a - acc_max;              // 加速度上限：a <= acc_max
    double amin = acc_min - a;               // 加速度下限：a >= acc_min
    double kappamax = kappa - kappa_max;    // 曲率上限：kappa <= kappa_max
    double kappamin = kappa_min - kappa;     // 曲率下限：kappa >= kappa_min
    double dkappamax = dkappa - dkappa_max;  // 曲率变化率上限：dkappa <= dkappa_max
    double dkappamin = dkappa_min - dkappa;   // 曲率变化率下限：dkappa >= dkappa_min
    
    // 末步不约束加速度
    if (index != N) {
        ck << amax, amin, kappamax, kappamin, dkappamax, dkappamin;
    } else {
        ck << -1, -1, kappamax, kappamin, dkappamax, dkappamin;
    }
    
    return ck;
}

// 约束梯度计算
std::pair<Eigen::Matrix<double, 6, 5>, Eigen::Matrix<double, 6, 2>>
CILQRSolver::get_constraint_derivatives(const Vector5d& state,
                                             const Eigen::Vector2d& control, size_t index,
                                             const ReferenceLine& ref_waypoints,
                                             const std::vector<RoutingLine>& obs_preds,
                                             const Eigen::Vector2d& road_boaders) {
    Eigen::Matrix<double, 6, 5> ckx;
    Eigen::Matrix<double, 6, 2> cku;
    ckx.setZero();
    cku.setZero();
    
    // 约束对状态的梯度
    // ∂(kappa - kappa_max)/∂kappa = 1
    ckx(2, 4) = 1.0;   // kappamax对kappa的梯度
    // ∂(kappa_min - kappa)/∂kappa = -1
    ckx(3, 4) = -1.0;  // kappamin对kappa的梯度
    
    // 约束对控制的梯度
    if (index != N) {
        // ∂(a - acc_max)/∂a = 1
        cku(0, 0) = 1.0;   // amax对a的梯度
        // ∂(acc_min - a)/∂a = -1
        cku(1, 0) = -1.0;  // amin对a的梯度
    }
    // ∂(dkappa - dkappa_max)/∂dkappa = 1
    cku(4, 1) = 1.0;   // dkappamax对dkappa的梯度
    // ∂(dkappa_min - dkappa)/∂dkappa = -1
    cku(5, 1) = -1.0;  // dkappamin对dkappa的梯度
    
    return std::make_pair(ckx, cku);
}

// ALM参数更新函数已移除，参数更新在 get_total_cost_derivatives_and_Hessians 中进行

// lagrangian_derivative_and_Hessian 函数实现（类似原生项目）
std::tuple<Eigen::VectorXd, Eigen::MatrixXd> CILQRSolver::lagrangian_derivative_and_Hessian(
    double c, Eigen::MatrixXd c_dot, double rho, double mu) {
    size_t dims = c_dot.rows();
    Eigen::VectorXd b_dot = Eigen::VectorXd::Zero(dims, 1);
    Eigen::MatrixXd b_ddot = Eigen::MatrixXd::Zero(dims, dims);

    if ((c + mu / rho) > 0) {
        b_dot = rho * (c + mu / rho) * c_dot;
        b_ddot = b_dot * c_dot.transpose();
    }

    return std::make_tuple(b_dot, b_ddot);
}

// 总代价函数计算
double CILQRSolver::get_total_cost(const Eigen::MatrixX2d& u, const MatrixX5d& x,
                                        const ReferenceLine& ref_waypoints, double ref_velo,
                                        const std::vector<RoutingLine>& obs_preds,
                                        const Eigen::Vector2d& road_boaders) {
    static int call_count = 0;
    call_count++;
    double total_cost = 0.0;
    
    // 获取参考状态
    Eigen::MatrixX3d ref_exact_points = get_ref_exact_points(x, ref_waypoints);
    Eigen::VectorXd ref_velocitys = Eigen::VectorXd::Constant(N + 1, ref_velo);
    MatrixX5d ref_states(N + 1, 5);
    ref_states << ref_exact_points.block(0, 0, ref_exact_points.rows(), 2), ref_velocitys,
                 ref_exact_points.col(2), Eigen::VectorXd::Zero(N + 1);
    
    // 计算参考曲率
    for (size_t i = 0; i <= N; ++i) {
        double s = 0.0;
        // 简化：从参考线计算s（实际应该从x,y投影）
        if (i < ref_waypoints.size()) {
            s = ref_waypoints.longitude[i];
        } else {
            s = ref_waypoints.length();
        }
        ref_states(i, 4) = path_model::calc_kappa_from_reference(ref_waypoints, s);
    }
    
    // 主循环：对每个时间步计算代价
    size_t num_obstacles = obs_preds.size();
    for (size_t i = 0; i < N; ++i) {
        // 状态误差
        Vector5d state_diff = x.row(i).transpose() - ref_states.row(i).transpose();
        
        // 状态代价
        double c_state = ref_x_weight * state_diff.transpose() * state_weight * state_diff;
        
        // 控制代价
        double c_ctrl = u(i, 0) * u(i, 0) * control_a_cost +
                       u(i, 1) * u(i, 1) * control_dkappa_cost;
        
        // 约束项（6个：加速度、曲率、曲率变化率）
        Eigen::Matrix<double, 6, 1> cki = get_constraints(x.row(i).transpose(), 
                                                          u.row(i).transpose(), 
                                                          i, ref_waypoints, 
                                                          obs_preds, road_boaders);
        
        // ALM约束代价
        double constrain_cost = 0.0;
        if (solve_type == SolveType::ALM) {
            // 只对违反的约束施加惩罚
            // 0-1: 加速度上下界
            if (cki(0) >= 0 || alm_mu(i, 0) > 0) {
                constrain_cost += augmented_lagrangian_item(cki(0), uki, alm_mu(i, 0));
            }
            if (cki(1) >= 0 || alm_mu(i, 1) > 0) {
                constrain_cost += augmented_lagrangian_item(cki(1), uki, alm_mu(i, 1));
            }
            // 2-3: 曲率上下界
            if (cki(2) >= 0 || alm_mu(i, 2) > 0) {
                constrain_cost += augmented_lagrangian_item(cki(2), uki, alm_mu(i, 2));
            }
            if (cki(3) >= 0 || alm_mu(i, 3) > 0) {
                constrain_cost += augmented_lagrangian_item(cki(3), uki, alm_mu(i, 3));
            }
            // 4-5: 曲率变化率上下界
            if (cki(4) >= 0 || alm_mu(i, 4) > 0) {
                constrain_cost += augmented_lagrangian_item(cki(4), uki, alm_mu(i, 4));
            }
            if (cki(5) >= 0 || alm_mu(i, 5) > 0) {
                constrain_cost += augmented_lagrangian_item(cki(5), uki, alm_mu(i, 5));
            }
        } else {
            // Barrier方法（如果将来需要）
            SPDLOG_ERROR("Barrier method not implemented for path model");
        }
        
        // 车道边界约束（使用ALM方法，类似原生项目）
        Vector5d x_k = x.row(i).transpose();
        Eigen::Vector3d ref_x_k = ref_exact_points.row(i);
        double d_sign = (x_k[1] - ref_x_k[1]) * cos(ref_x_k[2]) - (x_k[0] - ref_x_k[0]) * sin(ref_x_k[2]);
        double cur_d = path_model::sign(d_sign) * hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]);
        double pos_up_constr = cur_d - (road_boaders[0] - width / 2);  // 上边界约束
        double pos_lo_constr = (road_boaders[1] + width / 2) - cur_d;  // 下边界约束
        
        double road_boundary_cost = 0.0;
        if (solve_type == SolveType::ALM) {
            // 6-7: 车道边界上下界
            if (pos_up_constr >= 0 || alm_mu(i, 6) > 0) {
                road_boundary_cost += augmented_lagrangian_item(pos_up_constr, uki, alm_mu(i, 6));
            }
            if (pos_lo_constr >= 0 || alm_mu(i, 7) > 0) {
                road_boundary_cost += augmented_lagrangian_item(pos_lo_constr, uki, alm_mu(i, 7));
            }
        }
        
        // 障碍物避让约束（使用ALM方法，类似原生项目）
        double obstacle_cost = 0.0;
        for (size_t j = 0; j < num_obstacles; ++j) {
            Eigen::Vector3d obs_j_pred_k = obs_preds[j][i];
            Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
            if (solve_type == SolveType::ALM) {
                // 8+2j, 9+2j: 第j个障碍物的前后约束
                if (obs_j_constr[0] >= 0 || alm_mu(i, 8 + 2 * j) > 0) {
                    obstacle_cost += augmented_lagrangian_item(obs_j_constr[0], uki, alm_mu(i, 8 + 2 * j));
                }
                if (obs_j_constr[1] >= 0 || alm_mu(i, 9 + 2 * j) > 0) {
                    obstacle_cost += augmented_lagrangian_item(obs_j_constr[1], uki, alm_mu(i, 9 + 2 * j));
                }
            }
        }
        
        // 调试：只在第一次调用时输出前几个时间步的详细信息
        if (call_count == 1 && i < 3) {
            SPDLOG_DEBUG("Cost breakdown at step {}: state={:.6f}, ctrl={:.6f}, constraint={:.6f}, road_boundary={:.6f}, obstacle={:.6f}, "
                        "state_diff=[{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}], ref_state=[{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}], "
                        "actual_state=[{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}], cur_d={:.3f}",
                        i, c_state, c_ctrl, constrain_cost, road_boundary_cost, obstacle_cost,
                        state_diff(0), state_diff(1), state_diff(2), state_diff(3), state_diff(4),
                        ref_states(i, 0), ref_states(i, 1), ref_states(i, 2), ref_states(i, 3), ref_states(i, 4),
                        x(i, 0), x(i, 1), x(i, 2), x(i, 3), x(i, 4), cur_d);
        }
        
        total_cost += c_state + c_ctrl + constrain_cost + road_boundary_cost + obstacle_cost;
    }
    
    // 末步（终点）状态代价
    size_t i = N;
    Vector5d state_diff = x.row(i).transpose() - ref_states.row(i).transpose();
    double c_state = final_x_weight * state_diff.transpose() * state_weight * state_diff;
    
    // 末步约束（控制为0）
    // 末步约束（只有曲率和曲率变化率约束，没有加速度约束）
    Eigen::Vector2d zero_control = Eigen::Vector2d::Zero();
    Eigen::Matrix<double, 6, 1> cki = get_constraints(x.row(i).transpose(), 
                                                      zero_control, 
                                                      i, ref_waypoints, 
                                                      obs_preds, road_boaders);
    
    double constrain_cost = 0.0;
    if (solve_type == SolveType::ALM) {
        // 末步只有曲率和曲率变化率约束（索引2-5）
        if (cki(2) >= 0 || alm_mu(i - 1, 2) > 0) {
            constrain_cost += augmented_lagrangian_item(cki(2), uki, alm_mu(i - 1, 2));
        }
        if (cki(3) >= 0 || alm_mu(i - 1, 3) > 0) {
            constrain_cost += augmented_lagrangian_item(cki(3), uki, alm_mu(i - 1, 3));
        }
        if (cki(4) >= 0 || alm_mu(i - 1, 4) > 0) {
            constrain_cost += augmented_lagrangian_item(cki(4), uki, alm_mu(i - 1, 4));
        }
        if (cki(5) >= 0 || alm_mu(i - 1, 5) > 0) {
            constrain_cost += augmented_lagrangian_item(cki(5), uki, alm_mu(i - 1, 5));
        }
    }
    
    // 末步车道边界约束
    Vector5d x_k = x.row(i).transpose();
    Eigen::Vector3d ref_x_k = ref_exact_points.row(i);
    double d_sign = (x_k[1] - ref_x_k[1]) * cos(ref_x_k[2]) - (x_k[0] - ref_x_k[0]) * sin(ref_x_k[2]);
        double cur_d = path_model::sign(d_sign) * hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]);
    double pos_up_constr = cur_d - (road_boaders[0] - width / 2);
    double pos_lo_constr = (road_boaders[1] + width / 2) - cur_d;
    
    double road_boundary_cost = 0.0;
    if (solve_type == SolveType::ALM) {
        if (pos_up_constr >= 0 || alm_mu(i - 1, 6) > 0) {
            road_boundary_cost += augmented_lagrangian_item(pos_up_constr, uki, alm_mu(i - 1, 6));
        }
        if (pos_lo_constr >= 0 || alm_mu(i - 1, 7) > 0) {
            road_boundary_cost += augmented_lagrangian_item(pos_lo_constr, uki, alm_mu(i - 1, 7));
        }
    }
    
    // 末步障碍物约束
    double obstacle_cost = 0.0;
    for (size_t j = 0; j < num_obstacles; ++j) {
        Eigen::Vector3d obs_j_pred_k = obs_preds[j][i];
        Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
        if (solve_type == SolveType::ALM) {
            if (obs_j_constr[0] >= 0 || alm_mu(i - 1, 8 + 2 * j) > 0) {
                obstacle_cost += augmented_lagrangian_item(obs_j_constr[0], uki, alm_mu(i - 1, 8 + 2 * j));
            }
            if (obs_j_constr[1] >= 0 || alm_mu(i - 1, 9 + 2 * j) > 0) {
                obstacle_cost += augmented_lagrangian_item(obs_j_constr[1], uki, alm_mu(i - 1, 9 + 2 * j));
            }
        }
    }
    
    total_cost += c_state + constrain_cost + road_boundary_cost + obstacle_cost;
    
    return total_cost;
}

// 获取参考点（从5维状态）
Eigen::MatrixX3d CILQRSolver::get_ref_exact_points(const MatrixX5d& x,
                                                        const ReferenceLine& ref_waypoints) {
    uint16_t x_shape = x.rows();
    uint16_t start_index = 0;
    Eigen::MatrixX3d ref_exact_points = Eigen::MatrixX3d::Zero(x_shape, 3);
    
    for (uint16_t i = 0; i < x_shape; ++i) {
        int32_t min_idx = -1;
        double min_distance = std::numeric_limits<double>::max();
        for (size_t j = start_index; j < ref_waypoints.size(); ++j) {
            double distance = hypot(x(i, 0) - ref_waypoints.x[j], x(i, 1) - ref_waypoints.y[j]);
            if (distance < min_distance) {
                min_distance = distance;
                min_idx = j;
            }
        }
        if (min_idx >= 0) {
            ref_exact_points(i, 0) = ref_waypoints.x[min_idx];
            ref_exact_points(i, 1) = ref_waypoints.y[min_idx];
            ref_exact_points(i, 2) = ref_waypoints.yaw[min_idx];
            start_index = min_idx;
        }
    }
    
    return ref_exact_points;
}

// 障碍物避障约束（适配5维状态，但使用车辆前后中心点）
Eigen::Vector2d CILQRSolver::get_obstacle_avoidance_constr(const Vector5d& ego_state,
                                                                const Eigen::Vector3d& obs_state) {
    // 对于路径模型，我们使用车辆中心点和heading来计算前后中心点
    // 简化处理：使用车辆长度的一半作为前后中心点的偏移
    double half_length = length / 2.0;
    double heading = ego_state[3];
    
    Eigen::Vector2d ego_center = ego_state.head(2);
    Eigen::Vector2d heading_vec = Eigen::Vector2d{cos(heading), sin(heading)};
    
    Eigen::Vector2d ego_front = ego_center + half_length * heading_vec;
    Eigen::Vector2d ego_rear = ego_center - half_length * heading_vec;
    
    Eigen::Vector2d ellipse_ab = path_model::get_ellipsoid_obstacle_scales(obs_attr, 0.5 * width);
    double front_safety_margin = path_model::ellipsoid_safety_margin(ego_front, obs_state, ellipse_ab);
    double rear_safety_margin = path_model::ellipsoid_safety_margin(ego_rear, obs_state, ellipse_ab);
    
    return Eigen::Vector2d{front_safety_margin, rear_safety_margin};
}

std::tuple<Vector5d, Vector5d>
CILQRSolver::get_obstacle_avoidance_constr_derivatives(const Vector5d& ego_state,
                                                           const Eigen::Vector3d& obs_state) {
    double half_length = length / 2.0;
    double heading = ego_state[3];
    
    Eigen::Vector2d heading_vec = Eigen::Vector2d{cos(heading), sin(heading)};
    Eigen::Vector2d heading_perp = Eigen::Vector2d{-sin(heading), cos(heading)};
    
    Eigen::Vector2d ego_front = ego_state.head(2) + half_length * heading_vec;
    Eigen::Vector2d ego_rear = ego_state.head(2) - half_length * heading_vec;
    
    Eigen::Vector2d ellipse_ab = path_model::get_ellipsoid_obstacle_scales(obs_attr, 0.5 * width);
    
    // 计算前后中心点的梯度
    Eigen::Vector2d front_grad_xy = path_model::ellipsoid_safety_margin_derivatives(ego_front, obs_state, ellipse_ab);
    Eigen::Vector2d rear_grad_xy = path_model::ellipsoid_safety_margin_derivatives(ego_rear, obs_state, ellipse_ab);
    
    // 对heading的梯度（通过链式法则）
    double front_grad_heading = half_length * (front_grad_xy.transpose() * heading_perp)(0, 0);
    double rear_grad_heading = -half_length * (rear_grad_xy.transpose() * heading_perp)(0, 0);
    
    Vector5d front_grad, rear_grad;
    front_grad << front_grad_xy[0], front_grad_xy[1], 0.0, front_grad_heading, 0.0;
    rear_grad << rear_grad_xy[0], rear_grad_xy[1], 0.0, rear_grad_heading, 0.0;
    
    return std::make_tuple(front_grad, rear_grad);
}

// Backward Pass
std::tuple<Eigen::MatrixX2d, MatrixX5d, Eigen::Vector2d>
CILQRSolver::backward_pass(const Eigen::MatrixX2d& u, const MatrixX5d& x, double lamb,
                               const ReferenceLine& ref_waypoints, double ref_velo,
                               const std::vector<RoutingLine>& obs_preds,
                               const Eigen::Vector2d& road_boaders) {
    get_total_cost_derivatives_and_Hessians(u, x, ref_waypoints, ref_velo, obs_preds, road_boaders);
    auto [df_dx, df_du] = path_model::get_derivatives(x, u, dt, N);

    Eigen::Vector2d delta_V = {0.0, 0.0};
    Eigen::MatrixX2d d = Eigen::MatrixX2d::Zero(N, nu);
    MatrixX5d K = MatrixX5d::Zero(N * nu, nx);

    Vector5d V_x = l_x.bottomRows(1).transpose();
    Matrix5d V_xx = l_xx.bottomRows(5);

    // 末步约束已经在get_total_cost_derivatives_and_Hessians中处理
    // 这里不需要额外处理，因为约束的梯度已经添加到l_x和l_xx中

    for (int i = N - 1; i >= 0; --i) {
        // 提取A和B矩阵
        Matrix5d A = df_dx.block(nx * i, 0, nx, nx);
        Eigen::Matrix<double, 5, 2> B = df_du.block(nx * i, 0, nx, nu);
        
        // Q函数计算（约束的梯度已经在l_x和l_u中，这里直接使用）
        Vector5d Q_x = l_x.row(i).transpose() + A.transpose() * V_x;
        Eigen::Vector2d Q_u = l_u.row(i).transpose() + B.transpose() * V_x;
        Matrix5d Q_xx = l_xx.block(nx * i, 0, nx, nx) + A.transpose() * V_xx * A;
        Eigen::Matrix<double, 2, 5> Q_ux = l_ux.block(nu * i, 0, nu, nx) + B.transpose() * V_xx * A;
        Eigen::Matrix2d Q_uu = l_uu.block(nu * i, 0, nu, nu) + B.transpose() * V_xx * B;
        
        // 添加正则化项
        Eigen::Matrix2d Q_uu_reg = Q_uu + lamb * Eigen::Matrix2d::Identity();
        
        // 求解
        Eigen::LLT<Eigen::MatrixXd> llt_quu(Q_uu_reg);
        if (llt_quu.info() == Eigen::NumericalIssue) {
            SPDLOG_ERROR("[Backward pass] Non-PD Quu cur lamb: {}", lamb);
            current_solve_status = LQRSolveStatus::BACKWARD_PASS_FAIL;
            return std::make_tuple(d, K, delta_V);
        }
        Eigen::Matrix2d Q_uu_inv = Q_uu_reg.inverse();
        
        d.row(i) = -Q_uu_inv * Q_u;
        K.block(nu * i, 0, nu, nx) = -Q_uu_inv * Q_ux;
        
        // 更新值函数
        V_x = Q_x + K.block(nu * i, 0, nu, nx).transpose() * Q_uu * d.row(i).transpose() +
              K.block(nu * i, 0, nu, nx).transpose() * Q_u +
              Q_ux.transpose() * d.row(i).transpose();
        V_xx = Q_xx + K.block(nu * i, 0, nu, nx).transpose() * Q_uu * K.block(nu * i, 0, nu, nx) +
               K.block(nu * i, 0, nu, nx).transpose() * Q_ux +
               Q_ux.transpose() * K.block(nu * i, 0, nu, nx);
        
        // 预期代价降低
        delta_V[0] += (0.5 * d.row(i) * Q_uu_reg * d.row(i).transpose())(0, 0);
        delta_V[1] += (d.row(i) * Q_u)(0, 0);
    }

    return std::make_tuple(d, K, delta_V);
}

// Forward Pass
std::tuple<Eigen::MatrixX2d, MatrixX5d> CILQRSolver::forward_pass(
    const Eigen::MatrixX2d& u, const MatrixX5d& x, const Eigen::MatrixX2d& d,
    const MatrixX5d& K, double alpha) {
    Eigen::MatrixX2d new_u = Eigen::MatrixX2d::Zero(N, nu);
    MatrixX5d new_x = MatrixX5d::Zero(N + 1, nx);
    new_x.row(0) = x.row(0);

    for (uint32_t i = 0; i < N; ++i) {
        Vector5d delta_x = (new_x.row(i) - x.row(i)).transpose();
        Eigen::Vector2d new_u_i = u.row(i).transpose() + alpha * d.row(i).transpose() +
                                  K.block(nu * i, 0, nu, nx) * delta_x;
        new_u.row(i) = new_u_i;
        
        // 使用路径模型传播
        new_x.row(i + 1) = path_model::propagate(new_x.row(i).transpose(), new_u_i, dt);
    }

    return std::make_tuple(new_u, new_x);
}

// iter_step函数
std::tuple<Eigen::MatrixX2d, MatrixX5d, double> CILQRSolver::iter_step(
    const Eigen::MatrixX2d& u, const MatrixX5d& x, double lamb,
    const ReferenceLine& ref_waypoints, double ref_velo,
    const std::vector<RoutingLine>& obs_preds, const Eigen::Vector2d& road_boaders,
    bool& effective_flag) {
    effective_flag = false;
    
    auto [d, K, delta_V] = backward_pass(u, x, lamb, ref_waypoints, ref_velo, obs_preds, road_boaders);
    
    if (current_solve_status == LQRSolveStatus::BACKWARD_PASS_FAIL) {
        return std::make_tuple(u, x, get_total_cost(u, x, ref_waypoints, ref_velo, obs_preds, road_boaders));
    }
    
    double cost_old = get_total_cost(u, x, ref_waypoints, ref_velo, obs_preds, road_boaders);
    double cost_new = cost_old;
    
    // 线性搜索（减少选项以加快搜索）
    std::vector<double> alpha_options = {1.0, 0.5, 0.25};  // 从5个减少到3个
    double best_alpha = 0.0;
    Eigen::MatrixX2d best_u;
    MatrixX5d best_x;
    
    for (auto& alpha : alpha_options) {
        auto [new_u, new_x] = forward_pass(u, x, d, K, alpha);
        cost_new = get_total_cost(new_u, new_x, ref_waypoints, ref_velo, obs_preds, road_boaders);
        
        if (cost_new < cost_old) {
            best_alpha = alpha;
            best_u = new_u;
            best_x = new_x;
            effective_flag = true;
            break;
        }
    }
    
    if (effective_flag) {
        // 避免重复计算：直接使用线性搜索中找到的结果
        double cost_diff = std::abs(cost_old - cost_new);
        SPDLOG_DEBUG("Iter step: cost_old={:.6f}, cost_new={:.6f}, diff={:.6f}, threshold={:.6f}, alpha={:.3f}", 
                     cost_old, cost_new, cost_diff, convergence_threshold, best_alpha);
        
        if (cost_diff < convergence_threshold) {
            current_solve_status = LQRSolveStatus::CONVERGED;
            SPDLOG_DEBUG("Converged: cost difference {:.6f} < threshold {:.6f}", 
                         cost_diff, convergence_threshold);
        }
        
        return std::make_tuple(best_u, best_x, cost_new);
    } else {
        current_solve_status = LQRSolveStatus::FORWARD_PASS_FAIL;
        return std::make_tuple(u, x, cost_old);
    }
}

// 计算代价函数的梯度和Hessian
void CILQRSolver::get_total_cost_derivatives_and_Hessians(
    const Eigen::MatrixX2d& u, const MatrixX5d& x, const ReferenceLine& ref_waypoints,
    double ref_velo, const std::vector<RoutingLine>& obs_preds,
    const Eigen::Vector2d& road_boaders) {
    current_solve_status = LQRSolveStatus::RUNNING;
    l_x.setZero(N + 1, nx);
    l_u.setZero(N, nu);
    l_xx.setZero((N + 1) * nx, nx);
    l_uu.setZero(N * nu, nu);
    l_ux.setZero(N * nu, nx);
    
    // 获取参考状态
    Eigen::MatrixX3d ref_exact_points = get_ref_exact_points(x, ref_waypoints);
    Eigen::VectorXd ref_velocitys = Eigen::VectorXd::Constant(N + 1, ref_velo);
    MatrixX5d ref_states(N + 1, 5);
    ref_states << ref_exact_points.block(0, 0, ref_exact_points.rows(), 2), ref_velocitys,
                 ref_exact_points.col(2), Eigen::VectorXd::Zero(N + 1);
    
    // 计算参考曲率
    for (size_t i = 0; i <= N; ++i) {
        double s = 0.0;
        if (i < ref_waypoints.size()) {
            s = ref_waypoints.longitude[i];
        } else {
            s = ref_waypoints.length();
        }
        ref_states(i, 4) = path_model::calc_kappa_from_reference(ref_waypoints, s);
    }
    
    // 状态代价的梯度和Hessian
    for (size_t i = 0; i < N; ++i) {
        Vector5d state_diff = x.row(i).transpose() - ref_states.row(i).transpose();
        l_x.row(i) = (ref_x_weight * 2 * state_weight * state_diff).transpose();
        l_xx.block(nx * i, 0, nx, nx) = ref_x_weight * 2 * state_weight;
    }
    
    // 末步状态代价
    Vector5d state_diff = x.row(N).transpose() - ref_states.row(N).transpose();
    l_x.row(N) = (final_x_weight * 2 * state_weight * state_diff).transpose();
    l_xx.block(nx * N, 0, nx, nx) = final_x_weight * 2 * state_weight;
    
    // 控制代价的梯度和Hessian
    for (size_t i = 0; i < N; ++i) {
        l_u.row(i) = (2 * ctrl_weight * u.row(i).transpose()).transpose();
        l_uu.block(nu * i, 0, nu, nu) = 2 * ctrl_weight;
        
        // 控制约束的梯度和Hessian（使用ALM方法）
        if (solve_type == SolveType::ALM) {
            Eigen::Matrix<double, 6, 1> cki = get_constraints(x.row(i).transpose(), 
                                                              u.row(i).transpose(), 
                                                              i, ref_waypoints, 
                                                              obs_preds, road_boaders);
            auto [ckx, cku] = get_constraint_derivatives(x.row(i).transpose(), 
                                                         u.row(i).transpose(), 
                                                         i, ref_waypoints, 
                                                         obs_preds, road_boaders);
            
            // 加速度约束（索引0-1）
            Eigen::Vector2d acc_up_constr_over_u = {1.0, 0.0};
            Eigen::Vector2d acc_lo_constr_over_u = {-1.0, 0.0};
            auto [acc_up_barrier_over_u, acc_up_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(cki(0), acc_up_constr_over_u, uki, alm_mu(i, 0));
            auto [acc_lo_barrier_over_u, acc_lo_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(cki(1), acc_lo_constr_over_u, uki, alm_mu(i, 1));
            
            // 曲率约束（索引2-3，对状态kappa的约束）
            // 曲率变化率约束（索引4-5）
            Eigen::Vector2d dkappamax_constr_over_u = {0.0, 1.0};
            Eigen::Vector2d dkappamin_constr_over_u = {0.0, -1.0};
            auto [dkappamax_barrier_over_u, dkappamax_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(cki(4), dkappamax_constr_over_u, uki, alm_mu(i, 4));
            auto [dkappamin_barrier_over_u, dkappamin_barrier_over_uu] =
                lagrangian_derivative_and_Hessian(cki(5), dkappamin_constr_over_u, uki, alm_mu(i, 5));
            
            // 添加到控制梯度
            l_u.row(i) += acc_up_barrier_over_u.transpose() + acc_lo_barrier_over_u.transpose() +
                          dkappamax_barrier_over_u.transpose() + dkappamin_barrier_over_u.transpose();
            l_uu.block(nu * i, 0, nu, nu) += acc_up_barrier_over_uu + acc_lo_barrier_over_uu +
                                             dkappamax_barrier_over_uu + dkappamin_barrier_over_uu;
            
            // 曲率约束对状态的梯度（添加到l_x）
            Vector5d kappamax_constr_over_x = Vector5d::Zero();
            Vector5d kappamin_constr_over_x = Vector5d::Zero();
            kappamax_constr_over_x(4) = 1.0;   // ∂(kappa - kappa_max)/∂kappa = 1
            kappamin_constr_over_x(4) = -1.0;  // ∂(kappa_min - kappa)/∂kappa = -1
            auto [kappamax_barrier_over_x, kappamax_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(cki(2), kappamax_constr_over_x, uki, alm_mu(i, 2));
            auto [kappamin_barrier_over_x, kappamin_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(cki(3), kappamin_constr_over_x, uki, alm_mu(i, 3));
            
            l_x.row(i) += kappamax_barrier_over_x.transpose() + kappamin_barrier_over_x.transpose();
            l_xx.block(nx * i, 0, nx, nx) += kappamax_barrier_over_xx + kappamin_barrier_over_xx;
            
            // 更新alm_mu_next
            alm_mu_next(i, 0) = std::min(std::max(alm_mu(i, 0) + uki * cki(0), 0.0), max_mu);
            alm_mu_next(i, 1) = std::min(std::max(alm_mu(i, 1) + uki * cki(1), 0.0), max_mu);
            alm_mu_next(i, 2) = std::min(std::max(alm_mu(i, 2) + uki * cki(2), 0.0), max_mu);
            alm_mu_next(i, 3) = std::min(std::max(alm_mu(i, 3) + uki * cki(3), 0.0), max_mu);
            alm_mu_next(i, 4) = std::min(std::max(alm_mu(i, 4) + uki * cki(4), 0.0), max_mu);
            alm_mu_next(i, 5) = std::min(std::max(alm_mu(i, 5) + uki * cki(5), 0.0), max_mu);
        }
    }
    
    // 车道边界约束的梯度和Hessian（使用ALM方法）
    for (size_t i = 0; i < N; ++i) {
        Vector5d x_k = x.row(i).transpose();
        Eigen::Vector3d ref_x_k = ref_exact_points.row(i);
        
        // 计算横向距离
        double d_sign = (x_k[1] - ref_x_k[1]) * cos(ref_x_k[2]) - (x_k[0] - ref_x_k[0]) * sin(ref_x_k[2]);
        double dist_to_ref = hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]);
        double cur_d = path_model::sign(d_sign) * dist_to_ref;
        
        // 车道边界约束
        double pos_up_constr = cur_d - (road_boaders[0] - width / 2);
        double pos_lo_constr = (road_boaders[1] + width / 2) - cur_d;
        
        // 计算约束梯度（相对于状态x, y）
        Vector5d pos_up_grad = Vector5d::Zero();
        Vector5d pos_lo_grad = Vector5d::Zero();
        
        if (dist_to_ref > 1e-6) {
            double dx_norm = (x_k[0] - ref_x_k[0]) / dist_to_ref;
            double dy_norm = (x_k[1] - ref_x_k[1]) / dist_to_ref;
            if (d_sign < 0) {
                dx_norm = -dx_norm;
                dy_norm = -dy_norm;
            }
            pos_up_grad << dx_norm, dy_norm, 0.0, 0.0, 0.0;
            pos_lo_grad << -dx_norm, -dy_norm, 0.0, 0.0, 0.0;
        }
        
        // 使用ALM方法计算梯度和Hessian
        if (solve_type == SolveType::ALM) {
            auto [pos_up_barrier_over_x, pos_up_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(pos_up_constr, pos_up_grad, uki, alm_mu(i, 6));
            auto [pos_lo_barrier_over_x, pos_lo_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(pos_lo_constr, pos_lo_grad, uki, alm_mu(i, 7));
            
            l_x.row(i) += pos_up_barrier_over_x.transpose() + pos_lo_barrier_over_x.transpose();
            l_xx.block(nx * i, 0, nx, nx) += pos_up_barrier_over_xx + pos_lo_barrier_over_xx;
            
            // 更新alm_mu_next（车道边界约束）
            alm_mu_next(i, 6) = std::min(std::max(alm_mu(i, 6) + uki * pos_up_constr, 0.0), max_mu);
            alm_mu_next(i, 7) = std::min(std::max(alm_mu(i, 7) + uki * pos_lo_constr, 0.0), max_mu);
        }
    }
    
    // 障碍物避让约束的梯度和Hessian（使用ALM方法）
    size_t num_obstacles = obs_preds.size();
    for (size_t i = 0; i < N; ++i) {
        Vector5d x_k = x.row(i).transpose();
        
        for (size_t j = 0; j < num_obstacles; ++j) {
            Eigen::Vector3d obs_j_pred_k = obs_preds[j][i];
            Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
            auto [obs_j_front_grad, obs_j_rear_grad] = 
                get_obstacle_avoidance_constr_derivatives(x_k, obs_j_pred_k);
            
            if (solve_type == SolveType::ALM) {
                auto [obs_j_front_barrier_over_x, obs_j_front_barrier_over_xx] =
                    lagrangian_derivative_and_Hessian(obs_j_constr[0], obs_j_front_grad, uki, alm_mu(i, 8 + 2 * j));
                auto [obs_j_rear_barrier_over_x, obs_j_rear_barrier_over_xx] =
                    lagrangian_derivative_and_Hessian(obs_j_constr[1], obs_j_rear_grad, uki, alm_mu(i, 9 + 2 * j));
                
                l_x.row(i) += obs_j_front_barrier_over_x.transpose() + obs_j_rear_barrier_over_x.transpose();
                l_xx.block(nx * i, 0, nx, nx) += obs_j_front_barrier_over_xx + obs_j_rear_barrier_over_xx;
                
                // 更新alm_mu_next（障碍物约束）
                alm_mu_next(i, 8 + 2 * j) = std::min(std::max(alm_mu(i, 8 + 2 * j) + uki * obs_j_constr[0], 0.0), max_mu);
                alm_mu_next(i, 9 + 2 * j) = std::min(std::max(alm_mu(i, 9 + 2 * j) + uki * obs_j_constr[1], 0.0), max_mu);
            }
        }
    }
    
    // 末步也需要处理车道边界约束
    size_t i = N;
    Vector5d x_k = x.row(i).transpose();
    Eigen::Vector3d ref_x_k = ref_exact_points.row(i);
    
    double d_sign = (x_k[1] - ref_x_k[1]) * cos(ref_x_k[2]) - (x_k[0] - ref_x_k[0]) * sin(ref_x_k[2]);
    double dist_to_ref = hypot(x_k[0] - ref_x_k[0], x_k[1] - ref_x_k[1]);
    double cur_d = path_model::sign(d_sign) * dist_to_ref;
    
    double pos_up_constr = cur_d - (road_boaders[0] - width / 2);
    double pos_lo_constr = (road_boaders[1] + width / 2) - cur_d;
    
    Vector5d pos_up_grad = Vector5d::Zero();
    Vector5d pos_lo_grad = Vector5d::Zero();
    
    if (dist_to_ref > 1e-6) {
        double dx_norm = (x_k[0] - ref_x_k[0]) / dist_to_ref;
        double dy_norm = (x_k[1] - ref_x_k[1]) / dist_to_ref;
        if (d_sign < 0) {
            dx_norm = -dx_norm;
            dy_norm = -dy_norm;
        }
        pos_up_grad << dx_norm, dy_norm, 0.0, 0.0, 0.0;
        pos_lo_grad << -dx_norm, -dy_norm, 0.0, 0.0, 0.0;
    }
    
    // 末步车道边界约束
    if (solve_type == SolveType::ALM) {
        auto [pos_up_barrier_over_x, pos_up_barrier_over_xx] =
            lagrangian_derivative_and_Hessian(pos_up_constr, pos_up_grad, uki, alm_mu(i - 1, 6));
        auto [pos_lo_barrier_over_x, pos_lo_barrier_over_xx] =
            lagrangian_derivative_and_Hessian(pos_lo_constr, pos_lo_grad, uki, alm_mu(i - 1, 7));
        
        l_x.row(i) += pos_up_barrier_over_x.transpose() + pos_lo_barrier_over_x.transpose();
        l_xx.block(nx * i, 0, nx, nx) += pos_up_barrier_over_xx + pos_lo_barrier_over_xx;
    }
    
    // 末步障碍物约束
    for (size_t j = 0; j < num_obstacles; ++j) {
        Eigen::Vector3d obs_j_pred_k = obs_preds[j][i];
        Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
        auto [obs_j_front_grad, obs_j_rear_grad] = 
            get_obstacle_avoidance_constr_derivatives(x_k, obs_j_pred_k);
        
        if (solve_type == SolveType::ALM) {
            // 参考算法方式：使用uki
            auto [obs_j_front_barrier_over_x, obs_j_front_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(obs_j_constr[0], obs_j_front_grad, uki, alm_mu(i - 1, 8 + 2 * j));
            auto [obs_j_rear_barrier_over_x, obs_j_rear_barrier_over_xx] =
                lagrangian_derivative_and_Hessian(obs_j_constr[1], obs_j_rear_grad, uki, alm_mu(i - 1, 9 + 2 * j));
            
            l_x.row(i) += obs_j_front_barrier_over_x.transpose() + obs_j_rear_barrier_over_x.transpose();
            l_xx.block(nx * i, 0, nx, nx) += obs_j_front_barrier_over_xx + obs_j_rear_barrier_over_xx;
        }
    }
    
    // 末步也需要处理障碍物约束
    for (size_t j = 0; j < num_obstacles; ++j) {
        Eigen::Vector3d obs_j_pred_k = obs_preds[j][i];
        Eigen::Vector2d obs_j_constr = get_obstacle_avoidance_constr(x_k, obs_j_pred_k);
        // 末步障碍物约束已经在get_total_cost_derivatives_and_Hessians中处理
    }
}

}  // namespace cilqr
