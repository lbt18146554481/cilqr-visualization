#pragma once
#ifndef __CILQR_SOLVER_HPP
#define __CILQR_SOLVER_HPP

#include "types.hpp"
#include "algorithm_config.hpp"
#include "path_model_utils.hpp"

#include <Eigen/Core>
#include <tuple>
#include <vector>

namespace cilqr {

class CILQRSolver {
  public:
    CILQRSolver() = delete;
    explicit CILQRSolver(const AlgorithmConfig& config);
    ~CILQRSolver() {}

    // 求解接口
    std::tuple<Eigen::MatrixX2d, MatrixX5d> solve(
        const Vector5d& x0,  // [x, y, v, heading, kappa]
        const ReferenceLine& ref_waypoints,
        double ref_velo,
        const std::vector<RoutingLine>& obs_preds,
        const Eigen::Vector2d& road_boaders);

  private:
    std::tuple<Eigen::MatrixX2d, MatrixX5d> get_init_traj(const Vector5d& x0);
    std::tuple<Eigen::MatrixX2d, MatrixX5d> get_init_traj_increment(const Vector5d& x0);
    MatrixX5d const_velo_prediction(const Vector5d& x0, size_t steps, double dt);
    double get_total_cost(const Eigen::MatrixX2d& u, const MatrixX5d& x,
                          const ReferenceLine& ref_waypoints, double ref_velo,
                          const std::vector<RoutingLine>& obs_preds,
                          const Eigen::Vector2d& road_boaders);
    Eigen::MatrixX3d get_ref_exact_points(const MatrixX5d& x,
                                          const ReferenceLine& ref_waypoints);
    std::tuple<Eigen::MatrixX2d, MatrixX5d, double> iter_step(
        const Eigen::MatrixX2d& u, const MatrixX5d& x, double lamb,
        const ReferenceLine& ref_waypoints, double ref_velo,
        const std::vector<RoutingLine>& obs_preds, const Eigen::Vector2d& road_boaders,
        bool& effective_flag);
    std::tuple<Eigen::MatrixX2d, MatrixX5d, Eigen::Vector2d> backward_pass(
        const Eigen::MatrixX2d& u, const MatrixX5d& x, double lamb,
        const ReferenceLine& ref_waypoints, double ref_velo,
        const std::vector<RoutingLine>& obs_preds, const Eigen::Vector2d& road_boaders);
    std::tuple<Eigen::MatrixX2d, MatrixX5d> forward_pass(
        const Eigen::MatrixX2d& u,
        const MatrixX5d& x,
        const Eigen::MatrixX2d& d,
        const MatrixX5d& K,
        double alpha);
    void get_total_cost_derivatives_and_Hessians(
        const Eigen::MatrixX2d& u,
        const MatrixX5d& x,
        const ReferenceLine& ref_waypoints,
        double ref_velo,
        const std::vector<RoutingLine>& obs_preds,
        const Eigen::Vector2d& road_boaders);

    // 约束计算函数
    Eigen::Matrix<double, 6, 1> get_constraints(
        const Vector5d& state,
        const Eigen::Vector2d& control,
        size_t index,
        const ReferenceLine& ref_waypoints,
        const std::vector<RoutingLine>& obs_preds,
        const Eigen::Vector2d& road_boaders);
    
    std::pair<Eigen::Matrix<double, 6, 5>, Eigen::Matrix<double, 6, 2>> get_constraint_derivatives(
        const Vector5d& state,
        const Eigen::Vector2d& control,
        size_t index,
        const ReferenceLine& ref_waypoints,
        const std::vector<RoutingLine>& obs_preds,
        const Eigen::Vector2d& road_boaders);

    // 辅助函数
    Eigen::Vector2d get_obstacle_avoidance_constr(const Vector5d& ego_state,
                                                  const Eigen::Vector3d& obs_state);
    std::tuple<Vector5d, Vector5d> get_obstacle_avoidance_constr_derivatives(
        const Vector5d& ego_state, const Eigen::Vector3d& obs_state);

    double augmented_lagrangian_item(double c, double rho, double mu) {
        return rho * pow(std::max(c + mu / rho, 0.0), 2) / 2;
    }
    std::tuple<Eigen::VectorXd, Eigen::MatrixXd> lagrangian_derivative_and_Hessian(
        double c, Eigen::MatrixXd c_dot, double rho, double mu);

    // 配置参数
    AlgorithmConfig config_;
    
    // planning-related settings
    uint32_t N;
    double dt;
    uint32_t nx;  // 5
    uint32_t nu;  // 2
    Matrix5d state_weight;
    Eigen::Matrix2d ctrl_weight;
    bool use_last_solution;
    SolveType solve_type;

    // augmented lagrangian method settings
    double alm_rho;
    double alm_rho_init;
    double alm_gamma;
    double max_rho;
    double max_mu;
    double uki;
    double uki_init;
    Eigen::MatrixXd alm_mu;
    Eigen::MatrixXd alm_mu_next;
    
    // iteration-related settings
    uint32_t max_iter;
    double init_lamb;
    double lamb_decay;
    double lamb_amplify;
    double max_lamb;
    double convergence_threshold;
    double accept_step_threshold;

    // ego vehicle-related settings
    double width;
    double length;
    double velo_max;
    double velo_min;
    double acc_max;
    double acc_min;
    double kappa_max;
    double kappa_min;
    double dkappa_max;
    double dkappa_min;
    double ref_x_weight;
    double final_x_weight;
    double control_a_cost;
    double control_dkappa_cost;

    Eigen::Vector3d obs_attr;
    MatrixX5d l_x;
    Eigen::MatrixX2d l_u;
    MatrixX5d l_xx;
    Eigen::MatrixX2d l_xu;
    MatrixX5d l_ux;
    Eigen::MatrixX2d l_uu;

    bool is_first_solve;
    Eigen::MatrixX2d last_solve_u;
    MatrixX5d last_solve_x;
    double last_solve_J;
    LQRSolveStatus current_solve_status = LQRSolveStatus::RUNNING;
};

}  // namespace cilqr

#endif
