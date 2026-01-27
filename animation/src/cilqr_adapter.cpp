/*
 * @Author: puyu <yuu.pu@foxmail.com>
 * @Date: 2025-01-XX XX:XX:XX
 * @LastEditTime: 2025-01-XX XX:XX:XX
 * @FilePath: /toy-example-of-iLQR/src/cilqr_adapter.cpp
 * Copyright 2025 puyu, All Rights Reserved.
 * 
 * CILQR算法适配器层实现
 */

#include "cilqr_adapter.hpp"
#include <spdlog/spdlog.h>

CILQRAdapter::CILQRAdapter(const GlobalConfig* const config) {
    algo_config_ = create_algorithm_config(config);
    // 使用工厂创建优化器（参考 trajectory_smoother.cpp 的设计风格）
    optimizer_ = trajectory_optimizer::OptimizerFactory::Create(
        "cilqr", "trajectory smoother cilqr", algo_config_);
    if (!optimizer_) {
        SPDLOG_ERROR("Failed to create CILQR optimizer");
        throw std::runtime_error("Failed to create CILQR optimizer");
    }
}

cilqr::AlgorithmConfig CILQRAdapter::create_algorithm_config(const GlobalConfig* const config) {
    cilqr::AlgorithmConfig algo_config;
    
    algo_config.delta_t = config->get_config<double>("delta_t");
    algo_config.N = config->get_config<int>("lqr/N");
    algo_config.w_pos = config->get_config<double>("lqr/w_pos");
    algo_config.w_vel = config->get_config<double>("lqr/w_vel");
    algo_config.w_heading = config->has_key("lqr/w_heading")
                           ? config->get_config<double>("lqr/w_heading")
                           : config->get_config<double>("lqr/w_yaw");
    algo_config.w_kappa = config->has_key("lqr/w_kappa")
                         ? config->get_config<double>("lqr/w_kappa")
                         : algo_config.w_heading;
    algo_config.w_acc = config->get_config<double>("lqr/w_acc");
    algo_config.w_dkappa = config->has_key("lqr/w_dkappa")
                          ? config->get_config<double>("lqr/w_dkappa")
                          : algo_config.w_acc;
    algo_config.solve_type = config->get_config<std::string>("lqr/slove_type");
    algo_config.ref_x_weight = config->has_key("lqr/ref_x_weight")
                              ? config->get_config<double>("lqr/ref_x_weight")
                              : 1.0;
    algo_config.final_x_weight = config->has_key("lqr/final_x_weight")
                                ? config->get_config<double>("lqr/final_x_weight")
                                : 10.0;
    algo_config.uki_init = config->has_key("lqr/uki_init")
                          ? config->get_config<double>("lqr/uki_init")
                          : 20.0;
    algo_config.alm_rho_init = config->get_config<double>("lqr/alm_rho_init");
    algo_config.alm_gamma = config->get_config<double>("lqr/alm_gamma");
    algo_config.max_rho = config->get_config<double>("lqr/max_rho");
    algo_config.max_mu = config->get_config<double>("lqr/max_mu");
    algo_config.obstacle_exp_q1 = config->get_config<double>("lqr/obstacle_exp_q1");
    algo_config.obstacle_exp_q2 = config->get_config<double>("lqr/obstacle_exp_q2");
    algo_config.state_exp_q1 = config->get_config<double>("lqr/state_exp_q1");
    algo_config.state_exp_q2 = config->get_config<double>("lqr/state_exp_q2");
    algo_config.use_last_solution = config->get_config<bool>("lqr/use_last_solution");
    
    algo_config.max_iter = config->get_config<int>("iteration/max_iter");
    algo_config.init_lamb = config->get_config<double>("iteration/init_lamb");
    algo_config.lamb_decay = config->get_config<double>("iteration/lamb_decay");
    algo_config.lamb_amplify = config->get_config<double>("iteration/lamb_amplify");
    algo_config.max_lamb = config->get_config<double>("iteration/max_lamb");
    algo_config.convergence_threshold = config->get_config<double>("iteration/convergence_threshold");
    algo_config.accept_step_threshold = config->get_config<double>("iteration/accept_step_threshold");
    
    algo_config.width = config->get_config<double>("vehicle/width");
    algo_config.length = config->get_config<double>("vehicle/length");
    algo_config.velo_max = config->get_config<double>("vehicle/velo_max");
    algo_config.velo_min = config->get_config<double>("vehicle/velo_min");
    algo_config.acc_max = config->get_config<double>("vehicle/acc_max");
    algo_config.acc_min = config->get_config<double>("vehicle/acc_min");
    algo_config.kappa_max = config->has_key("vehicle/kappa_max")
                           ? config->get_config<double>("vehicle/kappa_max")
                           : 0.3;
    algo_config.kappa_min = config->has_key("vehicle/kappa_min")
                           ? config->get_config<double>("vehicle/kappa_min")
                           : -0.3;
    algo_config.dkappa_max = config->has_key("vehicle/dkappa_max")
                            ? config->get_config<double>("vehicle/dkappa_max")
                            : 0.1;
    algo_config.dkappa_min = config->has_key("vehicle/dkappa_min")
                            ? config->get_config<double>("vehicle/dkappa_min")
                            : -0.1;
    algo_config.d_safe = config->get_config<double>("vehicle/d_safe");
    
    return algo_config;
}

cilqr::ReferenceLine CILQRAdapter::convert_reference_line(const ReferenceLine& vis_ref) {
    cilqr::ReferenceLine algo_ref;
    algo_ref.x = vis_ref.x;
    algo_ref.y = vis_ref.y;
    algo_ref.yaw = vis_ref.yaw;
    algo_ref.longitude = vis_ref.longitude;
    // 注意：算法层可能需要简化版的spline数据，这里先保留基本接口
    return algo_ref;
}

std::vector<cilqr::RoutingLine> CILQRAdapter::convert_routing_lines(
    const std::vector<RoutingLine>& vis_routing) {
    std::vector<cilqr::RoutingLine> algo_routing;
    for (const auto& vis_line : vis_routing) {
        cilqr::RoutingLine algo_line;
        algo_line.size = vis_line.size;
        algo_line.x = vis_line.x;
        algo_line.y = vis_line.y;
        algo_line.yaw = vis_line.yaw;
        algo_routing.push_back(algo_line);
    }
    return algo_routing;
}

std::tuple<Eigen::MatrixX2d, MatrixX5d> CILQRAdapter::solve(
    const Vector5d& x0,
    const ReferenceLine& ref_waypoints,
    double ref_velo,
    const std::vector<RoutingLine>& obs_preds,
    const Eigen::Vector2d& road_boaders) {
    
    // 转换类型
    cilqr::ReferenceLine algo_ref = convert_reference_line(ref_waypoints);
    std::vector<cilqr::RoutingLine> algo_obs = convert_routing_lines(obs_preds);
    
    // 使用优化器基类接口执行优化（参考 trajectory_smoother.cpp 的设计风格）
    auto result = optimizer_->Execute(x0, algo_ref, ref_velo, algo_obs, road_boaders);
    
    if (!result.success) {
        SPDLOG_WARN("Optimization failed: {}", result.status_message);
        // 返回空结果
        return {Eigen::MatrixX2d(), MatrixX5d()};
    }
    
    return {result.control_sequence, result.state_sequence};
}
