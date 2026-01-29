#pragma once
#ifndef __ALGORITHM_CONFIG_HPP
#define __ALGORITHM_CONFIG_HPP

#include <string>
#include <memory>

namespace cilqr {

// 算法配置结构体（纯数据，不依赖YAML）
struct AlgorithmConfig {
    // 时间步长
    double delta_t = 0.1;
    
    // LQR参数
    uint32_t N = 20;  // 预测时域长度
    double w_pos = 1.0;
    double w_vel = 1.0;
    double w_heading = 20.0;
    double w_kappa = 10.0;
    double w_acc = 0.5;
    double w_dkappa = 25.0;
    std::string solve_type = "alm";  // "alm" or "barrier"
    double ref_x_weight = 1.0;
    double final_x_weight = 10.0;
    double uki_init = 20.0;
    double alm_rho_init = 20.0;
    double alm_gamma = 0.1;
    double max_rho = 100.0;
    double max_mu = 200.0;
    double obstacle_exp_q1 = 8.0;
    double obstacle_exp_q2 = 6.0;
    double state_exp_q1 = 3.0;
    double state_exp_q2 = 3.5;
    bool use_last_solution = true;
    
    // 迭代参数
    uint32_t max_iter = 15;
    double init_lamb = 5.0;
    double lamb_decay = 0.7;
    double lamb_amplify = 5.0;
    double max_lamb = 600.0;
    double convergence_threshold = 0.05;
    double accept_step_threshold = 0.3;
    
    // 车辆参数
    double width = 2.0;
    double length = 4.5;
    double velo_max = 10.0;
    double velo_min = 0.0;
    double acc_max = 3.0;
    double acc_min = -3.0;
    double kappa_max = 0.3;
    double kappa_min = -0.3;
    double dkappa_max = 0.1;
    double dkappa_min = -0.1;
    double d_safe = 0.8;
};

}  

#endif
