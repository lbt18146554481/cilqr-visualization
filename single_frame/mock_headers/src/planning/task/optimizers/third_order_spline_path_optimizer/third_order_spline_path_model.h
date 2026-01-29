/*
 * Mock header for ThirdOrderSplinePath
 * Based on /Users/lubitong/Desktop/algorithm/定义/third_order_spline_path_model.h
 * Adapted to ceshi::planning namespace
 */

#pragma once

#include <vector>
#include <Eigen/Dense>

namespace ceshi {
namespace planning {

namespace ThirdOrderSplinePath {

struct State {
    double state_l0 = 0.0;
    double state_dl = 0.0;
    double state_ddl = 0.0;
    
    void SetZero() {
        state_l0 = 0.0;
        state_dl = 0.0;
        state_ddl = 0.0;
    }
};

struct Control {
    double control_dddl = 0.0;
    void SetZero() { control_dddl = 0.0; }
};

struct OptVariables {
    State xk{};
    Control uk{};
};

} // namespace ThirdOrderSplinePath

} // namespace planning
} // namespace ceshi
