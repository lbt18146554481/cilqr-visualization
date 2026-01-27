/*
 * Mock header for ThirdOrderSplinePath
 */

#pragma once

#include <vector>

namespace ceshi {
namespace planning {

namespace ThirdOrderSplinePath {

struct State {
    double state_l0 = 0.0;
    double state_dl = 0.0;
    double state_ddl = 0.0;
};

struct OptVariables {
    State xk;
    struct {
        double control_dddl = 0.0;
    } uk;
};

} // namespace ThirdOrderSplinePath

} // namespace planning
} // namespace ceshi
