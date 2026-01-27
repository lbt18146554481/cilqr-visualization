/*
 * Mock header for InsidePlannerData
 */

#pragma once

#include "src/planning/common/path/path_point.h"

namespace ceshi {
namespace planning {

struct InsidePlannerData {
    double vel_x = 0.0;
    double vel_y = 0.0;
    double vel_v = 0.0;
    double vel_heading = 0.0;
    double vel_steer_angle = 0.0;
    SLPoint init_sl_point;
};

} // namespace planning
} // namespace ceshi
