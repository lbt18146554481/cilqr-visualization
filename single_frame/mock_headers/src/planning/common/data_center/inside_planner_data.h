/*
 * Mock header for InsidePlannerData
 * Based on /Users/lubitong/Desktop/algorithm/定义/inside_planner_data.h
 * Adapted to ceshi::planning namespace
 */

#pragma once

#include "src/planning/common/path/path_point.h"
#include "src/planning/common/trajectory/trajectory_point.h"

namespace ceshi {
namespace planning {

struct InsidePlannerData {
    // Vehicle environment data (trajectory_smoother主要使用这些)
    double vel_x = 0.0;
    double vel_y = 0.0;
    double vel_v = 0.0;
    double vel_heading = 0.0;
    double vel_a = 0.0;
    double vel_steer_angle = 0.0;  // percent
    
    // Initial point information
    TrajectoryPoint init_point{};
    SLPoint init_sl_point{};
    
    // 其他字段（简化版本，如果需要可以扩展）
    // 参考完整定义：/Users/lubitong/Desktop/algorithm/定义/inside_planner_data.h
};

} // namespace planning
} // namespace ceshi
