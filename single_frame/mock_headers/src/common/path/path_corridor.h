/*
 * Mock header for PathCorridor
 */

#pragma once

#include "src/common/trajectory/trajectory_point.h"
#include <vector>

namespace neolix {
namespace planning {

class Trajectory {
public:
    const std::vector<TrajectoryPoint>& trajectory_points() const { return points_; }
    std::vector<TrajectoryPoint>* mutable_trajectory_points() { return &points_; }
private:
    std::vector<TrajectoryPoint> points_;
};

struct SLBoundPoint {
    double s = 0.0;
    double hard_lb = -3.0;
    double hard_ub = 3.0;
};

class PathSLCorridor {
public:
    const std::vector<SLBoundPoint>& sl_bound_points() const { return points_; }
    std::vector<SLBoundPoint>* mutable_sl_bound_points() { return &points_; }
private:
    std::vector<SLBoundPoint> points_;
};

class PathCorridor {
public:
    const Trajectory& heuristic_trajectory() const { return heuristic_traj_; }
    Trajectory* mutable_heuristic_trajectory() { return &heuristic_traj_; }
    
    const PathSLCorridor& path_sl_corridor() const { return sl_corridor_; }
    PathSLCorridor* mutable_path_sl_corridor() { return &sl_corridor_; }
    
    // Additional method for frenet trajectory
    Trajectory* mutable_heuristic_frenet_trajectory() { return &frenet_traj_; }
    
private:
    Trajectory heuristic_traj_;
    Trajectory frenet_traj_;  // Frenet coordinate trajectory
    PathSLCorridor sl_corridor_;
};

} // namespace planning
} // namespace neolix
