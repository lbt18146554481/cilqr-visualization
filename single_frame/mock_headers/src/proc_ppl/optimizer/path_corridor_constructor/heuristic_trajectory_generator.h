/*
 * Mock header for HeuristicTrajectoryGenerator
 */

#pragma once

#include <memory>
#include <vector>
#include "src/common/reference_line/reference_line.h"
#include "src/common/trajectory/trajectory_point.h"

namespace neolix {
namespace planning {

class ReferencePath;
struct CoarseTrajectoryInfo {
    std::shared_ptr<ReferencePath> reference_path_ptr;
};

class ReferencePath {
public:
    void set_reference_line(std::shared_ptr<ReferenceLine> ref_line) {}
    std::vector<double>* mutable_init_s_state() {
        static std::vector<double> s_state(1, 0.0);
        return &s_state;
    }
    TrajectoryPoint* mutable_init_trajectory_point() {
        static TrajectoryPoint point;
        return &point;
    }
};

class Trajectory;

class HeuristicTrajectoryGenerator {
public:
    HeuristicTrajectoryGenerator(const CoarseTrajectoryInfo& info) {}
    bool Init() { return true; }
    bool Run(Trajectory* traj, Trajectory* frenet_traj) {
        // Mock implementation - return success
        return true;
    }
};

} // namespace planning
} // namespace neolix

// Alias for ad namespace (used in planning_test_execute.cpp)
namespace ad = neolix::planning;
