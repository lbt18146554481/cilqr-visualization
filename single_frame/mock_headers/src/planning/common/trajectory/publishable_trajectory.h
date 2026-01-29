/*
 * Mock header for PublishableTrajectory
 */

#pragma once

#include <vector>
#include "src/planning/common/trajectory/trajectory_point.h"

namespace ceshi {
namespace planning {

class PublishableTrajectory {
public:
    void set_trajectory_points(const std::vector<TrajectoryPoint>& points) {
        points_ = points;
    }
    const std::vector<TrajectoryPoint>& trajectory_points() const {
        return points_;
    }
private:
    std::vector<TrajectoryPoint> points_;
};

} // namespace planning
} // namespace ceshi
