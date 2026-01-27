/*
 * Mock header for PathData
 */

#pragma once

#include "path_point.h"
#include <vector>

namespace neolix {
namespace planning {

class DiscretizedPath {
public:
    const std::vector<PathPoint>& path_points() const { return points_; }
    std::vector<PathPoint>* mutable_path_points() { return &points_; }
    void clear() { points_.clear(); }
    
private:
    std::vector<PathPoint> points_;
};

class PathData {
public:
    const DiscretizedPath& discretized_path() const { return path_; }
    DiscretizedPath* mutable_discretized_path() { return &path_; }
    
private:
    DiscretizedPath path_;
};

} // namespace planning
} // namespace neolix
