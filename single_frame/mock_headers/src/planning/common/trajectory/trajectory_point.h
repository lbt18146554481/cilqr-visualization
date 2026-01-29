/*
 * Mock header for TrajectoryPoint
 * Based on /Users/lubitong/Desktop/algorithm/定义/trajectory_point.h
 * Adapted to ceshi::planning namespace
 */

#pragma once

#include "src/planning/common/path/path_point.h"

namespace ceshi {
namespace planning {

class TrajectoryPoint {
public:
    TrajectoryPoint() = default;
    
    TrajectoryPoint(const PathPoint& path_point, const double velocity,
                   const double acceleration, const double jerk,
                   const double relative_time)
        : path_point_(path_point), velocity_(velocity), 
          acceleration_(acceleration), jerk_(jerk), 
          relative_time_(relative_time), has_path_point_(true) {}
    
    // 直接访问方法（兼容原有代码）
    double x() const { return path_point_.x(); }
    double y() const { return path_point_.y(); }
    double theta() const { return path_point_.theta(); }
    double kappa() const { return path_point_.kappa(); }
    double velocity() const { return velocity_; }
    double s() const { return path_point_.s(); }
    double relative_time() const { return relative_time_; }
    double acceleration() const { return acceleration_; }
    double jerk() const { return jerk_; }
    bool has_path_point() const { return has_path_point_; }
    
    // path_point相关方法（trajectory_smoother需要）
    const PathPoint& path_point() const { return path_point_; }
    PathPoint* mutable_path_point() { return &path_point_; }
    
    // setter方法
    void set_x(double x) { path_point_.set_x(x); }
    void set_y(double y) { path_point_.set_y(y); }
    void set_theta(double theta) { path_point_.set_theta(theta); }
    void set_kappa(double kappa) { path_point_.set_kappa(kappa); }
    void set_velocity(double v) { velocity_ = v; }
    void set_s(double s) { path_point_.set_s(s); }
    void set_relative_time(double t) { relative_time_ = t; }
    void set_acceleration(double a) { acceleration_ = a; }
    void set_jerk(double j) { jerk_ = j; }
    void set_path_point(const PathPoint& path_point) { 
        path_point_ = path_point; 
        has_path_point_ = true;
    }
    
    void Clear() {
        path_point_ = PathPoint();
        velocity_ = 0.0;
        acceleration_ = 0.0;
        jerk_ = 0.0;
        relative_time_ = 0.0;
        has_path_point_ = false;
    }
    
private:
    PathPoint path_point_;
    double velocity_ = 0.0;
    double acceleration_ = 0.0;
    double jerk_ = 0.0;
    double relative_time_ = 0.0;
    bool has_path_point_ = false;
};

} // namespace planning
} // namespace ceshi
