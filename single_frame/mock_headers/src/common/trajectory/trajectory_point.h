/*
 * Mock header for TrajectoryPoint
 */

#pragma once

namespace neolix {
namespace planning {

class TrajectoryPoint {
public:
    TrajectoryPoint() = default;
    
    double x() const { return x_; }
    double y() const { return y_; }
    double theta() const { return theta_; }
    double kappa() const { return kappa_; }
    double dkappa() const { return dkappa_; }
    double velocity() const { return velocity_; }
    double acceleration() const { return acceleration_; }
    double relative_time() const { return relative_time_; }
    double s() const { return s_; }
    
    void set_x(double x) { x_ = x; }
    void set_y(double y) { y_ = y; }
    void set_theta(double theta) { theta_ = theta; }
    void set_kappa(double kappa) { kappa_ = kappa; }
    void set_dkappa(double dkappa) { dkappa_ = dkappa; }
    void set_velocity(double v) { velocity_ = v; }
    void set_acceleration(double a) { acceleration_ = a; }
    void set_relative_time(double t) { relative_time_ = t; }
    void set_s(double s) { s_ = s; }
    
private:
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0, kappa_ = 0.0, dkappa_ = 0.0;
    double velocity_ = 0.0, acceleration_ = 0.0, relative_time_ = 0.0, s_ = 0.0;
};

} // namespace planning
} // namespace neolix
