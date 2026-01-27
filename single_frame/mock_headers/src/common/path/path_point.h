/*
 * Mock header for PathPoint
 */

#pragma once

namespace neolix {
namespace planning {

class PathPoint {
public:
    double x() const { return x_; }
    double y() const { return y_; }
    double theta() const { return theta_; }
    double kappa() const { return kappa_; }
    double s() const { return s_; }
    
    void set_x(double x) { x_ = x; }
    void set_y(double y) { y_ = y; }
    void set_theta(double theta) { theta_ = theta; }
    void set_kappa(double kappa) { kappa_ = kappa; }
    void set_s(double s) { s_ = s; }
    
private:
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0, kappa_ = 0.0, s_ = 0.0;
};

} // namespace planning
} // namespace neolix
