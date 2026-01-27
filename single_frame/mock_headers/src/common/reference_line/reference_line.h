/*
 * Mock header for ReferenceLine
 */

#pragma once

#include <vector>
#include <memory>

namespace neolix {
namespace planning {

struct ReferencePoint {
    ReferencePoint() = default;
    ReferencePoint(double x, double y, double theta, double kappa, double dkappa, double ddkappa, double s)
        : x_(x), y_(y), theta_(theta), kappa_(kappa), dkappa_(dkappa), ddkappa_(ddkappa), s_(s) {}
    
    double x() const { return x_; }
    double y() const { return y_; }
    double theta() const { return theta_; }
    double kappa() const { return kappa_; }
    double dkappa() const { return dkappa_; }
    double ddkappa() const { return ddkappa_; }
    double s() const { return s_; }
    
private:
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0, kappa_ = 0.0, dkappa_ = 0.0, ddkappa_ = 0.0, s_ = 0.0;
};

struct SLPoint {
    double s() const { return s_; }
    double l() const { return l_; }
    void set_s(double s) { s_ = s; }
    void set_l(double l) { l_ = l; }
private:
    double s_ = 0.0, l_ = 0.0;
};

class ReferenceLine {
public:
    ReferenceLine() = default;
    ReferenceLine(const std::vector<ReferencePoint>& points) : points_(points) {}
    
    const std::vector<ReferencePoint>& ref_points() const { return points_; }
    bool XYToSL(const std::pair<double, double>& xy, SLPoint* sl) const {
        if (!sl || points_.empty()) {
            return false;
        }
        
        // 找到最近的参考点
        double min_dist = std::numeric_limits<double>::max();
        size_t nearest_idx = 0;
        double nearest_s = 0.0;
        
        for (size_t i = 0; i < points_.size(); ++i) {
            double dx = xy.first - points_[i].x();
            double dy = xy.second - points_[i].y();
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
                nearest_s = points_[i].s();
            }
        }
        
        // 计算横向偏移 l
        // 使用最近参考点的法向量
        if (nearest_idx < points_.size()) {
            const auto& ref_pt = points_[nearest_idx];
            double ref_theta = ref_pt.theta();
            double dx = xy.first - ref_pt.x();
            double dy = xy.second - ref_pt.y();
            
            // 将向量投影到参考点的法向量上（垂直于heading）
            double cos_theta = std::cos(ref_theta);
            double sin_theta = std::sin(ref_theta);
            // 法向量是 (-sin_theta, cos_theta)
            double l = -dx * sin_theta + dy * cos_theta;
            
            sl->set_s(nearest_s);
            sl->set_l(l);
            return true;
        }
        
        return false;
    }
    
private:
    std::vector<ReferencePoint> points_;
};

using ReferenceLineShrPtr = std::shared_ptr<ReferenceLine>;

} // namespace planning
} // namespace neolix
