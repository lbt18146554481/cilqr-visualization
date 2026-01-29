/*
 * Mock header for PathPoint (ceshi namespace)
 * Includes PathModel namespace for Frenet coordinates
 */

#pragma once

#include <vector>
#include <memory>
#include <limits>
#include <cmath>

namespace ceshi {
namespace planning {

// Forward declaration for CilqrCarModel types
namespace CilqrCarModel {
    struct PieceBoundary;
}

// PieceBoundary type definition (used by trajectory_smoother)
// Based on /Users/lubitong/Desktop/algorithm/定义/path_context.h
struct PieceBoundary {
    bool left_is_obs = false;
    bool right_is_obs = false;
    double s = 0.0;           // 采样点的纵向位置
    double left_bound = 0.0;  // 左边界（横向距离/相对l坐标）
    double right_bound = 0.0; // 右边界
    double left_lane_bound = 0.0;  // navigation lane bound
    double right_lane_bound = 0.0;
    double left_road_bound = 0.0;  // navigation road bound
    double right_road_bound = 0.0;
    double x = 0.0;  // 可选：用于存储xy坐标
    double y = 0.0;
    bool is_corner_vertex = false;
    
    void Reset() {
        left_is_obs = false;
        right_is_obs = false;
        s = 0.0;
        left_bound = 0.0;
        right_bound = 0.0;
        left_lane_bound = 0.0;
        right_lane_bound = 0.0;
        left_road_bound = 0.0;
        right_road_bound = 0.0;
        x = 0.0;
        y = 0.0;
        is_corner_vertex = false;
    }
};

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

class Vec2d {
public:
    Vec2d() : x_(0.0), y_(0.0) {}
    Vec2d(double x, double y) : x_(x), y_(y) {}
    double x() const { return x_; }
    double y() const { return y_; }
    void set_x(double x) { x_ = x; }
    void set_y(double y) { y_ = y; }
private:
    double x_, y_;
};

// TrajectoryPoint 已移动到 src/planning/common/trajectory/trajectory_point.h
// 前向声明以保持兼容性
namespace ceshi {
namespace planning {
    class TrajectoryPoint;  // 完整定义在 trajectory/trajectory_point.h
} // namespace planning
} // namespace ceshi

class ReferencePoint {
public:
    ReferencePoint() = default;
    ReferencePoint(double x, double y, double theta, double kappa, double dkappa, double ddkappa, double s)
        : x_(x), y_(y), theta_(theta), kappa_(kappa), dkappa_(dkappa), ddkappa_(ddkappa), s_(s) {}
    
    double x() const { return x_; }
    double y() const { return y_; }
    double theta() const { return theta_; }
    double heading() const { return theta_; }  // Alias for theta
    double kappa() const { return kappa_; }
    double dkappa() const { return dkappa_; }
    double ddkappa() const { return ddkappa_; }
    double s() const { return s_; }
    double left_bound() const { return left_bound_; }
    double right_bound() const { return right_bound_; }
    
    void set_left_bound(double lb) { left_bound_ = lb; }
    void set_right_bound(double rb) { right_bound_ = rb; }
    
private:
    double x_ = 0.0, y_ = 0.0, theta_ = 0.0, kappa_ = 0.0, dkappa_ = 0.0, ddkappa_ = 0.0, s_ = 0.0;
    double left_bound_ = 3.0, right_bound_ = -3.0;  // Default bounds
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
    bool GetPointInCartesianFrame(const SLPoint& sl, Vec2d* xy) const {
        if (xy) {
            xy->set_x(sl.s());
            xy->set_y(sl.l());
        }
        return true;
    }
    bool GetPointInFrenetFrame(const Vec2d& xy, SLPoint* sl) const {
        if (!sl || points_.empty()) {
            return false;
        }
        
        // 找到最近的参考点
        double min_dist = std::numeric_limits<double>::max();
        size_t nearest_idx = 0;
        double nearest_s = 0.0;
        
        for (size_t i = 0; i < points_.size(); ++i) {
            double dx = xy.x() - points_[i].x();
            double dy = xy.y() - points_[i].y();
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
                nearest_s = points_[i].s();
            }
        }
        
        // 计算横向偏移 l
        if (nearest_idx < points_.size()) {
            const auto& ref_pt = points_[nearest_idx];
            double ref_theta = ref_pt.theta();
            double dx = xy.x() - ref_pt.x();
            double dy = xy.y() - ref_pt.y();
            
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
    bool GetNearestRefPoint(const Vec2d& xy, ReferencePoint* ref) const {
        if (!ref || points_.empty()) {
            return false;
        }
        
        // 找到最近的参考点
        double min_dist = std::numeric_limits<double>::max();
        size_t nearest_idx = 0;
        
        for (size_t i = 0; i < points_.size(); ++i) {
            double dx = xy.x() - points_[i].x();
            double dy = xy.y() - points_[i].y();
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        if (nearest_idx < points_.size()) {
            *ref = points_[nearest_idx];
            return true;
        }
        
        return false;
    }
    
private:
    std::vector<ReferencePoint> points_;
};

using ReferenceLineShrPtr = std::shared_ptr<ReferenceLine>;

// PathModel namespace for Frenet coordinates
namespace PathModel {

struct PathModelState {
    double state_l = 0.0;
    double state_dl = 0.0;
    double state_ddl = 0.0;
};

struct PathModelControl {
    double control_dddl = 0.0;
};

} // namespace PathModel

using PathModelOutput = std::pair<std::vector<PathModel::PathModelControl>, std::vector<PathModel::PathModelState>>;

} // namespace planning
} // namespace ceshi
