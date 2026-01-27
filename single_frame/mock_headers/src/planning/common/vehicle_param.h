/*
 * Mock header for VehicleParam
 */

#pragma once

namespace ceshi {
namespace planning {

class VehicleParam {
public:
    static VehicleParam* Instance() {
        static VehicleParam instance;
        return &instance;
    }
    
    double wheel_base() const { return 2.7; }
    double max_steer_angle() const { return 30.0; }
    double steer_ratio() const { return 16.0; }
    
private:
    VehicleParam() = default;
};

} // namespace planning
} // namespace ceshi
