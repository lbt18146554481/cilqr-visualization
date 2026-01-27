/*
 * Mock header for planning_config_manager
 * This is a simplified version for compilation purposes
 */

#pragma once

#include <string>
#include <memory>

namespace config {

struct CilqrTrajectoryConf {
    double dt = 0.2;
    int horizon = 40;
    int min_horizon = 5;
    double min_speed = 2.0;
    double position_weight = 1.0;
    double heading_weight = 1.0;
    double kappa_weight = 4.0;
    double speed_weight = 1.0;
    double acc_weight = 1.0;
    double dkappa_weight = 2.0;
    double jerk_weight = 1.0;
    // Add more fields as needed
};

struct CilqrHeuristicTrajectoryConf {
    double dt = 0.2;
    double planned_time_length = 6.0;
    double planned_min_v = 2.0;
};

struct ReferencePathConf {
    bool enable_nudge = true;
};

struct PlanningConfig {
    CilqrTrajectoryConf cilqr_trajectory_conf;
    CilqrHeuristicTrajectoryConf cilqr_heuristic_trajectory_conf;
    ReferencePathConf reference_path_conf;
};

class PlanningConfigManager {
public:
    static PlanningConfigManager* Instance() {
        static PlanningConfigManager instance;
        return &instance;
    }
    
    void Init() {
        // Initialize with default values
    }
    
    PlanningConfig& planning_config() {
        return config_;
    }
    
    const PlanningConfig& planning_config() const {
        return config_;
    }
    
    PlanningConfig* mutable_planning_config() {
        return &config_;
    }
    
private:
    PlanningConfig config_;
};

} // namespace config
