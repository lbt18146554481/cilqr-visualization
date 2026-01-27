/*
 * Mock header for neodrive::common::config::CommonConfig
 */

#pragma once

namespace neodrive {
namespace common {
namespace config {

struct EgoCarConfig {
    double width = 1.8;
    double length = 4.5;
};

class CommonConfig {
public:
    static CommonConfig* Instance() {
        static CommonConfig instance;
        return &instance;
    }
    
    const EgoCarConfig& ego_car_config() const {
        return ego_car_config_;
    }
    
private:
    EgoCarConfig ego_car_config_;
};

} // namespace config
} // namespace common
} // namespace neodrive
