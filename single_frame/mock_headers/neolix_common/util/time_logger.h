/*
 * Mock header for TimeLogger
 */

#pragma once

namespace neolix {
namespace common {
namespace util {

class TimeLogger {
public:
    static TimeLogger& Instance() {
        static TimeLogger instance;
        return instance;
    }
    
    void Start(const std::string& name) {}
    void End(const std::string& name) {}
};

} // namespace util
} // namespace common
} // namespace neolix
