/*
 * Mock header for ScenarioTaskInterface
 */

#pragma once

namespace ceshi {
namespace planning {

enum class ErrorCode {
    PLANNING_OK = 0,
    PLANNING_ERROR_FAILED = 1
};

struct TaskInfo {
    // Empty for now
};

class ScenarioTaskInterface {
public:
    virtual ~ScenarioTaskInterface() = default;
    virtual ErrorCode Execute(TaskInfo& task_info) { return ErrorCode::PLANNING_OK; }
    virtual void SaveTaskResults(TaskInfo& task_info) {}
    virtual void Reset() {}
};

// Macros for singleton and registration (used in trajectory_smoother but not needed for adapter)
#define DECLARE_SINGLETON(ClassName) \
    static ClassName* Instance() { \
        static ClassName instance; \
        return &instance; \
    }

#define REGISTER_SCENARIO_TASK(ClassName) \
    static void __attribute__((unused)) __register_##ClassName() {}

} // namespace planning
} // namespace ceshi
