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

} // namespace planning
} // namespace ceshi
