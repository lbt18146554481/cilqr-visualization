/*
 * 简化的CILQR适配器
 * 直接使用参考算法，避免外部依赖
 */

#pragma once

#include <vector>
#include <memory>
#include <string>
#include <Eigen/Dense>

// 前向声明neolix类型（避免编译错误）
namespace neolix {
namespace planning {
    class ReferenceLine;
    class TrajectoryPoint;
    class PathCorridor;
    class PathData;
    
    struct CilqrPathOptimizerDebug {
        struct SolutionInfo {
            std::vector<Eigen::Matrix<double, 6, 1>> final_x_traj;  // 状态轨迹 [x, y, v, heading, kappa, a?]
            std::vector<Eigen::Matrix<double, 2, 1>> final_u_traj;  // 控制轨迹 [dkappa, jerk?]
        } solution_info;
    };
}
}

// 简化的适配器类
class CilqrTrajectoryOptimizer {
public:
    explicit CilqrTrajectoryOptimizer(const neolix::planning::ReferenceLine& reference_line);
    ~CilqrTrajectoryOptimizer();
    
    bool Init();
    bool Optimize(const neolix::planning::TrajectoryPoint& init_point,
                  const neolix::planning::PathCorridor& path_corridor);
    bool FillPathData(neolix::planning::PathData* path_data);
    void ExtractDebug(neolix::planning::CilqrPathOptimizerDebug* debug);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    
    CilqrTrajectoryOptimizer(const CilqrTrajectoryOptimizer&) = delete;
    CilqrTrajectoryOptimizer& operator=(const CilqrTrajectoryOptimizer&) = delete;
};
