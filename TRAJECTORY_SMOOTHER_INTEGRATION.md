# Trajectory Smoother 集成说明

## 概述

`trajectory_smoother.cpp` 和 `trajectory_smoother.h` 已被删除，其核心逻辑已完全在适配函数中实现。本文档详细说明逻辑是如何在其他文件中使用的。

## 原始 trajectory_smoother.cpp 的逻辑

原始文件（第38-58行）的核心逻辑：

```cpp
// 1. 检查速度条件
double vel_v = inside_data.vel_v;
if (vel_v >= 0.5) {
    // 2. 创建 CILQR 优化器
    PathConstrainedIterLqrDeciderAML cilqr("trajectory smoother cilqr");
    
    // 3. 调用 Execute 方法
    cilqr.Execute(init_state, inside_data, raw_trajectory_points, 
                  knots_delta_s, piece_boundaries, orin_boundaries, reference_line);
    
    // 4. 获取优化后的轨迹
    std::vector<TrajectoryPoint> rough_trajectory = raw_trajectory_points;
    cilqr.get_xystate_seqs(rough_trajectory);
    computed_trajectory_points = rough_trajectory;
} else {
    // 5. 速度太低，直接使用粗解
    computed_trajectory_points = raw_trajectory_points;
}

// 6. 计算 s 值（累积距离）
```

## 逻辑在适配函数中的实现

### 文件位置
`animation/src/trajectory_smoother_adapter.cpp`

### 完整调用流程

#### 第一步：motion_planning.cpp 调用适配函数

**文件：** `animation/src/motion_planning.cpp` (第277-280行)

```cpp
// 准备输入数据
cilqr::ReferenceLine algo_ref = utils::convert_to_algo_ref(center_lines[0]);
std::vector<cilqr::RoutingLine> algo_obs = utils::convert_to_algo_routing(...);

// 调用适配函数
std::vector<ceshi::planning::TrajectoryPoint> optimized_trajectory = 
    ceshi::planning::adapt_trajectory_smoother(
        ego_state_5d,      // 初始5D状态 [x, y, v, heading, kappa]
        algo_ref,          // 参考线
        target_velocity,   // 目标速度
        algo_obs,          // 障碍物预测轨迹
        road_borders       // 道路边界
    );
```

#### 第二步：适配函数的数据转换

**文件：** `animation/src/trajectory_smoother_adapter.cpp` (第33-139行)

适配函数执行以下转换：

1. **创建 InsidePlannerData** (第33-43行)
   ```cpp
   InsidePlannerData inside_data;
   inside_data.vel_x = initial_state[0];      // 从 Vector5d 提取
   inside_data.vel_y = initial_state[1];
   inside_data.vel_v = initial_state[2];
   inside_data.vel_heading = initial_state[3];
   ```

2. **创建 raw_trajectory_points** (第45-93行)
   ```cpp
   // 从参考线生成初始轨迹点
   for (size_t i = 0; i < reference_line.size(); ++i) {
       TrajectoryPoint point;
       point.set_x(reference_line.x[i]);
       point.set_y(reference_line.y[i]);
       point.set_theta(reference_line.yaw[i]);
       // 计算曲率、速度、时间等
   }
   ```

3. **创建 ReferenceLineShrPtr** (第97-118行)
   ```cpp
   // 将 cilqr::ReferenceLine 转换为 ReferenceLineShrPtr
   auto ref_line = std::make_shared<ReferenceLine>();
   // 填充 ReferencePoint 数据
   ```

4. **创建其他参数** (第122-139行)
   ```cpp
   ThirdOrderSplinePath::State init_state;  // 初始状态
   std::vector<double> knots_delta_s;       // 采样间隔
   std::vector<PieceBoundary> piece_boundaries;  // 边界信息
   ```

#### 第三步：核心算法调用（完全按照 trajectory_smoother.cpp 的逻辑）

**文件：** `animation/src/trajectory_smoother_adapter.cpp` (第141-170行)

```cpp
// 5. 调用 trajectory_smoother 的代码（完全按照trajectory_smoother.cpp的逻辑）
std::vector<TrajectoryPoint> computed_trajectory_points;

if (inside_data.vel_v >= 0.5) {  // ✅ 对应原文件第41行
    SPDLOG_INFO("raw_trajectory_points size:{}", raw_trajectory_points.size());
    
    // ✅ 对应原文件第47行：创建 CILQR 优化器
    PathConstrainedIterLqrDeciderAML cilqr("trajectory smoother cilqr");
    
    // ✅ 对应原文件第48-49行：调用 Execute
    ErrorCode error_code = cilqr.Execute(
        init_state, inside_data, raw_trajectory_points, 
        knots_delta_s, piece_boundaries, orin_boundaries, 
        reference_line_ptr
    );
    
    if (error_code == ErrorCode::PLANNING_OK) {
        // ✅ 对应原文件第50-52行：获取优化结果
        std::vector<TrajectoryPoint> rough_trajectory = raw_trajectory_points;
        bool success = cilqr.get_xystate_seqs(rough_trajectory);
        if (success) {
            computed_trajectory_points = rough_trajectory;
            SPDLOG_INFO("zzz_lqr:输出结果是lqr优化的结果");
        }
    }
} else {
    // ✅ 对应原文件第54-56行：速度太低，直接使用粗解
    computed_trajectory_points = raw_trajectory_points;
    SPDLOG_INFO("zzz_lqr:输出结果是直接使用粗解 (vel_v={:.3f} < 0.5)", inside_data.vel_v);
}
```

#### 第四步：计算 s 值（累积距离）

**文件：** `animation/src/trajectory_smoother_adapter.cpp` (第172-196行)

```cpp
// 6. 计算s值（按照trajectory_smoother.cpp的逻辑）
// ✅ 对应原文件第59-80行：手动计算 s 值
std::vector<TrajectoryPoint> final_trajectory_points;
if (!computed_trajectory_points.empty()) {
    double accumulated_s = 0.0;
    
    // First point
    TrajectoryPoint first_point = computed_trajectory_points[0];
    first_point.mutable_path_point()->set_s(accumulated_s);
    final_trajectory_points.push_back(first_point);
    
    // Subsequent points - 累积计算距离
    for (size_t i = 1; i < computed_trajectory_points.size(); ++i) {
        const auto& prev_point = final_trajectory_points.back().path_point();
        const auto& raw_curr_point = computed_trajectory_points[i];
        
        // 计算两点间的欧氏距离
        double dist = std::sqrt(
            std::pow(raw_curr_point.path_point().x() - prev_point.x(), 2) +
            std::pow(raw_curr_point.path_point().y() - prev_point.y(), 2)
        );
        accumulated_s += dist;  // 累积 s 值
        
        TrajectoryPoint new_point = raw_curr_point;
        new_point.mutable_path_point()->set_s(accumulated_s);
        final_trajectory_points.push_back(new_point);
    }
}
```

#### 第五步：结果转换回可视化格式

**文件：** `animation/src/motion_planning.cpp` (第282-306行)

```cpp
// 将TrajectoryPoint转换为MatrixX5d和MatrixX2d格式
size_t traj_size = optimized_trajectory.size();
new_x_5d = MatrixX5d::Zero(traj_size, 5);
new_u = Eigen::MatrixX2d::Zero(traj_size > 0 ? traj_size - 1 : 0, 2);

for (size_t i = 0; i < traj_size; ++i) {
    // 提取状态信息
    new_x_5d(i, 0) = optimized_trajectory[i].x();
    new_x_5d(i, 1) = optimized_trajectory[i].y();
    new_x_5d(i, 2) = optimized_trajectory[i].velocity();
    new_x_5d(i, 3) = optimized_trajectory[i].theta();
    new_x_5d(i, 4) = optimized_trajectory[i].kappa();
    
    // 计算控制输入（加速度和曲率变化率）
    if (i > 0 && i - 1 < new_u.rows()) {
        double dt = optimized_trajectory[i].relative_time() - 
                    optimized_trajectory[i-1].relative_time();
        new_u(i-1, 0) = optimized_trajectory[i].acceleration();
        new_u(i-1, 1) = (optimized_trajectory[i].kappa() - 
                        optimized_trajectory[i-1].kappa()) / dt;
    }
}
```

## 关键差异说明

### 为什么不需要 trajectory_smoother.cpp？

1. **规划框架依赖**
   - 原文件依赖 `TaskInfo& task_info`、`data_center_` 等规划框架接口
   - 可视化项目中没有这些接口

2. **直接调用核心算法**
   - 适配函数直接使用 `PathConstrainedIterLqrDeciderAML` 类
   - 跳过了 `TrajectorySmoother` 包装层

3. **数据格式转换**
   - 适配函数负责将可视化项目的数据格式转换为算法需要的格式
   - 原文件假设数据已经在规划框架的格式中

## 文件依赖关系

```
motion_planning.cpp
    ↓ 调用
trajectory_smoother_adapter.cpp
    ↓ 使用
PathConstrainedIterLqrDeciderAML (cilqr_iter_decider.h/cpp)
    ↓ 使用
PathCilqrSolverAML (cilqr_solver.h/cpp)
    ↓ 使用
PathCilqrModelCar (cilqr_model.h/cpp)
```

## 总结

- ✅ **核心逻辑完全保留**：速度检查、CILQR优化、结果获取、s值计算
- ✅ **功能完全实现**：所有关键步骤都在适配函数中实现
- ✅ **代码更简洁**：去除了规划框架依赖，直接使用核心算法类
- ✅ **易于维护**：逻辑集中在适配函数中，便于理解和修改

删除 `trajectory_smoother.cpp` 和 `trajectory_smoother.h` 不会影响功能，因为其核心逻辑已经完全在适配函数中实现。
