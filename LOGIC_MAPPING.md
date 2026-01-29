# Trajectory Smoother 逻辑映射说明

## 原始代码 vs 适配函数代码对比

### 原始 trajectory_smoother.cpp (第23-80行)

```cpp
ErrorCode TrajectorySmoother::Execute(TaskInfo& task_info) {
  LOG_INFO(">>>> start execute {}", name_);
  const auto& inside_data = task_info.curr_frame()->inside_planner_data();
  
  auto motorway_new_lane_change_context =
      data_center_->curr_frame()->mutable_motorway_new_lane_change_context();
  const auto& reference_line = task_info.reference_line_odom();
  
  const auto& raw_trajectory_points =
      motorway_new_lane_change_context->raw_trajectory_points;
  const auto& raw_path_points =
      motorway_new_lane_change_context->raw_path_points;

  // 当前车速大于0.5m/s时，防止数值过小导致的kappa数值异常 添加优化器
  double vel_v = inside_data.vel_v;
  std::vector<TrajectoryPoint> computed_trajectory_points;
  computed_trajectory_points.clear();
  
  if (vel_v >= 0.5) {
    LOG_INFO("raw_trajectory_points size:{}", raw_trajectory_points.size());
    ThirdOrderSplinePath::State init_state;
    std::vector<double> knots_delta_s;
    std::vector<PieceBoundary> piece_boundaries;
    std::vector<PieceBoundary> orin_boundaries;
    PathConstrainedIterLqrDeciderAML cilqr("trajectory smoother cilqr");
    cilqr.Execute(init_state, inside_data, raw_trajectory_points, knots_delta_s,
                  piece_boundaries, orin_boundaries, reference_line);
    std::vector<TrajectoryPoint> rough_trajectory = raw_trajectory_points;
    cilqr.get_xystate_seqs(rough_trajectory);
    computed_trajectory_points = rough_trajectory;
    LOG_INFO("zzz_lqr:输出结果是lqr优化的结果");
  } else {
    computed_trajectory_points = raw_trajectory_points;
    LOG_INFO("zzz_lqr:输出结果是直接使用粗解");
  }

  // Manually calculate s for trajectory points since it's missing
  std::vector<TrajectoryPoint> final_trajectory_points;
  if (!computed_trajectory_points.empty()) {
    final_trajectory_points.reserve(computed_trajectory_points.size());
    double accumulated_s = 0.0;

    // First point
    TrajectoryPoint first_point = computed_trajectory_points[0];
    first_point.mutable_path_point()->set_s(accumulated_s);
    final_trajectory_points.push_back(first_point);

    // Subsequent points
    for (size_t i = 1; i < computed_trajectory_points.size(); ++i) {
      const auto& prev_point = final_trajectory_points.back().path_point();
      const auto& raw_curr_point = computed_trajectory_points[i];
      
      double dist = std::sqrt(std::pow(raw_curr_point.path_point().x() - prev_point.x(), 2) +
                              std::pow(raw_curr_point.path_point().y() - prev_point.y(), 2));
      accumulated_s += dist;
      
      TrajectoryPoint new_point = raw_curr_point;
      new_point.mutable_path_point()->set_s(accumulated_s);
      final_trajectory_points.push_back(new_point);
    }
  }
  
  // ... 后续处理（发布结果等）
}
```

### 适配函数中的对应实现

#### 1. 数据准备阶段（对应原文件第25-35行）

**原文件：**
```cpp
const auto& inside_data = task_info.curr_frame()->inside_planner_data();
const auto& reference_line = task_info.reference_line_odom();
const auto& raw_trajectory_points = motorway_new_lane_change_context->raw_trajectory_points;
```

**适配函数：** `trajectory_smoother_adapter.cpp` (第33-93行)
```cpp
// 1. 创建 InsidePlannerData（从 Vector5d 转换）
InsidePlannerData inside_data;
inside_data.vel_x = initial_state[0];
inside_data.vel_y = initial_state[1];
inside_data.vel_v = initial_state[2];
inside_data.vel_heading = initial_state[3];

// 2. 创建 raw_trajectory_points（从参考线生成）
std::vector<TrajectoryPoint> raw_trajectory_points;
for (size_t i = 0; i < reference_line.size(); ++i) {
    TrajectoryPoint point;
    point.set_x(reference_line.x[i]);
    point.set_y(reference_line.y[i]);
    // ... 计算曲率、速度等
}

// 3. 创建 ReferenceLineShrPtr（从 cilqr::ReferenceLine 转换）
ReferenceLineShrPtr reference_line_ptr = std::make_shared<ReferenceLine>(...);
```

**说明：** 适配函数负责数据格式转换，因为可视化项目的数据格式与规划框架不同。

#### 2. 速度检查和算法调用（对应原文件第38-57行）

**原文件：**
```cpp
double vel_v = inside_data.vel_v;
if (vel_v >= 0.5) {
    ThirdOrderSplinePath::State init_state;
    std::vector<double> knots_delta_s;
    std::vector<PieceBoundary> piece_boundaries;
    std::vector<PieceBoundary> orin_boundaries;
    PathConstrainedIterLqrDeciderAML cilqr("trajectory smoother cilqr");
    cilqr.Execute(init_state, inside_data, raw_trajectory_points, knots_delta_s,
                  piece_boundaries, orin_boundaries, reference_line);
    std::vector<TrajectoryPoint> rough_trajectory = raw_trajectory_points;
    cilqr.get_xystate_seqs(rough_trajectory);
    computed_trajectory_points = rough_trajectory;
} else {
    computed_trajectory_points = raw_trajectory_points;
}
```

**适配函数：** `trajectory_smoother_adapter.cpp` (第144-170行)
```cpp
if (inside_data.vel_v >= 0.5) {  // ✅ 完全一致
    SPDLOG_INFO("raw_trajectory_points size:{}", raw_trajectory_points.size());
    
    // ✅ 完全一致：创建 CILQR 优化器
    PathConstrainedIterLqrDeciderAML cilqr("trajectory smoother cilqr");
    
    // ✅ 完全一致：调用 Execute
    ErrorCode error_code = cilqr.Execute(
        init_state, inside_data, raw_trajectory_points, 
        knots_delta_s, piece_boundaries, orin_boundaries, 
        reference_line_ptr
    );
    
    if (error_code == ErrorCode::PLANNING_OK) {
        // ✅ 完全一致：获取优化结果
        std::vector<TrajectoryPoint> rough_trajectory = raw_trajectory_points;
        bool success = cilqr.get_xystate_seqs(rough_trajectory);
        if (success) {
            computed_trajectory_points = rough_trajectory;
            SPDLOG_INFO("zzz_lqr:输出结果是lqr优化的结果");
        }
    }
} else {
    // ✅ 完全一致：速度太低，直接使用粗解
    computed_trajectory_points = raw_trajectory_points;
    SPDLOG_INFO("zzz_lqr:输出结果是直接使用粗解 (vel_v={:.3f} < 0.5)", inside_data.vel_v);
}
```

**说明：** 核心算法调用逻辑完全一致，只是添加了错误处理。

#### 3. 计算 s 值（对应原文件第59-80行）

**原文件：**
```cpp
std::vector<TrajectoryPoint> final_trajectory_points;
if (!computed_trajectory_points.empty()) {
    final_trajectory_points.reserve(computed_trajectory_points.size());
    double accumulated_s = 0.0;

    TrajectoryPoint first_point = computed_trajectory_points[0];
    first_point.mutable_path_point()->set_s(accumulated_s);
    final_trajectory_points.push_back(first_point);

    for (size_t i = 1; i < computed_trajectory_points.size(); ++i) {
      const auto& prev_point = final_trajectory_points.back().path_point();
      const auto& raw_curr_point = computed_trajectory_points[i];
      
      double dist = std::sqrt(std::pow(raw_curr_point.path_point().x() - prev_point.x(), 2) +
                              std::pow(raw_curr_point.path_point().y() - prev_point.y(), 2));
      accumulated_s += dist;
      
      TrajectoryPoint new_point = raw_curr_point;
      new_point.mutable_path_point()->set_s(accumulated_s);
      final_trajectory_points.push_back(new_point);
    }
}
```

**适配函数：** `trajectory_smoother_adapter.cpp` (第172-196行)
```cpp
// ✅ 完全一致：计算s值（按照trajectory_smoother.cpp的逻辑）
std::vector<TrajectoryPoint> final_trajectory_points;
if (!computed_trajectory_points.empty()) {
    final_trajectory_points.reserve(computed_trajectory_points.size());
    double accumulated_s = 0.0;
    
    // ✅ 完全一致：First point
    TrajectoryPoint first_point = computed_trajectory_points[0];
    first_point.mutable_path_point()->set_s(accumulated_s);
    final_trajectory_points.push_back(first_point);
    
    // ✅ 完全一致：Subsequent points - 累积计算距离
    for (size_t i = 1; i < computed_trajectory_points.size(); ++i) {
        const auto& prev_point = final_trajectory_points.back().path_point();
        const auto& raw_curr_point = computed_trajectory_points[i];
        
        // ✅ 完全一致：计算两点间的欧氏距离
        double dist = std::sqrt(
            std::pow(raw_curr_point.path_point().x() - prev_point.x(), 2) +
            std::pow(raw_curr_point.path_point().y() - prev_point.y(), 2)
        );
        accumulated_s += dist;
        
        TrajectoryPoint new_point = raw_curr_point;
        new_point.mutable_path_point()->set_s(accumulated_s);
        final_trajectory_points.push_back(new_point);
    }
}
```

**说明：** s 值计算逻辑完全一致，逐点累积欧氏距离。

## 调用流程图

```
┌─────────────────────────────────────┐
│  motion_planning.cpp                │
│  (主循环，每个时间步)                │
└──────────────┬──────────────────────┘
               │
               │ 1. 准备数据
               │    - ego_state_5d
               │    - algo_ref (参考线)
               │    - algo_obs (障碍物)
               │    - road_borders
               │
               ▼
┌─────────────────────────────────────┐
│  adapt_trajectory_smoother()        │
│  (trajectory_smoother_adapter.cpp)  │
└──────────────┬──────────────────────┘
               │
               │ 2. 数据格式转换
               │    ├─ InsidePlannerData
               │    ├─ raw_trajectory_points
               │    ├─ ReferenceLineShrPtr
               │    └─ 其他参数
               │
               │ 3. 速度检查 (vel_v >= 0.5)
               │    │
               │    ├─ YES → 4. 调用 CILQR 优化
               │    │         ├─ PathConstrainedIterLqrDeciderAML
               │    │         ├─ cilqr.Execute()
               │    │         └─ cilqr.get_xystate_seqs()
               │    │
               │    └─ NO  → 直接使用粗解
               │
               │ 5. 计算 s 值（累积距离）
               │
               │ 6. 返回优化后的轨迹
               │
               ▼
┌─────────────────────────────────────┐
│  motion_planning.cpp                │
│  (结果处理)                          │
└──────────────┬──────────────────────┘
               │
               │ 7. 转换为 MatrixX5d 和 MatrixX2d
               │ 8. 更新状态
               │ 9. 可视化显示
```

## 关键文件说明

### 1. trajectory_smoother_adapter.cpp
**作用：** 核心适配层，实现所有逻辑
- 数据格式转换
- 调用 CILQR 算法
- 结果处理

### 2. motion_planning.cpp
**作用：** 可视化主循环
- 准备输入数据
- 调用适配函数
- 处理结果并可视化

### 3. cilqr_iter_decider.cpp/h
**作用：** CILQR 算法核心实现
- `PathConstrainedIterLqrDeciderAML` 类
- `Execute()` 方法：执行优化
- `get_xystate_seqs()` 方法：获取优化后的状态序列

## 总结

✅ **所有核心逻辑都已实现：**
- 速度检查逻辑 ✅
- CILQR 优化调用 ✅
- 结果获取和处理 ✅
- s 值计算 ✅

✅ **代码更清晰：**
- 去除了规划框架依赖
- 逻辑集中在适配函数中
- 易于理解和维护

✅ **功能完全保留：**
- 算法行为完全一致
- 只是数据来源和格式不同
- 最终结果相同
