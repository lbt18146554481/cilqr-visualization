# 轨迹长度问题修复说明

## 问题描述

优化后的轨迹太短，没有启发式轨迹那么长。

## 问题原因

### 1. 预测时域长度（N）硬编码

代码中 `config_.N = 20` 是硬编码的，没有从配置文件读取。

### 2. 轨迹长度计算

- **优化轨迹长度** = N × delta_t = 20 × 0.2 = **4.0 秒**
- **启发式轨迹长度** = planned_time_length = **6.0 秒**
- **配置文件中的 horizon** = 40（应该对应 40 × 0.2 = 8.0 秒）

### 3. 配置未生效

虽然配置文件中设置了 `horizon: 40`，但代码没有读取这个值。

## 解决方案

### 修改内容

在 `cilqr_adapter_simple.cpp` 中，从配置文件读取 `horizon` 并设置到 `config_.N`：

```cpp
// 从配置文件读取 horizon（预测时域长度）
if (cilqr_conf.horizon > 0) {
    config_.N = static_cast<uint32_t>(cilqr_conf.horizon);
} else {
    // 如果没有设置 horizon，则根据 planned_time_length 计算
    double planned_time_length = heuristic_conf.planned_time_length;
    config_.N = static_cast<uint32_t>(std::ceil(planned_time_length / config_.delta_t));
}
```

### 修复效果

修复后：
- **从配置文件读取**：`horizon = 40` → `N = 40` → 轨迹长度 = 40 × 0.2 = **8.0 秒**
- **备用方案**：如果 `horizon` 未设置，根据 `planned_time_length = 6.0` 计算 → `N = 30` → 轨迹长度 = **6.0 秒**

## 验证方法

### 1. 查看日志输出

运行程序时，会输出配置信息：
```
CILQR Config: N=40, delta_t=0.2, trajectory_length=8.0s
```

### 2. 可视化对比

在 Web 界面中：
- 优化轨迹应该和启发式轨迹长度一致（或更长）
- 轨迹点数量应该匹配

### 3. 配置文件

确保 `planning_config.json` 中设置了：
```json
{
    "cilqr_trajectory_conf": {
        "horizon": 40,  // 预测时域长度
        "dt": 0.2       // 时间步长
    },
    "cilqr_heuristic_trajectory_conf": {
        "planned_time_length": 6.0  // 启发式轨迹长度（秒）
    }
}
```

## 轨迹长度计算

### 公式

```
轨迹长度（秒）= N × delta_t
轨迹点数 = N + 1（包含初始状态）
```

### 示例

| horizon | delta_t | 轨迹长度 | 轨迹点数 |
|---------|---------|---------|---------|
| 20 | 0.2 | 4.0s | 21 |
| 30 | 0.2 | 6.0s | 31 |
| 40 | 0.2 | 8.0s | 41 |

## 注意事项

1. **N 不能太大**：会增加计算量，影响实时性
2. **N 不能太小**：轨迹太短，无法覆盖完整路径
3. **建议值**：
   - 城市道路：N = 30-40（6-8秒）
   - 高速公路：N = 50-60（10-12秒）

## 相关文件

- `cilqr_adapter_simple.cpp`: 修复代码
- `planning_config.json`: 配置文件
- `algorithm/include/algorithm_config.hpp`: 算法配置结构
