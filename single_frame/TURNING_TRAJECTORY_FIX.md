# 转弯轨迹跟踪问题修复说明

## 问题描述

在左拐/右拐场景中，优化后的轨迹在转弯处偏离启发式轨迹，没有按照启发式轨迹的转弯去走。

## 问题原因

### 1. 权重配置过小

在 `planning_config.json` 中，关键权重设置过小：
- `position_weight`: 1（太小，无法有效跟踪位置）
- `heading_weight`: 1（太小，转弯时航向跟踪不足）
- `kappa_weight`: 4（可能不够）

### 2. 参考轨迹权重不足

`ref_x_weight` 默认值为 10.0，在转弯场景中可能不够强，导致优化轨迹偏离参考轨迹。

### 3. 权重覆盖问题

代码中虽然设置了合理的默认值（`w_pos = 10.0`, `w_heading = 20.0`），但从配置文件读取时会直接覆盖这些值，导致权重过小。

## 解决方案

### 1. 代码层面修改

在 `cilqr_adapter_simple.cpp` 中，增加了权重的最小值保护：

```cpp
// 位置权重：至少10.0，确保转弯时能跟踪参考轨迹
config_.w_pos = std::max(cilqr_conf.position_weight, 10.0);
// 航向权重：至少20.0，转弯时航向跟踪很重要
config_.w_heading = std::max(cilqr_conf.heading_weight, 20.0);
// 曲率权重：至少10.0，确保曲率跟踪
config_.w_kappa = std::max(cilqr_conf.kappa_weight, 10.0);
// ref_x_weight：增加参考轨迹权重，确保转弯时能跟踪启发式轨迹
config_.ref_x_weight = std::max(config_.ref_x_weight, 50.0);
```

### 2. 配置文件修改

更新了 `planning_config.json` 中的权重值：

```json
{
    "cilqr_trajectory_conf": {
        "position_weight": 10,    // 从 1 增加到 10
        "heading_weight": 20,     // 从 1 增加到 20
        "kappa_weight": 10        // 从 4 增加到 10
    }
}
```

### 3. 默认值调整

将 `ref_x_weight` 的默认值从 10.0 增加到 50.0：

```cpp
config_.ref_x_weight = 50.0;  // 从 10.0 增加到 50.0
```

## 权重说明

### 状态权重（state_weight）

状态权重矩阵的对角线元素：
- `w_pos`: 位置权重（x, y），影响位置跟踪
- `w_vel`: 速度权重，影响速度跟踪
- `w_heading`: 航向权重，影响航向跟踪（转弯时很重要）
- `w_kappa`: 曲率权重，影响曲率跟踪（转弯时很重要）

### 参考轨迹权重（ref_x_weight）

全局缩放因子，影响整个状态跟踪的强度：
- 值越大，优化轨迹越贴近参考轨迹
- 在转弯场景中需要更大的值

### 代价函数

状态代价的计算公式：
```cpp
c_state = ref_x_weight * state_diff^T * state_weight * state_diff
```

其中：
- `state_diff`: 状态误差（优化状态 - 参考状态）
- `state_weight`: 状态权重矩阵（对角矩阵）
- `ref_x_weight`: 参考轨迹权重（全局缩放因子）

## 效果预期

修改后，优化轨迹应该：
1. ✅ 在转弯处更好地跟踪启发式轨迹
2. ✅ 位置偏差减小
3. ✅ 航向偏差减小
4. ✅ 曲率跟踪更准确

## 如何调整

如果转弯跟踪仍然不够理想，可以进一步调整：

### 增加参考轨迹权重

```cpp
config_.ref_x_weight = 100.0;  // 进一步增加
```

### 增加位置和航向权重

```json
{
    "position_weight": 20,    // 进一步增加
    "heading_weight": 30      // 进一步增加
}
```

### 注意平衡

权重过大可能导致：
- 过度跟踪参考轨迹，忽略约束
- 优化收敛困难
- 控制量变化剧烈

建议逐步调整，找到平衡点。

## 测试建议

1. 测试左拐场景（`ref_mode: "left_turn"`）
2. 测试右拐场景（`ref_mode: "right_turn"`）
3. 测试 U 型转弯（`ref_mode: "u_turn"`）
4. 观察优化轨迹与启发式轨迹的偏差

## 相关文件

- `cilqr_adapter_simple.cpp`: 权重配置代码
- `planning_config.json`: 配置文件
- `algorithm/src/cilqr_solver.cpp`: 代价函数实现
