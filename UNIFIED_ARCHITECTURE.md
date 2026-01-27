# 统一架构说明文档

## 概述

本项目已统一使用**优化器基类系统**，所有前端都通过统一的 `TrajectoryOptimizer` 基类接口调用算法，实现了架构的统一和扩展性。

## 统一架构的优势

### 1. 前端无需修改

当添加新算法时，前端代码**完全不需要修改**：

```cpp
// 前端代码保持不变
auto optimizer = OptimizerFactory::Create("cilqr", name, config);
// 未来添加 MPC 算法，只需修改工厂类，前端代码不变
auto optimizer = OptimizerFactory::Create("mpc", name, mpc_config);
```

### 2. 统一接口

所有算法都通过相同的接口调用：

```cpp
// 所有算法都使用相同的接口
auto result = optimizer->Execute(...);
optimizer->GetStateSequence(...);
```

### 3. 易于扩展

添加新算法只需：
1. 创建新的优化器类（继承 `TrajectoryOptimizer`）
2. 在工厂类中注册

无需修改任何前端代码。

## 当前架构状态

### ✅ animation 项目（已统一）

**文件**：`animation/src/cilqr_adapter.cpp`

```cpp
// 使用优化器基类系统
optimizer_ = trajectory_optimizer::OptimizerFactory::Create(
    "cilqr", "trajectory smoother cilqr", algo_config_);

// 统一接口调用
auto result = optimizer_->Execute(...);
```

### ✅ single_frame 项目（已统一）

**文件**：`single_frame/cilqr_adapter_simple.cpp`

```cpp
// 使用优化器基类系统
optimizer_ = trajectory_optimizer::OptimizerFactory::Create(
    "cilqr", "single frame cilqr optimizer", config_);

// 统一接口调用
auto result = optimizer_->Execute(...);
```

## 架构层次

```
┌─────────────────────────────────────────┐
│   前端层                                 │
│   - animation/motion_planning.cpp       │
│   - single_frame/planning_test_execute.cpp │
└─────────────────────────────────────────┘
              │
              │ 调用适配器
              ▼
┌─────────────────────────────────────────┐
│   适配器层                               │
│   - CILQRAdapter (animation)            │
│   - CilqrTrajectoryOptimizer (single_frame) │
└─────────────────────────────────────────┘
              │
              │ 使用工厂创建
              ▼
┌─────────────────────────────────────────┐
│   优化器基类系统                         │
│   - OptimizerFactory (工厂类)            │
│   - TrajectoryOptimizer (基类)           │
│   - CILQROptimizer (派生类)              │
└─────────────────────────────────────────┘
              │
              │ 调用
              ▼
┌─────────────────────────────────────────┐
│   算法实现层                             │
│   - CILQRSolver (算法核心)               │
└─────────────────────────────────────────┘
```

## 添加新算法的步骤

### 步骤1：创建优化器类

```cpp
// algorithm/include/mpc_optimizer.hpp
class MPCOptimizer : public TrajectoryOptimizer {
    // 实现基类接口
};
```

### 步骤2：在工厂类中注册

```cpp
// algorithm/include/optimizer_factory.hpp
else if (optimizer_type == "mpc") {
    auto optimizer = std::make_unique<MPCOptimizer>(name, mpc_config);
    if (optimizer->Init()) {
        return optimizer;
    }
}
```

### 步骤3：前端使用（无需修改代码）

```cpp
// 前端代码完全不变，只需修改配置或参数
auto optimizer = OptimizerFactory::Create("mpc", name, mpc_config);
auto result = optimizer->Execute(...);
```

## 统一后的调用流程

### animation 项目

```
motion_planning.cpp
    ↓
CILQRAdapter::solve()
    ↓
OptimizerFactory::Create("cilqr", ...)
    ↓
CILQROptimizer::Execute()
    ↓
CILQRSolver::solve()
```

### single_frame 项目

```
planning_test_execute.cpp
    ↓
CilqrTrajectoryOptimizer::Optimize()
    ↓
OptimizerFactory::Create("cilqr", ...)
    ↓
CILQROptimizer::Execute()
    ↓
CILQRSolver::solve()
```

## 关键改进

### 改进前

- `animation` 使用 `CILQROptimizer`（统一接口）
- `single_frame` 直接使用 `CILQRSolver`（不统一）

### 改进后

- `animation` 使用 `CILQROptimizer`（统一接口）✅
- `single_frame` 使用 `CILQROptimizer`（统一接口）✅

**两个前端现在都使用相同的优化器基类系统！**

## 未来扩展示例

### 添加 MPC 算法

```cpp
// 1. 创建 mpc_optimizer.hpp
class MPCOptimizer : public TrajectoryOptimizer { ... };

// 2. 在工厂类中注册
OptimizerFactory::Create("mpc", ...) { ... }

// 3. 前端使用（代码不变）
auto optimizer = OptimizerFactory::Create("mpc", name, mpc_config);
```

### 前端代码完全不需要修改！

## 总结

✅ **统一架构已完成**
- 所有前端都使用 `TrajectoryOptimizer` 基类
- 通过 `OptimizerFactory` 创建优化器
- 统一的 `Execute()` 接口

✅ **易于扩展**
- 添加新算法只需实现派生类
- 前端代码无需修改
- 配置驱动，灵活切换算法

✅ **代码复用**
- 统一的错误处理
- 统一的结果格式
- 统一的接口规范
