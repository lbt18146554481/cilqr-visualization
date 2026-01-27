# 优化器基类使用指南

## 概述

本项目实现了统一的轨迹优化器基类接口，参考了 `trajectory_smoother.cpp` 的设计风格。所有优化算法都继承自 `TrajectoryOptimizer` 基类，提供统一的调用接口。

## 架构设计

```
TrajectoryOptimizer (基类)
    ├── CILQROptimizer (CILQR算法实现)
    └── [未来可扩展] MPCOptimizer, APOLLOOptimizer 等
```

## 基本使用

### 方式1: 直接创建优化器

```cpp
#include "trajectory_optimizer.hpp"
#include "cilqr_optimizer.hpp"
#include "algorithm_config.hpp"

// 创建配置
cilqr::AlgorithmConfig config;
config.delta_t = 0.1;
config.N = 20;
// ... 设置其他参数

// 创建优化器（参考 trajectory_smoother.cpp 第47行风格）
auto optimizer = std::make_unique<trajectory_optimizer::CILQROptimizer>(
    "my cilqr optimizer", config);

// 初始化
if (!optimizer->Init()) {
    // 处理初始化失败
    return;
}

// 准备输入
Vector5d initial_state;
initial_state << x0, y0, v0, heading0, kappa0;

cilqr::ReferenceLine reference_line;
// ... 填充参考线数据

std::vector<cilqr::RoutingLine> obstacles;
Eigen::Vector2d road_borders;

// 执行优化（参考 trajectory_smoother.cpp 第48行风格）
auto result = optimizer->Execute(
    initial_state,
    reference_line,
    reference_velocity,
    obstacles,
    road_borders
);

if (result.success) {
    // 获取状态序列（参考 trajectory_smoother.cpp 第51行风格）
    std::vector<trajectory_optimizer::TrajectoryPoint> trajectory_points;
    optimizer->GetStateSequence(trajectory_points);
    
    // 使用结果
    // ...
}
```

### 方式2: 使用工厂创建（推荐）

```cpp
#include "optimizer_factory.hpp"
#include "algorithm_config.hpp"

// 创建配置
cilqr::AlgorithmConfig config;
// ... 设置参数

// 使用工厂创建（可以轻松切换算法类型）
auto optimizer = trajectory_optimizer::OptimizerFactory::Create(
    "cilqr",  // 算法类型
    "trajectory smoother cilqr",  // 名称
    config
);

if (!optimizer) {
    // 处理创建失败
    return;
}

// 使用方式与方式1相同
auto result = optimizer->Execute(...);
```

## 在适配器层中使用

参考 `animation/src/cilqr_adapter.cpp` 的实现：

```cpp
CILQRAdapter::CILQRAdapter(const GlobalConfig* const config) {
    algo_config_ = create_algorithm_config(config);
    // 使用工厂创建优化器
    optimizer_ = trajectory_optimizer::OptimizerFactory::Create(
        "cilqr", "trajectory smoother cilqr", algo_config_);
    if (!optimizer_) {
        throw std::runtime_error("Failed to create CILQR optimizer");
    }
}

std::tuple<Eigen::MatrixX2d, MatrixX5d> CILQRAdapter::solve(...) {
    // 转换类型
    cilqr::ReferenceLine algo_ref = convert_reference_line(ref_waypoints);
    std::vector<cilqr::RoutingLine> algo_obs = convert_routing_lines(obs_preds);
    
    // 使用优化器基类接口执行优化
    auto result = optimizer_->Execute(x0, algo_ref, ref_velo, algo_obs, road_borders);
    
    if (!result.success) {
        return {Eigen::MatrixX2d(), MatrixX5d()};
    }
    
    return {result.control_sequence, result.state_sequence};
}
```

## 扩展新算法

要添加新的优化算法，只需继承 `TrajectoryOptimizer` 并实现接口：

```cpp
#include "trajectory_optimizer.hpp"

class MPCOptimizer : public TrajectoryOptimizer {
public:
    explicit MPCOptimizer(const std::string& name, const MPCConfig& config)
        : TrajectoryOptimizer(name), config_(config) {}
    
    bool Init() override {
        // MPC 初始化逻辑
        return true;
    }
    
    OptimizeResult Execute(
        const Vector5d& initial_state,
        const cilqr::ReferenceLine& reference_line,
        double reference_velocity,
        const std::vector<cilqr::RoutingLine>& obstacles,
        const Eigen::Vector2d& road_borders) override {
        
        OptimizeResult result;
        // MPC 优化逻辑
        // ...
        return result;
    }
    
    void GetStateSequence(std::vector<TrajectoryPoint>& trajectory_points) const override {
        // 转换结果
        // ...
    }
    
    std::string GetType() const override {
        return "MPC";
    }
    
private:
    MPCConfig config_;
    // MPC 特定的成员变量
};
```

然后在工厂类中注册：

```cpp
// optimizer_factory.hpp
static std::unique_ptr<TrajectoryOptimizer> Create(...) {
    if (optimizer_type == "cilqr") {
        // ...
    } else if (optimizer_type == "mpc") {
        auto optimizer = std::make_unique<MPCOptimizer>(name, mpc_config);
        if (optimizer->Init()) {
            return optimizer;
        }
    }
    return nullptr;
}
```

## 优势

1. **统一接口**: 所有优化器使用相同的 `Execute()` 接口
2. **易于扩展**: 新增算法只需继承基类并实现接口
3. **解耦设计**: 主程序不依赖具体算法实现
4. **风格一致**: 参考 `trajectory_smoother.cpp` 的设计风格
5. **易于测试**: 可以轻松mock优化器进行单元测试

## 接口说明

### TrajectoryOptimizer 基类

- `Init()`: 初始化优化器
- `Execute(...)`: 执行优化，返回 `OptimizeResult`
- `GetStateSequence(...)`: 获取状态序列（类似 `get_xystate_seqs`）
- `GetName()`: 获取优化器名称
- `GetType()`: 获取优化器类型
- `Reset()`: 重置状态

### OptimizeResult 结构

```cpp
struct OptimizeResult {
    Eigen::MatrixX2d control_sequence;  // 控制序列
    MatrixX5d state_sequence;           // 状态序列
    bool success;                      // 是否成功
    std::string status_message;        // 状态信息
    int iterations;                    // 迭代次数
    double final_cost;                 // 最终代价
};
```
