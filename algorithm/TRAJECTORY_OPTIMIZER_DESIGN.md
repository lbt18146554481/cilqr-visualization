# 轨迹优化器基类设计文档

## 目录

1. [概述](#概述)
2. [设计目标](#设计目标)
3. [架构设计](#架构设计)
4. [类详细说明](#类详细说明)
5. [API 文档](#api-文档)
6. [使用指南](#使用指南)
7. [扩展新算法](#扩展新算法)
8. [最佳实践](#最佳实践)
9. [常见问题](#常见问题)
10. [参考示例](#参考示例)

---

## 概述

本项目实现了一个统一的轨迹优化器基类接口系统，参考了 `trajectory_smoother.cpp` 的设计风格。所有轨迹优化算法（如 CILQR、MPC 等）都继承自 `TrajectoryOptimizer` 基类，提供统一的调用接口，便于代码维护和算法扩展。

### 核心特性

- ✅ **统一接口**：所有优化器使用相同的 `Execute()` 接口
- ✅ **易于扩展**：新增算法只需继承基类并实现接口
- ✅ **解耦设计**：主程序不依赖具体算法实现
- ✅ **风格一致**：参考 `trajectory_smoother.cpp` 的设计风格
- ✅ **类型安全**：使用基类指针，支持多态
- ✅ **向后兼容**：不改变原有 `CILQRSolver` 的实现

---

## 设计目标

### 1. 统一接口

所有轨迹优化算法都应该提供相同的调用接口，使得主程序可以无缝切换不同的优化算法。

**设计前的问题**：
```cpp
// 不同算法有不同的接口
cilqr_solver.solve(...);      // CILQR
mpc_optimizer.optimize(...);  // MPC
apollo_planner.plan(...);      // Apollo
```

**设计后的解决方案**：
```cpp
// 所有算法使用统一接口
optimizer->Execute(...);  // 统一接口
```

### 2. 易于扩展

当需要添加新的优化算法时，只需：
1. 继承 `TrajectoryOptimizer` 基类
2. 实现必要的虚函数
3. 在工厂类中注册

无需修改现有代码。

### 3. 代码复用

通过基类提供通用的功能（如输入验证、结果存储等），减少重复代码。

### 4. 参考现有设计

参考 `trajectory_smoother.cpp` 中的设计模式：
- 使用 `Execute()` 方法执行优化
- 使用 `GetStateSequence()` 方法获取结果
- 使用组合模式封装算法实现

---

## 架构设计

### 类继承关系

```
┌─────────────────────────────────────────────┐
│   TrajectoryOptimizer (抽象基类)            │
│   - 定义统一接口                             │
│   - Execute() 执行优化                      │
│   - GetStateSequence() 获取结果             │
│   - Init() 初始化                           │
└─────────────────────────────────────────────┘
                    ▲
                    │ 继承
        ┌───────────┴───────────┐
        │                       │
┌───────────────────┐  ┌───────────────────┐
│  CILQROptimizer   │  │  MPCOptimizer      │
│  (已实现)         │  │  (未来扩展)        │
│  - CILQR算法      │  │  - MPC算法         │
└───────────────────┘  └───────────────────┘
        │                       │
        └───────────┬───────────┘
                    │ 使用
        ┌───────────┴───────────┐
        │                       │
┌───────────────────┐  ┌───────────────────┐
│ OptimizerFactory  │  │  CILQRAdapter      │
│ (工厂类)          │  │  (适配器层)        │
│ - 创建优化器      │  │  - 类型转换        │
└───────────────────┘  └───────────────────┘
```

### 数据流

```
主程序/适配器层
    │
    │ 1. 创建优化器
    ▼
OptimizerFactory::Create()
    │
    │ 2. 返回基类指针
    ▼
TrajectoryOptimizer* optimizer
    │
    │ 3. 调用 Execute()
    ▼
optimizer->Execute(input)
    │
    │ 4. 多态调用具体实现
    ▼
CILQROptimizer::Execute()
    │
    │ 5. 调用底层求解器
    ▼
CILQRSolver::solve()
    │
    │ 6. 返回结果
    ▼
OptimizeResult
    │
    │ 7. 获取状态序列
    ▼
optimizer->GetStateSequence()
```

---

## 类详细说明

### 1. TrajectoryOptimizer（基类）

**文件位置**：`algorithm/include/trajectory_optimizer.hpp`

**命名空间**：`trajectory_optimizer`

#### 类定义

```cpp
class TrajectoryOptimizer {
public:
    // 构造函数
    explicit TrajectoryOptimizer(const std::string& name);
    
    // 虚析构函数
    virtual ~TrajectoryOptimizer() = default;
    
    // 纯虚函数：初始化优化器
    virtual bool Init() = 0;
    
    // 纯虚函数：执行优化（核心接口）
    virtual OptimizeResult Execute(
        const Vector5d& initial_state,
        const cilqr::ReferenceLine& reference_line,
        double reference_velocity,
        const std::vector<cilqr::RoutingLine>& obstacles,
        const Eigen::Vector2d& road_borders
    ) = 0;
    
    // 纯虚函数：获取状态序列
    virtual void GetStateSequence(
        std::vector<TrajectoryPoint>& trajectory_points
    ) const = 0;
    
    // 纯虚函数：获取优化器类型
    virtual std::string GetType() const = 0;
    
    // 虚函数：重置状态（可选实现）
    virtual void Reset();
    
    // 虚函数：验证输入（提供默认实现）
    virtual bool ValidateInput(...) const;
    
    // 获取优化器名称
    const std::string& GetName() const;
    
protected:
    std::string name_;              // 优化器名称
    OptimizeResult last_result_;    // 最后一次优化结果
    
private:
    // 禁止拷贝构造和赋值
    TrajectoryOptimizer(const TrajectoryOptimizer&) = delete;
    TrajectoryOptimizer& operator=(const TrajectoryOptimizer&) = delete;
};
```

#### 成员变量说明

| 变量名 | 类型 | 说明 |
|--------|------|------|
| `name_` | `std::string` | 优化器名称，用于日志和调试 |
| `last_result_` | `OptimizeResult` | 最后一次优化的结果，供 `GetStateSequence()` 使用 |

#### 方法说明

##### `Init()`

初始化优化器，在创建优化器后必须调用。

**返回值**：
- `true`：初始化成功
- `false`：初始化失败

**示例**：
```cpp
auto optimizer = std::make_unique<CILQROptimizer>(name, config);
if (!optimizer->Init()) {
    // 处理初始化失败
}
```

##### `Execute()`

执行轨迹优化，这是核心接口。

**参数说明**：

| 参数 | 类型 | 说明 |
|------|------|------|
| `initial_state` | `Vector5d` | 初始状态 `[x, y, v, heading, kappa]` |
| `reference_line` | `cilqr::ReferenceLine` | 参考路径，包含 x, y, yaw, longitude |
| `reference_velocity` | `double` | 参考速度（m/s） |
| `obstacles` | `std::vector<cilqr::RoutingLine>` | 障碍物预测轨迹列表 |
| `road_borders` | `Eigen::Vector2d` | 道路边界 `[upper, lower]` |

**返回值**：`OptimizeResult` 结构，包含：
- `control_sequence`：控制序列 `[acceleration, dkappa]`
- `state_sequence`：状态序列 `[x, y, v, heading, kappa]`
- `success`：是否成功
- `status_message`：状态信息
- `iterations`：迭代次数
- `final_cost`：最终代价

**示例**：
```cpp
auto result = optimizer->Execute(
    initial_state,
    reference_line,
    10.0,  // reference_velocity
    obstacles,
    road_borders
);

if (result.success) {
    // 使用结果
    auto& u = result.control_sequence;
    auto& x = result.state_sequence;
}
```

##### `GetStateSequence()`

获取优化后的状态序列，转换为 `TrajectoryPoint` 向量。

**参数**：
- `trajectory_points`：输出参数，用于存储轨迹点

**示例**：
```cpp
std::vector<TrajectoryPoint> trajectory_points;
optimizer->GetStateSequence(trajectory_points);

for (const auto& pt : trajectory_points) {
    std::cout << "x=" << pt.x << ", y=" << pt.y << std::endl;
}
```

##### `GetType()`

获取优化器类型字符串（如 "CILQR", "MPC"）。

##### `Reset()`

重置优化器状态，清除上次优化的结果。

##### `ValidateInput()`

验证输入参数是否有效，提供默认实现，派生类可以重写。

### 2. OptimizeResult（结果结构）

**定义位置**：`trajectory_optimizer.hpp`

```cpp
struct OptimizeResult {
    Eigen::MatrixX2d control_sequence;  // 控制序列 [acceleration, dkappa]
    MatrixX5d state_sequence;          // 状态序列 [x, y, v, heading, kappa]
    bool success = false;              // 是否成功
    std::string status_message;        // 状态信息
    int iterations = 0;                 // 迭代次数
    double final_cost = 0.0;           // 最终代价
};
```

**字段说明**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `control_sequence` | `Eigen::MatrixX2d` | N×2 矩阵，每行是 `[acceleration, dkappa]` |
| `state_sequence` | `MatrixX5d` | N×5 矩阵，每行是 `[x, y, v, heading, kappa]` |
| `success` | `bool` | 优化是否成功 |
| `status_message` | `std::string` | 状态信息，失败时包含错误信息 |
| `iterations` | `int` | 实际迭代次数 |
| `final_cost` | `double` | 最终代价值 |

### 3. TrajectoryPoint（轨迹点结构）

**定义位置**：`trajectory_optimizer.hpp`

```cpp
struct TrajectoryPoint {
    double x = 0.0;           // X 坐标
    double y = 0.0;           // Y 坐标
    double v = 0.0;           // 速度
    double heading = 0.0;     // 航向角
    double kappa = 0.0;       // 曲率
    double acceleration = 0.0; // 加速度
    double dkappa = 0.0;      // 曲率变化率
    double s = 0.0;           // 累积弧长
};
```

### 4. CILQROptimizer（CILQR 实现）

**文件位置**：`algorithm/include/cilqr_optimizer.hpp`

**命名空间**：`trajectory_optimizer`

#### 类定义

```cpp
class CILQROptimizer : public TrajectoryOptimizer {
public:
    explicit CILQROptimizer(
        const std::string& name,
        const cilqr::AlgorithmConfig& config
    );
    
    ~CILQROptimizer() override = default;
    
    bool Init() override;
    OptimizeResult Execute(...) override;
    void GetStateSequence(...) const override;
    std::string GetType() const override;
    void Reset() override;
    
    // CILQR 特有接口
    cilqr::LQRSolveStatus GetSolveStatus() const;
    
private:
    cilqr::AlgorithmConfig config_;
    std::unique_ptr<cilqr::CILQRSolver> solver_;
};
```

#### 实现细节

**Init() 实现**：
```cpp
bool CILQROptimizer::Init() {
    try {
        solver_ = std::make_unique<cilqr::CILQRSolver>(config_);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}
```

**Execute() 实现流程**：
1. 验证输入参数
2. 调用底层 `CILQRSolver::solve()`
3. 转换结果格式
4. 保存结果到 `last_result_`
5. 返回 `OptimizeResult`

**GetStateSequence() 实现流程**：
1. 检查 `last_result_` 是否成功
2. 从 `state_sequence` 和 `control_sequence` 提取数据
3. 转换为 `TrajectoryPoint` 向量
4. 计算累积弧长 `s`

### 5. OptimizerFactory（工厂类）

**文件位置**：`algorithm/include/optimizer_factory.hpp`

**命名空间**：`trajectory_optimizer`

#### 类定义

```cpp
class OptimizerFactory {
public:
    // 创建优化器（指定名称）
    static std::unique_ptr<TrajectoryOptimizer> Create(
        const std::string& optimizer_type,
        const std::string& name,
        const cilqr::AlgorithmConfig& config
    );
    
    // 创建优化器（使用默认名称）
    static std::unique_ptr<TrajectoryOptimizer> Create(
        const std::string& optimizer_type,
        const cilqr::AlgorithmConfig& config
    );
};
```

#### 支持的优化器类型

| 类型字符串 | 优化器类 | 说明 |
|-----------|---------|------|
| `"cilqr"` 或 `"CILQR"` | `CILQROptimizer` | CILQR 算法（已实现） |
| `"mpc"` 或 `"MPC"` | `MPCOptimizer` | MPC 算法（未来扩展） |

#### 使用示例

```cpp
// 方式1：指定名称
auto optimizer = OptimizerFactory::Create(
    "cilqr",
    "trajectory smoother cilqr",
    config
);

// 方式2：使用默认名称
auto optimizer = OptimizerFactory::Create("cilqr", config);
// 默认名称将是 "cilqr optimizer"
```

---

## API 文档

### 完整 API 参考

#### TrajectoryOptimizer

```cpp
namespace trajectory_optimizer {

class TrajectoryOptimizer {
public:
    // 构造函数
    explicit TrajectoryOptimizer(const std::string& name);
    
    // 析构函数
    virtual ~TrajectoryOptimizer() = default;
    
    // 初始化优化器
    // 返回值：true 表示成功，false 表示失败
    virtual bool Init() = 0;
    
    // 执行优化
    // 参数：
    //   - initial_state: 初始状态 [x, y, v, heading, kappa]
    //   - reference_line: 参考路径
    //   - reference_velocity: 参考速度 (m/s)
    //   - obstacles: 障碍物预测轨迹列表
    //   - road_borders: 道路边界 [upper, lower]
    // 返回值：OptimizeResult 结构
    virtual OptimizeResult Execute(
        const Vector5d& initial_state,
        const cilqr::ReferenceLine& reference_line,
        double reference_velocity,
        const std::vector<cilqr::RoutingLine>& obstacles,
        const Eigen::Vector2d& road_borders
    ) = 0;
    
    // 获取状态序列
    // 参数：
    //   - trajectory_points: 输出参数，存储轨迹点
    virtual void GetStateSequence(
        std::vector<TrajectoryPoint>& trajectory_points
    ) const = 0;
    
    // 获取优化器类型
    // 返回值：类型字符串，如 "CILQR", "MPC"
    virtual std::string GetType() const = 0;
    
    // 重置状态
    virtual void Reset();
    
    // 验证输入
    // 返回值：true 表示输入有效，false 表示无效
    virtual bool ValidateInput(
        const Vector5d& initial_state,
        const cilqr::ReferenceLine& reference_line,
        double reference_velocity,
        const std::vector<cilqr::RoutingLine>& obstacles,
        const Eigen::Vector2d& road_borders
    ) const;
    
    // 获取优化器名称
    const std::string& GetName() const;
};

} // namespace trajectory_optimizer
```

---

## 使用指南

### 快速开始

#### 1. 包含头文件

```cpp
#include "trajectory_optimizer.hpp"
#include "optimizer_factory.hpp"
#include "algorithm_config.hpp"
```

#### 2. 创建配置

```cpp
cilqr::AlgorithmConfig config;
config.delta_t = 0.1;           // 时间步长
config.N = 20;                  // 预测时域长度
config.w_pos = 10.0;            // 位置权重
config.w_vel = 1.0;             // 速度权重
config.w_heading = 20.0;        // 航向权重
config.w_kappa = 10.0;          // 曲率权重
config.w_acc = 0.5;             // 加速度权重
config.w_dkappa = 25.0;         // 曲率变化率权重
config.solve_type = "alm";      // 求解类型：alm 或 barrier
config.max_iter = 15;           // 最大迭代次数
// ... 设置其他参数
```

#### 3. 创建优化器

```cpp
// 使用工厂创建（推荐）
auto optimizer = trajectory_optimizer::OptimizerFactory::Create(
    "cilqr",
    "my optimizer",
    config
);

if (!optimizer) {
    std::cerr << "Failed to create optimizer" << std::endl;
    return -1;
}
```

#### 4. 准备输入数据

```cpp
// 初始状态 [x, y, v, heading, kappa]
Vector5d initial_state;
initial_state << 0.0, 0.0, 10.0, 0.0, 0.0;

// 参考路径
cilqr::ReferenceLine reference_line;
reference_line.x = {0, 10, 20, 30, 40};
reference_line.y = {0, 0, 0, 0, 0};
reference_line.yaw = {0, 0, 0, 0, 0};
reference_line.longitude = {0, 10, 20, 30, 40};

// 参考速度
double reference_velocity = 10.0;

// 障碍物（如果没有，传入空向量）
std::vector<cilqr::RoutingLine> obstacles;

// 道路边界 [upper, lower]
Eigen::Vector2d road_borders(3.6, -3.6);
```

#### 5. 执行优化

```cpp
auto result = optimizer->Execute(
    initial_state,
    reference_line,
    reference_velocity,
    obstacles,
    road_borders
);
```

#### 6. 检查结果

```cpp
if (result.success) {
    std::cout << "Optimization succeeded!" << std::endl;
    std::cout << "Iterations: " << result.iterations << std::endl;
    std::cout << "Final cost: " << result.final_cost << std::endl;
    
    // 使用控制序列和状态序列
    const auto& u = result.control_sequence;  // N×2 矩阵
    const auto& x = result.state_sequence;      // N×5 矩阵
    
    // 访问第 i 个时间步的数据
    int i = 0;
    double acc = u(i, 0);        // 加速度
    double dkappa = u(i, 1);     // 曲率变化率
    double x_pos = x(i, 0);      // X 坐标
    double y_pos = x(i, 1);      // Y 坐标
    double v = x(i, 2);          // 速度
    double heading = x(i, 3);    // 航向角
    double kappa = x(i, 4);      // 曲率
} else {
    std::cerr << "Optimization failed: " << result.status_message << std::endl;
}
```

#### 7. 获取状态序列

```cpp
std::vector<trajectory_optimizer::TrajectoryPoint> trajectory_points;
optimizer->GetStateSequence(trajectory_points);

for (size_t i = 0; i < trajectory_points.size(); ++i) {
    const auto& pt = trajectory_points[i];
    std::cout << "Point " << i << ": "
              << "x=" << pt.x
              << ", y=" << pt.y
              << ", v=" << pt.v
              << ", heading=" << pt.heading
              << ", kappa=" << pt.kappa
              << ", s=" << pt.s << std::endl;
}
```

### 完整示例

```cpp
#include "trajectory_optimizer.hpp"
#include "optimizer_factory.hpp"
#include "algorithm_config.hpp"
#include <iostream>

int main() {
    // 1. 创建配置
    cilqr::AlgorithmConfig config;
    config.delta_t = 0.1;
    config.N = 20;
    config.w_pos = 10.0;
    config.w_vel = 1.0;
    config.w_heading = 20.0;
    config.w_kappa = 10.0;
    config.w_acc = 0.5;
    config.w_dkappa = 25.0;
    config.solve_type = "alm";
    config.max_iter = 15;
    config.width = 2.0;
    config.length = 4.5;
    config.velo_max = 10.0;
    config.velo_min = 0.0;
    config.acc_max = 3.0;
    config.acc_min = -3.0;
    config.kappa_max = 0.3;
    config.kappa_min = -0.3;
    config.dkappa_max = 0.1;
    config.dkappa_min = -0.1;
    config.d_safe = 0.8;
    
    // 2. 创建优化器
    auto optimizer = trajectory_optimizer::OptimizerFactory::Create(
        "cilqr",
        "test optimizer",
        config
    );
    
    if (!optimizer) {
        std::cerr << "Failed to create optimizer" << std::endl;
        return -1;
    }
    
    // 3. 准备输入
    Vector5d initial_state;
    initial_state << 0.0, 0.0, 10.0, 0.0, 0.0;
    
    cilqr::ReferenceLine reference_line;
    for (int i = 0; i < 50; ++i) {
        reference_line.x.push_back(i * 1.0);
        reference_line.y.push_back(0.0);
        reference_line.yaw.push_back(0.0);
        reference_line.longitude.push_back(i * 1.0);
    }
    
    double reference_velocity = 10.0;
    std::vector<cilqr::RoutingLine> obstacles;
    Eigen::Vector2d road_borders(3.6, -3.6);
    
    // 4. 执行优化
    std::cout << "Executing optimization..." << std::endl;
    auto result = optimizer->Execute(
        initial_state,
        reference_line,
        reference_velocity,
        obstacles,
        road_borders
    );
    
    // 5. 检查结果
    if (result.success) {
        std::cout << "Optimization succeeded!" << std::endl;
        std::cout << "Iterations: " << result.iterations << std::endl;
        std::cout << "State sequence size: " << result.state_sequence.rows() << std::endl;
        std::cout << "Control sequence size: " << result.control_sequence.rows() << std::endl;
        
        // 6. 获取状态序列
        std::vector<trajectory_optimizer::TrajectoryPoint> trajectory_points;
        optimizer->GetStateSequence(trajectory_points);
        
        std::cout << "\nTrajectory points:" << std::endl;
        for (size_t i = 0; i < trajectory_points.size(); ++i) {
            const auto& pt = trajectory_points[i];
            if (i % 5 == 0) {  // 每5个点打印一次
                std::cout << "Point " << i << ": "
                          << "x=" << pt.x
                          << ", y=" << pt.y
                          << ", v=" << pt.v
                          << ", heading=" << pt.heading
                          << ", kappa=" << pt.kappa << std::endl;
            }
        }
    } else {
        std::cerr << "Optimization failed: " << result.status_message << std::endl;
        return -1;
    }
    
    return 0;
}
```

---

## 扩展新算法

### 步骤1：创建优化器类

创建新的头文件，例如 `mpc_optimizer.hpp`：

```cpp
#pragma once
#ifndef __MPC_OPTIMIZER_HPP
#define __MPC_OPTIMIZER_HPP

#include "trajectory_optimizer.hpp"
#include "mpc_config.hpp"  // 你的 MPC 配置结构
#include <memory>

namespace trajectory_optimizer {

class MPCOptimizer : public TrajectoryOptimizer {
public:
    explicit MPCOptimizer(const std::string& name, const MPCConfig& config)
        : TrajectoryOptimizer(name), config_(config) {}
    
    ~MPCOptimizer() override = default;
    
    // 实现基类接口
    bool Init() override {
        // MPC 初始化逻辑
        // 例如：创建求解器、设置参数等
        try {
            mpc_solver_ = std::make_unique<MPCSolver>(config_);
            return true;
        } catch (const std::exception& e) {
            return false;
        }
    }
    
    OptimizeResult Execute(
        const Vector5d& initial_state,
        const cilqr::ReferenceLine& reference_line,
        double reference_velocity,
        const std::vector<cilqr::RoutingLine>& obstacles,
        const Eigen::Vector2d& road_borders) override {
        
        OptimizeResult result;
        result.success = false;
        
        // 验证输入
        if (!ValidateInput(initial_state, reference_line, reference_velocity, 
                          obstacles, road_borders)) {
            result.status_message = "Invalid input parameters";
            return result;
        }
        
        try {
            // 调用 MPC 求解器
            auto [u, x] = mpc_solver_->solve(
                initial_state,
                reference_line,
                reference_velocity,
                obstacles,
                road_borders
            );
            
            // 转换结果
            result.control_sequence = u;
            result.state_sequence = x;
            result.success = true;
            result.status_message = "MPC optimization completed";
            result.iterations = mpc_solver_->get_iterations();
            result.final_cost = mpc_solver_->get_cost();
            
            // 保存结果
            last_result_ = result;
            
        } catch (const std::exception& e) {
            result.status_message = std::string("MPC failed: ") + e.what();
        }
        
        return result;
    }
    
    void GetStateSequence(std::vector<TrajectoryPoint>& trajectory_points) const override {
        trajectory_points.clear();
        
        if (!last_result_.success) {
            return;
        }
        
        const auto& x = last_result_.state_sequence;
        const auto& u = last_result_.control_sequence;
        
        double s_accum = 0.0;
        for (int i = 0; i < x.rows(); ++i) {
            TrajectoryPoint pt;
            pt.x = x(i, 0);
            pt.y = x(i, 1);
            pt.v = x(i, 2);
            pt.heading = x(i, 3);
            pt.kappa = x(i, 4);
            
            if (i < u.rows()) {
                pt.acceleration = u(i, 0);
                pt.dkappa = u(i, 1);
            }
            
            if (i > 0) {
                double dx = pt.x - trajectory_points.back().x;
                double dy = pt.y - trajectory_points.back().y;
                s_accum += std::sqrt(dx * dx + dy * dy);
            }
            pt.s = s_accum;
            
            trajectory_points.push_back(pt);
        }
    }
    
    std::string GetType() const override {
        return "MPC";
    }
    
    void Reset() override {
        TrajectoryOptimizer::Reset();
        last_result_ = OptimizeResult();
        // MPC 特定的重置逻辑
    }
    
private:
    MPCConfig config_;
    std::unique_ptr<MPCSolver> mpc_solver_;
};

} // namespace trajectory_optimizer

#endif
```

### 步骤2：在工厂类中注册

修改 `optimizer_factory.hpp`：

```cpp
#include "trajectory_optimizer.hpp"
#include "cilqr_optimizer.hpp"
#include "mpc_optimizer.hpp"  // 添加新头文件
#include "algorithm_config.hpp"
#include "mpc_config.hpp"     // 添加 MPC 配置头文件

namespace trajectory_optimizer {

class OptimizerFactory {
public:
    static std::unique_ptr<TrajectoryOptimizer> Create(
        const std::string& optimizer_type,
        const std::string& name,
        const cilqr::AlgorithmConfig& config) {
        
        if (optimizer_type == "cilqr" || optimizer_type == "CILQR") {
            auto optimizer = std::make_unique<CILQROptimizer>(name, config);
            if (optimizer->Init()) {
                return optimizer;
            }
        } else if (optimizer_type == "mpc" || optimizer_type == "MPC") {
            // 需要将 cilqr::AlgorithmConfig 转换为 MPCConfig
            MPCConfig mpc_config = convert_to_mpc_config(config);
            auto optimizer = std::make_unique<MPCOptimizer>(name, mpc_config);
            if (optimizer->Init()) {
                return optimizer;
            }
        }
        
        return nullptr;
    }
    
    // 可以添加重载版本，直接接受 MPCConfig
    static std::unique_ptr<TrajectoryOptimizer> Create(
        const std::string& optimizer_type,
        const std::string& name,
        const MPCConfig& config) {
        
        if (optimizer_type == "mpc" || optimizer_type == "MPC") {
            auto optimizer = std::make_unique<MPCOptimizer>(name, config);
            if (optimizer->Init()) {
                return optimizer;
            }
        }
        
        return nullptr;
    }
};

} // namespace trajectory_optimizer
```

### 步骤3：使用新算法

```cpp
// 创建 MPC 优化器
auto optimizer = OptimizerFactory::Create("mpc", "my mpc optimizer", mpc_config);

// 使用方式完全相同
auto result = optimizer->Execute(...);
```

---

## 最佳实践

### 1. 错误处理

始终检查优化器创建和优化结果：

```cpp
// 检查创建
auto optimizer = OptimizerFactory::Create("cilqr", name, config);
if (!optimizer) {
    // 处理创建失败
    return;
}

// 检查优化结果
auto result = optimizer->Execute(...);
if (!result.success) {
    // 处理优化失败
    std::cerr << "Error: " << result.status_message << std::endl;
    return;
}
```

### 2. 资源管理

使用智能指针管理优化器生命周期：

```cpp
// 推荐：使用 unique_ptr
auto optimizer = OptimizerFactory::Create("cilqr", name, config);

// 如果需要共享，使用 shared_ptr
std::shared_ptr<TrajectoryOptimizer> shared_optimizer(
    OptimizerFactory::Create("cilqr", name, config).release()
);
```

### 3. 配置管理

将配置集中管理，便于修改和测试：

```cpp
class ConfigManager {
public:
    static cilqr::AlgorithmConfig GetDefaultConfig() {
        cilqr::AlgorithmConfig config;
        config.delta_t = 0.1;
        config.N = 20;
        // ... 设置默认值
        return config;
    }
    
    static cilqr::AlgorithmConfig LoadFromFile(const std::string& path) {
        // 从文件加载配置
    }
};

// 使用
auto config = ConfigManager::GetDefaultConfig();
auto optimizer = OptimizerFactory::Create("cilqr", name, config);
```

### 4. 日志记录

在关键步骤添加日志：

```cpp
#include <spdlog/spdlog.h>

auto optimizer = OptimizerFactory::Create("cilqr", name, config);
SPDLOG_INFO("Created optimizer: {}", optimizer->GetName());

auto result = optimizer->Execute(...);
if (result.success) {
    SPDLOG_INFO("Optimization succeeded in {} iterations", result.iterations);
} else {
    SPDLOG_ERROR("Optimization failed: {}", result.status_message);
}
```

### 5. 性能优化

对于频繁调用的场景，复用优化器实例：

```cpp
// 创建一次，多次使用
auto optimizer = OptimizerFactory::Create("cilqr", name, config);

for (int i = 0; i < 100; ++i) {
    // 每次循环只调用 Execute，不重新创建优化器
    auto result = optimizer->Execute(...);
    // ...
}
```

### 6. 多态使用

利用多态特性，统一处理不同类型的优化器：

```cpp
std::vector<std::unique_ptr<TrajectoryOptimizer>> optimizers;

// 添加不同类型的优化器
optimizers.push_back(OptimizerFactory::Create("cilqr", "cilqr1", config1));
optimizers.push_back(OptimizerFactory::Create("mpc", "mpc1", mpc_config));

// 统一调用
for (auto& optimizer : optimizers) {
    auto result = optimizer->Execute(...);
    std::cout << optimizer->GetType() << " result: " 
              << (result.success ? "success" : "failed") << std::endl;
}
```

---

## 常见问题

### Q1: 如何切换不同的优化算法？

**A**: 只需修改工厂创建时的类型字符串：

```cpp
// 使用 CILQR
auto optimizer = OptimizerFactory::Create("cilqr", name, config);

// 切换到 MPC
auto optimizer = OptimizerFactory::Create("mpc", name, mpc_config);
```

### Q2: 优化失败时如何处理？

**A**: 检查 `OptimizeResult::success` 字段：

```cpp
auto result = optimizer->Execute(...);
if (!result.success) {
    // 查看错误信息
    std::cerr << "Error: " << result.status_message << std::endl;
    
    // 可以尝试重置优化器
    optimizer->Reset();
    
    // 或者使用备用方案
    // ...
}
```

### Q3: 如何获取优化过程中的详细信息？

**A**: 可以通过 `OptimizeResult` 获取：

```cpp
auto result = optimizer->Execute(...);
std::cout << "Iterations: " << result.iterations << std::endl;
std::cout << "Final cost: " << result.final_cost << std::endl;
std::cout << "Status: " << result.status_message << std::endl;
```

### Q4: 优化器可以多线程使用吗？

**A**: 每个线程应该创建自己的优化器实例：

```cpp
// 每个线程创建独立的优化器
void thread_function() {
    auto optimizer = OptimizerFactory::Create("cilqr", name, config);
    // 使用优化器
}
```

### Q5: 如何自定义输入验证？

**A**: 在派生类中重写 `ValidateInput()` 方法：

```cpp
class MyOptimizer : public TrajectoryOptimizer {
    bool ValidateInput(...) const override {
        // 调用基类验证
        if (!TrajectoryOptimizer::ValidateInput(...)) {
            return false;
        }
        
        // 添加自定义验证
        if (initial_state(2) < 0) {  // 速度不能为负
            return false;
        }
        
        return true;
    }
};
```

### Q6: 如何调试优化问题？

**A**: 使用以下方法：

```cpp
// 1. 启用详细日志
SPDLOG_SET_LEVEL(spdlog::level::debug);

// 2. 检查输入
if (!optimizer->ValidateInput(...)) {
    std::cerr << "Invalid input!" << std::endl;
}

// 3. 检查结果
auto result = optimizer->Execute(...);
if (!result.success) {
    std::cerr << "Error: " << result.status_message << std::endl;
}

// 4. 检查状态序列
std::vector<TrajectoryPoint> points;
optimizer->GetStateSequence(points);
if (points.empty()) {
    std::cerr << "No trajectory points!" << std::endl;
}
```

---

## 参考示例

### 示例1：在适配器中使用（参考 trajectory_smoother.cpp）

```cpp
// animation/src/cilqr_adapter.cpp

CILQRAdapter::CILQRAdapter(const GlobalConfig* const config) {
    // 1. 创建配置
    algo_config_ = create_algorithm_config(config);
    
    // 2. 使用工厂创建优化器（参考 trajectory_smoother.cpp 第47行）
    optimizer_ = OptimizerFactory::Create(
        "cilqr",
        "trajectory smoother cilqr",  // 名称
        algo_config_
    );
    
    if (!optimizer_) {
        SPDLOG_ERROR("Failed to create CILQR optimizer");
        throw std::runtime_error("Failed to create CILQR optimizer");
    }
}

std::tuple<Eigen::MatrixX2d, MatrixX5d> CILQRAdapter::solve(...) {
    // 3. 转换类型
    cilqr::ReferenceLine algo_ref = convert_reference_line(ref_waypoints);
    std::vector<cilqr::RoutingLine> algo_obs = convert_routing_lines(obs_preds);
    
    // 4. 执行优化（参考 trajectory_smoother.cpp 第48行）
    auto result = optimizer_->Execute(x0, algo_ref, ref_velo, algo_obs, road_borders);
    
    // 5. 检查结果
    if (!result.success) {
        SPDLOG_WARN("Optimization failed: {}", result.status_message);
        return {Eigen::MatrixX2d(), MatrixX5d()};
    }
    
    // 6. 返回结果
    return {result.control_sequence, result.state_sequence};
}
```

### 示例2：在主函数中使用

```cpp
// animation/src/motion_planning.cpp

int main(int argc, char** argv) {
    // ... 加载配置 ...
    
    // 创建优化器
    auto optimizer = OptimizerFactory::Create(
        "cilqr",
        "motion planning optimizer",
        algo_config
    );
    
    if (!optimizer) {
        SPDLOG_ERROR("Failed to create optimizer");
        return -1;
    }
    
    // 主循环
    for (double t = 0.; t < max_simulation_time; t += delta_t) {
        // 准备输入
        Vector5d ego_state_5d = ...;
        cilqr::ReferenceLine ref_line = ...;
        double ref_velo = ...;
        std::vector<cilqr::RoutingLine> obstacles = ...;
        Eigen::Vector2d road_borders = ...;
        
        // 执行优化
        auto result = optimizer->Execute(
            ego_state_5d,
            ref_line,
            ref_velo,
            obstacles,
            road_borders
        );
        
        if (result.success) {
            // 使用结果
            Eigen::MatrixX2d new_u = result.control_sequence;
            MatrixX5d new_x_5d = result.state_sequence;
            
            // 可视化或进一步处理
            // ...
        } else {
            SPDLOG_WARN("Optimization failed at t={}: {}", t, result.status_message);
        }
    }
    
    return 0;
}
```

### 示例3：比较不同算法

```cpp
void compare_algorithms() {
    // 准备相同的输入
    Vector5d initial_state = ...;
    cilqr::ReferenceLine reference_line = ...;
    // ...
    
    // 测试 CILQR
    auto cilqr_optimizer = OptimizerFactory::Create("cilqr", "test", cilqr_config);
    auto cilqr_result = cilqr_optimizer->Execute(...);
    
    // 测试 MPC（如果已实现）
    auto mpc_optimizer = OptimizerFactory::Create("mpc", "test", mpc_config);
    auto mpc_result = mpc_optimizer->Execute(...);
    
    // 比较结果
    std::cout << "CILQR: " << (cilqr_result.success ? "success" : "failed")
              << ", cost=" << cilqr_result.final_cost << std::endl;
    std::cout << "MPC: " << (mpc_result.success ? "success" : "failed")
              << ", cost=" << mpc_result.final_cost << std::endl;
}
```

---

## 总结

本设计文档详细介绍了轨迹优化器基类的设计和使用方法。通过统一的接口和工厂模式，使得：

1. **代码更清晰**：统一的接口使得代码更易理解
2. **易于扩展**：添加新算法只需实现基类接口
3. **易于维护**：修改基类接口可以影响所有派生类
4. **易于测试**：可以轻松 mock 优化器进行单元测试

如有任何问题或建议，欢迎提出！

---

**文档版本**：1.0  
**最后更新**：2025-01-XX  
**作者**：puyu
