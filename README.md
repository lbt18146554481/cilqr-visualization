# CILQR Visualization Project

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-3.10+-green.svg)](https://cmake.org/)

本项目是一个完整的 CILQR（约束迭代线性二次调节器）算法可视化系统，包含算法核心库和两个可视化工具，用于展示和测试 CILQR 算法在自动驾驶运动规划中的应用。

## ✨ 特性

- 🎯 **统一的优化器接口**：基于策略模式的优化器基类系统，易于扩展新算法
- 🎬 **动画可视化**：生成轨迹规划的 GIF 动画
- 🌐 **Web 交互界面**：实时调整参数并查看优化结果
- 📊 **多种场景支持**：直道、弯道、借道超车等场景
- 🔧 **可扩展架构**：添加新算法无需修改前端代码

## 🚀 快速开始

## 项目结构

```
cilqr_visualization/
├── 3rdparty/                    # 统一的第三方库目录
│   ├── fmt/                    # 格式化库
│   ├── spdlog/                 # 日志库
│   ├── yaml-cpp/               # YAML解析库
│   ├── jsoncpp/                # JSON解析库
│   └── matplotlibcpp.h         # Matplotlib C++包装
│
├── algorithm/                  # CILQR算法核心库（共享）
│   ├── include/                # 头文件
│   │   ├── types.hpp           # 基础类型定义（Vector5d, ReferenceLine等）
│   │   ├── algorithm_config.hpp # 算法配置接口
│   │   ├── cilqr_solver.hpp   # CILQR求解器接口
│   │   └── path_model_utils.hpp # 路径模型工具函数
│   ├── src/                    # 源文件
│   │   ├── cilqr_solver.cpp   # CILQR求解器实现
│   │   └── path_model_utils.cpp # 路径模型工具函数实现
│   ├── CMakeLists.txt         # 构建配置
│   └── README.md              # 算法库说明文档
│
├── animation/                 # 动画演示项目（生成GIF动画）
│   ├── include/                # 头文件
│   │   ├── cilqr_adapter.hpp  # 适配器层（连接算法和可视化）
│   │   ├── cubic_spline.hpp   # 三次样条插值
│   │   ├── global_config.hpp  # 全局配置管理
│   │   └── utils.hpp          # 工具函数
│   ├── src/                    # 源文件
│   │   ├── cilqr_adapter.cpp  # 适配器实现
│   │   ├── motion_planning.cpp # 主程序入口
│   │   ├── cubic_spline.cpp   # 样条插值实现
│   │   ├── global_config.cpp  # 配置管理实现
│   │   └── utils.cpp          # 可视化工具函数
│   ├── config/                 # 场景配置文件
│   │   ├── scenario_three_bend.yaml      # 三弯道场景
│   │   ├── scenario_three_straight.yaml  # 三直道场景
│   │   ├── scenario_two_straight.yaml    # 两直道场景
│   │   ├── scenario_two_borrow.yaml       # 借道超车场景
│   │   └── scenario_path_model.yaml      # 路径模型场景
│   ├── scripts/                # Python辅助脚本
│   │   └── utils/
│   │       └── imshow.py       # 车辆图像显示工具
│   ├── images/                 # 输出图像和GIF
│   ├── CMakeLists.txt         # 构建配置
│   ├── 1_build.sh             # 编译和运行脚本
│   └── README.md              # 动画项目说明
│
└── single_frame/              # 单帧可视化项目（Web交互界面）
    ├── mock_headers/          # Mock头文件（编译辅助）
    ├── cilqr_adapter_simple.cpp/h          # 适配器（连接算法和可视化）
    ├── planning_test_execute.cpp          # 主程序入口
    ├── planning_config.json               # 配置文件
    ├── web_visualize.py                   # Web服务器和前端
    ├── CMakeLists.txt                    # 构建配置
    ├── 2_build.sh                        # 构建脚本
    ├── start.sh                          # 启动脚本
    └── README.md                         # 单帧项目说明
```


### Linux 安装依赖

#### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    libeigen3-dev \
    libjsoncpp-dev \
    libyaml-cpp-dev \
    python3 \
    python3-pip \
    python3-dev

# 安装Python依赖（用于动画项目）
pip3 install numpy matplotlib
```

#### Fedora/RHEL/CentOS

```bash
sudo dnf install -y \
    gcc-c++ \
    cmake \
    eigen3-devel \
    jsoncpp-devel \
    yaml-cpp-devel \
    python3 \
    python3-pip \
    python3-devel

# 安装Python依赖
pip3 install numpy matplotlib
```

#### Arch Linux

```bash
sudo pacman -S \
    base-devel \
    cmake \
    eigen \
    jsoncpp \
    yaml-cpp \
    python \
    python-pip

# 安装Python依赖
pip install numpy matplotlib
```

### macOS 安装依赖

```bash
brew install eigen jsoncpp cmake python3
pip3 install numpy matplotlib
```

## 动画项目

### 编译

```bash
cd animation
rm -rf build  # 如之前有编译过，先清理
./1_build.sh
```

或者手动编译：

```bash
cd animation
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

### 运行

运行不同的场景配置：

```bash
# 三弯道场景
./build/motion_planning -c ./config/scenario_three_bend.yaml

# 三直道场景
./build/motion_planning -c ./config/scenario_three_straight.yaml

# 两直道场景
./build/motion_planning -c ./config/scenario_two_straight.yaml

# 借道超车场景
./build/motion_planning -c ./config/scenario_two_borrow.yaml

# 路径模型场景
./build/motion_planning -c ./config/scenario_path_model.yaml
```

### 配置文件说明

配置文件位于 `animation/config/` 目录下，采用 YAML 格式，可以修改以下参数：

#### 算法参数
- `w_pos`: 位置权重（x, y）
- `w_heading`: 航向权重
- `w_kappa`: 曲率权重
- `w_vel`: 速度权重
- `w_acc`: 加速度权重
- `w_dkappa`: 曲率变化率权重
- `ref_x_weight`: 参考轨迹权重
- `max_iter`: 最大迭代次数
- `convergence_threshold`: 收敛阈值

#### 初始条件
- `init_state`: 车辆初始状态 `[x, y, v, heading, kappa]`
- `ref_velo`: 参考速度

#### 场景信息
- `obstacles`: 障碍物位置列表
- `road_boundaries`: 道路边界
- `reference_line`: 参考路径点

### 输出

运行后会生成：
- **轨迹规划的可视化图像**：保存在 `animation/images/` 目录
- **动画 GIF 文件**：展示车辆运动轨迹的动画效果

### 技术细节

- **适配器模式**：`CILQRAdapter` 类负责将可视化层的数据结构转换为算法层的数据结构
- **配置管理**：使用单例模式 `GlobalConfig` 管理全局配置
- **可视化**：使用 matplotlib-cpp 进行 C++ 绘图，Python 脚本辅助显示车辆图像

---

## 单帧项目（single_frame/）

### 编译

```bash
cd single_frame
rm -rf build_x86_64_Release  # 如有编译过，先清理
./2_build.sh
```

或者手动编译：

```bash
cd single_frame
mkdir -p build_x86_64_Release
cd build_x86_64_Release
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

### 启动

```bash
cd single_frame
./start.sh
```

或者直接运行 Python Web 服务器：

```bash
cd single_frame
python3 web_visualize.py
```

### 访问

启动成功后，在浏览器中打开：

**http://localhost:8081**

## 🏗️ 架构设计

本项目采用统一的优化器基类系统，支持轻松扩展新算法：

```
前端层 (animation/single_frame)
    ↓
适配器层 (CILQRAdapter)
    ↓
优化器基类系统 (TrajectoryOptimizer)
    ↓
具体算法实现 (CILQROptimizer)
    ↓
算法核心 (CILQRSolver)
```

详细设计文档请参考：
- [统一架构说明](UNIFIED_ARCHITECTURE.md)
- [优化器设计文档](algorithm/TRAJECTORY_OPTIMIZER_DESIGN.md)
- [使用指南](algorithm/OPTIMIZER_USAGE.md)

## 🤝 贡献指南

欢迎贡献！请查看 [CONTRIBUTING.md](CONTRIBUTING.md) 了解详细信息。

## 📄 许可证

本项目采用 [MIT License](LICENSE)。

## 📚 相关文档

- [GitHub 设置指南](GITHUB_SETUP.md) - 如何将项目上传到 GitHub
- [架构总结](ARCHITECTURE_SUMMARY.md) - 项目架构概述
- [编译测试结果](BUILD_TEST_RESULTS.md) - 编译和测试信息

## 🙏 致谢

- 使用了 [fmt](https://github.com/fmtlib/fmt) 格式化库
- 使用了 [spdlog](https://github.com/gabime/spdlog) 日志库
- 使用了 [yaml-cpp](https://github.com/jbeder/yaml-cpp) YAML 解析库
- 使用了 [jsoncpp](https://github.com/open-source-parsers/jsoncpp) JSON 解析库
- 使用了 [Eigen](https://eigen.tuxfamily.org/) 线性代数库
