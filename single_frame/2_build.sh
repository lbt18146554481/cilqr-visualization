#!/bin/bash

# Visual项目构建脚本

set -e  # 遇到错误立即退出

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build_x86_64_Release"
EXECUTABLE_PATH="${BUILD_DIR}/planning_test_execute"

echo "=========================================="
echo "Building Visual Project"
echo "=========================================="

# 创建build目录
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

# 运行CMake配置
echo "Running CMake configuration..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# 编译
echo "Building..."
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

# 检查可执行文件
if [ -f "${EXECUTABLE_PATH}" ]; then
    echo "=========================================="
    echo "Build successful!"
    echo "Executable: ${EXECUTABLE_PATH}"
    echo "=========================================="
    
    # 测试运行
    echo ""
    echo "Testing executable..."
    if [ -f "${SCRIPT_DIR}/planning_config.json" ]; then
        "${EXECUTABLE_PATH}" "${SCRIPT_DIR}/planning_config.json" 2>&1 | head -20
        echo ""
        echo "Executable test completed."
    else
        echo "Warning: planning_config.json not found, skipping test"
    fi
else
    echo "=========================================="
    echo "Build failed! Executable not found."
    echo "=========================================="
    exit 1
fi
