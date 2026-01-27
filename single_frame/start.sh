#!/bin/bash

# Visual项目启动脚本

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build_x86_64_Release"
EXECUTABLE_PATH="${BUILD_DIR}/planning_test_execute"
WEB_SERVER="${SCRIPT_DIR}/web_visualize.py"

echo "=========================================="
echo "Starting Visual Project"
echo "=========================================="

# 检查可执行文件是否存在
if [ ! -f "${EXECUTABLE_PATH}" ]; then
    echo "Error: Executable not found: ${EXECUTABLE_PATH}"
    echo "Please run ./2_build.sh first to build the project."
    exit 1
fi

# 检查配置文件是否存在
if [ ! -f "${SCRIPT_DIR}/planning_config.json" ]; then
    echo "Error: Config file not found: ${SCRIPT_DIR}/planning_config.json"
    exit 1
fi

# 检查Python是否可用
if ! command -v python3 &> /dev/null; then
    echo "Error: python3 not found. Please install Python 3."
    exit 1
fi

# 启动Web服务器


cd "${SCRIPT_DIR}"
python3 "${WEB_SERVER}"
