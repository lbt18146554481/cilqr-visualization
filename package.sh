#!/bin/bash

# CILQR可视化项目 - 打包脚本
# 用于将项目打包成压缩包

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="cilqr_visualization"
VERSION=$(date +"%Y%m%d_%H%M%S")
PACKAGE_NAME="${PROJECT_NAME}_${VERSION}"
OUTPUT_DIR="$SCRIPT_DIR"
ARCHIVE_NAME="${PACKAGE_NAME}.tar.gz"

echo "=========================================="
echo "CILQR可视化项目 - 打包脚本"
echo "=========================================="
echo ""
echo "项目目录: $SCRIPT_DIR"
echo "输出目录: $OUTPUT_DIR"
echo "打包名称: $ARCHIVE_NAME"
echo ""

cd "$SCRIPT_DIR"

# 创建临时目录用于打包
TEMP_DIR=$(mktemp -d)
PACKAGE_DIR="${TEMP_DIR}/${PACKAGE_NAME}"
mkdir -p "$PACKAGE_DIR"

echo "创建临时打包目录: $PACKAGE_DIR"
echo ""

# 复制文件，排除不需要的内容
echo "复制项目文件..."

rsync -av \
    --exclude='.git' \
    --exclude='build' \
    --exclude='build_*' \
    --exclude='build_x86_64_*' \
    --exclude='cmake-build-*' \
    --exclude='*.xcodeproj' \
    --exclude='*.xcworkspace' \
    --exclude='CMakeCache.txt' \
    --exclude='CMakeFiles' \
    --exclude='cmake_install.cmake' \
    --exclude='Makefile' \
    --exclude='*.o' \
    --exclude='*.a' \
    --exclude='*.so' \
    --exclude='*.dylib' \
    --exclude='*.dll' \
    --exclude='*.exe' \
    --exclude='*.out' \
    --exclude='motion_planning' \
    --exclude='planning_test_execute' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='*.pyo' \
    --exclude='*.log' \
    --exclude='*.gif' \
    --exclude='*.png' \
    --exclude='*.jpg' \
    --exclude='*.jpeg' \
    --exclude='images/*.gif' \
    --exclude='images/*.png' \
    --exclude='*.tmp' \
    --exclude='*.bak' \
    --exclude='*.swp' \
    --exclude='.DS_Store' \
    --exclude='Thumbs.db' \
    --exclude='3rdparty/*/build' \
    --exclude='3rdparty/*/CMakeFiles' \
    --exclude='3rdparty/jsoncpp_install' \
    --exclude='3rdparty/库的压缩包' \
    --exclude='*.zip' \
    --exclude='*.tar.gz' \
    --exclude='*.tar' \
    "$SCRIPT_DIR/" "$PACKAGE_DIR/"

echo ""
echo "文件复制完成"
echo ""

# 创建压缩包
echo "创建压缩包: $ARCHIVE_NAME"
cd "$TEMP_DIR"
tar -czf "$OUTPUT_DIR/$ARCHIVE_NAME" "$PACKAGE_NAME"

# 清理临时目录
rm -rf "$TEMP_DIR"

echo ""
echo "=========================================="
echo "打包完成！"
echo "=========================================="
echo ""
echo "压缩包位置: $OUTPUT_DIR/$ARCHIVE_NAME"
echo ""
echo "文件大小:"
ls -lh "$OUTPUT_DIR/$ARCHIVE_NAME" | awk '{print "  " $5 " (" $9 ")"}'
echo ""
