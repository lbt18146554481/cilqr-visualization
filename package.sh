#!/bin/bash

# CILQR可视化项目 - 通用打包脚本
# 用于将项目打包成 tar.gz 格式

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="cilqr_visualization"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
VERSION=$(date +"%Y%m%d")
PACKAGE_NAME="${PROJECT_NAME}_${VERSION}"
PACKAGE_FILE="${PARENT_DIR}/${PACKAGE_NAME}.tar.gz"

echo "=========================================="
echo "CILQR可视化项目 - 打包脚本"
echo "=========================================="
echo ""
echo "项目目录: $SCRIPT_DIR"
echo "输出文件: $PACKAGE_FILE"
echo ""

cd "$SCRIPT_DIR"

# 检查是否有未提交的更改（可选）
if [ -d ".git" ]; then
    echo "检测到Git仓库，检查状态..."
    if ! git diff --quiet 2>/dev/null || ! git diff --cached --quiet 2>/dev/null; then
        echo "⚠️  警告: 有未提交的更改"
        read -p "是否继续打包? (y/n) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

echo "开始打包..."
echo "排除的文件/目录:"
echo "  - 构建目录 (build*, build_x86_64_Release)"
echo "  - Python缓存 (__pycache__, *.pyc)"
echo "  - Git目录 (.git)"
echo "  - IDE配置 (.vscode, .idea)"
echo "  - 临时文件 (*.log, *.swp, .DS_Store)"
echo "  - 第三方库的测试/示例/文档"
echo ""

# 创建临时目录用于打包
TEMP_DIR=$(mktemp -d)
PACKAGE_DIR="${TEMP_DIR}/${PACKAGE_NAME}"
mkdir -p "$PACKAGE_DIR"

echo "创建临时打包目录: $PACKAGE_DIR"
echo ""

# 复制文件，排除不需要的内容
echo "复制项目文件..."

rsync -av \
    --exclude='build' \
    --exclude='build_*' \
    --exclude='build_x86_64_Release' \
    --exclude='.git' \
    --exclude='.vscode' \
    --exclude='.idea' \
    --exclude='__pycache__' \
    --exclude='**/__pycache__' \
    --exclude='*.pyc' \
    --exclude='**/*.pyc' \
    --exclude='.DS_Store' \
    --exclude='**/.DS_Store' \
    --exclude='*.log' \
    --exclude='*.swp' \
    --exclude='*.swo' \
    --exclude='*~' \
    --exclude='.clang-format' \
    --exclude='3rdparty/**/build' \
    --exclude='3rdparty/**/test' \
    --exclude='3rdparty/**/example' \
    --exclude='3rdparty/**/doc' \
    --exclude='3rdparty/**/bench' \
    --exclude='3rdparty/**/benchmark' \
    --exclude='3rdparty/**/fuzz' \
    --exclude='3rdparty/**/fuzzing' \
    --exclude='3rdparty/**/support' \
    --exclude='3rdparty/**/devtools' \
    --exclude='3rdparty/**/ci' \
    --exclude='3rdparty/**/cmake' \
    --exclude='3rdparty/**/pkg-config' \
    --exclude='3rdparty/**/*.cmake.in' \
    --exclude='3rdparty/**/*.in' \
    --exclude='3rdparty/**/meson.build' \
    --exclude='3rdparty/**/meson_options.txt' \
    --exclude='3rdparty/**/BUILD.bazel' \
    --exclude='3rdparty/**/appveyor.yml' \
    --exclude='3rdparty/**/CONTRIBUTING.md' \
    --exclude='3rdparty/**/ChangeLog.md' \
    --exclude='3rdparty/**/reformat.sh' \
    --exclude='3rdparty/**/get_version.pl' \
    --exclude='3rdparty/**/amalgamate.py' \
    --exclude='3rdparty/**/dev.makefile' \
    --exclude='3rdparty/**/doxybuild.py' \
    --exclude='3rdparty/**/AUTHORS' \
    --exclude='3rdparty/库的压缩包' \
    "$SCRIPT_DIR/" "$PACKAGE_DIR/"

echo ""
echo "文件复制完成"
echo ""

# 计算打包后的大小
PACKAGE_SIZE=$(du -sh "$PACKAGE_DIR" | cut -f1)
echo "打包后大小: $PACKAGE_SIZE"
echo ""

# 创建压缩包
echo "创建压缩包: $PACKAGE_FILE"
cd "$TEMP_DIR"
tar -czf "$PACKAGE_FILE" "$PACKAGE_NAME"

# 计算压缩包大小
COMPRESSED_SIZE=$(du -sh "$PACKAGE_FILE" | cut -f1)
echo ""
echo "压缩完成!"
echo "压缩包大小: $COMPRESSED_SIZE"
echo ""

# 清理临时目录
echo "清理临时文件..."
rm -rf "$TEMP_DIR"

echo ""
echo "=========================================="
echo "打包完成!"
echo "=========================================="
echo ""
echo "压缩包位置: $PACKAGE_FILE"
echo "压缩包大小: $COMPRESSED_SIZE"
echo ""
echo "解压命令:"
echo "  tar -xzf $PACKAGE_FILE"
echo ""
