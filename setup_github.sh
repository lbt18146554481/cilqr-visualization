#!/bin/bash

# GitHub 仓库设置脚本
# 使用方法: ./setup_github.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=========================================="
echo "GitHub 仓库设置脚本"
echo "=========================================="
echo ""

# 检查是否已有 git 仓库
if [ -d .git ]; then
    echo "✓ 检测到已有 git 仓库"
    CURRENT_REMOTE=$(git remote get-url origin 2>/dev/null || echo "")
    if [ -n "$CURRENT_REMOTE" ]; then
        echo "  当前远程仓库: $CURRENT_REMOTE"
        read -p "是否要更换远程仓库? (y/n): " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            read -p "请输入新的 GitHub 仓库 URL: " NEW_REMOTE
            git remote set-url origin "$NEW_REMOTE"
            echo "✓ 已更新远程仓库地址"
        fi
    fi
else
    echo "初始化 git 仓库..."
    git init
    echo "✓ Git 仓库初始化完成"
fi

# 检查 .gitignore
if [ ! -f .gitignore ]; then
    echo "⚠️  警告: .gitignore 文件不存在"
else
    echo "✓ .gitignore 文件存在"
fi

# 检查 LICENSE
if [ ! -f LICENSE ]; then
    echo "⚠️  警告: LICENSE 文件不存在"
else
    echo "✓ LICENSE 文件存在"
fi

# 显示当前状态
echo ""
echo "当前 git 状态:"
git status --short | head -20 || echo "  无更改"

echo ""
echo "=========================================="
echo "下一步操作："
echo "=========================================="
echo ""
echo "1. 在 GitHub 上创建新仓库："
echo "   - 访问 https://github.com/new"
echo "   - 填写仓库名称（例如: cilqr-visualization）"
echo "   - 选择 Public 或 Private"
echo "   - 不要勾选 'Initialize with README'"
echo ""
echo "2. 添加文件并提交："
echo "   git add ."
echo "   git commit -m 'Initial commit: CILQR visualization project'"
echo ""
echo "3. 连接远程仓库并推送："
echo "   git remote add origin https://github.com/YOUR_USERNAME/cilqr-visualization.git"
echo "   git branch -M main"
echo "   git push -u origin main"
echo ""
echo "或者使用 SSH（如果已配置）："
echo "   git remote add origin git@github.com:YOUR_USERNAME/cilqr-visualization.git"
echo "   git branch -M main"
echo "   git push -u origin main"
echo ""
echo "详细说明请查看 GITHUB_SETUP.md"
echo ""
