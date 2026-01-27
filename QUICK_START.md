# 快速开始指南

## 1. 初始化 Git 仓库

```bash
cd /Users/lubitong/Desktop/algorithm/cilqr_visualization

# 初始化仓库
git init

# 添加所有文件
git add .

# 创建初始提交
git commit -m "Initial commit: CILQR visualization project"
```

## 2. 在 GitHub 上创建仓库

1. 访问 https://github.com/new
2. 填写仓库信息：
   - Repository name: `cilqr-visualization`
   - Description: "CILQR algorithm visualization system"
   - 选择 Public 或 Private
   - **不要**勾选 "Initialize with README"
3. 点击 "Create repository"

## 3. 连接并推送

```bash
# 添加远程仓库（替换 YOUR_USERNAME）
git remote add origin https://github.com/YOUR_USERNAME/cilqr-visualization.git

# 重命名分支为 main
git branch -M main

# 推送到 GitHub
git push -u origin main
```

## 4. 验证

访问 https://github.com/YOUR_USERNAME/cilqr-visualization 查看你的仓库。

## 使用自动化脚本

也可以直接运行：
```bash
./setup_github.sh
```

然后按照提示操作。
