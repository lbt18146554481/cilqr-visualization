# GitHub 仓库设置指南

## 步骤 1：初始化 Git 仓库

如果项目还没有独立的 git 仓库，执行：

```bash
cd /Users/lubitong/Desktop/algorithm/cilqr_visualization

# 初始化 git 仓库（如果还没有）
git init

# 添加所有文件
git add .

# 创建初始提交
git commit -m "Initial commit: CILQR visualization project"
```

## 步骤 2：在 GitHub 上创建仓库

1. 登录 GitHub
2. 点击右上角的 "+" → "New repository"
3. 填写仓库信息：
   - **Repository name**: `cilqr-visualization`（或你喜欢的名字）
   - **Description**: "CILQR (Constrained Iterative Linear Quadratic Regulator) algorithm visualization system for autonomous driving motion planning"
   - **Visibility**: Public 或 Private（根据你的需求）
   - **不要**勾选 "Initialize this repository with a README"（因为本地已有）
4. 点击 "Create repository"

## 步骤 3：连接本地仓库到 GitHub

GitHub 创建仓库后会显示命令，执行：

```bash
cd /Users/lubitong/Desktop/algorithm/cilqr_visualization

# 添加远程仓库（替换 YOUR_USERNAME 为你的 GitHub 用户名）
git remote add origin https://github.com/YOUR_USERNAME/cilqr-visualization.git

# 或者使用 SSH（如果你配置了 SSH key）
# git remote add origin git@github.com:YOUR_USERNAME/cilqr-visualization.git

# 推送到 GitHub
git branch -M main  # 重命名分支为 main（如果当前不是 main）
git push -u origin main
```

## 步骤 4：验证

访问你的 GitHub 仓库页面，应该能看到所有文件已经上传。

## 后续操作

### 添加更多文件

```bash
git add .
git commit -m "描述你的更改"
git push
```

### 创建新分支

```bash
git checkout -b feature/new-feature
# 进行开发...
git push -u origin feature/new-feature
```

### 添加标签（版本发布）

```bash
git tag -a v1.0.0 -m "First release"
git push origin v1.0.0
```

## 注意事项

### 1. 大文件处理

如果第三方库文件太大，考虑：
- 使用 Git LFS（Large File Storage）
- 或者使用 submodule 引用第三方库
- 或者只上传必要的第三方库文件

### 2. 敏感信息

确保 `.gitignore` 已正确配置，不要提交：
- 编译产物（build/）
- 个人配置
- 敏感密钥

### 3. 第三方库许可证

确保第三方库的许可证兼容：
- `fmt`: MIT License ✅
- `spdlog`: MIT License ✅
- `yaml-cpp`: MIT License ✅
- `jsoncpp`: Public Domain / MIT ✅
- `matplotlibcpp.h`: 需要检查许可证

## 推荐的仓库结构

```
cilqr-visualization/
├── .github/
│   └── workflows/
│       └── ci.yml          # CI/CD 配置
├── .gitignore              # Git 忽略文件
├── LICENSE                 # 许可证
├── README.md              # 项目说明
├── 3rdparty/              # 第三方库
├── algorithm/             # 算法核心
├── animation/             # 动画项目
└── single_frame/         # Web 项目
```

## 可选：添加 GitHub Actions CI/CD

已创建 `.github/workflows/ci.yml`，会自动：
- 在 Ubuntu 和 macOS 上测试编译
- 支持 Release 和 Debug 模式
- 自动安装依赖

只需推送到 GitHub 即可自动运行。
