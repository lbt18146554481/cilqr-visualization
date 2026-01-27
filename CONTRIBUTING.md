# 贡献指南

感谢你对 CILQR Visualization 项目的关注！

## 如何贡献

### 报告问题

如果发现 bug 或有功能建议，请：
1. 在 GitHub Issues 中搜索是否已有相关问题
2. 如果没有，创建新的 Issue，描述：
   - 问题描述
   - 复现步骤
   - 预期行为
   - 实际行为
   - 环境信息（操作系统、编译器版本等）

### 提交代码

1. Fork 本仓库
2. 创建功能分支：`git checkout -b feature/your-feature`
3. 提交更改：`git commit -m "Add: your feature description"`
4. 推送到分支：`git push origin feature/your-feature`
5. 创建 Pull Request

### 代码规范

- 使用 C++17 标准
- 遵循现有的代码风格
- 添加必要的注释
- 确保代码能编译通过

### 添加新算法

参考 `algorithm/TRAJECTORY_OPTIMIZER_DESIGN.md` 文档：
1. 继承 `TrajectoryOptimizer` 基类
2. 实现必要的虚函数
3. 在 `OptimizerFactory` 中注册
4. 添加测试和文档

## 许可证

贡献的代码将使用与项目相同的 MIT 许可证。
