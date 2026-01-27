# 轨迹长度调试指南

## 问题
优化轨迹比启发式轨迹短很多。

## 调试步骤

### 1. 查看终端输出

重新运行项目后，在终端中查找以下调试信息：

```
CILQR Config: N=40, delta_t=0.2, trajectory_length=8.0s
DEBUG: Optimized trajectory: XX points, heuristic trajectory: XX points
DEBUG: Optimized trajectory length: X.Xs
DEBUG: FillPathData: optimized_states_.size()=XX, expected N+1=41
```

### 2. 检查配置读取

确保 `planning_config.json` 中设置了：
```json
{
    "cilqr_trajectory_conf": {
        "horizon": 40,  // 预测时域长度
        "dt": 0.2       // 时间步长
    }
}
```

### 3. 预期值

- **N = 40** → 轨迹点数应该是 **41**（N+1）
- **delta_t = 0.2** → 轨迹长度应该是 **8.0 秒**（40 × 0.2）

### 4. 如果 N 值不对

检查日志输出中的 `N=` 值：
- 如果 `N=20`，说明配置没有正确读取，使用了默认值
- 如果 `N=40`，但轨迹点数不对，说明问题在求解器返回

### 5. 如果轨迹点数正确但长度不对

检查 `delta_t` 值是否正确。

## 常见问题

### Q: 为什么看不到调试输出？

A: 调试输出在 stdout，Web 服务器会捕获。如果看不到，检查：
1. Web 服务器是否正常运行
2. 是否在浏览器中触发了优化请求
3. 查看 Web 服务器的完整输出（包括 stderr）

### Q: N 值读取失败怎么办？

A: 代码有备用方案：
- 如果 `horizon` 未设置，会根据 `planned_time_length` 计算
- `planned_time_length = 6.0` → `N = ceil(6.0 / 0.2) = 30`

### Q: 如何手动测试？

A: 可以直接运行可执行文件：
```bash
cd single_frame
./build_x86_64_Release/planning_test_execute planning_config.json
```

查看输出中的调试信息。
