---
name: test-driven-development
description: 先写测试 → 看它失败 → 写最少代码 → 看它通过 → 提交
---

# Test-Driven Development — 测试驱动

## 何时用
- 写纯逻辑函数（几何、决策、摆位）

## 循环（红灯→绿灯→重构）
1. 写一个会失败的测试（offline_test.cpp 加用例）
2. 跑：确认失败（红灯）
3. 写最少代码让它过
4. 跑：确认通过（绿灯）
5. 重构（可选），再跑一遍确认仍绿
6. 提交

## 本项目落地
- 测试文件：src/offline_test.cpp（无平台依赖，可离线跑）
- 常用断言模式：
```cpp
static int test_xxx() { ... if (fail) { printf("FAIL..."); return 1; } return 0; }
```
- 平台相关（摆位在真实平台的表现）无法离线测 → 用 tools/py 复盘 .rlg 验证