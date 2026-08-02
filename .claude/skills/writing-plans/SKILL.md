---
name: writing-plans
description: 把需求拆成 2-5 分钟一个的小任务（每个可独立完成验证）
---

# Writing Plans — 任务拆分

## 何时用
- 接到一个多步骤任务（如"做一个完整的射门模块"）

## 步骤
1. 用 spec 先定"做完"的标准
2. 拆任务：每步 2-5 分钟能完成、能验证
3. 每步写清：做什么 → 怎么验证（命令）→ 完成标志
4. 每完成一步立即验证并打勾，不攒着

## 本项目常用验证命令
```bat
cmake --build build --config Release        :: 编译
build\Release\offline_test.exe             :: 冒烟测试
python tools\pylg_analyzer.py 日志.rlg   :: 复盘
```

## 拆分示例（"让守门员不出禁区"）
1. 读 field_info.hpp 现有禁区判断 → 验证：grep 函数名
2. 在 run_goalie 里 clamp 目标点 → 验证：编译过
3. 加 offline_test 用例（球在门区外时目标点不出禁区）→ 验证：测试过
4. 提交 + 记录 docs/06 调参