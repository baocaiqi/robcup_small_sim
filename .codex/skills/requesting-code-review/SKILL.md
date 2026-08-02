---
name: requesting-code-review
description: 派子 agent 对照计划审查代码，按严重度处理意见
---

# Requesting Code Review — 代码审查

## 何时用
- 提交 PR / 合并到 main 之前（至少一次）

## 流程
1. 确保 offline_test 全绿 + 已提交
2. 请 review（队友或 AI）：
   `请 review 我最近一次提交，对照 spec 检查：逻辑正确性 / 边界情况 / 平台兼容性（32位/MSVC/接口签名）`
3. 按严重度处理：
   - 阻断（会 crash/加载失败/判罚违规）→ 必须修
   - 建议（可读性/性能）→ 记 TODO 或下次
4. review 通过后才合并 main

## 本项目 review 重点清单
- [ ] simuro_interface.hpp 签名没动
- [ ] 新文件加进了 CMakeLists.txt（CORE_SOURCES）
- [ ] 数值在 220×180 场地内（摆位/站位 clamp）
- [ ] 中文注释，来源标注（移植/重写）