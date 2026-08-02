---
name: incremental-implementation
description: 一次只改一小块，改完测完提交完再继续
---

# Incremental Implementation — 小步实现

## 何时用
- 所有代码改动（默认工作方式）

## 规则
1. 一次改动只做一件事（改一个函数/一个参数/一个文件）
2. 改完立刻编译 + 跑 offline_test
3. 绿了才 commit（小提交，信息写清"改了什么+为什么"）
4. 没绿就回退，不带着坏状态继续

## 提交信息模板
```
<type>: <一句中文说明>

<可选>为什么/影响（2-3 行）
```
type: feat / fix / refactor / docs / test

## 反例（禁止）
- 一个提交里同时改 motion + shoot + formation
- "先写完再一起测"（一次改 10 个文件）