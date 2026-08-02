---
name: using-git-worktrees
description: 创建隔离的并行分支（改坏了直接丢弃）
---

# Using Git Worktrees — 分支隔离

## 何时用
- 3 人并行开发同一仓库；实验性改动（调参大改/重构）

## 常用命令
```bash
# 开一个隔离工作区
git worktree add ../robcup_small_sim_feat_shoot feat/shoot
# 在隔离区开发 → 测试 → 提交
# 合并回主工作区
cd /d/robcup5v5足球仿真组小型/strategy_5v5
git merge feat/shoot
git worktree remove ../robcup_small_sim_feat_shoot
```

## 注意
- 每个分支只在独立工作区开发，main 永远可运行
- 改坏了：`git worktree remove --force` 丢弃，不污染 main
- 3 人共用 GitHub 远程：push 前先 pull（或各自分支再 PR）