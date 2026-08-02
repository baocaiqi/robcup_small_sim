---
name: file-search
description: 用 ripgrep 替代默认搜索，大仓库秒出结果
---

# File Search — 高效搜索

## 本项目规模
- 43 文件、约 3000 行 C++——不算大仓库，但养成习惯
- 优先用 git grep / rg（装 ripgrep 后）

## 常用搜索
```bash
# 找函数定义/引用
git grep -n "run_goalie"
# 找所有用到某常量的地方
git grep -n "GOAL_WIDTH"
# 找 TODO
git grep -n "TODO"
# 找来源标注（移植自哪）
git grep -n "移植自"
```

## 规则
- 搜代码用 grep（不要整文件读一遍）
- 搜历史用 git log -S（哪个提交引入了这个改动）