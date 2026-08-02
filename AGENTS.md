# 🤖 三人小队 AI 协作指南（精简版）

> 继承自中型组 robcup_mid_sim 的 AGENTS.md，按 3 人小组裁剪。
> 所有开发建议从**模块目录**开始新会话（如 `cd strategy_5v5`），根目录本文件自动加载。

## 一句话工作流（每次会话 3 步）

1. **保存上下文**：聊到关键节点就让 agent 把决策/进展写入 `docs/03-开发进度跟踪.md`
2. **从模块目录启动**：`cd D:\robcup5v5足球仿真组小型\strategy_5v5 && claude`（或 codex）
3. **用人话说目标**：agent 自动匹配技能。示例：
   - "帮我做射门模块，先设计方案确认再实现"
   - "修 bug：守门员站位抖，先复现再定位"
   - "review 我改的 motion.cpp，跑 offline_test 验证"

## 团队铁律

1. **不要改 `simuro_interface.hpp` 的接口签名**——平台按 mangled 名加载，改了就加载失败
2. **必须 32 位 + MSVC**：`cmake -A Win32`，MinGW 编出来平台不认
3. **改代码前先跑测试**：`cmake --build build --config Release && build\Release\offline_test.exe`
4. **不引入第三方库**（平台加载的是裸 DLL，别依赖外部运行时）
5. **每次重要改动后更新 `docs/06-调参记录.md`**（数值改了哪些、为什么、效果）

## 三人分工（详见 docs/05-三人分工方案.md）

| 队员 | 模块 | 主要文件 |
|------|------|---------|
| A（队长） | 大脑：世界模型/角色分配/调度/摆位 | `world_model.cpp` `role_assignment.cpp` `strategy.cpp` `formation.cpp` |
| B | 运动与进攻：差速轮控制/射门/传球 | `motion.cpp` `shoot.cpp` `pass.cpp` |
| C | 防守与工程：守门员/防守/调参/日志复盘 | `roles.cpp`(goalie/passive) `defense.cpp` `tools/py/` |

## 测试命令速查

```bat
cmake --build build --config Release
build\Release\offline_test.exe              :: 冒烟测试（策略 300 帧 + 摆位 12 态）
python tools\py\rlg_analyzer.py 日志.rlg    :: 复盘比赛日志
```

## 常用约定

- 坐标系：cm/度，原点左下角 (0,0)，蓝队守右门 (x=220)
- 代码注释用中文；新模块先写头文件接口再写实现
- 分支：`main` 保持可运行；大功能开 `feat/xxx` 分支，review 后合入
