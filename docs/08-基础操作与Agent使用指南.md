# 08 · 基础操作与 Agent 使用指南（新人必读）

> 给刚加入的队员：**Git 怎么用**、**怎么编译测试**、**怎么让 AI Agent（Claude Code / Codex）帮你干活**。
> 看完这篇 + `docs/05-四人分工方案.md` 你的队员章节，就可以开工了。
> 进阶内容：`docs/01`（架构）、`docs/02`（平台接口红线）、`docs/04`（算法详解）、`docs/07`（部署）。

---

## 一、基础操作

### 1. Git —— 团队协作的地基

**概念一句话**：代码存在两个地方——你自己电脑（本地仓库）和 GitHub（远程仓库）。改代码 = 本地改 → `commit` 存个档 → `push` 同步到 GitHub；队友的改动用 `pull` 拉下来。

**第一次拿到代码（只做一次）**：

```bash
git clone git@github.com:baocaiqi/robcup_small_sim.git
cd robcup_small_sim
```

**日常循环（每次开发都用）**：

```bash
git pull                 # 1. 先拉队友的最新改动（避免冲突）
# ……改代码……
git status               # 2. 看改了哪些文件（红色=改过）
git add .                # 3. 把改动加入暂存区
git commit -m "feat: 传球模块加接应点前移"   # 4. 存档（消息格式见下）
git push                 # 5. 推到 GitHub
```

**分支（大功能别直接改 main）**：

```bash
git checkout -b feat/pass-improve   # 新建分支并切换
# ……开发完、测试通过……
git push origin feat/pass-improve   # 推分支
# 在 GitHub 上发起 Pull Request，review 后合入 main
git checkout main && git pull       # 合完后回到 main
```

**提交消息规范**（看 `git log` 学格式）：

| 前缀 | 用途 | 例子 |
|------|------|------|
| `feat:` | 新功能 | `feat: pass 加路线避挡检测` |
| `fix:` | 修 bug | `fix: 守门员出击后不归位` |
| `docs:` | 文档改动 | `docs: 补充调参记录` |
| `refactor:` | 重构（行为不变） | `refactor: motion 参数表抽成常量` |

**常见问题（照着做）**：

```bash
# 改错了想撤销
git restore 文件名            # 撤销未 commit 的改动
git checkout -- 文件名        # 同上（老版本写法）

# push 被拒（队友先推了）→ 先拉再推
git pull --rebase
git push

# 想看历史
git log --oneline -10

# 删错了分支
git branch -D 分支名
```

> ⚠️ 铁律：**不 commit 的改动等于没做**；push 前一定先 `git pull`；main 分支保持可编译可运行。

### 2. 编译与测试（Windows + VS2022）

**第一次（生成 32 位工程，只做一次）**：

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A Win32 -DBUILD_TEST=ON
```

**日常循环**：

```bat
cmake --build build --config Release     :: 编译
build\Release\offline_test.exe           :: 冒烟测试（策略300帧+12态摆位，必须全绿再提交）
copy /Y build\bin\Release\Strategy4Blue.dll C:\Strategy\   :: 拷到平台
copy /Y build\bin\Release\Strategy4Yellow.dll C:\Strategy\
:: 双击 C:\Strategy\SimuroSot5.exe 开赛看效果
```

> ⚠️ 三条红线（详见 docs/02）：DLL 必须 **32 位 + MSVC** 编译；文件名必须保留 `Strategy4Blue.dll` / `Strategy4Yellow.dll`；**不要改 `simuro_interface.hpp`**。

### 3. 比赛复盘（看日志找问题）

```bash
python tools/py/rlg_analyzer.py 比赛.rlg --frame N     # 看单帧（谁在哪、球在哪）
python tools/py/rlg_analyzer.py 比赛.rlg --csv out.csv # 导出全场轨迹
```

丢球了先复盘：守门员站位？谁没回防？球是怎么进的——带着证据去改代码，别瞎调参。

---

## 二、Agent 使用指南（让 AI 帮你写代码）

### 1. 什么是 Agent，能干什么

Agent 是一个在你终端里跑的 AI 程序员（本仓库配的是 **Claude Code** 或 **Codex**）：你说目标，它读代码、改文件、跑测试、提交，一步到位。它**不是搜索引擎**，是"坐在你旁边的结对程序员"。

本仓库已配好整套协作设施：`AGENTS.md`（自动加载的团队规则）、12 个团队 Skill（`.claude/skills/` 和 `.codex/skills/`，见 AGENTS.md 表格）、`docs/` 全套文档。**从仓库目录启动会话，这些会自动生效。**

### 2. 怎么启动

```bash
cd /你的路径/robcup_small_sim
claude        # Claude Code 用户
codex         # Codex 用户
```

### 3. 高效提问模板（决定它干得好不好）

**差的问法** ❌："帮我优化一下传球。"

**好的问法** ✅（目标 + 模块 + 验收）：

> "改 pass.cpp：现在的接应点固定是队友位置前 5cm，太死板。改成按队友与球门的距离动态前移（越近前移越多，上限 15cm）。改完跑 offline_test 验证全绿，更新 docs/06-调参记录。"

三个要素缺一不可：**做什么**（具体模块 + 具体行为）、**为什么**（旧逻辑的问题）、**怎么算成功**（验收标准）。Agent 擅长执行，不擅长替你猜需求。

### 4. 团队 Skill 怎么用

Skill 是预置的工作流。两种触发方式：
- 对话里直接说人话：**"先写方案再动手"**（= spec-driven-development）、**"先写测试再写代码"**（= TDD）、**"改完 review 一遍"**（= requesting-code-review）
- 或打斜杠命令：`/spec-driven-development`、`/test-driven-development`、`/systematic-debugging`

常用场景对照（详见 AGENTS.md 表格）：

| 你要干什么 | 说这句话 / 用这个 skill |
|-----------|------------------------|
| 需求不清楚，怕做错方向 | "先帮我澄清需求，一次问一个问题"（brainstorming） |
| 新功能 | "先写规格再动手"（spec-driven-development） |
| 改代码怕改坏 | "先写测试"（test-driven-development） |
| 程序出 bug | "按 4 步根因排查：复现→定位→缩减→修复"（systematic-debugging） |
| 改完了 | "review 一下我这次改动"（requesting-code-review） |
| 说做完了 | 它必须贴出命令 + 输出（verification-before-completion） |

### 5. 每次会话必做三件事（上下文三件套）

Agent 每次会话是"失忆"的，你负责喂上下文：

1. **看进度**：`docs/03-开发进度跟踪.md`（现在做到哪了）
2. **看调参**：`docs/06-调参记录.md`（哪些参数动过、效果如何）
3. **看你的模块**：你负责的 `.hpp` 头文件（接口长什么样）

```bash
# 例：B 队员（持球进攻）开工第一句
claude
> 读一下 docs/03 和 docs/06，然后我们改 motion 的 Ka 表，让机器人到位不抖。
```

### 6. Agent 边界与铁律（防翻车）

- **别让 agent 改这些**：`simuro_interface.hpp`（改签名=平台加载失败）、DLL 位数/文件名
- **它说"做完了"要验证**：让它贴 `offline_test.exe` 的输出，别只信嘴
- **一次只干一件事**：`incremental-implementation`——小块改、小块测、小块提交
- **重大改动先写方案**：让它在 `docs/` 里写清方案再动代码，你确认后它再实现
- **你才是队长**：agent 给的建议是提案，关键决策（架构、参数）你来拍板
- **沟通要勤**：你的改动涉及别人模块（如 C 改 `plan_pass` 影响 B 的 run_active），改完说一声，别闷头推

---

## 开工清单（新人第一天）

```text
[ ] git clone 仓库成功，git log 能看到历史
[ ] cmake 生成 + 编译通过 + offline_test 全绿
[ ] 读完 docs/05 你对应的队员章节
[ ] 用 claude/codex 各试一次（哪怕只是"帮我解释 roles.cpp 在干什么"）
[ ] 第一次 commit + push 成功
```
