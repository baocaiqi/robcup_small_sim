# ua（understand-anything）使用说明

> 面向后续接手的 AI / 开发者。看完这篇就知道 ua 是什么、图谱在哪、怎么看、怎么重新生成。

## 这是什么

**understand-anything**（简称 ua，命令 `/understand`）是一个**全 LLM 管线**的代码库分析工具：扫描全部文件 → 批量让 LLM 读代码产出语义节点/边 → 分层 → 生成导览 tour → 校验 → 落盘 `knowledge-graph.json`。产物是可交互的知识图谱（浏览器 dashboard）。

> 项目主页（可选阅读）：https://github.com/Egonex-AI/Understand-Anything

**特点对比 cbm**：ua 贵（72 文件一次全量分析约 30-50 万 token，花在子代理）但产出"人话"级理解（分层、导览、文档关系、中文摘要）；cbm 便宜（索引免费）但偏结构查询。日常查代码用 cbm，理解全局/带新人/写文档用 ua。

## 本项目产物

| 文件 | 说明 |
|------|------|
| `/home/lalo/work/robcup/robcup_small_sim/.ua/knowledge-graph.json` | 主图谱（**168 节点 / 530 边 / 8 层 / 13 步 tour**，version 1.0.0） |
| `.ua/meta.json` | lastAnalyzedAt / gitCommitHash / analyzedFiles=72 |
| `.ua/fingerprints.json` | 结构指纹基线（增量更新用，别删） |
| `.ua/config.json` | 配置（outputLanguage=zh） |
| `.ua/intermediate/scan-result.json` | 扫描清单（增量更新复用，别删） |
| `/home/lalo/work/robcup/work/knowledge-graph.json` | 分析整理目录副本（work 目录） |
| `/home/lalo/work/robcup/work/07-ua知识图谱说明.md` | 图谱统计与修复记录 |

图谱内容速览：function 92 / document 37 / file 34 / class 4 / config 1；边以 documents(151)/calls(103)/contains(100)/exports(69)/imports(48)/related(51) 为主。

## 怎么看

**方式 1：交互式 dashboard（推荐）**

```bash
# 需要 node ≥22 + pnpm ≥10，插件在 ~/.understand-anything/repo/understand-anything-plugin
# 先装依赖（一次性）：
cd ~/.understand-anything/repo/understand-anything-plugin/packages/dashboard && pnpm install
cd ~/.understand-anything/repo/understand-anything-plugin && pnpm --filter @understand-anything/core build
# 启动（指向项目，GRAPH_DIR 告诉服务图谱在哪）：
cd ~/.understand-anything/repo/understand-anything-plugin/packages/dashboard \
  && GRAPH_DIR=/home/lalo/work/robcup/robcup_small_sim npx vite --host 127.0.0.1
```

启动后终端输出 `🔑 Dashboard URL: http://127.0.0.1:5173/?token=<TOKEN>`，**必须带 token** 访问，否则被访问门挡住。看：节点关系图、8 个分层、13 步 tour 导览（含 4 条 languageLesson：mangled 导出名 / header-only inline / sigmoid 速度曲线 / CMake Win32）。

**方式 2：直接读 JSON**（不依赖服务）

```bash
jq '.layers[].name' .ua/knowledge-graph.json                 # 分层
jq '.nodes[] | select(.type=="function") | .name' ...        # 函数清单
jq '.tour[] | {order,title,nodeIds}' ...                     # 导览
```

**方式 3：让 AI 查**——把 knowledge-graph.json 给任何 AI，让它按上面 168 节点/530 边回答；或复用 understand-chat 技能。

## 怎么重新生成 / 更新

```text
全量重建：  /understand --full --language zh /home/lalo/work/robcup/robcup_small_sim
增量更新：  /understand /home/lalo/work/robcup/robcup_small_sim   （git 有改动时自动按 changed-files 增量）
只看审查：  /understand --review /home/lalo/work/robcup/robcup_small_sim
排除文件：  /understand --exclude "tests/*,docs/06-调参记录.md" --full <path>
```

流程 7 阶段：扫描 → 分批 → 批量分析（子代理）→ 组装审查 → 架构分层 → tour 导览 → 校验落盘。
注意：
- 语言默认跟随会话检测，本项目已存 `outputLanguage=zh`，输出中文
- `.understandignore` 控制排除，改后需 `--full` 才生效
- 增量更新依赖 `fingerprints.json` 基线，别删
- 每次全量约 30-50 万 token（子代理消耗，主会话只收报告），项目越大越贵；改代码后优先走增量

## 本项目已知缺陷与修复记录

1. **UTF-16LE 文件导致 imports 边缺失**：`src/pass.cpp` / `src/roles.cpp` / `src/offline_test.cpp` 是 UTF-16LE，ua 的 extract-structure/import-map（tree-sitter）解析失败。首版图谱 imports 只 30 条，且缺这 3 文件全部 file→file imports 边。**已于 2026-08-26 手工补 18 条**（按源码 #include，weight 0.7），现 imports=48，重校验 0 issues。
   - 若后续增量更新覆写图谱，**记得重挂这 18 条边**（或先把 3 文件转码 UTF-8 再跑，避免重蹈覆辙）。
2. **2 条 calls 边被合并脚本丢弃**（strategy.cpp:run → motion.hpp:stop、→ role_assignment.hpp:assign）：目标函数在头文件未建模，属已知行为。
3. 扫描数 72 vs 预期 71：多出 `.ua/config.json` 自身，正常。

## 常见问题

Q: 图谱和 cbm 图节点数为什么差很多？
A: ua 只保存语义节点（file/function/class/document，168），cbm 保存全部结构节点（含 Section/Field/Variable，556）。不是错误。

Q: 改了代码要重新分析吗？
A: 小改走增量（自动）；动了接口/新增模块建议 `--full`。

Q: dashboard 打不开？
A: 确认带 `?token=`；确认端口 5173 没被占；确认 `.ua/knowledge-graph.json` 存在；缺依赖就跑上面的 pnpm install。