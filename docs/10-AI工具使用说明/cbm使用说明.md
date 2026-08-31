# cbm（codebase-memory）使用说明

> 面向后续接手的 AI / 开发者。看完这篇就知道 cbm 是什么、数据在哪、怎么查、有什么坑。

## 这是什么

**codebase-memory-mcp**（简称 cbm）是一个 MCP 服务，把代码库索引成**知识图谱**（函数/类/文件/调用关系），提供图查询工具代替 grep。索引全部**本地**跑（tree-sitter + LSP），不消耗 LLM token；只有查询时消耗少量 token。

> 项目主页（可选阅读）：https://github.com/DeusData/codebase-memory-mcp

- 本机配置：`~/.pi/agent/`（AGENTS.md 规范 + extensions/cbmem.ts + skills/codebase-memory）
- 项目索引名：`robcup_small_sim`，源路径 `/home/lalo/work/robcup/robcup_small_sim`
- 当前状态：**556 节点 / 1185 边**（commit `1f0115c`），ADR 已写入

## 工具清单与用途

| 工具 | 用途 |
|------|------|
| `search_graph` | 找函数/类/路由/变量（BM25 全文 + 名称正则 + 语义向量） |
| `trace_path` | 调用链：谁调 X（inbound）/ X 调谁（outbound）/ 双向 |
| `get_code_snippet` | 读指定函数的源码（先 search_graph 拿 qualified_name） |
| `get_architecture` | 项目总览：分层/聚类/热力点/边界 |
| `query_graph` | Cypher 复杂查询（聚合、多跳） |
| `detect_changes` | git diff → 影响面（blast radius） |
| `check_index_coverage` | 断言前核查索引覆盖（**必用**） |
| `index_status` / `index_repository` | 看状态 / 重新索引 |
| `manage_adr` | 读/写架构决策记录 |
| `ingest_traces` | 注入运行时调用数据增强图谱 |

## 常用查询配方（本项目实测）

```text
找防守相关函数      search_graph(query="defense mark threat intercept", project=robcup_small_sim)
找名字含 X 的       search_graph(name_pattern=".*plan_shoot.*", project=robcup_small_sim)
谁调用 plan_shoot   trace_path(function_name="robcup_small_sim.src.shoot.simuro5.plan_shoot", direction="inbound", project=robcup_small_sim)
读某函数源码        get_code_snippet(qualified_name="...", project=robcup_small_sim)
看项目架构          get_architecture(aspects=["overview","structure","hotspots","clusters"], project=robcup_small_sim)
找死代码            query_graph("MATCH (f:Function) WHERE f.transitive_loop_depth IS NULL AND NOT EXISTS(()-[:CALLS]->(f)) RETURN f.qualified_name", project=robcup_small_sim)
git 改动影响面      detect_changes(since="HEAD~5", project=robcup_small_sim)
```

路径注意：qualified_name 带项目前缀 `robcup_small_sim.`，用错前缀查不到。

## 证据铁律（重要）

1. **阴性结论先查覆盖**：图说"没人调用/不存在"时，先 `check_index_coverage`。partial/skipped 的文件必须 grep 源码核实。
2. **parse_partial 文件以源码为准**：本项目 `src/roles.cpp`、`src/pass.cpp`、`src/offline_test.cpp`、`include/simuro5/simuro_interface.hpp`(L91-103) 索引不全。
3. 结果 `has_more: true` 要分页（offset += limit）。
4. git 有改动后图会陈旧：先 `index_status` 看 generation，必要时 `index_repository` 重索引（full 模式含语义边）。

## 本项目的坑

- **UTF-16LE 编码文件**：pass.cpp / roles.cpp / offline_test.cpp 是 UTF-16LE，tree-sitter 解析不了 → 函数节点和 imports 边缺失。这是 parse_partial 的真根因。缓解：分析这些文件直接看源码；要修复可 iconv 转码后重索引（勿直接改仓库源文件编码）。
- 别名前缀：block nibble 的 `builtins.len/print` 等 Python 内建也建了节点，查询时会被混入，注意过滤。
- 索引是 per-commit 快照，合入新代码记得重索引。

## 常见问题

Q: cbm 和 ua 查出来节点数不一样？
A: 正常。cbm 是结构图（含 Section/Field/Variable 等细粒度节点，556），ua 是语义图（只保留 168 个有意义的 file/function/class/document）。两者互补。

Q: 怎么重新索引？
A: `index_repository(repo_path="/home/lalo/work/robcup/robcup_small_sim", mode="full")`。几秒到几分钟，索引不花 LLM token。

Q: 结果可信度？
A: 看 `check_index_coverage`。no_recorded_issue = 无已知缺口（不等于绝对完整）；partial = 指定行范围缺失，先读源码。