# cbm 索引报告（codebase-memory）

## 索引信息

- 项目名：`robcup_small_sim`
- 源路径：`/home/lalo/work/robcup/robcup_small_sim`
- 模式：full（含相似度/语义边）
- 结果：**556 节点 / 1185 边**，全部索引成功
- 日志：`/home/lalo/.cache/codebase-memory-mcp/logs/robcup_small_sim-1787719147.log`

## 排除项（设计使然，非失败）

- `.git`、`.claude` 目录被排除

## 部分解析文件（parse_partial，图内构造可能缺失）

| 文件 | 缺失范围 | 影响 |
|------|---------|------|
| `include/simuro5/simuro_interface.hpp` | L91-103 | 官方接口区，无业务价值 |
| `src/offline_test.cpp` | 全部 L1-567 | 测试代码，函数图缺失 |
| `src/pass.cpp` | 大量行 | pass 函数在图内缺失（已用 grep 补全文档） |
| `src/roles.cpp` | 全部 L1-412 | 角色行为函数图缺失（已用 grep 补全：spread_y/run_goalie/run_active/run_passive/run_assist/run_midfield） |

> 处理原则：这些文件的分析以源码为准，cbm 调用图仅作参考。

## 根因补充（2026-08-26）

`src/pass.cpp`、`src/roles.cpp`、`src/offline_test.cpp` 为 **UTF-16LE 编码**（`file` 确认），
tree-sitter 无法解析，导致：
- cbm 标记 parse_partial（函数/imports 缺失）
- ua 的 extract-structure/import-map 同样解析失败（importMap 为空，缺 18 条 imports 边，已手工补齐）

处理建议：若需完整索引，可 `iconv -f UTF-16LE -t UTF-8` 转码后重新索引（注意勿改动仓库源文件，或改编码后提交）。

## 查询入口

- 架构总览：`get_architecture(project=robcup_small_sim)`
- 函数/类搜索：`search_graph(query=...)`
- 调用链：`trace_path(function_name=..., project=robcup_small_sim)`
- 影响分析（git diff → blast radius）：`detect_changes(project=robcup_small_sim, since=...)`
