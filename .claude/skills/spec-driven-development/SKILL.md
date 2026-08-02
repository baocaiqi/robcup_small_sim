---
name: spec-driven-development
description: 先写规格说明，再写代码（没 spec 不动手）
---

# Spec-Driven Development — 先规格后代码

## 何时用
- 新模块（如传球时机判断）、新接口（改头文件签名）

## 步骤
1. 写规格（10-30 行 Markdown）：
   - 输入/输出（精确到类型和单位，如 cm/度）
   - 行为规则（if/else 描述）
   - 边界情况（球出界、对方守门员挡住、PlayMode 变化）
   - 验收标准（能跑什么测试）
2. 把规格贴给队友 review（>=1 人同意）
3. 按规格实现 → 测试 → 文档（docs/04 同步）

## 本仓库规格模板（新模块照抄）
```markdown
## <模块名> 规格
输入: WorldModel& + 参数
输出: 目标点(x,y) / 决策结果
规则:
  1. ...
边界:
  - ...
验收: offline_test 增加 <case>
```