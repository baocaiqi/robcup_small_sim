---
name: systematic-debugging
description: 4 步根因分析：复现→定位→缩减→修复+防守
---

# Systematic Debugging — 系统化调试

## 步骤
1. **复现**：稳定复现（offline_test 加用例；或复盘 .rlg 找到问题帧）
2. **定位**：二分法缩小范围（注释掉一半逻辑看现象变不变）
3. **缩减**：构造最小复现（固定输入，去掉无关分支）
4. **修复 + 防守**：修根因（不是打补丁），并加测试防止回归

## 本项目常见故障
| 现象 | 排查方向 |
|------|---------|
| DLL 加载失败/队名不显示 | 位数？mangled 名？C:\Strategy 路径？（docs/02） |
| 机器人原地转圈 | rotation 符号/归一化；Position 的 theta_e 处理 |
| 追球过头 | chase_ball 减速逻辑；球预测超前 |
| 角色不停换人 | role_assignment 惯性 -8 不够；站位置重叠 |
| 被罚点球 | 门区/禁区判断（field_info）与规则不符 |

## 防守惯例
修完 bug 后在 offline_test 加一条对应用例。