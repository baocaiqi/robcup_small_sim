---
name: verification-before-completion
description: 声称做完了之前强制跑测试验证，不口头说完成
---

# Verification Before Completion — 完成前验证

## 铁律
说"做完了"之前，必须给出**命令 + 输出**证明，不凭感觉。

## 本项目验收命令（三件套）
```bat
:: 1. 编译
cmake --build build --config Release
:: 输出: 0 error

:: 2. 冒烟测试
build\Release\offline_test.exe
:: 输出: === ALL TESTS PASSED ===

:: 3. (改了数值/决策后) 复盘
python tools\pylg_analyzer.py <一场比赛的.rlg> --frame <关键帧>
```

## 验收清单
- [ ] 编译 0 error（贴输出）
- [ ] offline_test 全绿（贴输出）
- [ ] 改动的参数已记录到 docs/06-调参记录.md
- [ ] 已提交（git log 可查）