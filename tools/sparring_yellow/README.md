# 陪练黄队（官方 demo 改版）— 一键重建工程

> ⚠️ **红线提醒（AGENTS.md 原创红线 #1）**：官方 demo 的**源码不进本仓库、不进提交材料**。
> 本目录只放**我们自己的东西**：补丁脚本、重建工程、单元测试、说明。
> 官方源码仍放在平台自带目录（`C:\Strategy\src\Strategy4Yellow`）与真机工作目录
> （`C:\Strategy\.tmp\demo_yellow`）。

## 这是什么、为什么需要它

真机比赛里黄队（对手）默认是**官方 demo 的二进制 DLL**，我们改不动它。为了做"可复现的陪练对手"，
我们把平台自带的官方 demo 源码**在我们自己的机器上**编译成 `Strategy4Yellow.dll`，
并加了**只针对禁区位置**的补丁（不让它全员冲进我们小禁区——那会让平台反复判罚、比赛碎片化，
详见 `docs/06` 第 45/48 轮、`docs/21`）。

**它不是我们的比赛策略**，只是训练对手；比赛提交材料里不会有它。

## 一键重建（Windows + VS2022 + Python）

```bat
:: 1) 从平台自带源码重建"陪练黄队"（会自动拷贝源码 → 打补丁 → 编译）
cmake -S tools\sparring_yellow -B build\sparring_yellow -G "Visual Studio 17 2022" -A Win32
cmake --build build\sparring_yellow --config Release
:: 产物：build\sparring_yellow\Release\Strategy4Yellow.dll

:: 2) 跑"禁区纪律"单测（不需要平台）
build\sparring_yellow\Release\test_discipline.exe

:: 3) 部署到平台（先关掉 SimuroSot5，再覆盖）
copy /Y build\sparring_yellow\Release\Strategy4Yellow.dll C:\Strategy\
```

源码目录默认取 `C:\Strategy\src\Strategy4Yellow`；不一样就加
`-DDEMO_SRC_DIR="D:\path\to\Strategy4Yellow"`。

## 补丁做了什么（`tools/py/patch_demo_yellow.py`）

1. **禁区位置限制（v2 保留的三条）**：门区 y 带内站位目标 x ≤ 164；大禁区最多 1 人；
   实际位置踩进危险带立刻退回。
   ——目的：不反复触发平台判罚（真机实测停表 8.6~19.0 次/分 → 1.1~6.1 次/分）。
2. **门球落点 (10,70) → (10,90)**：原来球被摆在门柱线上，demo 门将只会"站到球的位置"而不穿球推，
   球永不动 → 平台每 5 秒重发门球（一场连发 8 次以上）。改到门前正中后可正常开球。
3. **`abs(double)` → `fabs`**：官方 2018 源码在 VS2022/MSVC 14.4x 下重载歧义（error C2668），
   纯机械替换，行为不变 —— 保证"原始源码 → 一键可编译"。

脚本可重放、可回滚：锚点唯一才改、写临时文件再原子替换、GBK 编码预检（官方源码是 GBK）。

## 单测覆盖什么（`test_discipline.cpp`）

针对 v2 的三条位置限制 + 两个"不误伤正常进攻"的对照：
门区带内不许再压 / 带外照常追 / 过冲后主动退回 / 大禁区最多 1 人（最近球者放行、非最近者挡住）/
球在门区里时压到区外等球。
