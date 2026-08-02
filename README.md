# FIRA 仿真 5v5 策略库（SimuroSot5）

> RoboCup 中国赛 FIRA 小型组 · 仿真 5vs5 项目的比赛策略代码。
> 三人小组，从零搭建的完整比赛策略。

## 这是什么

官方平台（SimuroSot5）是一个 Windows 仿真程序：策略 = 一个 C++ **DLL**，平台每周期把全场
信息（`Environment`，含双方 5 个机器人、球三帧、场地边界、比赛状态）传给我们，我们算完
后写回 5 个机器人的**左右轮速**。本项目就是那个 DLL 的源码。

| 产物 | 说明 |
|------|------|
| `Strategy4Blue.dll` | 蓝队策略（守右门 x=220，攻左） |
| `Strategy4Yellow.dll` | 黄队策略（守左门 x=0，攻右） |
| `tools/py/` | Python 工具：.rlg 比赛日志复盘、坐标调试 |

## 快速上手（Windows + VS2022 BuildTools）

```bat
:: 1. 生成 32 位工程（官方平台是 i386，DLL 必须 32 位！）
cmake -S . -B build -G "Visual Studio 17 2022" -A Win32 -DBUILD_TEST=ON

:: 2. 编译
cmake --build build --config Release

:: 3. 产物在 build/bin/Release/ 下：
::    Strategy4Blue.dll / Strategy4Yellow.dll

:: 4. 离线冒烟测试（不依赖平台）
build\Release\offline_test.exe
```

比赛前把两个 DLL 拷贝到 `C:\Strategy\`（覆盖同名文件），启动 `SimuroSot5.exe` 开赛。
**注意：文件名必须保持 `Strategy4Blue.dll` / `Strategy4Yellow.dll`**。

> 为什么必须 MSVC 编译？平台按 C++ mangled 导出名（如 `?RunStrategy@@YAXPAUEnvironment@@@Z`）
> 加载 DLL，MinGW 的符号命名不兼容。见 `docs/02-平台接口说明.md`。

## 目录结构

```text
include/simuro5/        策略内核头文件
  simuro_interface.hpp   官方平台接口（struct/枚举/导出函数，勿改）
  team.hpp               蓝黄镜像参数化
  geometry.hpp           几何工具
  field_info.hpp         场地常量与区域判断（禁区/球门/中圈）
  world_model.hpp        世界模型包装（Environment → 内部结构）
  situation.hpp          局势分析与站位参考点
  role_assignment.hpp    效用函数角色分配
  motion.hpp             差速轮控制（Position/Chase/Angle）
  shoot.hpp / pass.hpp / defense.hpp   行为模块
  roles.hpp              角色薄壳（goalie/active/passive/assist/midfield）
  formation.hpp          死球摆位（开球/争球/点球/任意球/门球）
  strategy.hpp           主调度
src/
  dll_blue.cpp           蓝队导出壳（SetFormerRobots 等 5 个接口）
  dll_yellow.cpp         黄队导出壳
  offline_test.cpp       离线冒烟测试
  *.cpp                  各模块实现
docs/                    规则速查/架构/接口/分工/进度/规范/调参/部署
tools/py/                Python：constants/geometry/rlg_analyzer
```

## 比赛关键参数速查

- 场地 220×180 cm，原点左下角，角度制；球门宽 40cm（y∈[70,110]）
- 每队 5 人，**1 号=守门员**；蓝队守右门（x=220），黄队守左门（x=0）
- 比赛 2 个半场各 5 分钟；PlayMode 12 种（开球/争球/点球/任意球/门球）
- 详细规则见 `docs/00-比赛规则速查.md`，接口细节见 `docs/02-平台接口说明.md`

## 参考资料（本机）

- 官方平台安装包：`D:\SIM5_platform\SIM5Installation_2023.exe`（自解压 rar，目标 C:\Strategy）
- 官方模板源码：`D:\SIM5_platform\SIM5_extracted\src\Strategy4Blue|Yellow\`
- 比赛规则 PDF：`D:\robcup5v5足球仿真组小型\4.1赛事规则...pdf`
- 用户手册：`D:\Users Manual for SimuroSot5 in Chinese（2023）.pdf`
