# refs —— 参考资料库（已下载）

抓取时间：2025-08-26。来源：GitHub / archive.robocup.info / robogames.net / docin。

## 01_FIRA论文/（等价物 1）

| 文件 | 内容 |
|------|------|
| SOME_ADVICE_ON_FIRAS_SIMUROSOT_2007.pdf | 武汉理工 Xun Li，平台历史/缺陷/改进建议（FIRA 研讨会） |

> 未抓到（付费墙）：Situation evaluation in SimuroSot（Emerald 2012）、SimuroSot Strategy Development Kit（IEEE 2018）——DOI 在 `07-GitHub参考项目笔记.md` 附录 A。

## 02_RoboCup冠军TDP/（等价物 2，10 篇）

RoboCup 2D 仿真组冠军队伍年度"经验论文"（学术版 wp）：

| 队伍 | 年份 | 看点 |
|------|------|------|
| HELIOS | 2021/2022/2023 | 三连冠期间；对手建模、评估系统 |
| CYRUS | 2021/2022/2023 | 2021 冠军；传球预测、无球跑位、LSTM 降噪 |
| FRA-UNIted | 2023 | RL 训练一对一抢断（MDP 两球员环境） |
| YuShan | 2023 | 离线训练 + LambdaMART 进攻方向选择 |
| Hades | 2023 | 蒙特卡洛树 + RL 防守优化 |
| RoboCIn | 2023 | 自适应策略 + 开源日志分析器 |

> 全部来自 archive.robocup.info（官方存档，免登录）。2D 组 11v11 另一平台，思想通用、代码不通用。

## 03_中文培训/（等价物 3）

| 路径 | 内容 |
|------|------|
| DDDDDangbu-中型组比赛总结/ | 中国机器人大赛**中型组**仿真赛完整工程（FSM/角色分配/传球/运动控制，3000 行 18 模块），README 即"比赛技术总结"模板 |
| ayozzet-FIRA培训/ | FIRA Simurosot 马来西亚 2024 培训（Turtlesim/Gazebo/OpenCV 入门） |
| BIT珠海_FIRA仿真5v5培训资料.md | 北理工珠海 FIRA 5v5 培训资料（豆丁网）——**只抓到目录壳，全文需豆丁会员**，原文：https://www.docin.com/p-274690707.html |

## 使用提醒

- **查重红线**：全部资料只做思想/结构参考，禁止把代码抄进自己项目
- 英文论文可先看摘要/图表，重点吸收"怎么赢的"结论，不用全读
