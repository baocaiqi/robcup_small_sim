// ============================================================
// roles.hpp — 角色薄调度壳（角色只做调度，算法在公共模块）
// 每个角色文件只做一件事：根据当前局面决定调用哪个公共模块。
// 所有算法在 shoot/pass/defense/motion 公共模块中实现。
// ============================================================
#ifndef SIMURO5_ROLES_HPP
#define SIMURO5_ROLES_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

// ============================================================
// 第 49 轮「推球守卫」总开关（2026-09-12 正式落入主分支）
// ------------------------------------------------------------
//   true  = 第 49 轮原行为：**死球/摆位期 或 球在四角黄区内** → 一律不碰球
//           （治平台 `No pushing` 犯规：每 4 次白送对手 1 球）。
//           代价：角上的球没人碰、观感被动（实测"球推到门前没人管"）。
//   false = 等于 9/11「连胜版 38E8389E」的行为（真机 5 场胜利那一版）。
//
//   真机取舍依据（docs/06 第 50/51 轮、docs/21）：
//     守卫开：停表 8.6~19.0 次/分 → 1.1~6.1 次/分；但黄队纪律版胜率不及连胜版
//     守卫关：回到连胜版行为（用户拍板口径：先要成绩）
//   若后续 `No pushing` 犯规回升（4 次 = 1 球），把这里改回 true 即可，其它代码不用动。
// ============================================================
constexpr bool kNoPushGuardEnabled = false;

// 守门员：站球门前跟球 y；球逼近时出击
void run_goalie(WorldModel &wm, int id);

// 追球者（持球核心）：射门 → 传球推进 → 追球
void run_active(WorldModel &wm, int id);

// 防守站位：站球-门连线拦截点
void run_passive(WorldModel &wm, int id);

// 助攻跑位：站助攻参考点
void run_assist(WorldModel &wm, int id);

// 中场衔接：站中场参考点
void run_midfield(WorldModel &wm, int id);

}  // namespace simuro5
#endif
