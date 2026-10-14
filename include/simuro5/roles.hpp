// roles.hpp — 角色薄调度壳：按局面决定调用哪个公共模块，算法在 shoot/pass/defense/motion
#ifndef SIMURO5_ROLES_HPP
#define SIMURO5_ROLES_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

// 推球守卫总开关：true = 死球/摆位期或球在角区时一律不碰球（防 No pushing 犯规）。
// ⚠️ 2026-10-13 用户指令**重新打开**（原为 false = 9/11「连胜版」口径）：
//   真机取证 `C:\Strategy\SimuroSot5.log`：**我方 blue 73 次** `Violated No pushing` vs 对手 yellow 31 次；
//   平台口径「每 4 次送对手 1 球」（docs/06 第 49 轮），那是一条我们一直在流血的口子。
//   实现是运行时可注入的旋钮（TUNABLE）：比赛 DLL 不能读参数文件，出厂走默认值 1.0。
//   ⚠️ 2026-10-14 测试层整体移除：原来用 `sim_bench --params roles.kNoPushGuard=0` 做单变量 A/B，
//      该仿真器已删除 ⇒ 现在只剩「改默认值重编」这一条对照/回滚路径。
// ⚠️ 注意：它治的是「No pushing 累积送球」，**不是**点球来源 —— 按我们自己的规则表
//   （docs/00:50、docs/06:4111-4112）点球由**门区纪律**判（门区内除门将外停留 >20 周期 / 门区 2+ 人）；
//   且 docs/work/要求与问题-20260926.md:119 已有先例：一段 7 次点球里 `Violated` **一条都没有**。
bool no_push_guard_enabled();

namespace detail {
// 纯推球许可判断；显式守卫参数仅用于测试 seam，比赛路径始终传正式配置值。
bool push_allowed_with_guard(const WorldModel &wm, bool guard_enabled);
// 带机器人身份的测试 seam；RestartSetup 例外仍须由现有 restart 归属/执行者确定。
bool actor_touch_allowed_with_guard(const WorldModel &wm, int robot_id, bool guard_enabled);
}  // namespace detail

// ⚠️ 2026-10-13 删除 `gk_cover_line_point`（门线封堵）：真机 1699 万帧只命中 26 帧，
//    门槛自相矛盾（tta<=8.3 帧 ∧ dist<77.49cm ⇒ 要求球速 >=373cm/s），且它要管的局面
//    已由 `gk_rule_shot_block` 覆盖（球离门线 <=9cm 时目标逐值相同）。删除理由写在 roles.cpp。

// 门将对准球的直线提前堵点（慢球朝门/沿门线滚）；false = 不触发
bool gk_line_block_point(const WorldModel &wm, int id, double &tx, double &ty);

// 己方门区纪律兜底（2026-10-13）：非 ACTIVE 角色跑完后统一调一次。
//   真机依据：判我们点球那几场官方 log 里 `Violated No pushing` 是 **0 次**，点球按
//   规则 7.10.1/7.10.2 是**门区纪律**判的（门区内除门将外停留 >20 连续周期 / 门区 2+ 人）；
//   执行这条纪律的 `own_box_clamp` 原来只被 `run_zone` 调用，而 `strategy.kZoneMode` 默认 0
//   ⇒ 死代码。形状：19:30 那场非门将球员在自家小门区里**连续 69 帧**（规则线 20）。
//   调用方负责先排除"传球任务参与者"（任务优先于站位纪律）。
void zone_box_clamp_role(WorldModel &wm, int id);

// 守门员：站球门前跟球 y；球逼近时出击
void run_goalie(WorldModel &wm, int id);

// 球外侧禁推：球已进我方门口且门将在球的场侧时，返回横移让开到球侧方的目标点；false = 不触发
bool gk_side_step_point(const WorldModel &wm, int id, double &tx, double &ty);

// 出球方向打分：往「队友多、对手少」的空当清；返回背离己门的单位方向 (dirx,diry)
void gk_clear_direction(const WorldModel &wm, int id,
                        double bx, double by, double &dirx, double &diry);
// 门前静止球（门球）无人抢时的推球方向：默认朝前带少许外侧斜度
void gk_restart_direction(const WorldModel &wm, int id,
                          double bx, double by, double &dirx, double &diry);

// 射门助跑距离 cm：球后多远开始冲（常规 kPrepDist，罚点球 kPenaltyPrepDist）
double shoot_prep_dist(const WorldModel &wm);

// 追球者（持球核心）：射门 → 传球推进 → 追球
void run_active(WorldModel &wm, int id);

// 重启抢球：球还钉在重启摆放点/基本静止时，球后离球最近的那台直接穿球推出去；
// 返回 true = 本帧已被接管（调用方不要再跑该角色的分支）。不在球后/非我方开球 → false
bool restart_charge(WorldModel &wm, int id);

// 这个 gameState 是不是「该我们开球」（争球算双方都可抢）
bool our_restart(const WorldModel &wm);

// 角色执行前取消不安全的配合任务/临时控球；飞行中不按散球或重算线路取消
void cancel_unsafe_coop_pass(WorldModel &wm);

// 防守站位：站球-门连线拦截点
void run_passive(WorldModel &wm, int id);

// 助攻跑位：站助攻参考点
void run_assist(WorldModel &wm, int id);

// 中场衔接：站中场参考点
void run_midfield(WorldModel &wm, int id);

// 前场散球逼抢：strategy 选出的逼抢者跑过去抢散球
void run_press(WorldModel &wm, int id);

// 官方式分区站位（strategy.kZoneMode 开关），取代 PASSIVE/ASSIST/MIDFIELD 三套分支
void run_zone(WorldModel &wm, int id);

}  // namespace simuro5
#endif
