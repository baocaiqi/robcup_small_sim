// roles.hpp — 角色薄调度壳：按局面决定调用哪个公共模块，算法在 shoot/pass/defense/motion
#ifndef SIMURO5_ROLES_HPP
#define SIMURO5_ROLES_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

// 推球守卫总开关：true = 死球/摆位期或球在角区时一律不碰球（防 No pushing 犯规）
constexpr bool kNoPushGuardEnabled = false;

// 门将门线封堵：球在门框内轨迹上且快到门线时，把门线预测落点（贴线 3cm）写入 (tx,ty)
bool gk_cover_line_point(const WorldModel &wm, int id, double &tx, double &ty);

// 门将对准球的直线提前堵点（慢球朝门/沿门线滚）；false = 不触发
bool gk_line_block_point(const WorldModel &wm, int id, double &tx, double &ty);

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

// 球权滞回刷新：门将除外最近队友 <=kPossessOn 判"在我们手上"、>=kPossessOff 判"无主"，
//   中间沿用上一帧（40Hz 每帧翻转会让全队在两套站位之间来回抽搐）
void update_our_possession(WorldModel &wm);

// 球权中心化总闸：本帧是否由"无球收拢/有球接应"接管非持球人（传球链路生效期间一律不接管）；
//   判定必须在帧初做一次，帧内复用，否则同帧前后会跑两套逻辑
bool ball_centric_engaged(const WorldModel &wm);

// 防聚集硬约束：目标点离任一非门将队友 < kTeammateGap 时，沿"远离该队友"方向推到 gap 外
void separate_from_teammates(const WorldModel &wm, int id, double &tx, double &ty);

// 球权中心化目标点：无球=朝球收拢占位（3 槽贪心），有球=球前接应两点+掩护一点
//   返回 false 表示该队员不参与（门将/持球人/球无效）
bool ball_centric_target(const WorldModel &wm, int id, double &tx, double &ty);

// 按球权中心化目标点驱动该队员
void run_ball_centric(WorldModel &wm, int id);

}  // namespace simuro5
#endif
