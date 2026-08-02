// ============================================================
// roles.hpp — 角色薄调度壳（沿用中型组「薄壳」设计原则）
// 每个角色文件只做一件事：根据当前局面决定调用哪个公共模块。
// 所有算法在 shoot/pass/defense/motion 公共模块中实现。
// ============================================================
#ifndef SIMURO5_ROLES_HPP
#define SIMURO5_ROLES_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

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
