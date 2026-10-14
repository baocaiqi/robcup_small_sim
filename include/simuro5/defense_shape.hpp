// defense_shape.hpp — 失球权时"球门侧保底人数"（原点左下角 cm，颜色无关）
// 参考对手 Strategy4Blue.dll 的防守站位思想（球在己半场时始终有人站在球与门之间、
//   站位点用威胁场挑"门线最危险的 y"，见 _re/spec/ThreatField-规格.md SPV2 一节），自研实现。
// 每帧在角色函数跑完后调用一次：球门侧人数不够，就从"人在球场侧"的无球队员里挑最近的一个，
//   改派到球-门连线上的护门点。只改目标点，轮速仍由 motion.cpp 出。
#ifndef SIMURO5_DEFENSE_SHAPE_HPP
#define SIMURO5_DEFENSE_SHAPE_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

// 返回被改派的队员 id（没有改派返回 -1）。skip[i]=true 的队员不参与（传球任务/压迫者等）。
int defense_shape_apply(WorldModel &wm, const bool skip[PLAYERS_PER_SIDE]);

}  // namespace simuro5
#endif
