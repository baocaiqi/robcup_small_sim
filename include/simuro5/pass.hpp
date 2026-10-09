// pass.hpp — 传球决策；小型组无踢球动作，传球 = 带球向接应点推进
#ifndef SIMURO5_PASS_HPP
#define SIMURO5_PASS_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

struct PassPlan {
    bool viable = false;
    int receiver_id = -1;
    double target_x = 0, target_y = 0;   // 带球推进目标（接应点前方）
};

PassPlan plan_pass(const WorldModel &wm, int passer_id);

// 接球人能否在球到达后容差（帧）内赶到锁定点
bool pass_receiver_ready(const WorldModel &wm);

// 是否有对手比我方接球人明显更早到锁定点（Receiving 阶段恒 false）
bool pass_opponent_arrives_first(const WorldModel &wm);

struct CoopPass {
    bool viable = false;
    int receiver_id = -1;
    double rx = 0, ry = 0;          // 接球点（队友角色锚点）
    double score = 0.0;             // 接球后射门质量 0~1
    double dir_x = 0, dir_y = 1;
    double aim_rot = 0.0;           // 机头角度 度
};
CoopPass plan_coop_pass(const WorldModel &wm, int passer_id);

// 门前横传 + 门柱包抄（docs/17 的 P4）：球在对方门前一侧时，把球横送到远门柱、
//   另一人包抄。接球点由本函数直接算出（角色锚点够不到门柱）。
CoopPass plan_cross(const WorldModel &wm, int passer_id);

// ── 传球安全闸（实现与内部状态在 roles.cpp）──
//   pass.cpp 的 plan_cross 要复用同一套护栏，所以提到头文件、脱离 roles.cpp 的匿名 namespace。
//   加这两条**没有放宽任何门槛**：判据一字未改，只是让 pass.cpp 也能调用。
bool pass_context_safe(const WorldModel &wm);
bool pass_target_safe(const WorldModel &wm, int passer, int receiver,
                      double tx, double ty, bool check_lane);

}  // namespace simuro5
#endif
