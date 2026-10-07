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

}  // namespace simuro5
#endif
