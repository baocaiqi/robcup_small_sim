// ============================================================
// pass.hpp — 传球决策（简化版）
// 传球决策（简化版）思路：
//   1. 选一个接应队友（除持球者外离对方球门更近的）
//   2. 检查球→接应点路线是否被对手挡住（点-线段距离）
// 小型组无踢球动作，传球的执行 = 带球向接应点推进。
// ============================================================
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

// 当前锁定接球任务的到位判断：接球人能在球到达后容差时间内赶到锁定点才放行。
// 时间单位为帧；只读取 coop_pass_task 的 receiver_id / rx / ry，不重选人、不改点。
bool pass_receiver_ready(const WorldModel &wm);

// 出球前抢点判断：五名对手中若有人会比锁定接球人明显更早到锁点，则返回 true。
// 只读当前统一任务，不重选接球人或接球点；Receiving 阶段始终返回 false。
bool pass_opponent_arrives_first(const WorldModel &wm);

// ============================================================
// 配合进攻的传球方案（docs/06 第 69 轮，用户 2026-09-15 指令）
//   规则：对每个队友算"接球后能射门的机会质量"，取最高者传给**他**；
//        前提是该队友能**安全接到**球（传球线路无遮挡 + 接球点 20cm 内无对手
//        + 距离 ≤120cm）；**只向前传**（接球点到对方门必须比球更近）。
//   接球点用各角色的锚点（assist/mid/passive 的站位点，situation.cpp 每帧算出），
//   所以接球人是在"他正在去的落位"上接球，而不是他此刻的位置。
// ============================================================
struct CoopPass {
    bool viable = false;
    int receiver_id = -1;
    double rx = 0, ry = 0;          // 接球点（队友锚点）
    double score = 0.0;             // 接球后的射门机会质量（0~1）
    double dir_x = 0, dir_y = 1;    // 推球方向（球→接球点，单位向量）
    double aim_rot = 0.0;           // 机头应朝角度（度）
};
CoopPass plan_coop_pass(const WorldModel &wm, int passer_id);

}  // namespace simuro5
#endif
