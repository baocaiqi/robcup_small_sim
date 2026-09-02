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
    // —— docs/14 图论链规划：链信息（仅调试/测试用，不影响既有调用方）——
    double chain_value = 0.0;            // 最优链总代价（越小越好；>=1e6 视为无可行链）
    int    chain_len   = 1;              // 链跳数（含第一跳；1 = 第一跳到射门区即止）
    int    hop2_id     = -1;             // 第二跳接应队友 id（-1 = 无第二跳）
};

PassPlan plan_pass(const WorldModel &wm, int passer_id);

}  // namespace simuro5
#endif
