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

}  // namespace simuro5
#endif
