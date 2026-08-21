#include "simuro5/defense.hpp"
#include "simuro5/field_info.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

DefensePlan plan_defense(const WorldModel &wm) {
    DefensePlan plan;
    const TeamContext &ctx = wm.ctx;
    plan.target_x = wm.passive_x;
    plan.target_y = wm.passive_y;
    // 兑底：防守点若落入己方大禁区则推出（防堆叠送点球）
    if (in_penalty_area(ctx, plan.target_x, plan.target_y)) {
        plan.target_x = ctx.our_goal_x() + ctx.attack_dir() * 85.0;
        plan.target_y = 90.0;
    }
    return plan;
}

}  // namespace simuro5
