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
    // 防推球犯规：防守点距球保持 >= 8cm（球周围不挤球不推球）
    double db = dist(plan.target_x, plan.target_y, wm.ball.x, wm.ball.y);
    if (db < 8.0) {
        double ang = atan2(wm.ball.y - plan.target_y, wm.ball.x - plan.target_x);
        plan.target_x = wm.ball.x - 8.0 * cos(ang);
        plan.target_y = wm.ball.y - 8.0 * sin(ang);
    }
    return plan;
}

}  // namespace simuro5
