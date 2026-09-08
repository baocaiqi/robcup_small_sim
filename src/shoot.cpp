#include "simuro5/shoot.hpp"
#include "simuro5/field_info.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

ShootPlan plan_shoot(const WorldModel &wm, int shooter_id) {
    ShootPlan plan;
    const TeamContext &ctx = wm.ctx;

    double bx = wm.ball.x, by = wm.ball.y;
    double ogx = ctx.opp_goal_x();
    double ad = ctx.attack_dir();

    // 条件：球在本方前方、距对方球门不太远（<70cm 才考虑射门）
    double dgoal = dist(bx, by, ogx, 90.0);
    // [A/B 隔离中] 点球放宽曾导致真机 0:3（14:46 场，黄攻蓝门区 2.5%→45%）——
    //   怀疑 in_penalty_exec 误判使运动战也放宽。先回退验证因果，确认后再恢复。
    if (dgoal > 70.0 || dgoal < 5.0) return plan;

    // 找对方守门员（对手 0 号或离门最近者）的 y
    double gk_y = 90.0;
    double gk_best = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = std::fabs(wm.opp[i].x - ogx);
        if (d < gk_best) { gk_best = d; gk_y = wm.opp[i].y; }
    }

    // 选择门柱开口：优先选离守门员远的一侧，其次选球当前 y 侧
    double goal_y_mid = 90.0;
    double half = TeamContext::GOAL_WIDTH / 2.0;      // 20
    double aim_up   = goal_y_mid + half - 4.0;        // 106 上柱内侧
    double aim_down = goal_y_mid - half + 4.0;        // 74  下柱内侧
    double gap_up   = std::fabs(aim_up - gk_y);
    double gap_down = std::fabs(aim_down - gk_y);
    double aim_y = (gap_up >= gap_down) ? aim_up : aim_down;

    // 射门目标点：球后方推球，方向对准开口
    double to_gx = ogx + ad * 5.0;                    // 门线上前 5cm
    // 球到开口的延长线方向
    double dirx = to_gx - bx, diry = aim_y - by;
    double len = std::hypot(dirx, diry);
    if (len < 1e-6) return plan;
    dirx /= len; diry /= len;

    plan.aim_y = aim_y;
    plan.dir_x = dirx; plan.dir_y = diry;   // 推球方向（两段式推射站位用，roles.cpp run_active）
    plan.viable = true;
    plan.target_x = bx - dirx * 8.0;                  // 球后方 8cm 推球点
    plan.target_y = by - diry * 8.0;
    return plan;
}

}  // namespace simuro5
