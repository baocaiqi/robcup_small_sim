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
    int gk_i = -1;
    double gk_best = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = std::fabs(wm.opp[i].x - ogx);
        if (d < gk_best) { gk_best = d; gk_y = wm.opp[i].y; gk_i = i; }
    }

    // —— 开口选择 v2：选「球所在侧 + 门将未覆盖」的空隙中心（射门角度更直），
    //    不再盲目选"离门将远的柱"（球在左而门将在左时斜射右柱大角度难进）。
    //    门框 y∈[74,106]（GOAL_WIDTH=40），门将有效覆盖 [gk_y-10, gk_y+10]。 ——
    const double kGoalHalf = TeamContext::GOAL_WIDTH / 2.0;   // 20
    const double kGkCover  = 6.0;                            // 门将有效覆盖半径 cm（可调，docs/06）
    const double kShotBlock = 10.0;                           // 射门线路阻挡阈值 cm（同 pass BLOCK_THRESHOLD）
    double aim_up   = 90.0 + kGoalHalf - 4.0;                 // 106 上柱内侧
    double aim_down = 90.0 - kGoalHalf + 4.0;                 // 74  下柱内侧
    bool up_free   = (gk_y + kGkCover) < aim_up;              // 上侧有空隙
    bool down_free = (gk_y - kGkCover) > aim_down;            // 下侧有空隙
    double up_center   = ((gk_y + kGkCover) + aim_up) / 2.0;  // 上空隙中心
    double down_center = (aim_down + (gk_y - kGkCover)) / 2.0;// 下空隙中心
    bool ball_up = (by > 90.0);

    // 候选开口（球侧优先，其次另一侧），再按线路是否被挡筛选
    double aim_y = 0.0;
    double cand[2]; int ncand = 0;
    if (ball_up)      { if (up_free)   cand[ncand++] = up_center;   if (down_free) cand[ncand++] = down_center; }
    else              { if (down_free) cand[ncand++] = down_center; if (up_free)   cand[ncand++] = up_center;   }
    if (ncand == 0) return plan;   // 门将覆盖全门（几乎不会发生，防御）

    for (int c = 0; c < ncand; ++c) {
        aim_y = cand[c];
        // 射门线路阻挡检查：球→开口线段被非门将对手挡住 → 换下一候选
        bool blocked = false;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            if (i == gk_i) continue;   // 门将已被开口选择避开，不重复判挡
            double dd = point_to_segment_dist(wm.opp[i].x, wm.opp[i].y, bx, by, ogx, aim_y);
            if (dd < kShotBlock) { blocked = true; break; }
        }
        if (!blocked) break;
        if (c == ncand - 1) return plan;   // 所有开口都被挡 → 不射（转传球/带球）
    }

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
