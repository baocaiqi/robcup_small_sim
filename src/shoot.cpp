// shoot.cpp — 射门决策（无踢球动作，推球方向≈机头方向）：直线取门张角空隙中心 + 借墙反射
#include "simuro5/shoot.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/geometry.hpp"
#define TUNABLE_PREFIX "shoot."
#include "simuro5/tunable.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

long g_bank_plans = 0;
long g_bank_frames = 0;
bool g_bank_prev = false;

namespace {

// 射程(cm)：常规 70 / 点球 110（罚球点距门 92）/ 远射上限 110；≤70 无条件可射，近于 5cm 不射
TUNABLE(kMaxShotNormal, 70.0);
TUNABLE(kMaxShotPenalty, 110.0);
TUNABLE(kMaxShotFar, 110.0);
// 远射档开关：sim A/B 反对，单常量改回 false 即回退
constexpr bool   kFarShotEnabled = true;
TUNABLE(kMinShot, 5.0);
constexpr double kLegacyRange= 70.0;
// 远射最小净开口角(度)；开口饱和 19.2°、球速饱和 8cm/帧、GK 遮挡半径 8cm、路线检查 30cm 判挡 8cm、quality 权重
TUNABLE(kMinOpen, 5.0);
TUNABLE(kAngleFull, 19.2041);
TUNABLE(kSpeedFull, 8.0);
TUNABLE(kGkRadius, 8.0);
TUNABLE(kLaneLen, 30.0);
TUNABLE(kLaneBlockR, 8.0);
TUNABLE(kWOpen, 0.371001);
TUNABLE(kWDist, 0.3);
TUNABLE(kWSpeed, 0.2);


// 借墙参数（专用旋钮，不动 field_info 被门将共用的 0.66）：法向恢复 0.45、切向 0.81；路程上限 300cm、反弹点满分 40cm 低过 12cm 否决
//   开口满分 18°、入射陡度下限 0.25、放行 0.42、须优于直线 0.145、直线没戏 0.471、准备点 17.3cm 边距 7.9cm、蜂群射程 160cm、权重；总开关 kBankEnabled
TUNABLE(kBankWallRest, 0.45);
TUNABLE(kBankWallFric, 0.81);
TUNABLE(kBankMaxDist, 300);
TUNABLE(kBankCornerFull, 40.0);
TUNABLE(kBankCornerMin, 12.0);
TUNABLE(kBankAngleFull, 18.0);
TUNABLE(kBankMinSlope, 0.25);
TUNABLE(kBankMinQ, 0.42);
TUNABLE(kBankMargin, 0.144751);
TUNABLE(kBankDirectWeak, 0.471292);
TUNABLE(kBankPrepDist, 17.2849);
TUNABLE(kBankCarryMax, 160.0);
TUNABLE(kBankPrepMargin, 7.94513);
TUNABLE(kBankWOpen, 0.274509);
TUNABLE(kBankWDist, 0.25);
TUNABLE(kBankWBounce, 0.25);
TUNABLE(kBankWSpd, 0.20);
constexpr bool kBankEnabled = true;

// 真机借墙测试档：1 = 主攻只要几何合法就打墙（不看三道门槛）+ 关掉蜂群借墙推进，只测一个队员
TUNABLE(kBankForceTest, 0.0);

// 走廊拐弯参数：球在 x=30 时，从边路 y=20 看门只张开 12.6°，从中路 y=90 看是 67.4°（宽 5.4 倍）
//   触发：离门 < MaxDist 且离中线 > MinLateral；离门 < MinDist 或离中线 < MinLateral 时交给正常射门
TUNABLE(kCorridorMaxDist, 105.0);
TUNABLE(kCorridorMinDist, 30.0);
TUNABLE(kCorridorMinLateral, 22.0);
// 走廊门点 = 门线前 Gate cm 处的中线，向离门将远的一侧偏 GkShift（避免直接把球推给门将）
TUNABLE(kCorridorGate, 22.0);
TUNABLE(kCorridorGkShift, 0.35);
// 准备点距球 PrepDist cm，离边线留 Margin cm：球贴边线时横向分量被夹住 → 先往前推，多脚渐进拐进中路
TUNABLE(kCorridorPrepDist, 23.0);
TUNABLE(kCorridorMargin, 11.0);
// 推进方向必须有的朝对方门分量（cos 下限）；太小说明只能横推，放弃交给别的分支
TUNABLE(kCorridorMinFwd, 0.25);

double deg(double rad) { return rad * 180.0 / SIMURO5_PI; }

// 对方守门员 = 离对方门线最近者，返回下标并回填 y
int find_opp_goalie(const WorldModel &wm, double ogx, double &gky) {
    int idx = -1;
    double best = 1e9;
    gky = 90.0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = std::fabs(wm.opp[i].x - ogx);
        if (d < best) { best = d; gky = wm.opp[i].y; idx = i; }
    }
    return idx;
}

// 直线射门：净开口 = 门张角扣掉门将遮挡后的最大连续空隙，取空隙中心为瞄准方向
ShootPlan build_direct(const WorldModel &wm) {
    ShootPlan plan;
    const TeamContext &ctx = wm.ctx;

    double bx = wm.ball.x, by = wm.ball.y;
    double ogx = ctx.opp_goal_x();
    double dgoal = dist(bx, by, ogx, 90.0);
    plan.shot_dist = dgoal;
    plan.penalty = wm.in_penalty_exec;
    const double max_shot = plan.penalty ? kMaxShotPenalty
                                         : (kFarShotEnabled ? kMaxShotFar : kMaxShotNormal);
    if (dgoal > max_shot || dgoal < kMinShot) return plan;

    double gky = 90.0;
    int gk_idx = find_opp_goalie(wm, ogx, gky);

    double mid  = angle_to(bx, by, ogx, 90.0);
    double a_lo = angle_to(bx, by, ogx, goal_y_low());
    double a_hi = angle_to(bx, by, ogx, goal_y_high());
    double half = std::fabs(angle_diff(a_lo, a_hi)) / 2.0;

    double gk_dist = dist(bx, by, ogx, gky);
    double gk_half = (gk_dist > 1e-6) ? deg(std::atan2(kGkRadius, gk_dist)) : 90.0;
    double gk_mid  = angle_diff(angle_to(bx, by, ogx, gky), mid);
    double gl = gk_mid - gk_half, gh = gk_mid + gk_half;

    double open_angle = 0.0, aim_rel = 0.0;
    double lo1 = -half, hi1 = std::min(half, gl);
    double lo2 = std::max(-half, gh), hi2 = half;
    if (hi1 - lo1 > open_angle) { open_angle = hi1 - lo1; aim_rel = (lo1 + hi1) / 2.0; }
    if (hi2 - lo2 > open_angle) { open_angle = hi2 - lo2; aim_rel = (lo2 + hi2) / 2.0; }
    plan.open_angle = open_angle;

    const bool dyn = kFarShotEnabled && (dgoal > kLegacyRange) && !plan.penalty;
    if (dyn && open_angle < kMinOpen) return plan;

    double aim_deg = mid + aim_rel;
    double ra = aim_deg * SIMURO5_PI / 180.0;
    double dirx = std::cos(ra), diry = std::sin(ra);

    CircleObstacle obs[PLAYERS_PER_SIDE];
    int no = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (i == gk_idx) continue;
        obs[no].x = wm.opp[i].x; obs[no].y = wm.opp[i].y; obs[no].r = kLaneBlockR;
        ++no;
    }
    double lx = bx + dirx * kLaneLen, ly = by + diry * kLaneLen;
    plan.lane_blocked = !segment_clear_of_circles(bx, by, lx, ly, obs, no);
    if (dyn && plan.lane_blocked) return plan;

    double speed = std::hypot(wm.ball.vx, wm.ball.vy);
    double q_open = clamp(open_angle / kAngleFull, 0.0, 1.0);
    double q_dist = clamp((kMaxShotFar - dgoal) / kMaxShotFar, 0.0, 1.0);
    double q_spd  = clamp(speed / kSpeedFull, 0.0, 1.0);
    plan.quality = kWOpen * q_open + kWDist * q_dist + kWSpeed * q_spd;
    if (plan.penalty) plan.quality = 1.0;

    plan.aim_y = clamp(by + std::tan(ra) * (ogx - bx), goal_y_low(), goal_y_high());
    plan.dir_x = dirx;
    plan.dir_y = diry;
    plan.aim_rot = angle_to(0.0, 0.0, dirx, diry);
    plan.target_x = bx - dirx * 8.0;
    plan.target_y = by - diry * 8.0;
    plan.viable = true;
    return plan;
}

// 借墙射门：按各向异性反射闭式解求反弹点（入射角≠反射角，镜面做法会系统性打偏），再四项算质量
ShootPlan build_bank(const WorldModel &wm, double max_shot) {
    ShootPlan none;
    const TeamContext &ctx = wm.ctx;
    const double bx = wm.ball.x, by = wm.ball.y;
    const double ogx = ctx.opp_goal_x();
    const double dgoal = dist(bx, by, ogx, 90.0);
    none.shot_dist = dgoal;
    none.penalty = wm.in_penalty_exec;

    if (none.penalty) return none;
    if (dgoal > max_shot || dgoal < kMinShot) return none;

    double gky = 90.0;
    const int gk_idx = find_opp_goalie(wm, ogx, gky);

    // 第一段（球→反弹点）含门将，第二段（反弹点→门）跳过门将（遮挡另按角度算）
    CircleObstacle all5[PLAYERS_PER_SIDE], no_gk[PLAYERS_PER_SIDE];
    int n_all = 0, n_no = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        all5[n_all].x = wm.opp[i].x; all5[n_all].y = wm.opp[i].y; all5[n_all].r = kLaneBlockR;
        ++n_all;
        if (i == gk_idx) continue;
        no_gk[n_no].x = wm.opp[i].x; no_gk[n_no].y = wm.opp[i].y; no_gk[n_no].r = kLaneBlockR;
        ++n_no;
    }

    const double tys[3]   = {goal_y_low() + 4.0, 90.0, goal_y_high() - 4.0};
    const double walls[2] = {0.0, TeamContext::FIELD_WIDTH};
    const double kFric = kBankWallFric, kRest = kBankWallRest;

    ShootPlan best;
    double best_q = 0.0;
    const double ad = ctx.attack_dir();
    if (dgoal < 1e-6) return none;
    for (int wi = 0; wi < 2; ++wi) {
        const double wall = walls[wi];
        for (int ti = 0; ti < 3; ++ti) {
            const double ty = tys[ti];

            const double c1 = kFric * (ty - wall);
            const double c2 = kRest * (wall - by);
            const double den = c1 - c2;
            if (std::fabs(den) < 1e-9) continue;
            const double rx = (c1 * bx - c2 * ogx) / den;
            // 反弹点须落在「球 → 对方门」之间（按进攻方向判，蓝黄通用）
            if ((rx - bx) * ad <= 3.0) continue;
            if ((ogx - rx) * ad <= 3.0) continue;
            const double corner_d = (ogx - rx) * ad;
            if (corner_d < kBankCornerMin) continue;
            const double ix = rx - bx, iy = wall - by;
            const double len_in = std::hypot(ix, iy);
            if (len_in < 1e-6) continue;
            if (std::fabs(ix) < 1e-6) continue;
            const double slope = std::fabs(iy) / std::fabs(ix);
            if (slope < kBankMinSlope) continue;
            const double ux = ix / len_in, uy = iy / len_in;
            const double leg2 = dist(rx, wall, ogx, ty);
            const double L = len_in + leg2;

            if (!segment_clear_of_circles(bx, by, rx, wall, all5, n_all)) continue;
            if (!segment_clear_of_circles(rx, wall, ogx, ty, no_gk, n_no)) continue;

            // 推球准备点须在场内且不进对方门区
            const double px = bx - ux * kBankPrepDist, py = by - uy * kBankPrepDist;
            if (px < kBankPrepMargin || px > TeamContext::FIELD_LENGTH - kBankPrepMargin ||
                py < kBankPrepMargin || py > TeamContext::FIELD_WIDTH - kBankPrepMargin)
                continue;
            if (in_opp_goal_area(ctx, px, py)) continue;

            // 从反弹点看门：门将用实际坐标（投到门线上会算错遮挡角 5~10°）
            const double gk_x_r = (gk_idx >= 0) ? wm.opp[gk_idx].x : ogx;
            const double mid_r = angle_to(rx, wall, ogx, 90.0);
            const double a_lo_r = angle_to(rx, wall, ogx, goal_y_low());
            const double a_hi_r = angle_to(rx, wall, ogx, goal_y_high());
            const double half_r = std::fabs(angle_diff(a_lo_r, a_hi_r)) / 2.0;
            const double gkd_r = dist(rx, wall, gk_x_r, gky);
            const double gk_half_r = (gkd_r > 1e-6) ? deg(std::atan2(kGkRadius, gkd_r)) : 90.0;
            const double gk_mid_r = angle_diff(angle_to(rx, wall, gk_x_r, gky), mid_r);
            const double gl_r = gk_mid_r - gk_half_r, gh_r = gk_mid_r + gk_half_r;
            const double a_t = angle_diff(angle_to(rx, wall, ogx, ty), mid_r);
            if (a_t < -half_r || a_t > half_r) continue;
            if (a_t > gl_r && a_t < gh_r) continue;
            const double lo_gap = clamp(gl_r, -half_r, half_r) + half_r;
            const double hi_gap = half_r - clamp(gh_r, -half_r, half_r);
            const double gap = (a_t <= gl_r) ? lo_gap : hi_gap;
            if (gap <= 0.0) continue;

            const double q_open   = clamp(gap / kBankAngleFull, 0.0, 1.0);
            const double den_d    = std::max(kBankMaxDist - dgoal, 30.0);
            const double q_dist   = clamp((kBankMaxDist - L) / den_d, 0.0, 1.0);
            const double q_inc    = clamp(slope, 0.0, 1.0);
            const double q_corner = clamp(corner_d / kBankCornerFull, 0.0, 1.0);
            const double q_bounce = q_inc * q_corner;
            const double retain = std::hypot(ix * kFric, iy * kRest) / len_in;
            const double v_along = std::max(0.0, wm.ball.vx * ux + wm.ball.vy * uy);
            const double q_spd = clamp(v_along * retain / kSpeedFull, 0.0, 1.0);
            const double q = kBankWOpen * q_open + kBankWDist * q_dist +
                             kBankWBounce * q_bounce + kBankWSpd * q_spd;
            if (q <= best_q) continue;

            best_q = q;
            best = none;
            best.viable = true;
            best.bank = true;
            best.bank_wall = wall;
            best.bounce_x = rx;
            best.bank_quality = q;
            best.path_len = L;
            best.open_angle = gap;
            best.lane_blocked = false;
            best.dir_x = ux;
            best.dir_y = uy;
            best.aim_rot = angle_to(0.0, 0.0, ux, uy);
            best.aim_y = ty;
            best.target_x = bx - ux * 8.0;
            best.target_y = by - uy * 8.0;
        }
    }
    best.quality = best.bank_quality;
    return best;
}

}

ShootPlan plan_shoot(const WorldModel &wm, int /*shooter_id*/) {
    ShootPlan direct = build_direct(wm);
    if (!kBankEnabled) { g_bank_prev = false; return direct; }

    // 测试档：门前 ≤70cm 的无条件射区仍走直线（借墙天生更远更慢，抢它白扔机会）
    if (kBankForceTest >= 0.5) {
        ShootPlan bank = build_bank(wm, kFarShotEnabled ? kMaxShotFar : kMaxShotNormal);
        if (bank.viable && bank.shot_dist > kLegacyRange) {
            ++g_bank_frames;
            if (!g_bank_prev) ++g_bank_plans;
            g_bank_prev = true;
            return bank;
        }
        g_bank_prev = false;
        return direct;
    }

    // 直线能射且不算差 → 不换；借墙须比直线好 kBankMargin 才采纳
    if (direct.viable && direct.quality >= kBankDirectWeak) { g_bank_prev = false; return direct; }

    ShootPlan bank = build_bank(wm, kFarShotEnabled ? kMaxShotFar : kMaxShotNormal);
    if (bank.viable && bank.bank_quality >= kBankMinQ &&
        bank.bank_quality >= direct.quality + kBankMargin) {
        ++g_bank_frames;
        if (!g_bank_prev) ++g_bank_plans;
        g_bank_prev = true;
        return bank;
    }
    g_bank_prev = false;
    return direct;
}

ShootPlan plan_bank_carry(const WorldModel &wm) {
    if (!kBankEnabled) return ShootPlan{};
    if (kBankForceTest >= 0.5) return ShootPlan{};
    return build_bank(wm, kBankCarryMax);
}

// 走廊拐弯：把球从边路往门前中路推，让下一次射门面对更宽的门
ShootPlan plan_corridor(const WorldModel &wm, int shooter_id) {
    (void)shooter_id;
    ShootPlan plan;
    plan.corridor = true;
    const TeamContext &ctx = wm.ctx;
    if (wm.in_penalty_exec) return plan;

    const double ad = ctx.attack_dir();
    const double ogx = ctx.opp_goal_x();
    const double bx = wm.ball.x, by = wm.ball.y;
    const double dgoal = dist(bx, by, ogx, 90.0);
    plan.shot_dist = dgoal;
    if (dgoal > kCorridorMaxDist || dgoal < kCorridorMinDist) return plan;

    const double lateral = std::fabs(by - 90.0);
    if (lateral < kCorridorMinLateral) return plan;

    // 门点：中线上门前 Gate 处（在场内一侧），向离门将远的一侧偏；两侧都试，取第一条不被挡的线路
    double gky = 90.0;
    const int gk_idx = find_opp_goalie(wm, ogx, gky);
    const double gate_x = ogx - ad * kCorridorGate;
    const double bias_dn = clamp(90.0 + (90.0 - gky) * kCorridorGkShift, goal_y_low() + 3.0, goal_y_high() - 3.0);
    const double gate_try[2] = { bias_dn, 180.0 - bias_dn };

    for (int t = 0; t < 2; ++t) {
        const double gate_y = clamp(gate_try[t], goal_y_low() + 3.0, goal_y_high() - 3.0);

        double dx = gate_x - bx, dy = gate_y - by;
        const double len = std::hypot(dx, dy);
        if (len < 1e-6) continue;
        dx /= len; dy /= len;

        // 准备点（球后 PrepDist，沿推球线）必须留在场内：夹住横向分量，剩余给朝前分量
        const double y_lo_room = (by - kCorridorMargin) / kCorridorPrepDist;
        const double y_hi_room = (TeamContext::FIELD_WIDTH - kCorridorMargin - by) / kCorridorPrepDist;
        double uy = clamp(dy, -std::max(0.0, y_hi_room), std::max(0.0, y_lo_room));
        const double ux_mag = std::sqrt(std::max(0.0, 1.0 - uy * uy));
        if (ux_mag < kCorridorMinFwd) continue;          // 只能横推 → 交给别的分支
        const double ux = ad * ux_mag;

        // 前 LaneLen cm 有人挡 → 换另一侧门点
        CircleObstacle obs[PLAYERS_PER_SIDE];
        int no = 0;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            if (i == gk_idx) continue;                   // 门将用门点偏向绕开，不算挡
            obs[no].x = wm.opp[i].x; obs[no].y = wm.opp[i].y; obs[no].r = kLaneBlockR;
            ++no;
        }
        if (!segment_clear_of_circles(bx, by, bx + ux * kLaneLen, by + uy * kLaneLen, obs, no)) continue;

        plan.dir_x = ux; plan.dir_y = uy;
        plan.aim_rot = angle_to(0.0, 0.0, ux, uy);
        plan.aim_y = gate_y;
        plan.gate_y = gate_y;
        plan.target_x = bx - ux * 8.0;
        plan.target_y = by - uy * 8.0;
        plan.quality = 0.0;
        plan.viable = true;
        return plan;
    }
    return plan;
}

long bank_plan_count() { return g_bank_plans; }
long bank_frame_count() { return g_bank_frames; }

}  // namespace simuro5
