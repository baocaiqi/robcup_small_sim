// ============================================================
// shoot.cpp — 射门决策实现（docs/15 P0-3：机会质量驱动的动态射程）
//
// 模型（角度遮挡几何，解析解）：
//   门张角区间 [−half, +half]（相对「球→门中心」方向 mid）；
//   GK 遮挡区间 [gk_mid−gk_half, gk_mid+gk_half]；
//   有效开口 = 门张角 \ 遮挡角 的最大连续空隙，取空隙中心为射门方向；
//   路线 30cm 段做圆盘碰撞（除 GK 外防守者半径 8cm）。
//
// 与历史教训的关系（docs/03 R17-18：放宽 70cm 射程 → 真机 0:3）：
//   不再做射程一刀切。远射仅在「净开口 ≥8° 且 路线通」时放行——
//   GK 封死时远射自动被拒，这正是 0:3 事故中缺失的闸门。
// ============================================================
#include "simuro5/shoot.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/geometry.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

namespace {

// —— 机会质量参数（docs/15 §2.5 参数表，数值集中此处，变更同步 docs/06）——
constexpr double kMaxShot    = 110.0;   // 最大射程 cm（原 70，配合开口/线路双闸门放宽）
constexpr double kMinShot    = 5.0;    // 球距门线过近(<5cm)不射（无可推空间）
constexpr double kMinOpen    = 8.0;    // 可射最小净开口角（度）
constexpr double kAngleFull  = 18.0;   // 开口评分饱和角（度，开口再大收益封顶）
constexpr double kSpeedFull  = 8.0;    // 球速评分饱和（cm/帧）
constexpr double kGkRadius   = 8.0;    // GK 有效遮挡半径（本体 6 + 扑救余量 2）
constexpr double kLaneLen    = 30.0;   // 射门路线拦截检查长度 cm
constexpr double kLaneBlockR = 8.0;    // 拦截者判挡半径（本体 6 + 余量 2）
constexpr double kWOpen = 0.5, kWDist = 0.3, kWSpeed = 0.2;  // quality 权重

double deg(double rad) { return rad * 180.0 / SIMURO5_PI; }

}  // namespace

ShootPlan plan_shoot(const WorldModel &wm, int /*shooter_id*/) {
    ShootPlan plan;
    const TeamContext &ctx = wm.ctx;

    double bx = wm.ball.x, by = wm.ball.y;
    double ogx = ctx.opp_goal_x();                 // 对方门线 x
    double dgoal = dist(bx, by, ogx, 90.0);
    plan.shot_dist = dgoal;
    if (dgoal > kMaxShot || dgoal < kMinShot) return plan;

    // —— 找对方守门员：离球门线最近者 ——
    double gkx = ogx, gky = 90.0;
    double gk_best = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = std::fabs(wm.opp[i].x - ogx);
        if (d < gk_best) { gk_best = d; gkx = wm.opp[i].x; gky = wm.opp[i].y; }
    }

    // —— 门张角（相对「球→门中心」方向的角区间 [−half,+half]，度）——
    double mid   = angle_to(bx, by, ogx, 90.0);
    double a_lo  = angle_to(bx, by, ogx, goal_y_low());    // 下柱
    double a_hi  = angle_to(bx, by, ogx, goal_y_high());   // 上柱
    double half  = std::fabs(angle_diff(a_lo, a_hi)) / 2.0;

    // —— GK 遮挡区间（相对 mid）——
    double gk_dist = dist(bx, by, gkx, gky);
    double gk_half = (gk_dist > 1e-6) ? deg(std::atan2(kGkRadius, gk_dist)) : 90.0;
    double gk_mid  = angle_diff(angle_to(bx, by, gkx, gky), mid);
    double gl = gk_mid - gk_half, gh = gk_mid + gk_half;

    // —— 有效开口 = 门张角 \ 遮挡角 的最大连续空隙 ——
    double open_angle = 0.0, aim_rel = 0.0;
    double lo1 = -half, hi1 = std::min(half, gl);          // 下段候选
    double lo2 = std::max(-half, gh), hi2 = half;          // 上段候选
    if (hi1 - lo1 > open_angle) { open_angle = hi1 - lo1; aim_rel = (lo1 + hi1) / 2.0; }
    if (hi2 - lo2 > open_angle) { open_angle = hi2 - lo2; aim_rel = (lo2 + hi2) / 2.0; }
    plan.open_angle = open_angle;
    // —— 射程闸门（docs/15 P0-3 A/B 校准）——
    // ≤70cm（旧射程）：维持"无条件可射"，不做开口/路线闸门——sim 校验过的
    //   进球主力区，闸门在这里会砍掉脚本 GK 也能扑住的盲射（sim A/B 净胜 6.9→4.4）；
    // 70~110cm 动态激进区：净开口≥8° 且 路线通 才放行——0:3 事故正是远距盲射，
    //   该区必须带依据（开口大、没人站射门线）。
    const double kLegacyRange = 70.0;
    const bool dyn = (dgoal > kLegacyRange);
    if (dyn && open_angle < kMinOpen) return plan;         // 远射 GK 封死 → 不射

    // —— 射门方向 = 最大空隙中心（连续值，不再 74/106 两选一）——
    double aim_deg = mid + aim_rel;
    double ra = aim_deg * SIMURO5_PI / 180.0;
    double dirx = std::cos(ra), diry = std::sin(ra);

    // —— 路线拦截：球→开口中心方向 30cm 段，GK 之外防守者圆盘碰撞 ——
    CircleObstacle obs[PLAYERS_PER_SIDE];
    int no = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (std::fabs(wm.opp[i].x - gkx) < 1e-6 && std::fabs(wm.opp[i].y - gky) < 1e-6)
            continue;                                      // 跳过 GK（遮挡已计入开口几何）
        obs[no].x = wm.opp[i].x; obs[no].y = wm.opp[i].y; obs[no].r = kLaneBlockR;
        ++no;
    }
    double lx = bx + dirx * kLaneLen, ly = by + diry * kLaneLen;
    plan.lane_blocked = !segment_clear_of_circles(bx, by, lx, ly, obs, no);
    if (dyn && plan.lane_blocked) return plan;             // 远射且有人站射门线 → 不硬射

    // —— 机会质量 quality ∈ [0,1] ——
    double speed = std::hypot(wm.ball.vx, wm.ball.vy);
    double q_open = clamp(open_angle / kAngleFull, 0.0, 1.0);
    double q_dist = clamp((kMaxShot - dgoal) / kMaxShot, 0.0, 1.0);
    double q_spd  = clamp(speed / kSpeedFull, 0.0, 1.0);
    plan.quality = kWOpen * q_open + kWDist * q_dist + kWSpeed * q_spd;

    // —— 输出（两段式推射执行体兼容：dir 单位向量 + 球后 8cm 推球点）——
    plan.aim_y = clamp(by + std::tan(ra) * (ogx - bx),
                       goal_y_low(), goal_y_high());
    plan.dir_x = dirx;
    plan.dir_y = diry;
    plan.target_x = bx - dirx * 8.0;
    plan.target_y = by - diry * 8.0;
    plan.viable = true;
    return plan;
}

}  // namespace simuro5