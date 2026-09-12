// ============================================================
// shoot.cpp — 射门决策（docs/18 §8：机会质量驱动的动态射程）
//
// 本平台没有踢球动作：射门 = 用身体把球推出去，**球出射方向 ≈ 撞球瞬间的机头方向**。
// 所以本模块只负责"往哪儿瞄"，执行（到位+转正+推穿）由 roles.cpp + motion 负责。
//
// 角度遮挡几何（解析解，比"两个门柱口挑一个"精确）：
//   门张角区间 [−half, +half]（相对「球→门中心」方向）；
//   GK 遮挡区间 [gk_mid−gk_half, gk_mid+gk_half]（GK 半径 kGkRadius 在该距离的张角）；
//   净开口 = 门张角 \ 遮挡角 的最大连续空隙，取空隙**中心**为射门方向（连续值）。
//
// 射程闸门（docs/03 R17-18 的教训：无闸门放宽 70cm → 真机 0:3）：
//   · ≤70cm：维持"无条件可射"——sim A/B 验证过的进球主力区，
//     在这里加闸门会砍掉边际射门（实测净胜 5.7→4.4），不能碰；
//   · 70~110cm：**净开口 ≥8° 且 30cm 路线无遮挡** 才放行——0:3 事故正是远距盲射；
//   · 点球执行期：旁路闸门。罚球点距门 92cm（>70），门张角 ±12.3°、GK 遮挡 ±5.0°
//     → 净开口只剩 7.3°，按闸门会被永远拒掉（这正是"点球 0/3"的第二个根因）。
// ============================================================
#include "simuro5/shoot.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/geometry.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

namespace {

// —— 机会质量参数（数值集中此处，变更同步 docs/06）——
// ⚠️ 射程结论（docs/18 §8，两条独立证据都指向"别放宽"）：
//   · 真机历史：无闸门放宽 70cm → 真机 0:3（docs/03 R17-18，A/B 确认有副作用）
//   · 本轮 sim 4 种子 A/B：放宽到 110cm（带闸门）净胜 -1.0（噪声 σ=0.66）→ 已回退
//   因此 kFarShotEnabled 默认 false：远射档作为**能力**保留（一个常量可开），
//   等真机对准率提上来、有真机 A/B 数据后再单开一轮评估。
constexpr double kMaxShotNormal  = 70.0;   // 常规射程（旧口径，sim/真机都验证过的主力区）
constexpr double kMaxShotPenalty = 110.0;  // 点球：罚球点距门 92cm，必须能射
constexpr double kMaxShotFar     = 110.0;  // 远射档上限
// 远射档开关：**用户 2026-09-11 决定开启**（真机观查）。注意 sim A/B 是反对的：
//   50 场×4 种子净胜 -1.0~-1.68（σ=0.66）；docs/06 第 47 轮有完整数据。
//   用户理由：sim 的 carry 机制/弱脚本门将无法复现真机"射正率 8% vs 对手 48%"的问题，
//   该项只能真机裁决。若真机验证下来进攻变差，把这里改回 false 即回退（单常量）。
constexpr bool   kFarShotEnabled = true;
constexpr double kMinShot    = 5.0;     // 球距门线过近(<5cm)不射（无可推空间）
constexpr double kLegacyRange= 70.0;    // ≤此距离维持无条件可射（A/B 校准，勿当参数乱调）
constexpr double kMinOpen    = 8.0;     // 远射放行的最小净开口角（度）
constexpr double kAngleFull  = 18.0;    // 开口评分饱和角（度）
constexpr double kSpeedFull  = 8.0;     // 球速评分饱和（cm/帧）
constexpr double kGkRadius   = 8.0;     // GK 有效遮挡半径（本体 6 + 扑救余量 2）
constexpr double kLaneLen    = 30.0;    // 射门路线拦截检查长度 cm
constexpr double kLaneBlockR = 8.0;     // 拦截者判挡半径（本体 6 + 余量 2）
constexpr double kWOpen = 0.5, kWDist = 0.3, kWSpeed = 0.2;   // quality 权重

double deg(double rad) { return rad * 180.0 / SIMURO5_PI; }

}  // namespace

ShootPlan plan_shoot(const WorldModel &wm, int /*shooter_id*/) {
    ShootPlan plan;
    const TeamContext &ctx = wm.ctx;

    double bx = wm.ball.x, by = wm.ball.y;
    double ogx = ctx.opp_goal_x();                 // 对方门线 x
    double dgoal = dist(bx, by, ogx, 90.0);
    plan.shot_dist = dgoal;
    plan.penalty = wm.in_penalty_exec;
    const double max_shot = plan.penalty ? kMaxShotPenalty
                                         : (kFarShotEnabled ? kMaxShotFar : kMaxShotNormal);
    if (dgoal > max_shot || dgoal < kMinShot) return plan;

    // —— 对方守门员：离门线最近者 ——
    double gky = 90.0;
    double gk_best = 1e9;
    int gk_idx = -1;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = std::fabs(wm.opp[i].x - ogx);
        if (d < gk_best) { gk_best = d; gky = wm.opp[i].y; gk_idx = i; }
    }

    // —— 门张角（相对「球→门中心」方向，度）——
    double mid  = angle_to(bx, by, ogx, 90.0);
    double a_lo = angle_to(bx, by, ogx, goal_y_low());
    double a_hi = angle_to(bx, by, ogx, goal_y_high());
    double half = std::fabs(angle_diff(a_lo, a_hi)) / 2.0;

    // —— GK 遮挡区间（相对 mid）——
    double gk_dist = dist(bx, by, ogx, gky);
    double gk_half = (gk_dist > 1e-6) ? deg(std::atan2(kGkRadius, gk_dist)) : 90.0;
    double gk_mid  = angle_diff(angle_to(bx, by, ogx, gky), mid);
    double gl = gk_mid - gk_half, gh = gk_mid + gk_half;

    // —— 净开口 = 门张角 \ 遮挡角 的最大连续空隙，取空隙中心为瞄准方向 ——
    double open_angle = 0.0, aim_rel = 0.0;
    double lo1 = -half, hi1 = std::min(half, gl);          // 下段候选
    double lo2 = std::max(-half, gh), hi2 = half;          // 上段候选
    if (hi1 - lo1 > open_angle) { open_angle = hi1 - lo1; aim_rel = (lo1 + hi1) / 2.0; }
    if (hi2 - lo2 > open_angle) { open_angle = hi2 - lo2; aim_rel = (lo2 + hi2) / 2.0; }
    plan.open_angle = open_angle;

    // —— 射程闸门（远射档才判；点球永远旁路）——
    const bool dyn = kFarShotEnabled && (dgoal > kLegacyRange) && !plan.penalty;
    if (dyn && open_angle < kMinOpen) return plan;

    // —— 瞄准：**连续瞄准** = 门张角 \ GK 遮挡角 的最大空隙中心（用户决定启用）——
    //   旧口径是"门柱内侧两定点(74/106)挑离 GK 远的一侧"；连续瞄准的好处是 GK 偏一侧时
    //   瞄的是真实空隙中心、且开口很小时能反映出来（配合远射档闸门）。
    //   ⚠️ sim A/B：连续瞄准单独贡献约 -0.33（4σ）→ 已如实记录（docs/06 第 47 轮），
    //   真机观查；若真机变差，恢复下面注释里的旧"两定点"实现即可。
    //   旧实现（保留备查）：
    //     double goal_half_w = GOAL_WIDTH/2; aim_up = 90+goal_half_w-4; aim_down = 90-goal_half_w+4;
    //     aim_y = |aim_up-gky| >= |aim_down-gky| ? aim_up : aim_down;
    double aim_deg = mid + aim_rel;
    double ra = aim_deg * SIMURO5_PI / 180.0;
    double dirx = std::cos(ra), diry = std::sin(ra);

    // —— 路线拦截：球→开口方向 30cm 段，GK 之外防守者圆盘碰撞 ——
    CircleObstacle obs[PLAYERS_PER_SIDE];
    int no = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (i == gk_idx) continue;                         // 跳过 GK（遮挡已计入开口几何）
        obs[no].x = wm.opp[i].x; obs[no].y = wm.opp[i].y; obs[no].r = kLaneBlockR;
        ++no;
    }
    double lx = bx + dirx * kLaneLen, ly = by + diry * kLaneLen;
    plan.lane_blocked = !segment_clear_of_circles(bx, by, lx, ly, obs, no);
    if (dyn && plan.lane_blocked) return plan;             // 远射且有人站射门线 → 不硬射

    // —— 机会质量 quality ∈ [0,1] ——
    double speed = std::hypot(wm.ball.vx, wm.ball.vy);
    double q_open = clamp(open_angle / kAngleFull, 0.0, 1.0);
    double q_dist = clamp((kMaxShotFar - dgoal) / kMaxShotFar, 0.0, 1.0);
    double q_spd  = clamp(speed / kSpeedFull, 0.0, 1.0);
    plan.quality = kWOpen * q_open + kWDist * q_dist + kWSpeed * q_spd;
    // 点球：白送的射门机会，quality（球静止 → 只有 0.2 出头）不作数
    if (plan.penalty) plan.quality = 1.0;

    // —— 输出（两段式推射执行体兼容：dir 单位向量 + 瞄准角 + 球后 8cm 推球点）——
    plan.aim_y = clamp(by + std::tan(ra) * (ogx - bx), goal_y_low(), goal_y_high());
    plan.dir_x = dirx;
    plan.dir_y = diry;
    plan.aim_rot = angle_to(0.0, 0.0, dirx, diry);         // 机头应朝的角度(度)
    plan.target_x = bx - dirx * 8.0;
    plan.target_y = by - diry * 8.0;
    plan.viable = true;
    return plan;
}

}  // namespace simuro5
