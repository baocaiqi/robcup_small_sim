// ============================================================
// shoot.cpp — 射门决策（docs/18 §8：机会质量驱动的动态射程）
//              + 借墙射门（bank shot，docs/06 第 65 轮，用户 2026-09-14 指令）
//
// 本平台没有踢球动作：射门 = 用身体把球推出去，**球出射方向 ≈ 撞球瞬间的机头方向**。
// 所以本模块只负责"往哪儿瞄"，执行（到位+转正+推穿）由 roles.cpp + motion 负责。
//
// 直线射门：角度遮挡几何（解析解，比"两个门柱口挑一个"精确）：
//   门张角区间 [−half, +half]（相对「球→门中心」方向）；
//   GK 遮挡区间 [gk_mid−gk_half, gk_mid+gk_half]（GK 半径 kGkRadius 在该距离的张角）；
//   净开口 = 门张角 \ 遮挡角 的最大连续空隙，取空隙**中心**为射门方向（连续值）。
//
// 借墙射门（本文件下半部分）：直线被封（门将站位挡住开口 / 路线有人）时的**换角度**打法。
//   ⚠️ 关键物理（2026-09-14 实测 118 场真机 .rlg、273 个弹墙样本）：
//     球撞墙后 **法向分量只剩 0.66（中位）、切向保住 0.81** ⇒ **入射角 ≠ 反射角**，
//     出射线会明显往墙那边"扫"。所以**不能**用"把球门对墙镜像、连线求交点"的镜面做法
//     （那只在弹性各向同性时成立），必须按各向异性反射解方程（有闭式解，见 build_bank）。
//     对照：sim_bench 的 kWallRest=0.45 偏保守；defense.hpp 的 predict_y_at_x_reflect
//     用理想镜面（=1.0）偏乐观——两处都与实测不符，已在 docs/06 第 65 轮记录。
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

// 借墙方案统计（纯统计，不影响决策；声明见 shoot.hpp）
long g_bank_plans = 0;      // 新机会次数（上升沿）
long g_bank_frames = 0;     // 采纳帧数
bool g_bank_prev = false;   // 上一帧是否是借墙方案

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

// —— 借墙射门参数（2026-09-14；实测口径与推导见文件头 + docs/06 第 65 轮）——
constexpr double kBankRest   = 0.66;    // 法向恢复系数（实测中位 0.66/0.69/0.67/0.56）
constexpr double kBankFric   = 0.81;    // 切向保持（实测中位 0.78~0.81）
constexpr double kBankMaxDist= 260.0;   // 借墙总路程上限 cm（超过则距离项 0）
constexpr double kBankCornerFull = 40.0;// 反弹点离对方门线多远算满分（否则像"蹭门柱"）
constexpr double kBankCornerMin  = 12.0;// 反弹点离门线近于此 → 直接否决
constexpr double kBankAngleFull  = 18.0;// 借墙的开口满分角（与直线同口径）
constexpr double kBankMinSlope   = 0.25;// 入射"陡度"下限 |法向|/|切向|：太低=贴墙扫，不可靠
constexpr double kBankMinQ       = 0.45;// 借墙放行阈值
constexpr double kBankMargin     = 0.10;// 必须比直线好这么多才换（不打平就换）
constexpr double kBankDirectWeak = 0.35;// 直线 quality 低于此才算"没戏"，才考虑借墙
constexpr double kBankPrepDist   = 20.0;// 准备点=球后 20cm（与 roles.cpp 口径一致，做合法性检查）
constexpr double kBankPrepMargin = 5.0; // 准备点离场边余量
constexpr double kBankWOpen = 0.30, kBankWDist = 0.25, kBankWBounce = 0.25, kBankWSpd = 0.20;
// 借墙射门总开关：默认开。sim A/B 用它做"只隔离借墙"的对照（同一份代码跑开/关两批），
// 真机若验证下来进攻变差，改回 false 即回退（单常量，和 kFarShotEnabled 一个套路）。
constexpr bool kBankEnabled = true;

double deg(double rad) { return rad * 180.0 / SIMURO5_PI; }

// 对方守门员 = 离对方门线最近者，返回其下标并回填 y
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

// ============================================================
// 直线射门（原实现原样搬进来，行为不变）
// ============================================================
ShootPlan build_direct(const WorldModel &wm) {
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
    int gk_idx = find_opp_goalie(wm, ogx, gky);

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

// ============================================================
// 借墙射门：给定「墙 + 门内目标点 ty」求反弹点，再按四项算机会质量
// ------------------------------------------------------------
// 各向异性反射闭式解（推导）：
//   入射向量        i = (rx−bx, W−by)                      W = 墙的 y
//   出射向量（实测） o = ( i.x·kFric , −i.y·kRest )         切向×0.81，法向反号×0.66
//   要求出射方向指向目标 T=(ogx, ty)：
//       o × (T−R) = 0
//   令 c1 = kFric·(ty−W)、c2 = kRest·(W−by)，整理成 rx 的一次方程：
//       c1·(rx−bx) + c2·(ogx−rx) = 0
//       ⇒ rx = (c1·bx − c2·ogx) / (c1 − c2)
//   （镜面做法就是把 kRest/kFric 都当 1，本平台会系统性打偏 → 所以必须按实测系数解）
// ============================================================
ShootPlan build_bank(const WorldModel &wm) {
    ShootPlan none;
    const TeamContext &ctx = wm.ctx;
    const double bx = wm.ball.x, by = wm.ball.y;
    const double ogx = ctx.opp_goal_x();
    const double dgoal = dist(bx, by, ogx, 90.0);
    none.shot_dist = dgoal;
    none.penalty = wm.in_penalty_exec;

    // 点球不借墙（白送的直线机会）；射程沿用同一闸门，不趁机放宽
    if (none.penalty) return none;
    const double max_shot = kFarShotEnabled ? kMaxShotFar : kMaxShotNormal;
    if (dgoal > max_shot || dgoal < kMinShot) return none;

    double gky = 90.0;
    const int gk_idx = find_opp_goalie(wm, ogx, gky);

    // 障碍集合：第一段（球→反弹点）把门将也算进去（它可能出击到路上）；
    //          第二段（反弹点→球门）跳过门将——门将的遮挡用角度几何单独算
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

    ShootPlan best;
    double best_q = 0.0;
    const double ad = ctx.attack_dir();      // 进攻方向：蓝队 -1（攻 x=0），黄队 +1（攻 x=220）
    if (dgoal < 1e-6) return none;
    for (int wi = 0; wi < 2; ++wi) {
        const double wall = walls[wi];
        for (int ti = 0; ti < 3; ++ti) {
            const double ty = tys[ti];

            // ① 反射点闭式解
            const double c1 = kBankFric * (ty - wall);
            const double c2 = kBankRest * (wall - by);
            const double den = c1 - c2;
            if (std::fabs(den) < 1e-9) continue;
            const double rx = (c1 * bx - c2 * ogx) / den;
            // 反弹点必须落在「球 → 对方门」这段上（按进攻方向判，蓝黄通用）
            if ((rx - bx) * ad <= 3.0) continue;
            if ((ogx - rx) * ad <= 3.0) continue;
            const double corner_d = (ogx - rx) * ad;          // 反弹点到对方门线的距离
            if (corner_d < kBankCornerMin) continue;          // 太贴角区（像蹭门柱）
            const double ix = rx - bx, iy = wall - by;
            const double len_in = std::hypot(ix, iy);
            if (len_in < 1e-6) continue;
            if (std::fabs(ix) < 1e-6) continue;                // 纯法向入射：退化
            const double slope = std::fabs(iy) / std::fabs(ix);
            if (slope < kBankMinSlope) continue;               // 入射太"扫"，落点不可靠
            const double ux = ix / len_in, uy = iy / len_in;
            const double leg2 = dist(rx, wall, ogx, ty);
            const double L = len_in + leg2;

            // ② 遮挡：两段路线（第一段含门将，第二段不含）都要通
            if (!segment_clear_of_circles(bx, by, rx, wall, all5, n_all)) continue;
            if (!segment_clear_of_circles(rx, wall, ogx, ty, no_gk, n_no)) continue;

            // ③ 推球准备点合法性：球后 20cm 必须在场内、且不进对方门区（禁区纪律）
            const double px = bx - ux * kBankPrepDist, py = by - uy * kBankPrepDist;
            if (px < kBankPrepMargin || px > TeamContext::FIELD_LENGTH - kBankPrepMargin ||
                py < kBankPrepMargin || py > TeamContext::FIELD_WIDTH - kBankPrepMargin)
                continue;
            if (in_opp_goal_area(ctx, px, py)) continue;

            // ④ 从反弹点看门：目标方向是否被门将挡住；没挡住才算空隙宽度
            // ⚠️ 门将位置用**实际坐标**，不用"投影到门线上"（直线路径的老约定）：
            //    借墙时反弹点离门线只有几十厘米，把门将投到门线上会算错遮挡角
            //    （实测差 5~10°，会误判"目标被挡"或"没被挡"）。门线只用来算门框本身。
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
            if (a_t < -half_r || a_t > half_r) continue;       // 目标点不在门框内
            if (a_t > gl_r && a_t < gh_r) continue;            // 正被门将挡着 → 换墙/换点
            const double lo_gap = clamp(gl_r, -half_r, half_r) + half_r;   // 下侧空隙宽
            const double hi_gap = half_r - clamp(gh_r, -half_r, half_r);   // 上侧空隙宽
            const double gap = (a_t <= gl_r) ? lo_gap : hi_gap;
            if (gap <= 0.0) continue;

            // ⑤ 四项机会质量
            const double q_open   = clamp(gap / kBankAngleFull, 0.0, 1.0);
            const double den_d    = std::max(kBankMaxDist - dgoal, 30.0);
            const double q_dist   = clamp((kBankMaxDist - L) / den_d, 0.0, 1.0);
            const double q_inc    = clamp(slope, 0.0, 1.0);
            const double q_corner = clamp(corner_d / kBankCornerFull, 0.0, 1.0);
            const double q_bounce = q_inc * q_corner;
            // 撞墙保持率按实际入射方向算：|o| / |i|
            const double retain = std::hypot(ix * kBankFric, iy * kBankRest) / len_in;
            const double v_along = std::max(0.0, wm.ball.vx * ux + wm.ball.vy * uy);
            const double q_spd = clamp(v_along * retain / kSpeedFull, 0.0, 1.0);
            const double q = kBankWOpen * q_open + kBankWDist * q_dist +
                             kBankWBounce * q_bounce + kBankWSpd * q_spd;
            if (q <= best_q) continue;

            best_q = q;
            best = none;                       // 继承 shot_dist / penalty
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
    // quality 用借墙质量（roles.cpp 的机会闸门按它放行；shot_dist ≤70 时本来就无条件放行）
    best.quality = best.bank_quality;
    return best;
}

}  // namespace

ShootPlan plan_shoot(const WorldModel &wm, int /*shooter_id*/) {
    ShootPlan direct = build_direct(wm);
    if (!kBankEnabled) { g_bank_prev = false; return direct; }
    // 直线能射且不算差 → 不换（借墙天生更远更慢，只在直线没戏时换角度）
    if (direct.viable && direct.quality >= kBankDirectWeak) { g_bank_prev = false; return direct; }

    ShootPlan bank = build_bank(wm);
    if (bank.viable && bank.bank_quality >= kBankMinQ &&
        bank.bank_quality >= direct.quality + kBankMargin) {
        ++g_bank_frames;                          // 统计：本帧采纳了借墙方案
        if (!g_bank_prev) ++g_bank_plans;         // 上一次不是借墙 → 记一次**新机会**（上升沿）
        g_bank_prev = true;
        return bank;
    }
    g_bank_prev = false;
    return direct;
}

// 统计接口（声明见 shoot.hpp）
long bank_plan_count() { return g_bank_plans; }
long bank_frame_count() { return g_bank_frames; }

}  // namespace simuro5
