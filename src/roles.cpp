#include "simuro5/roles.hpp"
#include "simuro5/role_assignment.hpp"
#include "simuro5/motion.hpp"
#include "simuro5/shoot.hpp"
#include "simuro5/pass.hpp"
#include "simuro5/defense.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/route.hpp"
#define TUNABLE_PREFIX "roles."
#include "simuro5/tunable.hpp"
#include <cmath>
#include <cstdio>
#include "simuro5/branch_trace.hpp"   // 须在所有 include 之后（仅诊断构建生效）

namespace simuro5 {

namespace {
bool touch_policy_allows(const WorldModel &wm, bool stage_allows_touch, bool guard_enabled) {
    if (!stage_allows_touch) return false;
    if (!guard_enabled) return true;
    return !in_no_push_zone(wm.ball.x, wm.ball.y);
}

enum class RestartOwner { Unknown, Blue, Yellow };

RestartOwner restart_owner(int game_state) {
    switch (game_state) {
        case PM_PlaceKick_Blue:
        case PM_PenaltyKick_Blue:
        case PM_FreeKick_Blue:
        case PM_GoalKick_Blue:
            return RestartOwner::Blue;
        case PM_PlaceKick_Yellow:
        case PM_PenaltyKick_Yellow:
        case PM_FreeKick_Yellow:
        case PM_GoalKick_Yellow:
            return RestartOwner::Yellow;
        default:
            return RestartOwner::Unknown;  // FreeBall 只给区域，不给球队归属
    }
}

bool restart_owned_by_us(const WorldModel &wm) {
    const RestartOwner owner = restart_owner(wm.game_state);
    return (wm.ctx.is_blue && owner == RestartOwner::Blue) ||
           (!wm.ctx.is_blue && owner == RestartOwner::Yellow);
}

bool is_our_goal_kick(const WorldModel &wm) {
    return (wm.ctx.is_blue && wm.game_state == PM_GoalKick_Blue) ||
           (!wm.ctx.is_blue && wm.game_state == PM_GoalKick_Yellow);
}

bool restart_exception_for_actor(const WorldModel &wm, int id) {
    if (wm.runtime_phase != RuntimePhase::RestartSetup || !wm.restart_armed ||
        !restart_owned_by_us(wm)) return false;

    if (is_our_goal_kick(wm))
        return id == 0 && wm.role[0] == ROLE_GOALIE;

    switch (wm.game_state) {
        case PM_PlaceKick_Blue:
        case PM_PlaceKick_Yellow:
        case PM_PenaltyKick_Blue:
        case PM_PenaltyKick_Yellow:
        case PM_FreeKick_Blue:
        case PM_FreeKick_Yellow:
            return id == wm.active_id && id > 0 && id < PLAYERS_PER_SIDE &&
                   wm.role[id] == ROLE_ACTIVE;
        default:
            return false;
    }
}
}  // namespace

namespace detail {
bool push_allowed_with_guard(const WorldModel &wm, bool guard_enabled) {
    return touch_policy_allows(wm, wm.runtime_phase == RuntimePhase::Running, guard_enabled);
}

bool actor_touch_allowed_with_guard(const WorldModel &wm, int robot_id, bool guard_enabled) {
    if (robot_id < 0 || robot_id >= PLAYERS_PER_SIDE) return false;
    const bool stage_allows_touch = wm.runtime_phase == RuntimePhase::Running ||
                                    restart_exception_for_actor(wm, robot_id);
    return touch_policy_allows(wm, stage_allows_touch, guard_enabled);
}
}  // namespace detail

namespace {
// 门将 y 夹取范围（门框 y∈[70,110]），下缘留 4cm 防撞下门柱
// ⚠️ 2026-10-10 试过放宽到 70/110（真机 8 场 20 个失球里 55% 从离门柱 12cm 内进，
//   想让它能站到门柱上），**实测把门球几何搞坏了、已回退**：
//     · `gk_goal_kick_reach` 的 y66 从 38 帧劣化到 82 帧（护栏 ≤70），y72 的"旧口径"分支从 89 → -1；
//     · 那套发球几何是扫 126 种门球摆位调出来的（docs/06:4332），动 y 带就动它。
//   ⇒ 结论：想封门柱不能靠放宽这个全局带，得单独重构发球路径。先保持 74/106，
//     并留成旋钮（改一行就能再试）。
TUNABLE(kGkYLo, 74.0);
TUNABLE(kGkYHi, 106.0);
// 封角（球-门连线）用的横向带。2026-10-10 由写死的 78/102 放宽到 74/106：
//   它比门框（70/110）窄 8cm，而 `gk_block_cy` 是「对手拿球时」门将横向目标的唯一来源
//   ⇒ 球从 y=105 进时门将最多只能到 102。真机 8 场 20 个失球：离最近门柱 <5cm 的占 30%、
//   <12cm 的占 55%。放宽只影响 `gk_block_cy`（封角/封球门线），**不碰门球几何**（那条走
//   gk_reachable_dir + kGkKickBand），所以不会重演第 70 轮放宽全局 y 带时踩的坑。
TUNABLE(kGkBlockYLo, 74.0);
TUNABLE(kGkBlockYHi, 106.0);

// 定位路径换新驱动律（2026-10-10）。**实测更慢，所以默认 0（关闭）**：
//   同一场景（门将朝场内 rot=180、需横移 12cm 重新站位）：
//     旧律 5 帧到 5cm、40 帧后剩 5.6cm ； 新律 40 帧都没到、剩 10.9cm。
//   原因：新律的横向阻尼 `motion.kLatDamp` 在"目标在正侧方"时把油门减半。
//   `docs/06:4333` 那个用例是"在侧方 6.6cm 处碰球"——**转向比平移重要**，所以新律赢；
//   而门将定位是"横移十几厘米重新站位"——**油门减半就是纯损失**。两个场景不能混用结论。
//   代码保留（1 = 打开），`offline_test` 的 `test_gk_lateral_transit` 记录了这组数字。
TUNABLE(kGkNewDrive, 0.0);

// 瞄准点「朝门中心收缩」系数（2026-10-10，真机 8 场数据解出来的）。
//   门将提前站位用 `90 + kGkShrink × (预测进门点 − 90)`，而不是直接站到预测点上。
//   为什么 k<1 反而更准：接触只需 6cm，所以"离预测点差 5cm"照样能挡；而落点预测在
//   远端差 8~13cm（球会被碰、方向被改写），硬追单个预测点等于赌那一次估计。
//   真机 639 个"同时有预测与真实越线"的样本，`|站位 − 真实越线 y| <= 6cm` 的命中率：
//     k=1.0（现状）39.9%   k=0.5 30.0%   **k=0.7 48.8%**   k=0.0（守门中心）31.0%
//     分档看（到线帧数）：3~8 帧 62.4%→65.6%、8~15 帧 49.6%→52.8%、
//                        15~25 帧 38.2%→49.1%、25~45 帧 25.0%→**50.8%**
//   附带好处：收缩后要横移的距离也变短，正好缓解"0.40s 横移 > 0.25s 可信窗口"。
//   只作用于**提前站位**（`gk_rule_default` / `gk_block_cy`）；最后一刻的封门线
//   （`gk_cover_line_point`）、门前死守、门球几何全部用原始预测，不受影响。
//   1.0 = 回退成"直接站到预测点"。
TUNABLE(kGkShrink, 0.7);
constexpr double kGkBlockLineDepth = 3.0;  // cm：门前死守 / 慢球堵门线时贴门线的深度（闸零与 gk_slow_block 共用）

// 对方机器人避障半径（本体 6 + 净空 4）；与 plan_route 的 margin 绑定，调小须同步调 margin
TUNABLE(kRouteInflate, 10.0);
TUNABLE(kNoAlignWait, 1.0);

// 避障移动：对方 5 车作圆盘障碍，直线被挡走可见图+Dijkstra；decel 时近目标 10cm 内线性减速
void move_avoiding(WorldModel &wm, RobotState &r, int id,
                   double tx, double ty, bool decel = false) {
    CircleObstacle obs[PLAYERS_PER_SIDE];
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        obs[i].x = wm.opp[i].x;
        obs[i].y = wm.opp[i].y;
        obs[i].r = kRouteInflate;
    }
    RoutePlan rt = plan_route(r.x, r.y, tx, ty, obs, PLAYERS_PER_SIDE);
    if (rt.found && rt.n_wp > 2) {
        wm.route_wp_next[id] = 0;      // 每帧重置（follow_route 自动跳段）
        motion::follow_route(r, rt, wm.route_wp_next[id], motion::TM_PASS);   // 追球=经过型
    } else {
        motion::position(r, tx, ty, motion::TM_PASS);
    }
    if (decel && kNoAlignWait < 0.5) {
        double dg = dist(r.x, r.y, tx, ty);
        if (dg < 10.0) { r.vl *= dg / 10.0; r.vr *= dg / 10.0; }   // 防冲过头
    }
}

// 推球守卫：死球/摆位期或球在角区时不碰球（平台 No pushing 犯规，每 4 次送对手 1 球）
bool push_allowed(const WorldModel &wm) {
    return detail::push_allowed_with_guard(wm, kNoPushGuardEnabled);
}

bool robot_touch_allowed(const WorldModel &wm, int id) {
    return detail::actor_touch_allowed_with_guard(wm, id, kNoPushGuardEnabled);
}

bool prep_point_ok(double px, double py) {
    if (!kNoPushGuardEnabled) { (void)px; (void)py; return true; }
    return !in_no_push_zone(px, py);
}

void hold_out_of_corner(WorldModel &wm, RobotState &r) {
    (void)wm;
    motion::stop(r);
}

// ═══════════════════════════════════════════════════════════════════════════
// 重启抢球（2026-10-11，用户要求"重新开球必须直接冲球，不许左右挪丢球权"）
//
// 为什么需要：逐帧取证（sim_trace --target-csv，900 帧）——
//   · 复位那一帧，站在球边 7.5cm 的那台拿的是"抢球"角色，目标点是**球心**；
//     而"主攻"那台在 49cm 外。等主攻换到近的人身上要 7 帧（0.18s），这期间球已经跑了 21cm。
//   · 抢球角色把目标取在球心 ⇒ 机头与目标夹角一旦落到 60~95°，`motion::position`
//     只会"先转身再走"（85~95° 前进分量硬性为 0）⇒ 实测 85~95° 档 92.5% 的帧**没有靠近球**。
// 做法（一条硬规则，不参与上层分支）：
//   球还钉在重启摆放点（runtime_phase=RestartSetup）或基本静止时，**球后（推进方向反侧）离球最近**的
//   我方场上队员，目标取「沿"我→球"方向再往前 kRestartThrough cm」——
//   即**穿过球**。这样目标方向与"机头对准球"方向完全共线（夹角≈0），
//   距离恒 ≥ kRestartThrough ⇒ 不会触发 de<1 停车、也不会被侧方死区拦住，必然直着推穿。
// ═══════════════════════════════════════════════════════════════════════════
// ⚠️ 默认 0 = 关。实测（40 局 × 3 种子 × scripted/wall，`--restart-gamestate` 口径）：
//   叠加本规则后 net 3.975 → 3.083、自己门区2+人 8.1 → 8.0 帧。**在 sim 上更差，所以不随构建生效**，
//   真机 A/B 想要"重启必直穿球"就把它改成 1.0 重编（用例 test_restart_charge 已把语义钉住）。
TUNABLE(kRestartCharge, 0.0);
// 额外的"球静止"判定（cm/帧）：默认 0 = 关。只用 runtime_phase 判重启 —— 因为实测（sim 默认口径球速
// 恒为 0、真机慢滚球也常见）一旦拿"球速小"当条件，本规则会在活球里抢走 53% 的帧，整场行为被换掉。
// 若真机发现某个重启态平台既不换 gameState、球也没跳 >30cm（live_play 不翻假），再把它设成 0.3 重编。
TUNABLE(kRestartStill, 0.0);
TUNABLE(kRestartRange, 60.0);    // cm：离球近于此的队员才接管（远的不抢，避免整场都在冲球）
TUNABLE(kRestartThrough, 30.0);  // cm：目标点取"穿过球再往前"这么远
TUNABLE(kRestartFwdMin, 0.0);    // 推球方向的朝前分量下限：<0（朝自家门推）一律不接管

BallState chase_target(const WorldModel &wm) {
    BallState t = wm.ball_pred;
    if (std::hypot(t.x - wm.ball.x, t.y - wm.ball.y) > 80.0) {
        t.x = wm.ball.x; t.y = wm.ball.y;
    }
    return t;
}

double spread_y(const WorldModel &wm, double bx, double by,
                double threat_radius, double max_offset) {
    double best_d2 = threat_radius * threat_radius;
    double nearest_dy = 0.0;
    bool found = false;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double dx = wm.opp[i].x - bx;
        double dy = wm.opp[i].y - by;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {
            best_d2 = d2;
            nearest_dy = dy;
            found = true;
        }
    }
    if (!found) return by;
    double frac = 1.0 - std::sqrt(best_d2) / threat_radius;
    double sign = (nearest_dy >= 0.0) ? -1.0 : 1.0;   // 敌人在上 → 往下躲
    return by + sign * max_offset * clamp(frac, 0.0, 1.0);
}

}  // anonymous namespace

// 这个 gameState 是不是"该我们开球"（争球算双方都可抢）
bool our_restart(const WorldModel &wm) {
    const bool blue = wm.ctx.is_blue;
    switch (wm.game_state) {
        case PM_FreeBall_LeftTop:
        case PM_FreeBall_LeftBot:
        case PM_FreeBall_RightTop:
        case PM_FreeBall_RightBot:  return true;
        case PM_PlaceKick_Blue:     return blue;
        case PM_PlaceKick_Yellow:   return !blue;
        case PM_FreeKick_Blue:      return blue;
        case PM_FreeKick_Yellow:    return !blue;
        case PM_GoalKick_Blue:      return blue;
        case PM_GoalKick_Yellow:    return !blue;
        case PM_PenaltyKick_Blue:   return blue;
        case PM_PenaltyKick_Yellow: return !blue;
        default:                    return true;   // 活球/未知态：不拦
    }
}

bool restart_charge(WorldModel &wm, int id) {
    if (kRestartCharge < 0.5 || id <= 0 || id >= PLAYERS_PER_SIDE) return false;
    if (!wm.ball.valid || wm.in_penalty_exec) return false;
    if (wm.game_state != PM_PlayOn && !our_restart(wm)) return false;   // 对方开球：不许去抢
    if (wm.runtime_phase == RuntimePhase::Running) return false;   // 只在"球还钉在重启摆放点"时接管（活球一律交回原逻辑）
    const TeamContext &ctx = wm.ctx;
    const double bx = wm.ball.x, by = wm.ball.y;
    if (!std::isfinite(bx) || !std::isfinite(by)) return false;
    if (in_goal_area_rule(ctx, bx, by, 5.0, 5.0)) return false;   // 球在自家门区：那是门将的活
    if (in_opp_goal_area(ctx, bx, by)) return false;              // 球在对方门区：不许进（规则 7.10 + 防挤堆）
    const double ad = ctx.attack_dir();
    // 选人：必须是"球后"（推球方向朝对方门）且离球最近的那台；不在球后的人冲过去会把球顶向自家门
    int pick = -1;
    double best = 1e18;
    for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {
        const RobotState &r = wm.home[i];
        if (!std::isfinite(r.x) || !std::isfinite(r.y)) continue;
        const double dx = bx - r.x, dy = by - r.y, n = std::hypot(dx, dy);
        if (n < 1e-6 || dx * ad / n < kRestartFwdMin) continue;      // 朝向分量不够 → 不接管
        if (n < best) { best = n; pick = i; }
    }
    if (pick != id || best > kRestartRange) return false;
    RobotState &r = wm.home[id];
    const double dx = bx - r.x, dy = by - r.y, n = std::hypot(dx, dy);
    motion::position(r, bx + dx / n * kRestartThrough, by + dy / n * kRestartThrough,
                     motion::TM_PASS);   // 目标在球的正后方延长线上 ⇒ 机头正对球、直线推穿
    return true;
}

// 守门员：每帧感知一次（GkView），按优先级逐条试规则表，第一条命中即出动作

TUNABLE(kGkHoldPassGap, 9.0);    // cm：门将要从球的场侧绕到球前时，横向必须让开球这么多。
                                 // 车身半径4 + 球半径2 = 6cm 就接触，9cm 留 3cm 富余保证蹭不到球。
TUNABLE(kGkLastDitch, 22.0);      // cm：球离门线 < 本值 → 门前死守
TUNABLE(kGkLastDitchGuard, 28.0); // cm：兜底规则"进门线"的触发距离（原写死 14，实测丢球时门将
                                  //   离球 25.5cm，差 11cm 就永远不触发）
TUNABLE(kGkHardMaxOut, 60.0);     // cm：门将离门线的硬上限。罚球区 80cm 深，但实测有 1.8~2.4%
                                  //   的帧离门线 >100cm、最大 184cm，说明有路径绕过 clamp_goalie_area
TUNABLE(kGkStayHomeBall, 30.0);  // cm：球离门线 >= 本值就算"球还远"，门将一律缩回门里。
                                 //   远射时站门线封角最大，跑出来只会把门口让开。
TUNABLE(kGkMaxDepth, 5.0);       // cm："缩在门里"时允许的最大出位深度。
TUNABLE(kGkNoPushDist, 64.8878);  // cm：球进我方门口这个距离内才管
TUNABLE(kGkSideClear, 38.0544);  // cm：侧向让开距离
TUNABLE(kGkBackOff, 9.358);  // cm：场侧回撤（目标是球后，绝不越球）
TUNABLE(kGkBehindMargin, 6.75991);  // cm：球须明显越过门将这么多才让开
// 门前横移（2026-10-08）：门将已站在球与门之间时，y 是"对到球的横移线"还是"往球外侧让"。
//   1 = 对到球（新，默认）；0 = 回滚成旧口径（by + side*lat，会被 clamp 到门框两端）。
//   实测：门前 17~18cm 的慢横移 93% 命中 gk_rule_clear，旧口径目标 y 只有 72.5/107.5 两个值。
TUNABLE(kGkTkCross, 1.0);

// ── 慢球不许"让开"（2026-10-09）──
//   失球取证（sim_bench 3 局 72000 帧 + 门将逐帧探针，两个慢球失球都是这一种）：
//   球一旦比门将更靠自家门（ball_goal < 门将离门线 − 6），旧口径就把门将的 x 钉死、
//   命令它"往球的侧方让开 kGkHoldPassGap+2" —— 对快球正确（追不上，硬碰就是推球入门），
//   对慢球是灾难：慢球给得起 0.7 秒，而门将恰好在这 0.7 秒里被命令"别往门前走"。
//   第2球实测：球以 0.65cm/帧 从离门线 9.7cm 爬到 0.4cm 用了 28 帧，门将目标恒为
//   (自己的x, 球的y−11) ⇒ x 从 204.3 反向退到 185.5（旧驱动纯侧向目标会画弧）、
//   y 该动 16cm 一步没动；等闸零把它拉回"门线上球的 y"时球只剩 0.4cm。
// ⚠️ 默认 0 = 关闭。2026-10-09 实测：打开（1.5）在 sim 上**更差**，见 docs/06 第 68 轮，
//   所以不随构建生效；真机 A/B 时把它改成 1.5 重编即可（关掉时与改前逐位一致，已核）。
TUNABLE(kGkBehindBlockSpeed, 0.0);  // cm/帧：球速低于此值 ⇒ 门将去堵门线而不"让开"。0 = 关闭
TUNABLE(kGkPassClear, 7.0);         // cm：门将走位路线离球心须留的净空（车身半径4+球半径2=6 接触）

// ── 推球准备点「可达带」（2026-10-10，与站位带分开）──
//   门球/门前静止球的准备点在球后 kGkPushDist；它站不站得下要按**推球用的老手感**判，
//   不能跟着站位带一起放宽：把站位带放到 70/110 后，球在 y=72 时 gk_reachable_dir 不再
//   镜像推球方向（它以为 y=72 站得下），结果直推推不出去 —— 用例 y72 从 89 帧变成 -1。
//   旧带 74/106 是扫过 126 种门球摆位调出来的（docs/06:4332），这里原样保留。
TUNABLE(kGkKickBandLo, 74.0);
TUNABLE(kGkKickBandHi, 106.0);

// ── 门前横传 + 门柱包抄（docs/17 的 P4）──
//   横传的几何/选择常量都在 pass.cpp（plan_cross）；这里只管包抄者到位后的迎球口径。
TUNABLE(kCrossFaceOn, 1.0);       // 0 = 包抄者只走位、不做"到位转正"（回滚观察用）
TUNABLE(kCrossArriveDist, 8.0);   // cm：离包抄点这么近就算到位
TUNABLE(kCrossArriveAng, 40.0);   // 度：到位转正的容差

bool gk_side_step_point(const WorldModel &wm, int id, double &tx, double &ty) {
    const TeamContext &ctx = wm.ctx;
    const RobotState &r = wm.home[id];
    double bx = wm.ball.x, by = wm.ball.y;
    if (ctx.dist_our_goal(bx) >= kGkNoPushDist) return false;   // 球还远：按常规防
    double gside = ball_goal_side(ctx, bx);
    // 球离门线 <12cm 时让开门槛取 0：只要球更靠己门就必须让开，绝不照常清球
    double behind_margin = (ctx.dist_our_goal(bx) < 12.0) ? 0.0 : kGkBehindMargin;
    if ((bx - r.x) * gside <= behind_margin) return false;      // 齐平/门侧 → 照常清球
    if (std::fabs(r.y - by) >= kGkSideClear) return false;      // 已让开：允许绕到球的门侧
    double side = (r.y >= by) ? 1.0 : -1.0;
    tx = bx - gside * kGkBackOff;                               // 球后（场侧）
    ty = clamp(by + side * kGkSideClear, kGkYLo, kGkYHi);
    clamp_goalie_area(ctx, tx, ty);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
// 【2026-10-13 删除】门线封堵 `gk_cover_line_point` / `gk_rule_cover_line`
//   （原四个旋钮 `kCoverLineDanger/Tta/Dist/GiveUp` 一并删除）
//
// 它自称"最后一道防线"（球在门框内轨迹上 → 抢门线落点），但两条独立取证都说它没在跑：
//   ① 真机 1 699 万帧门将逐帧 trace（`C:\Strategy\goalie_trace.csv`，10-04 构建，规则表同今天）：
//      这条规则**只命中 26 帧 = 0.00015%**；`default` 51.6% + `opp_ball_lock` 34.2% 才是实战。
//   ② 门槛之间自相矛盾：`dist_our_goal < 77.49` ∧ `tta <= 8.3 帧` 要求球速 >= 9.3cm/帧 = 373cm/s，
//      而真机越线球速中位 ≈80cm/s（2cm/帧）。两个门合起来只剩"球已进到离门线 6~17cm 的 2~4 帧"。
//   ③ 就算只把 `tta` 按真实球速放到 40 帧（复算 505 帧够格），其中 **243 帧（48%）被更早的
//      `opp_ball_lock` 抢走**、83 帧被 `line_block`、74 帧被 `ball_behind` ⇒ 只改门槛是**表面功夫**。
//   ④ 它要管的局面已由 `gk_rule_shot_block` 覆盖（on_target 即触发，无速度门；522 帧），
//      而且球离门线 <= 9cm 时 shot_block 的 `out_depth = max(3, ball_goal - kGkBallSideMargin=6)`
//      = 3cm ⇒ 与它下的目标 **(门线前 3cm, 预测落点)** 逐值相同。9~77cm 档 shot_block 站在
//      `kGkGuardDist=10cm`，比"退回门线"更保守，没有证据说后者更好。
//   ⑤ 删掉它还有个附带好处：`goal_line_hold`（贴门线+贴球 → 站定防乌龙，index 7）不再被它抢帧。
//
// ⚠️ 别凭"某个旋钮改了没用"就断定死代码（docs/06 第 74 轮补记的教训）—— 这次不是灵敏度，
//    是 1 699 万帧真机计数 + 门槛算术 + 与 shot_block 的目标逐值对比。
//    回归测试见 `test_goalie_last_ditch_line`（钉住"球贴门线 9cm 内 + 门将已在球的门侧
//    ⇒ 目标 = 门线前 3cm、y = 预测落点"这条行为仍由规则表提供）。
// ═══════════════════════════════════════════════════════════════════════════

TUNABLE(kGkLineBlock, 1.0);      // 总开关（0 = 回滚）
TUNABLE(kGkLbMinSpeed, 0.2);     // cm/帧：朝门速度下限（更慢交给门前静止球推出）
TUNABLE(kGkLbMaxSpeed, 2.5);     // cm/帧：A 只管慢球
TUNABLE(kGkLbRange, 100.0);      // cm：A 球离门线多近才对线
TUNABLE(kGkLbDepth, 10.0);       // cm：A 站位离门线
TUNABLE(kGkLbFreeDist, 15.0);    // cm：A 最近对手离球须大于此
TUNABLE(kGkLbLineBand, 12.0);    // cm：B 球离门线多近算"贴门线滚"
TUNABLE(kGkLbRollSpeed, 0.3);    // cm/帧：B 沿门线速度下限
TUNABLE(kGkLbAhead, 12.0);       // cm：B 站在球前方多远
TUNABLE(kGkLbBand, 30.0);        // cm：B 门口带半宽（|by-90| 小于此即在带内）

bool gk_line_block_point(const WorldModel &wm, int id, double &tx, double &ty) {
    if (kGkLineBlock < 0.5) return false;
    const TeamContext &ctx = wm.ctx;
    const RobotState &r = wm.home[id];
    const double bx = wm.ball.x, by = wm.ball.y, vx = wm.ball.vx, vy = wm.ball.vy;
    const double ball_goal = ctx.dist_our_goal(bx);
    const double danger = ball_danger_speed(wm);
    double y_at_goal = 0.0;
    if (danger > kGkLbMinSpeed && danger <= kGkLbMaxSpeed && ball_goal < kGkLbRange &&
        ball_goal >= ctx.dist_our_goal(r.x) - 2.0 && opp_clear_dist(wm) > kGkLbFreeDist &&
        predict_y_at_x(bx, by, vx, vy, ctx.our_goal_x(), y_at_goal) &&
        y_at_goal >= goal_y_low() - 2.0 && y_at_goal <= goal_y_high() + 2.0) {
        double depth = std::max(3.0, std::min(kGkLbDepth, ball_goal - 8.0));
        for (; depth > 3.0; depth -= 1.0) {
            double y = 0.0;
            if (predict_y_at_x(bx, by, vx, vy, ctx.our_goal_x() + ctx.attack_dir() * depth, y) &&
                y >= kGkYLo && y <= kGkYHi) break;
        }
        tx = ctx.our_goal_x() + ctx.attack_dir() * depth;
        if (!predict_y_at_x(bx, by, vx, vy, tx, ty)) ty = y_at_goal;
        ty = clamp(ty, kGkYLo, kGkYHi);
        clamp_goalie_area(ctx, tx, ty);
        return true;
    }
    // B：球贴门线滚向门口（门口带放宽到 30 罩住 y≈110~125 的弹跳区，避免 vy 抖动掉线）
    if (ball_goal < kGkLbLineBand && std::fabs(vy) > kGkLbRollSpeed &&
        ((by - 90.0) * vy < 0.0 || std::fabs(by - 90.0) < kGkLbBand)) {
        tx = ctx.our_goal_x() + ctx.attack_dir() * clamp(ball_goal - 1.0, 3.0, kGkLbLineBand);
        ty = clamp(by + (vy > 0.0 ? 1.0 : -1.0) * kGkLbAhead, kGkYLo, kGkYHi);
        clamp_goalie_area(ctx, tx, ty);
        return true;
    }
    return false;
}

TUNABLE(kClearAngleStep, 10.0);    // 扫候选角步长(度)
TUNABLE(kClearSectorSigma, 30.0);  // 高斯衰减宽度(度)
TUNABLE(kTeamWeight, 1.0);         // 每个队友方向加分
TUNABLE(kOppWeight, 1.5);          // 每个对手方向扣分
TUNABLE(kClearEdgeBonus, 2.0);     // 越靠侧面越加分
TUNABLE(kClearMinSide, 30.0);      // 最少偏离正前方角度（不许正面直线开球）
void gk_clear_direction(const WorldModel &wm, int id,
                        double bx, double by, double &dirx, double &diry) {
    const TeamContext &ctx = wm.ctx;
    double base_ang = (ctx.attack_dir() > 0.0) ? 0.0 : 180.0;
    double best_score = -1e9;
    double best_phi = 0.0;
    for (double phi = -85.0; phi <= 85.0 + 1e-9; phi += kClearAngleStep) {
        if (std::fabs(phi) < kClearMinSide) continue;
        double ang = base_ang + phi;
        double score = kClearEdgeBonus * std::fabs(std::sin(phi * SIMURO5_PI / 180.0));
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            if (i == id) continue;
            double a = angle_diff(angle_to(bx, by, wm.home[i].x, wm.home[i].y), ang);
            score += kTeamWeight * std::exp(-(a * a) / (2.0 * kClearSectorSigma * kClearSectorSigma));
        }
        for (int j = 0; j < PLAYERS_PER_SIDE; ++j) {
            double a = angle_diff(angle_to(bx, by, wm.opp[j].x, wm.opp[j].y), ang);
            score -= kOppWeight * std::exp(-(a * a) / (2.0 * kClearSectorSigma * kClearSectorSigma));
        }
        if (score > best_score) { best_score = score; best_phi = phi; }
    }
    double rad = (base_ang + best_phi) * SIMURO5_PI / 180.0;
    dirx = std::cos(rad);
    diry = std::sin(rad);
}

namespace {
TUNABLE(kGkKickFwd, 1.0);      // 回滚开关（0 = 原"朝侧面空当推"）
TUNABLE(kGkKickFwdDy, 0.35);   // 往外侧的斜度，sin 值
}
void gk_restart_direction(const WorldModel &wm, int id,
                          double bx, double by, double &dirx, double &diry) {
    if (kGkKickFwd < 0.5) { gk_clear_direction(wm, id, bx, by, dirx, diry); return; }
    diry = (by < 90.0 ? -1.0 : 1.0) * kGkKickFwdDy;
    dirx = wm.ctx.attack_dir() * std::sqrt(1.0 - diry * diry);
}

namespace {

constexpr double kGkGuardDist        = 10.0;   // 常规站位：门线前 cm
// ⚠️ 2026-10-10：这两个值**故意保持 76/104 不放宽**（尝试放到 71/109 后门球用例 y66 从 38 帧
//   劣化到 82 帧，越过 <=70 的护栏）。它们被门球/死球那几条（`gk_rule_no_push`、
//   `gk_rule_opp_ball_lock` 的兜底）用着，那套几何是扫 126 种摆位调出来的。
//   「站位要能封住门柱」这件事改由 `gk_rule_default` 直接夹到 `kGkYLo/kGkYHi`（70/110）来做。
TUNABLE(kGkTrackYLo, 76.0);
TUNABLE(kGkTrackYHi, 104.0);
constexpr double kGkMinSpeed         = 5.0;    // 朝门球速（cm/帧）低于此值不前压
constexpr double kGkFastShotSpeed    = 12.0;   // 朝门球速达到此值 → 前压到罚球区前缘
TUNABLE(kGkBlockMaxOut, 22.0);   // cm：门将封角站位的最外深度（旧实现写死 40，实测门将跑到
                                 // 离门线 34~49cm 后回不来，丢球时球已在其身后）。
TUNABLE(kGkBallSideMargin, 6.0); // cm：门将站位的最外深度 = 球离门线距离 - 本值。
                                 // 即门将永远比赛球更靠门至少这么远，球越近它越必须缩回门线。
constexpr double kGkOppPullback      = 20.0;   // 罚球区内每个对手让前压深度回缩（cm），防埋伏回敲
constexpr double kGkOppFrontPad      = 8.0;    // 前压深度上限：对方最前插球员身后余量（cm）
constexpr double kGkClearDist        = 20.0;   // 球进此距离 → 脚下清球
TUNABLE(kGkClearAlignTol, 7.0);   // 穿球前允许的机头偏差（度）：必须够小
                                // （原 20° 时门将带着 19° 偏航开推，球被越推越偏，
                                //   最后门将冲过球、球往自家门滚，触发新一轮来回蹭）
constexpr double kGkPushDist         = 8.0;    // 推球准备点：球后 cm
constexpr double kGkLateral          = 30.0;   // 静止球在门将身后时绕球侧移（cm）

TUNABLE(kGkAlignAcross, 3.0);   // "已站到球后"允许的横向偏差 cm。
                                // 它只当"承诺发球"的入口；入门之后一律不再重判，
                                // 所以门限窄不会像旧代码那样被骑着来回翻（旧代码没有承诺位）。
TUNABLE(kGkKickThrough, 45.0);  // 穿球目标：球前 cm（越远球被带得越快）
TUNABLE(kGkServeReady, 3.0);     // cm：承诺期的解锁余量（冲到球前这么多就算推空）
TUNABLE(kGkServeGlide, 12.0);   // cm：滑行段（TM_PASS）进此范围才切刹车精调。
                                 //   ⚠️ 不能缩小：TM_PASS 满速约 150cm/s，4cm 内刹不住会冲过
                                 //   准备点，导致"永远没到"来回抖（离线测试已证）。
TUNABLE(kGkPrepNeedAlign, 0.0); // 0=就位段不要求已经对准（转正留给开哨后那一帧）    // cm：离后撤点还远于此值就带速滑过去，进此范围才刹车精调
TUNABLE(kGkPrepPass, 1.0);      // 1=准备点带速走（TM_PASS）；0=精确站位
constexpr double kGkBallBehindMargin = 6.0;    // 球比门将靠门超过此值 → 强制回门
constexpr double kGkBallGoalSide     = 15.0;   // 强制回门：回到球的门侧 cm
constexpr double kGkBallRetreatLat   = 30.0;   // 强制回门：绕球侧移 cm
constexpr double kGkNoRoom           = 9.0;    // cm：球离门线小于此值 → 门侧站不下车身，不绕
constexpr double kGkPathClear        = 10.0;   // cm：回门路线离球的最小净空
constexpr int    kGkBehindHorizon    = 10;     // 帧：滚动球外推帧数
constexpr double kGkBehindBackOff    = 10.0;   // cm：回门走不了时停在球的场侧
constexpr int    kGkOppHoldFrames    = 3;     // 帧：对方持球滞回（防视觉闪断）

// 门将每帧的感知量：所有规则读同一份
struct GkView {
    double bx, by, vx, vy;
    double danger;       // 球朝己门的速度分量（横滚≈0、背离=0）
    double db;           // 门将到球距离
    double ball_goal;    // 球到己门线距离
    double gside;        // 球门在球的哪一侧（沿 x，±1）
    double opp_dmin;     // 最近对手到球距离
    bool   ball_still;   // 球速 <0.2cm/帧，或命中"门球角"启发式（含慢滚球，见 kGkCreepBand）
    bool   ball_dead;    // **真静止**：速度 < kGkDeadStill（平台把球钉住 = 门球/死球）
    bool   opp_has_ball; // 对方持球（带滞回）
    bool   heading_goal; // 球会到达己方门线
    bool   on_target;    // 且过门线时在门框内
    double y_at_goal;    // 过门线时的 y（原始预测，最后一刻的封线/死守用它）
    double y_aim;        // 提前站位的瞄准点 = 90 + kGkShrink×(预测 − 90)（见 kGkShrink 说明）
};

TUNABLE(kGkCreepStill, 1.0);     // 0 = 回滚（只认 <0.2cm/帧）
// 「真静止」阈值（cm/帧）：低于它 = 平台把球钉在摆点上（门球/死球）。
//   真机 rlg 实测：门球摆点 (205.2,77.7) 连续 89 帧球速恒为 **0.00**，
//   而"滚进门的慢球"是 0.05~0.65cm/帧 —— 用 0.02 能把两者干净分开。
//   这个判据只用来让**门球**免于"球在身后 → 绕行/让开/倒车"那几条防守规则；
//   慢滚球一律走原口径（`test_gk_last_ditch_no_freeze` 就是拿 0.05cm/帧盯着的）。
TUNABLE(kGkDeadStill, 0.02);
// 真静止球豁免"身后绕行/门前死守倒车"两条（1=开，默认；0=回滚）。见 gk_rule_ball_behind 与闸零处的说明。
TUNABLE(kTkDeadSkip, 1.0);
// 承诺期内是否"离后撤点太远就退回就位段"（1=开；0=关，默认）
//   ❌ 2026-10-11 实测（replay_trace 真机帧 2700~2830）：开了之后**推穿动作彻底消失**——
//   "穿球 A" 目标从 12 帧 → **0 帧**，门将改成在后撤点附近反复停靠（B 41→71 帧），
//   因为一离开后撤点去穿球就被这条判"离得远"而退回。留着旋钮只作记录，默认关。
TUNABLE(kGkServeKeep, 0.0);
TUNABLE(kGkServeKeepDist, 8.0);   // cm：承诺期内离后撤点超过此值 → 退回就位段重新停靠
// ═══ 二次修复（2026-10-11 晚）：重新开球时必须作废"发球承诺" ═══
//   真机取证（replay_trace 回放 20261011144043 帧 4650~4754，**修复版 DLL 确实在场**）：
//   平台把球摆回门球点 (205.2,77.7) 的那一帧，我们自己的 `goalie_serve_phase` 还是上一轮留下的 1，
//   于是门将在**离球 y 方向 12.2cm**、人还在门口 (214.8,89.9) 的状态下直接进承诺期：
//     4656~4669 原地拧机头（14 帧，vl=14 vr=-14 一格没挪）→ 4670 满速冲向穿球目标 (163.1,74.0)；
//   而"当前位置 → 穿球目标"这条直线在球的 x=205.2 处高 **9.2cm** —— 根本没过球心，
//   只是**擦着球边**滑过去（实测最近 6.5cm，球只被蹭到 0.13cm/帧 = 5cm/s，等于没推）。
//   冲过头之后 `ball_behind` 判"球已在我身后" → 命令**全速倒车**（vl=-150 vr=-126），
//   门将 x 在 [192.4,224.1] 之间来回冲、球 89 帧几乎不动 —— 这就是"每次发球都往旁边冲"。
//   平台重新摆球在帧内是 40cm 级瞬移（world_model 已把该帧球速清零，所以速度判据认不出它），
//   只有拿 ball_last 的位置差分才认得出。
TUNABLE(kGkRestartReset, 1.0);   // 1=球被平台重新摆放 → 承诺期清零、强制先站到球后（默认）；0=回滚
// cm/帧：球位单帧跳变超过它 = 平台重新摆球。取 30 与 world_model 自己的"复位"判据（kMaxBallVel）同口径，
//   因为正常活球最快也就 22cm/帧，超过 30 只可能是被摆过。
constexpr double kGkRestartJump = 30.0;
// ═══ 三次修复（2026-10-11 夜，第 76 轮）：第 75 轮部署后那场（20261011163823）门球仍 125 帧才推出 ═══
//   replay_trace 回放帧 7837~7960 看到两个新原因：
//   ① **带着惯性进承诺期**：门将从门口斜着往后撤点赶（vl≈46），进到 3cm 就算"到位"、切成原地转正，
//      但真机车有惯性——"只转不走"的那 14 帧里它照样滑了 **10cm**（(214.2,83.0)→(209.5,73.8)），
//      滑到球的另一侧去了；再按出球方向满速推，那条线离球心 5cm，只擦到球边 → 冲过头 → 倒车。
//      第 74 轮的 kGkServeBrake 想管这个但拿的是 r.vl/r.vr（上一帧**命令**，刹车时本来就大，永不释放）；
//      这里改用门将**真实**的帧间位移（run_goalie 自己记上一帧位置）判"停稳了没"。
//   ② **重新摆球没认出来**：球从 (195.1,86.0) 被摆回 (205.2,77.7) 只跳了 13.1cm（<30），
//      kGkRestartJump 认不出 → 承诺期 ph=1 粘着 → 门将在门口原地拧 16 帧后满速冲、又擦球边。
//      但门将自己同一帧被平台**瞬移** 11.3cm（正常最快 ~4cm/帧），这个信号认得出来。
TUNABLE(kGkServeSettle, 0.0);    // cm/帧：门将真实帧间位移低于它才算"停稳"、才许进承诺期；0=回滚
TUNABLE(kGkTeleportReset, 1.0);  // 1=门将被平台瞬移也算"重新开球"、承诺期清零（默认）；0=回滚
// cm/帧：门将单帧位移超过它 = 被平台摆回去了。正常满速 150cm/s ≈ 3.75cm/帧，真机重新开球实测 11~24cm。
constexpr double kGkTeleportJump = 8.0;
// 承诺期推球方向改成"门将 → 球心"（1=开；0=回滚成固定出球方向）。
//   原因 ① 的另一种治法：车滑偏了几 cm，仍按预设出球方向直推，那条线就不过球心 → 擦球边；
//   改成朝球心推，偏了也是正面顶到球，不需要"退回重摆"（第 74/75 轮两次退回重摆都循环翻车）。
//   只在偏差不超过 kGkAimBallMaxDeg 时才改，偏太多仍走原方向（随后由"推空/门侧"守卫解锁）。
TUNABLE(kGkAimBall, 1.0);
TUNABLE(kGkAimBallMaxDeg, 40.0);
// 承诺期的**第三个**解锁条件：球还**一动不动**（说明我们压根还没碰到它）却横向偏出 → 现在冲过去必然擦球边。
//   判据用 ball_dead（严格 <0.02cm/帧）而不是"离后撤点多远"：球一旦被我们碰到就会动，
//   这条立刻自动失效 —— 所以它绝不可能在推穿途中把门将拽回来，这正是 kGkServeKeep（距离判据）翻车的原因。
//   门限取 kGkAlignAcross*1.5（=4.5cm），比进承诺期的 3cm 宽 1.5cm 回差，保证"已站在后撤点"时不会来回翻。
//   ❌ 但实测这条**默认必须关**（和 kGkServeKeep 同样的翻车方式）：
//     126 种门球摆位回归从 `y72=89 / y78=42 / y66=38` 退化成 `y72=162 / y78=42 / y66=-1`
//     （y66 干脆 200 帧没把球弄出去），`门球静止球门将 200 帧内未触球` 直接变红。
//     原因：横向在 4.5~6cm 之间时 `gliding` 成立 → 用 TM_PASS 满速冲回后撤点 → 冲过 → 又偏出 → 循环。
//     ⇒ 真正管用的只有上面那条"重新摆球时清零承诺"（kGkRestartReset）。留旋钮作记录。
TUNABLE(kGkServeRealign, 0.0);   // 1=承诺期里球没被碰到又横向偏出就退回重摆；0=回滚（默认，见上）
// 承诺位里"残余速度还大就先用 TM_STOP 刹住"（2026-10-11 试做，**默认关**）。
//   ❌ 实测有害：`r.vl/r.vr` 是**上一帧我们下发的轮速**（平台上报口径），刹车时它本来就高，
//   于是这条永远不释放 —— offline_test 的 `gk goal kick reach` 从 y72=89 直接退化成
//   y72=99 / y78=165 / y66=-1（200 帧一次都没碰球）。留着旋钮只作记录，别再开。
TUNABLE(kGkServeBrake, 0.0);
// 门球"从球的场侧绕回球后"的侧移量（cm）：原来写死 22 —— 真机看着就是"每次发球都往旁边冲一大步"。
TUNABLE(kGkServeDetourLat, 14.0);
TUNABLE(kGkCreepBand, 1.0);      // 1 = 只认"真门球角"（新，默认）；0 = 回滚成旧的"离中线 12.3cm 即算"
constexpr double kGkCreepSpeed  = 0.5;    // cm/帧：慢爬上限
constexpr double kGkKickSpotDx  = 14.8;   // cm：门球点离门线
constexpr double kGkKickSpotDy  = 12.3;   // cm：门球点离中线
constexpr double kGkCreepRange  = 12.0;   // cm：离门球点多近才算

bool gk_goal_kick_creep(const GkView &v) {
    if (kGkCreepStill < 0.5) return false;
    if (std::hypot(v.vx, v.vy) >= kGkCreepSpeed) return false;
    if (v.vx * v.gside > 0.0) return false;                       // 朝门滚 → 活球
    // ⚠️ 旧判据是 hypot(ball_goal − 14.8, ||by−90| − 12.3|) < 12 —— 第二项在 |by−90|=12.3 处取
    //   **最小值**，也就是"球离中线 12.3cm 就算门球角"，把门前中路慢球全误判成死门球。
    //   实测：门前 <25cm 时 restart_kick 占 46%，而它 gk_set_play=true ⇒ **整段跳过守门两闸**，
    //   门将就地站成桩子（失球前 30 帧位移 0.0~0.8cm）。
    //   真门球点在门区两侧角上（|by−90| ≈ 12.3），所以新判据要求 |by−90| 落在这个带里。
    const double dy_off = std::fabs(v.by - 90.0);
    if (kGkCreepBand > 0.5 && std::fabs(dy_off - kGkKickSpotDy) > kGkCreepRange) return false;
    return std::hypot(v.ball_goal - kGkKickSpotDx, dy_off - kGkKickSpotDy) <
           kGkCreepRange;
}

GkView gk_view(const WorldModel &wm, int id) {
    const TeamContext &ctx = wm.ctx;
    const RobotState &r = wm.home[id];
    GkView v;
    v.bx = wm.ball.x; v.by = wm.ball.y;
    v.vx = wm.ball.vx; v.vy = wm.ball.vy;
    v.danger = ball_danger_speed(wm);
    v.db = dist(r.x, r.y, v.bx, v.by);
    v.ball_goal = ctx.dist_our_goal(v.bx);
    v.gside = ball_goal_side(ctx, v.bx);
    v.opp_dmin = opp_clear_dist(wm);
    v.ball_still = std::hypot(v.vx, v.vy) < 0.2 || gk_goal_kick_creep(v);
    v.opp_has_ball = false;
    v.y_at_goal = 90.0;
    v.heading_goal = predict_y_at_x(v.bx, v.by, v.vx, v.vy, ctx.our_goal_x(), v.y_at_goal);
    v.on_target = v.heading_goal &&
                  v.y_at_goal >= goal_y_low() && v.y_at_goal <= goal_y_high();
    // 提前站位的瞄准点：朝门中心收缩（kGkShrink）。预测不到就退回跟球的当前 y。
    v.y_aim = 90.0 + kGkShrink * ((v.on_target ? v.y_at_goal : v.by) - 90.0);
    // 真静止（平台把球钉在摆点上）：门球/死球。区别于 ball_still —— 后者还包含"门球角"启发式
    // 命中的**慢滚球**（0.05~0.65cm/帧），那些是"球滚向自家门"的防守场景，绝不能用这里的豁免。
    v.ball_dead = std::hypot(v.vx, v.vy) < kGkDeadStill;
    return v;
}

// 平台这一帧是不是把球重新摆过了？（进球/门球/任意球/点球都会瞬移）
//   ⚠️ 不能用球速判：world_model 对 >30cm/帧 的跳变会把该帧速度**清零**，速度看起来跟"静止门球"一样。
//   所以只能拿 wm.ball_last（平台上一帧的球位）与当前球位做差分。
//   ⚠️ 必须先要 `ball_last.valid`：离线上单测里手搓的 WorldModel 从不填上一帧球位（x=y=0），
//      不设这道门就会把"球心离原点 200cm"当成瞬移，于是每帧清零承诺、门将永远进不了推穿段。
inline bool gk_ball_replaced(const WorldModel &wm) {
    return wm.ball_last.valid &&
           std::hypot(wm.ball.x - wm.ball_last.x, wm.ball.y - wm.ball_last.y) > kGkRestartJump;
}

void gk_update_opp_hold(WorldModel &wm, GkView &v) {
    if (wm.possession == Possession::Opponent) {
        wm.goalie_opp_hold = kGkOppHoldFrames;
    } else if (wm.goalie_opp_hold > 0) {
        --wm.goalie_opp_hold;
    }
    v.opp_has_ball = (wm.goalie_opp_hold > 0);
}

double gk_line_x(const TeamContext &ctx, double depth) {
    return ctx.our_goal_x() + ctx.attack_dir() * depth;
}

void gk_goto(const TeamContext &ctx, RobotState &r, double x, double y,
             motion::TargetMode mode) {
    y = clamp(y, kGkYLo, kGkYHi);
    clamp_goalie_area(ctx, x, y);
    motion::position(r, x, y, mode);
}

inline double gk_across(const RobotState &r, const GkView &v, double dx, double dy) {
    return (r.x - v.bx) * (-dy) + (r.y - v.by) * dx;
}

// 绕球往哪一侧躲：只看球离门中心哪边近，就往另一边绕。
// 绝不能用门将自己的 y 来选边——门将一移动就跨过球的 y 线，选边当帧翻转，目标点横跳几十 cm。
inline double gk_detour_side(double by) { return (by < 90.0) ? 1.0 : -1.0; }

bool gk_aligned(const RobotState &r, const GkView &v, double dx, double dy) {
    double along = (r.x - v.bx) * dx + (r.y - v.by) * dy;
    return (along <= 0.0) && (std::fabs(gk_across(r, v, dx, dy)) <= kGkAlignAcross);
}

bool gk_turn_to(RobotState &r, double dx, double dy) {
    double aim = angle_to(0.0, 0.0, dx, dy);
    if (std::fabs(angle_diff(aim, r.rot)) > kGkClearAlignTol) {
        motion::position_aligned(r, r.x, r.y, aim, 2.0, kGkClearAlignTol);
        return true;
    }
    return false;
}

TUNABLE(kGkPredictBlock, 1.0);  // 0 = 回退到纯"球→门心"直线
inline double gk_block_cy(const GkView &v, double tx) {
    if (kGkPredictBlock > 0.5 && v.on_target) {
        double cy = 90.0;
        if (!predict_y_at_x(v.bx, v.by, v.vx, v.vy, tx, cy))
            cy = v.y_at_goal;                     // 预测不到就退到门线进门点
        cy = 90.0 + kGkShrink * (cy - 90.0);      // 提前站位 → 朝门中心收缩（见 kGkShrink 说明）
        return clamp(cy, kGkBlockYLo, kGkBlockYHi);
    }
    double back = std::max(0.0, v.ball_goal - 12.0);
    return clamp(90.0 + (v.by - 90.0) * (back / std::max(1.0, v.ball_goal)),
                 kGkBlockYLo, kGkBlockYHi);
}

void gk_block_ball_line(const TeamContext &ctx, RobotState &r, const GkView &v) {
    double cx = gk_line_x(ctx, goalie_block_depth(ctx, v.bx, kGkGuardDist,
                                                  kGkBlockMaxOut, kGkBallSideMargin));
    double cy = gk_block_cy(v, cx);
    clamp_goalie_area(ctx, cx, cy);
    motion::position(r, cx, cy);
}

// 对方罚点球识别：平台执行期不报点球态，故改认可观测量——球静止在我方罚球点即当点球守
TUNABLE(kGkPenSpotGuard, 1.0);   // 0 = 回滚（只认 game_state）
constexpr double kGkPenSpotDist  = 39.4;  // cm：罚球点到门线
constexpr double kGkPenSpotTol   = 0.4;   // cm：容差（放宽会误判"活球恰好停在点附近"）
constexpr double kGkPenSpotStill = 0.05;  // cm/帧：点球静止速度上限

bool gk_opp_penalty_spot(const WorldModel &wm, const GkView &v) {
    if (kGkPenSpotGuard < 0.5) return false;
    if (std::hypot(v.vx, v.vy) >= kGkPenSpotStill) return false;
    return std::fabs(v.bx - gk_line_x(wm.ctx, kGkPenSpotDist)) < kGkPenSpotTol &&
           std::fabs(v.by - 90.0) < kGkPenSpotTol;
}

bool gk_rule_penalty(WorldModel &wm, int id, const GkView &v) {
    if (wm.game_state == PM_PenaltyKick_Blue || wm.game_state == PM_PenaltyKick_Yellow ||
        gk_opp_penalty_spot(wm, v)) {
        motion::position(wm.home[id], gk_line_x(wm.ctx, 3.0), 90.0);
        return true;
    }
    return false;
}

double seg_point_dist(double x0, double y0, double x1, double y1, double px, double py) {
    double dx = x1 - x0, dy = y1 - y0;
    double len2 = dx * dx + dy * dy;
    double t = (len2 < 1e-9) ? 0.0 : clamp(((px - x0) * dx + (py - y0) * dy) / len2, 0.0, 1.0);
    return std::hypot(x0 + t * dx - px, y0 + t * dy - py);
}

bool gk_path_hits_ball(const RobotState &r, const GkView &v, double tx, double ty) {
    for (int k = 0; k <= kGkBehindHorizon; k += 2) {
        if (seg_point_dist(r.x, r.y, tx, ty, v.bx + v.vx * k, v.by + v.vy * k) < kGkPathClear)
            return true;
    }
    return false;
}

// 慢球堵门线（2026-10-09）：球已越过门将、但球速慢到我们来得及站到它前面时的走位。
//   旧口径（x 钉死 + 往球的侧方让 kGkHoldPassGap+2）对快球是对的，对慢球就是"站着看球进门"。
//   这里改成主动去堵，按"这条直线会不会蹭到球"分档 —— 车身半径4+球半径2=6cm 就接触，
//   蹭过去 = 把球顶过门线 = 乌龙，所以每一档都要先量净空：
//     ① 人已在球的门侧，或直着去"门线上球的 y"不蹭球 → 去封门线（对准预测进门点）
//     ② 会蹭到 → 先在当前 y 上把 x 越过球（下一帧就变成 ①）
//     ③ 连直着超越都蹭得到 → 绕到球的门侧 + 横向让开 kGkBallRetreatLat
//     ④ 都走不通 → 原地站住
//   ⚠️ 四档没有一档是"朝自家门让开" —— 取证里最致命的就是球慢、门将却在这段时间越走越远。
//   返回 false = 球不慢（或旋钮关了），调用方按"让开"的老口径处理。
bool gk_slow_block(WorldModel &wm, int id, const GkView &v) {
    if (kGkBehindBlockSpeed <= 0.0) return false;
    if (std::hypot(v.vx, v.vy) >= kGkBehindBlockSpeed) return false;
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    const double bx_line = gk_line_x(ctx, kGkBlockLineDepth);
    const double ty = clamp(v.on_target ? v.y_at_goal : v.by, kGkYLo, kGkYHi);
    if ((r.x - v.bx) * v.gside > 0.0 ||
        seg_point_dist(r.x, r.y, bx_line, ty, v.bx, v.by) >= kGkPassClear) {
        motion::position(r, bx_line, ty, motion::TM_PASS);                       // ①
        return true;
    }
    if (seg_point_dist(r.x, r.y, bx_line, r.y, v.bx, v.by) >= kGkPassClear) {
        motion::position(r, bx_line, clamp(r.y, kGkYLo, kGkYHi), motion::TM_PASS);  // ②
        return true;
    }
    double cx = v.bx + v.gside * kGkBallGoalSide;
    double cy = clamp(v.by + ((r.y >= v.by) ? 1.0 : -1.0) * kGkBallRetreatLat, kGkYLo, kGkYHi);
    clamp_goalie_area(ctx, cx, cy);
    if (seg_point_dist(r.x, r.y, cx, cy, v.bx, v.by) >= kGkPassClear) {
        motion::position(r, cx, cy, motion::TM_PASS);                            // ③
        return true;
    }
    motion::stop(r);                                                             // ④
    return true;
}

bool gk_rule_ball_behind(WorldModel &wm, int id, const GkView &v) {
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    // ⚠️ 真静止的球（门球/死球，平台钉住）**没有"滚到门将身后"这回事**：它是我们要开出去的球，
    //   交给 gk_rule_restart_kick。真机取证（replay_trace 回放 20261010193949 帧 2729~2741）：
    //   门球摆点离门线 14.8cm，门将一走到球后（x<205.2）这条就命中 → 命令它"钉住 x、横移 11cm"，
    //   而运动学在 rot≈-162° 下把它变成**全速倒车**（vl=-150 vr=-126），门将被甩到 x=192.9，
    //   于是"永远对不准、一直在旁边冲"，89 帧球一动不动。
    if (kTkDeadSkip > 0.5 && v.ball_dead) return false;
    if (!(v.ball_goal < ctx.dist_our_goal(r.x) - kGkBallBehindMargin)) return false;
    double side = (r.y >= v.by) ? 1.0 : -1.0;                   // 往自己那侧绕，少掉头
    double gx = v.bx + v.gside * kGkBallGoalSide;
    double gy = v.by + side * kGkBallRetreatLat;
    double cx = gx, cy = clamp(gy, kGkYLo, kGkYHi);
    clamp_goalie_area(ctx, cx, cy);
    if (v.ball_goal < kGkNoRoom || gk_path_hits_ball(r, v, cx, cy)) {
        // 门前死守区内：原实现在这里"站到球的场侧 10cm 等"（TM_STOP），实测就是它把门将钉在
        //   离门 23cm 处连续 55 帧 0 位移，而球以 0.27cm/帧 从门将身后滚进网（13 个失球里约 10 个）。
        //   改成主动绕：先全速横移到让开球的一侧（x 不动，绝不朝门推球），下一帧直线通了再插回门线。
        if (v.ball_goal < kGkLastDitch) {
            // 慢球：有时间站到它前面 → 主动去堵（见 gk_slow_block）。旧口径"钉住 x 只横向让开"
            //   只留给快球 —— 快球让开是对的（追不上，硬碰就是把球推过门线）。
            if (gk_slow_block(wm, id, v)) return true;
            double s2 = (r.y >= v.by) ? 1.0 : -1.0;
            motion::position(r, r.x,
                             clamp(v.by + s2 * (kGkHoldPassGap + 2.0), kGkYLo, kGkYHi),
                             motion::TM_PASS);
            return true;
        }
        double tx = v.bx - v.gside * kGkBehindBackOff, ty = gy;  // 球的场侧，y 只夹罚球区
        clamp_goalie_area(ctx, tx, ty);
        motion::position(r, tx, ty, motion::TM_STOP);
        return true;
    }
    motion::position(r, cx, cy, motion::TM_PASS);
    return true;
}

bool gk_rule_line_block(WorldModel &wm, int id, const GkView &v) {
    double tx = 0.0, ty = 0.0;
    if (!gk_line_block_point(wm, id, tx, ty)) return false;
    RobotState &r = wm.home[id];
    if (std::fabs(v.by - 90.0) < 20.0 && gk_path_hits_ball(r, v, tx, ty) &&
        dist(r.x, r.y, tx, ty) > 4.0) return false;
    double dd = std::hypot(tx - r.x, ty - r.y);
    motion::position(r, tx, ty, (dd > 15.0) ? motion::TM_PASS : motion::TM_STOP);
    return true;
}

bool gk_rule_no_push(WorldModel &wm, int id, const GkView &v) {
    if (robot_touch_allowed(wm, id)) return false;
    motion::position(wm.home[id], gk_line_x(wm.ctx, kGkGuardDist),
                     clamp(v.by, kGkTrackYLo, kGkTrackYHi), motion::TM_STOP);
    return true;
}

// 门前静止球不许走"侧向让开"（2026-10-11，用户投诉"门将发球一直对不准、每次都往旁边冲"）
//   真机取证（replay_trace 逐帧回放 20261010193949）：
//     我家门球摆点 = (205.2, 77.7)，离门线只有 **14.8cm**；门将正常站在 (214.8, 89.9)。
//     `gk_rule_restart_kick` 让它去"球后 8cm"的后撤点 (213.2, 77.7) —— 一切正常；
//     但只要它一离开门线过了 x<=212，`gk_side_step_point` 的判据
//     `(bx−r.x)·gside <= kGkBehindMargin` 就成立（它以为"球已越过门将"），
//     于是命令它去 `(球后 9.4cm, 球侧 ±38cm 夹到 106)` —— **横向 28cm 的大横跳**；
//     同时这条不是"出球模式"，闸零立刻接管，把它改成**全速倒车**（实测 vl=-150 vr=-126）。
//     结果：门将 x 在 [192.9, 225.4] 来回冲、89 帧球一动不动，最后把球顶向自家门。
//   静止的球没有"滚过门将"这回事 —— 门球/死球一律交给 gk_rule_restart_kick 穿球推出去。
TUNABLE(kGkStillNoStep, 1.0);    // 0 = 回滚（静止球也允许侧向让开）
bool gk_rule_side_step(WorldModel &wm, int id, const GkView &v) {
    if (kGkStillNoStep > 0.5 && v.ball_still) return false;
    double tx = 0.0, ty = 0.0;
    if (!gk_side_step_point(wm, id, tx, ty)) return false;
    motion::position(wm.home[id], tx, ty, motion::TM_PASS);
    return true;
}

// 对方持球：绝不前出，锁门前浅位跟球 y；已带到门口且门将贴球时上前封球-门连线
TUNABLE(kGkOppLockEnable, 1.0);  // 1=开（球远时也封角），0=锁死门线前 10cm
TUNABLE(kGkOppLockDepth, 30.0);  // cm：封角深度上限
bool gk_rule_opp_ball_lock(WorldModel &wm, int id, const GkView &v) {
    if (!v.opp_has_ball) return false;
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    if (v.ball_goal < 30.0 && v.db < 25.0) {
        gk_block_ball_line(ctx, r, v);
    } else if (kGkOppLockEnable < 0.5) {
        motion::position(r, gk_line_x(ctx, kGkGuardDist), clamp(v.by, kGkTrackYLo, kGkTrackYHi));
    } else {
        double depth = std::min(kGkOppLockDepth, std::max(kGkGuardDist, v.ball_goal - 12.0));
        double cx = gk_line_x(ctx, depth);
        double cy = gk_block_cy(v, cx);
        clamp_goalie_area(ctx, cx, cy);
        motion::position(r, cx, cy);
    }
    return true;
}

TUNABLE(kGkKickReach, 1.0);  // 0 = 回滚（不修正推球方向）
void gk_reachable_dir(const TeamContext &ctx, const GkView &v, double &dx, double &dy) {
    if (kGkKickReach < 0.5) return;
    auto ok = [&](double ddy) {
        double py = v.by - ddy * kGkPushDist;
        return py >= kGkKickBandLo && py <= kGkKickBandHi;
    };
    if (ok(dy)) return;
    if (ok(-dy)) { dy = -dy; return; }
    dy = clamp((v.by - clamp(v.by, kGkKickBandLo + 1.0, kGkKickBandHi - 1.0)) / kGkPushDist, -0.8, 0.8);
    dx = ctx.attack_dir() * std::sqrt(1.0 - dy * dy);
}

// 退回就位段：远处带速滑（TM_PASS）、进 kGkServeGlide 之内就切 TM_STOP 精调停靠。
//   两者按距离分工、不重叠（TM_PASS ≈150cm/s，4cm 内刹不住）。原来这三处是重复代码，收成一个函数。
void gk_back_to_prep(const TeamContext &ctx, RobotState &r, const GkView &v, bool contested,
                     bool clamped, double px, double pyc, double dx, double dy) {
    double to_prep = std::hypot(r.x - px, r.y - pyc);
    bool gliding = !contested && !clamped && kGkPrepPass > 0.5 &&
                   to_prep > kGkServeGlide &&
                   std::fabs(gk_across(r, v, dx, dy)) <= kGkAlignAcross * 2.0;
    gk_goto(ctx, r, px, pyc, gliding ? motion::TM_PASS : motion::TM_STOP);
}

bool gk_rule_restart_kick(WorldModel &wm, int id, const GkView &v) {
    if (!(v.ball_goal < 45.0 && v.ball_still)) return false;
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    bool contested = v.opp_dmin < 40.0;
    double dx = ctx.attack_dir(), dy = 0.0;
    if (!contested) gk_restart_direction(wm, id, v.bx, v.by, dx, dy);
    gk_reachable_dir(ctx, v, dx, dy);
    // 发球拆成两段，中间用一个承诺位隔开：
    //   第 0 段：走到"球后 kGkPushDist"这个准备点，并且到位时顺带把机头对准出球方向；
    //   第 1 段：承诺之后只干一件事——朝球后 45cm 一口气推穿，绝不再回头重判到位/横向/距离。
    // 为什么必须承诺：推穿这个动作本身会把门将带离准备点、也会让横向偏差变大，
    // 一旦拿这些量当"要不要继续推"的条件，门将就会在"推穿"和"绕回球后"之间每 3 帧翻一次，
    // 结果原地左右蹭（实测旧逻辑要 130 帧 = 3.25 秒才把球弄出去）。
    double px = v.bx - dx * kGkPushDist, py = v.by - dy * kGkPushDist;
    // 门将活动范围 y∈[kGkYLo,kGkYHi]，球贴着下沿时理论后撤点够不到，只能取夹住后的点
    double pyc = clamp(py, kGkYLo, kGkYHi);
    bool clamped = (pyc != py);
    double ax = dx, ay = dy;                 // 实际推球方向
    if (clamped) {
        // 够不到理论后撤点 → 门将只能站在球的斜后上方，这时若还朝出球方向推，
        // 会从球旁边擦过去把球拨错方向 → 改成"机头对准球、沿门将→球这条线推出去"。
        double bdx = v.bx - r.x, bdy = v.by - r.y, n = std::hypot(bdx, bdy);
        if (n > 1e-6) { ax = bdx / n; ay = bdy / n; }
    }
    if (wm.goalie_serve_phase == 0) {
        // 球贴门线（<15cm）且门将在球外侧：直奔球后会穿球把球顶进自家门 → 先横移到球侧 22cm
        // 贴门线且门将在球的场侧：直奔球后会穿球把球顶进自家门。但只在【横向让不开球】
        // 时才需要横摆；横向已经让开就全速直插准备点。横摆本身也用 TM_PASS（目标离球 22cm）。
        if (v.ball_goal < 15.0 && (r.x - v.bx) * v.gside < 0.0 &&
            std::fabs(r.y - v.by) <= kGkHoldPassGap) {
            // 侧移量 22 → 14cm（2026-10-11）：22cm 在真机录像里就是"门将每次发球都往旁边冲一大步"
            //   （replay_trace 20261010193949 帧 2727~2746 共 20 帧被送到 (195.2,99.7)）。
            //   车身半宽 4 + 球 2 = 6cm 就够擦身而过，14cm 留 8cm 富余；126 种门球摆位回归逐值不变。
            gk_goto(ctx, r, v.bx - v.gside * 10.0, v.by + gk_detour_side(v.by) * kGkServeDetourLat,
                    motion::TM_PASS);
            return true;
        }
        // 三个动作各用一个只管一件事的原语，别混：
        //   position_aligned / arrive_facing 都想"又要走位又要转正"，两个目标互相扯，
        //   实测 position_aligned 59 帧只挪 4.5cm、arrive_facing 到位后转过头转不停。
        // 入口要求"停在后撤点附近且已对准"：后撤点是精确停靠点，因为从斜后方推球会把球拨偏。
        // 还要求"真停稳了"（见 kGkServeSettle）：带着速度进承诺期，原地转正那十几帧会靠惯性滑出去 10cm。
        bool settled = kGkServeSettle <= 0.0 || wm.goalie_moved <= kGkServeSettle;
        bool at_ready = std::hypot(r.x - px, r.y - pyc) <= kGkServeReady && settled &&
                        (clamped || kGkPrepNeedAlign < 0.5 || gk_aligned(r, v, dx, dy));
        if (!at_ready) {
            // 远距离接近要带速滑（别提前刹车，白磨帧数），但滑行停不准，
            // 所以进到 kGkServeGlide 之内就切回 TM_STOP 精调停靠 —— 两者按距离分工，不重叠。
            gk_back_to_prep(ctx, r, v, contested, clamped, px, pyc, dx, dy);
            return true;
        }
        wm.goalie_serve_phase = 1;
    }
    // ⚠️ 承诺期的第三个解锁条件（2026-10-11 二次修复，见 kGkServeRealign 说明）：
    //   球还一动不动（我们根本没碰到它）却横向偏出 > 4.5cm → 现在这条直线必然擦球边，
    //   先退回就位段把横向摆正。球一被碰到就会动，`ball_dead` 立刻变假，所以推穿途中不会被它拽回来。
    if (kGkServeRealign > 0.5 && !clamped && v.ball_dead &&
        std::fabs(gk_across(r, v, ax, ay)) > kGkAlignAcross * 1.5) {
        wm.goalie_serve_phase = 0;
        gk_back_to_prep(ctx, r, v, contested, clamped, px, pyc, dx, dy);
        return true;
    }
    // 承诺期唯一的解锁条件：真的冲到球前面去了（说明这一下没擦到球，推空了）
    // ——只有推空才会满足，正常推穿时球一直被顶在门将前面，所以不会把状态翻回去。
    if ((r.x - v.bx) * ax + (r.y - v.by) * ay > kGkServeReady) {
        wm.goalie_serve_phase = 0;
    }
    // 承诺期的**第二个**解锁条件（2026-10-11）：不是"站在球后对准"就退回就位段。
    //   `goalie_serve_phase` 是跨帧粘住的：上一轮门球留下的 ph=1，会让门将在离后撤点
    //   12.3cm、横向差 10.5cm 的地方就直接 TM_PASS 满速穿球 → 冲过球 → 触发绕球/闸零 →
    //   回来再冲…… 实测门将 x 在 [192.9,225.4] 之间来回冲 89 帧、球一动不动。
    //   （`kGkPrepNeedAlign=0` 只管"入口不要求对准"，但**承诺期内**一旦横向偏出就必须重摆。）
    if (kGkServeKeep > 0.5 && !clamped &&
        (std::hypot(r.x - px, r.y - pyc) > kGkServeKeepDist ||
         std::fabs(gk_across(r, v, ax, ay)) > kGkAlignAcross * 3.0)) {
        wm.goalie_serve_phase = 0;
    }
    // ⚠️ 进承诺位前先把**残余速度**刹掉（2026-10-11）：门将从后撤点往门侧**倒车**过来时带着动量，
    //   直接切"原地转正"只是命令 vl=-14/vr=+14（纯转、零平动），物理里的倒车动量还在，
    //   实测它靠惯性从 x=214 滑到 220（后撤点 212.7）→ 被"离后撤点 >8cm"解锁踢回就位段 →
    //   再倒回来 → **永远进不了推穿**（replay_trace 20261010193949 帧 2759~2770）。
    //   r.vl/r.vr 此刻仍是平台上报的轮速（本帧还没被我们改写），拿它当"还没停稳"的判据。
    if (kGkServeBrake > 0.0 && std::fabs(r.vl) + std::fabs(r.vr) > kGkServeBrake) {
        gk_goto(ctx, r, px, pyc, motion::TM_STOP);
        return true;
    }
    if (kGkAimBall > 0.5 && !clamped) {
        double bdx = v.bx - r.x, bdy = v.by - r.y, n = std::hypot(bdx, bdy);
        if (n > 1e-6 && (bdx * ax + bdy * ay) / n >= std::cos(kGkAimBallMaxDeg * SIMURO5_PI / 180.0)) {
            ax = bdx / n; ay = bdy / n;
        }
    }
    if (gk_turn_to(r, ax, ay)) return true;    // 只管原地转正
    // ═══ 开哨后全部防乌龙保险就这两条，其余限制一律取消 ═══
    //   ① 门将必须在球的门侧：否则朝球推进就是把球顶进自家门；
    //   ② 出球方向必须有朝场外分量：否则等于主动往自家门推。
    //   任一条不成立 → 立刻退回就位段全速重摆（TM_PASS），本帧绝不推球。
    if ((r.x - v.bx) * v.gside <= 0.0 || ax * ctx.attack_dir() <= 0.0) {
        wm.goalie_serve_phase = 0;
        // ⚠️ 这里原来**无条件用 TM_PASS**（不减速）→ 满速 150cm/s 冲向后撤点，而
        //   `at_ready` 要求停在 3cm 内 —— 刹车距离要 28cm，于是永远进不了承诺期：
        //   实测门将 x 在 [192.9, 225.4] 之间来回冲（replay_trace 20261010193949 帧 2726~2790）。
        //   改成和就位段同一套分工：远处带速滑、进 kGkServeGlide 就切 TM_STOP 精调（与上面的注释一致）。
        gk_back_to_prep(ctx, r, v, contested, clamped, px, pyc, dx, dy);
        return true;
    }
    if (gk_turn_to(r, ax, ay)) return true;
    gk_goto(ctx, r, v.bx + ax * kGkKickThrough, v.by + ay * kGkKickThrough, motion::TM_PASS);
    return true;
}

bool gk_rule_opp_kick_line(WorldModel &wm, int id, const GkView &) {
    double y_pred = 90.0;
    if (!opp_kick_target_y(wm, y_pred)) return false;
    motion::position(wm.home[id], gk_line_x(wm.ctx, 3.0), clamp(y_pred, kGkYLo, kGkYHi),
                     motion::TM_PASS);
    return true;
}

bool gk_rule_press_door(WorldModel &wm, int id, const GkView &v) {
    if (!(v.ball_goal < 45.0 && v.opp_dmin < 25.0)) return false;
    gk_block_ball_line(wm.ctx, wm.home[id], v);
    return true;
}

TUNABLE(kGkHoldSlide, 1.0);              // 0 = 回滚（贴球一律站定）
constexpr double kGkHoldSlideDanger = 0.1;   // cm/帧：朝门速度高于此才算在滚进门
TUNABLE(kGkOwnGoalPad, 6.0);     // cm：防乌龙余量。球比门将更靠自家门（含此余量）时，门将这一帧
                                 // 的动作整条换成安全动作，绝不允许朝自家门推进。
TUNABLE(kGkFaceTol, 12.0);       // 度：守门时允许的机头偏差。超过就拧机头，把门将拧到直着对准球。
TUNABLE(kGkSpinGain, 0.8);       // 轮速差 / 度：盯球拧机头的比例增益
TUNABLE(kGkSpinMax, 60.0);       // 轮速差上限：盯球拧机头最多叠这么多（不动位移）
TUNABLE(kGkHoldCover, 2.0);      // cm：门将中心离进门点小于此才算"已挡住"。
                                 // 原 5.0 太松：实测丢球时门将横向只偏 4.6cm 就自认挡住了、原地不动，
                                 // 球从它旁边滚进网。收紧后偏 2~5cm 会走下面的滑移分支贴到球的进门点。
// 门前死守区：球离门线多近就进"死守"——此区内门将唯一正解是回到门线并让开球，
//   任何"站到球外侧等着推"的预备位一律作废（20cm 内、十几 cm/s 的慢球根本不用推，站进门里就是挡）。

bool gk_rule_goal_line_hold(WorldModel &wm, int id, const GkView &v) {
    if (!(v.ball_goal < 15.0 && v.db < kGkLastDitchGuard)) return false;
    TRACE_MARK(wm.home[id]);
    RobotState &r = wm.home[id];
    // 门将在球的"场侧"= 球已经跑到门将身后了。原地冻住等于把门让开：
    //   实测丢球形状——球 x=2.6 贴着门线从 y=16 滚到 y=72，门将连续 30 帧 0 位移。
    // 但"退回门线"这一步本身是朝自家门驱动，而此分支下场侧意味着门将离球常在 6cm 内
    // （已在接触范围）—— 朝门驱动就是把球推进自家门（乌龙）。所以分两种走法：
    //   ① 横向已让开球（> kGkHoldPassGap）→ 可以安全退回门线并对到球的 y；
    //   ② 横向贴着球 → 只横向对 y，x 原地不动，绝不朝门动一下。
    if ((r.x - v.bx) * v.gside <= 0.0) {
        double ty = clamp(v.by, kGkYLo, kGkYHi);
        if (std::fabs(r.y - v.by) > kGkHoldPassGap) {
            motion::position(r, gk_line_x(wm.ctx, 3.0), ty, motion::TM_STOP);
        } else {
            motion::position(r, r.x, ty, motion::TM_STOP);
        }
        return true;
    }
    if (kGkHoldSlide > 0.5 && v.danger > kGkHoldSlideDanger && v.on_target &&
        (r.x - v.bx) * v.gside > 0.0 && std::fabs(r.y - v.y_at_goal) > kGkHoldCover) {
        motion::position(r, r.x, clamp(v.y_at_goal, kGkYLo, kGkYHi), motion::TM_STOP);
        return true;
    }
    r.vl = 0.0;
    r.vr = 0.0;
    return true;
}

bool gk_rule_clear(WorldModel &wm, int id, const GkView &v) {
    if (!(v.db < kGkClearDist)) return false;
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    double dx = 0.0, dy = 0.0;
    if (v.on_target) {
        dx = ctx.attack_dir();
    } else {
        gk_clear_direction(wm, id, v.bx, v.by, dx, dy);
    }
    double len = std::hypot(dx, dy);
    if (len < 1e-6) { dx = ctx.opp_goal_x() - v.bx; dy = 0.0; len = std::hypot(dx, dy); }
    if (len < 1e-6) { dx = 0.0; dy = 1.0; len = 1.0; }
    dx /= len; dy /= len;

    double tx, ty;
    motion::TargetMode mode;
    if (v.ball_goal < ctx.dist_our_goal(r.x)) {
        // ① 球在门将的**场侧**（门将已经在球与门之间）——只该做一件事：
        //    把 y 对准球，站到球的横移线上去挡；x 保持在球后朝场外（推球方向）。
        // 旧写法 ty = clamp(by + side*30, 74, 106) 是"往球外侧让 30cm"：
        //   球 y=80 时 side=+1 ⇒ 目标 110 ⇒ clamp 成 106 —— 球在 80、门将却被叫去 106，
        //   **门将朝远离球的方向走**；球跨过 y=90 时目标还在 74/106 之间横跳 32cm。
        //   实测 sim_trace --gk-probe：门前慢横移的 |目标y−球y| 均值 11.36cm ——
        //   这不是"追得慢"，是"追的目标错了"。
        // 防乌龙不靠这个 y 偏移：推球方向由 tx（球后 kGkPushDist、朝场外）保证。
        tx = v.bx + v.gside * kGkPushDist;
        ty = (kGkTkCross > 0.5) ? clamp(v.by, kGkYLo, kGkYHi)     // 新：直接对到球的横移线
                                : clamp(v.by + gk_detour_side(v.by) * (v.ball_still ? kGkLateral : 12.0),
                                        kGkYLo, kGkYHi);           // 旧：往球外侧让（回滚口径）
        mode = motion::TM_STOP;
    } else if (kGkTkCross > 0.5) {
        // ② 球在门将的**门侧**（球比门将更靠自家门）——最危险的一种：
        //    门将此时在球的场侧，**任何朝场外的平移都会先穿过球**，把球铲进自家门。
        // 旧写法 tx = bx + dx*45（dx 是"解围方向"）实测就是**朝自家门线**：
        //   球(203,80)、门将(209,88) ⇒ dir=(-0.09,+1.00) ⇒ 目标 (199.1,125) → y 夹到 107.5，
        //   也就是"门前横移时目标 y 为什么是 107.5"的答案，x 还朝门口压了 19cm。
        // 现在把两件事拆开：x 只做防乌龙（留在门线前 kGkGuardDist，不朝门口压）；
        //   y 去对球的横移线（门框内）—— 本帧只横移不前进 ⇒ 不穿球铲进自家门，也不背对球。
        tx = gk_line_x(ctx, kGkGuardDist);
        ty = clamp(v.by, kGkYLo, kGkYHi);
        mode = motion::TM_STOP;
    } else {
        tx = v.bx + dx * kGkKickThrough;                  // 回滚口径：球前 kGkKickThrough
        ty = v.by + dy * kGkKickThrough;
        mode = motion::TM_PASS;
    }
    clamp_goalie_area(ctx, tx, ty);
    motion::position(r, tx, ty, mode);
    return true;
}

bool gk_rule_wall_ball(WorldModel &wm, int id, const GkView &v) {
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    if (v.ball_goal < 160.0 && (v.by < 30.0 || v.by > 150.0) &&
        v.vx * (ctx.our_goal_x() - v.bx) > 0.0 && v.on_target) {
        double ty = clamp(v.y_at_goal, kGkYLo, kGkYHi);
        double tx = gk_line_x(ctx, 3.0);
        double dd = std::hypot(tx - r.x, ty - r.y);
        motion::position(r, tx, ty, (dd > 15.0) ? motion::TM_PASS : motion::TM_STOP);
        return true;
    }
    return false;
}

bool gk_rule_shot_block(WorldModel &wm, int id, const GkView &v) {
    if (!v.on_target) return false;
    const TeamContext &ctx = wm.ctx;
    int opp_in_box = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (in_penalty_area(ctx, wm.opp[i].x, wm.opp[i].y))
            ++opp_in_box;
    }
    double frac = clamp((v.danger - kGkMinSpeed) / (kGkFastShotSpeed - kGkMinSpeed), 0.0, 1.0);
    double depth = kGkGuardDist + frac * (80.0 - kGkGuardDist);
    depth = std::max(kGkGuardDist, depth - opp_in_box * kGkOppPullback);
    // 前压深度上限：不越过对方最前插球员（留 kGkOppFrontPad），下限锁 kGkGuardDist
    double opp_front = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i)
        opp_front = std::min(opp_front, ctx.dist_our_goal(wm.opp[i].x));
    depth = std::max(kGkGuardDist, std::min(depth, opp_front - kGkOppFrontPad));
    // 球越近，门将越必须缩回门线；站位深度不得比球离门线的距离更靠外超过 kGkBallSideMargin。
    // 旧写法在 15cm 处硬切换（>15 用 depth、<=15 用 3.0）：球在 16cm 时门将冲到 30cm 外
    // （实测最远 34.3cm），球一到 14.9cm 又要求它瞬间回 3cm —— 物理上回不来，于是丢球。
    double out_depth = std::min(depth, std::max(3.0, v.ball_goal - kGkBallSideMargin));
    double out_x = gk_line_x(ctx, out_depth);
    double iy = 90.0;
    if (!predict_y_at_x(v.bx, v.by, v.vx, v.vy, out_x, iy)) {
        if (std::fabs(v.bx - ctx.our_goal_x()) > 1e-6) {
            double t = (out_x - ctx.our_goal_x()) / (v.bx - ctx.our_goal_x());
            t = std::max(0.0, std::min(1.0, t));
            iy = 90.0 + t * (v.by - 90.0);
        }
    }
    if (std::fabs(v.y_at_goal - 90.0) > std::fabs(iy - 90.0)) iy = v.y_at_goal;
    clamp_goalie_area(ctx, out_x, iy);
    motion::position(wm.home[id], out_x, iy);
    return true;
}

bool gk_rule_default(WorldModel &wm, int id, const GkView &v) {
    // 2026-10-10：横向改用「预测进门点」而不是裸跟球的当前 y。
    //   真机 8 场 20 个失球的数字：门将横向误差中位 8cm、越线球速中位 80cm/s（0.25s 走 20cm）、
    //   门将离球最近中位 7cm —— 而车身 4 + 球 2 = 6cm 就接触 ⇒ 它**只差几厘米**。
    //   在 80cm/s 下追"球现在在哪"必然滞后 8cm 量级；改成瞄"球会从门的哪个 y 进"，
    //   并且**朝门中心收缩**（`v.y_aim` = 90 + kGkShrink×(预测−90)，真机数据解出 k≈0.7：
    //   远端预测差 8~13cm，硬追单个预测点不如"信它七成 + 门中心三成"，<=6cm 命中率 39.9%→48.8%）。
    motion::position(wm.home[id], gk_line_x(wm.ctx, v.ball_goal < 15.0 ? 3.0 : kGkGuardDist),
                     clamp(v.y_aim, kGkYLo, kGkYHi));
    return true;
}

using GkRule = bool (*)(WorldModel &, int, const GkView &);

const GkRule kGoalieRules[] = {
    gk_rule_no_push,        // 死球期/球在角区：不触球
    gk_rule_side_step,      // 门口球已越过门将：先侧向让开
    gk_rule_opp_ball_lock,  // 对方持球：锁门前浅位（门口贴球时上前封角）
    gk_rule_restart_kick,   // 门前静止球：穿球推出
    gk_rule_opp_kick_line,  // 对手准备出脚：按机头方向提前封线
    gk_rule_press_door,     // 门前对手贴球：封球-门连线
    gk_rule_goal_line_hold, // 贴门线+贴球：站定防乌龙
    gk_rule_clear,          // 球在脚下：解围
    gk_rule_wall_ball,      // 贴墙朝门滚：提前到门线落点
    gk_rule_shot_block,     // 会进门的球：动态前压封角
    gk_rule_default,        // 无威胁：门前跟球 y
};

// 诊断：记录门将每帧命中规则到 C:\Strategy\goalie_trace_v2.csv（CMake 选项 HNNU_TRACE，默认 OFF）
#if defined(SIMURO5_HNNU_TRACE) || defined(SIMURO5_GK_PROBE)
const char *const kGoalieRuleNames[] = {
    "no_push", "side_step", "opp_ball_lock", "restart_kick",
    "opp_kick_line", "press_door", "goal_line_hold", "clear", "wall_ball",
    "shot_block", "default",
};
#endif

#ifdef SIMURO5_HNNU_TRACE
// ⚠️ 2026-10-13：换新文件 `goalie_trace_v2.csv`（带表头），并新增**当帧最终目标点 `tgt_x/tgt_y`**。
//   为什么必须有这一列：真机两场蓝位（20261013143755 / 145057）实测 ——
//     · 预测落点误差中位 **0.4cm**、≤6cm 命中 75%（预测没问题）
//     · 门将真实站位离真实越线点中位 **16.5cm**、≤6cm 只有 12%（门将不在该在的地方）
//   没有 `tgt_*` 就分不清是"目标算错"还是"门将没走到"。记录点也从"规则表之后"挪到
//   **守门两闸之后**（见 `run_goalie` 末尾），所以打印出来的就是这一帧真正下发的目标点。
static void gk_trace(const char *rule, const WorldModel &wm, int id, const GkView &v) {
    static FILE *fp = nullptr;
    static long frame = 0;
    if (!fp) {
        fp = std::fopen("C:\\Strategy\\goalie_trace_v2.csv", "a");
        if (fp)
            std::fprintf(fp, "frame,rule,ball_x,ball_y,gk_x,gk_y,tgt_x,tgt_y,ball_goal,"
                             "db,opp_dmin,opp_has_ball,y_at_goal,on_target,y_aim,danger,"
                             "gk_moved,ball_vx,ball_vy\n");
    }
    if (!fp) return;
    ++frame;
    const RobotState &gr = wm.home[id];
    std::fprintf(fp, "%ld,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                 frame, rule, wm.ball.x, wm.ball.y, gr.x, gr.y, gr.tgt_x, gr.tgt_y,
                 v.ball_goal, v.db, v.opp_dmin, (int)v.opp_has_ball, v.y_at_goal,
                 (int)v.on_target, v.y_aim, v.danger, wm.goalie_moved,
                 wm.ball.vx, wm.ball.vy);
    if ((frame % 40) == 0) std::fflush(fp);
}

#define GK_HIT(lit) hit = (lit)
#elif defined(SIMURO5_GK_PROBE)
#define GK_HIT(lit) g_gk_rule_probe = (lit)
#else
#define GK_HIT(lit) ((void)0)
#endif  // SIMURO5_HNNU_TRACE

}  // anonymous namespace

#ifdef SIMURO5_GK_PROBE
const char *g_gk_rule_probe = "none";
#endif

void run_goalie(WorldModel &wm, int id) {
    RobotState &gr = wm.home[id];
    bool gk_set_play = false;
    {   // ═══ 决策段：整段保持旧驱动律 ⇒ 目标点/规则命中/闸门判定与改前逐位一致 ═══
    motion::LegacyDriveScope legacy_drive;
    GkView v = gk_view(wm, id);
    // 平台重新摆球（进球/门球/任意球/点球后的复位）= 上一轮发球那套承诺全部作废。
    //   不清零的话门将会"人还在门口、离球 12cm"就直接进承诺期满速冲 —— 擦球边、冲过头、
    //   被 ball_behind 判成"球在身后"再全速倒车，来回冲球却一动不动。见 kGkRestartReset 说明。
    // 门将自己的真实帧间位移：上一帧位置由这里自己记（平台 vl/vr 只是上一帧命令，不是实际速度）。
    //   单帧位移 > kGkTeleportJump = 被平台摆回门口，同样算"重新开球"（球可能只被挪了十几 cm，见第 76 轮说明）。
    bool gk_teleported = false;
    if (wm.goalie_last_id == id) {
        double moved = std::hypot(gr.x - wm.goalie_last_x, gr.y - wm.goalie_last_y);
        gk_teleported = moved > kGkTeleportJump;
        wm.goalie_moved = gk_teleported ? 0.0 : moved;
    } else {
        wm.goalie_moved = 0.0;
    }
    wm.goalie_last_id = id; wm.goalie_last_x = gr.x; wm.goalie_last_y = gr.y;
    if (kGkRestartReset > 0.5 &&
        (gk_ball_replaced(wm) || (kGkTeleportReset > 0.5 && gk_teleported)))
        wm.goalie_serve_phase = 0;
#ifdef SIMURO5_GK_PROBE
    // 把本帧 GkView 的关键量交给探针（sim_trace --gk-probe 打印用）
    motion::g_gk_probe.y_at_goal    = v.y_at_goal;
    motion::g_gk_probe.ball_goal    = v.ball_goal;
    motion::g_gk_probe.db           = v.db;
    motion::g_gk_probe.opp_dmin     = v.opp_dmin;
    motion::g_gk_probe.heading_goal = v.heading_goal ? 1 : 0;
    motion::g_gk_probe.on_target    = v.on_target ? 1 : 0;
#endif
#ifdef SIMURO5_GK_PROBE
    g_gk_rule_probe = "none";
    motion::g_gk_probe.rule_idx = -1;        // -1 = 前置规则(penalty/line_block/ball_behind)或未命中
    motion::g_gk_probe.begin();              // 整段（规则 + 两闸）记下最后一个目标点
#endif
#ifdef SIMURO5_HNNU_TRACE
    const char *hit = "none";
#endif
    // 出球模式 = 门球推穿(restart_kick) / 点球守(penalty) / 死球(no_push)。
    // 这三种动作本身就要求"机头对准出球方向 + 以速度推穿球"，下面的守门两闸会把它们打坏
    // （离线测试已证：门球 200 帧碰不到球、横向 13cm 时高速直冲、点球守会离位），所以整段跳过。
    // （`gk_set_play` 提到函数开头声明了，因为驱动段要用它）
    if (gk_rule_penalty(wm, id, v)) { GK_HIT("penalty"); gk_set_play = true; }
    else if (gk_rule_line_block(wm, id, v)) { GK_HIT("line_block"); }
    else if (gk_rule_ball_behind(wm, id, v)) { GK_HIT("ball_behind"); }
    else {
        gk_update_opp_hold(wm, v);
        for (int i = 0; i < (int)(sizeof(kGoalieRules) / sizeof(kGoalieRules[0])); ++i) {
            if (kGoalieRules[i](wm, id, v)) {
                GK_HIT(kGoalieRuleNames[i]);
#ifdef SIMURO5_GK_PROBE
                motion::g_gk_probe.rule_idx = i;
#endif
                gk_set_play = (i == 0) || (i == 3);   // 0=no_push(死球)  3=restart_kick(门球)
                break;
            }
        }
    }
    // ⚠️ 诊断记录不放这里：此时两闸还没跑，`gr.tgt_*` 只是规则表的目标。
    //   挪到函数末尾（两闸之后），才拿得到这一帧真正下发的 `tgt_x/tgt_y`。

    // ═══════ 守门两闸：只对守门模式生效，出球模式整段跳过 ═══════
    const TeamContext &gctx = wm.ctx;
    // 规则主动"站定不动"（vl=vr=0）时不加闸：那是规则按门前几何做的防乌龙决定
    // （球就贴在自己脚下时，连原地转身都可能把它蹭进门），闸门不许把它解冻。
    bool gk_frozen = (gr.vl == 0.0 && gr.vr == 0.0);
    if (!gk_set_play) {
        // 闸零（门前死守，最高优先，连"站定不动"也要让位）：球进死守区后，门将唯一允许的动作
        //   是"让开球 + 回门线"。实测丢球形状：球在门前 14.7cm，门将在球的场侧 23~28cm 处站定，
        //   球以 0.27cm/帧 滚进网，55 帧里门将一步没走。这里把这条死锁强行拆掉。
        //   横向贴着球（|Δy| <= kGkHoldPassGap）时只横着让开（x 钉死，绝不朝门推球）；
        //   让开后立刻全速插回门线并对到球的 y —— 此时直线与球有 >=9cm 横向间隔，蹭不到。
        //   但只接管"门将已经站到球的场侧（死锁位）"或"离门线太远"这两种情况：
        //   门将已经在球与门之间时是正常封角位，交给下面原有规则做滑移封挡，别抢它的活。
        bool gk_field_side = ((gr.x - v.bx) * v.gside <= 0.0);
        bool gk_too_far = (gctx.dist_our_goal(gr.x) > kGkLastDitch * 0.75);
        // ⚠️ 真静止球（门球/死球）不进这条"钉住 x 横向让开 / 全速倒车"：它是我们要开出去的球。
        //   同上的真机取证：这条把正在就位的门将命令成全速倒车，是"一直在旁边冲"的直接来源。
        //   ⚠️ 判据必须是 ball_dead（严格 <0.02）而不是 ball_still —— 后者含"门球角"启发式命中的
        //   慢滚球（0.05~0.65cm/帧），那些正是这条要救的（撞门柱滚进来的球），豁免它们会让
        //   `test_gk_last_ditch_no_freeze` 立刻变红。
        bool gk_dead_skip = (kTkDeadSkip > 0.5 && v.ball_dead);
        if (!gk_dead_skip && v.ball_goal < kGkLastDitch && (gk_field_side || gk_too_far)) {
            // 慢球先走"主动去堵"（与 gk_rule_ball_behind 同一处逻辑，避免两处口径不一致）；
            //   球不慢才退回原来的"钉住 x 横向让开 / 横向已让开就插回门线"。
            if (!gk_slow_block(wm, id, v)) {
                double side0 = (gr.y >= v.by) ? 1.0 : -1.0;
                if (std::fabs(gr.y - v.by) <= kGkHoldPassGap)
                    motion::position(gr, gr.x,
                                     clamp(v.by + side0 * (kGkHoldPassGap + 2.0), kGkYLo, kGkYHi),
                                     motion::TM_PASS);
                else
                    motion::position(gr, gk_line_x(gctx, kGkBlockLineDepth),
                                     clamp(v.by, kGkYLo, kGkYHi), motion::TM_PASS);
            }
        } else if (!gk_frozen) {
        // 闸二（时刻盯球）：门将在球与自家门之间时（朝球 = 朝场外，安全），机头必须直着对准球。
        //   只给左右轮各加一个方向相反的转动分量：平动分量 (vl+vr)/2 完全不变，
        //   所以门将的位移一点没改（横向挪到球前照样挪），只是机头被拧向球。
        if ((gr.x - v.bx) * v.gside > 0.0) {
            double aim = angle_to(gr.x, gr.y, v.bx, v.by);
            double err = angle_diff(aim, gr.rot);
            if (std::fabs(err) > kGkFaceTol) {
                double spin = clamp((err > 0.0 ? err - kGkFaceTol : err + kGkFaceTol) * kGkSpinGain,
                                    -kGkSpinMax, kGkSpinMax);
                gr.vl -= spin;
                gr.vr += spin;
            }
        }
        }
        // 闸一（防乌龙）：球比门将更靠自家门（含余量）时，门将已经站在球的"场侧"了 ——
        //   此时它只要朝自家门方向走一步，就是拿身体把球顶进自家门。所以从轮速里把
        //   "朝自家门方向的平动分量"整条扣掉：转动分量保留（它照样能转身），横向平动也保留
        //   （它照样能横移挡到球前），但绝不可能朝自家门推进一步。
        //   横向已经让开球（> kGkHoldPassGap）时不干预：那条通道是安全的（车身4+球2<9），
        //   门将本来就该从那儿全速插到球与自家门之间。
        //   ★ 2026-10-12：从上面的 `else if (!gk_frozen)` 里**提出来**，让"闸零/闸二留下的
        //   任何指令"都过这道闸。真机两次乌龙（20261012095147 帧 322~340 / 1117~1137，我们打
        //   黄位）暴露出一个**恶性正反馈**：门将一旦蹭到球，`ball_dead` 立刻变假 ⇒
        //   `gk_dead_skip` 失效 ⇒ 闸零抢走分支，而闸一住在 else-if 里
        //   ⇒ **门将刚碰到球，保护它的那道闸就自己关了**。
        //   帧 330 的实际命令 vl=-135.7 vr=-150.0 本来会被本闸扣成纯原地转（vf=0），
        //   却全速朝自家门推了出去，把球从 (14.7,77.7) 一路送进网（15 帧后越线 y=92.5）。
        //   本闸对**正常发球是惰性的**：正经就位时门将在球后（更靠门），
        //   `ball_goal < dist_our_goal(gr.x) + pad` 不成立，所以不会干扰穿球。
        //   放在闸三之前：闸三是"超界硬拽回门线"的兜底，不该被本闸废掉。
        if (!gk_frozen &&
            v.ball_goal < gctx.dist_our_goal(gr.x) + kGkOwnGoalPad &&
            std::fabs(gr.y - v.by) <= kGkHoldPassGap) {
            double vf = (gr.vl + gr.vr) * 0.5;      // 沿机头的平动分量
            double spin = (gr.vr - gr.vl) * 0.5;    // 转动分量（差速，原样保留）
            if (std::fabs(vf) > 1e-9) {
                double dir = (vf >= 0.0) ? gr.rot : gr.rot + 180.0;   // 平动实际朝向
                double goalward = (v.gside > 0.0) ? 0.0 : 180.0;       // 朝自家门的方向
                if (std::cos((dir - goalward) * 3.14159265358979323846 / 180.0) > 0.0)
                    vf = 0.0;                        // 有朝门分量 → 平动归零
            }
            gr.vl = vf - spin;
            gr.vr = vf + spin;
        }
        // 闸三（硬边界）：门将绝不许离开己方站区。罚球区只有 80cm 深，实测却有 1.8~2.4% 的帧
        //   离门线 >100cm、最大 184cm（最出位那帧我方五人全在对方半场、门将 184）—— 说明有路径
        //   绕过了 clamp_goalie_area。这里在最外层兜底：一旦超限就全速拽回门线。
        if (gctx.dist_our_goal(gr.x) > kGkHardMaxOut) {
            gk_goto(gctx, gr, gk_line_x(gctx, kGkGuardDist),
                    clamp(gr.y, kGkYLo, kGkYHi), motion::TM_PASS);
        }
        // 闸四（缩在门里）：球离门线还远（>= kGkStayHomeBall）时，把出位深度压回 kGkMaxDepth，
        //   横向站位保持不变（只往回收 x，不动 y）。远射时站门线封角最大；
        //   跑出来只会把门口让开 —— 实测丢球形状里门将常在离门 20~28cm 处。
        //   只有"球进到近距离（需要推球/封堵）"或出球模式才准出来。
        if (v.ball_goal >= kGkStayHomeBall && gctx.dist_our_goal(gr.x) > kGkMaxDepth) {
            //   ⚠️ 2026-10-09 试过把第三参改成"跟球的 y"（stay_y = on_target ? y_at_goal : by），
            //   理由是 clamp(gr.y) 看着像"把横向跟踪整段关掉"。**实测更差，已回退**：
            //   同一 bench、6 种子×3 局（默认口径与 --true-ball-history 两种都跑）：
            //     · 只改这一处：默认口径 net 74.0→69.7、失球 13.33→17.00；
            //       tbh net 99.7→87.0、失球 14.33→17.33；9 局里"平台记录门将最后触球" 1→4 次。
            //   原因：本闸本来就是故意"只收 x、不动 y" —— 横向留给规则表（opp_ball_lock 用
            //   gk_block_cy 按球门框+对手位置算，比裸跟球 y 可信）。让它也管横向，
            //   等于用最粗的那个口径盖掉前面所有规则的横向选择。
            motion::position(gr, gk_line_x(gctx, kGkMaxDepth),
                             clamp(gr.y, kGkYLo, kGkYHi),
                             motion::TM_PASS);
        }
    }
#ifdef SIMURO5_HNNU_TRACE
    gk_trace(hit, wm, id, v);   // 决策段末尾（两闸之后）：tgt_x/tgt_y = 本帧真正下发的目标点
#endif
    }   // ═══ 决策段结束：以下不再吃旧驱动律 ═══

    // ── 定位路径用**新驱动律**重发同一个目标（2026-10-10）──
    //   门将原来整段吃旧律（`LegacyDriveScope`）。旧律在"目标在正侧方"时不给横向阻尼、
    //   单轮截断还会吃掉转向差 ⇒ 实测横向目标比新律慢 26%（17 帧 vs 23 帧触球，docs/06:4333）。
    //   真机证据：门将末 10 帧只移动 5.8cm（= 23cm/s），而平台线性能力约 150cm/s；
    //   它从最优待命位（门中心 y=90，实测越线 y 中位 89.8）到平均落点要横移 9.3cm
    //   ⇒ 按 23cm/s 需 0.40s，但预测可信窗口只有 0.25s（误差 4.6cm）——**0.40 > 0.25
    //   就是"门将离球中位 7cm"的出处**。这里把最后一步的轮速换算换成新律来补这个缺口。
    //   ⚠️ 两个例外必须保留：
    //     ① 出球/死球模式整段跳过 —— 旧律是扫 126 种门球摆位调出来的（docs/06:4332）；
    //     ② 规则主动"站定不动"（vl=vr=0，防乌龙决定）不许被解冻。
    if (kGkNewDrive > 0.5 && !gk_set_play && !(gr.vl == 0.0 && gr.vr == 0.0)) {
        motion::position(gr, gr.tgt_x, gr.tgt_y,
                         gr.tgt_mode == (int)motion::TM_PASS ? motion::TM_PASS : motion::TM_STOP);
    }
#ifdef SIMURO5_GK_PROBE
    motion::g_gk_probe.finish();
#endif
}
#undef GK_HIT

// 对方门区停留红线：门区除门将外纯停留 >8 帧即计（平台 20 周期判点球，须留撤出时间）
TUNABLE(kReboundRushSpeed, 7.34364);  // cm/帧：反弹球可抢速度阈值
TUNABLE(kReboundRushDist, 51.3294);  // cm：距球超过此值不冲（就近补）
// 门前补射触发区（run_swarm）：只补贴门线的漏球
TUNABLE(kReboundZoneLo, 30.0);   // cm：离对方门线 >= 此值才冲（低于此值球太深门区，门外等二点）
TUNABLE(kReboundZoneHi, 40.0);   // cm：离对方门线 < 此值才冲（更远的慢球交给别人）
TUNABLE(kReboundMinSpeed, 0.5);  // cm/帧：球速低于此值视为停球，不冲（保留门外等二点，防门区停球误冲）
TUNABLE(kReboundRushDepth, 30.0);  // cm：冲的目标点越过球多深（推球板在机身前 ~7.5cm，20cm 会把球推到
                                   //   离门线还差 ~7cm 处停住进不去；加深到 30 才够把球推过门线）
TUNABLE(kMaxShootPushes, 1);  // 同一轮进攻连续推球尝试上限（防禁区死磕送判罚）
// 净开口小于此角(度)就宁可先拐进中路：边路看门只有 12° 上下，中路有 67°
TUNABLE(kCorridorShotOpen, 12.0);
// 走廊拐弯总开关（1 开 0 关）；关掉即回到「边路硬推」的老行为
TUNABLE(kCorridorEnabled, 1.0);
TUNABLE(kPrepDist, 23.5578);
TUNABLE(kPenaltyPrepDist, 15.0);
double shoot_prep_dist(const WorldModel &wm) {
    return wm.in_penalty_exec ? kPenaltyPrepDist : kPrepDist;
}
TUNABLE(kPrepPosTol, 4.01318);
TUNABLE(kPrepAngTol, 13.1809);
TUNABLE(kShootAlignTimeout, 54.7289);
TUNABLE(kDribAngTol, 31.423);
constexpr bool kContestEnabled = true;   // 回滚开关
TUNABLE(kContestOppDist, 20.0);
TUNABLE(kContestReach, 60.0);
TUNABLE(kContestLead, 8.0);
TUNABLE(kContestNoWait, 1.0);
TUNABLE(kContestPush, 1.0);
TUNABLE(kContestCharge, 1.0);      // 回滚开关
TUNABLE(kChargeMinFwd, -0.2);      // cos：偏离进攻方向在此以内都可冲
TUNABLE(kChargeMinFwdOwn, 0.3);    // cos：己方门前必须明显朝前推
TUNABLE(kChargeOwnGuard, 70.0);    // cm
TUNABLE(kChargeThrough, 20.0);     // cm：目标 = 球心沿冲撞方向再过 20cm（穿球）
TUNABLE(kPassivePress, 1.0);   // PASSIVE 争抢时从球门侧上抢
TUNABLE(kPassivePressDepth, 130.0);   // cm：离己门此深度内才上抢
// 门前第二落点（2026-10-10）：球进到门前这个距离内、且我已经在球的门侧 → 上去把它推出去。
//   原来写死 25cm。真机 8 场 20 个失球里 **11 球发生在门前 60cm 内**，而那 0.5 秒里
//   我方最近的非门将中位离球 29cm —— 看着"有人"，但没人真的去抢；门将常是唯一在场的人。
//   "我已在球的门侧"这条护栏保留：从门侧推球只会把球推离自家门，不可能乌龙（与清道夫同源）。
TUNABLE(kDoorPushDist, 60.0);   // cm：球离己方门线小于此值就允许门侧上抢（0 = 关闭，回到 25）
TUNABLE(kCoverDetour, 1.0);        // 回滚开关
TUNABLE(kCoverClear, 12.0);        // cm：路线离球心小于此值算会撞球
TUNABLE(kCoverDetourLat, 22.0);    // cm：绕行点横向距离
TUNABLE(kDefFaceIncoming, 1.0);    // 回滚开关（0 = 只给位置不给朝向）
TUNABLE(kDefArriveDist, 6.0);      // cm：进入该半径即停车转正迎球
TUNABLE(kDefFaceAngTol, 12.0);     // 度：迎球朝向允许误差
TUNABLE(kActiveGaLimit, 8);
TUNABLE(kActiveGaTotal, 18);  // 在门区总时长兜底（平台 20 周期判罚红线，留 2 帧余量）

namespace {
constexpr int kPassTaskFrames = 80;
constexpr double kPassReleaseTravel = 6.0;
constexpr double kPassReleaseSpeed = 1.0;
constexpr double kPassReleaseSeparation = 14.0;
constexpr double kPassReceiveDistance = 12.0;
constexpr double kPassReceiveSpeed = 3.0;
constexpr int kPassReceiveFrames = 2;
constexpr double kPassControlDistance = 20.0;
constexpr int kPassControlLooseFrames = 3;
TUNABLE(kPassReceiveSlowRadius, 30.0);       // cm：球进入此距离后切换接球控制
TUNABLE(kPassReceiveMinDriveScale, 0.25);   // 贴球时保留的最小平移比例
TUNABLE(kPassReceiveDirMinSpeed, 0.5);      // cm/帧：低于此值不采信球速方向
TUNABLE(kPassReceiveDirMaxSpeed, 15.0);     // cm/帧：高于此值视为异常速度
TUNABLE(kPassReceiveAngleTol, 12.0);        // 度：面向来球的允许误差

TUNABLE(kRecvMeetBall, 1.0);
TUNABLE(kRecvLead, 6.0);        // 帧：比球早到这么多帧
TUNABLE(kRecvSpeed, 2.0);       // cm/帧：接球人估算速度
TUNABLE(kRecvMeetMaxOff, 60.0); // cm：会合点相对锁点的最大偏移（防追回自家半场）
TUNABLE(kRecvArriveDist, 8.0);  // cm：进入即停车转正迎球
TUNABLE(kRecvNoReverse, 1.0);   // 1=接球路径禁止倒车

bool pass_context_safe_impl(const WorldModel &wm) {
    // 活球判定用 runtime_phase（Running 即活球，语义同原 live_play：球离开重启摆放点就算开球），
    //   不用 game_state==PM_PlayOn：平台活球期 gameState 恒为 FreeBall(1~4)/PlaceKick(5)，从无 PM_PlayOn(0)，
    //   旧写法让传球在真机活球期恒被拦死（与反击窗口同一坑，见 gamestate-playon-never-live）。
    //   合并取舍（2026-10-12）：队员的 runtime_phase 取代原 live_play，语义等价；
    //   runtime_phase == Running 时不得被陈旧的 raw game_state 挡住，它才是唯一常规动作闸。
    if (!wm.ball.valid || wm.runtime_phase != RuntimePhase::Running ||
        wm.in_penalty_exec ||
        wm.threat_level >= 0.6 || in_no_push_zone(wm.ball.x, wm.ball.y)) return false;
    double theirs = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        theirs = std::min(theirs, dist(wm.ball.x, wm.ball.y, wm.opp[i].x, wm.opp[i].y));
    }
    if (wm.possession == Possession::Opponent) return false;
    if (wm.ctx.dist_our_goal(wm.ball.x) < 80.0 &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 3.0 && theirs < 100.0) return false;
    if (shot_on_target(wm) && ball_danger_speed(wm) > rebound_min_danger()) return false;
    return true;
}

bool pass_target_safe_impl(const WorldModel &wm, int passer, int receiver,
                           double tx, double ty, bool check_lane) {
    if (!pass_context_safe_impl(wm) || passer != 1 || receiver < 2 || receiver > 4 ||
        !std::isfinite(tx) || !std::isfinite(ty) || tx < 0 || tx > 220 || ty < 0 || ty > 180 ||
        in_opp_goal_area(wm.ctx, tx, ty) || in_goal_area(wm.ctx, tx, ty) || in_no_push_zone(tx, ty)) return false;
    for (int id : {passer, receiver}) {
        if (wm.ga_cooldown[id] > 0 || wm.ga_overstay[id] >= 15 || in_goal_area(wm.ctx, wm.home[id].x, wm.home[id].y)) return false;
    }
    if (wm.active_ga_frames > kActiveGaLimit || wm.active_ga_total > kActiveGaTotal) return false;
    if (!check_lane) return true;
    CircleObstacle obs[PLAYERS_PER_SIDE];
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (dist(tx, ty, wm.opp[i].x, wm.opp[i].y) < 20.0) return false;
        obs[i] = {wm.opp[i].x, wm.opp[i].y, 8.0};
    }
    return segment_clear_of_circles(wm.ball.x, wm.ball.y, tx, ty, obs, PLAYERS_PER_SIDE);
}

bool pass_control_safe(const WorldModel &wm) {
    int id = wm.coop_ball_control.receiver_id;
    // 合并（2026-10-12）：队员 ff2e542 的 active_id == 1 保守门槛（非 1 号主攻不建传球任务）；
    //   并去掉我们的 coop_ball_control.game_state 锁 —— 该字段被球权统一删掉，
    //   语义（"临时控球不能跨比赛阶段存活"）由 runtime_phase + 任务生命周期取消覆盖
    return wm.active_id == 1 && pass_context_safe_impl(wm) &&
        id >= 1 && id <= 4 && wm.role[id] != ROLE_ACTIVE &&
        wm.ga_cooldown[id] <= 0 &&
        wm.ga_overstay[id] < 15 && !in_goal_area(wm.ctx, wm.home[id].x, wm.home[id].y) &&
        !in_opp_goal_area(wm.ctx, wm.home[id].x, wm.home[id].y);
}

void observe_pass_lifecycle(WorldModel &wm) {
    auto &task = wm.coop_pass_task;
    if (task.active && task.passer_id == wm.active_id && task.receiver_id >= 1 && task.receiver_id <= 4 &&
        task.receiver_id != task.passer_id &&
        // 合并（2026-10-12）：用队员的 runtime_phase 替代我们的 task.game_state 锁（同一语义）
        task.frames_left > 0 && wm.runtime_phase == RuntimePhase::Running && pass_context_safe_impl(wm)) {
        if (task.phase == CoopPassPhase::Preparing && task.observing_push) {
            const RobotState &passer = wm.home[task.passer_id];
            double progress = (wm.ball.x - task.push_ball_x) * task.push_dir_x +
                              (wm.ball.y - task.push_ball_y) * task.push_dir_y;
            double speed = wm.ball.vx * task.push_dir_x + wm.ball.vy * task.push_dir_y;
            double ahead = (wm.ball.x - passer.x) * task.push_dir_x + (wm.ball.y - passer.y) * task.push_dir_y;
            if (progress >= kPassReleaseTravel && speed >= kPassReleaseSpeed && ahead > 0.0 &&
                dist(wm.ball.x, wm.ball.y, passer.x, passer.y) >= kPassReleaseSeparation) {
                task.phase = CoopPassPhase::Receiving;
                wm.coop_released();
            }
        }
        if (task.phase == CoopPassPhase::Receiving) {
            const RobotState &receiver = wm.home[task.receiver_id];
            double db = dist(receiver.x, receiver.y, wm.ball.x, wm.ball.y);
            double opp_min = 1e9, teammate_min = 1e9;
            for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
                opp_min = std::min(opp_min, dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y));
                if (i != task.receiver_id) teammate_min = std::min(teammate_min, dist(wm.home[i].x, wm.home[i].y, wm.ball.x, wm.ball.y));
            }
            bool evidence = wm.possession == Possession::Ours || db + 5.0 < opp_min;
            bool received = db < kPassReceiveDistance && db < teammate_min && db < opp_min && evidence &&
                            std::hypot(wm.ball.vx, wm.ball.vy) <= kPassReceiveSpeed;
            task.receive_frames = received ? task.receive_frames + 1 : 0;
            if (task.receive_frames >= kPassReceiveFrames &&
                pass_target_safe_impl(wm, task.passer_id, task.receiver_id, task.rx, task.ry, false)) {
                task.phase = CoopPassPhase::Received;
                wm.coop_ball_control = {true, task.receiver_id, 0};
                wm.coop_finish(CoopOutcome::Success);
                wm.coop_control_entered();
            }
        }
    }
    auto &control = wm.coop_ball_control;
    if (control.active && pass_control_safe(wm)) {
        const auto &r = wm.home[control.receiver_id];
        double db = dist(r.x, r.y, wm.ball.x, wm.ball.y);
        control.loose_frames = db > kPassControlDistance ? control.loose_frames + 1 : 0;
        if (control.loose_frames >= kPassControlLooseFrames) wm.coop_control_end(CoopOutcome::LooseBall);
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            if (i == control.receiver_id) continue;
            double other = dist(wm.home[i].x, wm.home[i].y, wm.ball.x, wm.ball.y);
            if (other < kPassReceiveDistance && other + 5.0 < db) wm.coop_control_end(CoopOutcome::TeammateTakeover);
        }
    }
}

bool pass_carry_point_safe(const WorldModel &wm, double x, double y) {
    return x >= 6.0 && x <= 214.0 && y >= 6.0 && y <= 174.0 &&
        !in_goal_area(wm.ctx, x, y) && !in_opp_goal_area(wm.ctx, x, y) && !in_no_push_zone(x, y);
}

void carry_pass_ball(WorldModel &wm, int id) {
    RobotState &r = wm.home[id];
    double bx = wm.ball.x, by = wm.ball.y;
    double tx = clamp(bx + wm.ctx.attack_dir() * 20.0, 6.0, 214.0), ty = by;
    if (!pass_carry_point_safe(wm, tx, ty)) { tx = bx; ty = clamp(by + (by >= 90.0 ? 20.0 : -20.0), 6.0, 174.0); }
    double length = dist(bx, by, tx, ty);
    if (length < 1e-6 || !pass_carry_point_safe(wm, tx, ty)) { wm.coop_control_end(CoopOutcome::CarryPoint); motion::stop(r); return; }
    double dx = (tx - bx) / length, dy = (ty - by) / length;
    double side = (bx - r.x) * dx + (by - r.y) * dy;
    if (side > 0.0 && dist(r.x, r.y, bx, by) < kPassReceiveDistance) {
        if (motion::position_aligned(r, r.x, r.y, angle_to(0.0, 0.0, dx, dy), kPrepPosTol, kPrepAngTol))
            motion::position(r, tx, ty, motion::TM_PASS);
        return;
    }
    double px = bx - dx * 8.0, py = by - dy * 8.0;
    if (side < -3.0) { px = bx - dy * 22.0; py = by + dx * 22.0; }
    if (!pass_carry_point_safe(wm, px, py)) { wm.coop_control_end(CoopOutcome::CarryPoint); motion::stop(r); return; }
    motion::position_aligned(r, px, py, angle_to(0.0, 0.0, dx, dy), kPrepPosTol, kPrepAngTol);
}

void cancel_unsafe_pass_task(WorldModel &wm) {
    auto &task = wm.coop_pass_task;
    const bool invalid_actors = task.active &&
        (task.passer_id != 1 || task.passer_id != wm.active_id ||
         task.receiver_id < 2 || task.receiver_id > 4);
    if (task.active && (wm.runtime_phase != RuntimePhase::Running || task.frames_left <= 0 ||
        // 合并（2026-10-12）：保留队员的 invalid_actors 参与者身份校验
        invalid_actors ||
        !pass_target_safe_impl(wm, task.passer_id, task.receiver_id, task.rx, task.ry,
                          task.phase == CoopPassPhase::Preparing))) {
        CoopOutcome reason = CoopOutcome::InvalidTarget;
        if (wm.runtime_phase != RuntimePhase::Running) reason = CoopOutcome::PhaseInterrupted;
        else if (task.frames_left <= 0)
            reason = task.phase == CoopPassPhase::Preparing ? CoopOutcome::PrepareTimeout : CoopOutcome::ReceiveTimeout;
        else if (invalid_actors) reason = CoopOutcome::InvalidTarget;
        else if (wm.in_penalty_exec) reason = CoopOutcome::Penalty;
        else if (in_no_push_zone(wm.ball.x, wm.ball.y)) reason = CoopOutcome::Corner;
        else if (wm.threat_level >= 0.6) reason = CoopOutcome::HighThreat;
        else if (wm.possession == Possession::Opponent) reason = CoopOutcome::Intercepted;
        else if (wm.active_ga_frames > kActiveGaLimit || wm.active_ga_total > kActiveGaTotal ||
                 wm.ga_cooldown[task.passer_id] > 0 || wm.ga_cooldown[task.receiver_id] > 0)
            reason = CoopOutcome::GoalDiscipline;
        else if (wm.ctx.dist_our_goal(wm.ball.x) < 80.0 && std::hypot(wm.ball.vx, wm.ball.vy) < 3.0)
            reason = CoopOutcome::EmergencyDefense;
        else {
            bool marked = false;
            for (int i = 0; i < PLAYERS_PER_SIDE; ++i)
                marked = marked || dist(task.rx, task.ry, wm.opp[i].x, wm.opp[i].y) < 20.0;
            if (marked) reason = CoopOutcome::ReceiverMarked;
            else if (task.phase == CoopPassPhase::Preparing) {
                CircleObstacle obs[PLAYERS_PER_SIDE];
                for (int i = 0; i < PLAYERS_PER_SIDE; ++i)
                    obs[i] = {wm.opp[i].x, wm.opp[i].y, 8.0};
                if (!segment_clear_of_circles(wm.ball.x, wm.ball.y, task.rx, task.ry, obs, PLAYERS_PER_SIDE))
                    reason = CoopOutcome::LaneBlocked;
            }
        }
        wm.coop_finish(reason);
    }
    if (task.active && pass_opponent_arrives_first(wm)) wm.coop_finish(CoopOutcome::OpponentFirst);
    if (wm.coop_ball_control.active && !pass_control_safe(wm))
        wm.coop_control_end(wm.runtime_phase == RuntimePhase::Running ?
                            CoopOutcome::InvalidTarget : CoopOutcome::PhaseInterrupted);
}

double pass_receive_facing(const WorldModel &wm, const CoopPassTask &task, const RobotState &receiver) {
    const double rel_x = receiver.x - wm.ball.x, rel_y = receiver.y - wm.ball.y;
    const double rel_d = std::hypot(rel_x, rel_y);
    const double ball_speed = std::hypot(wm.ball.vx, wm.ball.vy);
    const double min_speed = std::isfinite(kPassReceiveDirMinSpeed) && kPassReceiveDirMinSpeed > 1e-6
                           ? kPassReceiveDirMinSpeed : 0.5;
    const double max_speed = std::isfinite(kPassReceiveDirMaxSpeed) && kPassReceiveDirMaxSpeed >= min_speed
                           ? kPassReceiveDirMaxSpeed : 15.0;
    const double approach = std::isfinite(rel_d) && rel_d > 1e-6
                          ? (wm.ball.vx * rel_x + wm.ball.vy * rel_y) / rel_d : 0.0;
    if (std::isfinite(ball_speed) && ball_speed >= min_speed && ball_speed <= max_speed &&
        std::isfinite(approach) && approach >= min_speed) {
        return angle_to(0.0, 0.0, -wm.ball.vx, -wm.ball.vy);
    }
    if (std::isfinite(rel_d) && rel_d > 1e-6)
        return angle_to(receiver.x, receiver.y, wm.ball.x, wm.ball.y);
    const double push_len = std::hypot(task.push_dir_x, task.push_dir_y);
    if (std::isfinite(push_len) && push_len > 1e-6)
        return angle_to(0.0, 0.0, -task.push_dir_x, -task.push_dir_y);
    return receiver.rot;
}

// 接球目标点：球在飞 → 会合点（我比球早到 kRecvLead 帧的点）；球停着或会合点太远 → 锁点
bool receiver_meeting_target(const WorldModel &wm, const CoopPassTask &task, int id,
                             double &tx, double &ty, double &out_aim) {
    tx = task.rx; ty = task.ry; out_aim = 0.0;
    if (kRecvMeetBall < 0.5) return false;
    const RobotState &r = wm.home[id];
    double mx = 0.0, my = 0.0, maim = 0.0;
    if (!ball_meeting_point(wm, r.x, r.y, kRecvSpeed, kRecvLead, mx, my, maim)) return false;
    if (dist(mx, my, task.rx, task.ry) > kRecvMeetMaxOff) return false;
    // 纪律红线：会合点未经场地/门区过滤，落在任一门区或角区时一律弃用、回退锁点
    const TeamContext &ctx = wm.ctx;
    if (in_opp_goal_area(ctx, mx, my) || in_goal_area(ctx, mx, my) || in_no_push_zone(mx, my)) return false;
    tx = mx; ty = my; out_aim = maim;
    return true;
}

void turn_in_place(RobotState &r, double te_deg) {
    double w = clamp(0.22 * te_deg, -14.0, 14.0);
    if (std::fabs(w) < 2.5) w = (te_deg > 0.0) ? 2.5 : -2.5;
    r.vl = clamp(-w, -motion::kMaxWheel, motion::kMaxWheel);
    r.vr = clamp( w, -motion::kMaxWheel, motion::kMaxWheel);
}

void run_receiving_receiver(WorldModel &wm, const CoopPassTask &task, int id) {
    RobotState &receiver = wm.home[id];
    double tx = task.rx, ty = task.ry, meet_aim = 0.0;
    const bool has_meet = receiver_meeting_target(wm, task, id, tx, ty, meet_aim);
    auto go_to = [&](double gx, double gy) {
        if (kRecvNoReverse > 0.5) {
            const double te_go = angle_diff(angle_to(receiver.x, receiver.y, gx, gy), receiver.rot);
            if (std::fabs(te_go) > 95.0) { turn_in_place(receiver, te_go); return; }
        }
        motion::position(receiver, gx, gy);
    };

    const double ball_distance = dist(receiver.x, receiver.y, wm.ball.x, wm.ball.y);
    if (!std::isfinite(ball_distance) || !std::isfinite(kPassReceiveSlowRadius) ||
        kPassReceiveSlowRadius <= 1e-6 || ball_distance >= kPassReceiveSlowRadius) {
        if (has_meet) motion::arrive_facing(receiver, tx, ty, meet_aim, kRecvArriveDist,
                                            kPassReceiveAngleTol, /*allow_reverse=*/kRecvNoReverse < 0.5);
        else go_to(tx, ty);
        return;
    }

    const double desired_rot = pass_receive_facing(wm, task, receiver);
    const double angle_tol = std::isfinite(kPassReceiveAngleTol) && kPassReceiveAngleTol > 0.0
                           ? kPassReceiveAngleTol : 12.0;
    if (!std::isfinite(desired_rot) || !std::isfinite(receiver.rot)) {
        motion::stop(receiver);
        return;
    }
    const double facing_error = angle_diff(desired_rot, receiver.rot);
    if (std::fabs(facing_error) > angle_tol) {
        motion::position_aligned(receiver, receiver.x, receiver.y, desired_rot, 3.0, angle_tol);
    } else {
        const double target_distance = dist(receiver.x, receiver.y, tx, ty);
        if (!std::isfinite(target_distance) || target_distance <= 3.0) {
            motion::stop(receiver);
        } else {
            const double target_rot = angle_to(receiver.x, receiver.y, tx, ty);
            const double forward_error = std::fabs(angle_diff(target_rot, desired_rot));
            const double reverse_error = std::fabs(angle_diff(target_rot + 180.0, desired_rot));
            if (std::min(forward_error, reverse_error) <= 2.0 * angle_tol)
                go_to(tx, ty);
            else
                motion::stop(receiver);
        }
    }

    const double min_scale = std::isfinite(kPassReceiveMinDriveScale)
                           ? clamp(kPassReceiveMinDriveScale, 0.0, 1.0) : 0.25;
    const double drive_scale = clamp(ball_distance / kPassReceiveSlowRadius, min_scale, 1.0);
    double drive = 0.5 * (receiver.vl + receiver.vr);
    const double turn = 0.5 * (receiver.vr - receiver.vl);
    drive *= drive_scale;
    receiver.vl = clamp(drive - turn, -motion::kMaxWheel, motion::kMaxWheel);
    receiver.vr = clamp(drive + turn, -motion::kMaxWheel, motion::kMaxWheel);
}

void run_preparing_receiver(WorldModel &wm, const CoopPassTask &task, int id) {
    RobotState &receiver = wm.home[id];
    double tx = task.rx, ty = task.ry, aim = 0.0;
    if (receiver_meeting_target(wm, task, id, tx, ty, aim)) {
        motion::arrive_facing(receiver, tx, ty, aim, kRecvArriveDist, kPassReceiveAngleTol,
                              /*allow_reverse=*/kRecvNoReverse < 0.5);
        return;
    }
    if (kRecvNoReverse > 0.5) {
        const double te_go = angle_diff(angle_to(receiver.x, receiver.y, tx, ty), receiver.rot);
        if (std::fabs(te_go) > 95.0) { turn_in_place(receiver, te_go); return; }
    }
    motion::position(receiver, tx, ty);
}

bool run_pass_receiver(WorldModel &wm, int id) {
    cancel_unsafe_pass_task(wm);
    if (wm.coop_ball_control.active && wm.coop_ball_control.receiver_id == id) { carry_pass_ball(wm, id); return true; }
    const auto &task = wm.coop_pass_task;
    if (!task.active || task.receiver_id != id) return false;
    if (task.phase == CoopPassPhase::Receiving) run_receiving_receiver(wm, task, id);
    else run_preparing_receiver(wm, task, id);
    return true;
}
}

void cancel_unsafe_coop_pass(WorldModel &wm) { cancel_unsafe_pass_task(wm); }

bool pass_context_safe(const WorldModel &wm) { return pass_context_safe_impl(wm); }
bool pass_target_safe(const WorldModel &wm, int passer, int receiver,
                      double tx, double ty, bool check_lane) {
    return pass_target_safe_impl(wm, passer, receiver, tx, ty, check_lane);
}

// ── 以下为队员分支新增（合并 2026-10-12 保留）──
// Ordinary 的准备是“先到球后，再对准，再等接球人”；CoopPass 与射门继续走原分支。
static void execute_ordinary_pass_prep(WorldModel &wm, int id) {
    CoopPassTask &task = wm.coop_pass_task;
    RobotState &r = wm.home[id];
    const double dx = task.rx - wm.ball.x;
    const double dy = task.ry - wm.ball.y;
    const double length = std::hypot(dx, dy);
    if (!std::isfinite(length) || length <= 1e-6 ||
        !std::isfinite(wm.ball.x) || !std::isfinite(wm.ball.y) ||
        !std::isfinite(task.rx) || !std::isfinite(task.ry) ||
        !std::isfinite(r.x) || !std::isfinite(r.y) || !std::isfinite(r.rot)) {
        motion::stop(r);
        return;
    }

    const double dir_x = dx / length;
    const double dir_y = dy / length;
    task.push_dir_x = dir_x;
    task.push_dir_y = dir_y;
    const double aim_rot = angle_to(0.0, 0.0, dir_x, dir_y);
    const double prep_x = wm.ball.x - dir_x * kPrepDist;
    const double prep_y = wm.ball.y - dir_y * kPrepDist;
    const double db = dist(r.x, r.y, wm.ball.x, wm.ball.y);
    const double behind = (wm.ball.x - r.x) * dir_x + (wm.ball.y - r.y) * dir_y;
    const double heading_error = angle_diff(aim_rot, r.rot);
    const bool distance_ok = std::isfinite(db) && db < kPrepDist + 6.0;
    const bool behind_ok = std::isfinite(behind) && behind > 0.0;
    const bool heading_ok = std::isfinite(heading_error) &&
                            std::fabs(heading_error) <= kPrepAngTol;

    // 首次合法 Push 后只执行固定穿球点；本帧的安全取消已在 run_active 入口完成。
    if (task.push_target_locked) {
        if (!task.observing_push && db < kPassReceiveDistance) {
            task.observing_push = true;
            task.push_ball_x = wm.ball.x;
            task.push_ball_y = wm.ball.y;
        }
        motion::position(r, task.locked_push_target_x, task.locked_push_target_y, motion::TM_PASS);
        return;
    }

    if (!std::isfinite(prep_x) || !std::isfinite(prep_y) || !prep_point_ok(prep_x, prep_y)) {
        motion::stop(r);
        return;
    }

    const double nx = -dir_y, ny = dir_x;
    double bypass_distance = 1e9;
    if (task.ordinary_prep_side != 0) {
        const double bypass_x = wm.ball.x + nx * task.ordinary_prep_side * 22.0;
        const double bypass_y = wm.ball.y + ny * task.ordinary_prep_side * 22.0;
        bypass_distance = dist(r.x, r.y, bypass_x, bypass_y);
    }
    if (task.ordinary_prep_phase == OrdinaryPassPrepPhase::GoPrepPoint && !behind_ok &&
        bypass_distance > kPrepPosTol) {
        task.ordinary_prep_phase = OrdinaryPassPrepPhase::GoPrepSide;
        const double plus_x = wm.ball.x + nx * 22.0, plus_y = wm.ball.y + ny * 22.0;
        const double minus_x = wm.ball.x - nx * 22.0, minus_y = wm.ball.y - ny * 22.0;
        task.ordinary_prep_side = dist(r.x, r.y, plus_x, plus_y) <=
                                  dist(r.x, r.y, minus_x, minus_y) ? 1 : -1;
    }

    if (task.ordinary_prep_phase == OrdinaryPassPrepPhase::GoPrepSide) {
        double side_x = wm.ball.x + nx * task.ordinary_prep_side * 22.0;
        double side_y = wm.ball.y + ny * task.ordinary_prep_side * 22.0;
        if (!prep_point_ok(side_x, side_y)) {
            task.ordinary_prep_side = -task.ordinary_prep_side;
            side_x = wm.ball.x + nx * task.ordinary_prep_side * 22.0;
            side_y = wm.ball.y + ny * task.ordinary_prep_side * 22.0;
        }
        if (!prep_point_ok(side_x, side_y)) {
            motion::stop(r);
            return;
        }
        if (dist(r.x, r.y, side_x, side_y) > kPrepPosTol) {
            motion::position(r, side_x, side_y, motion::TM_PASS);
            return;
        }
        task.ordinary_prep_phase = OrdinaryPassPrepPhase::GoPrepPoint;
    }

    if (task.ordinary_prep_phase == OrdinaryPassPrepPhase::GoPrepPoint) {
        if (behind_ok && dist(r.x, r.y, prep_x, prep_y) <= kPrepPosTol) {
            task.ordinary_prep_phase = OrdinaryPassPrepPhase::Align;
        } else {
            motion::position(r, prep_x, prep_y, motion::TM_PASS);
            return;
        }
    }

    if (task.ordinary_prep_phase == OrdinaryPassPrepPhase::Align) {
        if (distance_ok && behind_ok && heading_ok) {
            task.ordinary_prep_phase = OrdinaryPassPrepPhase::Prepared;
        } else if (!distance_ok || !behind_ok) {
            // 状态保持在 Align，轻微几何扰动不会退回 GoPrep；只有出界才回到准备区。
            motion::position_aligned(r, prep_x, prep_y, aim_rot, kPrepPosTol, kPrepAngTol);
            return;
        } else {
            motion::position_aligned(r, r.x, r.y, aim_rot, kPrepPosTol, kPrepAngTol);
            return;
        }
    }

    if (task.ordinary_prep_phase != OrdinaryPassPrepPhase::Prepared) {
        motion::stop(r);
        return;
    }
    if (!distance_ok || !behind_ok || !heading_ok) {
        // Prepared 是单向锁存阶段；姿态被扰动时安全回到准备姿态，不来回切阶段。
        motion::position_aligned(r, prep_x, prep_y, aim_rot, kPrepPosTol, kPrepAngTol);
        return;
    }
    if (!pass_receiver_ready(wm)) {
        motion::position_aligned(r, r.x, r.y, aim_rot, kPrepPosTol, kPrepAngTol);
        return;
    }

    // 出球动作与实际 Release 分开：这里锁定 waypoint，球的证据仍由生命周期观察器判定。
    task.push_target_locked = true;
    task.push_start_ball_x = wm.ball.x;
    task.push_start_ball_y = wm.ball.y;
    task.push_start_dir_x = dir_x;
    task.push_start_dir_y = dir_y;
    task.locked_push_target_x = task.push_start_ball_x + task.push_start_dir_x * 20.0;
    task.locked_push_target_y = task.push_start_ball_y + task.push_start_dir_y * 20.0;
    if (!task.observing_push && db < kPassReceiveDistance) {
        task.observing_push = true;
        task.push_ball_x = wm.ball.x;
        task.push_ball_y = wm.ball.y;
    }
    motion::position(r, task.locked_push_target_x, task.locked_push_target_y, motion::TM_PASS);
}

void run_active(WorldModel &wm, int id) {
    RobotState &r = wm.home[id];
    const TeamContext &ctx = wm.ctx;
    const bool had_pass_task = wm.coop_pass_task.active || wm.coop_ball_control.active;
    if (wm.coop_pass_task.active) --wm.coop_pass_task.frames_left;
    observe_pass_lifecycle(wm);
    cancel_unsafe_pass_task(wm);
    const bool fixed_pass_actor = id == 1 && wm.active_id == 1;

    if (!wm.in_penalty_exec) wm.pen_aim_locked = false;

    // 对方门区停留计数：球不在门区或球不在脚下(>25cm) 即计纯停留，离开清零；总时长另设兜底
    if (in_opp_goal_area(ctx, r.x, r.y)) {
        ++wm.active_ga_total;
        bool ball_in_ga = in_opp_goal_area(ctx, wm.ball.x, wm.ball.y);
        if (!ball_in_ga || dist(r.x, r.y, wm.ball.x, wm.ball.y) > 25.0) ++wm.active_ga_frames;
    } else {
        wm.active_ga_frames = 0;
        wm.active_ga_total = 0;
    }

    if (wm.shoot_push_cd > 0) --wm.shoot_push_cd;
    if (ctx.dist_opp_goal(wm.ball.x) > 75.0 ||
        (!wm.we_have_ball && dist(r.x, r.y, wm.ball.x, wm.ball.y) > 25.0)) {
        wm.shoot_push_count = 0;
        wm.shoot_push_last_side = 0;
    }

    if ((wm.ball.x < 30.0 || wm.ball.x > 190.0) &&
        (wm.ball.y < 30.0 || wm.ball.y > 150.0) &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 1.0) {
        ++wm.corner_ball_frames;
    } else {
        wm.corner_ball_frames = 0;
    }

    if (!robot_touch_allowed(wm, id)) { if (wm.coop_pass_task.active) wm.coop_finish(CoopOutcome::PushForbidden); hold_out_of_corner(wm, r); return; }

    // 对方门球/定位球重启：球停死在对方门区，别冲进去抢；先站罚球区外沿外等开球
    //   例外：我方主罚点球（in_penalty_exec）必须去踢
    if (!wm.in_penalty_exec &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 1.0 &&
        std::fabs(wm.ball.x - ctx.opp_goal_x()) < 50.0 &&
        std::fabs(wm.ball.y - 90.0) < 20.0) {
        if (++wm.dead_ball_frames > 100) {
            wm.dead_ball_frames = 0;
        } else {
            if (wm.coop_pass_task.active) wm.coop_finish(CoopOutcome::DeadBall);
            double hold_x = ctx.opp_goal_x() - ctx.attack_dir() * 85.0;
            double hold_y = clamp(wm.ball.y, 72.5, 107.5);
            motion::position(r, hold_x, hold_y);
            return;
        }
    } else {
        wm.dead_ball_frames = 0;
    }

    // 门区停留时限（须在射门之前，否则停留帧数无限累积）：超限后传球给接应，无传球则撤出门区
    if (wm.active_ga_frames > kActiveGaLimit || wm.active_ga_total > kActiveGaTotal) {
        if (wm.coop_pass_task.active) wm.coop_finish(CoopOutcome::GoalDiscipline);
        if (wm.coop_ball_control.active) wm.coop_control_end(CoopOutcome::GoalDiscipline);
        ++wm.ga_retreat_fires;   // 诊断用
        PassPlan pp_ga = fixed_pass_actor ? plan_pass(wm, id) : PassPlan{};
        if (pp_ga.viable) { motion::position(r, pp_ga.target_x, pp_ga.target_y); return; }
        double ogx = ctx.opp_goal_x(), ad = ctx.attack_dir();
        motion::position(r, ogx - ad * 60.0, clamp(wm.ball.y, 72.5, 107.5));
        return;
    }

    if ((wm.coop_pass_task.active && wm.coop_pass_task.phase == CoopPassPhase::Receiving) ||
        wm.coop_ball_control.active) {
        motion::stop(r);
        return;
    }

    // 争抢态：对手已贴球且我够得着 → 不原地转正、不等接球人、不去队友接球点
    double opp_ball = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i)
        opp_ball = std::min(opp_ball, dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y));
    const double my_ball = dist(r.x, r.y, wm.ball.x, wm.ball.y);
    const bool contested = kContestEnabled && !wm.in_penalty_exec &&
                           opp_ball < kContestOppDist && my_ball < kContestReach &&
                           my_ball > opp_ball - kContestLead;
    if (contested && kContestNoWait > 0.5 && wm.coop_pass_task.active && !pass_receiver_ready(wm))
        wm.coop_finish(CoopOutcome::OpponentFirst);
    // 角区救球：只救角区外环的卡球（该处推球合法）；球压在角心则不救，等平台判僵局重置
    if (wm.corner_ball_frames > 30) {
        double db = dist(r.x, r.y, wm.ball.x, wm.ball.y);
        if (db < 80.0) {
            bool deep = in_no_push_zone(wm.ball.x, wm.ball.y);   // 是否在禁推角区
            if (!deep) {
                if (wm.coop_pass_task.active) wm.coop_finish(CoopOutcome::Corner);
                ++wm.corner_rescue_events;   // 统计用
                double ex = 110.0 - wm.ball.x, ey = 90.0 - wm.ball.y;
                double elen = std::hypot(ex, ey);
                if (elen > 1e-6) { ex /= elen; ey /= elen; }
                double back_x = wm.ball.x - ex * 8.0, back_y = wm.ball.y - ey * 8.0;
                double te_ball = angle_diff(angle_to(r.x, r.y, wm.ball.x, wm.ball.y), r.rot);
                if (db < 22.0 && std::fabs(te_ball) < 40.0) {
                    motion::position(r, wm.ball.x + ex * 30.0, wm.ball.y + ey * 30.0, motion::TM_PASS);   // 穿球踢向场心
                } else if (!in_no_push_zone(back_x, back_y)) {
                    motion::position(r, back_x, back_y);
                } else {
                    hold_out_of_corner(wm, r);   // 准备点在角区内 → 退到场心侧
                }
                return;
            }
        }
    }

    if (wm.coop_pass_task.active && wm.coop_pass_task.kind == PassTaskKind::Ordinary) {
        auto &task = wm.coop_pass_task;
        if (task.phase == CoopPassPhase::Receiving) { motion::stop(r); return; }
        execute_ordinary_pass_prep(wm, id);
        return;
    }

    // 射门：贴身直线推穿——目标 = 球前 20cm，推球方向 = 瞄准线（不被接近轨迹带偏）
    ShootPlan sp = plan_shoot(wm, id);
#ifdef SIMURO5_HNNU_TRACE
    // 门前终结诊断：球离对方门 <40cm 且 active 贴球时，记射门决策内部状态 S 行，
    //   回答「拿到球为什么不射」——viable/开口角/质量/争抢态（C:\Strategy\gate_trace.csv）
    if (wm.ctx.dist_opp_goal(wm.ball.x) < 40.0) {
        static FILE *fp = nullptr;
        static long n = 0;
        if (!fp) fp = std::fopen("C:\\Strategy\\gate_trace.csv", "a");
        if (fp) {
            double gk_y = 90.0, gk_d = 1e9;
            for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
                double dgk = wm.ctx.dist_opp_goal(wm.opp[i].x);
                if (dgk < gk_d) { gk_d = dgk; gk_y = wm.opp[i].y; }
            }
            std::fprintf(fp, "S,%ld,%.1f,%.1f,%.1f,%d,%.2f,%.3f,%.1f,%d,%.1f\n",
                         n++, wm.ball.x, wm.ball.y, wm.ctx.dist_opp_goal(wm.ball.x),
                         sp.viable, sp.open_angle, sp.quality, sp.shot_dist, (int)contested, gk_y);
            std::fflush(fp);
        }
    }
#endif
    if (contested && kContestCharge > 0.5 && !wm.coop_pass_task.active &&
        !in_no_push_zone(wm.ball.x, wm.ball.y)) {
        const double bx = wm.ball.x, by = wm.ball.y;
        const double dx = bx - r.x, dy = by - r.y, d = std::hypot(dx, dy);
        const bool shot_behind = sp.viable && (dx * sp.dir_x + dy * sp.dir_y) > 0.0;
        if (d > 1e-6 && !shot_behind) {
            const double fwd = dx * ctx.attack_dir() / d;
            const double need = (ctx.dist_our_goal(bx) < kChargeOwnGuard) ? kChargeMinFwdOwn : kChargeMinFwd;
            if (fwd >= need) {
                motion::position(r, bx + dx / d * kChargeThrough, by + dy / d * kChargeThrough, motion::TM_PASS);
                TRACE_MARK(r);
                return;
            }
        }
    }
    // 配合进攻：队友射门机会比我高 0.15 以上且能安全接到 → 传给他（安全性在 pass.cpp 判定）
    CoopPass cp;
    if (wm.coop_pass_task.active) {
        const auto &task = wm.coop_pass_task;
        cp.viable = true; cp.receiver_id = task.receiver_id; cp.rx = task.rx; cp.ry = task.ry;
        double length = dist(wm.ball.x, wm.ball.y, cp.rx, cp.ry);
        if (length > 1e-6) { cp.dir_x = (cp.rx - wm.ball.x) / length; cp.dir_y = (cp.ry - wm.ball.y) / length; }
        cp.aim_rot = angle_to(0.0, 0.0, cp.dir_x, cp.dir_y);
    } else if (fixed_pass_actor && !had_pass_task && pass_context_safe_impl(wm)) {
        cp = plan_coop_pass(wm, id);
        cp.viable = cp.viable && cp.score > sp.quality + 0.15;
        // 门前横传优先（docs/17 的 P4）：球在对方门前一侧时，把球横送远门柱比
        //   「球前 40cm 的常规接应点」更值。复用同一套 pass_target_safe / prep 护栏，
        //   所以没有新开出任何危险传球的口子；质量分低于直接射门时仍会被顶掉。
        CoopPass cross = plan_cross(wm, id);
        if (cross.viable && cross.score > sp.quality + 0.15 &&
            (!cp.viable || cross.score >= cp.score)) {
            cp = cross;
        }
    }
#ifdef SIMURO5_HNNU_TRACE
    // 角区回做验证：记录 plan_coop_pass 是否找到门前接应点（viable/队友射门质量 score/自己射门质量 quality）
    {
        static FILE *fp = nullptr; static long n = 0;
        if (!fp) fp = std::fopen("C:\\Strategy\\pass_trace.csv", "a");
        if (fp) {
            std::fprintf(fp, "%ld,%.1f,%.1f,%.1f,%d,%.3f,%.3f,%d,%.1f\n",
                         n++, wm.ball.x, wm.ball.y, wm.ctx.dist_opp_goal(wm.ball.x),
                         cp.viable, cp.score, sp.quality, id, wm.threat_level);
            std::fflush(fp);
        }
    }
#endif
    const bool existing_coop_task = wm.coop_pass_task.active && wm.coop_pass_task.kind == PassTaskKind::Coop;
    const bool coop_preferred = existing_coop_task ||
        (cp.viable && !wm.in_penalty_exec && cp.score > sp.quality + 0.15);
    const bool coop_pass = coop_preferred &&
        (wm.coop_pass_task.active || pass_target_safe_impl(wm, id, cp.receiver_id, cp.rx, cp.ry, true));
    if (coop_pass) {
        sp.dir_x = cp.dir_x; sp.dir_y = cp.dir_y; sp.aim_rot = cp.aim_rot;
        sp.target_x = wm.ball.x - cp.dir_x * 8.0; sp.target_y = wm.ball.y - cp.dir_y * 8.0;
        sp.viable = true;
    }
    // 走廊拐弯：没射门机会、或机会窄到只有一条缝（边路看门本来就只有 12° 上下）时，
    //   改推「球前中路」而不是往缝里捅
    bool corridor_now = false;
    if (kCorridorEnabled >= 0.5 && !coop_pass && (!sp.viable || sp.open_angle < kCorridorShotOpen)) {
        ShootPlan ca = plan_corridor(wm, id);
        if (ca.viable) { sp = ca; corridor_now = true; }
    }
    // 点球执行期：瞄准方向只锁一次（每帧重算会让准备点漂移、折返时把球推偏）
    if (!corridor_now && wm.in_penalty_exec && sp.viable) {
        if (!wm.pen_aim_locked) {
            wm.pen_aim_locked = true;
            wm.pen_aim_rot = sp.aim_rot;
            wm.pen_dir_x = sp.dir_x;
            wm.pen_dir_y = sp.dir_y;
            wm.pen_aim_y = sp.aim_y;
        } else {
            sp.aim_rot = wm.pen_aim_rot;
            sp.dir_x = wm.pen_dir_x;
            sp.dir_y = wm.pen_dir_y;
            sp.aim_y = wm.pen_aim_y;
        }
    }
    // 射门机会闸门：≤70cm 无条件射；70~110cm 远射要 quality ≥ kShootNowQ（远射档已开启）
    // 走廊拐弯不走质量闸门：它本来就是在「射门没机会」时才出现的
    const double kShootNowQ = 0.35;
    bool shoot_now = corridor_now ||
                     (sp.viable && (sp.shot_dist <= 70.0 || sp.quality >= kShootNowQ || coop_pass));
    if (shoot_now && (coop_pass || corridor_now || wm.shoot_push_count < kMaxShootPushes)) {
        double bx = wm.ball.x, by = wm.ball.y;
        int this_side = (sp.aim_y > 90.0) ? 1 : -1;
        // 变角推射：已推 >=2 次且仍瞄同一侧 → 强制换另一侧（借墙方案不适用）
        if (!sp.bank && !coop_pass && !corridor_now && wm.shoot_push_count >= 2 && wm.shoot_push_last_side == this_side &&
            wm.shoot_push_last_side != 0) {
            double ogx = ctx.opp_goal_x(), ad2 = ctx.attack_dir();
            double oy = 90.0 - (sp.aim_y - 90.0);
            double dx = (ogx + ad2 * 5.0) - bx, dy = oy - by;
            double len = std::hypot(dx, dy);
            if (len > 1e-6) { dx /= len; dy /= len; sp.aim_rot = angle_to(0.0, 0.0, dx, dy); }
            sp.dir_x = dx; sp.dir_y = dy;
            this_side = -this_side;
        }
        const double prep_d = shoot_prep_dist(wm);   // 罚点球用更长的助跑
        double px = bx - sp.dir_x * prep_d;          // 准备点 = 球后 prep_d，落在瞄准线上
        double py = by - sp.dir_y * prep_d;
        // 准备点也不许落在角区（否则驱车过去会穿过球、把球顶进角里）
        if (!prep_point_ok(px, py) || (coop_pass && in_no_push_zone(px, py))) {
            if (coop_pass && wm.coop_pass_task.active) wm.coop_finish(CoopOutcome::PrepPoint);
            hold_out_of_corner(wm, r); return;
        }
        if (coop_pass && !wm.coop_pass_task.active) {
            wm.coop_pass_task = {};
            wm.coop_pass_task.active = true; wm.coop_pass_task.passer_id = id;
            wm.coop_pass_task.receiver_id = cp.receiver_id; wm.coop_pass_task.rx = cp.rx; wm.coop_pass_task.ry = cp.ry;
            wm.coop_pass_task.frames_left = kPassTaskFrames;
            wm.coop_pass_task.kind = PassTaskKind::Coop; wm.coop_created();
        }
        if (coop_pass && !pass_receiver_ready(wm)) {
            if (!contested || kContestNoWait < 0.5) { motion::stop(r); return; }
            wm.coop_finish(CoopOutcome::OpponentFirst);   // 争抢态：不停车等人，照原方向推
        }
        double db = dist(r.x, r.y, bx, by);
        double te_head = angle_diff(sp.aim_rot, r.rot);
        // 人在球的门侧后方：球−人在瞄准方向上的投影 >0（否则往前推是把球往回推）
        bool behind = ((bx - r.x) * sp.dir_x + (by - r.y) * sp.dir_y) > 0.0;
        bool near = db < prep_d + 6.0;
        bool ready = false;
        // 罚点球执行：在球后且够近 → 就地转正后直接冲穿球，绝不倒车助跑
        if (wm.in_penalty_exec) {
            if (behind && near) {
                if (std::fabs(te_head) <= kPrepAngTol) {
                    ready = true;                          // 朝向够准 → 立刻推穿
                } else if (wm.shoot_align_frames >= kShootAlignTimeout) {
                    ready = true;                          // 兜底：宁可打偏也别重发
                    wm.shoot_align_frames = 0;
                } else {
                    ++wm.shoot_align_frames;
                    motion::position_aligned(r, r.x, r.y, sp.aim_rot, kPrepPosTol, kPrepAngTol);
                    return;                                // 只原地转正
                }
            } else if (behind) {
                wm.shoot_align_frames = 0;
                motion::position(r, px, py, motion::TM_PASS);
                return;
            } else {
                wm.shoot_align_frames = 0;
                motion::position_aligned(r, px, py, sp.aim_rot, kPrepPosTol, kPrepAngTol);
                return;
            }
        } else if (kNoAlignWait > 0.5) {
            // 在球后就直接推穿；不在球后赶去球后点（经过型，不对准）
            wm.shoot_align_frames = 0;
            if (behind) ready = true;
            else { motion::position(r, px, py, motion::TM_PASS); ready = false; }
        } else if (contested && behind && kContestPush > 0.5) {
            wm.shoot_align_frames = 0;
            ready = true;
        } else if (behind && near) {
            if (std::fabs(te_head) <= kPrepAngTol) {
                ready = true;
            } else if (wm.shoot_align_frames >= kShootAlignTimeout) {
                ready = true;                  // 超时兜底
                wm.shoot_align_frames = 0;
            } else {
                ++wm.shoot_align_frames;
                motion::position_aligned(r, r.x, r.y, sp.aim_rot, kPrepPosTol, kPrepAngTol);
                return;
            }
        } else {
            wm.shoot_align_frames = 0;
            ready = motion::position_aligned(r, px, py, sp.aim_rot, kPrepPosTol, kPrepAngTol);
            if (ready && !near) ready = false;  // 到了准备点仍够不着 → 继续靠近
        }
        // 禁止过冲后反向穿球：球在推球方向反面时我已在球的球门侧，须先绕弧线回球正后方
        {
            const double side = (bx - r.x) * sp.dir_x + (by - r.y) * sp.dir_y;
            if (side < -3.0) {
                const double nx = -sp.dir_y, ny = sp.dir_x;          // 垂直瞄准方向
                double cx = bx + nx * 22.0, cy = by + ny * 22.0;    // 侧向绕行点
                if (dist(r.x, r.y, cx, cy) >
                    dist(r.x, r.y, bx - nx * 22.0, by - ny * 22.0)) {
                    cx = bx - nx * 22.0; cy = by - ny * 22.0;       // 选更近的一侧绕
                }
                wm.shoot_align_frames = 0;
                motion::position(r, cx, cy, motion::TM_PASS);       // 绕行，绝不穿球
                return;
            }
        }
        if (ready) {
            wm.shoot_align_frames = 0;
            motion::position(r, bx + sp.dir_x * 20.0, by + sp.dir_y * 20.0, motion::TM_PASS);
            if (coop_pass && !wm.coop_pass_task.observing_push && db < kPassReceiveDistance) {
                auto &task = wm.coop_pass_task; task.observing_push = true;
                task.push_ball_x = bx; task.push_ball_y = by; task.push_dir_x = sp.dir_x; task.push_dir_y = sp.dir_y;
            }
            if (!coop_pass && !corridor_now && wm.shoot_push_cd <= 0 && std::hypot(wm.ball.vx, wm.ball.vy) > 5.0) {
                ++wm.shoot_push_count;
                wm.shoot_push_cd = 20;
                wm.shoot_push_last_side = this_side;
            }
        }
        return;
    }

    double db = dist(r.x, r.y, wm.ball.x, wm.ball.y);
    double opp_d = 1e9, opp_y = 90.0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y);
        if (d < opp_d) { opp_d = d; opp_y = wm.opp[i].y; }
    }
    const bool own_ball = db < 15.0 && db < opp_d;
    PassPlan pp = fixed_pass_actor ? plan_pass(wm, id) : PassPlan{};
    if (pp.viable && !coop_preferred) {
        if (!wm.coop_pass_task.active && !had_pass_task && pass_target_safe_impl(wm, id, pp.receiver_id, pp.target_x, pp.target_y, true)) {
            wm.coop_pass_task = {};
            wm.coop_pass_task.active = true; wm.coop_pass_task.passer_id = id;
            wm.coop_pass_task.receiver_id = pp.receiver_id; wm.coop_pass_task.rx = pp.target_x; wm.coop_pass_task.ry = pp.target_y;
            wm.coop_pass_task.frames_left = kPassTaskFrames;
            wm.coop_pass_task.kind = PassTaskKind::Ordinary; wm.coop_created();
        }
        if (wm.coop_pass_task.active && wm.coop_pass_task.kind == PassTaskKind::Ordinary) {
            execute_ordinary_pass_prep(wm, id);
        } else motion::position(r, pp.target_x, pp.target_y);
        return;
    }

    if (own_ball) {
        // 围困检测：球周围 25cm 内 ≥2 个对手或最近对手 <12cm → 优先传球，无则带球离场
        int swarm = 0, near_i = -1;
        double opp_near = 1e9;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            double d = dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y);
            if (d < 25.0) swarm++;
            if (d < opp_near) { opp_near = d; near_i = i; }
        }
        if (swarm >= 2 || opp_near < 12.0) {
            PassPlan pp2 = fixed_pass_actor ? plan_pass(wm, id) : PassPlan{};
            if (pp2.viable) { motion::position(r, pp2.target_x, pp2.target_y); return; }
            if (near_i >= 0 && opp_near > 1e-6) {
                double dx = wm.ball.x - wm.opp[near_i].x;
                double dy = wm.ball.y - wm.opp[near_i].y;
                double len = std::hypot(dx, dy);
                if (len > 1e-6) { dx /= len; dy /= len; }
                double te_e = angle_diff(angle_to(r.x, r.y, wm.ball.x, wm.ball.y), r.rot);
                if (db < 12.0 && std::fabs(te_e) < 40.0) {
                    motion::position(r, wm.ball.x + dx * 25.0, wm.ball.y + dy * 25.0, motion::TM_PASS);   // 带离围困
                } else {
                    motion::position(r, wm.ball.x - dx * 20.0, wm.ball.y - dy * 20.0);
                }
                return;
            }
        }
        // 正常带球推进：朝门柱开口方向（避开最近防守者）走"贴球后直线推穿"，与射门同款
        double ogx = ctx.opp_goal_x();
        double aim_y = (opp_y > 90.0) ? 106.0 : 74.0;   // 防守者偏下 → 带上柱口
        double dirx = (ogx + ctx.attack_dir() * 5.0) - wm.ball.x;
        double diry = aim_y - wm.ball.y;
        double len = std::hypot(dirx, diry);
        if (len > 1e-6) { dirx /= len; diry /= len; }
        double aim_rot2 = angle_to(0.0, 0.0, dirx, diry);
        double te_head2 = angle_diff(aim_rot2, r.rot);
        bool behind2 = ((wm.ball.x - r.x) * dirx + (wm.ball.y - r.y) * diry) > 0.0;
        if (kNoAlignWait > 0.5 && behind2) {
            wm.shoot_align_frames = 0;
            motion::position(r, wm.ball.x + dirx * 20.0, wm.ball.y + diry * 20.0, motion::TM_PASS);
        } else if (db < 12.0 && behind2 && std::fabs(te_head2) <= kDribAngTol) {
            motion::position(r, wm.ball.x + dirx * 20.0, wm.ball.y + diry * 20.0, motion::TM_PASS);   // 带球推进
        } else if (!prep_point_ok(wm.ball.x - dirx * 20.0, wm.ball.y - diry * 20.0)) {
            // 球后站位点在角区 → 不绕球后（会穿过球把球顶进角里）
            hold_out_of_corner(wm, r);
        } else if (db < 16.0 && behind2 && wm.shoot_align_frames < kShootAlignTimeout) {
            if (++wm.shoot_align_frames >= kShootAlignTimeout) {
                motion::position(r, wm.ball.x - dirx * 20.0, wm.ball.y - diry * 20.0);
            } else {
                motion::position_aligned(r, r.x, r.y, aim_rot2, kPrepPosTol, kPrepAngTol);
            }
        } else {
            motion::position(r, wm.ball.x - dirx * 20.0, wm.ball.y - diry * 20.0,
                             kNoAlignWait > 0.5 ? motion::TM_PASS : motion::TM_STOP);
        }
    } else {
        // 球不在脚下/争抢中：追预测球位（带减速）；预测位在对方罚球区内则追到外沿等球弹出
        BallState chased = chase_target(wm);
        // 门前 100cm 锥形区：直线追球必穿门区，故追到门区外沿等球弹出
        if (wm.ctx.dist_opp_goal(chased.x) < 100.0 && std::fabs(chased.y - 90.0) < 45.0) {
            double bspeed = std::hypot(wm.ball.vx, wm.ball.vy);
            bool rebound_rush = (bspeed < kReboundRushSpeed) &&
                                dist(r.x, r.y, chased.x, chased.y) < kReboundRushDist;
            if (!rebound_rush) {
                chased.x = wm.ctx.opp_goal_x() - wm.ctx.attack_dir() * 85.0;
                chased.y = clamp(chased.y, 72.5, 107.5);
            }
        }
        move_avoiding(wm, r, id, chased.x, chased.y, true);
    }
}

// 人盯人执行体：盯住对手 t 的整套动作（门区不追/贴身逼抢/堵传球线/门侧站位/禁区纪律）
static void run_mark_body(WorldModel &wm, int id, int t) {
    if (t < 0 || t >= PLAYERS_PER_SIDE) return;
    // 被盯者缩在对方门区且球不在门区时不追进去（门区 2+ 人/停留超时 → 罚点球），改站前缘外
    if (in_opp_goal_area(wm.ctx, wm.opp[t].x, wm.opp[t].y) &&
        !in_opp_goal_area(wm.ctx, wm.ball.x, wm.ball.y)) {
        double fx = 0.0, fy = 0.0;
        opp_goal_area_front(wm.ctx, wm.ball.y, fx, fy);
        motion::position(wm.home[id], fx, fy);
        return;
    }
    // 被盯者离球 <25cm（控球/即将接球）时放弃站连线，直接冲球贴身逼抢
    double d_opp_ball = dist(wm.ball.x, wm.ball.y, wm.opp[t].x, wm.opp[t].y);
    // 球在我方门区内不逼抢（门前交给门将，否则 2+ 人违规），站门区外等解围
    if (d_opp_ball < 25.0 && !in_goal_area(wm.ctx, wm.ball.x, wm.ball.y)) {
        double d_home_goal = wm.ctx.dist_our_goal(wm.home[id].x);
        double d_ball_goal = wm.ctx.dist_our_goal(wm.ball.x);
        if (d_home_goal < d_ball_goal && wm.stealer_id == id) {
            motion::chase_ball(wm.home[id], chase_target(wm));
            return;
        }
    }
    double gx = wm.ctx.our_goal_x(), gy = 90.0;
    double px = wm.opp[t].x + wm.opp_vx[t] * mark_lead();
    double py = wm.opp[t].y + wm.opp_vy[t] * mark_lead();
    double d_ball = dist(wm.ball.x, wm.ball.y, wm.opp[t].x, wm.opp[t].y);
    double ref_x = gx, ref_y = gy;                       // 默认站门侧
    if (d_ball > 15.0 && d_ball < mark_pass_lane_dist()) {
        ref_x = wm.ball.x; ref_y = wm.ball.y;            // 堵传球线
    }
    double dx = ref_x - px, dy = ref_y - py;
    double len = std::hypot(dx, dy);
    if (len > 1e-6) { dx /= len; dy /= len; }
    double mx = px + dx * mark_dist();
    double my = py + dy * mark_dist();
    // 站位点若落入己方罚球区（只有门将能进）→ 推到罚球区前缘 5cm
    if (in_penalty_area(wm.ctx, mx, my)) {
        mx = wm.ctx.our_goal_x() + wm.ctx.attack_dir() * 85.0;
        my = clamp(py, 72.5, 107.5);
    }
    mx = clamp(mx, 0.0, TeamContext::FIELD_LENGTH);
    my = clamp(my, 0.0, TeamContext::FIELD_WIDTH);
    // 对方门区禁入：盯人站位不得进入对方门区（防 2+ 人违规判点球）
    clamp_out_opp_goal_area(wm.ctx, mx, my);
    motion::position(wm.home[id], mx, my);
    return;
}

void run_passive(WorldModel &wm, int id) {
    if (run_pass_receiver(wm, id)) return;
    // 门前协防：球在我方门前 80cm 内近静止且对方已逼近时，钉到「球-门连线」护门点与门将包夹
    if (wm.ctx.dist_our_goal(wm.ball.x) < 80.0 &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 3.0) {
        double opp_dmin = opp_clear_dist(wm);
        if (opp_dmin < 100.0) {
            // 门前清道夫：仅当本角色比球更靠己门（已在球门侧）时才朝球推出去
            //   从门侧推球只会把球顶向场内，不可能乌龙；球在场侧时保持原护栏
            double dbp = dist(wm.ball.x, wm.ball.y, wm.home[id].x, wm.home[id].y);
            if (wm.ctx.dist_our_goal(wm.ball.x) < kDoorPushDist &&
                !in_goal_area_rule(wm.ctx, wm.ball.x, wm.ball.y, 5.0, 5.0) &&   // 球在裁判门区：交门将（进去即计点球）
                wm.ctx.dist_our_goal(wm.home[id].x) < wm.ctx.dist_our_goal(wm.ball.x)) {
                motion::chase_ball(wm.home[id], chase_target(wm));   // 门侧推球：只会推离己门
                return;
            }
            // 门线球站定防乌龙：球贴门线且本角色贴球 → 停轮站定用身体封角，绝不碰球拖动
            //   （只在场侧生效，球门侧已由上面清道夫接管）
            if (wm.ctx.dist_our_goal(wm.ball.x) < 15.0 && dbp < 14.0) {
                TRACE_MARK(wm.home[id]);
                wm.home[id].vl = 0.0;
                wm.home[id].vr = 0.0;
                return;
            }
            double cx = 0.0, cy = 0.0;
            goal_cover_point(wm, cx, cy);
            const RobotState &me = wm.home[id];
            if (kCoverDetour > 0.5 &&
                wm.ctx.dist_our_goal(me.x) > wm.ctx.dist_our_goal(wm.ball.x) - 3.0 &&   // 人在球的场侧
                seg_point_dist(me.x, me.y, cx, cy, wm.ball.x, wm.ball.y) < kCoverClear) {
                double side = (std::fabs(me.y - wm.ball.y) > 1.0) ? (me.y > wm.ball.y ? 1.0 : -1.0)
                                                                  : (wm.ball.y < 90.0 ? 1.0 : -1.0);
                const double g = -wm.ctx.attack_dir();   // 朝己门
                double wx = wm.ball.x + g * 6.0, wy = wm.ball.y + side * kCoverDetourLat;
                if (wy < 5.0 || wy > 175.0) wy = wm.ball.y - side * kCoverDetourLat;   // 贴边：换另一侧绕
                if (in_goal_area_rule(wm.ctx, wx, wy)) {   // 绕行点进裁判门区：宁可停车也不穿球
                    TRACE_MARK(wm.home[id]);
                    wm.home[id].vl = 0.0;
                    wm.home[id].vr = 0.0;
                    return;
                }
                motion::position(wm.home[id], wx, wy);
                return;
            }
            motion::position(wm.home[id], cx, cy);
            return;
        }
    }
    if (wm.threat_level >= 0.6) {
        int t = wm.mark_assign_valid ? wm.mark_assign[id]
                                  : pick_mark_target(wm, wm.mark_target);
        wm.mark_target = t;
        run_mark_body(wm, id, t);
    }
    // 争抢上抢：威胁分低时也可出手（只在球门侧、球不在罚球区、离己门 kPassivePressDepth 内）
    if (kPassivePress > 0.5 && !in_penalty_area(wm.ctx, wm.ball.x, wm.ball.y) &&
        wm.ctx.dist_our_goal(wm.ball.x) < kPassivePressDepth) {
        const RobotState &me = wm.home[id];
        double my_d = dist(me.x, me.y, wm.ball.x, wm.ball.y), opp_d = 1e9;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i)
            opp_d = std::min(opp_d, dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y));
        if (opp_d < kContestOppDist && my_d < kContestReach && wm.stealer_id == id &&
            wm.ctx.dist_our_goal(me.x) < wm.ctx.dist_our_goal(wm.ball.x)) {
            motion::chase_ball(wm.home[id], chase_target(wm));
            return;
        }
    }
    DefensePlan dp = plan_defense(wm, id);
    // 到位迎球：断球点在球来路上时到点后转正，机头朝球来方向（否则球被横顶、动量不抵消）
    if (dp.face_incoming && kDefFaceIncoming > 0.5)
        motion::arrive_facing(wm.home[id], dp.target_x, dp.target_y, dp.aim_rot,
                              kDefArriveDist, kDefFaceAngTol);
    else
        motion::position(wm.home[id], dp.target_x, dp.target_y);
}

// 分道压迫进攻：ASSIST 管上半道、MIDFIELD 管下半道，从球后拱球（推进方向取射门/借墙方案）
namespace {
constexpr bool kSwarmEnabled = true;
TUNABLE(kSwarmOwnGuard, 60.0);  // 球离我方门线近于此（cm）→ 不压迫，交回防守
TUNABLE(kLaneShare, 18.0);  // 中路共管带半宽（cm）
TUNABLE(kHerdBack, 11.0);  // 球后落位距离（cm）
TUNABLE(kHerdLatTol, 6.5);  // 横向偏差在此内视为对准 → 推穿（cm）
TUNABLE(kHerdThrough, 22.0);  // 推穿目标在球前方的距离（cm）
TUNABLE(kHerdSide, 17.0);  // 人在球前时横向绕行距离（cm）
TUNABLE(kWeakBack, 22.0);  // 弱侧跟进：落后球的距离（cm）
TUNABLE(kWeakLaneY, 42.0);  // 弱侧跟进：离中线的距离（cm）
TUNABLE(kEscortBack, 16.0);  // 护送：落后球的距离（cm）
TUNABLE(kEscortSide, 22.0);  // 护送：横向错开（cm）
// ── 错层 / 分道（2026-10-09）──
//   取证：两个翼挤在同一条"推穿轴"上排队是 23.9% 帧贴住的来源（77.6% 的紧贴对两车都在球后方）。
//   下面三个主力分支原来一次都没用 lane，目标横向零偏移 ⇒ 两车落点只差 0~10cm（接触 8cm）。
TUNABLE(kHerdLane, 16.0);    // cm：两翼同时在球附近时，各自的横向错开量（各偏这么多）
TUNABLE(kHerdNear, 70.0);    // cm：另一翼离球在此内才算"会挤到一起"，才做横向错开
TUNABLE(kHerdYield, 12.0);   // cm：另一翼离球近这么多，本翼就让位去弱侧保护点
TUNABLE(kOppBoxMargin, 10.0);  // 对方门区外扩余量（cm）

bool near_opp_box(const TeamContext &ctx, double x, double y) {
    return ctx.dist_opp_goal(x) < 50.0 + kOppBoxMargin &&
           std::fabs(y - 90.0) < 27.5 + kOppBoxMargin;
}

TUNABLE(kOppBoxDetourPad, 8.0);   // 绕行角点离外扩门区的余量（cm）
void opp_box_detour(const TeamContext &ctx, double rx, double ry, double &tx, double &ty) {
    const double depth = 50.0 + kOppBoxMargin, half = 27.5 + kOppBoxMargin;
    const double front_x = ctx.opp_goal_x() - ctx.attack_dir() * (depth + kOppBoxDetourPad);
    if (near_opp_box(ctx, rx, ry)) { tx = front_x; ty = ry; return; }
    bool cross = false;
    for (int k = 1; k <= 20 && !cross; ++k) {
        double s = k / 20.0;
        cross = near_opp_box(ctx, rx + (tx - rx) * s, ry + (ty - ry) * s);
    }
    if (!cross) return;
    const bool beside = ctx.dist_opp_goal(rx) < depth;   // 人在门区侧面（没到前沿外）
    const double side = ((beside ? ry : ty) >= 90.0) ? 1.0 : -1.0;
    tx = front_x;
    ty = 90.0 + side * (half + kOppBoxDetourPad);
}

void herd_direction(const WorldModel &wm, int id, double &ux, double &uy) {
    ShootPlan sp = plan_shoot(wm, id);
    if (sp.viable) { ux = sp.dir_x; uy = sp.dir_y; return; }
    ShootPlan bk = plan_bank_carry(wm);
    if (bk.viable) { ux = bk.dir_x; uy = bk.dir_y; return; }
    double dx = wm.ctx.opp_goal_x() - wm.ball.x, dy = 90.0 - wm.ball.y;
    double len = std::hypot(dx, dy);
    if (len < 1e-6) { ux = wm.ctx.attack_dir(); uy = 0.0; return; }
    ux = dx / len; uy = dy / len;
}

bool holds_ball(const WorldModel &wm, int i, double ux, double uy) {
    double ox = wm.home[i].x - wm.ball.x, oy = wm.home[i].y - wm.ball.y;
    double behind = -(ox * ux + oy * uy);
    double side = std::fabs(ox * -uy + oy * ux);
    return behind > 0.0 && behind < 18.0 && side < 9.0;
}

void swarm_move(WorldModel &wm, int id, double tx, double ty) {
    const TeamContext &ctx = wm.ctx;
    tx = clamp(tx, 4.0, TeamContext::FIELD_LENGTH - 4.0);
    ty = clamp(ty, 4.0, TeamContext::FIELD_WIDTH - 4.0);
    if (near_opp_box(ctx, tx, ty))
        tx = ctx.opp_goal_x() - ctx.attack_dir() * (58.0 + kOppBoxMargin);
    opp_box_detour(ctx, wm.home[id].x, wm.home[id].y, tx, ty);
    motion::position(wm.home[id], tx, ty, motion::TM_PASS);
}

bool run_swarm(WorldModel &wm, int id, double lane) {
    if (!kSwarmEnabled) return false;
    const TeamContext &ctx = wm.ctx;
    if (wm.runtime_phase != RuntimePhase::Running || wm.in_penalty_exec) return false;
    if (wm.coop_pass_task.active &&
        (id == wm.coop_pass_task.passer_id || id == wm.coop_pass_task.receiver_id)) return false;
    if (wm.coop_ball_control.active && id == wm.coop_ball_control.receiver_id) return false;
    if (id == wm.sweeper_id || !push_allowed(wm)) return false;
    const double bx = wm.ball.x, by = wm.ball.y, ad = ctx.attack_dir();
    if (ctx.dist_our_goal(bx) < kSwarmOwnGuard) return false;

    if (near_opp_box(ctx, bx, by)) {
        // 门前补射：球贴对方门线 kReboundZoneLo~kReboundZoneHi cm 且还在滚（门将刚扑出的反弹球）
        //   → 负责这半边的队员朝门穿球补一脚；离门更远的慢球 / 门区内停球 → 走下方「门外等二点」
        //   （交给别人 + 防 2+ 人门区判点球）
        const double bspeed = std::hypot(wm.ball.vx, wm.ball.vy);
        const double dg = ctx.dist_opp_goal(bx);
        if (bspeed > kReboundMinSpeed && bspeed < kReboundRushSpeed &&
            (by - 90.0) * lane >= 0.0 && dg >= kReboundZoneLo && dg < kReboundZoneHi) {
            double dx = ctx.opp_goal_x() - bx, dy = 90.0 - by;
            double len = std::hypot(dx, dy);
            if (len > 1e-6) { dx /= len; dy /= len; }
            motion::position(wm.home[id], bx + dx * kReboundRushDepth, by + dy * kReboundRushDepth, motion::TM_PASS);
#ifdef SIMURO5_HNNU_TRACE
            { static FILE *fp = nullptr; static long n = 0;
              if (!fp) fp = std::fopen("C:\\Strategy\\rebound_trace.csv", "a");
              if (fp) { std::fprintf(fp, "%ld,%.1f,%.1f,%.1f,%d,%.1f\n",
                  n++, bx, by, bspeed, id, lane); std::fflush(fp); } }
#endif
            TRACE_MARK(wm.home[id]);
            return true;
        }
        swarm_move(wm, id, ctx.opp_goal_x() - ad * (58.0 + kOppBoxMargin), 90.0 + lane * 32.0);  TRACE_MARK(wm.home[id]);
        return true;
    }
    if ((by - 90.0) * lane < -kLaneShare) {
        swarm_move(wm, id, bx - ad * kWeakBack, 90.0 + lane * kWeakLaneY);  TRACE_MARK(wm.home[id]);
        return true;
    }

    double ux = 0.0, uy = 0.0;
    herd_direction(wm, id, ux, uy);
    const double nx = -uy, ny = ux;
    const RobotState &r = wm.home[id];
    const double ox = r.x - bx, oy = r.y - by;
    const double behind = -(ox * ux + oy * uy);        // >0：人在球后（推进方向的反侧）
    const double side_off = ox * nx + oy * ny;          // 人在推进线哪一侧、偏多少
    const double side = (side_off >= 0.0) ? 1.0 : -1.0;

    // ── 错层 / 分道（2026-10-09）──
    // ① 另一翼明显更近 → 本翼让位去弱侧保护点（错层，不是紧环 —— docs/03 对球权中心化 v1 的结论）；
    // ② 另一翼也在球附近 → 本翼按 lane 横向错开 kHerdLane，两翼各站一条线；
    // ③ 另一翼离得远 → 保持原样（别为了不在场的人让出推进轴）。
    int wing = -1;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (i == id || wm.role[i] == ROLE_GOALIE) continue;
        if (wm.role[i] == ROLE_ASSIST || wm.role[i] == ROLE_MIDFIELD) { wing = i; break; }
    }
    double lat = 0.0;                 // 本帧横向错开量（沿 nx,ny）
    double side_out = side;           // 绕球 / 护送时往哪一侧让
    if (kHerdLane > 0.0 && wing >= 0) {          // kHerdLane = 0 → 整条错层/让位关闭（回滚）
        const double d_me = dist(r.x, r.y, bx, by);
        const double d_wing = dist(wm.home[wing].x, wm.home[wing].y, bx, by);
        if (d_wing + kHerdYield < d_me) {                        // ① 让位
            swarm_move(wm, id, bx - ad * kWeakBack, 90.0 + lane * kWeakLaneY);
            TRACE_MARK(wm.home[id]);
            return true;
        }
        if (d_wing < kHerdNear) { lat = lane * kHerdLane; side_out = lane; }   // ② 错开
    }

    if (!holds_ball(wm, id, ux, uy)) {
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            if (i == id || wm.role[i] == ROLE_GOALIE) continue;
            if (!holds_ball(wm, i, ux, uy)) continue;
            swarm_move(wm, id, bx - ux * kEscortBack + nx * side_out * kEscortSide,
                       by - uy * kEscortBack + ny * side_out * kEscortSide);  TRACE_MARK(wm.home[id]);
            return true;
        }
    }

    if (behind > 0.0 && std::fabs(side_off) < kHerdLatTol) {
        swarm_move(wm, id, bx + ux * kHerdThrough + nx * lat,
                   by + uy * kHerdThrough + ny * lat);  TRACE_MARK(wm.home[id]);
    } else if (behind > -3.0) {
        double back = kHerdBack + 0.5 * std::fabs(side_off);
        double px = bx - ux * back + nx * lat, py = by - uy * back + ny * lat;
        if (!prep_point_ok(px, py)) return false;
        swarm_move(wm, id, px, py);  TRACE_MARK(wm.home[id]);
    } else {
        // 人在球前面：先横绕到球侧，绝不直线穿过球
        swarm_move(wm, id, bx - ux * 4.0 + nx * side_out * kHerdSide,
                   by - uy * 4.0 + ny * side_out * kHerdSide);  TRACE_MARK(wm.home[id]);
    }
    return true;
}
}  // anonymous namespace

// 门前横传的**包抄者**走位（docs/17 的 P4）。
//   条件与 run_active 里注入 plan_cross 的条件同源（同一个 plan_cross），
//   保证"有人横传"和"有人去门柱"同时发生，不会各走各的。
//   包抄点由 plan_cross 直接给出（远门柱、且压在对方门区**外**，那里有平台判点球的红线）。
//   ⚠️ 这里**故意不加** clamp_out_opp_goal_area：那道 clamp 会把点拉回禁区外沿，
//      它正是"门柱包抄"以前永远实现不了的原因。点球红线由 plan_cross 的 kCrossOutPad 保证。
bool run_cross_finisher(WorldModel &wm, int id) {
    if (!wm.we_have_ball) return false;          // 只在"我们持球、要组织横传"时才去包抄
    if (!pass_context_safe(wm)) return false;
    CoopPass c = plan_cross(wm, wm.active_id);   // 门前一侧 + 横传几何 + kCrossEnabled 都在这道闸里
    if (!c.viable || c.receiver_id != id) return false;

    RobotState &r = wm.home[id];
    // 已就位 → 就地停住等球，别再往球门里蹭（省下点球计时的帧数）
    if (dist(r.x, r.y, c.rx, c.ry) <= 6.0) { motion::position(r, c.rx, c.ry, motion::TM_STOP); return true; }
    if (kCrossFaceOn < 0.5) { motion::position(r, c.rx, c.ry, motion::TM_PASS); return true; }

    // 到位顺带转正：用现成的 plan_shoot 算"球在包抄点时"该朝哪（拿不到就面向球门）
    double aim = angle_to(r.x, r.y, wm.ctx.opp_goal_x(), 90.0);
    WorldModel tmp = wm;
    tmp.ball.x = c.rx; tmp.ball.y = c.ry; tmp.ball.vx = 0.0; tmp.ball.vy = 0.0;
    tmp.in_penalty_exec = false;
    ShootPlan ps = plan_shoot(tmp, id);
    if (ps.viable) aim = ps.aim_rot;
    motion::arrive_facing(r, c.rx, c.ry, aim, kCrossArriveDist, kCrossArriveAng);
    return true;
}

void run_assist(WorldModel &wm, int id) {
    if (run_pass_receiver(wm, id)) return;
    if (run_cross_finisher(wm, id)) return;      // 门前横传包抄（优先于常规"边路尾随"）
    if (run_swarm(wm, id, +1.0)) return;
    if ((wm.threat_level > 0.3 || wm.no_possession_frames >= 2) && wm.counter_attack_frames <= 0) {
        double bbx = wm.ball.x;
        bool ball_opp_half = (wm.ctx.attack_dir() > 0) ? (bbx > 110.0) : (bbx < 110.0);
        if (ball_opp_half && !wm.we_have_ball &&
            !(wm.ctx.dist_opp_goal(bbx) < 100.0 && std::fabs(wm.ball.y - 90.0) < 45.0) &&
            dist(wm.home[id].x, wm.home[id].y, bbx, wm.ball.y) < 130.0) {
            double pxx = wm.ball_pred.x, pyy = wm.ball_pred.y;
            if (wm.ctx.dist_opp_goal(pxx) < 100.0 && std::fabs(pyy - 90.0) < 45.0) {
                pxx = wm.ctx.opp_goal_x() - wm.ctx.attack_dir() * 85.0;
                pyy = clamp(pyy, 72.5, 107.5);
            }
            motion::position(wm.home[id], pxx, pyy, motion::TM_PASS);   // 压上追球
            return;
        }

        if (id == wm.sweeper_id) {
            motion::position(wm.home[id], wm.sweeper_x, wm.sweeper_y);
            return;
        }
        if (shot_on_target(wm) && ball_danger_speed(wm) > rebound_min_danger()) {
            double rx = 0.0, ry = 0.0;
            rebound_point(wm, +30.0, rx, ry);
            motion::position(wm.home[id], rx, ry);
            return;
        }
        if (wm.mark_assign_valid) {
            run_mark_body(wm, id, wm.mark_assign[id]);
            if (wm.mark_assign[id] >= 0) return;
        }
        double dtx = 0.0, dty = 0.0;
        if (double_team_point(wm, id, dtx, dty)) {
            motion::position(wm.home[id], dtx, dty);
            return;
        }
        DefensePlan dp = plan_defense(wm, id);
        double ty = clamp(dp.target_y + 30.0, 20.0, 160.0);
        if (dp.face_incoming && kDefFaceIncoming > 0.5)   // 到位迎球
            motion::arrive_facing(wm.home[id], dp.target_x, ty, dp.aim_rot,
                                  kDefArriveDist, kDefFaceAngTol);
        else
            motion::position(wm.home[id], dp.target_x, ty);
        return;
    }
    // 进攻分支：站助攻点，先躲敌人再与中场在 Y 轴上互相推开
    constexpr double THREAT_RADIUS = 30.0;   // 敌方威胁检测半径 cm
    constexpr double MAX_OFFSET     = 20.0;   // 最大允许横向偏移 cm
    constexpr double FIELD_MARGIN   = 6.0;    // 目标点离边线最小距离 cm
    constexpr double TEAM_SPACING   = 25.0;   // 与中场的最小 Y 间距（防挤堆）
    double tx = wm.assist_x;                  // X 保持 situation.cpp 输出
    double ty = spread_y(wm, wm.assist_x, wm.assist_y, THREAT_RADIUS, MAX_OFFSET);   // 躲敌人
    if (fabs(ty - wm.mid_y) < TEAM_SPACING) {
        ty = wm.mid_y + ((ty >= wm.mid_y) ? TEAM_SPACING : -TEAM_SPACING);
    }
    tx = clamp(tx, FIELD_MARGIN, TeamContext::FIELD_LENGTH - FIELD_MARGIN);
    ty = clamp(ty, FIELD_MARGIN, TeamContext::FIELD_WIDTH  - FIELD_MARGIN);
    clamp_out_opp_goal_area(wm.ctx, tx, ty);
    motion::position(wm.home[id], tx, ty);
}

void run_midfield(WorldModel &wm, int id) {
    if (run_pass_receiver(wm, id)) return;
    if (run_swarm(wm, id, -1.0)) return;
    if ((wm.threat_level > 0.3 || wm.no_possession_frames >= 2) && wm.counter_attack_frames <= 0) {
        if (id == wm.sweeper_id) {
            motion::position(wm.home[id], wm.sweeper_x, wm.sweeper_y);
            return;
        }
        if (shot_on_target(wm) && ball_danger_speed(wm) > rebound_min_danger()) {
            double rx = 0.0, ry = 0.0;
            rebound_point(wm, -30.0, rx, ry);
            motion::position(wm.home[id], rx, ry);
            return;
        }
        if (wm.mark_assign_valid) {
            run_mark_body(wm, id, wm.mark_assign[id]);
            if (wm.mark_assign[id] >= 0) return;
        }
        double dtx = 0.0, dty = 0.0;
        if (double_team_point(wm, id, dtx, dty)) {
            motion::position(wm.home[id], dtx, dty);
            return;
        }
        // 球在对方半场（对手解围/球权争夺中）：不去对方门前截球——球-门连线
        //   截点落对方门前/门区，会把 MID 拉进门区（sim debug f9109 实测
        //   MID 滞留门区 13 帧、与 ACTIVE 2+ 人违规 2.2 次/场），站中线等球弹回。
        double bbx2 = wm.ball.x;
        bool ball_opp_half2 = (wm.ctx.attack_dir() > 0) ? (bbx2 > 110.0) : (bbx2 < 110.0);
        if (ball_opp_half2) {
            motion::position(wm.home[id], 110.0, clamp(wm.home[id].y, 20.0, 160.0));
            return;
        }
        DefensePlan dp = plan_defense(wm, id);
        double ty = clamp(dp.target_y - 30.0, 20.0, 160.0);
        if (dp.face_incoming && kDefFaceIncoming > 0.5)   // 到位迎球
            motion::arrive_facing(wm.home[id], dp.target_x, ty, dp.aim_rot,
                                  kDefArriveDist, kDefFaceAngTol);
        else
            motion::position(wm.home[id], dp.target_x, ty);
        return;
    }
    constexpr double THREAT_RADIUS = 30.0;   // 敌方威胁检测半径 cm
    constexpr double MAX_OFFSET     = 20.0;   // 最大允许横向偏移 cm
    constexpr double FIELD_MARGIN   = 6.0;    // 目标点离边线最小距离 cm
    constexpr double TEAM_SPACING   = 25.0;   // 与助攻的最小 Y 间距（防挤堆）
    double tx = wm.mid_x;                     // X 保持 situation.cpp 输出
    double ty = spread_y(wm, wm.mid_x, wm.mid_y, THREAT_RADIUS, MAX_OFFSET);   // 躲敌人
    if (fabs(ty - wm.assist_y) < TEAM_SPACING) {
        ty = wm.assist_y + ((ty >= wm.assist_y) ? TEAM_SPACING : -TEAM_SPACING);
    }
    tx = clamp(tx, FIELD_MARGIN, TeamContext::FIELD_LENGTH - FIELD_MARGIN);
    ty = clamp(ty, FIELD_MARGIN, TeamContext::FIELD_WIDTH  - FIELD_MARGIN);
    clamp_out_opp_goal_area(wm.ctx, tx, ty);
    motion::position(wm.home[id], tx, ty);
}

void run_press(WorldModel &wm, int id) {
    RobotState &r = wm.home[id];
    if (!push_allowed(wm)) { hold_out_of_corner(wm, r); return; }
    BallState chased = chase_target(wm);
    if (wm.ctx.dist_opp_goal(chased.x) < 100.0 && std::fabs(chased.y - 90.0) < 45.0) {
        chased.x = wm.ctx.opp_goal_x() - wm.ctx.attack_dir() * 85.0;
        chased.y = clamp(chased.y, 72.5, 107.5);
    }
    if (wm.role[id] != ROLE_ACTIVE)   // 非主攻的路线不穿对方门区
        opp_box_detour(wm.ctx, r.x, r.y, chased.x, chased.y);
    move_avoiding(wm, r, id, chased.x, chased.y, true);
}

// 官方式分区：4 个场上球员各管一片，球进自己的片就直冲球，否则站随球平移的点

// 己方禁区纪律：分区球员一律不进小禁区；大禁区只放离球最近的一个（禁区内至多 主攻+1 人）
static void own_box_clamp(const WorldModel &wm, int id, double &tx, double &ty) {
    const TeamContext &ctx = wm.ctx;
    double fx = ctx.dist_our_goal(tx);
    int best = -1; double bd = 1e18;
    for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {
        if (wm.role[i] == ROLE_ACTIVE || wm.role[i] == ROLE_GOALIE) continue;
        double d = dist(wm.home[i].x, wm.home[i].y, wm.ball.x, wm.ball.y);
        if (d < bd) { bd = d; best = i; }
    }
    if (fx < 17.0 && ty > 62.0 && ty < 118.0) fx = 17.0;
    if (id != best) {
        const RobotState &r = wm.home[id];
        // 已在大禁区 → 先沿 x 直线退出，不横穿禁区去新站位点
        if (ctx.dist_our_goal(r.x) < 45.0 && r.y > 40.0 && r.y < 140.0) { fx = 48.0; ty = r.y; }
        else if (fx < 45.0 && ty > 40.0 && ty < 140.0) fx = 45.0;
    }
    tx = ctx.our_goal_x() + ctx.attack_dir() * fx;
}

void run_zone(WorldModel &wm, int id) {
    if (run_pass_receiver(wm, id)) return;
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    const double ad = ctx.attack_dir();
    const double bfx = ctx.dist_our_goal(wm.ball.x), by = wm.ball.y;
    auto X = [&](double fx) { return ctx.our_goal_x() + ad * fx; };   // 距己方门线 → 绝对 x

    bool chase = false;
    double sfx = 0.0, sy = 90.0;                                    // 不追时的站位点
    if (wm.role[id] == ROLE_ASSIST || wm.role[id] == ROLE_MIDFIELD) {
        const bool up = (wm.role[id] == ROLE_ASSIST);               // ASSIST 管上翼、MIDFIELD 管下翼
        const double yb = up ? by : 180.0 - by;                     // 统一成"上翼"口径
        double ty = 0.0;
        if (yb < 45.0)         { sfx = bfx - 8.0; ty = 120.0; }
        else if (yb > 135.0)   chase = true;
        else if (bfx < 25.0)   { sfx = 20.0; ty = (yb < 80.0) ? 150.0 : 120.0; }
        else if (yb > 80.0)    chase = true;
        else                   { sfx = bfx - 20.0; ty = 140.0; }
        sy = up ? ty : 180.0 - ty;
    } else {                                                        // PASSIVE = 中卫
        if (bfx > 130.0)                 { sfx = 110.0; sy = 90.0; }
        else if (by > 130.0 || by < 50.0) { sfx = std::max(bfx - 45.0, 25.0); sy = 90.0; }
        else if (bfx < 25.0)             { sfx = 25.0; sy = by; }
        else                             chase = true;
    }

    if (!chase) {
        double tx = X(clamp(sfx, 15.0, 205.0)), ty = clamp(sy, 8.0, 172.0);
        if (ctx.dist_opp_goal(tx) < 100.0 && std::fabs(ty - 90.0) < 45.0) tx = ctx.opp_goal_x() - ad * 85.0;
        own_box_clamp(wm, id, tx, ty);
        opp_box_detour(ctx, r.x, r.y, tx, ty);
        motion::position(r, tx, ty);
        return;
    }

    if (in_no_push_zone(wm.ball.x, wm.ball.y)) { hold_out_of_corner(wm, r); return; }
    BallState c = chase_target(wm);
    if (ctx.dist_opp_goal(c.x) < 100.0 && std::fabs(c.y - 90.0) < 45.0) {
        double tx = ctx.opp_goal_x() - ad * 85.0, ty = clamp(c.y, 72.5, 107.5);
        opp_box_detour(ctx, r.x, r.y, tx, ty);
        motion::position(r, tx, ty, motion::TM_PASS);
        return;
    }
    const double dx = c.x - r.x, dy = c.y - r.y, d = std::hypot(dx, dy);
    const double fwd = d > 1e-6 ? dx * ad / d : 1.0;                // 人→球方向在进攻方向上的分量
    const double need = (ctx.dist_our_goal(c.x) < kChargeOwnGuard) ? kChargeMinFwdOwn : kChargeMinFwd;
    if (fwd >= need) {
        double tx = c.x + (d > 1e-6 ? dx / d : ad) * kChargeThrough, ty = c.y + (d > 1e-6 ? dy / d : 0.0) * kChargeThrough;
        own_box_clamp(wm, id, tx, ty);
        opp_box_detour(ctx, r.x, r.y, tx, ty);
        motion::position(r, tx, ty, motion::TM_PASS);
        return;
    }
    // 站在球的进攻侧（撞过去会把球往自家门送）→ 从球侧绕到球后，绝不穿球
    const double side = (r.y >= c.y) ? 1.0 : -1.0;
    double tx = c.x - ad * 20.0, ty = c.y;
    if (dist(r.x, r.y, c.x, c.y) < 25.0 || (r.x - c.x) * ad > 0.0) { tx = c.x - ad * 5.0; ty = c.y + side * 22.0; }
    own_box_clamp(wm, id, tx, ty);
    motion::position(r, tx, ty, motion::TM_PASS);
}

}  // namespace simuro5
