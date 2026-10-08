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
// 门将 y 夹取范围（门框 y∈[70,110]），下缘留 4cm 防撞下门柱
constexpr double kGkYLo = 74.0, kGkYHi = 106.0;
constexpr double kGkBlockYLo = 78.0, kGkBlockYHi = 102.0;

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
    if (!kNoPushGuardEnabled) { (void)wm; return true; }       // 守卫关闭：不拦
    if (wm.game_state != PM_PlayOn) return false;             // 死球/摆位/重启期
    return !in_no_push_zone(wm.ball.x, wm.ball.y);            // 球未贴角
}

bool prep_point_ok(double px, double py) {
    if (!kNoPushGuardEnabled) { (void)px; (void)py; return true; }
    return !in_no_push_zone(px, py);
}

void hold_out_of_corner(WorldModel &wm, RobotState &r) {
    (void)wm;
    motion::stop(r);
}

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

// 守门员：每帧感知一次（GkView），按优先级逐条试规则表，第一条命中即出动作

TUNABLE(kGkNoPushDist, 64.8878);  // cm：球进我方门口这个距离内才管
TUNABLE(kGkSideClear, 38.0544);  // cm：侧向让开距离
TUNABLE(kGkBackOff, 9.358);  // cm：场侧回撤（目标是球后，绝不越球）
TUNABLE(kGkBehindMargin, 6.75991);  // cm：球须明显越过门将这么多才让开

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

TUNABLE(kCoverLineDanger, 0.8);  // cm/帧：球朝门速度下限
TUNABLE(kCoverLineTta, 8.3);  // 帧：到门线时间上限
TUNABLE(kCoverLineDist, 77.4913);  // cm：球离门线多近才抢
TUNABLE(kCoverLineGiveUp, 7);  // cm：门将贴球到此距离 → 让位给清球

bool gk_cover_line_point(const WorldModel &wm, int id, double &tx, double &ty) {
    const TeamContext &ctx = wm.ctx;
    const RobotState &r = wm.home[id];
    double bx = wm.ball.x;
    double y_at_goal = 0.0;
    if (!predict_y_at_x(bx, wm.ball.y, wm.ball.vx, wm.ball.vy,
                        ctx.our_goal_x(), y_at_goal))
        return false;
    if (y_at_goal < goal_y_low() || y_at_goal > goal_y_high())
        return false;                                   // 会偏出或打门柱
    if (ball_danger_speed(wm) <= kCoverLineDanger) return false;      // 太慢：站线跟球就够
    if (ctx.dist_our_goal(bx) >= kCoverLineDist) return false;
    if (std::fabs(wm.ball.vx) > 1e-9) {
        double tta = std::fabs(ctx.our_goal_x() - bx) / std::fabs(wm.ball.vx);
        if (tta > kCoverLineTta) return false;
    }
    if (dist(r.x, r.y, bx, wm.ball.y) < kCoverLineGiveUp) return false;  // 贴球了 → 清球优先
    tx = ctx.our_goal_x() + ctx.attack_dir() * 3.0;      // 贴门线 3cm
    ty = clamp(y_at_goal, kGkYLo, kGkYHi);                  // 预测落点，夹在门框内侧
    return true;
}

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
constexpr double kGkTrackYLo         = 76.0;   // 常规站位 y 跟球范围（门宽内侧留余量）
constexpr double kGkTrackYHi         = 104.0;
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
TUNABLE(kGkServeGlide, 12.0);    // cm：离后撤点还远于此值就带速滑过去，进此范围才刹车精调
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
    bool   ball_still;   // 球速 <0.2cm/帧
    bool   opp_has_ball; // 对方持球（带滞回）
    bool   heading_goal; // 球会到达己方门线
    bool   on_target;    // 且过门线时在门框内
    double y_at_goal;    // 过门线时的 y
};

TUNABLE(kGkCreepStill, 1.0);     // 0 = 回滚（只认 <0.2cm/帧）
constexpr double kGkCreepSpeed  = 0.5;    // cm/帧：慢爬上限
constexpr double kGkKickSpotDx  = 14.8;   // cm：门球点离门线
constexpr double kGkKickSpotDy  = 12.3;   // cm：门球点离中线
constexpr double kGkCreepRange  = 12.0;   // cm：离门球点多近才算

bool gk_goal_kick_creep(const GkView &v) {
    if (kGkCreepStill < 0.5) return false;
    if (std::hypot(v.vx, v.vy) >= kGkCreepSpeed) return false;
    if (v.vx * v.gside > 0.0) return false;                       // 朝门滚 → 活球
    return std::hypot(v.ball_goal - kGkKickSpotDx, std::fabs(v.by - 90.0) - kGkKickSpotDy) <
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
    return v;
}

void gk_update_opp_hold(WorldModel &wm, GkView &v) {
    if (v.opp_dmin < 15.0) {
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

bool gk_rule_ball_behind(WorldModel &wm, int id, const GkView &v) {
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    if (!(v.ball_goal < ctx.dist_our_goal(r.x) - kGkBallBehindMargin)) return false;
    double side = (r.y >= v.by) ? 1.0 : -1.0;                   // 往自己那侧绕，少掉头
    double gx = v.bx + v.gside * kGkBallGoalSide;
    double gy = v.by + side * kGkBallRetreatLat;
    double cx = gx, cy = clamp(gy, kGkYLo, kGkYHi);
    clamp_goalie_area(ctx, cx, cy);
    if (v.ball_goal < kGkNoRoom || gk_path_hits_ball(r, v, cx, cy)) {
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
    if (push_allowed(wm)) return false;
    motion::position(wm.home[id], gk_line_x(wm.ctx, kGkGuardDist),
                     clamp(v.by, kGkTrackYLo, kGkTrackYHi), motion::TM_STOP);
    return true;
}

bool gk_rule_side_step(WorldModel &wm, int id, const GkView &) {
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
        return py >= kGkYLo && py <= kGkYHi;
    };
    if (ok(dy)) return;
    if (ok(-dy)) { dy = -dy; return; }
    dy = clamp((v.by - clamp(v.by, kGkYLo + 1.0, kGkYHi - 1.0)) / kGkPushDist, -0.8, 0.8);
    dx = ctx.attack_dir() * std::sqrt(1.0 - dy * dy);
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
        if (v.ball_goal < 15.0 && (r.x - v.bx) * v.gside < 0.0) {
            gk_goto(ctx, r, v.bx - v.gside * 10.0, v.by + gk_detour_side(v.by) * 22.0,
                    motion::TM_STOP);
            return true;
        }
        // 三个动作各用一个只管一件事的原语，别混：
        //   position_aligned / arrive_facing 都想"又要走位又要转正"，两个目标互相扯，
        //   实测 position_aligned 59 帧只挪 4.5cm、arrive_facing 到位后转过头转不停。
        // 入口要求"停在后撤点附近且已对准"：后撤点是精确停靠点，因为从斜后方推球会把球拨偏。
        bool at_ready = std::hypot(r.x - px, r.y - pyc) <= kGkServeReady &&
                        (clamped || gk_aligned(r, v, dx, dy));
        if (!at_ready) {
            // 远距离接近要带速滑（别提前刹车，白磨帧数），但滑行停不准，
            // 所以进到 kGkServeGlide 之内就切回 TM_STOP 精调停靠 —— 两者按距离分工，不重叠。
            double to_prep = std::hypot(r.x - px, r.y - pyc);
            bool gliding = !contested && !clamped && kGkPrepPass > 0.5 &&
                           to_prep > kGkServeGlide &&
                           std::fabs(gk_across(r, v, dx, dy)) <= kGkAlignAcross * 2.0;
            gk_goto(ctx, r, px, pyc, gliding ? motion::TM_PASS : motion::TM_STOP);
            return true;
        }
        wm.goalie_serve_phase = 1;
    }
    // 承诺期唯一的解锁条件：真的冲到球前面去了（说明这一下没擦到球，推空了）
    // ——只有推空才会满足，正常推穿时球一直被顶在门将前面，所以不会把状态翻回去。
    if ((r.x - v.bx) * ax + (r.y - v.by) * ay > kGkServeReady) {
        wm.goalie_serve_phase = 0;
    }
    if (gk_turn_to(r, ax, ay)) return true;    // 只管原地转正
    gk_goto(ctx, r, v.bx + ax * kGkKickThrough, v.by + ay * kGkKickThrough, motion::TM_PASS);
    return true;
}

bool gk_rule_cover_line(WorldModel &wm, int id, const GkView &) {
    double tx = 0.0, ty = 0.0;
    if (!gk_cover_line_point(wm, id, tx, ty)) return false;
    motion::position(wm.home[id], tx, ty, motion::TM_PASS);
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
TUNABLE(kGkHoldPassGap, 9.0);    // cm：门将要从球的场侧绕到球前时，横向必须让开球这么多。
                                 // 车身半径4 + 球半径2 = 6cm 就接触，9cm 留 3cm 富余保证蹭不到球。
TUNABLE(kGkOwnGoalPad, 6.0);     // cm：防乌龙余量。球比门将更靠自家门（含此余量）时，门将这一帧
                                 // 的动作整条换成安全动作，绝不允许朝自家门推进。
TUNABLE(kGkFaceTol, 12.0);       // 度：守门时允许的机头偏差。超过就拧机头，把门将拧到直着对准球。
TUNABLE(kGkSpinGain, 0.8);       // 轮速差 / 度：盯球拧机头的比例增益
TUNABLE(kGkSpinMax, 60.0);       // 轮速差上限：盯球拧机头最多叠这么多（不动位移）
TUNABLE(kGkHoldCover, 2.0);      // cm：门将中心离进门点小于此才算"已挡住"。
                                 // 原 5.0 太松：实测丢球时门将横向只偏 4.6cm 就自认挡住了、原地不动，
                                 // 球从它旁边滚进网。收紧后偏 2~5cm 会走下面的滑移分支贴到球的进门点。
bool gk_rule_goal_line_hold(WorldModel &wm, int id, const GkView &v) {
    if (!(v.ball_goal < 15.0 && v.db < 14.0)) return false;
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
        double side = gk_detour_side(v.by);               // 只看球的位置选边，避免左右翻
        double lat  = v.ball_still ? kGkLateral : 12.0;   // 球在动：小侧移贴近截下
        tx = v.bx + v.gside * kGkPushDist;
        ty = clamp(v.by + side * lat, kGkYLo, kGkYHi);
        mode = motion::TM_STOP;
    } else {
        tx = v.bx + dx * kGkKickThrough;                  // 这里 y 只夹罚球区，不夹门框
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
    motion::position(wm.home[id], gk_line_x(wm.ctx, v.ball_goal < 15.0 ? 3.0 : kGkGuardDist),
                     clamp(v.by, kGkTrackYLo, kGkTrackYHi));
    return true;
}

using GkRule = bool (*)(WorldModel &, int, const GkView &);

const GkRule kGoalieRules[] = {
    gk_rule_no_push,        // 死球期/球在角区：不触球
    gk_rule_side_step,      // 门口球已越过门将：先侧向让开
    gk_rule_opp_ball_lock,  // 对方持球：锁门前浅位（门口贴球时上前封角）
    gk_rule_restart_kick,   // 门前静止球：穿球推出
    gk_rule_cover_line,     // 球在门框内轨迹上：抢门线落点
    gk_rule_opp_kick_line,  // 对手准备出脚：按机头方向提前封线
    gk_rule_press_door,     // 门前对手贴球：封球-门连线
    gk_rule_goal_line_hold, // 贴门线+贴球：站定防乌龙
    gk_rule_clear,          // 球在脚下：解围
    gk_rule_wall_ball,      // 贴墙朝门滚：提前到门线落点
    gk_rule_shot_block,     // 会进门的球：动态前压封角
    gk_rule_default,        // 无威胁：门前跟球 y
};

// 诊断：记录门将每帧命中规则到 C:\Strategy\goalie_trace.csv（CMake 选项 HNNU_TRACE，默认 OFF）
#if defined(SIMURO5_HNNU_TRACE) || defined(SIMURO5_GK_PROBE)
const char *const kGoalieRuleNames[] = {
    "no_push", "side_step", "opp_ball_lock", "restart_kick", "cover_line",
    "opp_kick_line", "press_door", "goal_line_hold", "clear", "wall_ball",
    "shot_block", "default",
};
#endif

#ifdef SIMURO5_HNNU_TRACE
static void gk_trace(const char *rule, const WorldModel &wm, int id, const GkView &v) {
    static FILE *fp = nullptr;
    static long frame = 0;
    if (!fp) fp = std::fopen("C:\\Strategy\\goalie_trace.csv", "a");
    if (!fp) return;
    ++frame;
    std::fprintf(fp, "%ld,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%.2f,%.2f\n",
                 frame, rule, wm.ball.x, wm.ball.y, wm.home[id].x, wm.home[id].y,
                 v.ball_goal, v.db, v.opp_dmin, (int)v.opp_has_ball, wm.ball.vx, wm.ball.vy);
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
    motion::LegacyDriveScope legacy_drive;   // 门将保持旧驱动口径
    GkView v = gk_view(wm, id);
#ifdef SIMURO5_GK_PROBE
    g_gk_rule_probe = "none";
#endif
#ifdef SIMURO5_HNNU_TRACE
    const char *hit = "none";
#endif
    // 出球模式 = 门球推穿(restart_kick) / 点球守(penalty) / 死球(no_push)。
    // 这三种动作本身就要求"机头对准出球方向 + 以速度推穿球"，下面的守门两闸会把它们打坏
    // （离线测试已证：门球 200 帧碰不到球、横向 13cm 时高速直冲、点球守会离位），所以整段跳过。
    bool gk_set_play = false;
    if (gk_rule_penalty(wm, id, v)) { GK_HIT("penalty"); gk_set_play = true; }
    else if (gk_rule_line_block(wm, id, v)) { GK_HIT("line_block"); }
    else if (gk_rule_ball_behind(wm, id, v)) { GK_HIT("ball_behind"); }
    else {
        gk_update_opp_hold(wm, v);
        for (int i = 0; i < (int)(sizeof(kGoalieRules) / sizeof(kGoalieRules[0])); ++i) {
            if (kGoalieRules[i](wm, id, v)) {
                GK_HIT(kGoalieRuleNames[i]);
                gk_set_play = (i == 0) || (i == 3);   // 0=no_push(死球)  3=restart_kick(门球)
                break;
            }
        }
    }
#ifdef SIMURO5_HNNU_TRACE
    gk_trace(hit, wm, id, v);
#endif

    // ═══════ 守门两闸：只对守门模式生效，出球模式整段跳过 ═══════
    RobotState &gr = wm.home[id];
    const TeamContext &gctx = wm.ctx;
    // 规则主动"站定不动"（vl=vr=0）时不加闸：那是规则按门前几何做的防乌龙决定
    // （球就贴在自己脚下时，连原地转身都可能把它蹭进门），闸门不许把它解冻。
    bool gk_frozen = (gr.vl == 0.0 && gr.vr == 0.0);
    if (!gk_set_play && !gk_frozen) {

        // 闸一（防乌龙）：球比门将更靠自家门（含余量）时，门将已经站在球的"场侧"了 ——
        //   此时它只要朝自家门方向走一步，就是拿身体把球顶进自家门。所以从轮速里把
        //   "朝自家门方向的平动分量"整条扣掉：转动分量保留（它照样能转身），横向平动也保留
        //   （它照样能横移挡到球前），但绝不可能朝自家门推进一步。
        //   横向已经让开球（> kGkHoldPassGap）时不干预：那条通道是安全的（车身4+球2<9），
        //   门将本来就该从那儿全速插到球与自家门之间。
        if (v.ball_goal < gctx.dist_our_goal(gr.x) + kGkOwnGoalPad &&
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
}
#undef GK_HIT

// 对方门区停留红线：门区除门将外纯停留 >8 帧即计（平台 20 周期判点球，须留撤出时间）
TUNABLE(kReboundRushSpeed, 7.34364);  // cm/帧：反弹球可抢速度阈值
TUNABLE(kReboundRushDist, 51.3294);  // cm：距球超过此值不冲（就近补）
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

bool pass_context_safe(const WorldModel &wm) {
    if (!wm.ball.valid || wm.game_state != PM_PlayOn || wm.in_penalty_exec ||
        wm.threat_level >= 0.6 || in_no_push_zone(wm.ball.x, wm.ball.y)) return false;
    double ours = 1e9, theirs = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        ours = std::min(ours, dist(wm.ball.x, wm.ball.y, wm.home[i].x, wm.home[i].y));
        theirs = std::min(theirs, dist(wm.ball.x, wm.ball.y, wm.opp[i].x, wm.opp[i].y));
    }
    if ((theirs < 12.0 && theirs + 5.0 < ours) ||
        (wm.whos_ball == 2 && !(ours < 12.0 && ours + 5.0 < theirs))) return false;
    if (wm.ctx.dist_our_goal(wm.ball.x) < 80.0 &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 3.0 && theirs < 100.0) return false;
    if (shot_on_target(wm) && ball_danger_speed(wm) > rebound_min_danger()) return false;
    return true;
}

bool pass_target_safe(const WorldModel &wm, int passer, int receiver,
                      double tx, double ty, bool check_lane) {
    if (!pass_context_safe(wm) || passer != 1 || receiver < 2 || receiver > 4 ||
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
    return pass_context_safe(wm) && id >= 1 && id <= 4 && wm.role[id] != ROLE_ACTIVE &&
        wm.coop_ball_control.game_state == wm.game_state && wm.ga_cooldown[id] <= 0 &&
        wm.ga_overstay[id] < 15 && !in_goal_area(wm.ctx, wm.home[id].x, wm.home[id].y) &&
        !in_opp_goal_area(wm.ctx, wm.home[id].x, wm.home[id].y);
}

void observe_pass_lifecycle(WorldModel &wm) {
    auto &task = wm.coop_pass_task;
    if (task.active && task.passer_id == wm.active_id && task.receiver_id >= 1 && task.receiver_id <= 4 &&
        task.receiver_id != task.passer_id &&
        task.frames_left > 0 && task.game_state == wm.game_state && pass_context_safe(wm)) {
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
            bool evidence = wm.we_have_ball || wm.whos_ball == 1 || db + 5.0 < opp_min;
            bool received = db < kPassReceiveDistance && db < teammate_min && db < opp_min && evidence &&
                            std::hypot(wm.ball.vx, wm.ball.vy) <= kPassReceiveSpeed;
            task.receive_frames = received ? task.receive_frames + 1 : 0;
            if (task.receive_frames >= kPassReceiveFrames &&
                pass_target_safe(wm, task.passer_id, task.receiver_id, task.rx, task.ry, false)) {
                task.phase = CoopPassPhase::Received;
                wm.coop_ball_control = {true, task.receiver_id, wm.game_state, 0};
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
    if (task.active && (task.frames_left <= 0 || task.game_state != wm.game_state ||
        !pass_target_safe(wm, task.passer_id, task.receiver_id, task.rx, task.ry,
                          task.phase == CoopPassPhase::Preparing))) {
        CoopOutcome reason = CoopOutcome::InvalidTarget;
        if (task.game_state != wm.game_state) reason = CoopOutcome::GameState;
        else if (task.frames_left <= 0)
            reason = task.phase == CoopPassPhase::Preparing ? CoopOutcome::PrepareTimeout : CoopOutcome::ReceiveTimeout;
        else if (wm.in_penalty_exec) reason = CoopOutcome::Penalty;
        else if (in_no_push_zone(wm.ball.x, wm.ball.y)) reason = CoopOutcome::Corner;
        else if (wm.threat_level >= 0.6) reason = CoopOutcome::HighThreat;
        else if (wm.whos_ball == 2) reason = CoopOutcome::Intercepted;
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
    if (wm.coop_ball_control.active && !pass_control_safe(wm)) wm.coop_control_end(CoopOutcome::InvalidTarget);
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

void run_active(WorldModel &wm, int id) {
    RobotState &r = wm.home[id];
    const TeamContext &ctx = wm.ctx;
    const bool had_pass_task = wm.coop_pass_task.active || wm.coop_ball_control.active;
    if (wm.coop_pass_task.active) --wm.coop_pass_task.frames_left;
    observe_pass_lifecycle(wm);
    cancel_unsafe_pass_task(wm);

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

    if (!push_allowed(wm)) { if (wm.coop_pass_task.active) wm.coop_finish(CoopOutcome::PushForbidden); hold_out_of_corner(wm, r); return; }

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
        PassPlan pp_ga = plan_pass(wm, id);
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
        if (!pass_receiver_ready(wm)) { motion::stop(r); return; }
        double length = dist(wm.ball.x, wm.ball.y, task.rx, task.ry);
        if (length > 1e-6) {
            task.push_dir_x = (task.rx - wm.ball.x) / length;
            task.push_dir_y = (task.ry - wm.ball.y) / length;
            if (!task.observing_push && dist(r.x, r.y, wm.ball.x, wm.ball.y) < kPassReceiveDistance) {
                task.observing_push = true; task.push_ball_x = wm.ball.x; task.push_ball_y = wm.ball.y;
            }
            motion::position(r, wm.ball.x + task.push_dir_x * 20.0,
                             wm.ball.y + task.push_dir_y * 20.0, motion::TM_PASS);
        }
        return;
    }

    // 射门：贴身直线推穿——目标 = 球前 20cm，推球方向 = 瞄准线（不被接近轨迹带偏）
    ShootPlan sp = plan_shoot(wm, id);
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
    } else if (!had_pass_task && pass_context_safe(wm)) {
        cp = plan_coop_pass(wm, id);
        cp.viable = cp.viable && cp.score > sp.quality + 0.15;
    }
    const bool existing_coop_task = wm.coop_pass_task.active && wm.coop_pass_task.kind == PassTaskKind::Coop;
    const bool coop_preferred = existing_coop_task ||
        (cp.viable && !wm.in_penalty_exec && cp.score > sp.quality + 0.15);
    const bool coop_pass = coop_preferred &&
        (wm.coop_pass_task.active || pass_target_safe(wm, id, cp.receiver_id, cp.rx, cp.ry, true));
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
            wm.coop_pass_task.frames_left = kPassTaskFrames; wm.coop_pass_task.game_state = wm.game_state;
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
    PassPlan pp = plan_pass(wm, id);
    if (pp.viable && !coop_preferred) {
        if (!wm.coop_pass_task.active && !had_pass_task && pass_target_safe(wm, id, pp.receiver_id, pp.target_x, pp.target_y, true)) {
            wm.coop_pass_task = {};
            wm.coop_pass_task.active = true; wm.coop_pass_task.passer_id = id;
            wm.coop_pass_task.receiver_id = pp.receiver_id; wm.coop_pass_task.rx = pp.target_x; wm.coop_pass_task.ry = pp.target_y;
            wm.coop_pass_task.frames_left = kPassTaskFrames; wm.coop_pass_task.game_state = wm.game_state;
            wm.coop_pass_task.kind = PassTaskKind::Ordinary; wm.coop_created();
        }
        if (wm.coop_pass_task.active && wm.coop_pass_task.kind == PassTaskKind::Ordinary) {
            auto &task = wm.coop_pass_task;
            if (!pass_receiver_ready(wm)) {
                if (!contested || kContestNoWait < 0.5) { motion::stop(r); return; }
                wm.coop_finish(CoopOutcome::OpponentFirst);   // 争抢态：不停车等人
            }
            double length = dist(wm.ball.x, wm.ball.y, task.rx, task.ry);
            if (length > 1e-6) {
                task.push_dir_x = (task.rx - wm.ball.x) / length; task.push_dir_y = (task.ry - wm.ball.y) / length;
                if (!task.observing_push && dist(r.x, r.y, wm.ball.x, wm.ball.y) < kPassReceiveDistance) {
                    task.observing_push = true; task.push_ball_x = wm.ball.x; task.push_ball_y = wm.ball.y;
                }
            }
            motion::position(r, task.rx, task.ry);
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
            PassPlan pp2 = plan_pass(wm, id);
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
            if (wm.ctx.dist_our_goal(wm.ball.x) < 25.0 &&
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
    if (!wm.live_play || wm.in_penalty_exec) return false;   // 认活球（平台 gameState 不回 PlayOn）
    if (wm.coop_pass_task.active &&
        (id == wm.coop_pass_task.passer_id || id == wm.coop_pass_task.receiver_id)) return false;
    if (wm.coop_ball_control.active && id == wm.coop_ball_control.receiver_id) return false;
    if (id == wm.sweeper_id || !push_allowed(wm)) return false;
    const double bx = wm.ball.x, by = wm.ball.y, ad = ctx.attack_dir();
    if (ctx.dist_our_goal(bx) < kSwarmOwnGuard) return false;

    if (near_opp_box(ctx, bx, by)) {
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

    if (!holds_ball(wm, id, ux, uy)) {
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            if (i == id || wm.role[i] == ROLE_GOALIE) continue;
            if (!holds_ball(wm, i, ux, uy)) continue;
            swarm_move(wm, id, bx - ux * kEscortBack + nx * side * kEscortSide,
                       by - uy * kEscortBack + ny * side * kEscortSide);  TRACE_MARK(wm.home[id]);
            return true;
        }
    }

    if (behind > 0.0 && std::fabs(side_off) < kHerdLatTol) {
        swarm_move(wm, id, bx + ux * kHerdThrough, by + uy * kHerdThrough);  TRACE_MARK(wm.home[id]);
    } else if (behind > -3.0) {
        double back = kHerdBack + 0.5 * std::fabs(side_off);
        double px = bx - ux * back, py = by - uy * back;
        if (!prep_point_ok(px, py)) return false;
        swarm_move(wm, id, px, py);  TRACE_MARK(wm.home[id]);
    } else {
        // 人在球前面：先横绕到球侧，绝不直线穿过球
        swarm_move(wm, id, bx - ux * 4.0 + nx * side * kHerdSide,
                   by - uy * 4.0 + ny * side * kHerdSide);  TRACE_MARK(wm.home[id]);
    }
    return true;
}
}  // anonymous namespace

void run_assist(WorldModel &wm, int id) {
    if (run_pass_receiver(wm, id)) return;
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
