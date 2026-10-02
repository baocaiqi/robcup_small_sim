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
#include "simuro5/branch_trace.hpp"   // 须在所有 include 之后（诊断构建才生效）

namespace simuro5 {

namespace {
// 门将 y 夹取范围（门框 y∈[70,110] 内侧留 4cm）：门将动作/解围/封线/出击共用上下限。
//   封角度更窄（封"球-门"连线要贴线、留 12cm 才堵得住角度），故另有 kGkBlockYLo/Hi。
constexpr double kGkYLo = 74.0, kGkYHi = 106.0;
constexpr double kGkBlockYLo = 78.0, kGkBlockYHi = 102.0;

// ============================================================
// docs/15 P0-4：避障移动 helper（追球/移动避障路径层接线，纯路径无射门依赖）
// ============================================================
// 对方机器人避障半径（本体 6 + 净空 4）。
//   ⚠️ 与 plan_route 的 margin（默认 0.5）是绑定关系：路径允许侵入本膨胀圈最多 margin，
//   即实际净空 ∈ [3.5, 4.0]cm。要调小本值必须同时调小 margin，否则会真撞（见 route.hpp 契约）。
TUNABLE(kRouteInflate, 10.0);
// 第 90 轮（用户："近球减速、停稳、对准角度等待……直接精简，像官方一样"）：
//   官方 demo 近球不减速（sigmoid 速度律到接触前都 ≈70）、不原地转正、不等对准，目标就是球。
//   =1：主攻追球去掉 10cm 线性减速；射门/带球只要在球后就直接穿球推，不原地转正、不超时死等；
//   不在球后则 TM_PASS 赶去球后点（不做到点对准）。罚点球执行、防过冲反推、门区纪律照旧。
//   =0 回滚到第 89 轮行为。
TUNABLE(kNoAlignWait, 1.0);

// 避障移动：从 (r.x,r.y) 向 (tx,ty)，对方 5 机器人作圆盘障碍。
//   直线通 → 直线（最短即最优）；直线被挡 → 可见图+Dijkstra 绕行；
//   目标被障碍吞 / 无通路 → 回退直线（plan_route found=false）。
//   decel=true 时（追球场景）接近目标 10cm 内线性减速，防冲过头把球铲偏。
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
        wm.route_wp_next[id] = 0;      // 每帧重置：路径变了不沿用旧段（follow_route 自动跳段）
        motion::follow_route(r, rt, wm.route_wp_next[id], motion::TM_PASS);   // 追球=经过型
    } else {
        motion::position(r, tx, ty, motion::TM_PASS);
    }
    if (decel && kNoAlignWait < 0.5) {
        double dg = dist(r.x, r.y, tx, ty);
        if (dg < 10.0) { r.vl *= dg / 10.0; r.vr *= dg / 10.0; }   // 同 chase_ball 防冲
    }
}

// ============================================================
// 无球接应拉开（队员C）：站位点附近有敌方机器人时，沿 Y 轴横向躲开。
//   · 只微调 Y，保留 A 输出的原始 X（全局跑位点由 situation.cpp 决定，这里只做局部微调）
//   · 判距用 dx*dx+dy*dy，风格对齐 pass.cpp 的 count_near_opponent
//   · 返回微调后的 y；威胁半径内无敌人则原样返回 by
// ============================================================
// ============================================================
// docs/06 第 49 轮：推球合法性守卫（治真机 "No pushing" 犯规）
//   平台规则：① 死球/重启（摆位）期间推球 = 犯规；② 角落黄区内推球 = 犯规（每 4 次 +1 球）。
//   真机日志：10:56 场我方被判 4 次 `FreeBall RightBot/RightTop … blue team Violated
//   No pushing` → 4 次 = 白送 1 球；09:44 场（改前）0 次。
//   本平台"我们所有让球动起来的动作都是推球"（没有踢球动作），所以守必须是**动作入口级**的：
//   球在角区/平台在死球期时，一律不碰球（`hold_out_of_corner` = 停住），让平台按规则处理。
// ============================================================
bool push_allowed(const WorldModel &wm) {
    if (!kNoPushGuardEnabled) { (void)wm; return true; }       // 守卫关闭：不拦（见 roles.hpp）
    if (wm.game_state != PM_PlayOn) return false;             // 死球/摆位/重启期
    return !in_no_push_zone(wm.ball.x, wm.ball.y);            // 球未贴角
}

// "球后准备点"是否合法（docs/06 第 49 轮）：准备点也不许落在角区——
//   否则机器人驱车过去时会穿过球、把球往角心顶 → "No pushing" 犯规。
bool prep_point_ok(double px, double py) {
    if (!kNoPushGuardEnabled) { (void)px; (void)py; return true; }
    return !in_no_push_zone(px, py);
}

// 不许推球时的动作（docs/06 第 49 轮）：**只停不动**。
//   理由：本平台没有踢球动作，任何"朝球的移动"都是推球；而规则只罚"推球"，
//   站在角区里不动并不犯规。所以最安全且最易验证的动作就是停住——
//   既不碰球、也不会像"绕到场心侧"那样让路径穿过球（那等于继续推）。
//   球若卡在角区，交给平台按规则判 FreeBall（代价 < 每 4 次犯规白送 1 球）。
void hold_out_of_corner(WorldModel &wm, RobotState &r) {
    (void)wm;
    motion::stop(r);
}

// 追球目标校验（docs/13 防守修复 F2）：平台预测(ball_pred)在进球/FreeBall/定位球
//   重置期会冻结在错误位置（真实 9/2 丢球 2：ACTIVE 追幻影 40 帧脱位、
//   4v5 被 demo 运动战破门）——预测点离实际球 >80cm 即视为不可信，改用实际球位。
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
        if (d2 < best_d2) {      // 半径内最近的敌人
            best_d2 = d2;
            nearest_dy = dy;
            found = true;
        }
    }
    if (!found) return by;
    // 越近偏移越大（d=0→max_offset，d=radius→0），朝敌人反方向横向躲
    double frac = 1.0 - std::sqrt(best_d2) / threat_radius;
    double sign = (nearest_dy >= 0.0) ? -1.0 : 1.0;   // 敌人在上 → 往下躲
    return by + sign * max_offset * clamp(frac, 0.0, 1.0);
}

}  // anonymous namespace

// ============================================================
// 守门员
// ------------------------------------------------------------
// run_goalie = 每帧感知一次（GkView）+ 按优先级逐条试规则表，第一条命中的规则出动作。
// 每条规则的来由（真机复盘场次、调参轮次）见 docs/06「第 80 轮：门将规则表」，
// 代码里只写"为什么"。新增门将行为 = 在 kGoalieRules 里插一行，并说清排在谁前面、为什么。
// ============================================================

// 球外侧禁推（第 58、75 轮）：球已到门口、且球比门将更靠己门（门将在场侧）时，
//   门将朝球走会把球顶进自家门。先横移到球侧方 kGkSideClear（保持在球后 kGkBackOff，
//   绝不越过球）；让开后本规则不再触发，门将才绕到门侧去推或走门线封堵。
TUNABLE(kGkNoPushDist, 64.8878);  // 球进我方门口这个距离内才管（cm）
TUNABLE(kGkSideClear, 38.0544);  // 侧向让开距离（cm）
TUNABLE(kGkBackOff, 9.358);  // 场侧回撤（cm）：目标是球后，绝不越球
TUNABLE(kGkBehindMargin, 6.75991);  // 球必须已"明显越过门将"这么多才让开（cm）；齐平时照常清球

bool gk_side_step_point(const WorldModel &wm, int id, double &tx, double &ty) {
    const TeamContext &ctx = wm.ctx;
    const RobotState &r = wm.home[id];
    double bx = wm.ball.x, by = wm.ball.y;
    if (ctx.dist_our_goal(bx) >= kGkNoPushDist) return false;   // 球还远：按常规防
    double gside = ball_goal_side(ctx, bx);        // 球门在球的哪一侧
    // —— 第 75 轮（2026-09-23 真机复盘）：球贴门线时门槛降到 0 ——
    //   真机 f726 铁证（14:40 场丢球1，黑匣子逐帧）：球离门线 1.01cm、门将在场侧 5.9cm，
    //   球只比门将靠门 4.14cm，而 kGkBehindMargin=6.76cm ⇒ 4.14 ≤ 6.76 成立
    //   ⇒ 本条护栏判定"球还没明显越过门将、齐平"而**放弃让开**，落到下面"照常清球"分支，
    //   门将遂朝球推进：球速 +0.33 → +1.33 cm/帧（翻 4 倍）滚进自家门。
    //   同一局另外 2 球同型（门将 8.0/8.6cm、场侧、球速 +0.57→+1.69、+0.33→+1.03）。
    //   改法：球离门线 <12cm 时门槛取 0 —— 只要球比门将更靠己门（场侧）就**必须让开**，
    //   绝不"照常清球"；门将已在球门侧（含齐平）时门槛仍为负值域，合法清球动作不受影响。
    double behind_margin = (ctx.dist_our_goal(bx) < 12.0) ? 0.0 : kGkBehindMargin;
    if ((bx - r.x) * gside <= behind_margin) return false;      // 没明显越过（齐平/门侧）→ 照常清球
    if (std::fabs(r.y - by) >= kGkSideClear) return false;      // 已让开：允许绕到球的门侧
    double side = (r.y >= by) ? 1.0 : -1.0;
    tx = bx - gside * kGkBackOff;                               // 球后 10cm（场侧）
    ty = clamp(by + side * kGkSideClear, kGkYLo, kGkYHi);
    clamp_goalie_area(ctx, tx, ty);
    return true;
}

// 门线封堵（第 55 轮）：球已在门框内的轨迹上且快到门线时，只有门线上的预测落点拦得到
//   （追球侧方点、退到门前 40cm 都来不及）。门将已贴球则让位给清球。
TUNABLE(kCoverLineDanger, 0.8);  // cm/帧：球朝门速度下限（40cm/s）
TUNABLE(kCoverLineTta, 8.3);  // 帧：到门线时间上限（0.55s）
TUNABLE(kCoverLineDist, 77.4913);  // cm：球离门线多近才抢
TUNABLE(kCoverLineGiveUp, 7);  // cm：门将已贴球到此距离 → 让位给清球

bool gk_cover_line_point(const WorldModel &wm, int id, double &tx, double &ty) {
    const TeamContext &ctx = wm.ctx;
    const RobotState &r = wm.home[id];
    double bx = wm.ball.x;
    double y_at_goal = 0.0;
    if (!predict_y_at_x(bx, wm.ball.y, wm.ball.vx, wm.ball.vy,
                        ctx.our_goal_x(), y_at_goal))
        return false;                                   // 球不到门线 / 只有 y 向运动
    if (y_at_goal < goal_y_low() || y_at_goal > goal_y_high())
        return false;                                   // 会偏出或打门柱，不用抢
    if (ball_danger_speed(wm) <= kCoverLineDanger) return false;      // 太慢：站线跟球就够
    if (ctx.dist_our_goal(bx) >= kCoverLineDist) return false;        // 还远：按常规防
    if (std::fabs(wm.ball.vx) > 1e-9) {
        double tta = std::fabs(ctx.our_goal_x() - bx) / std::fabs(wm.ball.vx);
        if (tta > kCoverLineTta) return false;                        // 还没到该抢的时候
    }
    if (dist(r.x, r.y, bx, wm.ball.y) < kCoverLineGiveUp) return false;  // 贴球了 → 清球优先
    tx = ctx.our_goal_x() + ctx.attack_dir() * 3.0;      // 贴门线 3cm（不给角度）
    ty = clamp(y_at_goal, kGkYLo, kGkYHi);                  // 预测落点，夹在门框内侧
    return true;
}

// 对准球的直线提前堵（第 87 轮，用户真机反馈「慢球进门，门将还往旁边挪」）：
//   09-28 真机逐帧：① 中路慢球（0.47cm/帧、落点 86.8）门将本在线上，脚下清球分支让它
//   转身冲向球后 30cm，差速车画大弧跑到 y=61，球慢慢滚进空门；② 4 个球贴门线
//   （x≈217.5）滚向门口，门将停在 x≈207~210 的场侧，离滚动路径 7~10cm，球从身边滚进。
//   改法：A 慢速朝门球 → 站在"球→进门点"直线上、门前 kGkLbDepth（不冲、不绕）；
//         B 贴门线滚向门口 → 站到滚动路径 x 上、球前方 kGkLbAhead（门框内侧），迎球挡死。
TUNABLE(kGkLineBlock, 1.0);      // 总开关（0 = 回滚）
TUNABLE(kGkLbMinSpeed, 0.2);     // cm/帧：朝门速度下限（更慢交给门前静止球推出）
TUNABLE(kGkLbMaxSpeed, 2.5);     // cm/帧：A 只管慢球，快球仍由门线封堵/前压封角处理
TUNABLE(kGkLbRange, 100.0);      // cm：A 球离门线多近才对线
TUNABLE(kGkLbDepth, 10.0);       // cm：A 站位离门线（=常规站位深度）
TUNABLE(kGkLbFreeDist, 15.0);    // cm：A 最近对手离球要大于此（对方带球交给持球硬锁）
TUNABLE(kGkLbLineBand, 12.0);    // cm：B 球离门线多近算"贴门线滚"（第99轮 12→20 引入下路失球，已回滚）
TUNABLE(kGkLbRollSpeed, 0.3);    // cm/帧：B 沿门线速度下限
TUNABLE(kGkLbAhead, 12.0);       // cm：B 站在球前方多远
TUNABLE(kGkLbBand, 30.0);        // cm：B 门口带半宽（|by-90|<此值即带内；第98轮 20→30 罩住门口弹跳区）

bool gk_line_block_point(const WorldModel &wm, int id, double &tx, double &ty) {
    if (kGkLineBlock < 0.5) return false;
    const TeamContext &ctx = wm.ctx;
    const RobotState &r = wm.home[id];
    const double bx = wm.ball.x, by = wm.ball.y, vx = wm.ball.vx, vy = wm.ball.vy;
    const double ball_goal = ctx.dist_our_goal(bx);
    const double danger = ball_danger_speed(wm);
    double y_at_goal = 0.0;
    // A：慢球朝门、进门点在门框内、球没越过门将、对方没贴球
    if (danger > kGkLbMinSpeed && danger <= kGkLbMaxSpeed && ball_goal < kGkLbRange &&
        ball_goal >= ctx.dist_our_goal(r.x) - 2.0 && opp_clear_dist(wm) > kGkLbFreeDist &&
        predict_y_at_x(bx, by, vx, vy, ctx.our_goal_x(), y_at_goal) &&
        y_at_goal >= goal_y_low() - 2.0 && y_at_goal <= goal_y_high() + 2.0) {
        // 从 kGkLbDepth 往门线收，取第一个直线 y 仍在门将活动范围内的深度（斜线球贴门站）
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
    // B：球贴门线滚向门口（或已在门口前）。
    //   第 99 轮改（用户指令）：旧"门口带"硬编码 |by-90|<20，球滚到门口带外侧
    //   2~3cm（y≈110~113）弹跳时 vy 上下抖动，方向判断 (by-90)*vy<0 一抖就失配，
    //   门将"断线"掉回常规站位，球从门将和门线之间的缝漏进（第 2 场整场唯一丢球
    //   实测：球贴门线 x≈218 从上路 y=125 滚到 y=100 进门，门将停在 x≈211 没迎上去）。
    //   改法：门口带 |by-90|<kGkLbBand 从 20 放宽到 30，把 y≈110~125 的弹跳区罩进来，
    //   球在带内弹跳时无论 vy 方向都稳定触发、不掉线；带外仍靠方向判断区分
    //   "滚向门口"与"滚离门口"（后者不追，别被带走的球牵出门口）。
    //   同轮踩坑：门槛 kGkLbLineBand 也顺手 12→20，结果让 B 分支提前抢了 shot_block
    //   （下路球离门线 12~20cm 时门将就"贴门线滚球"处理、y clamp 到 74 下不去，漏成
    //   下路进球，6 球全打门柱内侧）→ 门槛已回滚 12，本注释只保留门口带放宽。
    if (ball_goal < kGkLbLineBand && std::fabs(vy) > kGkLbRollSpeed &&
        ((by - 90.0) * vy < 0.0 || std::fabs(by - 90.0) < kGkLbBand)) {
        // 门将中心与球同一 x（略深 1cm，站在球的门侧，不触发"球已越过门将"让开）
        tx = ctx.our_goal_x() + ctx.attack_dir() * clamp(ball_goal - 1.0, 3.0, kGkLbLineBand);
        ty = clamp(by + (vy > 0.0 ? 1.0 : -1.0) * kGkLbAhead, kGkYLo, kGkYHi);
        clamp_goalie_area(ctx, tx, ty);
        return true;
    }
    return false;
}

// gk_clear_direction 出球方向打分的 6 个旋钮（原函数内局部 const，提为 TUNABLE 便于调参）：
TUNABLE(kClearAngleStep, 10.0);    // 扫候选角步长(度)
TUNABLE(kClearSectorSigma, 30.0);  // 队友/对手方向的高斯衰减宽度(度)
TUNABLE(kTeamWeight, 1.0);         // 每个队友方向加分
TUNABLE(kOppWeight, 1.5);          // 每个对手方向扣分
TUNABLE(kClearEdgeBonus, 2.0);     // 越靠侧面越加分
TUNABLE(kClearMinSide, 30.0);      // 最少偏离正前方角度：绝不许正面直线开球

// 出球方向角度打分（docs/24 第二步）：门将解围/推球时不正面直线踢，扫候选角往
//   「队友密度高、对手密度低」的空当清。基准轴 = 背离己方球门（蓝=180°、黄=0°），
//   扫 ±75°，方向内队友越多/对手越少分越高，越靠侧面越加分。返回单位方向 (dirx,diry)。
//   门球重启推球(branch 8) 与 脚下清球(clear branch) 共用——此前只改了后者，前者仍
//   直线推出门区（真机复盘「门将从门口开球没改善」的根因）。
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

constexpr double kGkGuardDist        = 10.0;   // 常规站位：门线前 cm
constexpr double kGkTrackYLo         = 76.0;   // 常规站位 y 跟球范围（门宽内侧留余量）
constexpr double kGkTrackYHi         = 104.0;
constexpr double kGkMinSpeed         = 5.0;    // 朝门球速（cm/帧）低于此值不前压
constexpr double kGkFastShotSpeed    = 12.0;   // 朝门球速达到此值 → 前压到罚球区前缘
constexpr double kGkOppPullback      = 20.0;   // 罚球区内每个对手让前压深度回缩（cm），防埋伏回敲
constexpr double kGkOppFrontPad      = 8.0;    // 前压深度上限：对方最前插球员身后余量（cm）
constexpr double kGkClearDist        = 20.0;   // 球进此距离 → 脚下清球
constexpr double kGkClearAlignTol    = 20.0;   // 穿球前允许的机头偏差（度）
constexpr double kGkPushDist         = 8.0;    // 推球准备点：球后 cm
constexpr double kGkLateral          = 30.0;   // 静止球在门将身后时绕球侧移（cm）
constexpr double kGkKickThrough      = 30.0;   // 穿球目标：球前 cm（越远出球越快）
constexpr double kGkBallBehindMargin = 6.0;    // 球比门将靠门超过此值 → 强制回门
constexpr double kGkBallGoalSide     = 15.0;   // 强制回门：回到球的门侧 cm
constexpr double kGkBallRetreatLat   = 30.0;   // 强制回门：绕球侧移 cm
constexpr double kGkNoRoom           = 9.0;    // 强制回门：球离门线小于此值 → 门侧站不下车身，不绕
constexpr double kGkPathClear        = 10.0;   // 强制回门：路线离球的最小净空（半对角 5.3 + 球半径 2.1 + 余量）
constexpr int    kGkBehindHorizon    = 10;     // 强制回门：滚动球外推帧数
constexpr double kGkBehindBackOff    = 10.0;   // 强制回门走不了时：停在球的场侧 cm
constexpr int    kGkOppHoldFrames    = 3;     // 对方持球滞回帧数（防视觉闪断一帧就冲出去）

// 门将每帧的感知量：所有规则读同一份，规则之间不再各算各的。
struct GkView {
    double bx, by, vx, vy;
    double danger;       // 球朝己门的速度分量（横滚≈0、背离=0）
    double db;           // 门将到球距离
    double ball_goal;    // 球到己门线距离
    double gside;        // 球门在球的哪一侧（沿 x，±1）
    double opp_dmin;     // 最近对手到球距离
    bool   ball_still;   // 球速 < 0.2cm/帧（队员第81轮 2.0→0.2：慢滚活球≈0.4 不再当静止球出击）
    bool   opp_has_ball; // 对方持球（带滞回，由 gk_update_opp_hold 写入）
    bool   heading_goal; // 球会到达己方门线
    bool   on_target;    // 且过门线时在门框内
    double y_at_goal;    // 过门线时的 y
};

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
    v.ball_still = std::hypot(v.vx, v.vy) < 0.2;
    v.opp_has_ball = false;
    v.y_at_goal = 90.0;
    v.heading_goal = predict_y_at_x(v.bx, v.by, v.vx, v.vy, ctx.our_goal_x(), v.y_at_goal);
    v.on_target = v.heading_goal &&
                  v.y_at_goal >= goal_y_low() && v.y_at_goal <= goal_y_high();
    return v;
}

// 对方持球判定带滞回：连续 kGkOppHoldFrames 帧没检测到才撤销。
void gk_update_opp_hold(WorldModel &wm, GkView &v) {
    if (v.opp_dmin < 15.0) {
        wm.goalie_opp_hold = kGkOppHoldFrames;
    } else if (wm.goalie_opp_hold > 0) {
        --wm.goalie_opp_hold;
    }
    v.opp_has_ball = (wm.goalie_opp_hold > 0);
}

// 门线前 depth cm 的 x
double gk_line_x(const TeamContext &ctx, double depth) {
    return ctx.our_goal_x() + ctx.attack_dir() * depth;
}

// 走到 (x,y)：y 夹在门框内侧 [kGkYLo,kGkYHi]，再夹回罚球区
void gk_goto(const TeamContext &ctx, RobotState &r, double x, double y,
             motion::TargetMode mode) {
    y = clamp(y, kGkYLo, kGkYHi);
    clamp_goalie_area(ctx, x, y);
    motion::position(r, x, y, mode);
}

// 门将是否已在推球方向 (dx,dy) 的正后方（横向偏差 ≤3cm）——三点共线才能直线穿球
bool gk_aligned(const RobotState &r, const GkView &v, double dx, double dy) {
    double along  = (r.x - v.bx) * dx + (r.y - v.by) * dy;
    double across = (r.x - v.bx) * (-dy) + (r.y - v.by) * dx;
    return (along <= 0.0) && (std::fabs(across) <= 3.0);
}

// 机头偏离推球方向太多时原地转正（返回 true = 本帧在转）。
//   不先转正的话 motion::position 会落进 |te|∈(85°,95°) 的原地自转死区，贴着球晃不推。
bool gk_turn_to(RobotState &r, double dx, double dy) {
    double aim = angle_to(0.0, 0.0, dx, dy);
    if (std::fabs(angle_diff(aim, r.rot)) > kGkClearAlignTol) {
        motion::position_aligned(r, r.x, r.y, aim, 2.0, kGkClearAlignTol);
        return true;
    }
    return false;
}

// 站在球-门心连线上、球前 12cm 处封角度（深度夹在 [kGkGuardDist, 40]，不过度上抢）
void gk_block_ball_line(const TeamContext &ctx, RobotState &r, const GkView &v) {
    double back  = std::max(0.0, v.ball_goal - 12.0);
    double cx = gk_line_x(ctx, goalie_block_depth(ctx, v.bx, kGkGuardDist));
    double cy = clamp(90.0 + (v.by - 90.0) * (back / std::max(1.0, v.ball_goal)), kGkBlockYLo, kGkBlockYHi);
    clamp_goalie_area(ctx, cx, cy);
    motion::position(r, cx, cy);
}

// ---------------- 规则：命中则写好动作并返回 true ----------------

// 点球：锁门线中央
bool gk_rule_penalty(WorldModel &wm, int id, const GkView &) {
    if (wm.game_state == PM_PenaltyKick_Blue || wm.game_state == PM_PenaltyKick_Yellow) {
        motion::position(wm.home[id], gk_line_x(wm.ctx, 3.0), 90.0);
        return true;
    }
    return false;
}

// 线段 (x0,y0)→(x1,y1) 到点 (px,py) 的最近距离
double seg_point_dist(double x0, double y0, double x1, double y1, double px, double py) {
    double dx = x1 - x0, dy = y1 - y0;
    double len2 = dx * dx + dy * dy;
    double t = (len2 < 1e-9) ? 0.0 : clamp(((px - x0) * dx + (py - y0) * dy) / len2, 0.0, 1.0);
    return std::hypot(x0 + t * dx - px, y0 + t * dy - py);
}

// 回门路线是否会碰球：球按当前速度外推 kGkBehindHorizon 帧，逐点查到路线的净空
bool gk_path_hits_ball(const RobotState &r, const GkView &v, double tx, double ty) {
    for (int k = 0; k <= kGkBehindHorizon; k += 2) {
        if (seg_point_dist(r.x, r.y, tx, ty, v.bx + v.vx * k, v.by + v.vy * k) < kGkPathClear)
            return true;
    }
    return false;
}

// 球被甩到门将身后：绕到球的门侧。但门口没有门侧空间（球离门线 < kGkNoRoom）、
//   或直线回门会擦到球（含滚动外推）时，从场侧直开门线就是把球推进自家门
//   （2026-09-28 真机 3 个乌龙全是这样）→ 只在球的场侧横移让开，不碰球。
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

// 对准球的直线提前堵（见 gk_line_block_point）。去目标的路线会擦到在门口前的球时
//   （从场侧把球顶进门）不接，交给"球被甩到身后"/侧步让开。
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

// 死球期 / 球在角区：推球即犯规 → 只回门前站位，不触球
bool gk_rule_no_push(WorldModel &wm, int id, const GkView &v) {
    if (push_allowed(wm)) return false;
    motion::position(wm.home[id], gk_line_x(wm.ctx, kGkGuardDist),
                     clamp(v.by, kGkTrackYLo, kGkTrackYHi), motion::TM_STOP);
    return true;
}

// 门口球已越过门将：先侧向让开，别从场侧把球追进门（见 gk_side_step_point）
bool gk_rule_side_step(WorldModel &wm, int id, const GkView &) {
    double tx = 0.0, ty = 0.0;
    if (!gk_side_step_point(wm, id, tx, ty)) return false;
    motion::position(wm.home[id], tx, ty, motion::TM_PASS);
    return true;
}

// 对方持球：绝不前出（对方一变向就甩开门将、自摆乌龙），锁门前浅位跟球 y。
//   例外：对方已带到门口（<30cm）且门将贴球（<25cm）→ 上前封球-门连线，否则 1v1 门洞大开。
TUNABLE(kGkOppLockEnable, 1.0);  // 封角前压总开关：1=开（对方持球球远时也封角），0=回退锁死门线前10cm
TUNABLE(kGkOppLockDepth, 30.0);  // 封角深度上限 cm（kGkOppLockEnable=1 时生效）
bool gk_rule_opp_ball_lock(WorldModel &wm, int id, const GkView &v) {
    if (!v.opp_has_ball) return false;
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    if (v.ball_goal < 30.0 && v.db < 25.0) {
        gk_block_ball_line(ctx, r, v);
    } else if (kGkOppLockEnable < 0.5) {
        // 总开关关：恢复「锁死门线前10cm」旧行为
        motion::position(r, gk_line_x(ctx, kGkGuardDist), clamp(v.by, kGkTrackYLo, kGkTrackYHi));
    } else {
        // 对方持球但球还没到门口（>30cm 或 门将离球≥25cm）→ 也站球-门连线封角：
        //   深度 = 球距门−12cm（连续），上限收窄到 kGkOppLockDepth(30)。
        //   不再钉死门线前10cm 干等单刀（见 docs/06 第101轮：opp_ball_lock 94.7% 时间走锁门线）。
        double back  = std::max(0.0, v.ball_goal - 12.0);
        double depth = std::min(kGkOppLockDepth, std::max(kGkGuardDist, v.ball_goal - 12.0));
        double cx = gk_line_x(ctx, depth);
        double cy = clamp(90.0 + (v.by - 90.0) * (back / std::max(1.0, v.ball_goal)),
                          kGkBlockYLo, kGkBlockYHi);
        clamp_goalie_area(ctx, cx, cy);
        motion::position(r, cx, cy);
    }
    return true;
}

// 第 92 轮：推球准备点（球后 kGkPushDist）的 y 若落在门将站位带 [kGkYLo,kGkYHi] 外，
//   gk_goto 会把目标夹回带内 → 永远站不到球后、永远"未对准"，来回冲不触球
//   （真机 s50/s52/s53 门球 y=72 卡 98~197+ 帧）。先镜像到另一侧（往边路推，不喂中路），
//   仍够不着再直线推出。
TUNABLE(kGkKickReach, 1.0);  // 0 = 回滚（不修正推球方向）
void gk_reachable_dir(const TeamContext &ctx, const GkView &v, double &dx, double &dy) {
    if (kGkKickReach < 0.5) return;
    auto ok = [&](double ddy) {
        double py = v.by - ddy * kGkPushDist;
        return py >= kGkYLo && py <= kGkYHi;
    };
    if (ok(dy)) return;
    if (ok(-dy)) { dy = -dy; return; }
    // 球本身在带外（y<74 或 >106）：取最小斜度让准备点正好落回带内
    dy = clamp((v.by - clamp(v.by, kGkYLo + 1.0, kGkYHi - 1.0)) / kGkPushDist, -0.8, 0.8);
    dx = ctx.attack_dir() * std::sqrt(1.0 - dy * dy);
}

// 门前静止球（门球/定位球重启）：门将主动穿球推出，否则球滚回门线外反复触发门球。
bool gk_rule_restart_kick(WorldModel &wm, int id, const GkView &v) {
    if (!(v.ball_goal < 45.0 && v.ball_still)) return false;
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    bool contested = v.opp_dmin < 40.0;
    bool goal_side = (r.x - v.bx) * v.gside > 0.0;
    // 贴门线（<20cm）或对手正抢（<40cm），且门将已在球门侧：不绕侧面，直接穿球推出。
    //   绕行会被强制回门/侧步让开来回拽，被抢时弧线也输给直线冲球的对手。
    //   被抢 → 直线远离己门抢时间；无人抢 → 往侧面空当推。
    if ((v.ball_goal < 20.0 || contested) && goal_side) {
        double dx = ctx.attack_dir(), dy = 0.0;
        if (!contested) gk_clear_direction(wm, id, v.bx, v.by, dx, dy);
        gk_reachable_dir(ctx, v, dx, dy);
        if (v.db < 25.0 && gk_aligned(r, v, dx, dy)) {
            if (!gk_turn_to(r, dx, dy))
                gk_goto(ctx, r, v.bx + dx * kGkKickThrough, v.by + dy * kGkKickThrough,
                        motion::TM_PASS);
            return true;
        }
        gk_goto(ctx, r, v.bx - dx * kGkPushDist, v.by - dy * kGkPushDist, motion::TM_PASS);
        return true;
    }
    double dx = 0.0, dy = 0.0;
    gk_clear_direction(wm, id, v.bx, v.by, dx, dy);
    gk_reachable_dir(ctx, v, dx, dy);
    bool ready = v.db < 25.0 && gk_aligned(r, v, dx, dy);
    if (ready && gk_turn_to(r, dx, dy)) return true;
    // 球贴门线（<15cm）且门将在球外侧：直奔球后会穿球把球顶进自家门 → 先横移到球侧 22cm
    if (v.ball_goal < 15.0 && (r.x - v.bx) * v.gside < 0.0) {
        double side = (r.y >= v.by) ? 1.0 : -1.0;
        gk_goto(ctx, r, v.bx - v.gside * 10.0, v.by + side * 22.0, motion::TM_STOP);
        return true;
    }
    if (ready) {
        gk_goto(ctx, r, v.bx + dx * kGkKickThrough, v.by + dy * kGkKickThrough, motion::TM_PASS);
    } else {
        gk_goto(ctx, r, v.bx - dx * kGkPushDist, v.by - dy * kGkPushDist, motion::TM_STOP);
    }
    return true;
}

// 球已在门框内轨迹上：抢门线预测落点（排在封连线/清球之前，那两个目标点会被带偏）
bool gk_rule_cover_line(WorldModel &wm, int id, const GkView &) {
    double tx = 0.0, ty = 0.0;
    if (!gk_cover_line_point(wm, id, tx, ty)) return false;
    motion::position(wm.home[id], tx, ty, motion::TM_PASS);
    return true;
}

// 对手贴着静止球、机头对准：按它的朝向预测出球线，提前封门线落点
bool gk_rule_opp_kick_line(WorldModel &wm, int id, const GkView &) {
    double y_pred = 90.0;
    if (!opp_kick_target_y(wm, y_pred)) return false;
    motion::position(wm.home[id], gk_line_x(wm.ctx, 3.0), clamp(y_pred, kGkYLo, kGkYHi),
                     motion::TM_PASS);
    return true;
}

// 门前对手贴球（滞回已撤但仍 <25cm）：不冲球（对方一变向就换侧进门），封球-门连线
bool gk_rule_press_door(WorldModel &wm, int id, const GkView &v) {
    if (!(v.ball_goal < 45.0 && v.opp_dmin < 25.0)) return false;
    gk_block_ball_line(wm.ctx, wm.home[id], v);
    return true;
}

// 门将贴球站定防乌龙（docs/06 第101轮）：球贴门线且门将贴球、对手已离开球 →
//   停轮站定，用身体封门，绝不绕行/推球。clear 的"绕到球门侧+侧移"分支在贴球时
//   会把球拖进自家门（真机 6:1 复盘 5 个疑似乌龙全是这样，门将前后 -6~-15cm = 站球外侧）。
//   —— 对齐 run_passive 第37轮"门线球站定防乌龙"，补齐门将缺的这条兜底。
//   排在 clear 之前：对手贴球时 press_door 已在上游封连线，此处只管"自由球贴门线"。
bool gk_rule_goal_line_hold(WorldModel &wm, int id, const GkView &v) {
    if (!(v.ball_goal < 15.0 && v.db < 14.0)) return false;
    TRACE_MARK(wm.home[id]);
    wm.home[id].vl = 0.0;
    wm.home[id].vr = 0.0;
    return true;
}

// 球在脚下：解围。会进门的球沿球-门连线水平推（门将始终挡在线上，慢球不从身侧漏），
//   否则往侧面空当清。球在门将身后（差值 ≤ kGkBallBehindMargin，更大的已被强制回门接走）
//   → 沿 y 侧移绕到门侧，绝不直线穿球；门将在球后 → 直线穿球，保证出球速度。
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
        double side = (r.y >= v.by) ? 1.0 : -1.0;
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

// 贴边墙朝门滚、且会进门的球：提前赶到门线预测落点（赶路不刹车，最后 15cm 才停准）
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

// 会进门的球：按朝门球速动态前压封角（慢球贴门，快球到罚球区前缘），罚球区内有对手则回缩
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
    // 前压深度上限：不越过对方最靠近己门（最前插）的球员，留 kGkOppFrontPad 余量。
    //   保证门将始终站在"对方最前插球员"和球门之间，身后不留回敲/插上的空当；
    //   下限仍锁 kGkGuardDist，对方压得再深门将也不退到门线后。
    double opp_front = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i)
        opp_front = std::min(opp_front, ctx.dist_our_goal(wm.opp[i].x));
    depth = std::max(kGkGuardDist, std::min(depth, opp_front - kGkOppFrontPad));
    double out_x = gk_line_x(ctx, v.ball_goal < 15.0 ? 3.0 : depth);
    // 拦截点 y：球轨迹在 out_x 处的 y；不可用时取球-门连线与 out_x 的交点
    double iy = 90.0;
    if (!predict_y_at_x(v.bx, v.by, v.vx, v.vy, out_x, iy)) {
        if (std::fabs(v.bx - ctx.our_goal_x()) > 1e-6) {
            double t = (out_x - ctx.our_goal_x()) / (v.bx - ctx.our_goal_x());
            t = std::max(0.0, std::min(1.0, t));
            iy = 90.0 + t * (v.by - 90.0);
        }
    }
    // 进门点比拦截点更靠上/下角时改封进门点（demo 爱打门柱内侧上角）
    if (std::fabs(v.y_at_goal - 90.0) > std::fabs(iy - 90.0)) iy = v.y_at_goal;
    clamp_goalie_area(ctx, out_x, iy);
    motion::position(wm.home[id], out_x, iy);
    return true;
}

// 无威胁：门前站位跟球 y；球贴门线（<15cm）时后撤到 3cm，防球沿门线从身后滚过
bool gk_rule_default(WorldModel &wm, int id, const GkView &v) {
    motion::position(wm.home[id], gk_line_x(wm.ctx, v.ball_goal < 15.0 ? 3.0 : kGkGuardDist),
                     clamp(v.by, kGkTrackYLo, kGkTrackYHi));
    return true;
}

using GkRule = bool (*)(WorldModel &, int, const GkView &);

// 门将规则表：优先级从高到低，第一条命中即生效。
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

// 临时诊断（第101轮单刀复盘，用完删）：记录门将每帧命中规则到 C:\Strategy\goalie_trace.csv
const char *const kGoalieRuleNames[] = {
    "no_push", "side_step", "opp_ball_lock", "restart_kick", "cover_line",
    "opp_kick_line", "press_door", "goal_line_hold", "clear", "wall_ball",
    "shot_block", "default",
};

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

}  // anonymous namespace

void run_goalie(WorldModel &wm, int id) {
    GkView v = gk_view(wm, id);
    const char *hit = "none";
    // 点球和"球被甩到身后"排在对方持球滞回更新之前：命中的帧不推进滞回计数。
    if (gk_rule_penalty(wm, id, v)) { hit = "penalty"; }
    else if (gk_rule_line_block(wm, id, v)) { hit = "line_block"; }
    else if (gk_rule_ball_behind(wm, id, v)) { hit = "ball_behind"; }
    else {
        gk_update_opp_hold(wm, v);
        for (int i = 0; i < (int)(sizeof(kGoalieRules) / sizeof(kGoalieRules[0])); ++i) {
            if (kGoalieRules[i](wm, id, v)) { hit = kGoalieRuleNames[i]; break; }
        }
    }
    gk_trace(hit, wm, id, v);
}

// 对方门区停留红线（docs/13 方案 C）：规则"门区除门将外停留 >20 周期 → 罚点球"。
//   纯停留限 8 帧：人在门区且（球不在门区 或 球不在脚下>25cm）= 明显滞留。
//   必须留足"撤出时间"：实测撤出 30~40cm 要 10~12 帧，红线 20 帧内必须离开；
//   限 10 帧时 11+12=23 仍越线（sim 实测 3.4 次/场），8+12=20 刚好卡线。
//   带球推射（球在门区且贴脚≤25cm）不累计，限 8 不影响攻门。
//   总时长兜底 20 帧：带球反复推不进（真实平台 8/29 实测 21~30 帧被判 3 点球）
//   也不能无限续命，规则红线就是 20，宁可放弃机会不送点球。
// —— 补射跟进（docs/17 讲解稿规划）：射门被挡后球在门前低速反弹 → 追进去补一脚 ——
//   判据：球在门前锥形区内且球速低于可抢阈值 + 我方就近（距离限制保证追进门区
//   途中 4~8 帧内贴球，"纯停留"帧计数不超限）；贴球(≤25cm)后 active_ga_frames
//   清零（攻门作业不累计），走带球/推射链（plan_shoot 每帧重评，球距门<70cm 即射）；
//   追不到/超时由 kActiveGaLimit 撤出分支兜底，不会赖在门区送判罚。
TUNABLE(kReboundRushSpeed, 7.34364);  // cm/帧：反弹球可抢速度阈值（GK扑出/挡回典型 <10）
TUNABLE(kReboundRushDist, 51.3294);  // cm：我方距球超过此值不冲（就近补，防全场狂奔）
// —— 禁区前沿变角推射次数上限（docs/17，模仿官方"沿变角推球"）——
TUNABLE(kMaxShootPushes, 1);  // 同一轮进攻连续推球尝试上限（防禁区死磕送判罚）
// —— 到点定向射门（docs/18 §8）：准备点距离/位置容差/朝向容差 ——
//   球后 20cm：够得着球（下一帧直线推穿能碰到球心），又不至于贴太近把球顶走
//   位置容差 3cm：制动包线停住精度 ~1.5cm，留余量
//   朝向容差 10°：1m 处横向偏差 = 100·tan10° ≈ 17.6cm < 门半宽 20cm → 能射正；
//     旧口径是 40°（1m 处偏 92cm = 两个门宽），真机实测机头−瞄准线 p50=51.6°、≤10° 仅 6%
TUNABLE(kPrepDist, 23.5578);
// 罚点球助跑距离（cm）：出球速度 = 撞球瞬间的机头速度，20cm 助跑只有 ~103cm/s，
//   40cm 外的点球飞行 ~17 帧 → 门将横移 17cm 就够到（真机 09-13 rlg 帧 2350 球被打偏）。
TUNABLE(kPenaltyPrepDist, 15.0);  // 15cm（原 35：真机实证倒车太久会被截）
// ⚠️ 第 66 轮已删除 kPenaltyNoBackOpp（45cm）：
//   真机 16:01 场实测对手在 34cm 时**仍然倒了车**（说明这条阈值规则没起作用），
//   现在改成"点球一律不倒车"（见 run_active 的点球分支），不再依赖对手距离。
double shoot_prep_dist(const WorldModel &wm) {
    return wm.in_penalty_exec ? kPenaltyPrepDist : kPrepDist;
}
TUNABLE(kPrepPosTol, 4.01318);
TUNABLE(kPrepAngTol, 13.1809);
// 对准尝试超时（帧）：球一直在动/被抢，死等对准会把机会全耗掉 → 超时按当前朝向推
//   ⚠️ sim A/B 反对本项（净胜 -1.63）：sim 的 carry 机制隐含"机头对着球"、
//   且是弱脚本门将（"快推"优于"推准"）→ 由用户决定真机观查（docs/06 第 47 轮）。
TUNABLE(kShootAlignTimeout, 54.7289);
// 带球推进的机头对准容差（度）：比射门(10°)略松，但远紧于旧的 40°
TUNABLE(kDribAngTol, 31.423);
// 争抢态（第 81 轮，见 run_active）：对手离球 < kContestOppDist 且我离球 < kContestReach
constexpr bool kContestEnabled = true;   // 回滚开关
TUNABLE(kContestOppDist, 20.0);
TUNABLE(kContestReach, 60.0);
TUNABLE(kContestLead, 8.0);
// 子开关（消融用，1=开）：不等接球人 / 球后直接推穿。
// 曾试第三项"无球不去接球点"：200 局 scripted −2.4、yellow −1.1、self −1.4 → 已删。
TUNABLE(kContestNoWait, 1.0);
TUNABLE(kContestPush, 1.0);
// 争抢直冲（第 90 轮，学官方 demo：目标就是球、不绕球后、不减速）。
//   真机 09-28/29 黑匣子：争抢帧里主攻 60~74% 站在「撞过去球不会朝自家门」的一侧，
//   朝球速度中位却只有 0.24~0.34cm/帧（在绕球后准备点/原地转正），15 帧后我方先到球仅 18~30%。
//   推球方向 = 人→球方向；它在进攻方向上的分量 ≥ kChargeMinFwd 才冲（离己门 <kChargeOwnGuard 用更严的 kChargeMinFwdOwn）。
TUNABLE(kContestCharge, 1.0);      // 回滚开关
TUNABLE(kChargeMinFwd, -0.2);      // cos：约 ≤100° 偏离进攻方向都可冲（横推也算抢到）
TUNABLE(kChargeMinFwdOwn, 0.3);    // 己方门前：必须明显朝前推
TUNABLE(kChargeOwnGuard, 70.0);    // cm
TUNABLE(kChargeThrough, 20.0);     // 目标 = 球心沿冲撞方向再过 20cm（穿球，不在球前停）
TUNABLE(kPassivePress, 1.0);   // PASSIVE 争抢时从球门侧上抢（run_passive 末尾）
TUNABLE(kPassivePressDepth, 130.0);   // 深度扫 110/130/150/170/全场：130 为拐点（净胜 +4~5，单人滞留不增）
// 防守到位迎球（第 103 轮，用户指令："防守不能对准球冲过来的方向"）：
//   断球点在球来路上时，走位到点后原地转正、让机头朝球来的方向（-v）站定。
//   本平台球被撞出去的方向 ≈ 机头方向 ⇒ 斜着迎球只会把球横着顶出去、动量照旧。
TUNABLE(kDefFaceIncoming, 1.0);    // 回滚开关（0 = 旧的"只给位置不给朝向"）
TUNABLE(kDefArriveDist, 6.0);      // cm：进入该半径即停车转正迎球
TUNABLE(kDefFaceAngTol, 12.0);     // 度：迎球朝向允许误差
TUNABLE(kActiveGaLimit, 8);
TUNABLE(kActiveGaTotal, 18);  // 在门区总时长兜底：平台 20 周期判罚红线，留 2 帧余量
                                        // （8/29 实测被判滞留 21~30 帧；太紧会打断合法带球攻门 10~15 帧）

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

// ============================================================
// 接球会合点（第 103 轮，用户指令："接球需要提前到达位置，而不是后退、绕一下再接球"）
// ============================================================
// 总开关：1=接球人不去"锁点"站着等，而是算会合点、提前站过去迎球；0=回退原行为。
TUNABLE(kRecvMeetBall, 1.0);
// 提前量（帧）：接球人要比球早到这么多帧（转身迎球用的时间）。
TUNABLE(kRecvLead, 6.0);
// 接球人的估算速度（cm/帧）：与 pass.RECEIVER_READY_SPEED 同口径。
TUNABLE(kRecvSpeed, 2.0);
// 会合点相对锁点的最大偏移（cm）：球被踢飞/回传时不追出这个范围（防"追球追回自家半场"）。
TUNABLE(kRecvMeetMaxOff, 60.0);
// 到位判定半径（cm）：进入即停车原地转正迎球。
TUNABLE(kRecvArriveDist, 8.0);
// 接球路径禁止倒车（1=开，0=允许倒着走）。
TUNABLE(kRecvNoReverse, 1.0);

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
    // 旧安全规则先结算；只有它们均未取消、且球尚未确认出脚时，才补判对手抢点。
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

// 接球目标点（第 103 轮，用户指令："接球点的计算有问题 / 接球要提前到达位置"）：
//   球在飞 → 会合点（球来路上、"我比球早到 kRecvLead 帧"的点），提前站过去；
//   球停着 / 会合点离锁点太远（球被踢飞、回传）→ 锁点（原行为，不追出防区）。
//   返回 true 时 out_aim = 迎球朝向（球来向的反方向）。
bool receiver_meeting_target(const WorldModel &wm, const CoopPassTask &task, int id,
                             double &tx, double &ty, double &out_aim) {
    tx = task.rx; ty = task.ry; out_aim = 0.0;
    if (kRecvMeetBall < 0.5) return false;
    const RobotState &r = wm.home[id];
    double mx = 0.0, my = 0.0, maim = 0.0;
    if (!ball_meeting_point(wm, r.x, r.y, kRecvSpeed, kRecvLead, mx, my, maim)) return false;
    if (dist(mx, my, task.rx, task.ry) > kRecvMeetMaxOff) return false;
    // 纪律红线（第 103 轮 A/B 暴露）：会合点是"球轨迹上的原始点"，没经过 clamp_receive_point
    //   的场地/门区过滤。若它落在对方门区（球被压到门前时很常见），接球人就会踩门区 →
    //   sim 50 局实测"门区2+人帧 +40.8"直接出噪声带（犯规风险）。落红线内一律弃用、回退锁点。
    const TeamContext &ctx = wm.ctx;
    if (in_opp_goal_area(ctx, mx, my) || in_goal_area(ctx, mx, my) || in_no_push_zone(mx, my)) return false;
    tx = mx; ty = my; out_aim = maim;
    return true;
}

// 原地转身（不产生位移）：用于"禁止倒车"时先转向、再前进。
void turn_in_place(RobotState &r, double te_deg) {
    double w = clamp(0.22 * te_deg, -14.0, 14.0);
    if (std::fabs(w) < 2.5) w = (te_deg > 0.0) ? 2.5 : -2.5;
    r.vl = clamp(-w, -motion::kMaxWheel, motion::kMaxWheel);
    r.vr = clamp( w, -motion::kMaxWheel, motion::kMaxWheel);
}

void run_receiving_receiver(WorldModel &wm, const CoopPassTask &task, int id) {
    RobotState &receiver = wm.home[id];
    // 目标点：会合点优先（球在飞时"迎上去接"，不是"站在锁点等球滚过来"）。
    double tx = task.rx, ty = task.ry, meet_aim = 0.0;
    const bool has_meet = receiver_meeting_target(wm, task, id, tx, ty, meet_aim);
    // 禁止倒车（用户指令）：目标在正后方 → 先原地转身，绝不倒着走。
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
        // 近球时先原地迎球；把当前位置作为目标，避免离锁点稍远时 position_aligned 退回普通赶路。
        motion::position_aligned(receiver, receiver.x, receiver.y, desired_rot, 3.0, angle_tol);
    } else {
        const double target_distance = dist(receiver.x, receiver.y, tx, ty);
        if (!std::isfinite(target_distance) || target_distance <= 3.0) {
            motion::stop(receiver);
        } else {
            const double target_rot = angle_to(receiver.x, receiver.y, tx, ty);
            const double forward_error = std::fabs(angle_diff(target_rot, desired_rot));
            const double reverse_error = std::fabs(angle_diff(target_rot + 180.0, desired_rot));
            // 差速车不能横移：目标点若在机头侧面就先面向球等待；前后方向一致才低速补位。
            if (std::min(forward_error, reverse_error) <= 2.0 * angle_tol)
                go_to(tx, ty);
            else
                motion::stop(receiver);
        }
    }

    // 只缩小共同前进量，保留左右轮差值形成的转向；所以近球会收油，但仍能转身迎球。
    const double min_scale = std::isfinite(kPassReceiveMinDriveScale)
                           ? clamp(kPassReceiveMinDriveScale, 0.0, 1.0) : 0.25;
    const double drive_scale = clamp(ball_distance / kPassReceiveSlowRadius, min_scale, 1.0);
    double drive = 0.5 * (receiver.vl + receiver.vr);
    const double turn = 0.5 * (receiver.vr - receiver.vl);
    drive *= drive_scale;
    receiver.vl = clamp(drive - turn, -motion::kMaxWheel, motion::kMaxWheel);
    receiver.vr = clamp(drive + turn, -motion::kMaxWheel, motion::kMaxWheel);
}

// 接球准备（第 103 轮）：不是"走到锁点站着等球滚过来"，而是算会合点、
//   提前站过去、到位后原地转正迎球站定（球撞在身体正面）。
void run_preparing_receiver(WorldModel &wm, const CoopPassTask &task, int id) {
    RobotState &receiver = wm.home[id];
    double tx = task.rx, ty = task.ry, aim = 0.0;
    if (receiver_meeting_target(wm, task, id, tx, ty, aim)) {
        motion::arrive_facing(receiver, tx, ty, aim, kRecvArriveDist, kPassReceiveAngleTol,
                              /*allow_reverse=*/kRecvNoReverse < 0.5);
        return;
    }
    // 没有会合点（球还没出脚/球停着）：去锁点，但同样禁止倒车（用户指令）。
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

    // 点球瞄准锁（docs/06 第 66 轮）：执行期结束就在下一帧解锁，下次点球重新算方向。
    //   放在函数最前面 → 任何早退分支都绕不过（放在射门段里会被早退跳过，实测踩过）。
    if (!wm.in_penalty_exec) wm.pen_aim_locked = false;

    // 对方门区停留计数（docs/13 方案 C）：每帧更新，离开门区清零。
    //   口径：人在门区，若 球不在门区 或 球不在脚下(>25cm) → 计"纯停留"。
    //   —— 球在门区且贴着(≤25cm) = 正在攻门作业（直线推射 10 帧内完成），不累计；
    //   —— 其余（含追球沿边线穿过门区——实测违例主因：球沿左边线上行，ACTIVE
    //       直线追球正好切穿门区 30+ 帧）一律累计，超限撤出。
    //   总时长 active_ga_total 兜底：带球在门区反复推不进（真实平台 8/29 实测
    //   21~30 帧被判 3 点球）也不能无限续命。
    if (in_opp_goal_area(ctx, r.x, r.y)) {
        ++wm.active_ga_total;
        bool ball_in_ga = in_opp_goal_area(ctx, wm.ball.x, wm.ball.y);
        if (!ball_in_ga || dist(r.x, r.y, wm.ball.x, wm.ball.y) > 25.0) ++wm.active_ga_frames;
    } else {
        wm.active_ga_frames = 0;
        wm.active_ga_total = 0;
    }

    // —— 禁区推射计数维护（docs/17）——
    //   冷却递减；球离开射程(>75)或球权易主且人在球外(>25) → 清零重计下一轮。
    if (wm.shoot_push_cd > 0) --wm.shoot_push_cd;
    if (ctx.dist_opp_goal(wm.ball.x) > 75.0 ||
        (!wm.we_have_ball && dist(r.x, r.y, wm.ball.x, wm.ball.y) > 25.0)) {
        wm.shoot_push_count = 0;
        wm.shoot_push_last_side = 0;
    }

    // 角区卡球计时：球在角区(距角 <30cm)且基本静止(速度<1cm/帧) → 连续帧数+1；
    //   否则清零。（>30 帧才认为"真卡住"：路过/刚弹到角的球不算，避免救球喧宾夺主。）
    if ((wm.ball.x < 30.0 || wm.ball.x > 190.0) &&
        (wm.ball.y < 30.0 || wm.ball.y > 150.0) &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 1.0) {
        ++wm.corner_ball_frames;
    } else {
        wm.corner_ball_frames = 0;
    }

    // —— docs/06 第 49 轮：推球合法性守卫（动作入口级）——
    //   本平台"让球动起来"的所有动作都是推球（没有踢球动作），所以这里一旦拦住，
    //   后面的射门/带球/围困/传球/角区救球全部不会执行 → 不可能在角区或死球期推球。
    //   代价：角上的球不去碰（让平台判 FreeBall）；收益：不再吃"每 4 次 +1 球"的犯规。
    if (!push_allowed(wm)) { if (wm.coop_pass_task.active) wm.coop_finish(CoopOutcome::PushForbidden); hold_out_of_corner(wm, r); return; }

    // 对方门球/定位球重启：球停死在对方门区(球门前 50cm) → 别冲进去抢。
    //   球是死球，冲进去射门/追球会横穿全场撞进对方门区，冲撞对方门将(门区受保护)
    //   或在门区停留犯规；实测自博弈每局门球 ACTIVE 都从己方半场直冲对方门区。
    //   等球被开出(球速起来)再正常进攻；先站对方罚球区外沿外 5cm 等反击。
    //   ⚠️ 例外：我方主罚点球（in_penalty_exec，strategy.cpp 维护）——球同样静止在
    //   对方罚球点，但必须去踢！真实平台 8/29 点球 0/3 全丢的根因就是这里拦截：
    //   逐帧复盘 ACTIVE 助跑后只停在门区外沿 (85,90)，从不执行射门。
    if (!wm.in_penalty_exec &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 1.0 &&
        std::fabs(wm.ball.x - ctx.opp_goal_x()) < 50.0 &&
        std::fabs(wm.ball.y - 90.0) < 20.0) {
        // 等待超时：对方迟迟不开球（球卡死/对方无人处理）→ 主动去推/射，把球带出门区。
        //   真实 8/31 镜像内战 0:0 根因：球静止对方门区 (205,90) 135 秒无人处理，
        //   进攻方死球等待永不超时、门将清球又不穿过球 → 双方僵持。
        //   正常门球对方 2 秒内会来开（球动起来）→ 100 帧超时不会误伤。
        if (++wm.dead_ball_frames > 100) {
            wm.dead_ball_frames = 0;
            // 不 return —— 落到 plan_shoot/带球逻辑：进门区作业（方案 C 攻门豁免），
            // 直线推穿把球推出对方门区或直接射门
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

    // —— 对方门区停留时限（docs/13 方案 C，放在射门之前）——
    // 规则：门区除门将外单人停留 >20 周期 → 罚点球。真实平台 8/29 实测：
    // ACTIVE 带球在门区滞留 21~30 帧即被判 3 个点球（0/3 全丢）。
    // 必须在射门前检查：否则"射门优先"会让 ACTIVE 一直站在门区推球，
    // 停留帧数无限累积（sim 实测单人>20帧 1.9 次/场降不下去）。
    // 超限后不恋战：传球给禁区外沿接应，无传球则撤到门区前缘外（x=60cm 线，
    //   出区即清零，球留在门区由对方清掉，也比送点球划算）。
    if (wm.active_ga_frames > kActiveGaLimit || wm.active_ga_total > kActiveGaTotal) {
        if (wm.coop_pass_task.active) wm.coop_finish(CoopOutcome::GoalDiscipline);
        if (wm.coop_ball_control.active) wm.coop_control_end(CoopOutcome::GoalDiscipline);
        ++wm.ga_retreat_fires;   // 诊断用（sim_bench 验证超限撤出触发）
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

    // 争抢态（第 81 轮）：对手已贴球、我在够得着的距离 → 一切"等"的动作让位于上抢：
    //   不原地转正、不等接球人就位、不去队友的接球点。sim 分支追踪：争抢帧里 ACTIVE
    //   72% 在犹豫，前三名正是这三类"等"（去接球点 / 原地转正 / 停车等接球人）。
    double opp_ball = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i)
        opp_ball = std::min(opp_ball, dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y));
    // 我已明显先到（比对手近 kContestLead 以上）= 控球而非争抢，照常组织传球
    const double my_ball = dist(r.x, r.y, wm.ball.x, wm.ball.y);
    const bool contested = kContestEnabled && !wm.in_penalty_exec &&
                           opp_ball < kContestOppDist && my_ball < kContestReach &&
                           my_ball > opp_ball - kContestLead;
    if (contested && kContestNoWait > 0.5 && wm.coop_pass_task.active && !pass_receiver_ready(wm))
        wm.coop_finish(CoopOutcome::OpponentFirst);
    // 角区救球（治"FreeBall 13 次/场"：球卡四角无人救 → 判争球）——
    // ⚠️ docs/06 第 49 轮修订：**只在球离角足够远时才救**，且避开"绕到球后"这个动作。
    //   旧版只判球距角 <22cm 就算"深角不救"，实测仍吃 5 次 "No pushing" 犯规
    //   （因为"球后 8cm"的准备点会落在角区内 → 驱车过去把球顶进角里）。
    //   现口径：球距角 < kCornerNoPushR(35cm) 一律不救；准备点也必须 ≥35cm。
    // 仅救"角区外环"的卡球：该环在平台禁止推球区之外，推球合法；
    //   球压到角心则**不救**——规则"禁止推球区推球 = 犯规(每4次+1球)+判争球"，
    //   深角球等平台判僵局重置，比送犯规划算。
    //   救球执行：贴近时直线穿过球推向场心方向，让球离开墙角继续比赛；
    //   还远/在侧面时先绕到球后(角落侧)对准再推。
    if (wm.corner_ball_frames > 30) {
        double db = dist(r.x, r.y, wm.ball.x, wm.ball.y);
        if (db < 80.0) {
            bool deep = in_no_push_zone(wm.ball.x, wm.ball.y);   // 35cm 口径（原 22cm 太松）
            if (!deep) {
                if (wm.coop_pass_task.active) wm.coop_finish(CoopOutcome::Corner);
                ++wm.corner_rescue_events;   // 统计用（sim_bench 验证救球触发）
                double ex = 110.0 - wm.ball.x, ey = 90.0 - wm.ball.y;
                double elen = std::hypot(ex, ey);
                if (elen > 1e-6) { ex /= elen; ey /= elen; }
                // 准备点（球后 8cm，角落侧）也要在角区外，否则放弃救球
                double back_x = wm.ball.x - ex * 8.0, back_y = wm.ball.y - ey * 8.0;
                double te_ball = angle_diff(angle_to(r.x, r.y, wm.ball.x, wm.ball.y), r.rot);
                if (db < 22.0 && std::fabs(te_ball) < 40.0) {
                    motion::position(r, wm.ball.x + ex * 30.0, wm.ball.y + ey * 30.0, motion::TM_PASS);   // 穿球踢出场心=经过型
                } else if (!in_no_push_zone(back_x, back_y)) {
                    motion::position(r, back_x, back_y);
                } else {
                    hold_out_of_corner(wm, r);   // 准备点在角区内 → 不冒险，退到场心侧
                }
                return;
            }
        }
    }

    // 普通 PassPlan 已锁点后，和配合传球一样只沿锁定方向推进；不走射门计次。
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

    // —— 射门：直线推穿（治真实平台"带球射门系统性偏下"）——
    // 原单点推球：高速冲到球后 8cm 推球点时还边转边铲，推球方向 = 接近轨迹
    //   方向（被出发点带偏），球被斜推偏出（实测球 y 90→65 偏出门柱，助跑 175cm 太长）。
    // 现改为"贴球后直线推穿"：已贴在球后(≤22cm)且朝向大致对准(≤40°)时，
    //   目标=球前 20cm，直线加速穿过球，推球方向=瞄准线，方向不再被带偏；
    //   还远/在侧面时先绕到球后沿瞄准线的站位点（球后 20cm）对准再推。
    // ⚠️ docs/18 §8 记录：本轮试过把这里升级为"必须对准到 10° 才推（含就地转正）"，
    //   并用 motion::position_aligned 执行；sim 4 种子 A/B 净胜 -1.63（远超 0.66 噪声）
    //   → 已回退。原因：花时间转正 = 丢推进节奏/球权；sim 又是弱门将，"快推"比"推准"划算。
    //   真机（射正率 8% vs 对手 48%）是否值得为此付代价，**必须真机单开一轮验证**，
    //   不能拿 sim 判。到点定向的能力（motion::position_aligned）与单测已就位，随时可接。
    ShootPlan sp = plan_shoot(wm, id);
    // 争抢直冲（第 90 轮，见 kContestCharge）：已在射门线后面的交给下面射门推穿（方向更好），
    //   其余可安全撞的争抢帧直接朝球心全速冲过去。
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
    // —— 配合进攻（docs/06 第 69 轮，用户 2026-09-15 指令）——
    //   队友接球后的射门机会比我自己高 0.15 以上、且他能**安全接到** → 传给他（只向前传）。
    //   安全性（线路无遮挡/接球点 20cm 内无对手/距离≤120cm）在 pass.cpp 里判定；点球执行期不传。
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
    // —— 点球执行期：瞄准方向**锁一次**（docs/06 第 66 轮）——
    //   为什么：实测真机点球里瞄准方向每帧重算 → 准备点跟着漂移 → 机器人退到球后又折返，
    //   折返时偏离瞄准线 15~24cm，从球的侧上方掠过把球推偏（出球 0.7cm/帧、方向几乎垂直
    //   射门方向），平台判定"没开出"连发 3 次点球。
    //   锁定后执行期内方向恒定，机器人只需要"转正 → 推穿"两件事。
    if (wm.in_penalty_exec && sp.viable) {
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
    // 射门机会闸门（docs/18 §8）：≤70cm 无条件射（A/B 校准的主力区，别加闸门）；
    //   70~110cm 远射要 quality ≥ kShootNowQ。**远射档已按用户指令开启**
    //   （shoot.cpp `kFarShotEnabled = true`，2026-09-11；sim A/B 反对，数据见 docs/06 第 47 轮）。
    //   点球（penalty）quality 置 1 → 直接放行。
    const double kShootNowQ = 0.35;
    bool shoot_now = sp.viable &&
                     (sp.shot_dist <= 70.0 || sp.quality >= kShootNowQ || coop_pass);
    if (shoot_now && (coop_pass || wm.shoot_push_count < kMaxShootPushes)) {
        double bx = wm.ball.x, by = wm.ball.y;
        int this_side = (sp.aim_y > 90.0) ? 1 : -1;
        // 变角推射（docs/17）：同一轮已推 >=2 次且本次仍瞄上次同一侧 → 强制换另一侧重推
        // ⚠️ 借墙方案（docs/06 第 65 轮）**不适用**：那个覆盖逻辑是"直接瞄向门口的另一点"，
        //    用在借墙方案上会把方向改回直线（而直线正是被封掉才走借墙的）→ 必须排除。
        if (!sp.bank && !coop_pass && wm.shoot_push_count >= 2 && wm.shoot_push_last_side == this_side &&
            wm.shoot_push_last_side != 0) {
            double ogx = ctx.opp_goal_x(), ad2 = ctx.attack_dir();
            double oy = 90.0 - (sp.aim_y - 90.0);
            double dx = (ogx + ad2 * 5.0) - bx, dy = oy - by;
            double len = std::hypot(dx, dy);
            if (len > 1e-6) { dx /= len; dy /= len; sp.aim_rot = angle_to(0.0, 0.0, dx, dy); }
            sp.dir_x = dx; sp.dir_y = dy;
            this_side = -this_side;
        }
        // —— 到点定向执行（docs/18 §8）——
        const double prep_d = shoot_prep_dist(wm);   // 罚点球 35cm（助跑更长 → 出球更快）
        double px = bx - sp.dir_x * prep_d;          // 准备点 = 球后 prep_d，落在瞄准线上
        double py = by - sp.dir_y * prep_d;
        // docs/06 第 49 轮：准备点也不许落在角落黄区（否则驱车过去会穿过球、
        //   把球往角心顶 → "No pushing" 犯规）。球在对方门角附近射门时最易触发。
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
            wm.coop_finish(CoopOutcome::OpponentFirst);   // 争抢态：不停车等人，照原方向推出去
        }
        double db = dist(r.x, r.y, bx, by);
        double te_head = angle_diff(sp.aim_rot, r.rot);
        // 人在球的"门侧后方"：球−人 在瞄准方向上的投影 > 0 ⇔ 往前推把球送向球门
        //   （旧口径只判"机头对着球"，人站在球前面时会**把球往回推**）
        bool behind = ((bx - r.x) * sp.dir_x + (by - r.y) * sp.dir_y) > 0.0;
        bool near = db < prep_d + 6.0;
        bool ready = false;
        // —— 罚点球专用执行（docs/06 第 57 轮，真机 09-13 rlg 帧 2292~2338）——
        //   摆位后 1 号站在球后 4cm，却「就地转正」18 帧（位置一动不动，观感=「完全不动」），
        //   再倒退 20cm 去准备点，最后才以 103cm/s 把球推出——1.15 秒全浪费，射正也被扑。
        //   改成：**沿瞄准线倒车到助跑点**（倒车方向就是助跑方向，机头朝向天然保持），
        //   到位且朝向够准 → 直接冲穿球（35cm 助跑，出球更快、门将来不及横移）。
        if (wm.in_penalty_exec) {
            // —— docs/06 第 66 轮（用户真机实测"罚球还是太慢"）：**点球一律不倒车** ——
            //   实测证据（16:01 场帧 3596~3628）：旧逻辑先退到球后 23cm（32 帧 = 0.8 秒），
            //   折返时机器人一直在 y≈93→113（球在 y=89.8，即偏上 3~24cm），最后从球侧上方
            //   掠过 → 球被推向底角、出球只有 0.7cm/帧 → 平台判"没开出"，点球重发 3 次。
            //   旧代码本来有"对手 <kPenaltyNoBackOpp 就不倒车"的规则，但那次对手在 34cm
            //   却仍然倒了车 ⇒ 不再依赖对手距离：**只要在球后就绝不后退**，只就地转正。
            if (behind && near) {
                if (std::fabs(te_head) <= kPrepAngTol) {
                    ready = true;                          // 朝向够准 → 立刻推穿
                } else if (wm.shoot_align_frames >= kShootAlignTimeout) {
                    ready = true;                          // 兜底：不再死等（宁可打偏也别重发）
                    wm.shoot_align_frames = 0;
                } else {
                    ++wm.shoot_align_frames;
                    motion::position_aligned(r, r.x, r.y, sp.aim_rot, kPrepPosTol, kPrepAngTol);
                    return;                                // 只原地转正
                }
            } else if (behind) {
                // 在球后但太远 → 沿瞄准线靠近（这是"走过去"，不是"倒车助跑"）
                wm.shoot_align_frames = 0;
                motion::position(r, px, py, motion::TM_PASS);
                return;
            } else {
                // 站错一侧 → 必须先绕到球后（这个绕行是必要的，不算助跑）
                wm.shoot_align_frames = 0;
                motion::position_aligned(r, px, py, sp.aim_rot, kPrepPosTol, kPrepAngTol);
                return;
            }
        } else if (kNoAlignWait > 0.5) {
            // 第 90 轮官方式：在球后就直接推穿（不管远近、不管朝向）；不在球后 → 赶去球后点（经过型，不对准）
            wm.shoot_align_frames = 0;
            if (behind) ready = true;
            else { motion::position(r, px, py, motion::TM_PASS); ready = false; }
        } else if (contested && behind && kContestPush > 0.5) {
            // 争抢态：已在球后就直接推穿，不原地转正、不绕准备点（转正的 1 秒里球已被抢走）
            wm.shoot_align_frames = 0;
            ready = true;
        } else if (behind && near) {
            // ①a 已在球后方且够得着 → **就地转正**（不后退、不丢球权）
            //   教训：先前要求"退到球后 20cm 准备点"才对准，插桩实测射门分支一场进
            //   3600 次、推球 0 次（球一直在动，那个点不可达）→ 射门函数被门禁卡死。
            if (std::fabs(te_head) <= kPrepAngTol) {
                ready = true;
            } else if (wm.shoot_align_frames >= kShootAlignTimeout) {
                ready = true;                  // 超时兜底：不再死等
                wm.shoot_align_frames = 0;
            } else {
                ++wm.shoot_align_frames;
                motion::position_aligned(r, r.x, r.y, sp.aim_rot, kPrepPosTol, kPrepAngTol);
                return;
            }
        } else {
            // ①b 站错一侧 / 太远 → 去准备点并对准
            wm.shoot_align_frames = 0;
            ready = motion::position_aligned(r, px, py, sp.aim_rot, kPrepPosTol, kPrepAngTol);
            if (ready && !near) ready = false;  // 到了准备点但仍够不着球 → 继续靠近
        }
        // —— 禁止"过冲后反向穿球"（docs/06 第 70 轮；真机实证：主攻冲过球再倒着撞，
        //    把球推回自家门：球从帧 896 起朝 +x（自家门）加速到 +5.6cm/帧）——
        //   判据：球在我推球方向的反面（side<0，留 3cm 余量）⇒ 我已经在球的**球门侧**。
        //   此时沿瞄准线回推 = 用身体把球往自家门顶 ⇒ 必须先绕弧线（球侧方 22cm）回到球正后方。
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
            // 直线推穿 = 经过型：不套制动包线，保持推球力度
            motion::position(r, bx + sp.dir_x * 20.0, by + sp.dir_y * 20.0, motion::TM_PASS);
            if (coop_pass && !wm.coop_pass_task.observing_push && db < kPassReceiveDistance) {
                auto &task = wm.coop_pass_task; task.observing_push = true;
                task.push_ball_x = bx; task.push_ball_y = by; task.push_dir_x = sp.dir_x; task.push_dir_y = sp.dir_y;
            }
            // 计次：球已被推动(>5cm/帧)才记一次；cd=20 冷却防同一推多帧重复计
            if (!coop_pass && wm.shoot_push_cd <= 0 && std::hypot(wm.ball.vx, wm.ball.vy) > 5.0) {
                ++wm.shoot_push_count;
                wm.shoot_push_cd = 20;
                wm.shoot_push_last_side = this_side;
            }
        }
        return;
    }

    // 普通传球：plan_pass 的目标是**队友的接球点**（第 81/82 轮试过"球不在脚下不去接球点"与
    //   "穿过球推向接球点"，sim 300 局均净胜下降，保留原行为）。
    double db = dist(r.x, r.y, wm.ball.x, wm.ball.y);
    // 找离球最近的对方防守者（含守门员）：决定带球开口侧 + 判断球权是否在我
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
        // —— 围困检测（治"带球有进无退"）——
        // 球周围 25cm 内 ≥2 个 demo，或最近 demo <12cm → 不硬带：
        //   优先回传/横传（此时 assist 已在禁区外沿近端接应），无传球选择则带球离场。
        int swarm = 0, near_i = -1;
        double opp_near = 1e9;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            double d = dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y);
            if (d < 25.0) swarm++;
            if (d < opp_near) { opp_near = d; near_i = i; }
        }
        if (swarm >= 2 || opp_near < 12.0) {
            PassPlan pp2 = plan_pass(wm, id);
            // 围困时先解围；不发布普通接球任务，避免接球流程把脱困动作拖住。
            if (pp2.viable) { motion::position(r, pp2.target_x, pp2.target_y); return; }
            // 无传球选择：把球带离最近的围困者（向空档方向推，直线推穿不逗留）
            if (near_i >= 0 && opp_near > 1e-6) {
                double dx = wm.ball.x - wm.opp[near_i].x;
                double dy = wm.ball.y - wm.opp[near_i].y;
                double len = std::hypot(dx, dy);
                if (len > 1e-6) { dx /= len; dy /= len; }
                double te_e = angle_diff(angle_to(r.x, r.y, wm.ball.x, wm.ball.y), r.rot);
                if (db < 12.0 && std::fabs(te_e) < 40.0) {
                    motion::position(r, wm.ball.x + dx * 25.0, wm.ball.y + dy * 25.0, motion::TM_PASS);   // 带离围困=穿球经过型
                } else {
                    motion::position(r, wm.ball.x - dx * 20.0, wm.ball.y - dy * 20.0);
                }
                return;
            }
        }
        // —— 正常带球推进：球在我方脚下控制范围（我方是离球最近的人）——
        // 目标方向指向门柱开口（开口选「离最近防守者远」的一侧，避免直线撞进防守怀里）。
        // 执行方式与射门同款"直线推穿"：已贴在球后(≤12cm)且朝向对准(≤40°)时
        //   目标=球前 20cm 直线穿过球——sim 的 carry 在穿过瞬间生效，不会像原来
        //   停在球后 8cm 推球点那样死锁（死锁 → 球静止 → 判僵局 FreeBall 重置，
        //   sim 实测 158 次/场的主因之一）；还远/在侧面时先绕到球后 20cm 站位点对准。
        double ogx = ctx.opp_goal_x();
        double aim_y = (opp_y > 90.0) ? 106.0 : 74.0;   // 防守者偏下 → 带上柱口
        double dirx = (ogx + ctx.attack_dir() * 5.0) - wm.ball.x;
        double diry = aim_y - wm.ball.y;
        double len = std::hypot(dirx, diry);
        if (len > 1e-6) { dirx /= len; diry /= len; }
        double aim_rot2 = angle_to(0.0, 0.0, dirx, diry);
        double te_head2 = angle_diff(aim_rot2, r.rot);
        // 人在球的"推进侧后方"：球−人 在推进方向上的投影 > 0（否则往前推是把球往回推）
        bool behind2 = ((wm.ball.x - r.x) * dirx + (wm.ball.y - r.y) * diry) > 0.0;
        // 对准纪律同样适用于带球推进（docs/18 §8）：推球方向 = 撞球瞬间机头方向，
        //   旧口径只判"机头对着球"(|te_d|<40°)——不管推往哪、站错侧还会往回推。
        //   带球推进是推球次数最多的路径（sim 一场 183 次 vs 射门分支 45 次）。
        if (kNoAlignWait > 0.5 && behind2) {
            // 第 90 轮官方式：在推进侧后方就直接穿球推，不等贴近、不等对准
            wm.shoot_align_frames = 0;
            motion::position(r, wm.ball.x + dirx * 20.0, wm.ball.y + diry * 20.0, motion::TM_PASS);
        } else if (db < 12.0 && behind2 && std::fabs(te_head2) <= kDribAngTol) {
            motion::position(r, wm.ball.x + dirx * 20.0, wm.ball.y + diry * 20.0, motion::TM_PASS);   // 带球推进=穿球经过型
        } else if (!prep_point_ok(wm.ball.x - dirx * 20.0, wm.ball.y - diry * 20.0)) {
            // docs/06 第 49 轮：球后站位点落在角落黄区 → 不绕球后（会穿过球把球顶进角里）
            hold_out_of_corner(wm, r);
        } else if (db < 16.0 && behind2 && wm.shoot_align_frames < kShootAlignTimeout) {
            // 贴在球后但没对准 → 就地转正（不后退、不丢球权）；超时则退到球后站位点重来
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
        // 球不在脚下 / 争抢中：追预测球位（带减速防冲过头）
        // 对方罚球区防线（docs/13 攻击强化补充）：预测位在对方罚球区内
        //   （对手射门被挡/门将扑救后球滞留门区）→ 追到罚球区外沿等球弹出，
        //   不冲进门区——sim 实测 ACTIVE 追球穿门区 + MID 站位 = 2+ 人违规
        //   16 帧/场（debug f9109：球已弹到 x=59 球速 22，两人还滞留门区 13 帧）。
        BallState chased = chase_target(wm);
        // 门前 100cm 锥形区（dist_opp_goal<100 且 |y-90|<45）：直线追球路径必穿门区
        //   （sim debug：球射偏出门线 y=41，ACTIVE 追球穿门区 16 帧 = 2+ 人违规主因），
        //   追到门区外沿等球弹出；射门分支优先不受影响。
        if (wm.ctx.dist_opp_goal(chased.x) < 100.0 && std::fabs(chased.y - 90.0) < 45.0) {
            // 补射跟进：球在门前低速且我方就近 → 豁免 clamp、追进球区补射
            //   （球被 GK 扑出/防守者挡回的反弹是第二落点机会，错过了就白射；
            //   球高速滚向门 / 距球太远时不豁免——追不过去、也不值得长途进场）。
            double bspeed = std::hypot(wm.ball.vx, wm.ball.vy);
            bool rebound_rush = (bspeed < kReboundRushSpeed) &&
                                dist(r.x, r.y, chased.x, chased.y) < kReboundRushDist;
            if (!rebound_rush) {
                chased.x = wm.ctx.opp_goal_x() - wm.ctx.attack_dir() * 85.0;
                chased.y = clamp(chased.y, 72.5, 107.5);
            }
        }
        // docs/15 P0-4：追球避障——直线被对方挡才绕行（plan_route 2r 邻域裁剪
        //   后开销约全图 1/10）；decel=true 保留接近减速防冲过头。
        move_avoiding(wm, r, id, chased.x, chased.y, true);
    }
}

// 人盯人执行体（第 97 轮从 run_passive 里提出来的**纯搬家**，逐行行为不变）：
//   把「盯住对手 t」这一整套动作（对方门区不追 / 贴身逼抢 / 堵传球线 / 门侧站位 / 禁区纪律）
//   做成可复用函数——这样 PASSIVE 和「被匈牙利指派的 ASSIST/MIDFIELD」走**同一套**执行逻辑，
//   不会出现两套盯人写法各调各的（那种分叉正是第 97 轮要消灭的「各自贪心」）。
//   t < 0（没被指派）⇒ 什么都不做，调用方继续走后面的分支。
static void run_mark_body(WorldModel &wm, int id, int t) {
    if (t < 0 || t >= PLAYERS_PER_SIDE) return;
    // 对方门区聚集犯规防护（C 模块方案，余量并入 A/B 15cm 调参）：
    //   被盯者缩在对方门区且球不在门区时不追进去（门区 2+ 人 / 停留>20帧 → 罚点球），
    //   改站对方门区前缘外 15cm 等反击。
    if (in_opp_goal_area(wm.ctx, wm.opp[t].x, wm.opp[t].y) &&
        !in_opp_goal_area(wm.ctx, wm.ball.x, wm.ball.y)) {
        double fx = 0.0, fy = 0.0;
        opp_goal_area_front(wm.ctx, wm.ball.y, fx, fy);
        motion::position(wm.home[id], fx, fy);
        return;
    }
    // E2 盯人转逼抢（docs/13 攻击强化）：被盯者离球 <25cm（控球/即将接球）时
    //   放弃站连线、直接冲球贴身逼抢（抢下或破坏对手组织，配合 E1 双人夹抢）。
    double d_opp_ball = dist(wm.ball.x, wm.ball.y, wm.opp[t].x, wm.opp[t].y);
    // 球在我方门区内不逼抢：门前交给门将（否则与门将 2+ 人违规，
    //   sim 实测 E2 首版 2+人 0.5→2.3 次/场），站门区外等解围。
    if (d_opp_ball < 25.0 && !in_goal_area(wm.ctx, wm.ball.x, wm.ball.y)) {
        // 逼抢护栏（docs 第 73 轮 · 问题1 · B）：只从球门侧贴球。后卫从门侧追球，
        //   碰球只会把球顶向场内（远离己门）；从场侧追则把球顶向己门 = 乌龙
        //   （历史 3:8 已回滚）。球在门区外本已 >50cm 离门线，此护栏再堵死最后
        //   一条乌龙通道——从场侧时不追球、落到下面 mark 站位封线。
        double d_home_goal = wm.ctx.dist_our_goal(wm.home[id].x);
        double d_ball_goal = wm.ctx.dist_our_goal(wm.ball.x);
        if (d_home_goal < d_ball_goal) {   // 门将侧（离门更近）→ 追球把球顶离门
            motion::chase_ball(wm.home[id], chase_target(wm));
            return;
        }
    }
    // 站位：速度前馈预测被盯者未来位置（改「追着跑」为「截击」，
    //   同速追逐追不上移动目标），站到「被盯者→球门」连线上、离其 mark_dist 处，
    //   封住他的传/射路线。
    double gx = wm.ctx.our_goal_x(), gy = 90.0;
    double px = wm.opp[t].x + wm.opp_vx[t] * mark_lead();
    double py = wm.opp[t].y + wm.opp_vy[t] * mark_lead();
    // 站位方向选择：
    //   · 被盯者是接球者（非持球者）且离球够近 → 传球随时发生，
    //     站「球→被盯者」连线上（堵传球线，提前掐断传球）；
    //   · 否则（持球者 / 离球远）→ 站「被盯者→球门」连线上（封射门/再传）。
    double d_ball = dist(wm.ball.x, wm.ball.y, wm.opp[t].x, wm.opp[t].y);
    double ref_x = gx, ref_y = gy;                       // 默认 goal-side
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
    // 对方门区禁入（docs/13 方案 A）：盯人站位不得进入对方门区（防 2+ 人违规判点球）
    clamp_out_opp_goal_area(wm.ctx, mx, my);
    motion::position(wm.home[id], mx, my);
    return;
}

void run_passive(WorldModel &wm, int id) {
    if (run_pass_receiver(wm, id)) return;
    // —— 门前协防（docs/03 第14轮，参考官方 demo CenterDefender 球-门连线思想，自研实现）——
    // 根因（真 vs demo 2:7×2、0:6 复盘）：demo 把球控停我方门前 (205,90) 静止 1.5~2.2s，
    //   其追击手 Y5 从 100cm 外高速直冲抢点推射；原防守 double_team 球进罚球区
    //   return false（禁区纪律）、盯人站位被罚球区纪律推出 → 门前只剩门将 1v1。
    // 对策：球在我方门前 80cm 内、近静止、且对方已逼近(<100cm，正来抢/控这颗球)时，
    //   本角色钉到「球-门连线」护门点（球远站球后45、球近站球前8cm 堵推射线，见
    //   defense.cpp goal_cover_point）——正好卡在对方从弧顶/中场直冲门前的路径上，
    //   与门将形成双人包夹。对方离球远（我方门球/清球场景）不触发，不拉离防区。
    //   阈值 60→100（2026-08-31 第15轮）：rlg 复盘 B5 从后场到门前需 40+ 帧，
    //   Y5 从 60cm 冲球仅 ~24 帧——60cm 触发永远晚到；100cm 提前出发才有拦截余量。
    if (wm.ctx.dist_our_goal(wm.ball.x) < 80.0 &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 3.0) {
        double opp_dmin = opp_clear_dist(wm);
        if (opp_dmin < 100.0) {
            // —— 门前清道夫：**已在球门侧**时就把球真正清出去（第 74 轮，2026-09-23 真机复盘）——
            //   真机证据（09-23 两局 + 18 局汇总）：球停在自家门前 3~4cm 达 113 帧（2.8s）无人
            //   能清；26 个丢球里 23 个（88%）是"我方最后触球"；5 个丢球中 4 个门将都站在
            //   "球与门之间"的场侧 8~10cm。根因是**三条"防乌龙"规则叠加出门口无人区**：
            //     ① 球在门区不逼抢（交给门将）；② 只从球门侧贴球（场侧不追）；
            //     ③ 球贴门线时本角色"停轮站定、绝不碰球"（下方第 37 轮规则）。
            //   三条各自都对，合起来 = 谁都不碰球 → 球自己慢慢滚进门。
            //   这里只开一条**双向安全**的通道：仅当本角色比球更靠己门（已在球门侧）时才出手，
            //   朝球推过去（TM_PASS 不刹车）——从门侧推，球只会被顶向场内（-x），不可能乌龙；
            //   球在场侧时一律保持原护栏（绝不直撞，交给让位/站位封线兜底）。
            double dbp = dist(wm.ball.x, wm.ball.y, wm.home[id].x, wm.home[id].y);
            if (wm.ctx.dist_our_goal(wm.ball.x) < 25.0 &&
                !in_goal_area_rule(wm.ctx, wm.ball.x, wm.ball.y, 5.0, 5.0) &&   // 球在裁判门区：交门将（进去即计点球）
                wm.ctx.dist_our_goal(wm.home[id].x) < wm.ctx.dist_our_goal(wm.ball.x)) {
                motion::chase_ball(wm.home[id], chase_target(wm));   // 门侧推球：只会推离己门
                return;
            }
            // —— 门线球站定防乌龙（docs/06 第37轮）——
            // 真机 2 乌龙铁证（9/7 23:40 场 @124.8 门角 y35、@138.6 门线中央）：
            //   B4 护门点站位每帧随威胁 position 微调，而 B4 贴球(6-9cm) → 移动
            //   时把球拖着走（球 y110→101 随 B4 y106→96 = 拖进门内）。
            // 对策：球贴我方门线(<15cm)且本角色贴球(<14cm) → 停轮站定，用身体
            //   封 demo 推射角，绝不碰球不拖动（demo 推球用身体挡，不主动清）。
            //   ⚠️ 此规则只在**场侧**生效（球门侧已被上面清道夫接管，见第 74 轮）。
            if (wm.ctx.dist_our_goal(wm.ball.x) < 15.0 && dbp < 14.0) {
                TRACE_MARK(wm.home[id]);
                wm.home[id].vl = 0.0;
                wm.home[id].vr = 0.0;
                return;
            }
            double cx = 0.0, cy = 0.0;
            goal_cover_point(wm, cx, cy);
            motion::position(wm.home[id], cx, cy);
            return;
        }
    }
    // 人盯人（第 97 轮）：目标来自**带权匈牙利一一匹配**（defense::assign_marks 每帧写入
    //   wm.mark_assign）——从数学上杜绝「两个人盯同一个对手」，且换人惩罚 λ 直接进代价矩阵。
    //   匈牙利没生效时（威胁门槛没过 / defense.kMarkHungarian=0）回退旧的单目标贪心。
    //   执行体统一在 run_mark_body（PASSIVE 与被指派的 ASSIST/MIDFIELD 共用）。
    if (wm.threat_level >= 0.6) {
        int t = wm.mark_assign_valid ? wm.mark_assign[id]
                                  : pick_mark_target(wm, wm.mark_target);
        wm.mark_target = t;
        run_mark_body(wm, id, t);
    }
    // 争抢上抢（第 81 轮）：威胁分 < 0.6 时原本回防区点站着；sim 分支追踪显示这是全队
    //   最大的犹豫源——对手贴球、PASSIVE 是离球最近的己方场上队员，却在防区点上等。
    //   只在球门侧、球不在罚球区时出手（保留第 73 轮"只从门侧贴球"防乌龙护栏）。
    //   限离己门 kPassivePressDepth 内：放到全场时 sim 对方门区 2+ 人 761→1176 帧/场、
    //   单人滞留 23→30 次/场（真机罚点球源）。
    if (kPassivePress > 0.5 && !in_penalty_area(wm.ctx, wm.ball.x, wm.ball.y) &&
        wm.ctx.dist_our_goal(wm.ball.x) < kPassivePressDepth) {
        const RobotState &me = wm.home[id];
        double my_d = dist(me.x, me.y, wm.ball.x, wm.ball.y), opp_d = 1e9, mate_d = 1e9;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            opp_d = std::min(opp_d, dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y));
            if (i != id && wm.role[i] != ROLE_GOALIE)
                mate_d = std::min(mate_d, dist(wm.home[i].x, wm.home[i].y, wm.ball.x, wm.ball.y));
        }
        if (opp_d < kContestOppDist && my_d < kContestReach && my_d < mate_d &&
            wm.ctx.dist_our_goal(me.x) < wm.ctx.dist_our_goal(wm.ball.x)) {
            motion::chase_ball(wm.home[id], chase_target(wm));
            return;
        }
    }
    DefensePlan dp = plan_defense(wm, id);
    // 到位迎球（第 103 轮）：断球点在球来路上时，走位到点后原地转正、机头朝球来的方向站定。
    //   不这么做（旧行为）：机头朝的是"我自己的来路"，球从侧面撞上来只被横着顶一下，
    //   动量没抵消 → 球继续朝自家门滚，甚至被顶到更危险的角度。
    if (dp.face_incoming && kDefFaceIncoming > 0.5)
        motion::arrive_facing(wm.home[id], dp.target_x, dp.target_y, dp.aim_rot,
                              kDefArriveDist, kDefFaceAngTol);
    else
        motion::position(wm.home[id], dp.target_x, dp.target_y);
}

// ============================================================
// 分道压迫进攻（docs/06 第 79 轮，用户真机指令："采用官方的进攻思路 + 借墙射门"）
// ------------------------------------------------------------
// 官方 demo 能赢我们靠的是"人多往球上压"：球到哪一侧，那一侧就有人去拱球，另一侧的人
//   跟在后面等二点。这里按我们自己的框架重写成三件事：
//   ① 分道：ASSIST 管上半道（y>90）、MIDFIELD 管下半道；中路 ±kLaneShare 两人共管。
//   ② 从球后拱：推进方向 = 射门方案（含借墙）→ 借墙推进方案 → 门心，依次取第一个可用的；
//      人在球前面时先横绕到球侧、再落到球后——绝不从球前面往回顶（乌龙的主要来源）。
//   ③ 让位护送：已有队友在球后顶住球时，不去挤同一个球，站它侧后方接二点。
// 纪律：不进对方门区（2+ 人判点球）；角区/死球期不推；球离我方门太近交回原防守逻辑。
// 回滚：kSwarmEnabled 置 false 即恢复第 78 轮行为。
// ============================================================
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

// 第 89 轮：去目标的**路线**也不许穿对方门区（原先只夹目标点）。
//   真机 09-29 黑匣子：弱侧跟进目标在门区侧后方（如 (27,48)），ASSIST/MID 从门前斜插过去，
//   整条线穿过门区、被对方门将/后卫卡在里面 40~94 帧。
//   人在（外扩）门区内 → 先沿 x 直线退出到前沿外；路线穿门区 → 先去门区前沿角点
//   （人在门区侧面时取自己这侧的角，否则取目标那侧的角），逐帧重算。
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

// 推进方向（单位向量）：射门方案（plan_shoot 已含直线/借墙择优）→ 借墙推进 → 门心
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

// 队员 i 是否已在球后顶住球（沿推进方向在球后 18cm 内、横向偏差 <9cm）
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

// lane：+1 = 上半道（y>90），-1 = 下半道。返回 false → 本帧不压迫，调用方走原逻辑。
bool run_swarm(WorldModel &wm, int id, double lane) {
    if (!kSwarmEnabled) return false;
    const TeamContext &ctx = wm.ctx;
    if (!wm.live_play || wm.in_penalty_exec) return false;   // 第 88 轮：认活球（真机 gameState 不回 PlayOn）
    if (wm.coop_pass_task.active &&
        (id == wm.coop_pass_task.passer_id || id == wm.coop_pass_task.receiver_id)) return false;
    if (wm.coop_ball_control.active && id == wm.coop_ball_control.receiver_id) return false;
    if (id == wm.sweeper_id || !push_allowed(wm)) return false;
    const double bx = wm.ball.x, by = wm.ball.y, ad = ctx.attack_dir();
    if (ctx.dist_our_goal(bx) < kSwarmOwnGuard) return false;

    // 球在对方门区附近：门口交给 ACTIVE，自己在门区外沿本道等二点
    if (near_opp_box(ctx, bx, by)) {
        swarm_move(wm, id, ctx.opp_goal_x() - ad * (58.0 + kOppBoxMargin), 90.0 + lane * 32.0);  TRACE_MARK(wm.home[id]);
        return true;
    }
    // 球不在本道：弱侧跟进（落后球一段、贴本道）
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

    // 让位护送：别的队友已顶住球 → 站它侧后方（本人也顶住时照常推）
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
        // 对准了：沿推进方向推穿
        swarm_move(wm, id, bx + ux * kHerdThrough, by + uy * kHerdThrough);  TRACE_MARK(wm.home[id]);
    } else if (behind > -3.0) {
        // 在球侧后方：落到球后（偏得越多退得越远，免得斜插时蹭到球）
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
    // 威胁高：回防但分散站位（封上侧射门线，不与 passive 挤一点）
    // 反击快攻窗口（docs/13 方案 A）：断球后 counter_attack_frames>0 时豁免回防、立即前插接应，
    //   让 ACTIVE 断球有传球选择（治反击前场真空，真实 9/2 vs demo 0 射门威胁）。
    if ((wm.threat_level > 0.3 || wm.no_possession_frames >= 2) && wm.counter_attack_frames <= 0) {
        // E1 双人逼抢（docs/13 攻击强化）：球在对方半场且对手控球时，本角色放弃
        //   防守站位压上追球（与 ACTIVE 双人夹抢，抢下即反击，治"1 人追球断球率低"）。
        //   球进对方罚球区不追（禁区纪律，非门将不进对方禁区）；离球 >130cm 不追（防失位）。
        double bbx = wm.ball.x;
        bool ball_opp_half = (wm.ctx.attack_dir() > 0) ? (bbx > 110.0) : (bbx < 110.0);
        // 球在门前 100cm 锥形区不追（直线追球路径穿门区 = 2+ 人违规，同 run_active）
        if (ball_opp_half && !wm.we_have_ball &&
            !(wm.ctx.dist_opp_goal(bbx) < 100.0 && std::fabs(wm.ball.y - 90.0) < 45.0) &&
            dist(wm.home[id].x, wm.home[id].y, bbx, wm.ball.y) < 130.0) {
            // 追球目标同样夹在门前锥形区外（预测位可能滚进门区，直接追会穿门区）
            double pxx = wm.ball_pred.x, pyy = wm.ball_pred.y;
            if (wm.ctx.dist_opp_goal(pxx) < 100.0 && std::fabs(pyy - 90.0) < 45.0) {
                pxx = wm.ctx.opp_goal_x() - wm.ctx.attack_dir() * 85.0;
                pyy = clamp(pyy, 72.5, 107.5);
            }
            motion::position(wm.home[id], pxx, pyy, motion::TM_PASS);   // E1 压上追球=经过型
            return;
        }

        // 清道夫(远侧覆盖)：球在防守三区拉边时，本角色被 strategy.cpp 指派为清道夫，
        //   钉中路封远门柱/横传（触发与站位见 update_sweeper）。
        if (id == wm.sweeper_id) {
            motion::position(wm.home[id], wm.sweeper_x, wm.sweeper_y);
            return;
        }
        // 对方射门在门框内且够快 → 抢上侧反弹位（防补射）
        if (shot_on_target(wm) && ball_danger_speed(wm) > rebound_min_danger()) {
            double rx = 0.0, ry = 0.0;
            rebound_point(wm, +30.0, rx, ry);
            motion::position(wm.home[id], rx, ry);
            return;
        }
        // 盯人（第 97 轮）：被带权匈牙利分配到某个对手 → 走与 PASSIVE 同一套执行体。
        //   位置：排在"抢反弹位"之后（门前救险优先）、"双人夹抢"之前
        //   （既然分配已决定"这个对手归我"，就不必再去算夹抢点，避免两台车又扑同一个人）。
        //   没被分配（mark_assign=-1 / 匈牙利没生效）⇒ 什么都不做，继续走原逻辑。
        if (wm.mark_assign_valid) {
            run_mark_body(wm, id, wm.mark_assign[id]);
            if (wm.mark_assign[id] >= 0) return;
        }
        // 二抢一：持球者带球推进到门前危险区时，补一个防守者上前与 passive 包夹
        double dtx = 0.0, dty = 0.0;
        if (double_team_point(wm, id, dtx, dty)) {
            motion::position(wm.home[id], dtx, dty);
            return;
        }
        DefensePlan dp = plan_defense(wm, id);
        double ty = clamp(dp.target_y + 30.0, 20.0, 160.0);
        if (dp.face_incoming && kDefFaceIncoming > 0.5)   // 第 103 轮：到位迎球
            motion::arrive_facing(wm.home[id], dp.target_x, ty, dp.aim_rot,
                                  kDefArriveDist, kDefFaceAngTol);
        else
            motion::position(wm.home[id], dp.target_x, ty);
        return;
    }
    // —— 进攻分支：站 A 的助攻点；先躲敌人，再和队友 Y 轴互相推开 ——
    constexpr double THREAT_RADIUS = 30.0;   // 敌方威胁检测半径 cm
    constexpr double MAX_OFFSET     = 20.0;   // 最大允许横向偏移 cm
    constexpr double FIELD_MARGIN   = 6.0;    // 目标点离边线最小距离 cm
    constexpr double TEAM_SPACING   = 25.0;   // 与中场的最小 Y 间距（防挤堆）
    double tx = wm.assist_x;                  // X 保持 A 原始输出
    double ty = spread_y(wm, wm.assist_x, wm.assist_y, THREAT_RADIUS, MAX_OFFSET);   // ① 躲敌人（优先级高）
    // ② 队友推开：离中场站位点 Y 太近时，朝远离方向错开，X 不动
    if (fabs(ty - wm.mid_y) < TEAM_SPACING) {
        ty = wm.mid_y + ((ty >= wm.mid_y) ? TEAM_SPACING : -TEAM_SPACING);
    }
    tx = clamp(tx, FIELD_MARGIN, TeamContext::FIELD_LENGTH - FIELD_MARGIN);
    ty = clamp(ty, FIELD_MARGIN, TeamContext::FIELD_WIDTH  - FIELD_MARGIN);
    // 对方门区禁入（docs/13 方案 A）：助攻站位不得进入对方门区（防 2+ 人违规判点球）
    clamp_out_opp_goal_area(wm.ctx, tx, ty);
    motion::position(wm.home[id], tx, ty);
}

void run_midfield(WorldModel &wm, int id) {
    if (run_pass_receiver(wm, id)) return;
    if (run_swarm(wm, id, -1.0)) return;
    // 威胁高：回防但分散站位（封下侧射门线）
    // 反击快攻窗口（docs/13 方案 A）：断球后 counter_attack_frames>0 时豁免回防、立即前插接应，
    //   让 ACTIVE 断球有传球选择（治反击前场真空，真实 9/2 vs demo 0 射门威胁）。
    if ((wm.threat_level > 0.3 || wm.no_possession_frames >= 2) && wm.counter_attack_frames <= 0) {
        // 清道夫(远侧覆盖)：球在防守三区拉边时，本角色被 strategy.cpp 指派为清道夫，
        //   钉中路封远门柱/横传（触发与站位见 update_sweeper）。
        if (id == wm.sweeper_id) {
            motion::position(wm.home[id], wm.sweeper_x, wm.sweeper_y);
            return;
        }
        // 对方射门在门框内且够快 → 抢下侧反弹位（防补射）
        if (shot_on_target(wm) && ball_danger_speed(wm) > rebound_min_danger()) {
            double rx = 0.0, ry = 0.0;
            rebound_point(wm, -30.0, rx, ry);
            motion::position(wm.home[id], rx, ry);
            return;
        }
        // 盯人（第 97 轮）：被带权匈牙利分配到某个对手 → 走与 PASSIVE 同一套执行体。
        //   位置：排在"抢反弹位"之后（门前救险优先）、"双人夹抢"之前
        //   （既然分配已决定"这个对手归我"，就不必再去算夹抢点，避免两台车又扑同一个人）。
        //   没被分配（mark_assign=-1 / 匈牙利没生效）⇒ 什么都不做，继续走原逻辑。
        if (wm.mark_assign_valid) {
            run_mark_body(wm, id, wm.mark_assign[id]);
            if (wm.mark_assign[id] >= 0) return;
        }
        // 二抢一：持球者带球推进到门前危险区时，补一个防守者上前与 passive 包夹
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
        if (dp.face_incoming && kDefFaceIncoming > 0.5)   // 第 103 轮：到位迎球
            motion::arrive_facing(wm.home[id], dp.target_x, ty, dp.aim_rot,
                                  kDefArriveDist, kDefFaceAngTol);
        else
            motion::position(wm.home[id], dp.target_x, ty);
        return;
    }
    // —— 进攻分支：站 A 的中场点；先躲敌人，再和队友 Y 轴互相推开 ——
    constexpr double THREAT_RADIUS = 30.0;   // 敌方威胁检测半径 cm
    constexpr double MAX_OFFSET     = 20.0;   // 最大允许横向偏移 cm
    constexpr double FIELD_MARGIN   = 6.0;    // 目标点离边线最小距离 cm
    constexpr double TEAM_SPACING   = 25.0;   // 与助攻的最小 Y 间距（防挤堆）
    double tx = wm.mid_x;                     // X 保持 A 原始输出
    double ty = spread_y(wm, wm.mid_x, wm.mid_y, THREAT_RADIUS, MAX_OFFSET);   // ① 躲敌人（优先级高）
    // ② 队友推开：离助攻站位点 Y 太近时，朝远离方向错开，X 不动
    if (fabs(ty - wm.assist_y) < TEAM_SPACING) {
        ty = wm.assist_y + ((ty >= wm.assist_y) ? TEAM_SPACING : -TEAM_SPACING);
    }
    tx = clamp(tx, FIELD_MARGIN, TeamContext::FIELD_LENGTH - FIELD_MARGIN);
    ty = clamp(ty, FIELD_MARGIN, TeamContext::FIELD_WIDTH  - FIELD_MARGIN);
    // 对方门区禁入（docs/13 方案 A）：中场站位不得进入对方门区（防 2+ 人违规判点球）
    clamp_out_opp_goal_area(wm.ctx, tx, ty);
    motion::position(wm.home[id], tx, ty);
}

// ============================================================
// 前场散球逼抢（docs/06 第 83 轮）：strategy.cpp 选出的逼抢者跑过去抢散球。
//   复用 run_active 追球段三件套：push 守卫 → chase_target 目标 → 对方门区锥形区纪律
//   → move_avoiding 避障追球(带减速)。触发条件（前场+散球）由 update_presser 把关。
// ============================================================
void run_press(WorldModel &wm, int id) {
    RobotState &r = wm.home[id];
    if (!push_allowed(wm)) { hold_out_of_corner(wm, r); return; }
    BallState chased = chase_target(wm);
    // 对方门区锥形区纪律（同 run_active 追球段）：球在对方门前 100cm 锥形区
    //   （dist_opp_goal<100 且 |y-90|<45）→ 非门将不进门区，夹到门区外沿等球弹出。
    if (wm.ctx.dist_opp_goal(chased.x) < 100.0 && std::fabs(chased.y - 90.0) < 45.0) {
        chased.x = wm.ctx.opp_goal_x() - wm.ctx.attack_dir() * 85.0;
        chased.y = clamp(chased.y, 72.5, 107.5);
    }
    if (wm.role[id] != ROLE_ACTIVE)   // 第 89 轮：非主攻逼抢者的路线不穿对方门区
        opp_box_detour(wm.ctx, r.x, r.y, chased.x, chased.y);
    move_avoiding(wm, r, id, chased.x, chased.y, true);
}


// ============================================================
// 第 91 轮：官方式分区（用户："球到哪就追到哪，不需要另一套防守策略来回切"）
//   官方 demo 没有防守模式：4 个场上球员各管一片，球进自己的片就直冲球，不在就站随球平移的点。
//   进攻=防守=同一个动作 → 没有模式切换、没有犹豫。本函数取代 PASSIVE/ASSIST/MIDFIELD 的全部分支
//   （回防/清道夫/二抢一/护门点/反弹位/助攻点/逼抢者）；ACTIVE 仍走 run_active（= 官方中锋，永远追球）。
//   坐标：fx = 距己方门线，y 绝对值（上下两翼对称，无需镜像）。
//   保留：接球任务、对方门区纪律（夹 85cm 线 + 绕门区）、角区不推球、己方门前只准往外冲（防乌龙）。
//   回滚：strategy.kZoneMode=0。
// ============================================================

// 己方禁区纪律（裁判 Judge_PENALTY_KICK：球在本方 80cm 内时，GK 外 小禁区 >1 人 / 1 人满 20 帧、
//   或 小+大禁区 >3 人 / 3 人满 20 帧 ⇒ 判对方点球）。官方分区站位 (20,120)/(25,by) 正落在大禁区，
//   magic_rob 未加此纪律时 9.1 个点球/局。规则：分区球员一律不进小禁区；大禁区只放离球最近的一个分区球员
//   ⇒ 禁区内至多 主攻+1 = 2 人。
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
        // 已在大禁区（含 10cm 余量）→ 先沿 x 直线退出，不横穿禁区去新站位点
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

    // —— 分区：按官方 LeftWing / RightWing / CenterDefender 原样 ——
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

    // —— 追球：目标就是球，穿球不减速；只加两条官方没有的守卫 ——
    if (in_no_push_zone(wm.ball.x, wm.ball.y)) { hold_out_of_corner(wm, r); return; }
    BallState c = chase_target(wm);
    // 对方门前锥形区：非主攻不进，夹到门区外沿等球弹出（同 run_press）
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
