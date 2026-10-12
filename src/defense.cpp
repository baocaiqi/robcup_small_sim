// defense.cpp — 区域防守：沿球运动轨迹求断球点（原点左下角 cm，蓝队守 x=220）
#include "simuro5/defense.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/role_assignment.hpp"
#include "simuro5/hungarian.hpp"
#define TUNABLE_PREFIX "defense."
#include "simuro5/tunable.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {
// 犀利进攻防守总开关：1=防守收紧（更早回防、盯人更贴）；sharp_defense_on 供跨 TU 读取
TUNABLE(kSharpDefense, 1.0);

bool sharp_defense_on() { return kSharpDefense > 0.5; }

// 断球线距门线 cm / 球速阈值 cm/帧(低于即停球 → 回退静态点) / 断球点夹取范围 cm / 我方速度 cm/帧 / 可达余量倍数
TUNABLE(kInterceptLineDist, 48.5364);

TUNABLE(kMinBallSpeed, 1.35924);

TUNABLE(kMinX, 12.0);
TUNABLE(kMaxX, 208.0);
TUNABLE(kMinY, 15.0);
TUNABLE(kMaxY, 165.0);

TUNABLE(kMySpeed, 2.18624);
TUNABLE(kReachMargin, 1.04068);

// 主动截球：开关 / 采样步长 10cm、最远 150cm / 截点限我方门前 140cm / 对手贴球 20cm 算带球
// ⚠️ **2026-10-11 真机 A/B：默认从 1.0 改回 0.0**。
//   第 70 轮把它打开（0→1）后，真机 13 场（含 70+71+72 三轮一起上）实测：
//   失球 2.07→1.00/场（t=2.49，好），**但进球 1.53→0.85/场（t=1.88，坏）**，净胜只从 −0.54 到 −0.15（不显著）。
//   三轮捆在一起分不清谁的功劳/责任，而"提前拦截"的机制（后卫离开自己片区扑球轨迹）最可能吃掉进攻，
//   所以这一版**只把它单独关掉**做单变量 A/B：其它两轮（71 门将收缩、72 重启角色表）保持不动。
//   若 A/B 证明进球回来且失球没回去 ⇒ 保留 0；否则改回 1.0。
TUNABLE(kEarlyEnable, 0.0);
// ⚠️ 默认由 0 改为 1（2026-10-10）。第 100 轮实现时实测「0 次触发」而默认关，
//   原因是它只挂在 `plan_defense` 的 `else if` 上 —— 只有 `intercept_point` **失败**时才轮到它；
//   而"球朝门滚"时 `intercept_point` 几乎总会成功，所以这条分支走不到（docs/06:4455 当时就存疑）。
//   真机取证（8 场 20 个失球）：6~7 球是球从 70~190cm 外一路滚进来、越线前 0.5 秒两边都没人碰它；
//   45% 的失球我方没有任何非门将在球 30cm 内。这正是"飞球"窗口。
//   现在 `plan_defense` 在「交点追不上」时也会先问它 ⇒ 它才真正接上电。
TUNABLE(kEarlyStep, 10.0);
TUNABLE(kEarlyMaxDist, 150.0);
TUNABLE(kEarlyProtectDist, 140.0);
TUNABLE(kEarlyDribbleDist, 20.0);

// 球每帧衰减(真机 0.992~0.994) / 会合提前 6 帧 / 外推上限 40 帧 / 边界余量 12cm / 迎球朝向开关
TUNABLE(kBallDecay, 0.993);
TUNABLE(kMeetLead, 6.0);
TUNABLE(kMeetHorizon, 40.0);
TUNABLE(kMeetMargin, 12.0);
TUNABLE(kFaceIncoming, 1.0);

// 断球点 = 球轨迹 ∩ 门前拦截线；false = 球没朝门滚（调用方回退静态点）
bool intercept_point(const WorldModel &wm, double line_dist,
                     double &ix, double &iy) {
    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;
    double vx = wm.ball.vx, vy = wm.ball.vy;

    double line_x = ctx.our_goal_x() + ctx.attack_dir() * line_dist;

    double y_at_line = 0.0;
    if (!predict_y_at_x_reflect(bx, by, vx, vy, line_x, y_at_line)) {
        return false;
    }

    ix = clamp(line_x, kMinX, kMaxX);
    iy = clamp(y_at_line, kMinY, kMaxY);
    return true;
}

// 主动截球：沿球轨迹扫点，找最早「我能比球先到」的点（补横传/斜传盲区）
bool early_intercept_point(const WorldModel &wm, int defender_id,
                           double &out_x, double &out_y) {
    if (kEarlyEnable < 0.5) return false;
    if (opp_clear_dist(wm) < kEarlyDribbleDist) return false;

    const double spd = ball_speed(wm.ball.vx, wm.ball.vy);
    if (spd < kMinBallSpeed) return false;

    const double ux = wm.ball.vx / spd;
    const double uy = wm.ball.vy / spd;
    const double mx = wm.home[defender_id].x;
    const double my = wm.home[defender_id].y;

    for (double d = kEarlyStep; d <= kEarlyMaxDist; d += kEarlyStep) {
        const double px = wm.ball.x + ux * d;
        const double py = wm.ball.y + uy * d;
        if (px < kMinX || px > kMaxX || py < kMinY || py > kMaxY) continue;
        if (wm.ctx.dist_our_goal(px) > kEarlyProtectDist) continue;   // 不在防区不追
        const double t_ball = d / spd;
        const double t_me   = dist(mx, my, px, py) / kMySpeed;
        if (t_me <= t_ball * kReachMargin) {
            out_x = px;
            out_y = py;
            return true;
        }
    }
    return false;
}

// 会合点：沿球未来轨迹逐帧外推（含衰减），找「我比球早到 lead 帧」的点并给出迎球朝向
bool ball_meeting_point(const WorldModel &wm, double px, double py,
                        double my_speed, double lead_frames,
                        double &out_x, double &out_y, double &out_aim) {
    const double spd = ball_speed(wm.ball.vx, wm.ball.vy);
    if (spd < kMinBallSpeed) return false;
    if (!(my_speed > 1e-6) || !std::isfinite(my_speed)) return false;
    if (!std::isfinite(px) || !std::isfinite(py)) return false;

    const double lead = std::max(0.0, lead_frames);
    const double dec  = clamp(kBallDecay, 0.5, 1.0);
    const int horizon = (int)clamp(kMeetHorizon, 1.0, 120.0);
    double bx = wm.ball.x, by = wm.ball.y, vx = wm.ball.vx, vy = wm.ball.vy;

    for (int k = 1; k <= horizon; ++k) {
        bx += vx; by += vy; vx *= dec; vy *= dec;
        if (bx < kMeetMargin || bx > TeamContext::FIELD_LENGTH - kMeetMargin ||
            by < kMeetMargin || by > TeamContext::FIELD_WIDTH - kMeetMargin) break;
        const double t_me = dist(px, py, bx, by) / my_speed;
        if (t_me + lead <= (double)k) {
            out_x = bx; out_y = by;
            out_aim = angle_to(0.0, 0.0, -vx, -vy);
            return true;
        }
    }
    return false;
}

// 主入口：算 defender_id 的断球点；球朝门 = vx·(门x−球x) > 0
DefensePlan plan_defense(const WorldModel &wm, int defender_id) {
    DefensePlan plan;
    const TeamContext &ctx = wm.ctx;
    plan.ball_spd = ball_speed(wm.ball.vx, wm.ball.vy);

    double toward_goal = wm.ball.vx * (ctx.our_goal_x() - wm.ball.x);
    plan.approaching = toward_goal > 0.0;

    // 有威胁 → 真实轨迹断球点；追不上或球慢/背离 → 先找"更早的可截点"，再回退静态球-门连线点
    double ix = 0.0, iy = 0.0;
    bool on_ball_path = false;
    bool got_line = false;
    if (plan.approaching && plan.ball_spd >= kMinBallSpeed &&
        intercept_point(wm, kInterceptLineDist, ix, iy)) {
        double t_ball = dist(wm.ball.x, wm.ball.y, ix, iy) / plan.ball_spd;
        double t_me   = dist(wm.home[defender_id].x, wm.home[defender_id].y, ix, iy) / kMySpeed;
        got_line = (t_me <= t_ball * kReachMargin);
    }
    if (got_line) {
        plan.target_x = ix;
        plan.target_y = iy;
        on_ball_path = true;
    } else if (early_intercept_point(wm, defender_id, ix, iy)) {
        // 2026-10-10 接线：原来这里在"追不上门前 48.5cm 的交点"时**直接退回静态球-门连线点**
        //   ⇒ 球一旦进到离门线 48.5cm 以内，`intercept_point` 因为 t<0 必然失败，
        //     防守者就彻底站住不动了。真机 20 个失球里 11 球发生在门前 60cm 内。
        //   现在改成：先沿球轨迹找一个"我确实能比球先到"的更早点（kEarlyProtectDist 护栏仍然生效），
        //   只有连这个都找不到才站定。
        plan.target_x = ix;
        plan.target_y = iy;
        on_ball_path = true;
    } else {
        plan.target_x = wm.passive_x;
        plan.target_y = wm.passive_y;
    }

    // 迎球朝向：本平台球出射方向 ≈ 机头方向，斜着迎球会把球横顶出去
    if (kFaceIncoming > 0.5 && on_ball_path && plan.ball_spd >= kMinBallSpeed) {
        plan.aim_rot = angle_to(0.0, 0.0, -wm.ball.vx, -wm.ball.vy);
        plan.face_incoming = true;
    }

    // 规则红线：断球点不得进己方门区（只有守门员能进）
    if (in_goal_area(ctx, plan.target_x, plan.target_y)) {
        plan.target_x = clamp(ctx.our_goal_x() + ctx.attack_dir() * 55.0, kMinX, kMaxX);
        plan.target_y = clamp(wm.ball.y, 75.0, 105.0);
    }

    // 防推球：防守点落进己方大禁区就推出；离球 <8cm 时退到球后 8cm
    auto push_out_of_penalty = [&](double &tx, double &ty) {
        if (in_penalty_area(ctx, tx, ty)) {
            tx = ctx.our_goal_x() + ctx.attack_dir() * 85.0;
            ty = 90.0;
        }
    };
    push_out_of_penalty(plan.target_x, plan.target_y);
    double db = dist(plan.target_x, plan.target_y, wm.ball.x, wm.ball.y);
    if (db < 8.0) {
        double ang = atan2(wm.ball.y - plan.target_y, wm.ball.x - plan.target_x);
        plan.target_x = wm.ball.x - 8.0 * cos(ang);
        plan.target_y = wm.ball.y - 8.0 * sin(ang);
        push_out_of_penalty(plan.target_x, plan.target_y);
    }
    // 也不得进对方门区（2+ 人 → 罚点球）；前缘只「夹」不「覆盖」，留 8cm（蓝 58 / 黄 162）
    clamp_out_opp_goal_area(ctx, plan.target_x, plan.target_y);
    double ogx = ctx.opp_goal_x();
    double ad  = ctx.attack_dir();
    double opp_front = ogx - ad * 58.0;
    if (ad < 0.0) {
        if (plan.target_x < opp_front) plan.target_x = opp_front;
    } else {
        if (plan.target_x > opp_front) plan.target_x = opp_front;
    }

    return plan;
}

// 球-门连线护门点：球远站球后 45cm，中近站门前 50cm，贴门站球前 8cm 堵推射线
bool goal_cover_point(const WorldModel &wm, double &out_x, double &out_y) {
    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;
    double d_goal = ctx.dist_our_goal(bx);

    if (d_goal > 100.0) {
        double dx = ctx.our_goal_x() - bx, dy = 90.0 - by;
        double len = std::hypot(dx, dy);
        if (len < 1e-6) { out_x = ctx.our_goal_x() + ctx.attack_dir() * 45.0; out_y = 90.0; }
        else {
            out_x = bx + dx / len * 45.0;
            out_y = by + dy / len * 45.0;
        }
    } else if (d_goal > 45.0) {
        out_x = ctx.our_goal_x() + ctx.attack_dir() * 50.0;
        out_y = clamp(by, 72.5, 107.5);
    } else {
        // 球已贴门：必须站「球与门之间」，站门侧追不上被同向推的球
        double dx = ctx.our_goal_x() - bx, dy = 90.0 - by;
        double len = std::hypot(dx, dy);
        if (len < 1e-6) { out_x = bx + ctx.attack_dir() * 8.0; out_y = by; }
        else {
            out_x = bx + dx / len * 8.0;
            out_y = by + dy / len * 8.0;
        }
        // 朝门方向最多到门前 3cm；球更靠门就直接站门前堵门；入裁判门区则退到门区外缘 29cm
        double door_x = ctx.our_goal_x() + ctx.attack_dir() * 3.0;
        double lo = std::min(bx, door_x), hi = std::max(bx, door_x);
        out_x = clamp(out_x, lo, hi);
        const double door_gap = (door_x - ctx.our_goal_x()) * ctx.attack_dir();
        if ((bx - ctx.our_goal_x()) * ctx.attack_dir() < door_gap) out_x = door_x;
        if (in_goal_area_rule(ctx, out_x, out_y, 8.0, 6.0)) {
            out_x = ctx.our_goal_x() + ctx.attack_dir() * 29.0;
            return true;
        }
        out_x = clamp(out_x, kMinX, TeamContext::FIELD_LENGTH);   // 护门点可深入，不受 kMaxX 约束
        out_y = clamp(out_y, kMinY, kMaxY);
        return true;
    }
    out_x = clamp(out_x, kMinX, kMaxX);
    out_y = clamp(out_y, kMinY, kMaxY);
    return true;
}

// 人盯人威胁打分：控球 50/(d+K) + 位置 25/(d+K) + 接球 0.4·approach + 突破 0.3·danger
double mark_threat(double d_ball, double d_goal,
                   double approach_speed, double danger_speed, bool is_dribbler) {
    const double K = 10.0;
    double s = 50.0 / (d_ball + K);
    s      += 25.0 / (d_goal + K);
    double reach = 1.0 / (1.0 + d_ball / 20.0);
    s += 0.4 * approach_speed * reach;
    if (is_dribbler) s += 0.3 * danger_speed;
    return s;
}

// 选威胁最大者，含 5% 滞回与危险门限；离球离门都远则 -1 回区域防守
int pick_mark_target(const WorldModel &wm, int current_target) {
    const TeamContext &ctx = wm.ctx;
    double danger = ball_danger_speed(wm);

    int dribbler = nearest_opp_to_ball(wm);

    int best = -1;
    double best_score = -1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d_ball = dist(wm.ball.x, wm.ball.y, wm.opp[i].x, wm.opp[i].y);
        double d_goal = wm.ctx.dist_our_goal(wm.opp[i].x);
        double appr = ball_approach_speed(wm, wm.opp[i].x, wm.opp[i].y);
        double score = mark_threat(d_ball, d_goal, appr, danger, i == dribbler);
        if (score > best_score) { best_score = score; best = i; }
    }

    if (current_target >= 0 && current_target < PLAYERS_PER_SIDE &&
        best != current_target) {
        double cur_d_ball = dist(wm.ball.x, wm.ball.y,
                                 wm.opp[current_target].x, wm.opp[current_target].y);
        double cur_d_goal = wm.ctx.dist_our_goal(wm.opp[current_target].x);
        double cur_appr = ball_approach_speed(wm, wm.opp[current_target].x,
                                              wm.opp[current_target].y);
        double cur_score = mark_threat(cur_d_ball, cur_d_goal, cur_appr, danger,
                                       current_target == dribbler);
        if (best_score <= cur_score * 1.05) best = current_target;
    }

    if (best >= 0) {
        double d_ball = dist(wm.ball.x, wm.ball.y, wm.opp[best].x, wm.opp[best].y);
        double d_goal = wm.ctx.dist_our_goal(wm.opp[best].x);
        if (d_ball > mark_engage_ball_dist() && d_goal > mark_engage_goal_dist()) {
            return -1;
        }
    }
    return best;
}

// 二抢一门槛(cm)：夹抢距离 111、横向错开 32.5、离球 15 内算带球、53.8 内进禁区协防
TUNABLE(kDoubleTeamDangerDist, 111.151);
TUNABLE(kDoubleTeamLateral, 32.5);
TUNABLE(kDoubleTeamCarryDist, 15);
TUNABLE(kDoubleTeamCoverDist, 53.832);

// 带权匈牙利盯人分配（自研）：1=全队一一匹配，0=回退贪心；参与门槛 0.55、λ=25cm、EMA 0.35、量化 5cm、承诺 20 帧、无人盯代价 22cm×威胁
TUNABLE(kMarkHungarian, 1.0);

TUNABLE(kMarkRelGate, 0.55);

TUNABLE(kMarkLambda, 25.0);

TUNABLE(kMarkEma, 0.35);

TUNABLE(kMarkQuant, 5.0);

TUNABLE(kMarkCommit, 20.0);

TUNABLE(kMarkLeaveW, 22.0);

// 局面是否该盯人（威胁够高、非点球执行期、球有效）/ 清空本帧指派（不动 prev·commit）
static bool mark_gate_open(const WorldModel &wm) {
    return kMarkHungarian >= 0.5 && wm.ball.valid && !wm.in_penalty_exec &&
           wm.threat_level >= kMarkRelGate;
}

static void mark_clear(WorldModel &wm) {
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) wm.mark_assign[i] = -1;
    wm.mark_assign_valid = false;
}

// 全队最优一一匹配（行=可盯人防守者，列=对手+虚拟列，补虚拟行表示无人盯），写回指派并返回台数
int assign_marks(WorldModel &wm) {
    if (!mark_gate_open(wm)) { mark_clear(wm); return 0; }

    int rows[PLAYERS_PER_SIDE], nr = 0;
    for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {
        const int r = wm.role[i];
        if (r != ROLE_PASSIVE && r != ROLE_ASSIST && r != ROLE_MIDFIELD) continue;
        if (i == wm.sweeper_id || i == wm.presser_id) continue;
        rows[nr++] = i;
    }
    constexpr int M = PLAYERS_PER_SIDE;
    if (nr <= 0) { mark_clear(wm); return 0; }

    const double danger = ball_danger_speed(wm);
    const int dribbler = nearest_opp_to_ball(wm);
    double threat[M];
    for (int j = 0; j < M; ++j) {
        const double d_ball = dist(wm.ball.x, wm.ball.y, wm.opp[j].x, wm.opp[j].y);
        const double d_goal = wm.ctx.dist_our_goal(wm.opp[j].x);
        const double appr = ball_approach_speed(wm, wm.opp[j].x, wm.opp[j].y);
        threat[j] = mark_threat(d_ball, d_goal, appr, danger, j == dribbler);
    }

    // 代价 = 我到对手预测位置的距离(EMA 平滑+量化) + 换人惩罚 λ（承诺窗口内再加一个 λ）
    const double lambda = sharp_defense_on() ? sharp_mark_lambda() : kMarkLambda;
    double real[PLAYERS_PER_SIDE][M];
    for (int r = 0; r < nr; ++r) {
        const int i = rows[r];
        for (int j = 0; j < M; ++j) {
            const double px = wm.opp[j].x + wm.opp_vx[j] * mark_lead();
            const double py = wm.opp[j].y + wm.opp_vy[j] * mark_lead();
            const double now = dist(wm.home[i].x, wm.home[i].y, px, py);
            if (!wm.mark_ema_ready) wm.mark_cost_ema[i][j] = now;
            else wm.mark_cost_ema[i][j] = kMarkEma * now + (1.0 - kMarkEma) * wm.mark_cost_ema[i][j];
            const double q = (kMarkQuant > 1e-6)
                           ? std::floor(wm.mark_cost_ema[i][j] / kMarkQuant + 0.5) * kMarkQuant
                           : wm.mark_cost_ema[i][j];
            double lam = 0.0;
            if (wm.mark_prev_assign[i] >= 0 && wm.mark_prev_assign[i] != j) {
                lam = lambda + ((wm.mark_commit[i] > 0) ? lambda : 0.0);
            }
            real[r][j] = q + lam;
        }
    }
    wm.mark_ema_ready = true;

    // 补成方阵：虚拟行×对手 = kMarkLeaveW×威胁（越危险越贵），防守者×虚拟列 = 0
    const int n = nr + M;
    if (n > kHungarianMaxN) { mark_clear(wm); return 0; }
    double cost[kHungarianMaxN * kHungarianMaxN];
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            double value = 0.0;
            if (r < nr && c < M)       value = real[r][c];
            else if (r >= nr && c < M) value = kMarkLeaveW * threat[c];
            else                       value = 0.0;
            cost[r * n + c] = value;
        }
    }
    int row_to_col[kHungarianMaxN];
    if (!hungarian_solve(cost, n, row_to_col)) { mark_clear(wm); return 0; }

    // 落地：列在对手区就是真指派，写回状态并统计换人（-1→某人 不算换人）
    int assigned = 0, switches = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) wm.mark_assign[i] = -1;
    for (int r = 0; r < nr; ++r) {
        const int i = rows[r];
        const int col = row_to_col[r];
        const int target = (col >= 0 && col < M) ? col : -1;
        wm.mark_assign[i] = target;
        if (target >= 0) ++assigned;
        if (target != wm.mark_prev_assign[i]) {
            if (wm.mark_prev_assign[i] >= 0) ++switches;
            wm.mark_commit[i] = static_cast<int>(kMarkCommit);
        }
    }
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i)
        if (wm.mark_commit[i] > 0) --wm.mark_commit[i];
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) wm.mark_prev_assign[i] = wm.mark_assign[i];
    wm.mark_switch_events += switches;
    wm.mark_assign_valid = true;
    return assigned;
}

// 抢断唯一竞标（自研）：每帧只让 EV 最高且 >0 的一人抢球；候选 60cm、球速 ≤6cm/帧、价值 40、λ=8、有协防 30cm
TUNABLE(kStealEnable, 1.0);
TUNABLE(kStealDist, 60.0);
TUNABLE(kStealVMax, 6.0);
TUNABLE(kStealValue, 40.0);
TUNABLE(kStealLambda, 8.0);
TUNABLE(kStealA, 2.0);
TUNABLE(kStealB, 0.1);
TUNABLE(kStealC, 0.5);
TUNABLE(kStealD, 0.8);
TUNABLE(kStealCover, 30.0);

// EV = 成功率 p×夺回价值 − (1−p)×失位威胁（原盯人威胁分）− λ；太远/球太快返回 -1e9
static double steal_exposed_cost(const WorldModel &wm, int i) {
    int t = wm.mark_assign[i];
    if (t < 0 || t >= PLAYERS_PER_SIDE) return 0.0;
    const double danger = ball_danger_speed(wm);
    const int dribbler = nearest_opp_to_ball(wm);
    const double d_ball = dist(wm.ball.x, wm.ball.y, wm.opp[t].x, wm.opp[t].y);
    const double d_goal = wm.ctx.dist_our_goal(wm.opp[t].x);
    const double appr = ball_approach_speed(wm, wm.opp[t].x, wm.opp[t].y);
    return mark_threat(d_ball, d_goal, appr, danger, t == dribbler);
}

static double steal_ev(const WorldModel &wm, int i) {
    const RobotState &me = wm.home[i];
    const double d_ball = dist(me.x, me.y, wm.ball.x, wm.ball.y);
    const double opp_spd = ball_speed(wm.ball.vx, wm.ball.vy);
    if (d_ball > kStealDist || opp_spd > kStealVMax) return -1e9;

    bool has_cover = false;
    for (int k = 0; k < PLAYERS_PER_SIDE; ++k) {
        if (k == i || wm.role[k] == ROLE_GOALIE) continue;
        if (dist(me.x, me.y, wm.home[k].x, wm.home[k].y) < kStealCover) { has_cover = true; break; }
    }

    const double z = -kStealA + kStealB * d_ball + kStealC * opp_spd - (has_cover ? kStealD : 0.0);
    const double p = 1.0 / (1.0 + std::exp(z));
    const double gain = p * kStealValue;
    const double loss = (1.0 - p) * steal_exposed_cost(wm, i);
    return gain - loss - kStealLambda;
}

void steal_decide(WorldModel &wm) {
    wm.stealer_id = -1;
    if (kStealEnable < 0.5) return;   // 关 → 回退旧「谁近谁抢」
    double best = 0.0;
    for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {   // 0=门将，不参与
        const int r = wm.role[i];
        if (r != ROLE_PASSIVE && r != ROLE_ASSIST && r != ROLE_MIDFIELD) continue;
        if (i == wm.sweeper_id || i == wm.presser_id) continue;
        const double ev = steal_ev(wm, i);
        if (ev > best) { best = ev; wm.stealer_id = i; }
    }
}

// 二抢一站位：持球者压到门前危险区时算夹抢点（只让更近的非清道夫上前），false = 不夹
bool double_team_point(const WorldModel &wm, int defender_id,
                       double &out_x, double &out_y) {
    if (wm.threat_level < 0.6) return false;

    double dmin = 1e9;
    int dribbler = nearest_opp_to_ball(wm, &dmin);
    if (dribbler < 0 || dmin >= kDoubleTeamCarryDist) return false;

    if (in_penalty_area(wm.ctx, wm.ball.x, wm.ball.y)) {
        if (wm.ctx.dist_our_goal(wm.opp[dribbler].x) > kDoubleTeamCoverDist) return false;
    }

    if (wm.ctx.dist_our_goal(wm.opp[dribbler].x) > kDoubleTeamDangerDist) return false;

    double ox = wm.opp[dribbler].x, oy = wm.opp[dribbler].y;
    double my_d = dist(wm.home[defender_id].x, wm.home[defender_id].y, ox, oy);
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (i == defender_id || i == wm.sweeper_id) continue;
        if (wm.role[i] != ROLE_ASSIST && wm.role[i] != ROLE_MIDFIELD) continue;
        double d = dist(wm.home[i].x, wm.home[i].y, ox, oy);
        if (d < my_d) return false;
    }

    // 夹抢点：持球者推进方向（无速度则门向）前方 mark_dist 处，再横向错开到持球者一侧
    double gx = wm.ctx.our_goal_x(), gy = 90.0;
    double dx = gx - ox, dy = gy - oy;
    double len = std::hypot(dx, dy);
    if (len < 1e-6) return false;
    dx /= len; dy /= len;
    double nx = -dy, ny = dx;
    double side = (oy >= 90.0) ? 1.0 : -1.0;
    double vx = wm.opp_vx[dribbler], vy = wm.opp_vy[dribbler];
    double vlen = std::hypot(vx, vy);
    if (vlen > 0.5) {
        out_x = ox + (vx / vlen) * mark_dist() + nx * side * kDoubleTeamLateral;
        out_y = oy + (vy / vlen) * mark_dist() + ny * side * kDoubleTeamLateral;
    } else {
        out_x = ox + dx * mark_dist() + nx * side * kDoubleTeamLateral;
        out_y = oy + dy * mark_dist() + ny * side * kDoubleTeamLateral;
    }

    // 禁区纪律：夹抢点落入己方门区则退到门区前缘外（罚球区允许进）
    if (in_goal_area(wm.ctx, out_x, out_y)) {
        out_x = wm.ctx.our_goal_x() + wm.ctx.attack_dir() * 55.0;
        out_y = clamp(oy, 72.5, 107.5);
    }
    out_x = clamp(out_x, 0.0, TeamContext::FIELD_LENGTH);
    out_y = clamp(out_y, 0.0, TeamContext::FIELD_WIDTH);
    return true;
}

}  // namespace simuro5
