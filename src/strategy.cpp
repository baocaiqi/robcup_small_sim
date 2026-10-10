#include "simuro5/strategy.hpp"
#include "simuro5/roles.hpp"
#include "simuro5/motion.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/defense.hpp"
#define TUNABLE_PREFIX "strategy."
#include "simuro5/tunable.hpp"
#include <cmath>
#include "simuro5/branch_trace.hpp"   // 须在所有 include 之后

namespace simuro5 {

// 状态机：滞回帧数 / 威胁降档滞回帧(升快降慢，<=0关) / 反击窗口帧 / 分区防守开关(0=关)
TUNABLE(kStateHysteresisFrames, 3);
TUNABLE(kThreatHoldFrames, 10);
TUNABLE(kCounterWindowFrames, 30);
TUNABLE(kZoneMode, 0.0);

// 罚球点（真机实测）：距门线 39.4cm、容差 1.5cm、静止阈值 1.0cm/帧
TUNABLE(kPenaltySpotDist, 39.4);
TUNABLE(kPenaltySpotTol, 1.5);
TUNABLE(kPenaltySpotStill, 1.0);

// ═══ 门前禁止把球朝自家门推（防乌龙，2026-10-08）═══
//   失球归因（离线 bench 3 局 72000 帧，16 个失球）：
//     · 12 个是我们自己最后触球（乌龙），其中门将 4 个、主攻 3 个、中场 3 个
//     · 10/12 个乌龙是 >100cm/s 的高速球
//   根因：门前触球时把球朝自家门加速 —— "越解越往门里送"。
//   做法与门将已有的「闸一」同源：从轮速里**只扣掉朝自家门的平动分量**，
//   转动分量与横向分量全部保留 ⇒ 照样能转身、能横传、能把球捅到边路，
//   只是不许再朝自家门那一侧产生位移。
//   触发条件：球在己方半场靠近自家门（默认 150cm 内）+ 离球 ≤ 触球半径。
//   ⚠️ 距离 2026-10-08 由 40 放宽到 150：逐球取证发现乌龙**不是门前捅进去的** ——
//      我们碰球的位置离自家门线中位 98cm（最远 186cm），12/15 个乌龙在 60cm 之外，
//      即"在自家后场把球碰向自家门、然后一路滚进去"。40cm 的闸压根够不到这些球。
TUNABLE(kOwnGoalGuard, 1.0);         // 0 = 整条关闭（回滚）
TUNABLE(kOwnGoalGuardR, 8.0);        // cm：离球这么近才可能触球，才需要戴闸
TUNABLE(kOwnGoalGuardDist, 150.0);   // cm：球离己方门线这么近才戴闸

// 作用域：只对 1~4 号（门将有它自己的闸一，别重复扣）
void enforce_no_own_goal_touch(WorldModel &wm) {
    if (kOwnGoalGuard < 0.5 || !wm.ball.valid) return;
    const TeamContext &ctx = wm.ctx;
    if (ctx.dist_our_goal(wm.ball.x) > kOwnGoalGuardDist) return;   // 球还远 → 不关这事
    for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {
        RobotState &r = wm.home[i];
        if (!std::isfinite(r.x) || !std::isfinite(r.y)) continue;
        if (dist(r.x, r.y, wm.ball.x, wm.ball.y) > kOwnGoalGuardR) continue;

        const double vf = 0.5 * (r.vl + r.vr);        // 沿机头的平动分量
        const double spin = 0.5 * (r.vr - r.vl);      // 转动分量（原样保留）
        if (std::fabs(vf) <= 1e-9) continue;
        const double dir = (vf >= 0.0) ? r.rot : r.rot + 180.0;   // 平动实际朝向
        const double goalward = (ctx.attack_dir() > 0.0) ? 180.0 : 0.0;  // 朝自家门的方向
        const double c = std::cos((dir - goalward) * 3.14159265358979323846 / 180.0);
        if (c <= 0.0) continue;                       // 没有朝门分量 → 不动
        const double vf2 = vf * (1.0 - c);            // 按投影扣掉朝门那一份
        r.vl = vf2 - spin;
        r.vr = vf2 + spin;
    }
}

bool we_take_penalty_spot(const WorldModel &wm) {
    bool state_says_ours = (wm.ctx.is_blue && wm.game_state == PM_PenaltyKick_Blue) ||
                           (!wm.ctx.is_blue && wm.game_state == PM_PenaltyKick_Yellow);
    if (state_says_ours) return true;
    if (!wm.ball.valid) return false;
    if (std::hypot(wm.ball.vx, wm.ball.vy) >= kPenaltySpotStill) return false;
    double spot_x = wm.ctx.opp_goal_x() - wm.ctx.attack_dir() * kPenaltySpotDist;
    return std::fabs(wm.ball.x - spot_x) < kPenaltySpotTol &&
           std::fabs(wm.ball.y - 90.0) < kPenaltySpotTol;
}

// 传球参与者先执行任务；普通逼抢只接管没有任务的球员。
static bool pass_task_controls_robot(const WorldModel &wm, int id) {
    const CoopPassTask &task = wm.coop_pass_task;
    if (task.active && (id == task.passer_id || id == task.receiver_id)) return true;
    const CoopBallControl &control = wm.coop_ball_control;
    return control.active && (id == wm.active_id || id == control.receiver_id);
}

void Strategy::run(WorldModel &wm) {
    Situation sit = sit_.analyze(wm);
    wm.possession = sit.possession;
    wm.we_have_ball = (wm.possession == Possession::Ours);
    if (sit.whos_mismatch) ++wm.whos_disagree;

    // 我方点球执行期：平台执行期不报点球态，改用「球静止在对方罚球点」判据
    {
        bool we_take = we_take_penalty_spot(wm);
        if (we_take) {
            wm.in_penalty_exec = true;
        } else if (wm.in_penalty_exec) {
            bool ball_leaves = std::hypot(wm.ball.vx, wm.ball.vy) > 3.0 ||
                               std::fabs(wm.ball.x - wm.ctx.opp_goal_x()) > 60.0 ||
                               std::fabs(wm.ball.y - 90.0) > 30.0;
            if (wm.game_state != PM_PlayOn || ball_leaves) wm.in_penalty_exec = false;
        }
    }

    update_team_state(wm);

    sit_.update_stand_points(wm);

    // 先清失效任务，再按角色/逼抢派发；取消当帧即可恢复普通决策。
    cancel_unsafe_coop_pass(wm);
    ra_.assign(wm);

    update_sweeper(wm);

    update_presser(wm);

    assign_marks(wm);

    steal_decide(wm);

    // 5. 按角色执行；冷却期内仍在对方门区则撤出并跳过角色函数（防抖振卡区）
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (wm.role[i] == ROLE_GOALIE) { run_goalie(wm, i); continue; }
        if (wm.ga_cooldown[i] > 0) {
            --wm.ga_cooldown[i];
            bool in_ga = in_opp_goal_area(wm.ctx, wm.home[i].x, wm.home[i].y);
            bool ball_in_ga = in_opp_goal_area(wm.ctx, wm.ball.x, wm.ball.y);
            bool shooting_work = ball_in_ga &&
                dist(wm.home[i].x, wm.home[i].y, wm.ball.x, wm.ball.y) <= 25.0;
            bool penalty = wm.in_penalty_exec && wm.role[i] == ROLE_ACTIVE;
            if (in_ga && !shooting_work && !penalty) {
                if (wm.coop_pass_task.active && (i == wm.coop_pass_task.passer_id || i == wm.coop_pass_task.receiver_id))
                    wm.coop_finish(CoopOutcome::GoalDiscipline);
                if (wm.coop_ball_control.active && (i == wm.active_id || i == wm.coop_ball_control.receiver_id))
                    wm.coop_control_end(CoopOutcome::GoalDiscipline);
                double ogx = wm.ctx.opp_goal_x(), ad = wm.ctx.attack_dir();
                motion::position(wm.home[i], ogx - ad * 70.0, clamp(wm.home[i].y, 72.5, 107.5));
                continue;
            }
        }
        if (kZoneMode > 0.5 && wm.runtime_phase == RuntimePhase::Running &&
            !wm.in_penalty_exec && wm.role[i] != ROLE_ACTIVE) {
            run_zone(wm, i); continue;
        }
        // 重启抢球（roles.kRestartCharge，**默认关**：sim A/B 更差，见 roles.cpp 该处说明）：
        //   球还钉在重启摆放点时，球后离球最近的那台直接穿球推出去，不走上层会左右摆的分支
        if (restart_charge(wm, i)) continue;
        // 队员 57fad54：普通逼抢不许抢走有效传球任务的 passer / receiver / 临时控球参与者
        if (i == wm.presser_id && !pass_task_controls_robot(wm, i)) {
            run_press(wm, i); continue;
        }
        switch (wm.role[i]) {
            case ROLE_ACTIVE:   run_active(wm, i); break;
            case ROLE_PASSIVE:  run_passive(wm, i); break;
            case ROLE_ASSIST:   run_assist(wm, i); break;
            case ROLE_MIDFIELD: run_midfield(wm, i); break;
            default:            motion::stop(wm.home[i]); break;
        }
    }

    // 对方门区停留兜底：贴球(≤25cm)不算滞留，其余连续 >15 帧撤出 + 冷却 30 帧
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (wm.role[i] == ROLE_GOALIE) continue;
        double ogx = wm.ctx.opp_goal_x(), ad = wm.ctx.attack_dir();
        double hold_x = ogx - ad * 70.0, hold_y = clamp(wm.home[i].y, 72.5, 107.5);
        bool in_ga = in_opp_goal_area(wm.ctx, wm.home[i].x, wm.home[i].y);
        if (in_ga) {
            bool ball_in_ga = in_opp_goal_area(wm.ctx, wm.ball.x, wm.ball.y);
            bool shooting_work = ball_in_ga &&
                dist(wm.home[i].x, wm.home[i].y, wm.ball.x, wm.ball.y) <= 25.0;
            const int limit = (wm.role[i] == ROLE_ACTIVE) ? 15 : 0;
            if (!shooting_work && ++wm.ga_overstay[i] > limit) {
                if (wm.coop_pass_task.active && (i == wm.coop_pass_task.passer_id || i == wm.coop_pass_task.receiver_id))
                    wm.coop_finish(CoopOutcome::GoalDiscipline);
                if (wm.coop_ball_control.active && (i == wm.active_id || i == wm.coop_ball_control.receiver_id))
                    wm.coop_control_end(CoopOutcome::GoalDiscipline);
                wm.ga_overstay[i] = 0;
                wm.ga_cooldown[i] = 30;
                motion::position(wm.home[i], hold_x, hold_y);
            }
        } else {
            wm.ga_overstay[i] = 0;
        }
    }

    // 门区纪律：门区只能有门将；大禁区非门将最多 3 人（规则7.10.4 红线=4），超限顶离球最远
    enforce_own_goal_area(wm);
    enforce_own_penalty_count(wm);
    enforce_teammate_spacing(wm);      // 须在防乌龙闸之前（可能把队友推开的方向正朝自家门）
    enforce_no_own_goal_touch(wm);
}

// 裁判口径门区余量 X/Y cm（抗惯性过冲）；大禁区非门将上限 3 人、顶出再加 5cm
TUNABLE(kRuleBoxMarginX, 8.0);
TUNABLE(kRuleBoxMarginY, 6.0);
TUNABLE(kOwnBoxMaxOutfield, 3);
TUNABLE(kOwnBoxMarginOut, 5.0);

void enforce_own_goal_area(WorldModel &wm) {
    if (!wm.ball.valid) return;
    const double hold_x = wm.ctx.our_goal_x() + wm.ctx.attack_dir() * 58.0;  // 50 + 8 余量
    for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {
        RobotState &r = wm.home[i];
        if (in_goal_area(wm.ctx, r.x, r.y)) {
            motion::position(r, hold_x, clamp(r.y, 72.5, 107.5), motion::TM_PASS);
            continue;
        }
        // 裁判口径门区更浅更宽（含门柱两侧，见 in_goal_area_rule）：带余量提前顶出
        if (in_goal_area_rule(wm.ctx, r.x, r.y, kRuleBoxMarginX, kRuleBoxMarginY)) {
            double out_x = wm.ctx.our_goal_x() + wm.ctx.attack_dir() * (15.0 + kRuleBoxMarginX + 6.0);
            motion::position(r, out_x, r.y, motion::TM_PASS);
        }
    }
}

void enforce_own_penalty_count(WorldModel &wm) {
    if (!wm.ball.valid) return;
    const TeamContext &ctx = wm.ctx;
    int inside = 0, far_id = -1;
    double far_d = -1.0;
    for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {          // 0 号门将豁免（不算人、也不被驱动）
        const RobotState &r = wm.home[i];
        if (!in_penalty_area(ctx, r.x, r.y)) continue;
        ++inside;
        const double d = dist(r.x, r.y, wm.ball.x, wm.ball.y);
        if (d > far_d) { far_d = d; far_id = i; }
    }
    if (inside <= (int)kOwnBoxMaxOutfield || far_id < 0) return;
    RobotState &r = wm.home[far_id];
    const double out_x = ctx.our_goal_x() + ctx.attack_dir() * (80.0 + kOwnBoxMarginOut);
    motion::position(r, out_x, clamp(r.y, 72.5, 107.5), motion::TM_PASS);
}

// ═══ 同队最小间距（2026-10-09）═══
// 取证（sim_bench 3 局 72000 帧，`_dsh_session/now/` 四个脚本）：
//   · 23.9% 的帧有我方两台车 <12cm（两车半径各 4cm，接触距离 8cm）；<10cm 占 7.8%
//   · 贴住的连续段：中位 8 帧(0.2s)、p90 28 帧(0.7s)、最长 110 帧(2.75s)（黄位最长 6.88s）
//   · 紧贴对里 77.6% 是"两车都在球的后方"（排队等推球），48.9% 两人都在球 40cm 内
//   · 四人散布中位 62cm，而真机基线（docs/03）是 100~101cm
// 根因：全仓唯一的同队间距纪律只作用于 situation.cpp 的三个防守锚点（两两 20cm），
//   而争抢路径（run_swarm 的球后落点 / run_active 的准备点 / 护送 / 持球离场）全绕过它；
//   move_avoiding 的障碍表里只有对手，自己人一个都没有。
// 做法与防乌龙闸同源：把轮速拆成"平动 + 转动"，只往平动里加一个"背离队友"的分量，
//   转动分量（vl 与 vr 之差）原样保留 ⇒ 照样能转身、能对准球，只是不再往队友身上挤。
//   位移增量随重叠量线性消失（贴着阈值时为 0）⇒ 不会在门限上抖。
// ⚠️ 不含 0 号门将（与 enforce_own_goal_area 一致）：门将有自己的门线闸，且大量测试断言它的轮速。
// ⚠️ 纯侧向的推力不改（差速车横移不了）：那种"并排挤"交给 roles.cpp 的 run_swarm 错层处理。
// ⚠️ 默认 0 = 关闭。2026-10-09 实测（6 种子×3 局的同种子对照见 docs/06 第 69 轮）：
//   · 增益 6（温和）：<12cm 帧 23.03%→22.67%，几乎没用；
//   · 增益 25（强）：反而更差，23.03%→24.38% —— 13cm 处就被全速推开 ⇒ 冲过头、被角色控制器
//     拉回、再撞，变成"抖"；而且纯侧向（并排挤）的情况按构造它根本推不动（差速车横移不了）。
//   所以不随构建生效；真机 A/B 时把 `strategy.kTeammateMinSep` 改成 14 再重编。
TUNABLE(kTeammateMinSep, 0.0);    // cm：我方两车小于此距离就互相推开（0 = 整条关闭）
TUNABLE(kTeammatePush, 6.0);      // 推开增益：轮速增量 = 重叠量(cm) × 本值（1cm/帧 ≈ 35 单位）

void enforce_teammate_spacing(WorldModel &wm) {
    if (kTeammateMinSep <= 0.0) return;
    for (int a = 1; a < PLAYERS_PER_SIDE; ++a) {
        RobotState &ra = wm.home[a];
        if (!std::isfinite(ra.x) || !std::isfinite(ra.y)) continue;
        for (int b = a + 1; b < PLAYERS_PER_SIDE; ++b) {
            RobotState &rb = wm.home[b];
            if (!std::isfinite(rb.x) || !std::isfinite(rb.y)) continue;
            double dx = ra.x - rb.x, dy = ra.y - rb.y;
            double d = std::hypot(dx, dy);
            if (d >= kTeammateMinSep) continue;
            if (d < 1e-6) { dx = 1.0; dy = 0.0; d = 1.0; }   // 完全重合：随便挑一个方向
            const double ux = dx / d, uy = dy / d;
            const double need = (kTeammateMinSep - d) * 0.5;  // 各让一半
            // 只加平动：沿机头轴投影，正前方加、正后方减
            auto push = [&](RobotState &r, double wx, double wy) {
                const double fx = std::cos(r.rot * SIMURO5_PI / 180.0);
                const double fy = std::sin(r.rot * SIMURO5_PI / 180.0);
                const double c = wx * fx + wy * fy;
                if (std::fabs(c) < 1e-9) return;              // 纯侧向：推不动，交给角色自己解决
                const double add = std::copysign(need * kTeammatePush, c);
                r.vl = clamp(r.vl + add, -motion::kMaxWheel, motion::kMaxWheel);
                r.vr = clamp(r.vr + add, -motion::kMaxWheel, motion::kMaxWheel);
            };
            push(ra, ux, uy);        // a 背离 b
            push(rb, -ux, -uy);      // b 背离 a
        }
    }
}

void Strategy::update_team_state(WorldModel &wm) {
    if (wm.we_have_ball) { ++wm.possession_frames; wm.no_possession_frames = 0; }
    else                 { ++wm.no_possession_frames; wm.possession_frames = 0; }

    if (wm.we_have_ball && !wm.prev_we_have_ball &&
        wm.runtime_phase == RuntimePhase::Running) {
        wm.counter_attack_frames = kCounterWindowFrames;
    }
    if (wm.counter_attack_frames > 0) --wm.counter_attack_frames;
    wm.prev_we_have_ball = wm.we_have_ball;

    TeamState prev = wm.team_state;
    if (wm.team_state == TS_DEFENSE && wm.possession_frames >= kStateHysteresisFrames)
        wm.team_state = TS_ATTACK;
    else if (wm.team_state == TS_ATTACK && wm.no_possession_frames >= kStateHysteresisFrames)
        wm.team_state = TS_DEFENSE;
    wm.state_transition = (wm.team_state != prev);

    double raw_threat = threat_from_state(wm);
    if (kThreatHoldFrames <= 0 || raw_threat >= wm.threat_level) {
        wm.threat_level = raw_threat;
        wm.threat_hold_frames = 0;
    } else if (wm.team_state == TS_ATTACK) {
        wm.threat_level = raw_threat;
        wm.threat_hold_frames = 0;
    } else if (++wm.threat_hold_frames >= kThreatHoldFrames) {
        wm.threat_level = raw_threat;
        wm.threat_hold_frames = 0;
    }
}

double Strategy::threat_from_state(const WorldModel &wm) const {
    const TeamContext &ctx = wm.ctx;
    if (wm.team_state == TS_ATTACK) return 0.1;
    if (in_penalty_area(ctx, wm.ball.x, wm.ball.y)) return 1.0;
    bool our_half = ctx.attack_dir() > 0 ? (wm.ball.x < 110.0) : (wm.ball.x > 110.0);
    double threat = our_half ? 0.6 : 0.4;

    // 球朝己方门快速滚时提前升档（阈值 cm/帧，犀利防守档降到 4）
    const double danger_speed = sharp_defense_on() ? sharp_danger_speed() : 6.0;
    if (ball_danger_speed(wm) > danger_speed) {
        threat = our_half ? 0.8 : 0.6;
    }
    return threat;
}

void Strategy::update_sweeper(WorldModel &wm) {
    wm.sweeper_id = -1;
    if (wm.team_state != TS_DEFENSE) return;

    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;

    // 触发：球在本方防守三区且拉边(|y-90|>30)——静态球-门连线会让中路/远门柱真空
    const double third = TeamContext::FIELD_LENGTH / 3.0;
    bool our_third = (ctx.attack_dir() > 0) ? (bx < third)
                                            : (bx > TeamContext::FIELD_LENGTH - third);
    if (!our_third || std::fabs(by - 90.0) <= 30.0) return;

    int ids[2] = { -1, -1 };
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (wm.role[i] == ROLE_ASSIST)        ids[0] = i;
        else if (wm.role[i] == ROLE_MIDFIELD) ids[1] = i;
    }
    double d0 = (ids[0] >= 0) ? dist(bx, by, wm.home[ids[0]].x, wm.home[ids[0]].y) : -1.0;
    double d1 = (ids[1] >= 0) ? dist(bx, by, wm.home[ids[1]].x, wm.home[ids[1]].y) : -1.0;
    if (d0 < 0.0 && d1 < 0.0) return;
    wm.sweeper_id = (d0 >= d1) ? ids[0] : ids[1];

    // 清道夫站位：己方罚球区前缘外 85cm、中路 y=90（封中路与远门柱，不进罚球区）
    wm.sweeper_x = ctx.our_goal_x() + ctx.attack_dir() * 85.0;
    wm.sweeper_y = 90.0;
}

// 前场散球逼抢者：球在前场且静止/周围无对方时，从进攻三人组打分选一台（距离cm、球速cm/帧）
TUNABLE(kPressEnabled, 1.0);
TUNABLE(kPressMaxDist, 250.0);
TUNABLE(kPressOppClearDist, 40.0);
TUNABLE(kPressStillSpeed, 1.0);
TUNABLE(kForwardBonus, 0.6);
TUNABLE(kForwardMax, 60.0);
TUNABLE(kPressHysteresis, 15.0);
// 后场散球归属（2026-10-10）：见 update_presser 里的说明。0 = 关闭（回到"只在前场派"）
TUNABLE(kPressBackEnabled, 1.0);
TUNABLE(kPressBackMinGoalDist, 35.0);   // cm：球离己方门线近于此就不派（交给门将）

void Strategy::update_presser(WorldModel &wm) {
    int cur = wm.presser_id;
    wm.presser_id = -1;

    if (kPressEnabled <= 0.0) return;
    if (!wm.ball.valid) return;
    if (wm.runtime_phase != RuntimePhase::Running) return;
    if (wm.we_have_ball) return;

    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;

    bool front = (ctx.attack_dir() > 0) ? (bx > 110.0) : (bx < 110.0);
    // 2026-10-10：后场也派一个人去抢「没人管」的散球。
    //   真机 8 场 20 个失球取证：越线前 0.5 秒我方没有任何非门将在球 30cm 内占 **45%**；
    //   其中 6~7 球是球从 70~190cm 外一路滚进自家门、两边都没人碰它。
    //   原来 `if (!front) return;` —— 逼抢者**只在前场指派**，后场散球没有归属。
    //   离门线太近（< kPressBackMinGoalDist）不派：那里交给门将和门区纪律（别抢门将的活）。
    const bool back_loose = (kPressBackEnabled > 0.5) && !front &&
                            ctx.dist_our_goal(bx) > kPressBackMinGoalDist;
    if (!front && !back_loose) return;

    bool still = std::hypot(wm.ball.vx, wm.ball.vy) < kPressStillSpeed;
    if (!still) {
        double opp_min = 1e9;
        for (int j = 0; j < PLAYERS_PER_SIDE; ++j)
            opp_min = std::min(opp_min, dist(bx, by, wm.opp[j].x, wm.opp[j].y));
        if (opp_min <= kPressOppClearDist) return;
    }

    double ad = ctx.attack_dir();
    int best = -1;
    double best_score = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        int rl = wm.role[i];
        if (rl != ROLE_ACTIVE && rl != ROLE_ASSIST && rl != ROLE_MIDFIELD) continue;
        if (pass_task_controls_robot(wm, i)) continue;
        double d = dist(bx, by, wm.home[i].x, wm.home[i].y);
        if (d > kPressMaxDist) continue;
        double forward = clamp(ad * (bx - wm.home[i].x), 0.0, kForwardMax);
        double score = d - kForwardBonus * forward;
        if (i == cur) score -= kPressHysteresis;
        if (score < best_score) { best_score = score; best = i; }
    }
    wm.presser_id = best;
}

}  // namespace simuro5
