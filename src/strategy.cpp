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

void Strategy::run(WorldModel &wm) {
    Situation sit = sit_.analyze(wm);
    wm.we_have_ball = sit.we_have_ball;
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
        if (kZoneMode > 0.5 && wm.live_play && !wm.in_penalty_exec && wm.role[i] != ROLE_ACTIVE) {
            run_zone(wm, i); continue;
        }
        if (i == wm.presser_id) { run_press(wm, i); continue; }
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

void Strategy::update_team_state(WorldModel &wm) {
    if (wm.we_have_ball) { ++wm.possession_frames; wm.no_possession_frames = 0; }
    else                 { ++wm.no_possession_frames; wm.possession_frames = 0; }

    if (wm.we_have_ball && !wm.prev_we_have_ball && wm.game_state == PM_PlayOn) {
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

void Strategy::update_presser(WorldModel &wm) {
    int cur = wm.presser_id;
    wm.presser_id = -1;

    if (kPressEnabled <= 0.0) return;
    if (!wm.ball.valid) return;
    if (!wm.live_play) return;
    if (wm.we_have_ball) return;

    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;

    bool front = (ctx.attack_dir() > 0) ? (bx > 110.0) : (bx < 110.0);
    if (!front) return;

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
