#include "simuro5/world_model.hpp"
#include <cmath>

namespace simuro5 {

const char *coop_outcome_name(CoopOutcome result) {
    static const char *names[] = {"success", "prepare_timeout", "receive_timeout", "phase_interrupted",
        "invalid_ball", "penalty", "high_threat", "corner", "intercepted", "emergency_defense",
        "incoming_shot", "invalid_target", "goal_discipline", "receiver_marked", "lane_blocked",
        "push_forbidden", "dead_ball", "degenerate_target", "prep_point", "match_end",
        "loose_ball", "teammate_takeover", "carry_point", "opponent_first"};
    static_assert(sizeof(names) / sizeof(names[0]) == (int)CoopOutcome::Count, "统计原因表不完整");
    return names[(int)result];
}
void WorldModel::coop_created() {
    ++coop_stats.created;
    if (coop_observer) coop_observer(*this, "created", CoopOutcome::Count);
}
void WorldModel::coop_released() {
    ++coop_stats.released;
    if (coop_observer) coop_observer(*this, "released", CoopOutcome::Count);
    if (coop_pass_task.kind == PassTaskKind::Ordinary)
        coop_pass_task.clear_push_target_lock();
}
void WorldModel::coop_finish(CoopOutcome result) {
    if (!coop_pass_task.active) return;
    ++coop_stats.outcomes[(int)result];
    if (result == CoopOutcome::Success) ++coop_stats.received;
    if (coop_observer) coop_observer(*this, "finished", result);
    if (coop_pass_task.kind == PassTaskKind::Ordinary)
        coop_pass_task.clear_push_target_lock();
    coop_pass_task.active = false;
}
void WorldModel::coop_control_entered() {
    ++coop_stats.control_entered;
    if (coop_observer) coop_observer(*this, "control_entered", CoopOutcome::Count);
}
void WorldModel::coop_control_end(CoopOutcome reason) {
    if (!coop_ball_control.active) return;
    ++coop_stats.control_exits[(int)reason];
    if (coop_observer) coop_observer(*this, "control_exited", reason);
    coop_ball_control.active = false;
}

void WorldModel::update(const Environment *env, const TeamContext &ctx_) {
    ctx = ctx_;
    game_state_last = game_state;
    game_state = (int)env->gameState;
    whos_ball = env->whosBall;
    field = env->fieldBounds;
    goal = env->goalBounds;

    ball_last.x = env->lastBall.pos.x;      ball_last.y = env->lastBall.pos.y;
    ball_last.valid = true;
    ball.x = env->currentBall.pos.x;        ball.y = env->currentBall.pos.y;
    // 球速跳变滤波：进球/定位球重置会让球位瞬间跳变，单帧位移 >30cm/帧（正常 ≤22）判复位、速度清零
    const double kMaxBallVel = 30.0;
    {
        double bvx = ball.x - ball_last.x, bvy = ball.y - ball_last.y;
        ball.vx = (std::fabs(bvx) > kMaxBallVel) ? 0.0 : bvx;
        ball.vy = (std::fabs(bvy) > kMaxBallVel) ? 0.0 : bvy;
    }
    ball.valid = true;
    // 活球判定：球离开重启摆放点 6cm 即算开球
    {
        const double kLiveMoveDist = 6.0;
        const bool jump = std::fabs(ball.x - ball_last.x) > kMaxBallVel ||
                          std::fabs(ball.y - ball_last.y) > kMaxBallVel;
        if (game_state == PM_PlayOn) {
            runtime_phase = RuntimePhase::Running; restart_armed = false;
        } else if (!restart_armed || jump || game_state != game_state_last) {
            restart_armed = true; restart_x = ball.x; restart_y = ball.y;
            runtime_phase = RuntimePhase::RestartSetup;
        } else if (runtime_phase != RuntimePhase::Running &&
                   std::hypot(ball.x - restart_x, ball.y - restart_y) > kLiveMoveDist) {
            runtime_phase = RuntimePhase::Running;
        }
    }
    if (runtime_phase != RuntimePhase::Running) {
        coop_finish(CoopOutcome::PhaseInterrupted);
        coop_control_end(CoopOutcome::PhaseInterrupted);
    }
    ball_pred.x = env->predictedBall.pos.x; ball_pred.y = env->predictedBall.pos.y;
    ball_pred.valid = true;

    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        home[i].x = env->home[i].pos.x;
        home[i].y = env->home[i].pos.y;
        home[i].rot = env->home[i].rotation;
        home[i].vl = env->home[i].velocityLeft;
        home[i].vr = env->home[i].velocityRight;
    }

    // 对方无平台速度，需自差分；单帧位移 >8cm/帧 判为复位跳变、清零（机器人正常 ~2.5）
    const double kMaxOppVel = 8.0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        opp[i].x = env->opponent[i].pos.x;
        opp[i].y = env->opponent[i].pos.y;
        opp[i].rot = env->opponent[i].rotation;
        if (opp_vel_ready) {
            double dvx = opp[i].x - opp_last[i].x;
            double dvy = opp[i].y - opp_last[i].y;
            opp_vx[i] = (std::fabs(dvx) > kMaxOppVel) ? 0.0 : dvx;
            opp_vy[i] = (std::fabs(dvy) > kMaxOppVel) ? 0.0 : dvy;
        } else {
            opp_vx[i] = 0.0;
            opp_vy[i] = 0.0;
        }
        opp_last[i] = opp[i];
    }
    opp_vel_ready = true;
}

}  // namespace simuro5
