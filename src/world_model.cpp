#include "simuro5/world_model.hpp"
#include <cmath>

namespace simuro5 {

const char *coop_outcome_name(CoopOutcome result) {
    static const char *names[] = {"success", "prepare_timeout", "receive_timeout", "game_state",
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
}
void WorldModel::coop_finish(CoopOutcome result) {
    if (!coop_pass_task.active) return; // 多个纪律闸门同帧触发也只结算一次。
    ++coop_stats.outcomes[(int)result];
    if (result == CoopOutcome::Success) ++coop_stats.received;
    if (coop_observer) coop_observer(*this, "finished", result);
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
    game_state_last = game_state;        // 上一帧 PlayMode（点球执行期识别用）
    game_state = (int)env->gameState;
    if (game_state != game_state_last) {
        coop_finish(CoopOutcome::GameState);
        coop_control_end(CoopOutcome::GameState);
    }
    whos_ball = env->whosBall;
    field = env->fieldBounds;
    goal = env->goalBounds;

    // 球（三帧）
    ball_last.x = env->lastBall.pos.x;      ball_last.y = env->lastBall.pos.y;
    ball_last.valid = true;
    ball.x = env->currentBall.pos.x;        ball.y = env->currentBall.pos.y;
    // 球速跳变滤波：进球/FreeBall/定位球重置时球位瞬间跳变（实测 227→110 = -117cm/帧），
    //   直接差分会产出虚假巨大速度 → 死球判定失效 + 外推/预测方向误导
    //   （真实 9/2 丢球 2：重置后 ACTIVE 追错方向 40 帧脱位）。与对方机器人同款处理：
    //   单帧位移超上限视为复位跳变、速度清零。
    //   ⚠️ 阈值教训（0:3 惨败）：初版 12cm/帧 太激进——demo 传球/解围/射门球速常达
    //   20+cm/帧（sim 实测 22），被清零后 run_active "对方门球等待"误触发（球已高速
    //   开出但 vx=0 → 永远等球"动起来"）→ ACTIVE 不追球、进攻瘫痪（射门威胁 0.4%）。
    //   30cm/帧 滤掉重置跳变（100+），保留正常球速（≤22）。
    const double kMaxBallVel = 30.0;
    {
        double bvx = ball.x - ball_last.x, bvy = ball.y - ball_last.y;
        ball.vx = (std::fabs(bvx) > kMaxBallVel) ? 0.0 : bvx;
        ball.vy = (std::fabs(bvy) > kMaxBallVel) ? 0.0 : bvy;
    }
    ball.valid = true;
    ball_pred.x = env->predictedBall.pos.x; ball_pred.y = env->predictedBall.pos.y;
    ball_pred.valid = true;

    // 己方
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        home[i].x = env->home[i].pos.x;
        home[i].y = env->home[i].pos.y;
        home[i].rot = env->home[i].rotation;
        home[i].vl = env->home[i].velocityLeft;
        home[i].vr = env->home[i].velocityRight;
    }

    // 对方（对手无平台预测速度，需自己差分；单帧位移超上限判为复位跳变、速度清零）
    const double kMaxOppVel = 8.0;   // cm/帧：机器人正常 ~2.5，瞬移 50+，8 安全分隔
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
