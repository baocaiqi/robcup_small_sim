#include "simuro5/world_model.hpp"
#include <cmath>

namespace simuro5 {

void WorldModel::update(const Environment *env, const TeamContext &ctx_) {
    ctx = ctx_;
    game_state = (int)env->gameState;
    whos_ball = env->whosBall;
    field = env->fieldBounds;
    goal = env->goalBounds;

    // 球（三帧）
    ball_last.x = env->lastBall.pos.x;      ball_last.y = env->lastBall.pos.y;
    ball_last.valid = true;
    ball.x = env->currentBall.pos.x;        ball.y = env->currentBall.pos.y;
    ball.vx = ball.x - ball_last.x;         ball.vy = ball.y - ball_last.y;
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
