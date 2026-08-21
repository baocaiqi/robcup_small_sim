#include "simuro5/roles.hpp"
#include "simuro5/motion.hpp"
#include "simuro5/shoot.hpp"
#include "simuro5/pass.hpp"
#include "simuro5/defense.hpp"
#include "simuro5/field_info.hpp"
#include <cmath>

namespace simuro5 {

void run_goalie(WorldModel &wm, int id) {
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    // 点球：守门员锁定球门线中央（不出小禁区，防调度失位）
    if (wm.game_state == PM_PenaltyKick_Blue || wm.game_state == PM_PenaltyKick_Yellow) {
        motion::position(r, ctx.our_goal_x() + ctx.attack_dir() * 3.0, 90.0);
        return;
    }

    // 球门前 10cm，y 跟球（夹在门区内）
    double gx = ctx.our_goal_x() + ctx.attack_dir() * 10.0;
    double gy = clamp(wm.ball.y, 76.0, 104.0);

    // 球在我方球门前且很近 → 出击扑球
    double db = dist(r.x, r.y, wm.ball.x, wm.ball.y);
    if (db < 18.0 && wm.ball_our_goal_dist() < 45.0) {
        // 出击：冲向球，但目标 y 限制在球门范围附近（球门 70-110，留 2cm 扑球余量），
        // 防止球在边路时守门员追出球门被吊射空门
        double ty = clamp(wm.ball.y, 68.0, 112.0);
        motion::position(r, wm.ball.x, ty);
    } else {
        motion::position(r, gx, gy);
    }
}

void run_active(WorldModel &wm, int id) {
    RobotState &r = wm.home[id];

    ShootPlan sp = plan_shoot(wm, id);
    if (sp.viable) { motion::position(r, sp.target_x, sp.target_y); return; }

    PassPlan pp = plan_pass(wm, id);
    if (pp.viable) { motion::position(r, pp.target_x, pp.target_y); return; }

    motion::chase_ball(r, wm.ball_pred);
}

void run_passive(WorldModel &wm, int id) {
    DefensePlan dp = plan_defense(wm);
    motion::position(wm.home[id], dp.target_x, dp.target_y);
}

void run_assist(WorldModel &wm, int id) {
    motion::position(wm.home[id], wm.assist_x, wm.assist_y);
}

void run_midfield(WorldModel &wm, int id) {
    motion::position(wm.home[id], wm.mid_x, wm.mid_y);
}

}  // namespace simuro5
