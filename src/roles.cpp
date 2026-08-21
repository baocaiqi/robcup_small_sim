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

    // ============================================================
    // 可调参数（改动后记得同步 docs/06-调参记录.md）
    // ============================================================
    // 门前站位：球门中心前方 10cm；y 跟踪范围 [76,104]（门宽内侧留余量）
    const double kGuardDist = 10.0;
    const double kTrackYLo  = 76.0, kTrackYHi = 104.0;
    // 出击判定阈值：
    //   kMaxTTA   ：预测球到门线的时间上限(帧)。超过它=远期威胁，不出击。
    //   kMaxReach ：守门员出击能「够得着」球的距离上限(cm)。
    //   kMinSpeed ：球速下限(cm/帧)。太慢的球不用出击，等它滚到门前再拿。
    const double kMaxTTA   = 15.0;
    const double kMaxReach = 30.0;
    const double kMinSpeed = 5.0;

    double gx = ctx.our_goal_x() + ctx.attack_dir() * kGuardDist;
    double bx = wm.ball.x, by = wm.ball.y;
    double vx = wm.ball.vx, vy = wm.ball.vy;
    double speed = ball_speed(vx, vy);
    double db = dist(r.x, r.y, bx, by);      // 守门员到球的当前距离

    // ============================================================
    // 出击预判（最难的点）：用球速 (vx,vy) 做三角函数外推，
    // 判断「球会不会进门」+「多久到门」，再决定要不要离开门线出击。
    //
    // 出击太早 → 守门员离开门前，对方一横传/变向就是空门；
    // 出击太晚 → 球滚到门线才动，扑不到。
    // 所以只有当球「真的会进球」且「马上到门」才出击，否则守门不动。
    // ============================================================
    double y_at_goal = 90.0;
    // 球会不会到达门线：predict_y_at_x 返回 false = 球背离门或只有 y 向运动
    bool heading_goal = predict_y_at_x(bx, by, vx, vy, ctx.our_goal_x(), y_at_goal);
    // 到达门线时 y 落在门宽内 → 这球会进球（不是偏出/打墙）
    bool on_target = heading_goal &&
                     y_at_goal >= goal_y_low() && y_at_goal <= goal_y_high();
    // 到门线时间 TTA（帧）：距离 / 朝门速度分量
    double tta = 1e9;
    if (std::fabs(vx) > 1e-9)
        tta = std::fabs(ctx.our_goal_x() - bx) / std::fabs(vx);

    bool should_attack = on_target && tta < kMaxTTA && db < kMaxReach && speed > kMinSpeed;

    if (should_attack) {
        // 出击：扑向「球会到达的点」（门线内侧一点），而不是追球当前位置。
        //   这样封的是射门路线，不会跟在球屁股后面被溜。
        double aim_x = ctx.our_goal_x() + ctx.attack_dir() * 3.0;   // 门线前 3cm
        motion::position(r, aim_x, clamp(y_at_goal, 74.0, 106.0));
    } else if (on_target && tta < kMaxTTA) {
        // 会进球但还够不着球：提前站到预测入球 y，把门封死。
        motion::position(r, gx, clamp(y_at_goal, kTrackYLo, kTrackYHi));
    } else {
        // 无威胁 / 球慢：常规门前站位，y 跟球（夹在门区内）。
        motion::position(r, gx, clamp(by, kTrackYLo, kTrackYHi));
    }
}

void run_active(WorldModel &wm, int id) {
    RobotState &r = wm.home[id];
    const TeamContext &ctx = wm.ctx;

    ShootPlan sp = plan_shoot(wm, id);
    if (sp.viable) { motion::position(r, sp.target_x, sp.target_y); return; }

    PassPlan pp = plan_pass(wm, id);
    if (pp.viable) { motion::position(r, pp.target_x, pp.target_y); return; }

    double db = dist(r.x, r.y, wm.ball.x, wm.ball.y);
    // 找离球最近的对方防守者（含守门员）：决定带球开口侧 + 判断球权是否在我
    double opp_d = 1e9, opp_y = 90.0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y);
        if (d < opp_d) { opp_d = d; opp_y = wm.opp[i].y; }
    }

    if (db < 15.0 && db < opp_d) {
        // —— 带球推进：球在我方脚下控制范围（我方是离球最近的人）——
        // 目标 = 球后方 8cm 推球点，方向指向门柱开口；
        // 开口选「离最近防守者远」的一侧，避免直线撞进防守怀里。
        double ogx = ctx.opp_goal_x();
        double aim_y = (opp_y > 90.0) ? 106.0 : 74.0;   // 防守者偏下 → 带上柱口
        double dirx = (ogx + ctx.attack_dir() * 5.0) - wm.ball.x;
        double diry = aim_y - wm.ball.y;
        double len = std::hypot(dirx, diry);
        if (len > 1e-6) { dirx /= len; diry /= len; }
        // 球后方 8cm 推球点（和 shoot 同款推球机制）
        motion::position(r, wm.ball.x - dirx * 8.0, wm.ball.y - diry * 8.0);
    } else {
        // 球不在脚下 / 争抢中：追预测球位（带减速防冲过头）
        motion::chase_ball(r, wm.ball_pred);
    }
}

void run_passive(WorldModel &wm, int id) {
    DefensePlan dp = plan_defense(wm, id);
    motion::position(wm.home[id], dp.target_x, dp.target_y);
}

void run_assist(WorldModel &wm, int id) {
    // 威胁高：回防但分散站位（封上侧射门线，不与 passive 挤一点）
    if (wm.threat_level > 0.3) {
        DefensePlan dp = plan_defense(wm, id);
        double ty = clamp(dp.target_y + 30.0, 20.0, 160.0);
        motion::position(wm.home[id], dp.target_x, ty);
        return;
    }
    motion::position(wm.home[id], wm.assist_x, wm.assist_y);
}

void run_midfield(WorldModel &wm, int id) {
    // 威胁高：回防但分散站位（封下侧射门线）
    if (wm.threat_level > 0.3) {
        DefensePlan dp = plan_defense(wm, id);
        double ty = clamp(dp.target_y - 30.0, 20.0, 160.0);
        motion::position(wm.home[id], dp.target_x, ty);
        return;
    }
    motion::position(wm.home[id], wm.mid_x, wm.mid_y);
}

}  // namespace simuro5
