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

    ShootPlan sp = plan_shoot(wm, id);
    if (sp.viable) { motion::position(r, sp.target_x, sp.target_y); return; }

    PassPlan pp = plan_pass(wm, id);
    if (pp.viable) { motion::position(r, pp.target_x, pp.target_y); return; }

    motion::chase_ball(r, wm.ball_pred);
}

void run_passive(WorldModel &wm, int id) {
    DefensePlan dp = plan_defense(wm, id);
    motion::position(wm.home[id], dp.target_x, dp.target_y);
}

void run_assist(WorldModel &wm, int id) {
    motion::position(wm.home[id], wm.assist_x, wm.assist_y);
}

void run_midfield(WorldModel &wm, int id) {
    motion::position(wm.home[id], wm.mid_x, wm.mid_y);
}

}  // namespace simuro5
