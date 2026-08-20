// ============================================================
// defense.cpp — 区域防守：断球点计算（队员 D 负责）
//
// 见 defense.hpp 头注释：核心是用球速 (vx, vy) 的方向角做三角外推，
// 把断球点从「球-门连线静态点」升级成「球真实运动轨迹上的点」。
// ============================================================
#include "simuro5/defense.hpp"
#include "simuro5/field_info.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

// ============================================================
// 可调参数（改动后记得同步 docs/06-调参记录.md）
// ============================================================

// 拦截线距球门线的距离(cm)：球门前方多远处开始断球。
//   越近 → 站位越靠门，堵门更稳，但断球更晚、留给反应的时间更少；
//   越远 → 断球更早，但离门远、一旦被变向绕过就回不来。
//   默认 50cm（沿用旧版「球-门连线 50cm」的口径）。
static constexpr double kInterceptLineDist = 50.0;

// 球速阈值(cm/帧)：低于此值视为「球基本停着」，外推方向噪声大、
// 无断球价值，直接回退到静态球-门连线站位。
static constexpr double kMinBallSpeed = 3.0;

// 断球点夹取范围，防止站出场地或贴死边线。
static constexpr double kMinX = 12.0, kMaxX = 208.0;
static constexpr double kMinY = 15.0, kMaxY = 165.0;

// ============================================================
// 纯函数：球轨迹 ∩ 球门前拦截线
// ============================================================
bool intercept_point(const WorldModel &wm, double line_dist,
                     double &ix, double &iy) {
    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;
    double vx = wm.ball.vx, vy = wm.ball.vy;

    // 拦截线 x：球门线向场内退 line_dist。
    //   attack_dir() 指进攻方向（蓝队 -1，黄队 +1），
    //   所以「球门线 + attack_dir * line_dist」正好是向场内方向。
    //   蓝队：220 + (-1)*50 = 170；黄队：0 + (+1)*50 = 50。
    double line_x = ctx.our_goal_x() + ctx.attack_dir() * line_dist;

    // 三角函数外推：球沿 (vx,vy) 方向到达拦截线时的 y（见 defense.hpp）。
    double y_at_line = 0.0;
    if (!predict_y_at_x(bx, by, vx, vy, line_x, y_at_line)) {
        return false;   // vx≈0 或球背离球门 → 没有「朝门滚」的断球点
    }

    ix = clamp(line_x, kMinX, kMaxX);
    iy = clamp(y_at_line, kMinY, kMaxY);
    return true;
}

// ============================================================
// 主入口：2 号防守队员的断球点
// ============================================================
DefensePlan plan_defense(const WorldModel &wm) {
    DefensePlan plan;
    const TeamContext &ctx = wm.ctx;

    plan.ball_spd = ball_speed(wm.ball.vx, wm.ball.vy);

    // 球是否朝己方球门滚：
    //   用「速度 x 分量」与「球→门」的 x 方向是否同号判断。
    //   蓝队门在 +x 端(220)，球在门左侧，朝门 = vx>0；
    //   黄队门在 -x 端(0)，  球在门右侧，朝门 = vx<0。
    //   统一写成 vx * (goal_x - ball.x) > 0，蓝黄通用。
    double toward_goal = wm.ball.vx * (ctx.our_goal_x() - wm.ball.x);
    plan.approaching = toward_goal > 0.0;

    // 有威胁（朝门滚 + 球速够快）→ 用三角函数算真实断球点；
    // 否则（球慢 / 背离 / 无有效交点）→ 回退到 situation 给的
    // 球-门连线静态点 wm.passive_xy。
    double ix = 0.0, iy = 0.0;
    if (plan.approaching && plan.ball_spd >= kMinBallSpeed &&
        intercept_point(wm, kInterceptLineDist, ix, iy)) {
        plan.target_x = ix;
        plan.target_y = iy;
    } else {
        plan.target_x = wm.passive_x;
        plan.target_y = wm.passive_y;
    }

    // 规则红线：断球点不得进入己方门区（只有守门员能进）。
    //   蓝队门区 x∈[170,220]；拦截线本身 x=170 正好压门区前缘，
    //   若 y 也落在门宽范围内则再往外推 5cm，避免踩线犯规。
    if (in_goal_area(ctx, plan.target_x, plan.target_y)) {
        plan.target_x = clamp(ctx.our_goal_x() + ctx.attack_dir() * 55.0, kMinX, kMaxX);
        plan.target_y = clamp(wm.ball.y, 75.0, 105.0);
    }
    return plan;
}

}  // namespace simuro5
