// field_info.hpp — 场地常量与区域判断；场地 220×180cm，原点左下角，蓝队守 x=220 右门
// 球门宽 40cm（y∈[70,110]）；门区 50×15cm；罚球区 80×35cm；中圈 R25cm
#ifndef SIMURO5_FIELD_INFO_HPP
#define SIMURO5_FIELD_INFO_HPP

#include "simuro5/team.hpp"
#include "simuro5/geometry.hpp"

namespace simuro5 {

// 球撞边墙实测系数：法向恢复 0.66、切向保持 0.81 ⇒ 入射角≠反射角，不能用理想镜面
inline double ball_wall_rest() { return 0.66; }
inline double ball_wall_fric() { return 0.81; }

// 球门 y 范围（门线宽 40cm ⇒ y∈[70,110]）
inline double goal_y_low()  { return 90.0 - TeamContext::GOAL_WIDTH / 2.0; }
inline double goal_y_high() { return 90.0 + TeamContext::GOAL_WIDTH / 2.0; }

// 门区（小禁区 A）：球门前 50×15cm，y∈[75,105]
inline bool in_goal_area(const TeamContext &ctx, double x, double y) {
    double gx = ctx.our_goal_x();
    double x_lo = std::min(gx, gx + ctx.attack_dir() * 50.0);
    double x_hi = std::max(gx, gx + ctx.attack_dir() * 50.0);
    return in_rect(x, y, x_lo, x_hi, 75.0, 105.0);
}

// 我方门区的裁判口径：门线内 15cm（y∈[65,115]）+ 门内 15cm（y∈[70,110]）
// 比 in_goal_area（策略口径 50×30）浅而宽；非门将踩进来即计数且离开不清零
inline bool in_goal_area_rule(const TeamContext &ctx, double x, double y,
                              double margin_x = 0.0, double margin_y = 0.0) {
    double d = (x - ctx.our_goal_x()) * ctx.attack_dir();    // 离门线距离（场内为正，门后为负）
    if (d < -15.0 || d > 15.0 + margin_x) return false;
    double half = (d < 0.0) ? 20.0 : 25.0;
    return std::fabs(y - 90.0) <= half + margin_y;
}

// 罚球区（大禁区 A+B）：球门前 80×35cm，y∈[72.5,107.5]
inline bool in_penalty_area(const TeamContext &ctx, double x, double y) {
    double gx = ctx.our_goal_x();
    double x_lo = std::min(gx, gx + ctx.attack_dir() * 80.0);
    double x_hi = std::max(gx, gx + ctx.attack_dir() * 80.0);
    return in_rect(x, y, x_lo, x_hi, 72.5, 107.5);
}

// 守门员活动区 = 己方罚球区（大禁区 80×35）；出击/拦截点一律夹回区内
inline void clamp_goalie_area(const TeamContext &ctx, double &x, double &y) {
    double gx = ctx.our_goal_x();
    double x_lo = std::min(gx, gx + ctx.attack_dir() * 80.0);
    double x_hi = std::max(gx, gx + ctx.attack_dir() * 80.0);
    x = clamp(x, x_lo, x_hi);
    y = clamp(y, 72.5, 107.5);
}

// 对方罚球区（进攻方禁区内不能久留）
inline bool in_opp_penalty_area(const TeamContext &ctx, double x, double y) {
    TeamContext mirror = ctx; mirror.is_blue = !ctx.is_blue;
    return in_penalty_area(mirror, x, y);
}

// 对方门区（小禁区 50×15）：进攻方 2+ 人或单人停留>20 周期 → 判点球
inline bool in_opp_goal_area(const TeamContext &ctx, double x, double y) {
    // y 域比 in_goal_area 宽（62.5~117.5），不能复用其镜像，否则边缘闯入不判罚
    double ogx = ctx.opp_goal_x();
    double ad = ctx.attack_dir();
    double x_lo = std::min(ogx, ogx - ad * 50.0);
    double x_hi = std::max(ogx, ogx - ad * 50.0);
    return in_rect(x, y, x_lo, x_hi, 62.5, 117.5);
}

// 对方门区前缘站位：x 推到前缘外 15cm（朝场内），y 夹回 [62.5,117.5]
// 供 clamp_out_opp_goal_area 与 run_passive 复用（消除魔法数字 65/62.5/117.5）
inline void opp_goal_area_front(const TeamContext &ctx, double y_ref, double &x, double &y) {
    x = ctx.opp_goal_x() - ctx.attack_dir() * 65.0;
    y = clamp(y_ref, 62.5, 117.5);
}
// 注意：ACTIVE 带球/射门不调用本函数——单人压门抢射是正常进攻，只罚 2+ 人聚集或停留>20 帧
inline void clamp_out_opp_goal_area(const TeamContext &ctx, double &x, double &y) {
    if (!in_opp_goal_area(ctx, x, y)) return;
    double fx = 0.0, fy = 0.0;
    opp_goal_area_front(ctx, y, fx, fy);
    x = fx; y = fy;
}

// 禁止推球区（角落黄区）：距角 kCornerNoPushR 内推球=犯规（每 4 次 +1 球 + 争球）
// 规则未给黄区尺寸，保守取 35cm：宁可少救一次角球，也不吃犯规
constexpr double kCornerNoPushR = 35.0;   // cm：距任一角点 < 此值 → 禁止任何推球动作

// 到最近角点的距离 cm（场地 220×180 四角）
inline double dist_to_corner(double x, double y) {
    double dx = std::min(x, TeamContext::FIELD_LENGTH - x);
    double dy = std::min(y, TeamContext::FIELD_WIDTH - y);
    if (dx < 0.0) dx = 0.0;      // 场外按 0 处理 → 视为贴角
    if (dy < 0.0) dy = 0.0;
    return std::hypot(dx, dy);
}

// 是否在禁止推球的角区（球或绕球后的准备点落在这里 → 不许推）
inline bool in_no_push_zone(double x, double y) {
    return dist_to_corner(x, y) < kCornerNoPushR;
}

}  // namespace simuro5
#endif
