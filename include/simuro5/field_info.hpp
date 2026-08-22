// ============================================================
// field_info.hpp — 场地常量与区域判断
// 数值来源：官方规则 PDF「6.2 场地说明」+ 官方 demo 代码
//
//  场地 220×180 cm；球门宽 40cm，位于球门线中点 y=90 两侧 ±20
//  门区(小禁区 A)   50×15 cm（球门前）
//  罚球区(大禁区)   80×35 cm（A+B 区）
//  中圈半径 25cm；点球判罚区 120×80cm；禁止推球区(角落黄色区)
// ============================================================
#ifndef SIMURO5_FIELD_INFO_HPP
#define SIMURO5_FIELD_INFO_HPP

#include "simuro5/team.hpp"
#include "simuro5/geometry.hpp"

namespace simuro5 {

// 球门 y 范围（门线在球门前，宽 40cm => y ∈ [70, 110]）
inline double goal_y_low()  { return 90.0 - TeamContext::GOAL_WIDTH / 2.0; }  // 70
inline double goal_y_high() { return 90.0 + TeamContext::GOAL_WIDTH / 2.0; }  // 110

// 门区（小禁区 A）：球门前 50×15
//   - 蓝队球门在 x=220：x ∈ [220-50, 220], y ∈ [90-15, 90+15]
//   - 黄队球门在 x=0：  x ∈ [0, 50],      y ∈ [75, 105]
inline bool in_goal_area(const TeamContext &ctx, double x, double y) {
    double gx = ctx.our_goal_x();
    double x_lo = std::min(gx, gx + ctx.attack_dir() * 50.0);
    double x_hi = std::max(gx, gx + ctx.attack_dir() * 50.0);
    return in_rect(x, y, x_lo, x_hi, 75.0, 105.0);
}

// 罚球区（大禁区 A+B）：球门前 80×35
inline bool in_penalty_area(const TeamContext &ctx, double x, double y) {
    double gx = ctx.our_goal_x();
    double x_lo = std::min(gx, gx + ctx.attack_dir() * 80.0);
    double x_hi = std::max(gx, gx + ctx.attack_dir() * 80.0);
    return in_rect(x, y, x_lo, x_hi, 72.5, 107.5);
}

// 守门员活动区域 = 己方罚球区（大禁区 80×35）。
//   规则：守门员出「球门区(小禁区)」即不受保护；出罚球区则离门太远、易被过。
//   所以守门员出击/拦截点一律 clamp 回罚球区内，保证不越界（策略约束，非硬规则）。
inline void clamp_goalie_area(const TeamContext &ctx, double &x, double &y) {
    double gx = ctx.our_goal_x();
    double x_lo = std::min(gx, gx + ctx.attack_dir() * 80.0);
    double x_hi = std::max(gx, gx + ctx.attack_dir() * 80.0);
    x = clamp(x, x_lo, x_hi);
    y = clamp(y, 72.5, 107.5);
}

// 罚球区前缘 x：守门员可活动的最靠外一条线（球门前 80cm）
inline double penalty_front_x(const TeamContext &ctx) {
    return ctx.our_goal_x() + ctx.attack_dir() * 80.0;
}

// 对方罚球区（进攻方禁区内不能久留/越位参考）
inline bool in_opp_penalty_area(const TeamContext &ctx, double x, double y) {
    TeamContext mirror = ctx; mirror.is_blue = !ctx.is_blue;
    return in_penalty_area(mirror, x, y);
}

// 是否在场地内
inline bool in_field(double x, double y) {
    return in_rect(x, y, 0.0, TeamContext::FIELD_LENGTH, 0.0, TeamContext::FIELD_WIDTH);
}

// 球是否整体越过我方门线（进球判定，含门柱范围）
inline bool is_ball_in_our_goal(const TeamContext &ctx, double x, double y) {
    double gx = ctx.our_goal_x();
    bool beyond = ctx.attack_dir() > 0 ? (x < gx) : (x > gx);
    return beyond && y >= goal_y_low() && y <= goal_y_high();
}

// 球是否整体越过对方门线
inline bool is_ball_in_opp_goal(const TeamContext &ctx, double x, double y) {
    TeamContext mirror = ctx; mirror.is_blue = !ctx.is_blue;
    return is_ball_in_our_goal(mirror, x, y);
}

}  // namespace simuro5
#endif
