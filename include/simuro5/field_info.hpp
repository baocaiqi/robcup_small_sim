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

// 对方门区（小禁区 50×15）：FIRA 规则——进攻方在对方门区 2+ 人或单人停留>20周期 → 罚点球。
// 真实比赛我方场均被罚 1.9 个点球（sim_bench 复现：12.9 次违规/场），
// 非持球进攻者的站位必须避开对方门区。
inline bool in_opp_goal_area(const TeamContext &ctx, double x, double y) {
    // 对方门区 y 范围 = 门宽 70~110 外扩 7.5 = [62.5,117.5]，比 in_goal_area（我方门区防守用 [75,105]）宽；
    // 不能复用 in_goal_area 镜像，否则 y∈[62.5,75)∪(105,117.5] 的边缘闯入不会被判罚。
    double ogx = ctx.opp_goal_x();
    double ad = ctx.attack_dir();
    double x_lo = std::min(ogx, ogx - ad * 50.0);
    double x_hi = std::max(ogx, ogx - ad * 50.0);
    return in_rect(x, y, x_lo, x_hi, 62.5, 117.5);
}

// 对方门区前缘推荐站位：x = 门区前缘外 5cm（朝场内），y = 夹回对方门区 y 域 [62.5,117.5]。
// 供 clamp_out_opp_goal_area 与 run_passive 建议B 复用，消除魔法数字 55/62.5/117.5。
inline void opp_goal_area_front(const TeamContext &ctx, double y_ref, double &x, double &y) {
    x = ctx.opp_goal_x() - ctx.attack_dir() * 55.0;
    y = clamp(y_ref, 62.5, 117.5);
}

// 对方门区禁入约束：站位点落入对方门区时，沿 x 推到门区前缘外 5cm（y 夹回门区 y 域）。
// 注意：ACTIVE 带球/射门不调用本函数——单人压门抢射是正常进攻，
// 规则只罚"2+ 人聚集"和"单人停留>20 帧"。
inline void clamp_out_opp_goal_area(const TeamContext &ctx, double &x, double &y) {
    if (!in_opp_goal_area(ctx, x, y)) return;
    double fx = 0.0, fy = 0.0;
    opp_goal_area_front(ctx, y, fx, fy);   // x 推出前缘外 5cm，y 夹回判定域（当前为恒等，保留语义兜底）
    x = fx; y = fy;
}

// ============================================================
// 禁止推球区（四角黄色区域，规则 7.2.6）
//   任何队伍 2 个以上机器人在禁止推球区推球 → 犯规；
//   每半场每 4 次犯规 → 对手 +1 进球；犯规判争球。
//   位置：docs/00 实测记录为「场地四角黄色区域」；官方规则图 4 显示
//   黄色区域位于场地两端（8cm 标注，避开禁区高度带）。
//   实现按「四角矩形」可配置，尺寸 D6 真平台校准后只需改 CFG_CORNER_PUSH_SIZE。
// ============================================================
constexpr double CFG_CORNER_PUSH_SIZE = 8.0;   // 禁推区向场内尺寸 cm（初值：图 4 的 8cm 标注）

inline bool in_forbidden_push_area(double x, double y) {
    const double s = CFG_CORNER_PUSH_SIZE;
    const bool corner_x = (x <= s) || (x >= TeamContext::FIELD_LENGTH - s);
    const bool corner_y = (y <= s) || (y >= TeamContext::FIELD_WIDTH - s);
    return corner_x && corner_y;
}

// 点落入禁推区 → 推到边缘外 2cm（带出角区，避免 2+ 人在角里推球犯规）
inline void clamp_out_forbidden_push_area(double &x, double &y) {
    if (!in_forbidden_push_area(x, y)) return;
    const double s = CFG_CORNER_PUSH_SIZE;
    const double out = s + 2.0;
    if (x <= s) x = out;
    else if (x >= TeamContext::FIELD_LENGTH - s) x = TeamContext::FIELD_LENGTH - out;
    if (y <= s) y = out;
    else if (y >= TeamContext::FIELD_WIDTH - s) y = TeamContext::FIELD_WIDTH - out;
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
