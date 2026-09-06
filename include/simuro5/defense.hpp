// ============================================================
// defense.hpp — 区域防守：断球点计算工具箱（队员 D 负责）
//
// 核心思路（用三角函数算断球点）：
//   球在场上近似匀速直线运动，其运动方向角 θ = atan2(vy, vx)，
//   斜率 k = vy/vx = tan(θ)。防守队员不再站「球-己方球门连线」上
//   那个静态点，而是站到球【真实运动轨迹】上：沿 θ 方向外推，
//   求轨迹与「球门前拦截线」的交点 —— 提前占住这条轨迹，球自己
//   滚过来被断。
//
//   对比旧版「球-门连线 50cm 点」：
//     · 旧版只堵门（防正面射门），球斜着滚时站位会偏，断不到；
//     · 新版沿真实运动方向站位，斜向球也能正对轨迹断下。
//
// 本文件同时提供球轨迹外推工具，供守门员 run_goalie 的「出击预判」
// 复用（见 roles.cpp）。
// ============================================================
#ifndef SIMURO5_DEFENSE_HPP
#define SIMURO5_DEFENSE_HPP

#include "simuro5/world_model.hpp"
#include "simuro5/geometry.hpp"

namespace simuro5 {

// ============================================================
// 球轨迹外推工具（纯函数，可独立单测）
// ============================================================

// 球速大小（cm/帧 量级；速度由 world_model 用「本帧-上帧」差分得到）
inline double ball_speed(double vx, double vy) {
    return std::hypot(vx, vy);
}

// 球朝「己方球门心」方向的速度分量（cm/帧）。
//   点积投影：把球速向量投到「球→门心」方向上，等价 |v|·cos(θ)，
//   θ = 速度方向与门方向的夹角。朝门=正（全速），横滚≈0，背离门→负值截成 0。
//   与 ball_speed（原始速率）的区别：只看「朝门」这个方向有没有威胁，
//   横着滚 / 往对方门滚的球即使很快，也不构成我方门前威胁。
//   供守门员站位深度 + 持球者威胁使用。
inline double ball_danger_speed(const WorldModel &wm) {
    double gx = wm.ctx.our_goal_x();
    double dx = gx - wm.ball.x, dy = 90.0 - wm.ball.y;   // 球 → 门心 方向向量
    double d = std::hypot(dx, dy);
    if (d < 1e-6) return 0.0;
    return std::max(0.0, (wm.ball.vx * dx + wm.ball.vy * dy) / d);
}

// 球朝「某点 (px, py)」方向的速度分量（cm/帧）：判断球是否正冲向该点。
//   人盯人专用：球正朝某球员飞（快传/直塞）→ 该球员即将接球，威胁上升。
//   与 ball_danger_speed 的区别：投影基准是「球→该球员」，不是「球→门」。
inline double ball_approach_speed(const WorldModel &wm, double px, double py) {
    double dx = px - wm.ball.x, dy = py - wm.ball.y;     // 球 → 该球员 方向向量
    double d = std::hypot(dx, dy);
    if (d < 1e-6) return 0.0;
    return std::max(0.0, (wm.ball.vx * dx + wm.ball.vy * dy) / d);
}

// 球从 (x, y) 以速度 (vx, vy) 匀速直线运动，预测它到达
// 「竖线 x = target_x」时的 y 坐标。
//
// 三角函数原理（写在这里是为了把公式和直觉对上）：
//   运动方向角 θ = atan2(vy, vx) —— 这是唯一用到三角函数的地方；
//   到达 target_x 需要的时间 t = (target_x - x) / vx；
//   此时 y = y + vy * t = y + k * (target_x - x)，其中 k = vy/vx = tan(θ)。
//
// 代码里直接用代数形式（等价于 tan θ 的斜率），比「先 atan2 求角、
// 再 tan 反算」少一次往返、也避免 θ 接近 ±90° 时的数值问题。
//
// 返回 false（没有有效交点）：
//   - |vx| ≈ 0：球几乎只沿 y 方向滚，永远到不了这条竖线；
//   - t < 0：球背离 target_x 运动（时间倒流，说明球不是朝这个方向来）。
inline bool predict_y_at_x(double x, double y, double vx, double vy,
                           double target_x, double &out_y) {
    if (std::fabs(vx) < 1e-9) return false;
    double t = (target_x - x) / vx;
    if (t < 0.0) return false;
    out_y = y + vy * t;
    return true;
}

// 带边墙反弹的轨迹预测：同 predict_y_at_x，但球中途撞 y=0 / y=180 边墙时
//   按弹性反射（法向分量反号、切向不变）折返，预测反射后到达 target_x 时的 y。
//   .rlg 实测：撞墙后法向分量反号（近似弹性、略有衰减），切向基本不变。
//   只处理一次 y 墙反射；多次反射概率低、且断球点本就该保守回退，不做。
//   供区域防守断球点 intercept_point 用——球朝边线滚时直线外推会算出界，
//   实际球会弹回来，按反射后轨迹站位才断得到。
inline bool predict_y_at_x_reflect(double x, double y, double vx, double vy,
                                   double target_x, double &out_y) {
    if (std::fabs(vx) < 1e-9) return false;
    double t_total = (target_x - x) / vx;
    if (t_total < 0.0) return false;
    double y_end = y + vy * t_total;              // 直线终点
    if (y_end >= 0.0 && y_end <= 180.0) {         // 不撞边墙：同直线
        out_y = y_end;
        return true;
    }
    // 撞 y 墙：反射折返。y_end 出界 ⇒ vy 必非 0（否则 y_end≈y 在界内）
    double wall_y = (y_end < 0.0) ? 0.0 : 180.0;
    double t_wall = (wall_y - y) / vy;            // 到达墙的时间
    double t_rem  = t_total - t_wall;             // 反射后剩余时间（vx 不变）
    out_y = wall_y + (-vy) * t_rem;               // 反射后 vy' = -vy
    return true;
}

// 对方射门是否「在门框内且朝门」：球会到达己方门线且落点在门宽内。
//   复用 predict_y_at_x（匀速直线外推），供后卫抢反弹位 + 后续防补射用。
inline bool shot_on_target(const WorldModel &wm) {
    double y_at_goal = 90.0;
    if (!predict_y_at_x(wm.ball.x, wm.ball.y, wm.ball.vx, wm.ball.vy,
                        wm.ctx.our_goal_x(), y_at_goal))
        return false;
    const double half = TeamContext::GOAL_WIDTH / 2.0;   // 门宽 40 → 半宽 20，门范围 [70,110]
    return y_at_goal >= 90.0 - half && y_at_goal <= 90.0 + half;
}

// 门前抢反弹位：对方射门在门框内时，站位到罚球区前缘、预测入球点 y 上下两侧，
//   准备抢门将扑出/挡出的二次球（防补射）。
//   y_side：+30 上侧 / -30 下侧（与 assist/midfield 的 ±30 分散一致）。
inline void rebound_point(const WorldModel &wm, double y_side, double &x, double &y) {
    const TeamContext &ctx = wm.ctx;
    double y_at_goal = 90.0;
    predict_y_at_x(wm.ball.x, wm.ball.y, wm.ball.vx, wm.ball.vy,
                   ctx.our_goal_x(), y_at_goal);
    x = ctx.our_goal_x() + ctx.attack_dir() * 80.0;   // 罚球区前缘
    y = clamp(y_at_goal + y_side, 72.5, 107.5);
}

// 抢反弹位的最小朝门速度(cm/帧)：低于此视为慢球/带球，不抢反弹——
//   提前站过去浪费体力还留空档，只有真射门（快球朝门）才值得抢二次球。
inline double rebound_min_danger() { return 8.0; }

// ============================================================
// 防守计划
// ============================================================
struct DefensePlan {
    double target_x = 0, target_y = 90;   // 断球站位点（给 motion::position 用）
    bool   approaching = false;           // 球是否朝己方球门逼近（有预判价值）
    double ball_spd = 0;                  // 当前球速（cm/帧）
};

// 主入口：计算 2 号防守队员的断球点
// defender_id：防守队员在 home[] 中的下标（用于「可达性判断」——我赶不赶得上）
DefensePlan plan_defense(const WorldModel &wm, int defender_id);

// 球-门连线护门点（参考官方 demo CenterDefender 思想，自研实现）：
//   demo 中卫永远钉在「球与门之间」——球远站球后 45cm、球进门前站门前 25cm 线，
//   保证射门/冲锋路径上始终有人。这里按球距门分区给出护门站位：
//     · 球距门 >100cm：球-门连线、球向门方向 45cm（中远距拦截点）
//     · 球距门 45~100：门前 50cm 拦截线、y 跟球（压上断球）
//     · 球距门 <45   ：门前 20cm 线、y 跟球（堵射门角度，对应 demo 门前 25cm）
//   防守方进己方罚球区协防合法（规则只限制进攻方进对方门区），故不做禁区纪律；
//   只 clamp 场地边界。由 run_passive「门前协防」分支在对方逼近时调用。
bool goal_cover_point(const WorldModel &wm, double &out_x, double &out_y);

// 纯函数：断球点 = 球运动轨迹 ∩ 球门前 line_dist 处的拦截线
//   拦截线是与球门线平行、位于球门前 line_dist 处的竖线：
//     蓝队门 x=220 → 拦截线 x = 220 - 50 = 170（line_dist=50 时）
//     黄队门 x=0   → 拦截线 x = 0   + 50 = 50
// 返回 false 表示球没有朝门滚（无有效断球点），调用方应回退站位。
bool intercept_point(const WorldModel &wm, double line_dist,
                     double &ix, double &iy);

// ============================================================
// 人盯人（man-marking）：威胁打分 + 目标选择（队员 D 负责）
// ============================================================

// 盯人距离：防守队员贴到被盯球员多近(cm)。
//   太近(<8cm)会被判推球犯规，太远拦不住传/射。取 16cm 折中（原 12 实测超调到 8 犯规边）。
inline double mark_dist() { return 16.0; }

// 盯人预测帧数：用被盯者速度外推其未来位置再站位（速度前馈截击）。
//   同速追逐追不上移动目标，预测「几帧后会在哪」才能截住；太大易超调、太小追不上。
//   6→3（2026-08-26）：修正尺子后复盘实测 marker 平均离理想点 38~52cm「追不到」，
//   6 帧外推(≈15cm)过冲、目标点每帧跳，marker 永远追不上；降到 3 帧更稳。
inline double mark_lead() { return 3.0; }

// 堵传球线站位距离(cm)：被盯者是接球者（非持球者）且离球在此距离内 →
//   传球随时发生，marker 从 goal-side 换到「球→被盯者」连线，掐断传球。
//   下界 15cm 是「持球者」判定（离球 <15 视为正持球，堵射门而非传球）。
inline double mark_pass_lane_dist() { return 40.0; }

// 盯人危险门限：被盯者必须离球或离门足够近才值得贴，否则回区域防守。
//   复盘未贴住帧里 44~48% 被盯者离球 >40cm——追不危险的对手白费体力。
inline double mark_engage_ball_dist() { return 40.0; }
inline double mark_engage_goal_dist() { return 40.0; }

// 威胁打分（纯函数，可直接单测）：分越高越该被盯。
//   d_ball         ：该球员到球距离(cm)
//   d_goal         ：该球员到己方球门线距离(cm)
//   approach_speed ：球朝「该球员」的速度分量(cm/帧)——快传/直塞威胁
//   danger_speed   ：球朝「己方球门」的速度分量(cm/帧)——持球突破威胁
//   is_dribbler    ：该球员是否离球最近（持球者）
double mark_threat(double d_ball, double d_goal,
                   double approach_speed, double danger_speed, bool is_dribbler);

// 选出「进攻威胁最大」的对方球员下标(0~4)，含滞回 + 危险门限：
//   对 5 人打 mark_threat 取 argmax；若上一帧目标(current_target)仍在，
//   且新目标分数没超过它 10%，则继续盯旧目标——避免每帧换人原地转圈。
//   最后做危险门限：被盯者须离球或离门够近才值得贴，否则返回 -1（回区域防守）。
//   current_target：上一帧目标（-1=无）。返回新目标下标（-1=无人值得盯）。
int pick_mark_target(const WorldModel &wm, int current_target);

// 二抢一（双人夹击）站位：持球者带球推进到门前危险区时，为区域防守者
//   （assist/midfield 中非清道夫、离持球者更近者）算夹抢点。
//   defender_id：当前防守队员下标。返回 false=本轮不用夹抢（该防守者留在区域）。
//   返回 true 时 (out_x, out_y) 为夹抢站位点（已夹场地边界 + 禁区纪律）。
bool double_team_point(const WorldModel &wm, int defender_id,
                       double &out_x, double &out_y);

}  // namespace simuro5
#endif
