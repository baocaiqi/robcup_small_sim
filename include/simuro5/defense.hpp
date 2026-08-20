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

namespace simuro5 {

// ============================================================
// 球轨迹外推工具（纯函数，可独立单测）
// ============================================================

// 球速大小（cm/帧 量级；速度由 world_model 用「本帧-上帧」差分得到）
inline double ball_speed(double vx, double vy) {
    return std::hypot(vx, vy);
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

// ============================================================
// 防守计划
// ============================================================
struct DefensePlan {
    double target_x = 0, target_y = 90;   // 断球站位点（给 motion::position 用）
    bool   approaching = false;           // 球是否朝己方球门逼近（有预判价值）
    double ball_spd = 0;                  // 当前球速（cm/帧）
};

// 主入口：计算 2 号防守队员的断球点
DefensePlan plan_defense(const WorldModel &wm);

// 纯函数：断球点 = 球运动轨迹 ∩ 球门前 line_dist 处的拦截线
//   拦截线是与球门线平行、位于球门前 line_dist 处的竖线：
//     蓝队门 x=220 → 拦截线 x = 220 - 50 = 170（line_dist=50 时）
//     黄队门 x=0   → 拦截线 x = 0   + 50 = 50
// 返回 false 表示球没有朝门滚（无有效断球点），调用方应回退站位。
bool intercept_point(const WorldModel &wm, double line_dist,
                     double &ix, double &iy);

}  // namespace simuro5
#endif
