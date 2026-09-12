// ============================================================
// route.hpp — 避障路径规划（docs/23：均匀网格 A*）
//
// 问题：差速轮机器人从 S 到 T，途中 ≤5 个圆盘障碍（对方机器人，
//       半径 = 本体 6cm + 净空余量）——求「最优」折线路径。
//
// 算法（传统图论，自研实现）：
//   1. 快速路径：S→T 直线不穿任何圆盘 → 直线即最优，不进网格；
//   2. 4cm 均匀网格（55×45）+ 8 邻域 + A*（估价 f=g+h，h 用八方向距离，可采纳
//      ⇒ 第一次弹出终点的路径就是网格最优）；场地边线当作墙（中心距边 ≥4cm），
//      "整格与圆盘相交"的格判为不可走 ⇒ 网格路径严格不穿圆；
//   3. 视线拉直：贪心删中间点 → 2~4 个 waypoint 的折线；
//   4. 全圆安全校验：任一段对任一圆不安全 → found=false（调用方回退直线）。
//
// 为什么不用可见图（2026-09-12 用户指令换成 A*）：可见图表达不了场地墙、
//   也表达不了"沿圆弧绕行"（走廊被判无解就回退直线撞人）；网格 A* 两者天然覆盖。
//
// 输出 waypoint 序列（含起点与终点），由 motion::follow_route 逐段执行。
// ============================================================
#ifndef SIMURO5_ROUTE_HPP
#define SIMURO5_ROUTE_HPP

#include "simuro5/geometry.hpp"

namespace simuro5 {

// 路径规划结果（栈上定长）
// 顶点理论上界 = 2(S,T) + 4n(S/T切线) + 4*C(n,2)(圆对外公切线)，n=5 → 62
struct RoutePlan {
    bool found = false;
    double wp_x[64] = {0}, wp_y[64] = {0};   // waypoint 序列：wp[0]=起点，wp[n_wp-1]=终点
    int n_wp = 0;
    double length = 0.0;                      // 路径总长 cm
};

// 可见图最大障碍数（对方 5 机器人）
constexpr int ROUTE_MAX_OBSTACLES = 5;

// 从 (sx,sy) 到 (tx,ty) 的最优避障折线。
//   obs      : 圆盘障碍数组（坐标需已含净空 inflate）
//   n        : 障碍数（≤ ROUTE_MAX_OBSTACLES）
//   margin   : 判定时的额外数值余量 cm（默认 0.5，抵消"切线相切"的浮点抖动）
//
// ⚠️ margin 与 inflate 的契约（docs/18 顺手优化 ⑤，改参数前必读）：
//   判定半径用 (r − margin)，即**路径允许侵入膨胀圈最多 margin**（默认 0.5cm）。
//   所以调用方必须保证：
//       inflate ≥ 本体半径 + margin + 期望净空
//   现状（roles.cpp kRouteInflate = 10 = 本体 6 + 净空 4）：实际净空 ∈ [3.5, 4.0]cm，
//   单圆绕行实测路径离圆心 9.8cm（侵入膨胀圈 0.2cm，物理净空仍有 3.8cm）——安全。
//   ⚠️ 若有人把 inflate 调到 7（本体 6 + 净空 1），margin 0.5 会吃掉一半余量，
//   相切段就可能贴到 6.5cm ≈ 本体半径 → 真撞。要调小 inflate，必须同时调小 margin。
//
// 返回：
//   found=true  → wp 序列安全折线；found=false → 起点/终点在障碍内部或无法绕行
//   （调用方应回退直线移动，勿原地死等）
RoutePlan plan_route(double sx, double sy, double tx, double ty,
                     const CircleObstacle *obs, int n,
                     double margin = 0.5);

}  // namespace simuro5
#endif