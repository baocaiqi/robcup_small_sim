// ============================================================
// route.hpp — 避障路径规划（docs/15 P0）
//
// 问题：差速轮机器人从 S 到 T，途中 ≤5 个圆盘障碍（对方机器人，
//       半径 = 本体 6cm + 净空余量）——求「最优」折线路径。
//
// 算法（传统图论，自研实现，参考《Computational Geometry》可见图方法）：
//   1. 快速路径：S→T 直线不穿任何圆盘 → 直线即最优，不进图；
//   2. 可见图：顶点 = S、T + 每圆上「S/T 切线切点」+「圆-圆外公切线切点」
//      （路径绕多个错位防守者时必须走圆间公切线，只做外侧公切线，
//       不做内公切线——机器人不挤两人之间的缝，符合战术）；
//      边 = 两顶点连线不穿任何圆盘；权 = 欧氏距离；
//   3. Dijkstra（稠密小图 O(V²)）求 S→T 最短路；
//   4. 拉直优化：删除可去中间点（删除后线段仍安全）→ 更直的路径。
//
// 输出 waypoint 序列（含起点与终点），由 motion::follow_route 逐段执行。
// 差速轮转向代价一期不做——切点结构保证路径贴着障碍边缘、每段直线。
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
// 返回：
//   found=true  → wp 序列安全折线；found=false → 起点/终点在障碍内部或无法绕行
//   （调用方应回退直线移动，勿原地死等）
RoutePlan plan_route(double sx, double sy, double tx, double ty,
                     const CircleObstacle *obs, int n,
                     double margin = 0.5);

}  // namespace simuro5
#endif