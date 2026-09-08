// ============================================================
// geometry.hpp — 几何工具
// 单位：cm / 度
// ============================================================
#ifndef SIMURO5_GEOMETRY_HPP
#define SIMURO5_GEOMETRY_HPP

#include <cmath>
#include <algorithm>

namespace simuro5 {

constexpr double SIMURO5_PI = 3.14159265358979323846;

// 角度归一化到 (-180, 180]
inline double normalize_angle(double a) {
    while (a > 180.0)  a -= 360.0;
    while (a <= -180.0) a += 360.0;
    return a;
}

// 两角之差(带符号, 范围 (-180,180])
inline double angle_diff(double a, double b) {
    return normalize_angle(a - b);
}

// 两点距离
inline double dist(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

// 从 (x1,y1) 看向 (x2,y2) 的角度(度, 与场地坐标系一致: 0=+x, 90=+y)
inline double angle_to(double x1, double y1, double x2, double y2) {
    return normalize_angle(std::atan2(y2 - y1, x2 - x1) * 180.0 / SIMURO5_PI);
}

inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

// 点 (px,py) 到线段 (a1,a2) 的最短距离（传球/射门路线检测用）
double point_to_segment_dist(double px, double py,
                             double ax, double ay, double bx, double by);

// 点是否在矩形内（含边界）
inline bool in_rect(double px, double py, double left, double right, double bottom, double top) {
    return px >= left && px <= right && py >= bottom && py <= top;
}

// ============================================================
// 圆盘碰撞原语（docs/15 P0：射门/传球线路、路径段合法性的统一判定）
// 机器人/球等障碍一律建模为「圆盘 (x,y,r)」，检测 O(1) 解析解。
// ============================================================
struct CircleObstacle {
    double x = 0, y = 0, r = 0;
};

// 线段 (a→b) 是否与圆盘 (c,r) 相交（圆心到线段距离 ≤ r，相切也算挡）
inline bool segment_hits_circle(double ax, double ay, double bx, double by,
                                double cx, double cy, double r) {
    return point_to_segment_dist(cx, cy, ax, ay, bx, by) <= r;
}

// 线段 (a→b) 是否避开全部圆盘（遍历 n 个，n≤5 时 O(n) 已是最优下界）。
// 返回 false 时 *hit（可空）给出「穿透最深」的挡路圆盘，供上层决策用。
bool segment_clear_of_circles(double ax, double ay, double bx, double by,
                              const CircleObstacle *obs, int n,
                              CircleObstacle *hit = nullptr);

}  // namespace simuro5
#endif
