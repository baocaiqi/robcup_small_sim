// ============================================================
// geometry.hpp — 几何工具（移植自中型组 robcup_mid_sim 的
// nubot_interfaces 几何库，去掉了 ROS2 依赖）
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

}  // namespace simuro5
#endif
