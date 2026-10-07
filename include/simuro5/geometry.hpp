// geometry.hpp — 几何工具；单位 cm / 度（0=+x，90=+y，原点左下角）
#ifndef SIMURO5_GEOMETRY_HPP
#define SIMURO5_GEOMETRY_HPP

#include <cmath>
#include <algorithm>

namespace simuro5 {

constexpr double SIMURO5_PI = 3.14159265358979323846;

inline double normalize_angle(double a) {
    while (a > 180.0)  a -= 360.0;
    while (a <= -180.0) a += 360.0;
    return a;
}

inline double angle_diff(double a, double b) {
    return normalize_angle(a - b);
}

inline double dist(double x1, double y1, double x2, double y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

// 从 (x1,y1) 看向 (x2,y2) 的角度（度，与场地坐标系一致）
inline double angle_to(double x1, double y1, double x2, double y2) {
    return normalize_angle(std::atan2(y2 - y1, x2 - x1) * 180.0 / SIMURO5_PI);
}

inline double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
}

double point_to_segment_dist(double px, double py,
                             double ax, double ay, double bx, double by);

// 点是否在矩形内（含边界）
inline bool in_rect(double px, double py, double left, double right, double bottom, double top) {
    return px >= left && px <= right && py >= bottom && py <= top;
}

// 障碍统一建模为圆盘 (x,y,r)，cm
struct CircleObstacle {
    double x = 0, y = 0, r = 0;
};

// 线段是否与圆盘相交（相切也算挡）
inline bool segment_hits_circle(double ax, double ay, double bx, double by,
                                double cx, double cy, double r) {
    return point_to_segment_dist(cx, cy, ax, ay, bx, by) <= r;
}

// 线段是否避开全部圆盘；false 时 *hit（可空）给出穿透最深的圆盘
bool segment_clear_of_circles(double ax, double ay, double bx, double by,
                              const CircleObstacle *obs, int n,
                              CircleObstacle *hit = nullptr);

}  // namespace simuro5
#endif
