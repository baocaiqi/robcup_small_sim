#include "simuro5/geometry.hpp"

namespace simuro5 {

double point_to_segment_dist(double px, double py,
                             double ax, double ay, double bx, double by) {
    double dx = bx - ax, dy = by - ay;
    double len2 = dx * dx + dy * dy;
    if (len2 < 1e-9) return dist(px, py, ax, ay);
    double t = ((px - ax) * dx + (py - ay) * dy) / len2;
    t = clamp(t, 0.0, 1.0);
    return dist(px, py, ax + t * dx, ay + t * dy);
}

bool segment_clear_of_circles(double ax, double ay, double bx, double by,
                              const CircleObstacle *obs, int n,
                              CircleObstacle *hit) {
    double best_pen = -1.0;   // 最大「穿透深度」：正=被挡（相切也算，口径同 segment_hits_circle）
    int best = -1;
    for (int i = 0; i < n; ++i) {
        double d = point_to_segment_dist(obs[i].x, obs[i].y, ax, ay, bx, by);
        double pen = obs[i].r - d;
        if (pen >= 0.0 && pen > best_pen) { best_pen = pen; best = i; }
    }
    if (best < 0) return true;
    if (hit) *hit = obs[best];
    return false;
}

}  // namespace simuro5
