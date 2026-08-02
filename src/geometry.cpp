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

}  // namespace simuro5
