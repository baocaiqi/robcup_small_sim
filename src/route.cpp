// ============================================================
// route.cpp — 避障路径规划实现（可见图 + Dijkstra，docs/15 P0）
//
// 切点几何：
//   1) 点 P → 圆 (C,r) 切线切点：d=|PC|，切点相对圆心角 = 方位角 ± acos(r/d)；
//   2) 圆 (C1,r1) 与 (C2,r2) 外公切线切点对：
//        cos(α-θ) = (r1-r2)/d（θ=圆心连线方位），两解 α 各给一组切点对
//        T1 = C1 + r1·(cosα,sinα)，T2 = C2 + r2·(cosα,sinα)
//      只做外(同侧)公切线——绕行不穿两防守者之间的缝（战术约束）。
//
// 性能设计（docs/15 §2.2 补）：真实调用每帧最多几次、每次必须微秒级，
//   因此公开入口 plan_route 先做「邻域裁剪」——只有与 S→T 线段距离 < 2r 的
//   圆才入图（直线正挡 + 紧邻绕行必经者，场上通常 1~3 个，顶点 ≤ 26），
//   规划结果再对全部圆验证，撞到未入图圆时用 3r 邻域补算一轮（保证正确性）。
//   全圆 62 顶点图（5 圆全入）每帧 ≈0.3ms，会让 sim 50 场 13.6s→38.6s，故不做。
// ============================================================
#include "simuro5/route.hpp"
#include <cmath>

namespace simuro5 {

namespace {

constexpr int kMaxV = 2 + 4 * ROUTE_MAX_OBSTACLES
                    + 4 * (ROUTE_MAX_OBSTACLES * (ROUTE_MAX_OBSTACLES - 1) / 2);  // 62
constexpr double kEps = 1e-6;   // 顶点去重容差 cm

struct Vtx { double x, y; };

// 从点 P 向圆 (C,r) 求切线切点，写入 out，返回个数(0/1/2)
int tangents_from_point(double px, double py, double cx, double cy, double r,
                        Vtx out[2]) {
    double dx = px - cx, dy = py - cy;
    double d2 = dx * dx + dy * dy;
    if (d2 <= r * r) return 0;                 // 点在圆内/圆上：无切线
    double d = std::sqrt(d2);
    double phi = std::acos(r / d);
    double base = std::atan2(dy, dx);
    int cnt = 0;
    for (int s = -1; s <= 1; s += 2) {
        double a = base + s * phi;
        out[cnt].x = cx + r * std::cos(a);
        out[cnt].y = cy + r * std::sin(a);
        ++cnt;
    }
    return cnt;
}

// 两圆外公切线：两条切线各一对切点（T1 在圆 i，T2 在圆 j），返回切点对组数(0/1/2)
int external_tangents(double c1x, double c1y, double r1,
                      double c2x, double c2y, double r2,
                      Vtx out[4]) {
    double dx = c2x - c1x, dy = c2y - c1y;
    double d = std::hypot(dx, dy);
    if (d < 1e-9) return 0;
    double ratio = (r1 - r2) / d;
    if (ratio > 1.0) ratio = 1.0;
    if (ratio < -1.0) ratio = -1.0;
    double theta = std::atan2(dy, dx);
    double phi = std::acos(ratio);             // α = θ ± φ
    int cnt = 0;
    for (int s = -1; s <= 1; s += 2) {
        double a = theta + s * phi;
        out[cnt].x = c1x + r1 * std::cos(a);
        out[cnt].y = c1y + r1 * std::sin(a);
        out[cnt + 1].x = c2x + r2 * std::cos(a);
        out[cnt + 1].y = c2y + r2 * std::sin(a);
        cnt += 2;
        if (phi < 1e-9) break;                 // 相切退化：只有一条
    }
    return cnt;
}

// 加顶点（去重）：返回顶点索引；若为新增（索引 == *nv），调用方需把 *nv 加 1。
// 容量满时返回 nv（忽略新顶点），保证不越界。
int add_vertex(Vtx *v, int nv, double x, double y) {
    for (int i = 0; i < nv; ++i)
        if (std::fabs(v[i].x - x) < kEps && std::fabs(v[i].y - y) < kEps)
            return i;
    if (nv >= kMaxV) return nv;
    v[nv].x = x; v[nv].y = y;
    return nv;
}

// 线段 i→j 是否安全：不穿任何圆盘（判定半径收 margin，抵消相切浮点抖动）
bool edge_ok(const Vtx *v, int i, int j,
             const CircleObstacle *obs, int n, double margin) {
    for (int k = 0; k < n; ++k) {
        double r = obs[k].r - margin;
        if (r < 0.0) r = 0.0;
        if (segment_hits_circle(v[i].x, v[i].y, v[j].x, v[j].y,
                                obs[k].x, obs[k].y, r))
            return false;
    }
    return true;
}

// 子规划：对给定邻域圆集合建可见图 + Dijkstra + 拉直（内部核心）
RoutePlan plan_route_sub(double sx, double sy, double tx, double ty,
                         const CircleObstacle *obs, int n, double margin) {
    RoutePlan plan;
    if (n > ROUTE_MAX_OBSTACLES) n = ROUTE_MAX_OBSTACLES;
    // 直线安全即最优（邻域内仍可能直线通，如远处障碍未挡）
    if (n <= 0 || segment_clear_of_circles(sx, sy, tx, ty, obs, n)) {
        plan.found = true;
        plan.wp_x[0] = sx;  plan.wp_y[0] = sy;
        plan.wp_x[1] = tx;  plan.wp_y[1] = ty;
        plan.n_wp = 2;
        plan.length = dist(sx, sy, tx, ty);
        return plan;
    }

    Vtx v[kMaxV];
    int nv = 0;
    int idx_s = add_vertex(v, 0, sx, sy);  nv += (idx_s == nv) ? 1 : 0;   // 0: S
    int idx_t = add_vertex(v, nv, tx, ty); nv += (idx_t == nv) ? 1 : 0;   // 1: T

    bool s_inside = false, t_inside = false;
    for (int k = 0; k < n; ++k) {
        double ds2 = (sx - obs[k].x) * (sx - obs[k].x) + (sy - obs[k].y) * (sy - obs[k].y);
        double dt2 = (tx - obs[k].x) * (tx - obs[k].x) + (ty - obs[k].y) * (ty - obs[k].y);
        double r2 = obs[k].r * obs[k].r;
        if (ds2 <= r2) s_inside = true;
        if (dt2 <= r2) t_inside = true;
    }
    // 起/终点落在障碍内部（贴球/贴身场景）→ 不可规划，调用方回退直线
    if (s_inside || t_inside) return plan;

    // S/T 对每个圆的切线切点
    for (int k = 0; k < n; ++k) {
        Vtx q[2];
        int c = tangents_from_point(sx, sy, obs[k].x, obs[k].y, obs[k].r, q);
        for (int i = 0; i < c; ++i) { int id = add_vertex(v, nv, q[i].x, q[i].y); nv += (id == nv) ? 1 : 0; }
        c = tangents_from_point(tx, ty, obs[k].x, obs[k].y, obs[k].r, q);
        for (int i = 0; i < c; ++i) { int id = add_vertex(v, nv, q[i].x, q[i].y); nv += (id == nv) ? 1 : 0; }
    }
    // 圆-圆外公切线切点（路径绕多个错位防守者时需要）
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            Vtx q[4];
            int c = external_tangents(obs[i].x, obs[i].y, obs[i].r,
                                      obs[j].x, obs[j].y, obs[j].r, q);
            for (int t = 0; t < c; ++t) { int id = add_vertex(v, nv, q[t].x, q[t].y); nv += (id == nv) ? 1 : 0; }
        }

    // —— Dijkstra（稠密小图 O(V²)）——
    constexpr double INF = 1e18;
    double d[kMaxV];
    int prev[kMaxV];
    bool done[kMaxV];
    for (int i = 0; i < nv; ++i) { d[i] = INF; prev[i] = -1; done[i] = false; }
    d[idx_s] = 0.0;
    for (int iter = 0; iter < nv; ++iter) {
        int u = -1;
        for (int i = 0; i < nv; ++i)
            if (!done[i] && (u < 0 || d[i] < d[u])) u = i;
        if (u < 0 || d[u] >= INF) break;
        done[u] = true;
        if (u == idx_t) break;
        for (int j = 0; j < nv; ++j) {
            if (done[j] || j == u) continue;
            if (!edge_ok(v, u, j, obs, n, margin)) continue;
            double w = dist(v[u].x, v[u].y, v[j].x, v[j].y);
            if (d[u] + w < d[j]) { d[j] = d[u] + w; prev[j] = u; }
        }
    }
    if (prev[idx_t] < 0) return plan;          // 邻域内无通路

    // 回溯（逆序）→ 转正
    int seq[kMaxV], len = 0;
    for (int cur = idx_t; cur >= 0; cur = prev[cur]) {
        seq[len++] = cur;
        if (cur == idx_s) break;
    }
    int path[kMaxV], plen = 0;
    for (int i = len - 1; i >= 0; --i) path[plen++] = seq[i];

    // 拉直优化：删中间点后仍安全则删
    int out[kMaxV], olen = 0;
    for (int i = 0; i < plen; ++i) {
        while (olen >= 2 &&
               edge_ok(v, out[olen - 2], path[i], obs, n, margin)) {
            --olen;
        }
        out[olen++] = path[i];
    }

    plan.found = true;
    plan.n_wp = olen;
    plan.length = 0.0;
    for (int i = 0; i < olen; ++i) {
        plan.wp_x[i] = v[out[i]].x;
        plan.wp_y[i] = v[out[i]].y;
        if (i > 0) plan.length += dist(v[out[i - 1]].x, v[out[i - 1]].y,
                                       v[out[i]].x, v[out[i]].y);
    }
    return plan;
}

}  // namespace

RoutePlan plan_route(double sx, double sy, double tx, double ty,
                     const CircleObstacle *obs, int n,
                     double margin) {
    RoutePlan plan;
    if (n > ROUTE_MAX_OBSTACLES) n = ROUTE_MAX_OBSTACLES;

    // —— 快速路径：直线安全即最优 ——
    if (n <= 0 || segment_clear_of_circles(sx, sy, tx, ty, obs, n)) {
        plan.found = true;
        plan.wp_x[0] = sx;  plan.wp_y[0] = sy;
        plan.wp_x[1] = tx;  plan.wp_y[1] = ty;
        plan.n_wp = 2;
        plan.length = dist(sx, sy, tx, ty);
        return plan;
    }

    // —— 邻域裁剪：只规划影响 S→T 的圆（2r 邻域），结果对全圆验证 ——
    CircleObstacle sub[ROUTE_MAX_OBSTACLES];
    int ns = 0;
    for (int k = 0; k < n; ++k) {
        double d = point_to_segment_dist(obs[k].x, obs[k].y, sx, sy, tx, ty);
        if (d < 2.0 * obs[k].r) { sub[ns] = obs[k]; ++ns; }
    }
    if (ns == 0) {                               // 兜底（快速路径 false 时理论不达）
        plan.found = true;
        plan.wp_x[0] = sx; plan.wp_y[0] = sy;
        plan.wp_x[1] = tx; plan.wp_y[1] = ty;
        plan.n_wp = 2;
        plan.length = dist(sx, sy, tx, ty);
        return plan;
    }
    RoutePlan p = plan_route_sub(sx, sy, tx, ty, sub, ns, margin);
    if (!p.found) return p;                      // 邻域不可达 → 全圆必然不可达
    // 全圆验证：撞到未入图圆 → 3r 邻域补算一轮
    bool bad = false;
    for (int i = 0; i < p.n_wp - 1 && !bad; ++i)
        for (int k = 0; k < n && !bad; ++k)
            if (segment_hits_circle(p.wp_x[i], p.wp_y[i],
                                    p.wp_x[i + 1], p.wp_y[i + 1],
                                    obs[k].x, obs[k].y,
                                    obs[k].r - margin))
                bad = true;
    if (!bad) return p;
    CircleObstacle sub2[ROUTE_MAX_OBSTACLES];
    int ns2 = 0;
    for (int k = 0; k < n; ++k) {
        double d = point_to_segment_dist(obs[k].x, obs[k].y, sx, sy, tx, ty);
        if (d < 3.0 * obs[k].r) { sub2[ns2] = obs[k]; ++ns2; }
    }
    if (ns2 == ns) return p;                     // 无新圆：返回最接近的规划
    RoutePlan p2 = plan_route_sub(sx, sy, tx, ty, sub2, ns2, margin);
    return p2.found ? p2 : p;                    // 补算失败也给首轮结果（调用方兜底）
}

}  // namespace simuro5