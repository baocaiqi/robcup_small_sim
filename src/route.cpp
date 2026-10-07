// route.cpp — 网格 A* 避障路径规划；cm，原点左下角，蓝队守 x=220 右门
#include "simuro5/route.hpp"
#include "simuro5/team.hpp"

#include <cmath>
#include <cstring>

namespace simuro5 {

namespace {

constexpr double kCell = 4.0;
constexpr int kW = (int)(TeamContext::FIELD_LENGTH / kCell);
constexpr int kH = (int)(TeamContext::FIELD_WIDTH / kCell);
constexpr int kN = kW * kH;
// 半对角线余量 cm：整格都在圆外才可走
constexpr double kHalfDiag = 0.7071067811865476 * kCell;
constexpr double kWallMargin = 4.0;
constexpr int kMaxExpand = 6000;
constexpr float kInf = 1e9f;

inline double cx_of(int i) { return (i + 0.5) * kCell; }
inline double cy_of(int j) { return (j + 0.5) * kCell; }
inline int ix_of(double x) {
    int i = (int)(x / kCell);
    return i < 0 ? 0 : (i >= kW ? kW - 1 : i);
}
inline int iy_of(double y) {
    int j = (int)(y / kCell);
    return j < 0 ? 0 : (j >= kH ? kH - 1 : j);
}

// A* 缓冲区：函数内 static 定长、零分配（本 DLL 单线程）
unsigned char g_blocked[kN];
unsigned char g_state[kN];
float         g_g[kN];
int           g_parent[kN];
struct HeapNode { float f; int idx; };
HeapNode      g_heap[kN];
int           g_hn = 0;

inline void heap_push(float f, int idx) {
    int i = g_hn++;
    g_heap[i].f = f; g_heap[i].idx = idx;
    while (i > 0) {
        int p = (i - 1) >> 1;
        if (g_heap[p].f <= g_heap[i].f) break;
        HeapNode t = g_heap[p]; g_heap[p] = g_heap[i]; g_heap[i] = t;
        i = p;
    }
}

inline int heap_pop() {
    if (g_hn <= 0) return -1;
    int top = g_heap[0].idx;
    g_heap[0] = g_heap[--g_hn];
    int i = 0;
    for (;;) {
        int l = 2 * i + 1, r = l + 1, m = i;
        if (l < g_hn && g_heap[l].f < g_heap[m].f) m = l;
        if (r < g_hn && g_heap[r].f < g_heap[m].f) m = r;
        if (m == i) break;
        HeapNode t = g_heap[m]; g_heap[m] = g_heap[i]; g_heap[i] = t;
        i = m;
    }
    return top;
}

inline double octile(int i0, int j0, int i1, int j1) {
    int dx = i0 > i1 ? i0 - i1 : i1 - i0;
    int dy = j0 > j1 ? j0 - j1 : j1 - j0;
    int d = dx < dy ? dx : dy;
    return kCell * ((dx + dy) + (1.4142135623730951 - 2.0) * d);
}

bool seg_safe(double ax, double ay, double bx, double by,
              const CircleObstacle *obs, int n, double margin) {
    for (int k = 0; k < n; ++k) {
        double r = obs[k].r - margin;
        if (r < 0.0) r = 0.0;
        if (segment_hits_circle(ax, ay, bx, by, obs[k].x, obs[k].y, r)) return false;
    }
    return true;
}

// 不可走格：整格碰圆或太贴边线；起/终点格永远放行（否则可能判无解）
void build_blocked(const CircleObstacle *obs, int n, int s_idx, int t_idx) {
    std::memset(g_blocked, 0, sizeof(g_blocked));
    for (int j = 0; j < kH; ++j) {
        double y = cy_of(j);
        for (int i = 0; i < kW; ++i) {
            double x = cx_of(i);
            int idx = j * kW + i;
            if (idx == s_idx || idx == t_idx) continue;
            if (x < kWallMargin || x > TeamContext::FIELD_LENGTH - kWallMargin ||
                y < kWallMargin || y > TeamContext::FIELD_WIDTH - kWallMargin) {
                g_blocked[idx] = 1;
                continue;
            }
            for (int k = 0; k < n; ++k) {
                double dx = x - obs[k].x, dy = y - obs[k].y;
                double rr = obs[k].r + kHalfDiag;
                if (dx * dx + dy * dy < rr * rr) { g_blocked[idx] = 1; break; }
            }
        }
    }
}

int astar_cells(int s_idx, int t_idx, int &expand_out, int *chain) {
    for (int i = 0; i < kN; ++i) { g_state[i] = 0; g_parent[i] = -1; g_g[i] = kInf; }
    g_hn = 0;
    int si = s_idx % kW, sj = s_idx / kW;
    int ti = t_idx % kW, tj = t_idx / kW;
    g_g[s_idx] = 0.0f;
    g_state[s_idx] = 1;
    heap_push((float)octile(si, sj, ti, tj), s_idx);
    int expand = 0;
    bool reached = false;
    while (g_hn > 0 && expand < kMaxExpand) {
        int cur = heap_pop();
        if (g_state[cur] == 2) continue;
        g_state[cur] = 2;
        ++expand;
        if (cur == t_idx) { reached = true; break; }
        int ci = cur % kW, cj = cur / kW;
        for (int dj = -1; dj <= 1; ++dj)
            for (int di = -1; di <= 1; ++di) {
                if (di == 0 && dj == 0) continue;
                int ni = ci + di, nj = cj + dj;
                if (ni < 0 || ni >= kW || nj < 0 || nj >= kH) continue;
                int nb = nj * kW + ni;
                if (g_blocked[nb] || g_state[nb] == 2) continue;
                double step = (di != 0 && dj != 0) ? kCell * 1.4142135623730951 : kCell;
                float ng = g_g[cur] + (float)step;
                if (ng < g_g[nb]) {
                    g_g[nb] = ng;
                    g_parent[nb] = cur;
                    g_state[nb] = 1;
                    heap_push(ng + (float)octile(ni, nj, ti, tj), nb);
                }
            }
    }
    expand_out = expand;
    if (!reached) return 0;
    int m = 0;
    for (int cur = t_idx; cur >= 0; cur = g_parent[cur]) {
        chain[m++] = cur;
        if (cur == s_idx) break;
        if (m >= kN) break;
    }
    for (int a = 0, b = m - 1; a < b; ++a, --b) { int t = chain[a]; chain[a] = chain[b]; chain[b] = t; }
    return m;
}

// 视线拉直：贪心删中间点，把网格点列压成尽量少的折线
int pull_taut(double sx, double sy, double tx, double ty, const int *chain, int m,
              const CircleObstacle *obs, int n, double margin, RoutePlan &plan) {
    int npt = m + 2;
    auto px = [&](int k) -> double { return k == 0 ? sx : (k == npt - 1 ? tx : cx_of(chain[k - 1] % kW)); };
    auto py = [&](int k) -> double { return k == 0 ? sy : (k == npt - 1 ? ty : cy_of(chain[k - 1] / kW)); };
    int chosen[64];
    int nc = 0;
    chosen[nc++] = 0;
    int cur = 0;
    while (cur < npt - 1) {
        int far = cur + 1;
        for (int k = npt - 1; k > cur + 1; --k) {
            if (seg_safe(px(cur), py(cur), px(k), py(k), obs, n, margin)) { far = k; break; }
        }
        if (nc >= 63) return 0;
        chosen[nc++] = far;
        cur = far;
    }
    plan.found = true;
    plan.n_wp = nc;
    plan.length = 0.0;
    for (int k = 0; k < nc; ++k) {
        plan.wp_x[k] = px(chosen[k]);
        plan.wp_y[k] = py(chosen[k]);
        if (k > 0) plan.length += dist(plan.wp_x[k - 1], plan.wp_y[k - 1],
                                       plan.wp_x[k], plan.wp_y[k]);
    }
    return 1;
}

}  // namespace

RoutePlan plan_route(double sx, double sy, double tx, double ty,
                     const CircleObstacle *obs, int n,
                     double margin) {
    RoutePlan plan;
    if (n > ROUTE_MAX_OBSTACLES) n = ROUTE_MAX_OBSTACLES;

    if (n <= 0 || segment_clear_of_circles(sx, sy, tx, ty, obs, n)) {
        plan.found = true;
        plan.wp_x[0] = sx;  plan.wp_y[0] = sy;
        plan.wp_x[1] = tx;  plan.wp_y[1] = ty;
        plan.n_wp = 2;
        plan.length = dist(sx, sy, tx, ty);
        return plan;
    }

    for (int k = 0; k < n; ++k) {
        double ds2 = (sx - obs[k].x) * (sx - obs[k].x) + (sy - obs[k].y) * (sy - obs[k].y);
        double dt2 = (tx - obs[k].x) * (tx - obs[k].x) + (ty - obs[k].y) * (ty - obs[k].y);
        double r2 = obs[k].r * obs[k].r;
        if (ds2 <= r2 || dt2 <= r2) return plan;
    }

    int s_idx = iy_of(sy) * kW + ix_of(sx);
    int t_idx = iy_of(ty) * kW + ix_of(tx);
    build_blocked(obs, n, s_idx, t_idx);
    static int chain[kN];
    int expand = 0;
    int m = astar_cells(s_idx, t_idx, expand, chain);
    if (m <= 0) return plan;

    RoutePlan p;
    if (!pull_taut(sx, sy, tx, ty, chain, m, obs, n, margin, p)) return plan;

    // 全圆复核（正确性锁）：任一圆不安全即 found=false，调用方回退直线
    for (int i = 0; i + 1 < p.n_wp; ++i)
        for (int k = 0; k < n; ++k)
            if (segment_hits_circle(p.wp_x[i], p.wp_y[i], p.wp_x[i + 1], p.wp_y[i + 1],
                                    obs[k].x, obs[k].y, obs[k].r - margin))
                return plan;
    return p;
}

}  // namespace simuro5
