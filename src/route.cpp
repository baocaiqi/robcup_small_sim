// ============================================================
// route.cpp — 避障路径规划：均匀网格 A*（docs/23，替换可见图+Dijkstra）
//
// 问题：差速轮机器人从 S 到 T，途中 ≤5 个圆盘障碍（对方机器人，
//       半径 = 本体 6cm + 净空余量）——求「最优」折线路径。
//
// 为什么换成 A*（2026-09-12 用户指令）：
//   · 可见图表达不了"墙"：它只在障碍圆之间连切线，路径可以贴着场地边线甚至
//     出界（旧实现完全没有场地边界概念），也表达不了"沿圆弧绕行"（半径大/重叠
//     的走廊会被判成无解 → 回退直线 → 直着撞人）。
//   · 网格 A* 天然带边界（边线 = 障碍），一格一格走，既能绕大圆也能贴墙走；
//     代价函数还是"路径长度"，可读性/可调试性都比切点几何好。
//
// 算法（传统图论，自研实现）：
//   1. 快速路径：S→T 直线不穿任何圆盘 → 直线即最优，不建网格（占绝大多数调用）；
//   2. 均匀网格：4cm 格（55×45=2475 格），8 邻域；格代价 = 欧氏步长；
//   3. A* 搜索：估价 f = g（已走长度）+ h（八方向距离启发，可采纳 ⇒ 最优）；
//      "该格不可走"的判定见下方 kBlockPad/wall 说明；
//   4. 视线拉直：贪心删中间点（删除后线段仍安全就删）——把阶梯状网格路径
//      拉成 2~4 个 waypoint 的折线（本平台执行层 motion::follow_route 只认折线）；
//   5. 全圆安全校验：拉直后的每一段都必须对**全部**圆安全，否则 found=false
//      （调用方按既有约定回退直线移动，宁可诚实失败也不返回会撞的路径）。
//
// 与调用方的契约（roles.cpp kRouteInflate=10 / margin=0.5，改参数前必读 route.hpp）：
//   判定半径用 (r − margin)，即路径允许侵入膨胀圈最多 margin(0.5cm)；
//   调用方必须保证 inflate ≥ 本体半径 + margin + 期望净空。
//
// 性能（实测见 offline_test "plan_route 最坏配置"）：真实调用每帧最多几次，
//   直线通时 ~1µs；需要绕行时才建网格 + A*，实测均值在几十 µs 量级，
//   相对 25ms 的帧周期可忽略。缓冲区全部为函数内 static 定长数组（零分配）。
// ============================================================
#include "simuro5/route.hpp"
#include "simuro5/team.hpp"

#include <cmath>
#include <cstring>

namespace simuro5 {

namespace {

// ---------- 网格参数 ----------
constexpr double kCell = 4.0;                                        // 格边长 cm
constexpr int kW = (int)(TeamContext::FIELD_LENGTH / kCell);          // 55
constexpr int kH = (int)(TeamContext::FIELD_WIDTH / kCell);           // 45
constexpr int kN = kW * kH;                                           // 2475
// 半对角线：只有"整格都在圆外"才放行 ⇒ 相邻两格中心连线（必在该两格并集内）
// 也一定在圆外 ⇒ 网格路径严格安全，交给后面的视线拉直去贴近切线最优。
constexpr double kHalfDiag = 0.7071067811865476 * kCell;              // 2.83cm
// 场地边线：机器人中心离边线至少这么远（本体半径量级），等效于"墙"障碍。
constexpr double kWallMargin = 4.0;
constexpr int kMaxExpand = 6000;                                      // 扩展上限
constexpr float kInf = 1e9f;

inline double cx_of(int i) { return (i + 0.5) * kCell; }
inline double cy_of(int j) { return (j + 0.5) * kCell; }              // 格中心坐标
inline int ix_of(double x) {
    int i = (int)(x / kCell);
    return i < 0 ? 0 : (i >= kW ? kW - 1 : i);
}
inline int iy_of(double y) {
    int j = (int)(y / kCell);
    return j < 0 ? 0 : (j >= kH ? kH - 1 : j);
}

// ---------- A* 缓冲区（函数内 static：零分配、定长；本 DLL 单线程调用） ----------
unsigned char g_blocked[kN];      // 1 = 不可走
unsigned char g_state[kN];        // 0 未访问 / 1 在开表 / 2 已关
float         g_g[kN];            // 已走长度
int           g_parent[kN];       // 父格（-1 = 无）
struct HeapNode { float f; int idx; };
HeapNode      g_heap[kN];         // 开表（二叉堆）
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

inline int heap_pop() {                       // 返回格号；空堆返回 -1
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

// 八方向距离（可采纳启发：不超过真实剩余路程）
inline double octile(int i0, int j0, int i1, int j1) {
    int dx = i0 > i1 ? i0 - i1 : i1 - i0;
    int dy = j0 > j1 ? j0 - j1 : j1 - j0;
    int d = dx < dy ? dx : dy;
    return kCell * ((dx + dy) + (1.4142135623730951 - 2.0) * d);
}

// 线段是否安全（不穿任何圆盘；判定半径收 margin，抵消"贴边"浮点抖动）
bool seg_safe(double ax, double ay, double bx, double by,
              const CircleObstacle *obs, int n, double margin) {
    for (int k = 0; k < n; ++k) {
        double r = obs[k].r - margin;
        if (r < 0.0) r = 0.0;
        if (segment_hits_circle(ax, ay, bx, by, obs[k].x, obs[k].y, r)) return false;
    }
    return true;
}

// 建"不可走"格：① 整格与某圆盘相交（含半对角线余量）② 太贴场地边线。
//   起/终点所在格永远放行——机器人/目标就在那儿，不能因为贴墙或贴身就判无解。
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

// A* 搜索：返回是否找到；格序（起点格…终点格）写入 chain，返回格数。
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
        if (g_state[cur] == 2) continue;          // 过期条目
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
    // 逆序 → 正序
    for (int a = 0, b = m - 1; a < b; ++a, --b) { int t = chain[a]; chain[a] = chain[b]; chain[b] = t; }
    return m;
}

// 视线拉直：把阶梯状网格点列压成尽量少的折线（贪心"能看多远走多远"）
int pull_taut(double sx, double sy, double tx, double ty, const int *chain, int m,
              const CircleObstacle *obs, int n, double margin, RoutePlan &plan) {
    // 点列 = S + 各格中心 + T（起点格中心与 S 很近，拉直会自动删掉）
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
        if (nc >= 63) return 0;               // 容量兜底（正常 ≤ 2+2*障碍数）
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

    // —— 1. 快速路径：直线安全即最优（占绝大多数调用，不进网格）——
    if (n <= 0 || segment_clear_of_circles(sx, sy, tx, ty, obs, n)) {
        plan.found = true;
        plan.wp_x[0] = sx;  plan.wp_y[0] = sy;
        plan.wp_x[1] = tx;  plan.wp_y[1] = ty;
        plan.n_wp = 2;
        plan.length = dist(sx, sy, tx, ty);
        return plan;
    }

    // —— 2. 起/终点落在障碍内部（贴球/贴身）→ 不可规划，调用方回退直线 ——
    for (int k = 0; k < n; ++k) {
        double ds2 = (sx - obs[k].x) * (sx - obs[k].x) + (sy - obs[k].y) * (sy - obs[k].y);
        double dt2 = (tx - obs[k].x) * (tx - obs[k].x) + (ty - obs[k].y) * (ty - obs[k].y);
        double r2 = obs[k].r * obs[k].r;
        if (ds2 <= r2 || dt2 <= r2) return plan;      // found=false
    }

    // —— 3. 网格 A* ——
    int s_idx = iy_of(sy) * kW + ix_of(sx);
    int t_idx = iy_of(ty) * kW + ix_of(tx);
    build_blocked(obs, n, s_idx, t_idx);
    static int chain[kN];
    int expand = 0;
    int m = astar_cells(s_idx, t_idx, expand, chain);
    if (m <= 0) return plan;                          // 无通路（被围死）→ found=false

    // —— 4. 视线拉直 → waypoint 折线 ——
    RoutePlan p;
    if (!pull_taut(sx, sy, tx, ty, chain, m, obs, n, margin, p)) return plan;

    // —— 5. 全圆安全校验（正确性锁）——
    //   拉直用的是 (r−margin) 的判定半径（按契约允许贴边），这里再对全部圆复核一遍：
    //   found=true 必须意味着"对全部圆都安全"，做不到就报告不可规划，
    //   由调用方按既有约定回退直线（宁可诚实失败，也不返回会撞的路径）。
    for (int i = 0; i + 1 < p.n_wp; ++i)
        for (int k = 0; k < n; ++k)
            if (segment_hits_circle(p.wp_x[i], p.wp_y[i], p.wp_x[i + 1], p.wp_y[i + 1],
                                    obs[k].x, obs[k].y, obs[k].r - margin))
                return plan;                          // found=false
    return p;
}

}  // namespace simuro5
