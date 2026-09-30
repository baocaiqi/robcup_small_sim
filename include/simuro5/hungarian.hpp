// ============================================================
// hungarian.hpp — 带权匈牙利指派（Kuhn–Munkres 势函数法）
//
// 生活化比喻：**给一队人派活**。「这几个人去干那几件活，谁去干哪件，
//   才能让总代价（走路距离 + 换人的折腾）最小」——而且必须**一对一**，
//   不能两个人抢同一件活。匈牙利算法就是**多项式时间求这个最小总代价配对**的方法。
//
// 为什么不用贪心（每个人各自挑最划算的）：贪心会**撞车**——两个人各自 argmax
//   可能挑中同一个目标，于是最危险的那个反而没人管（这就是 docs/06 第 97 轮
//   之前盯人的真实毛病）。匈牙利天然是**一一匹配**，从数学上杜绝重复。
//
// 复杂度 O(n³)；本项目的规模：3 个防守者 ×（5 个对手 + 3 个虚拟列）= 8×8，
//   每帧跑一次的开销可以忽略（<1000 次浮点运算）。
//
// ⚠️ 合规说明（团队铁律 4 / 原创红线）：**参考公开的匈牙利算法思想（Kuhn 1955 /
//   Munkres 1957；常见实现形式为「势函数 + 增广路」），由本项目自研实现**，
//   没有拷贝任何第三方库代码；n ≤ 8 也不需要引外部依赖。
// ============================================================
#ifndef SIMURO5_HUNGARIAN_HPP
#define SIMURO5_HUNGARIAN_HPP

#include <cstddef>

namespace simuro5 {

// 本实现的规模上限：方阵阶数 n。
//   盯人分配的方阵 = 防守者(≤3) + 虚拟行(5) = 8 列/行 → 8；留出余量到 12，
//   这样即使将来把更多队员纳入分配也不会越界（数组只多 4 个 double，开销可忽略）。
constexpr int kHungarianMaxN = 12;

// 求 n×n 代价矩阵的最小总代价**完美匹配**（每行正好配一列，每列正好配一行）。
//
// 输入：
//   cost : 长度 n*n 的行优先矩阵（cost[r*n + c] = 第 r 行配第 c 列的代价）
//   n    : 阶数，1 ≤ n ≤ kHungarianMaxN
// 输出：
//   row_to_col : 长度 n，row_to_col[r] = 第 r 行匹配到的列号（0-based）
// 返回 false 表示参数非法（n 越界）——调用方应回退到旧逻辑。
//
// 实现：势函数（u/v）+ 增广路（way/p）。
//   维护「行势 u[i]」「列势 v[j]」，使**约化代价** c'(i,j) = cost(i,j) − u[i] − v[j] ≥ 0；
//   每轮为一行找增广路，沿最小约化代价把势更新到「出现零边」为止，再做一次交错路翻转。
//   放缩量 delta = min 未访问列的 minv，保证每轮至少一条边归零 → 总复杂度 O(n³)。
inline bool hungarian_solve(const double *cost, int n, int *row_to_col) {
    if (!cost || !row_to_col || n < 1 || n > kHungarianMaxN) return false;
    const double kInf = 1e18;
    double u[kHungarianMaxN + 1];
    double v[kHungarianMaxN + 1];
    int p[kHungarianMaxN + 1];      // p[j] = 当前配到第 j 列的行（0 = 未配）
    int way[kHungarianMaxN + 1];    // 增广路前驱：way[j] = 走到第 j 列之前在哪一列
    double minv[kHungarianMaxN + 1];
    bool used[kHungarianMaxN + 1];
    for (int j = 0; j <= n; ++j) { u[j] = 0.0; v[j] = 0.0; p[j] = 0; way[j] = 0; }
    for (int i = 1; i <= n; ++i) {          // 逐行加进来（1-based 内部下标，第 0 列是虚拟起点）
        p[0] = i;
        int j0 = 0;
        for (int j = 0; j <= n; ++j) { minv[j] = kInf; used[j] = false; }
        do {
            used[j0] = true;
            const int i0 = p[j0];
            int j1 = -1;
            double delta = kInf;
            for (int j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const double cur = cost[(i0 - 1) * n + (j - 1)] - u[i0] - v[j];
                if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                if (minv[j] < delta) { delta = minv[j]; j1 = j; }
            }
            if (j1 < 0) return false;        // 理论上不会发生（代价有限即可）
            for (int j = 0; j <= n; ++j) {
                if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                else         { minv[j] -= delta; }
            }
            j0 = j1;
        } while (p[j0] != 0);
        do {                                  // 沿交错路翻转匹配
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }
    for (int j = 1; j <= n; ++j) {
        if (p[j] >= 1 && p[j] <= n) row_to_col[p[j] - 1] = j - 1;
    }
    return true;
}

}  // namespace simuro5
#endif
