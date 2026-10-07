// hungarian.hpp — 带权匈牙利指派（Kuhn–Munkres 势函数法），O(n³)
// 一一匹配求最小总代价；参考公开算法思想（Kuhn 1955 / Munkres 1957），本项目自研实现
#ifndef SIMURO5_HUNGARIAN_HPP
#define SIMURO5_HUNGARIAN_HPP

#include <cstddef>

namespace simuro5 {

// 规模上限 n（本项目盯人 8×8，留余量到 12，数组只多 4 个 double）
constexpr int kHungarianMaxN = 12;

// 求 n×n 代价矩阵的最小总代价完美匹配（每行配一列）
// cost 为行优先 n*n；row_to_col[r] = 第 r 行匹配的列号；false = 参数非法（回退旧逻辑）
inline bool hungarian_solve(const double *cost, int n, int *row_to_col) {
    if (!cost || !row_to_col || n < 1 || n > kHungarianMaxN) return false;
    const double kInf = 1e18;
    double u[kHungarianMaxN + 1];
    double v[kHungarianMaxN + 1];
    int p[kHungarianMaxN + 1];      // p[j] = 当前配到第 j 列的行（0 = 未配）
    int way[kHungarianMaxN + 1];    // 增广路前驱
    double minv[kHungarianMaxN + 1];
    bool used[kHungarianMaxN + 1];
    for (int j = 0; j <= n; ++j) { u[j] = 0.0; v[j] = 0.0; p[j] = 0; way[j] = 0; }
    for (int i = 1; i <= n; ++i) {          // 1-based 内部下标，第 0 列是虚拟起点
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
            if (j1 < 0) return false;
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
