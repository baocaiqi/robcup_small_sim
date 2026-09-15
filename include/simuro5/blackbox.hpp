// blackbox.hpp — 临时测量仪器（仅测试用，比赛版不带）
// 每帧写一行 F 行 + 每次摆位回调写 P/B 行到 C:\\Strategy\\hnnu_blackbox.csv
// 绝不影响决策：文件打不开就静默放弃，最多试 3 次。
#ifndef SIMURO5_BLACKBOX_HPP
#define SIMURO5_BLACKBOX_HPP
#include <cstdio>
#include "simuro5/simuro_interface.hpp"

namespace simuro5 { namespace bb {
struct State { std::FILE *fp = nullptr; long frame = 0; int fails = 0; };
inline State &st() { static State s; return s; }        // 全程序唯一实例（inline + 局部 static）
inline std::FILE *fp() {
    State &s = st();
    if (!s.fp && s.fails < 3) {
        s.fp = std::fopen("C:\\Strategy\\hnnu_blackbox.csv", "a");
        if (s.fp) std::fprintf(s.fp, "# ---- new session ----\n");
        else ++s.fails;
    }
    return s.fp;
}
// F 行：平台原始字段 + 我们的判断
inline void frame(long gs, long whos, double bx, double by, double bvx, double bvy,
                  int we_have_ball, int in_penalty, double gk_x, double active_x) {
    std::FILE *f = fp(); if (!f) return;
    State &s = st(); ++s.frame;
    std::fprintf(f, "F,%ld,%ld,%ld,%.2f,%.2f,%.3f,%.3f,%d,%d,%.2f,%.2f\n",
                 s.frame, gs, whos, bx, by, bvx, bvy, we_have_ball, in_penalty, gk_x, active_x);
    if ((s.frame % 40) == 0) std::fflush(f);
}
// P 行：我们**写进去**的摆位（在 formation_* 之后调用，记的就是我们写出的值）
inline void placement(const char *tag, long gs, const Robot *r, int n) {
    std::FILE *f = fp(); if (!f) return;
    std::fprintf(f, "P,%ld,%s,%ld", st().frame, tag, gs);
    for (int i = 0; i < n; ++i) std::fprintf(f, ",%.2f,%.2f", r[i].pos.x, r[i].pos.y);
    std::fprintf(f, "\n"); std::fflush(f);
}
inline void setball(long gs, double x, double y) {
    std::FILE *f = fp(); if (!f) return;
    std::fprintf(f, "B,%ld,%.2f,%.2f,%ld\n", st().frame, x, y, gs); std::fflush(f);
}
// R 行：我方 5 台的 (x,y,rot,role) —— 供"抓屏叠加"工具把画面里的色块对回编号
//   role 在 wm.role[] 里（固定分工：0=门将 1=主攻 2=助攻 3=中场 4=后卫，见 roles.hpp）
inline void robots(long is_blue, const double *xs, const double *ys, const double *rots,
                   const int *roles, int n) {
    std::FILE *f = fp(); if (!f) return;
    std::fprintf(f, "R,%ld,%ld", st().frame, is_blue);
    for (int i = 0; i < n; ++i)
        std::fprintf(f, ",%.2f,%.2f,%.1f,%d", xs[i], ys[i], rots[i], roles[i]);
    std::fprintf(f, "\n");
    if ((st().frame % 40) == 0) std::fflush(f);
}
// O 行：对手 5 台的 (x,y,rot) —— 只为了叠加时多几个参照点（提高标定精度）
inline void opponents(const double *xs, const double *ys, const double *rots, int n) {
    std::FILE *f = fp(); if (!f) return;
    std::fprintf(f, "O,%ld", st().frame);
    for (int i = 0; i < n; ++i) std::fprintf(f, ",%.2f,%.2f,%.1f", xs[i], ys[i], rots[i]);
    std::fprintf(f, "\n");
    if ((st().frame % 40) == 0) std::fflush(f);
}
}}  // namespace simuro5::bb
#endif
