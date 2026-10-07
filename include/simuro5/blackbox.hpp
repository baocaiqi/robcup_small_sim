// blackbox.hpp — 诊断黑匣子：仅 SIMURO5_HNNU_TRACE 编译时生效（默认关），开启后往 C:\\Strategy\\hnnu_blackbox.csv 追加 F/P/B/R/O 行
#ifndef SIMURO5_BLACKBOX_HPP
#define SIMURO5_BLACKBOX_HPP
#include <cstdio>
#include "simuro5/simuro_interface.hpp"

namespace simuro5 { namespace bb {

#ifdef SIMURO5_HNNU_TRACE
inline constexpr bool kEnabled = true;
#else
inline constexpr bool kEnabled = false;
#endif

struct State { std::FILE *fp = nullptr; long frame = 0; int fails = 0; };
inline State &st() { static State s; return s; }
inline std::FILE *fp() {
    if (!kEnabled) return nullptr;
    State &s = st();
    if (!s.fp && s.fails < 3) {
        s.fp = std::fopen("C:\\Strategy\\hnnu_blackbox.csv", "a");
        if (s.fp) std::fprintf(s.fp, "# ---- new session ----\n");
        else ++s.fails;
    }
    return s.fp;
}
inline void frame(long gs, long whos, double bx, double by, double bvx, double bvy,
                  int we_have_ball, int in_penalty, double gk_x, double active_x) {
    std::FILE *f = fp(); if (!f) return;
    State &s = st(); ++s.frame;
    std::fprintf(f, "F,%ld,%ld,%ld,%.2f,%.2f,%.3f,%.3f,%d,%d,%.2f,%.2f\n",
                 s.frame, gs, whos, bx, by, bvx, bvy, we_have_ball, in_penalty, gk_x, active_x);
    if ((s.frame % 40) == 0) std::fflush(f);
}
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
inline void robots(long is_blue, const double *xs, const double *ys, const double *rots,
                   const int *roles, int n) {
    std::FILE *f = fp(); if (!f) return;
    std::fprintf(f, "R,%ld,%ld", st().frame, is_blue);
    for (int i = 0; i < n; ++i)
        std::fprintf(f, ",%.2f,%.2f,%.1f,%d", xs[i], ys[i], rots[i], roles[i]);
    std::fprintf(f, "\n");
    if ((st().frame % 40) == 0) std::fflush(f);
}
inline void opponents(const double *xs, const double *ys, const double *rots, int n) {
    std::FILE *f = fp(); if (!f) return;
    std::fprintf(f, "O,%ld", st().frame);
    for (int i = 0; i < n; ++i) std::fprintf(f, ",%.2f,%.2f,%.1f", xs[i], ys[i], rots[i]);
    std::fprintf(f, "\n");
    if ((st().frame % 40) == 0) std::fflush(f);
}
}}  // namespace simuro5::bb
#endif
