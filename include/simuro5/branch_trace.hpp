// branch_trace.hpp — 记录每台机器人本帧最后一条运动指令的源码行号（仅 SIMURO5_BRANCH_TRACE 生效）
#ifndef SIMURO5_BRANCH_TRACE_HPP
#define SIMURO5_BRANCH_TRACE_HPP
#ifdef SIMURO5_BRANCH_TRACE

#include "simuro5/motion.hpp"

namespace simuro5 {
namespace trace {
constexpr int kSlots = 16;
struct Mark { const void *robot; const char *file; int line; };
inline Mark g_marks[kSlots] = {};
inline void reset() { for (auto &m : g_marks) m = Mark{nullptr, nullptr, 0}; }
inline void mark(const void *r, const char *file, int line) {
    for (auto &m : g_marks) if (m.robot == r) { m.file = file; m.line = line; return; }
    for (auto &m : g_marks) if (!m.robot) { m = Mark{r, file, line}; return; }
}
inline const Mark *find(const void *r) {
    for (auto &m : g_marks) if (m.robot == r) return &m;
    return nullptr;
}
}  // namespace trace

namespace motion {
template <class... A> void position_tr(const char *f, int l, RobotState &r, A... a) {
    trace::mark(&r, f, l); position(r, a...);
}
template <class... A> bool position_aligned_tr(const char *f, int l, RobotState &r, A... a) {
    trace::mark(&r, f, l); return position_aligned(r, a...);
}
inline void stop_tr(const char *f, int l, RobotState &r) { trace::mark(&r, f, l); stop(r); }
inline void chase_ball_tr(const char *f, int l, RobotState &r, const BallState &p) {
    trace::mark(&r, f, l); chase_ball(r, p);
}
template <class... A> void follow_route_tr(const char *f, int l, RobotState &r, A &&...a) {
    trace::mark(&r, f, l); follow_route(r, a...);
}
}  // namespace motion
}  // namespace simuro5

#ifndef SIMURO5_TRACE_NO_MACROS
#define position(r, ...)         position_tr(__FILE__, __LINE__, r, __VA_ARGS__)
#define position_aligned(r, ...) position_aligned_tr(__FILE__, __LINE__, r, __VA_ARGS__)
#define stop(r)                  stop_tr(__FILE__, __LINE__, r)
#define chase_ball(r, p)         chase_ball_tr(__FILE__, __LINE__, r, p)
#define follow_route(r, ...)     follow_route_tr(__FILE__, __LINE__, r, __VA_ARGS__)
#define TRACE_MARK(r)            ::simuro5::trace::mark(&(r), __FILE__, __LINE__)
#endif  // SIMURO5_TRACE_NO_MACROS

#else
#define TRACE_MARK(r) ((void)0)
#endif  // SIMURO5_BRANCH_TRACE
#endif
