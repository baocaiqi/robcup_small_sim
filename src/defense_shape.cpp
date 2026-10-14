// defense_shape.cpp — 失球权时"球门侧保底人数"（参考对手防守站位思想，自研实现）
//
// 比喻：小区门口至少得留一个保安。球被抢走、大家都还在前面追的时候，
//   总得有一个人先跑回"球和球门之间"把路堵上，而不是全队跟在球屁股后面。
// 真机取证（2026-10-14 五场 rlg，我们黄位 vs 对手）：球在己半场时
//   对手"站在球门侧的非门将人数"平均 0.82~1.38，我们只有 0.77~0.93；
//   "球-门连线上最近一个人"的距离中位数，对手 11.8~16.3cm，我们 17.5~21.8cm。
//   这两个是**观察值**，不是我们照抄的值；下面的旋钮全是我们自己定的起点。
// 和对手的区别：对手是 SPV2 选点（候选点里挑威胁场 phi 低的、门线上找 phi 最高的 y）；
//   我们只借"门线最危险 y"这一条（wm.threat.line_max_y，按 kShapeThreatBlend 混进护门点的瞄准 y），
//   选人用"谁回位最快"，路线上会碰球就先横绕（防止把球顶回自家门）。还没真机验证。
#include "simuro5/defense_shape.hpp"
#include "simuro5/role_assignment.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/motion.hpp"
#define TUNABLE_PREFIX "shape."
#include "simuro5/tunable.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

// 总开关：0=关（回滚），1=开
TUNABLE(kShapeOn, 1.0);
// 球门侧至少要有几个非门将、非 ACTIVE 的队员
TUNABLE(kShapeMinBehind, 1.0);
// 球离己门线在 [Lo, Hi] cm 之间才管（太近交给门将/门前协防，太远不算己半场）
TUNABLE(kShapeBallGoalLo, 40.0);
TUNABLE(kShapeBallGoalHi, 130.0);
// "人在球门侧"的余量：比球更靠己门至少这么多 cm 才算
TUNABLE(kShapeBehindMargin, 8.0);
// 护门点：在球-门连线上，离球 kShapeDepth cm（但不超过球门距离的一半）
TUNABLE(kShapeDepth, 40.0);
// 护门点瞄准 y = 90 + Blend×(威胁场门线最危险 y − 90)；0 = 纯瞄门心
TUNABLE(kShapeThreatBlend, 0.5);
// 回位路线离球太近（cm）就先横绕，绕行横向偏移（cm）
TUNABLE(kShapePathClear, 14.0);
TUNABLE(kShapeDetourLat, 20.0);
// 选中后至少坚持多少帧（防止每帧换人来回抖）
TUNABLE(kShapeHoldFrames, 15.0);

namespace {

double seg_dist(double x0, double y0, double x1, double y1, double px, double py) {
    const double dx = x1 - x0, dy = y1 - y0, l2 = dx * dx + dy * dy;
    const double t = (l2 < 1e-9) ? 0.0 : clamp(((px - x0) * dx + (py - y0) * dy) / l2, 0.0, 1.0);
    return std::hypot(x0 + t * dx - px, y0 + t * dy - py);
}

struct ShapeState { int id = -1; int hold = 0; };
ShapeState &state(const WorldModel &wm) {
    static ShapeState s[2];                       // 黄蓝两队各一份，同进程里不会串
    return s[wm.ctx.is_blue ? 1 : 0];
}

}  // namespace

int defense_shape_apply(WorldModel &wm, const bool skip[PLAYERS_PER_SIDE]) {
    ShapeState &st = state(wm);
    if (st.hold > 0) --st.hold;
    if (kShapeOn < 0.5 || wm.runtime_phase != RuntimePhase::Running || wm.in_penalty_exec ||
        wm.we_have_ball || !wm.ball.valid) { st.id = -1; return -1; }
    const TeamContext &ctx = wm.ctx;
    const double bx = wm.ball.x, by = wm.ball.y;
    const double bg = ctx.dist_our_goal(bx);
    if (bg < kShapeBallGoalLo || bg > kShapeBallGoalHi) { st.id = -1; return -1; }

    auto eligible = [&](int i) {
        const int rl = wm.role[i];
        return !skip[i] && (rl == ROLE_PASSIVE || rl == ROLE_ASSIST || rl == ROLE_MIDFIELD);
    };
    auto behind = [&](int i) {
        return ctx.dist_our_goal(wm.home[i].x) < bg - kShapeBehindMargin;
    };
    int n_behind = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i)
        if (wm.role[i] != ROLE_GOALIE && wm.role[i] != ROLE_ACTIVE && behind(i)) ++n_behind;

    // 已选中的人还在回位途中：保持（人一旦站到球门侧，它也计入了 n_behind，所以要单独判）
    const bool keep = st.id >= 0 && st.hold > 0 && eligible(st.id);
    if (!keep) {
        if (n_behind >= (int)kShapeMinBehind) { st.id = -1; return -1; }
        // 选人：只从"人在球场侧"的里面挑，按到护门点的距离（≈回位时间）取最小
        st.id = -1;
        double best = 1e9;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            if (!eligible(i) || behind(i)) continue;
            const double d = std::hypot(wm.home[i].x - bx, wm.home[i].y - by);
            if (d < best) { best = d; st.id = i; }
        }
        if (st.id < 0) return -1;
        st.hold = (int)kShapeHoldFrames;
    }

    // 护门点：球 → 瞄准点（己门线，y 混入威胁场门线最危险 y）这条线上，离球 kShapeDepth
    double aim_y = 90.0;
    if (wm.threat.valid) aim_y += kShapeThreatBlend * (wm.threat.line_max_y - 90.0);
    aim_y = clamp(aim_y, goal_y_low(), goal_y_high());
    const double gx = ctx.our_goal_x();
    double ux = gx - bx, uy = aim_y - by;
    const double L = std::hypot(ux, uy);
    if (L < 1e-6) return -1;
    ux /= L; uy /= L;
    const double depth = std::min((double)kShapeDepth, 0.5 * L);
    double tx = bx + ux * depth, ty = by + uy * depth;
    // 规则红线：非门将不得进己方门区（裁判口径 + 余量）
    if (in_goal_area_rule(ctx, tx, ty, 8.0, 6.0))
        tx = gx + ctx.attack_dir() * 25.0;
    tx = clamp(tx, 6.0, TeamContext::FIELD_LENGTH - 6.0);
    ty = clamp(ty, 6.0, TeamContext::FIELD_WIDTH - 6.0);

    RobotState &r = wm.home[st.id];
    // 人在球场侧、直线回位会擦到球 → 先绕到球的侧面（绕向离人近的那侧），绝不从球身上穿过去
    if (seg_dist(r.x, r.y, tx, ty, bx, by) < kShapePathClear) {
        double side = (std::fabs(r.y - by) > 1.0) ? (r.y > by ? 1.0 : -1.0) : (by < 90.0 ? 1.0 : -1.0);
        double wy = by + side * kShapeDetourLat;
        if (wy < 6.0 || wy > TeamContext::FIELD_WIDTH - 6.0) wy = by - side * kShapeDetourLat;
        tx = bx; ty = wy;
    }
    const double far = std::hypot(tx - r.x, ty - r.y);
    motion::position(r, tx, ty, far > 30.0 ? motion::TM_PASS : motion::TM_STOP);
    return st.id;
}

}  // namespace simuro5
