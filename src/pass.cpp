#include "simuro5/pass.hpp"
#include "simuro5/geometry.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

namespace {
// 路线 (bx,by)->(tx,ty) 是否被某个对手机器人挡住（距离阈值 15cm）
bool route_blocked(const WorldModel &wm, double bx, double by, double tx, double ty) {
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = point_to_segment_dist(wm.opp[i].x, wm.opp[i].y, bx, by, tx, ty);
        if (d < 15.0) return true;
    }
    return false;
}
}  // namespace

PassPlan plan_pass(const WorldModel &wm, int passer_id) {
    PassPlan plan;
    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;

    // 选接应者：除持球者外，离对方球门最近、且不在射门角度内的队友
    int best = -1;
    double best_d = 1e9;
    for (int id = 0; id < PLAYERS_PER_SIDE; ++id) {
        if (id == passer_id) continue;
        double d = std::fabs(wm.home[id].x - ctx.opp_goal_x());
        if (d < best_d) { best_d = d; best = id; }
    }
    if (best < 0) return plan;

    double rx = wm.home[best].x, ry = wm.home[best].y;
    // 接应点：队友位置前方(朝球门方向) 5cm
    double ad = ctx.attack_dir();
    double rx2 = rx + ad * 5.0;

    // 路线被挡或太远就不传
    if (dist(bx, by, rx, ry) > 60.0) return plan;
    if (route_blocked(wm, bx, by, rx2, ry)) return plan;

    plan.viable = true;
    plan.receiver_id = best;
    plan.target_x = rx2;
    plan.target_y = ry;
    return plan;
}

}  // namespace simuro5
