#include "simuro5/role_assignment.hpp"
#include "simuro5/geometry.hpp"
#include <algorithm>
#include <cmath>

namespace simuro5 {

// 效用：越小越适合该角色（取距离，带当前角色惯性 +8）
double RoleAssignment::_utility(int id, const WorldModel &wm, int role) const {
    const double &rx = wm.home[id].x, &ry = wm.home[id].y;
    double d;
    switch (role) {
        case ROLE_ACTIVE:
            d = dist(rx, ry, wm.ball.x, wm.ball.y);
            break;
        case ROLE_PASSIVE:
            d = dist(rx, ry, wm.passive_x, wm.passive_y);
            break;
        case ROLE_ASSIST:
            d = dist(rx, ry, wm.assist_x, wm.assist_y);
            break;
        case ROLE_MIDFIELD:
            d = dist(rx, ry, wm.mid_x, wm.mid_y);
            break;
        default:
            d = 1e9;
    }
    // 角色惯性：维持当前角色更平滑（防止每帧换人）
    if (wm.role[id] == role) d -= 8.0;
    return d;
}

void RoleAssignment::assign(WorldModel &wm) {
    wm.role[0] = ROLE_GOALIE;

    // 贪心：4 个角色依次取效用最低且未被占用的机器人
    bool taken[PLAYERS_PER_SIDE] = {false};
    taken[0] = true;
    int roles[4] = {ROLE_ACTIVE, ROLE_PASSIVE, ROLE_ASSIST, ROLE_MIDFIELD};

    for (int r = 0; r < 4; ++r) {
        int best_id = -1;
        double best_u = 1e18;
        for (int id = 1; id < PLAYERS_PER_SIDE; ++id) {
            if (taken[id]) continue;
            double u = _utility(id, wm, roles[r]);
            if (u < best_u) { best_u = u; best_id = id; }
        }
        if (best_id < 0) break;
        taken[best_id] = true;
        wm.role[best_id] = roles[r];
        if (roles[r] == ROLE_ACTIVE) _last_active_idx = best_id;
    }

    // 兜底：未分配的给 PASSIVE
    for (int id = 1; id < PLAYERS_PER_SIDE; ++id)
        if (!taken[id]) wm.role[id] = ROLE_PASSIVE;
}

}  // namespace simuro5
