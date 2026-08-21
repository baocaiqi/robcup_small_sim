#include "simuro5/strategy.hpp"
#include "simuro5/roles.hpp"
#include "simuro5/motion.hpp"
#include <cmath>

namespace simuro5 {

void Strategy::run(WorldModel &wm) {
    // 1. 局势分析
    sit_.update_stand_points(wm);
    Situation sit = sit_.analyze(wm);
    wm.threat_level = sit.threat_level;
    wm.we_have_ball = sit.we_have_ball;

    // 2. 角色分配
    ra_.assign(wm);

    // 3. 按角色执行（薄壳调度）
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        switch (wm.role[i]) {
            case ROLE_GOALIE:   run_goalie(wm, i); break;
            case ROLE_ACTIVE:   run_active(wm, i); break;
            case ROLE_PASSIVE:  run_passive(wm, i); break;
            case ROLE_ASSIST:   run_assist(wm, i); break;
            case ROLE_MIDFIELD: run_midfield(wm, i); break;
            default:            motion::stop(wm.home[i]); break;
        }
    }
}

}  // namespace simuro5
