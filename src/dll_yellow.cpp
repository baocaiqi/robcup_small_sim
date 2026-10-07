// dll_yellow.cpp — 黄队导出壳：与蓝队共用核心代码；队名与蓝位一致报「Hnnu」，只影响平台显示
#include <windows.h>
#include <string.h>
#include "simuro5/simuro_interface.hpp"
#include "simuro5/team.hpp"
#include "simuro5/world_model.hpp"
#include "simuro5/strategy.hpp"
#include "simuro5/formation.hpp"

using namespace simuro5;

namespace {
TeamContext g_ctx{/*is_blue=*/false};
WorldModel g_wm;
Strategy g_strategy;
}  // namespace

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID) {
    (void)hModule; (void)reason;
    return TRUE;
}

STRATEGY4YELLOW_API void SetYellowTeamName(char* teamName) {
    strcpy(teamName, "Hnnu");
}

STRATEGY4YELLOW_API void SetFormerRobots(PlayMode gameState, Robot robots[]) {
    formation_former(g_ctx, gameState, robots);
}

STRATEGY4YELLOW_API void SetLaterRobots(PlayMode gameState, Robot formerRobots[],
                                        Vector3D ball, Robot laterRobots[]) {
    formation_later(g_ctx, gameState, formerRobots, ball, laterRobots);
}

STRATEGY4YELLOW_API void SetBall(PlayMode gameState, Vector3D *pBall) {
    formation_set_ball(g_ctx, gameState, pBall);
}

STRATEGY4YELLOW_API void RunStrategy(Environment *pEnv) {
    g_wm.update(pEnv, g_ctx);
    g_strategy.run(g_wm);
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        pEnv->home[i].velocityLeft = g_wm.home[i].vl;
        pEnv->home[i].velocityRight = g_wm.home[i].vr;
    }
}
