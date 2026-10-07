// dll_blue.cpp — 蓝队导出壳：文件名须为 Strategy4Blue.dll，接口签名不可改
#include <windows.h>
#include <string.h>
#include "simuro5/simuro_interface.hpp"
#include "simuro5/team.hpp"
#include "simuro5/world_model.hpp"
#include "simuro5/strategy.hpp"
#include "simuro5/formation.hpp"
#include "simuro5/blackbox.hpp"

using namespace simuro5;

namespace {
TeamContext g_ctx{/*is_blue=*/true};
WorldModel g_wm;
Strategy g_strategy;
}  // namespace

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID) {
    (void)hModule; (void)reason;
    return TRUE;
}

STRATEGY4BLUE_API void SetBlueTeamName(char* teamName) {
    strcpy(teamName, "Hnnu");
}

STRATEGY4BLUE_API void SetFormerRobots(PlayMode gameState, Robot robots[]) {
    formation_former(g_ctx, gameState, robots);
    bb::placement("former", (long)gameState, robots, PLAYERS_PER_SIDE);
}

STRATEGY4BLUE_API void SetLaterRobots(PlayMode gameState, Robot formerRobots[],
                                      Vector3D ball, Robot laterRobots[]) {
    bb::setball((long)gameState, ball.x, ball.y);
    formation_later(g_ctx, gameState, formerRobots, ball, laterRobots);
    bb::placement("later", (long)gameState, laterRobots, PLAYERS_PER_SIDE);
}

STRATEGY4BLUE_API void SetBall(PlayMode gameState, Vector3D *pBall) {
    formation_set_ball(g_ctx, gameState, pBall);
    bb::setball((long)gameState, pBall->x, pBall->y);
}

STRATEGY4BLUE_API void RunStrategy(Environment *pEnv) {
    g_wm.update(pEnv, g_ctx);
    g_strategy.run(g_wm);
    bb::frame((long)pEnv->gameState, (long)pEnv->whosBall,
              pEnv->currentBall.pos.x, pEnv->currentBall.pos.y,
              g_wm.ball.vx, g_wm.ball.vy, g_wm.we_have_ball ? 1 : 0,
              g_wm.in_penalty_exec ? 1 : 0, g_wm.home[0].x, g_wm.home[1].x);
    {
        double hx[PLAYERS_PER_SIDE], hy[PLAYERS_PER_SIDE], hr[PLAYERS_PER_SIDE];
        double ox[PLAYERS_PER_SIDE], oy[PLAYERS_PER_SIDE], orr[PLAYERS_PER_SIDE];
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            hx[i] = g_wm.home[i].x; hy[i] = g_wm.home[i].y; hr[i] = g_wm.home[i].rot;
            ox[i] = g_wm.opp[i].x;  oy[i] = g_wm.opp[i].y;  orr[i] = g_wm.opp[i].rot;
        }
        bb::robots(g_ctx.is_blue ? 1 : 0, hx, hy, hr, g_wm.role, PLAYERS_PER_SIDE);
        bb::opponents(ox, oy, orr, PLAYERS_PER_SIDE);
    }
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        pEnv->home[i].velocityLeft = g_wm.home[i].vl;
        pEnv->home[i].velocityRight = g_wm.home[i].vr;
    }
}
