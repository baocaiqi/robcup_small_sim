// ============================================================
// dll_blue.cpp — 蓝队导出壳（编译时定义 STRATEGY4BLUE_EXPORTS）
// 平台加载 Strategy4Blue.dll 后按 C++ mangled 名调用 5 个接口。
// 注意：文件名必须保持 Strategy4Blue.dll，接口签名不能改。
// ============================================================
#include <windows.h>
#include <string.h>
#include "simuro5/simuro_interface.hpp"
#include "simuro5/team.hpp"
#include "simuro5/world_model.hpp"
#include "simuro5/strategy.hpp"
#include "simuro5/formation.hpp"

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

// ⚠️ 改成你们自己的队名（平台控制台会显示）
STRATEGY4BLUE_API void SetBlueTeamName(char* teamName) {
    strcpy(teamName, "MyTeam-Blue");
}

STRATEGY4BLUE_API void SetFormerRobots(PlayMode gameState, Robot robots[]) {
    formation_former(g_ctx, gameState, robots);
}

STRATEGY4BLUE_API void SetLaterRobots(PlayMode gameState, Robot formerRobots[],
                                      Vector3D ball, Robot laterRobots[]) {
    formation_later(g_ctx, gameState, formerRobots, ball, laterRobots);
}

STRATEGY4BLUE_API void SetBall(PlayMode gameState, Vector3D *pBall) {
    formation_set_ball(g_ctx, gameState, pBall);
}

STRATEGY4BLUE_API void RunStrategy(Environment *pEnv) {
    g_wm.update(pEnv, g_ctx);
    g_strategy.run(g_wm);
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        pEnv->home[i].velocityLeft = g_wm.home[i].vl;
        pEnv->home[i].velocityRight = g_wm.home[i].vr;
    }
}
