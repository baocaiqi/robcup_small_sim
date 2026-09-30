// ============================================================
// dll_yellow.cpp — 黄队导出壳（编译时定义 STRATEGY4YELLOW_EXPORTS）
// 与蓝队共享同一套核心代码，仅队伍颜色不同。
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
TeamContext g_ctx{/*is_blue=*/false};
WorldModel g_wm;
Strategy g_strategy;
}  // namespace

BOOL APIENTRY DllMain(HMODULE hModule, DWORD reason, LPVOID) {
    (void)hModule; (void)reason;
    return TRUE;
}

// ⚠️ 黄队报「DEMO Yellow」——2026-09-30 用户指令：这份黄队 DLL 就是用来**替换官方 demo** 的
//    （平台黄队槽位 `C:\Strategy\Strategy4Yellow.dll` 原来是官方 DEMO Yellow），
//    保留 demo 队名 ⇒ 平台控制台显示与 `.rlg` 命名习惯（`…-5-DEMO Yellow-Hnnu.rlg`）完全不变。
//    ⚠️ 但此后带此名的比赛跑的是**我们的策略**，别再当成官方 demo 的成绩来读。
STRATEGY4YELLOW_API void SetYellowTeamName(char* teamName) {
    strcpy(teamName, "DEMO Yellow");   // 用户指令（2026-09-30）：替换官方 demo，沿用 demo 队名
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
