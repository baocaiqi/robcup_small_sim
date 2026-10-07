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

// 黄队报「Hnnu」——2026-10-06 晚用户指令：「是 HNNU 替换到黄队」（原话）。
//    历史：2026-09-30 用户指令曾让我们"替换官方 demo 并沿用 demo 队名（DEMO Yellow）"，
//    目的是让平台控制台/`.rlg` 命名与以前一样（`…-5-DEMO Yellow-Hnnu.rlg`）。
//    ⚠️ 但那个名字会让**我们自己的黄位成绩被误读成官方 demo 的成绩**（2026-10-06 晚真出过
//    这次误会：黄位换成本构建后日志变 `DEMO Yellow-Hnnu`，被当成"官方 demo 混进来了"）。
//    ⇒ 2026-10-06 晚改成与蓝位一致的 `Hnnu`：黄位日志从此读作 `Hnnu-<对手名>`，一眼看得出是我们。
//    注意：改这个只影响**平台显示的队名**，不影响任何战术逻辑。
STRATEGY4YELLOW_API void SetYellowTeamName(char* teamName) {
    strcpy(teamName, "Hnnu");   // 用户指令（2026-10-06）：我方黄位也报 Hnnu
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
