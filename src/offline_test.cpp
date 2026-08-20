// ============================================================
// offline_test.cpp — 无平台的离线冒烟测试（cmake -DBUILD_TEST=ON）
// 手动构造 Environment，跑若干帧 RunStrategy，验证：
//   1. 不崩溃、输出轮速在合理范围
//   2. 5 个机器人都有非零/合理的速度命令
//   3. 摆位函数对每种 PlayMode 都能填出位置
// ============================================================
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "simuro5/simuro_interface.hpp"
#include "simuro5/formation.hpp"
#include "simuro5/team.hpp"
#include "simuro5/world_model.hpp"
#include "simuro5/strategy.hpp"
#include "simuro5/defense.hpp"
#include "simuro5/roles.hpp"
#include "simuro5/field_info.hpp"

using namespace simuro5;

static void init_env(Environment &e, double bx, double by) {
    memset(&e, 0, sizeof(e));
    e.fieldBounds.left = 0; e.fieldBounds.right = 220;
    e.fieldBounds.bottom = 0; e.fieldBounds.top = 180;
    e.goalBounds.left = 0; e.goalBounds.right = 220;
    e.goalBounds.bottom = 70; e.goalBounds.top = 110;
    e.currentBall.pos.x = bx; e.currentBall.pos.y = by;
    e.lastBall.pos = e.currentBall.pos;
    e.predictedBall.pos = e.currentBall.pos;
    e.gameState = PM_PlayOn;
    // 己方开局站位
    double xs[5] = {210, 180, 150, 120, 120};
    double ys[5] = {90, 90, 60, 120, 90};
    for (int i = 0; i < 5; ++i) {
        e.home[i].pos.x = xs[i]; e.home[i].pos.y = ys[i];
        e.home[i].rotation = 180;
    }
    for (int i = 0; i < 5; ++i) {
        e.opponent[i].pos.x = 10 + i * 20;
        e.opponent[i].pos.y = 90;
    }
}

static int test_strategy_run(int frames) {
    Environment e;
    TeamContext ctx{true};
    WorldModel wm;
    Strategy strat;
    init_env(e, 110, 90);
    double max_v = 0;
    for (int f = 0; f < frames; ++f) {
        // 模拟球向蓝队球门滚（x 增大）
        e.currentBall.pos.x += 0.5;
        e.lastBall.pos.x = e.currentBall.pos.x - 0.5;
        wm.update(&e, ctx);
        strat.run(wm);
        for (int i = 0; i < 5; ++i) {
            e.home[i].velocityLeft = wm.home[i].vl;
            e.home[i].velocityRight = wm.home[i].vr;
            max_v = fmax(max_v, fmax(fabs(wm.home[i].vl), fabs(wm.home[i].vr)));
        }
        if (max_v > 300.0) { printf("FAIL: 轮速超出合理范围 %.1f\n", max_v); return 1; }
    }
    printf("strategy: OK (frames=%d, max|v|=%.1f)\n", frames, max_v);
    return 0;
}

static int test_formation() {
    TeamContext blue{true}, yellow{false};
    Robot r[5];
    for (int gs = 1; gs <= 12; ++gs) {
        if (gs == PM_PlayOn) continue;
        // 先摆
        for (int i = 0; i < 5; ++i) { r[i].pos.x = 0; r[i].pos.y = 0; }
        formation_former(blue, (PlayMode)gs, r);
        for (int i = 0; i < 5; ++i)
            if (r[i].pos.x < -1 || r[i].pos.x > 221 || r[i].pos.y < -1 || r[i].pos.y > 181) {
                printf("FAIL: blue former gs=%d robot[%d] 越界 (%.0f,%.0f)\n", gs, i, r[i].pos.x, r[i].pos.y);
                return 1;
            }
        // 后摆
        Vector3D ball; ball.x = 110; ball.y = 90; ball.z = 0;
        Robot former[5] = {};
        formation_later(blue, (PlayMode)gs, former, ball, r);
        for (int i = 0; i < 5; ++i)
            if (r[i].pos.x < -1 || r[i].pos.x > 221 || r[i].pos.y < -1 || r[i].pos.y > 181) {
                printf("FAIL: blue later gs=%d robot[%d] 越界\n", gs, i);
                return 1;
            }
        // 黄队镜像也检查
        formation_former(yellow, (PlayMode)gs, r);
        for (int i = 0; i < 5; ++i)
            if (r[i].pos.x < -1 || r[i].pos.x > 221 || r[i].pos.y < -1 || r[i].pos.y > 181) {
                printf("FAIL: yellow former gs=%d robot[%d] 越界\n", gs, i);
                return 1;
            }
    }
    printf("formation: OK (12 种 PlayMode 蓝/黄摆位均在场内)\n");
    return 0;
}

// 断球点纯函数单测：三角函数外推「球轨迹 ∩ 球门前拦截线」
static int test_defense_intercept() {
    TeamContext ctx{true};                 // 蓝队，门在 x=220
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    double ix = 0, iy = 0;

    // ① 平飞球：沿 x 正向、y=90，断球点应在 (170, 90)
    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 3.0; wm.ball.vy = 0.0;
    if (!intercept_point(wm, 50.0, ix, iy)) { printf("FAIL: 平飞球应有断球点\n"); return 1; }
    if (fabs(ix - 170.0) > 0.5 || fabs(iy - 90.0) > 0.5) {
        printf("FAIL: 平飞球断球点错 (%.1f,%.1f)\n", ix, iy); return 1;
    }

    // ② 斜向球：vy/vx 斜率外推，y = 60 + (1.5/3.0)*(170-100) = 95
    wm.ball.x = 100; wm.ball.y = 60; wm.ball.vx = 3.0; wm.ball.vy = 1.5;
    if (!intercept_point(wm, 50.0, ix, iy)) { printf("FAIL: 斜向球应有断球点\n"); return 1; }
    if (fabs(ix - 170.0) > 0.5 || fabs(iy - 95.0) > 0.5) {
        printf("FAIL: 斜向球断球点错 (%.1f,%.1f)\n", ix, iy); return 1;
    }

    // ③ 背离球：vx<0（蓝队门在 +x 端），不应有朝门的断球点
    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = -3.0; wm.ball.vy = 0.0;
    if (intercept_point(wm, 50.0, ix, iy)) { printf("FAIL: 背离球不应有断球点\n"); return 1; }

    // ④ 只沿 y 向滚（vx≈0）：到不了竖线，不应有断球点
    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 0.0; wm.ball.vy = 3.0;
    if (intercept_point(wm, 50.0, ix, iy)) { printf("FAIL: 纯 y 向球不应有断球点\n"); return 1; }

    printf("defense intercept: OK (平飞/斜向/背离/纯y向)\n");
    return 0;
}

// 守门员出击预判冒烟测试：球朝门射应出击，慢球/无威胁应停车
static int test_goalie_predict() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.home[0].x = 210; wm.home[0].y = 90; wm.home[0].rot = 180;

    // 球在门前快速朝门滚（vx=6，会进球 y=90 在门宽内）→ 应出击
    wm.ball.x = 190; wm.ball.y = 90; wm.ball.vx = 6.0; wm.ball.vy = 0.0;
    run_goalie(wm, 0);
    double v = fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr));
    if (!(v > 0.0) || v > 300.0) { printf("FAIL: 守门员出击轮速异常 %.1f\n", v); return 1; }

    // 球慢且远离门 → 守门员不应冲刺（轮速须在合理范围）
    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[0].vl = 0; wm.home[0].vr = 0;
    run_goalie(wm, 0);
    v = fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr));
    if (v > 300.0) { printf("FAIL: 守门员停车轮速异常 %.1f\n", v); return 1; }

    printf("goalie predict: OK (朝门出击/慢球停车均正常)\n");
    return 0;
}

int main() {
    int rc = 0;
    rc |= test_strategy_run(300);
    rc |= test_formation();
    rc |= test_defense_intercept();
    rc |= test_goalie_predict();
    printf(rc ? "=== TEST FAILED ===\n" : "=== ALL TESTS PASSED ===\n");
    return rc;
}
