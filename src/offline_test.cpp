// offline_test.cpp — 无平台的离线冒烟测试（cmake -DBUILD_TEST=ON）
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <cstdint>
#include <chrono>
#include <limits>
#include <algorithm>
#include "simuro5/simuro_interface.hpp"
#include "simuro5/formation.hpp"
#include "simuro5/team.hpp"
#include "simuro5/world_model.hpp"
#include "simuro5/strategy.hpp"
#include "simuro5/defense.hpp"
#include "simuro5/route.hpp"
#include "simuro5/motion.hpp"
#include "simuro5/roles.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/pass.hpp"
#include "simuro5/shoot.hpp"
#include "simuro5/hungarian.hpp"
#include "simuro5/tunable.hpp"   // 参数注册表：--params 注入 + 按当前旋钮值断言

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
        for (int i = 0; i < 5; ++i) { r[i].pos.x = 0; r[i].pos.y = 0; }
        formation_former(blue, (PlayMode)gs, r);
        for (int i = 0; i < 5; ++i)
            if (r[i].pos.x < -1 || r[i].pos.x > 221 || r[i].pos.y < -1 || r[i].pos.y > 181) {
                printf("FAIL: blue former gs=%d robot[%d] 越界 (%.0f,%.0f)\n", gs, i, r[i].pos.x, r[i].pos.y);
                return 1;
            }
        Vector3D ball; ball.x = 110; ball.y = 90; ball.z = 0;
        Robot former[5] = {};
        formation_later(blue, (PlayMode)gs, former, ball, r);
        for (int i = 0; i < 5; ++i)
            if (r[i].pos.x < -1 || r[i].pos.x > 221 || r[i].pos.y < -1 || r[i].pos.y > 181) {
                printf("FAIL: blue later gs=%d robot[%d] 越界\n", gs, i);
                return 1;
            }
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

    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 3.0; wm.ball.vy = 0.0;
    if (!intercept_point(wm, 50.0, ix, iy)) { printf("FAIL: 平飞球应有断球点\n"); return 1; }
    if (fabs(ix - 170.0) > 0.5 || fabs(iy - 90.0) > 0.5) {
        printf("FAIL: 平飞球断球点错 (%.1f,%.1f)\n", ix, iy); return 1;
    }

    wm.ball.x = 100; wm.ball.y = 60; wm.ball.vx = 3.0; wm.ball.vy = 1.5;
    if (!intercept_point(wm, 50.0, ix, iy)) { printf("FAIL: 斜向球应有断球点\n"); return 1; }
    if (fabs(ix - 170.0) > 0.5 || fabs(iy - 95.0) > 0.5) {
        printf("FAIL: 斜向球断球点错 (%.1f,%.1f)\n", ix, iy); return 1;
    }

    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = -3.0; wm.ball.vy = 0.0;
    if (intercept_point(wm, 50.0, ix, iy)) { printf("FAIL: 背离球不应有断球点\n"); return 1; }

    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 0.0; wm.ball.vy = 3.0;
    if (intercept_point(wm, 50.0, ix, iy)) { printf("FAIL: 纯 y 向球不应有断球点\n"); return 1; }

// 边界反弹非理想镜面：实测法向 0.66、切向 0.81，期望值 32.6/147.4 由此而来
    double ry = 0.0;
    predict_y_at_x_reflect(100.0, 20.0, 2.0, -2.0, 160.0, ry);   // 撞底墙
    if (fabs(ry - 32.6) > 0.3) {
        printf("FAIL: 撞底墙反射(实测系数) y=%.1f 应 32.6（镜面才会给 40）\n", ry); return 1;
    }
    predict_y_at_x_reflect(100.0, 160.0, 2.0, 2.0, 160.0, ry);   // 撞顶墙
    if (fabs(ry - 147.4) > 0.3) {
        printf("FAIL: 撞顶墙反射(实测系数) y=%.1f 应 147.4（镜面才会给 140）\n", ry); return 1;
    }

    printf("defense intercept: OK (平飞/斜向/背离/纯y向/实测系数反弹)\n");
    return 0;
}

// 守门员出击预判冒烟：球朝门射应出击，慢球/无威胁应停车
static int test_goalie_predict() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.home[0].x = 210; wm.home[0].y = 90; wm.home[0].rot = 180;

    wm.ball.x = 190; wm.ball.y = 90; wm.ball.vx = 6.0; wm.ball.vy = 0.0;
    run_goalie(wm, 0);
    double v = fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr));
    if (!(v > 0.0) || v > 300.0) { printf("FAIL: 守门员出击轮速异常 %.1f\n", v); return 1; }

    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[0].vl = 0; wm.home[0].vr = 0;
    run_goalie(wm, 0);
    v = fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr));
    if (v > 300.0) { printf("FAIL: 守门员停车轮速异常 %.1f\n", v); return 1; }

    printf("goalie predict: OK (朝门出击/慢球停车均正常)\n");
    return 0;
}

// 可达性判断单测：近处采纳截点、远处回退卡位
static int test_defense_reach() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.passive_x = 150; wm.passive_y = 70;

    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 3.0; wm.ball.vy = 0.0;

    wm.home[1].x = 168; wm.home[1].y = 90;
    DefensePlan a = plan_defense(wm, 1);
    if (fabs(a.target_x - wm.passive_x) < 0.5) {
        printf("FAIL: 近处应采纳截点而非卡位 (%.1f,%.1f)\n", a.target_x, a.target_y); return 1;
    }

    wm.home[1].x = 20; wm.home[1].y = 90;
    DefensePlan b = plan_defense(wm, 1);
    if (fabs(b.target_x - wm.passive_x) > 0.5 || fabs(b.target_y - wm.passive_y) > 0.5) {
        printf("FAIL: 远处应回退卡位 (%.1f,%.1f)\n", b.target_x, b.target_y); return 1;
    }

    printf("defense reach: OK (近处截断/远处回退)\n");
    return 0;
}

// 断球点落在球来路上时必须给出「迎球朝向」= 球来向的反方向
static int test_defense_face_incoming() {
    WorldModel wm;
    wm.ctx = TeamContext{true};          // 蓝队守 x=220
    wm.ball.valid = true;
    wm.passive_x = 150; wm.passive_y = 70;

    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 3.0; wm.ball.vy = 0.0;
    wm.home[1].x = 168; wm.home[1].y = 90;
    DefensePlan a = plan_defense(wm, 1);
    if (!a.face_incoming) { printf("FAIL: 断球点应给出迎球朝向\n"); return 1; }
    if (fabs(angle_diff(a.aim_rot, 180.0)) > 1e-6) {
        printf("FAIL: 正对来球应朝 180°，实际 %.1f\n", a.aim_rot); return 1;
    }

    wm.ball.x = 100; wm.ball.y = 60; wm.ball.vx = 3.0; wm.ball.vy = 1.5;
    wm.home[1].x = 168; wm.home[1].y = 95;
    DefensePlan b = plan_defense(wm, 1);
    const double want = angle_to(0.0, 0.0, -3.0, -1.5);
    if (!b.face_incoming || fabs(angle_diff(b.aim_rot, want)) > 1e-6) {
        printf("FAIL: 斜向球迎球朝向错 face=%d aim=%.1f 应 %.1f\n",
               (int)b.face_incoming, b.aim_rot, want); return 1;
    }

    wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[1].x = 168; wm.home[1].y = 90;
    DefensePlan c = plan_defense(wm, 1);
    if (c.face_incoming) { printf("FAIL: 球停着不该给迎球朝向\n"); return 1; }

    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 3.0; wm.ball.vy = 0.0;
    wm.home[1].x = 170; wm.home[1].y = 90; wm.home[1].rot = 0.0;
    DefensePlan d0 = plan_defense(wm, 1);                 // 先问"断球点在哪"（禁区纪律会把它推出己方禁区）
    wm.home[1].x = d0.target_x; wm.home[1].y = d0.target_y;   // 人站到点上
    wm.home[1].rot = 0.0;                                     // 但机头朝 +x（背对来球）
    DefensePlan d = plan_defense(wm, 1);
    if (!d.face_incoming) { printf("FAIL: 场景④应有迎球朝向\n"); return 1; }
    motion::arrive_facing(wm.home[1], d.target_x, d.target_y, d.aim_rot, 6.0, 12.0);
    const double common = fabs(wm.home[1].vl + wm.home[1].vr);
    if (!(wm.home[1].vl * wm.home[1].vr < 0.0) || common > 1e-9) {
        printf("FAIL: 到位迎球应原地转正 vl=%.1f vr=%.1f (点在 %.1f,%.1f 我在 %.1f,%.1f)\n",
               wm.home[1].vl, wm.home[1].vr, d.target_x, d.target_y, wm.home[1].x, wm.home[1].y);
        return 1;
    }

    printf("defense face incoming: OK (正对/斜向给朝向，停球不给，到位原地转正)\n");
    return 0;
}

// 会合点 = 沿球未来轨迹第一个「我比球早到 lead 帧」的点（球每帧只衰减 ~0.993，几乎不会自己停）
static int test_ball_meeting_point() {
    WorldModel wm;
    wm.ctx = TeamContext{true};
    wm.ball.valid = true;
    double mx = 0, my = 0, aim = 0;

    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    if (ball_meeting_point(wm, 130, 90, 2.0, 6.0, mx, my, aim)) {
        printf("FAIL: 停球不该有会合点\n"); return 1;
    }

    wm.ball.vx = 0.5;
    if (ball_meeting_point(wm, 130, 90, 2.0, 6.0, mx, my, aim)) {
        printf("FAIL: 慢球不该有会合点\n"); return 1;
    }

    wm.ball.vx = 5.0; wm.ball.vy = 0.0;
    if (!ball_meeting_point(wm, 140, 90, 2.0, 6.0, mx, my, aim)) {
        printf("FAIL: 正向来球应有会合点\n"); return 1;
    }
    if (fabs(my - 90.0) > 1e-9 || mx <= 100.0) {
        printf("FAIL: 会合点应在球轨迹前方 (%.1f,%.1f)\n", mx, my); return 1;
    }
    if (fabs(angle_diff(aim, 180.0)) > 1e-6) {   // 球朝 +x 来 → 机头应朝 -x
        printf("FAIL: 迎球朝向应 180°，实际 %.1f\n", aim); return 1;
    }
    {   // 用同一套衰减复算"球到会合点要几帧"，验证提前量确实成立
        const double dec = get_param("defense.kBallDecay", 0.993);
        double bx = 100.0, vx = 5.0;
        int frames = -1;
        for (int k = 1; k <= 40; ++k) {
            bx += vx; vx *= dec;
            if (bx >= mx - 1e-9) { frames = k; break; }
        }
        if (frames < 0) { printf("FAIL: 会合点不在 40 帧外推范围内\n"); return 1; }
        const double t_me = dist(140.0, 90.0, mx, my) / 2.0;
        if (t_me + 6.0 > (double)frames) {
            printf("FAIL: 会合点没留出提前量 t_me=%.2f+6 > 球到点 %d 帧\n", t_me, frames); return 1;
        }
    }

    if (ball_meeting_point(wm, 20, 20, 2.0, 6.0, mx, my, aim)) {
        printf("FAIL: 追不上的球不该给会合点 (%.1f,%.1f)\n", mx, my); return 1;
    }

    wm.ball.x = 100; wm.ball.y = 60; wm.ball.vx = 4.0; wm.ball.vy = 2.0;
    if (!ball_meeting_point(wm, 140, 80, 2.0, 6.0, mx, my, aim)) {
        printf("FAIL: 斜向球应有会合点\n"); return 1;
    }
    const double want = angle_to(0.0, 0.0, -4.0, -2.0);
    if (fabs(angle_diff(aim, want)) > 1e-6) {
        printf("FAIL: 斜向球迎球朝向错 aim=%.1f 应 %.1f\n", aim, want); return 1;
    }
    if (fabs((my - 60.0) * 4.0 - (mx - 100.0) * 2.0) > 2.0) {   // 近似落在斜率 2/4 的轨迹上
        printf("FAIL: 会合点偏离球轨迹 (%.1f,%.1f)\n", mx, my); return 1;
    }

    printf("ball meeting point: OK (停球/慢球/追不上不给，正向/斜向给点并留提前量)\n");
    return 0;
}

// 球-门连线护门点单测（远球连线 / 中近拦截线 / 贴门堵射）
static int test_goal_cover() {
    TeamContext ctx{true};   // 蓝队守右门 x=220
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;

    double cx = 0, cy = 0;

    wm.ball.x = 100; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 远球应返回 true\n"); return 1; }
    if (fabs(cx - 145.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: 远球护门点 (%.1f,%.1f) 应 (145,90)\n", cx, cy); return 1;
    }

    wm.ball.x = 160; wm.ball.y = 100;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 中近球应返回 true\n"); return 1; }
    if (fabs(cx - 170.0) > 0.5 || fabs(cy - 100.0) > 0.5) {
        printf("FAIL: 中近护门点 (%.1f,%.1f) 应 (170,100)\n", cx, cy); return 1;
    }

    wm.ball.x = 205; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 贴门球应返回 true\n"); return 1; }
    if (fabs(cx - 191.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: 贴门护门点 (%.1f,%.1f) 应 (191,90)（不进裁判门区）\n", cx, cy); return 1;
    }

    wm.ball.x = 150; wm.ball.y = 130;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 斜向球应返回 true\n"); return 1; }
    if (fabs(cx - 170.0) > 0.5 || fabs(cy - 107.5) > 0.5) {
        printf("FAIL: 斜向护门点 (%.1f,%.1f) 应 (170,107.5)\n", cx, cy); return 1;
    }

    wm.ball.x = 218; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 极贴门球应返回 true\n"); return 1; }
    if (fabs(cx - 191.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: 极贴门护门点 (%.1f,%.1f) 应 (191,90)（不进裁判门区）\n", cx, cy); return 1;
    }

    printf("goal cover: OK (远球连线/中近拦截/贴门堵射/斜向clamp)\n");
    return 0;
}

// 黄位镜像守卫：期望值 = 蓝位用例的 x 镜像（x' = 220−x）
static int test_goal_cover_yellow() {
    TeamContext ctx{false};   // 黄队守左门 x=0
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;

    double cx = 0, cy = 0;

    wm.ball.x = 120; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 远球应返回 true\n"); return 1; }
    if (fabs(cx - 75.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: [黄] 远球护门点 (%.1f,%.1f) 应 (75,90)（= 镜像蓝队 145）\n", cx, cy); return 1;
    }

    wm.ball.x = 60; wm.ball.y = 100;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 中近球应返回 true\n"); return 1; }
    if (fabs(cx - 50.0) > 0.5 || fabs(cy - 100.0) > 0.5) {
        printf("FAIL: [黄] 中近护门点 (%.1f,%.1f) 应 (50,100)（= 镜像蓝队 170）\n", cx, cy); return 1;
    }

    wm.ball.x = 15; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 贴门球应返回 true\n"); return 1; }
    if (fabs(cx - 29.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: [黄] 贴门护门点 (%.1f,%.1f) 应 (29,90)（= 镜像蓝队 191）\n", cx, cy); return 1;
    }

    wm.ball.x = 70; wm.ball.y = 130;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 斜向球应返回 true\n"); return 1; }
    if (fabs(cx - 50.0) > 0.5 || fabs(cy - 107.5) > 0.5) {
        printf("FAIL: [黄] 斜向护门点 (%.1f,%.1f) 应 (50,107.5)（= 镜像蓝队 170）\n", cx, cy); return 1;
    }

    wm.ball.x = 2; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 极贴门球应返回 true\n"); return 1; }
    if (fabs(cx - 29.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: [黄] 极贴门护门点 (%.1f,%.1f) 应 (29,90)（= 镜像蓝队 191）\n", cx, cy); return 1;
    }

    wm.ball.x = 30; wm.ball.y = 130;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 深偏球应返回 true\n"); return 1; }
    if (fabs(cx - 25.2) > 0.5 || fabs(cy - 123.6) > 0.5) {
        printf("FAIL: [黄] 深偏护门点 (%.1f,%.1f) 应 (25.2,123.6)（不该被钉死在门线）\n", cx, cy); return 1;
    }

    printf("goal cover (yellow): OK (镜像蓝位全部用例)\n");
    return 0;
}

// 攻防状态机单测：滞回防抖 + 事件标志 + 威胁分级
static int test_team_state() {
    TeamContext ctx{true};               // 蓝队，门在 x=220
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.ball.x = 150; wm.ball.y = 90;     // 蓝队半场
    Strategy strat;

    wm.team_state = TS_DEFENSE; wm.possession_frames = 0; wm.no_possession_frames = 0;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 10; wm.home[i].y = 90; wm.opp[i].x = 150; wm.opp[i].y = 90; }
    strat.run(wm);
    if (wm.team_state != TS_DEFENSE) { printf("FAIL: 初始应防守态\n"); return 1; }

    for (int i = 0; i < 5; ++i) { wm.home[i].x = 150; wm.home[i].y = 90; wm.opp[i].x = 10; wm.opp[i].y = 90; }
    strat.run(wm);
    strat.run(wm);
    if (wm.team_state != TS_DEFENSE) { printf("FAIL: 持球 2 帧不应切进攻（滞回）\n"); return 1; }

    strat.run(wm);
    if (wm.team_state != TS_ATTACK) { printf("FAIL: 连续持球 3 帧应切进攻态\n"); return 1; }
    if (wm.threat_level != 0.1) { printf("FAIL: 进攻态威胁应为 0.1 got %.2f\n", wm.threat_level); return 1; }

    for (int i = 0; i < 5; ++i) { wm.home[i].x = 10; wm.home[i].y = 90; wm.opp[i].x = 150; wm.opp[i].y = 90; }
    for (int f = 0; f < 3; ++f) strat.run(wm);
    if (wm.team_state != TS_DEFENSE) { printf("FAIL: 连续失球 3 帧应回防守态\n"); return 1; }
    if (wm.threat_level < 0.5) { printf("FAIL: 防守态(蓝半场)威胁应 0.6 got %.2f\n", wm.threat_level); return 1; }

    printf("team state: OK (滞回防抖/事件标志/威胁分级)\n");
    return 0;
}

// 固定角色单测：角色映射固定，不随位置/距离变化
static int test_fixed_roles() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.ball.x = 110; wm.ball.y = 90;
    Strategy strat;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 20 + i * 30; wm.home[i].y = 90; }
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 10 + i * 20; wm.opp[i].y = 90; }
    strat.run(wm);
    if (wm.role[0] != ROLE_GOALIE || wm.role[1] != ROLE_ACTIVE || wm.role[2] != ROLE_ASSIST ||
        wm.role[3] != ROLE_MIDFIELD || wm.role[4] != ROLE_PASSIVE) {
        printf("FAIL: 角色应固定 0=GK/1=ACTIVE/2=ASSIST/3=MID/4=PASSIVE (got %d%d%d%d%d)\n",
               wm.role[0], wm.role[1], wm.role[2], wm.role[3], wm.role[4]);
        return 1;
    }
    printf("fixed roles: OK (0=GK/1=ACTIVE/2=ASSIST/3=MID/4=PASSIVE)\n");
    return 0;
}

// 传球选点单测：威胁惩罚 / 边界夹取 / 短传优先
static int test_pass() {
    TeamContext ctx{true};               // 蓝队：门在 x=220，攻向左(对方门 x=0)
    WorldModel wm;
    wm.ctx = ctx;

    wm.home[0].x = 80; wm.home[0].y = 90;    // 持球者
    wm.home[1].x = 58; wm.home[1].y = 69;    // A → 接应点(52,69)
    wm.home[2].x = 58; wm.home[2].y = 111;   // B → 接应点(52,111)
    wm.home[3].x = 150; wm.home[3].y = 90;   // 其余队友距离>60，不可选
    wm.home[4].x = 80; wm.home[4].y = 170;
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 20; }
    wm.opp[0].x = 64; wm.opp[0].y = 53;      // 距 A 接应点 20cm，且不挡传球线
    {
        PassPlan p = plan_pass(wm, 0);
        if (!p.viable || p.receiver_id != 2) {
            printf("FAIL: 场景①威胁应惩罚A、选B(home[2]) got viable=%d recv=%d\n", p.viable, p.receiver_id);
            return 1;
        }
    }

    wm.home[0].x = 40; wm.home[0].y = 50;    // 持球者
    wm.home[1].x = 2;  wm.home[1].y = 50;    // 接应点原始 (-4,50) → 夹到 (6,50)
    wm.home[2].x = 40; wm.home[2].y = 160;   // 其余队友距离>60，不可选
    wm.home[3].x = 150; wm.home[3].y = 50;
    wm.home[4].x = 150; wm.home[4].y = 120;
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 20 + i * 25; }
    {
        PassPlan p = plan_pass(wm, 0);
        if (!p.viable || p.receiver_id != 1) {
            printf("FAIL: 场景②应选 home[1] got viable=%d recv=%d\n", p.viable, p.receiver_id);
            return 1;
        }
        if (fabs(p.target_x - 6.0) > 0.5 || fabs(p.target_y - 50.0) > 0.5) {
            printf("FAIL: 场景②接应点未正确夹取 (%.1f,%.1f)\n", p.target_x, p.target_y);
            return 1;
        }
        if (p.target_x < 0 || p.target_x > 220 || p.target_y < 0 || p.target_y > 180) {
            printf("FAIL: 场景②接应点越界 (%.1f,%.1f)\n", p.target_x, p.target_y);
            return 1;
        }
    }

    wm.home[0].x = 80; wm.home[0].y = 90;    // 持球者
    wm.home[1].x = 60; wm.home[1].y = 70;    // A → 接应点(54,70) 短
    wm.home[2].x = 60; wm.home[2].y = 50;    // B → 接应点(54,50) 长
    wm.home[3].x = 160; wm.home[3].y = 90;   // 其余队友距离>60，不可选
    wm.home[4].x = 80; wm.home[4].y = 170;
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 20; }
    {
        PassPlan p = plan_pass(wm, 0);
        if (!p.viable || p.receiver_id != 1) {
            printf("FAIL: 场景③应优先短传A(home[1]) got viable=%d recv=%d\n", p.viable, p.receiver_id);
            return 1;
        }
    }

    wm.role[0] = ROLE_GOALIE;
    wm.role[1] = ROLE_ACTIVE;
    wm.role[2] = ROLE_ASSIST;
    wm.role[3] = ROLE_MIDFIELD;
    wm.role[4] = ROLE_PASSIVE;
    wm.home[0].x = 210; wm.home[0].y = 90;    // GK 远，不可选
    wm.home[1].x = 80;  wm.home[1].y = 90;    // 持球者(ACTIVE)
    wm.home[2].x = 150; wm.home[2].y = 90;    // ASSIST 本体远（若用本体则>60 不可选）
    wm.home[3].x = 150; wm.home[3].y = 150;   // MIDFIELD 远
    wm.home[4].x = 150; wm.home[4].y = 30;    // PASSIVE 远
    wm.assist_x = 60;  wm.assist_y = 70;      // ASSIST 站位点（在传球距离内）
    wm.mid_x = 150;    wm.mid_y = 150;        // MIDFIELD 站位点远
    wm.passive_x = 150; wm.passive_y = 30;    // PASSIVE 站位点远
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 20; }
    {
        PassPlan p = plan_pass(wm, 1);
        if (!p.viable || p.receiver_id != 2) {
            printf("FAIL: 场景④应联动站位点选ASSIST(home[2]) got viable=%d recv=%d\n", p.viable, p.receiver_id);
            return 1;
        }
        const double off = simuro5::get_param("pass.OFFSET_BASE", 6.0);
        if (fabs(p.target_x - (60.0 - off)) > 0.5 || fabs(p.target_y - 70.0) > 0.5) {
            printf("FAIL: 场景④接应点未基于站位点 (%.1f,%.1f) 期望 x=%.1f（60 - OFFSET_BASE=%.1f）\n",
                   p.target_x, p.target_y, 60.0 - off, off);
            return 1;
        }
    }

    printf("pass: OK (威胁惩罚/边界夹取/短传优先/联动站位点)\n");
    return 0;
}

// 守门员二过一/远射场景冒烟：新决策分支不崩溃且轮速合理
static int test_goalie_scenarios() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.home[0].x = 210; wm.home[0].y = 90; wm.home[0].rot = 180;

    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 100 + i * 5; wm.opp[i].y = 90; }
    wm.ball.x = 150; wm.ball.y = 90; wm.ball.vx = 8.0; wm.ball.vy = 0.0;
    wm.opp[0].x = 149; wm.opp[0].y = 90;   // 带球者（离球最近）
    wm.opp[1].x = 165; wm.opp[1].y = 80;   // 接应者（更靠门 + 够近）
    run_goalie(wm, 0);
    if (fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr)) > 300.0) {
        printf("FAIL: 二过一场景轮速异常\n"); return 1;
    }

    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 10 + i * 10; wm.opp[i].y = 90; }
    wm.ball.x = 120; wm.ball.y = 90; wm.ball.vx = 10.0; wm.ball.vy = 0.0;
    run_goalie(wm, 0);
    if (fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr)) > 300.0) {
        printf("FAIL: 远射场景轮速异常\n"); return 1;
    }

    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 10 + i * 10; wm.opp[i].y = 90; }
    wm.ball.x = 120; wm.ball.y = 90; wm.ball.vx = 3.0; wm.ball.vy = 0.0;
    run_goalie(wm, 0);
    if (fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr)) > 300.0) {
        printf("FAIL: 慢球场景轮速异常\n"); return 1;
    }

    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 10 + i * 10; wm.opp[i].y = 90; }
    wm.opp[0].x = 170; wm.opp[0].y = 90;   // 埋伏在罚球区内
    wm.ball.x = 120; wm.ball.y = 90; wm.ball.vx = 10.0; wm.ball.vy = 0.0;
    run_goalie(wm, 0);
    if (fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr)) > 300.0) {
        printf("FAIL: 埋伏回缩场景轮速异常\n"); return 1;
    }

    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 10 + i * 10; wm.opp[i].y = 90; }
    wm.home[1].x = 150; wm.home[1].y = 90;   // 队友在己方半场
    wm.ball.x = 212; wm.ball.y = 90; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[0].x = 210; wm.home[0].y = 90;   // 门将（球夹在门将和门之间 → 应弧线绕）
    run_goalie(wm, 0);
    if (fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr)) > 300.0) {
        printf("FAIL: 解围场景轮速异常\n"); return 1;
    }

    printf("goalie scenarios: OK (二过一后退/远射前压/慢球贴门/埋伏回缩/解围均正常)\n");
    return 0;
}

// 门将乌龙三修：点球守线 / 门球慢爬按静止处理 / 贴线横移补位
static void gk_test_field(WorldModel &wm) {
    wm = WorldModel();
    wm.ctx = TeamContext{true};
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 120; wm.home[i].y = 30.0 + i * 30; wm.role[i] = ROLE_PASSIVE;
        wm.opp[i].x = 60;   wm.opp[i].y = 30.0 + i * 30;
    }
    wm.role[0] = ROLE_GOALIE;
}
static double gk_speed(const WorldModel &wm) {
    return std::fabs(wm.home[0].vl) + std::fabs(wm.home[0].vr);
}

static int test_goalie_og_fixes() {
    WorldModel wm;
    auto pen = [&](double knob) {
        set_param("roles.kGkPenSpotGuard", knob);
        gk_test_field(wm);
        wm.ball.x = 180.8; wm.ball.y = 89.7; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
        wm.opp[4].x = 165; wm.opp[4].y = 90;          // 主罚者在球后
        wm.home[0].x = 217.0; wm.home[0].y = 90.0; wm.home[0].rot = 90.0;
        run_goalie(wm, 0);
        return gk_speed(wm);
    };
    double pen_on = pen(1.0), pen_off = pen(0.0);
    set_param("roles.kGkPenSpotGuard", 1.0);
    if (pen_on > 5.0 || pen_off < 5.0) {
        printf("FAIL: 对方点球时门将应守门线不动 on=%.1f（旧行为应离位 off=%.1f）\n", pen_on, pen_off);
        return 1;
    }

    auto kick = [&](double vx, double knob) {
        set_param("roles.kGkCreepStill", knob);
        gk_test_field(wm);
        wm.ball.x = 205.2; wm.ball.y = 77.7; wm.ball.vx = vx; wm.ball.vy = 0.0;
        wm.home[0].x = 212.0; wm.home[0].y = 80.0; wm.home[0].rot = -107.0;
        run_goalie(wm, 0);
        return std::make_pair(wm.home[0].vl, wm.home[0].vr);
    };
    auto still = kick(0.0, 1.0), creep = kick(-0.31, 1.0);
    auto creep_off = kick(-0.31, 0.0), toward = kick(+0.31, 1.0);
    set_param("roles.kGkCreepStill", 1.0);
    if (still != creep) {
        printf("FAIL: 慢爬门球应按静止门球处理 still=(%.0f,%.0f) creep=(%.0f,%.0f)\n",
               still.first, still.second, creep.first, creep.second);
        return 1;
    }
    if (still == creep_off || still == toward) {
        printf("FAIL: 旋钮关/朝门滚的球不应走门球分支\n");
        return 1;
    }

    auto hold = [&](double gy, double knob) {
        set_param("roles.kGkHoldSlide", knob);
        gk_test_field(wm);
        wm.ball.x = 216.0; wm.ball.y = 80.0; wm.ball.vx = 0.2; wm.ball.vy = 0.1;
        wm.home[0].x = 219.0; wm.home[0].y = gy; wm.home[0].rot = 90.0;
        run_goalie(wm, 0);
        return gk_speed(wm);
    };
    double slide_on = hold(92.0, 1.0), slide_off = hold(92.0, 0.0), covered = hold(83.0, 1.0);
    set_param("roles.kGkHoldSlide", 1.0);
    if (slide_on < 5.0 || slide_off > 1e-9) {
        printf("FAIL: 贴线门将没挡住进门点应横移 on=%.1f（旧行为站定 off=%.1f）\n", slide_on, slide_off);
        return 1;
    }
    if (covered > 1e-9) {
        printf("FAIL: 门将已挡在进门点应站定 v=%.1f\n", covered);
        return 1;
    }
    printf("goalie og fixes: OK (点球守线/慢爬门球=静止门球/贴线横移补位)\n");
    return 0;
}

// 传球威胁距离加权：半径内近/远敌人惩罚不同
static int test_pass_threat_weight() {
    TeamContext ctx{true};               // 蓝队：门 x=220，攻向左(对方门 x=0)
    WorldModel wm;
    wm.ctx = ctx;
    wm.home[0].x = 80; wm.home[0].y = 90;    // 持球者
    wm.home[1].x = 58; wm.home[1].y = 70;    // A → 接应点(52,70)
    wm.home[2].x = 58; wm.home[2].y = 110;   // B → 接应点(52,110)
    wm.home[3].x = 150; wm.home[3].y = 90;   // 其余远(>60)，不可选
    wm.home[4].x = 80;  wm.home[4].y = 170;
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 20; }
    wm.opp[0].x = 37.4; wm.opp[0].y = 59.5;  // A 后方 18cm：距(52,70)=18、距传球线=18(>15 不挡)
    wm.opp[1].x = 30.8; wm.opp[1].y = 125.1; // B 后方 26cm：距(52,110)=26、距传球线=26(>15 不挡)
    PassPlan p = plan_pass(wm, 0);
    if (!p.viable || p.receiver_id != 2) {
        printf("FAIL: 距离加权应选B(home[2]) got viable=%d recv=%d\n", p.viable, p.receiver_id);
        return 1;
    }
    printf("pass threat weight: OK (近盯防惩罚>远盯防)\n");
    return 0;
}

// 无球接应拉开：站位点附近有敌人时沿 Y 轴横向躲开
static int test_roles_spread() {
    TeamContext ctx{true};               // 蓝队：门 x=220，攻向左
    WorldModel wm;
    wm.ctx = ctx;
    wm.threat_level = 0.1;               // <=0.3 走进攻分支
    wm.game_state = PM_PlaceKick_Blue;
    wm.live_play = false;

    auto scatter = [&]() {
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 25; }
    };

    scatter();
    wm.home[1].x = 100; wm.home[1].y = 90; wm.home[1].rot = 0;
    wm.assist_x = 140; wm.assist_y = 90;
    wm.mid_y = 150;                          // 队友错开，避免触发队友回避
    run_assist(wm, 1);
    if (fabs(wm.home[1].vl - wm.home[1].vr) > 0.5 || wm.home[1].vl <= 0.0) {
        printf("FAIL: 无敌人应直走 (vl=%.1f vr=%.1f)\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    scatter();
    wm.opp[0].x = 140; wm.opp[0].y = 100;   // 站位点上方 10cm
    wm.home[1].x = 100; wm.home[1].y = 90; wm.home[1].rot = 0;
    wm.assist_x = 140; wm.assist_y = 90;
    wm.mid_y = 150;                          // 队友错开，避免触发队友回避
    run_assist(wm, 1);
    if (!(wm.home[1].vl > wm.home[1].vr + 1.0)) {
        printf("FAIL: 敌人在上应往下躲(左转 vl>vr) got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    scatter();
    wm.opp[0].x = 140; wm.opp[0].y = 80;    // 站位点下方 10cm
    wm.home[1].x = 100; wm.home[1].y = 90; wm.home[1].rot = 0;
    wm.assist_x = 140; wm.assist_y = 90;
    wm.mid_y = 150;                          // 队友错开，避免触发队友回避
    run_assist(wm, 1);
    if (!(wm.home[1].vl < wm.home[1].vr - 1.0)) {
        printf("FAIL: 敌人在下应往上躲(右转 vl<vr) got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    scatter();
    wm.opp[0].x = 140; wm.opp[0].y = 100;   // 上方
    wm.home[1].x = 100; wm.home[1].y = 90; wm.home[1].rot = 0;
    wm.mid_x = 140; wm.mid_y = 90;
    wm.assist_y = 30;                        // 队友错开，避免触发队友回避
    run_midfield(wm, 1);
    if (!(wm.home[1].vl > wm.home[1].vr + 1.0)) {
        printf("FAIL: midfield 敌人在上应往下躲 got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    scatter();
    wm.opp[0].x = 140; wm.opp[0].y = 12;    // 上方，把目标往界外(y<0)推
    wm.home[1].x = 100; wm.home[1].y = 6;  wm.home[1].rot = 0;
    wm.assist_x = 140; wm.assist_y = 2;
    run_assist(wm, 1);
    if (fabs(wm.home[1].vl - wm.home[1].vr) > 2.0 ||
        fabs(wm.home[1].vl) > 300.0 || fabs(wm.home[1].vr) > 300.0) {
        printf("FAIL: 贴边站位应 clamp 回场内(直走) got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    scatter();
    wm.home[1].x = 100; wm.home[1].y = 90; wm.home[1].rot = 0;
    wm.assist_x = 140; wm.assist_y = 90;
    wm.mid_y = 100;                         // |90-100|=10 < 25，触发队友回避
    run_assist(wm, 1);
    if (!(wm.home[1].vl > wm.home[1].vr + 1.0)) {
        printf("FAIL: assist 队友过近应往下推(左转) got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    scatter();
    wm.home[1].x = 100; wm.home[1].y = 90; wm.home[1].rot = 0;
    wm.mid_x = 140; wm.mid_y = 90;
    wm.assist_y = 80;                       // |90-80|=10 < 25，触发队友回避
    run_midfield(wm, 1);
    if (!(wm.home[1].vl < wm.home[1].vr - 1.0)) {
        printf("FAIL: midfield 队友过近应往上推(右转) got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    printf("roles spread: OK (无敌人直走/上方下躲/下方上躲/midfield拉开/贴边clamp/队友回避)\n");
    return 0;
}

// 射门方案单测：dir 单位向量、推球点 = 球后 8cm、开口选 GK 远侧
static int test_shoot_plan() {
    TeamContext ctx{true};               // 蓝队：门 x=220，攻向左(对方门 x=0)
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.ball.x = 25; wm.ball.y = 90;      // 球在对方门前 25cm（dgoal<70 → 无条件可射区）
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 10 + i; wm.opp[i].y = 90; }
    wm.opp[0].x = 3;  wm.opp[0].y = 90;  // 对方守门员站门中央（离门线最近）
    {
        ShootPlan p = plan_shoot(wm, 1);
        if (!p.viable) { printf("FAIL: 球在门前应可射\n"); return 1; }
        double dl = std::hypot(p.dir_x, p.dir_y);
        if (fabs(dl - 1.0) > 1e-3) { printf("FAIL: dir 非单位 (%.3f)\n", dl); return 1; }
        if (p.dir_x >= 0.0) { printf("FAIL: 推球方向应朝对方门 (dir_x=%.2f)\n", p.dir_x); return 1; }
        if (fabs(p.target_x - (wm.ball.x - p.dir_x * 8.0)) > 0.5 ||
            fabs(p.target_y - (wm.ball.y - p.dir_y * 8.0)) > 0.5) {
            printf("FAIL: 推球点错 (%.1f,%.1f)\n", p.target_x, p.target_y); return 1;
        }
        if (fabs(angle_diff(p.aim_rot, angle_to(0, 0, p.dir_x, p.dir_y))) > 1e-6) {
            printf("FAIL: aim_rot 与 dir 不一致 %.2f vs %.2f\n",
                   p.aim_rot, angle_to(0, 0, p.dir_x, p.dir_y)); return 1;
        }
        if (fabs(p.shot_dist - 25.0) > 0.6) { printf("FAIL: shot_dist=%.1f\n", p.shot_dist); return 1; }
        if (p.aim_y < goal_y_low() - 0.1 || p.aim_y > goal_y_high() + 0.1) {
            printf("FAIL: 瞄准点出框 aim_y=%.1f\n", p.aim_y); return 1;
        }
    }
    wm.opp[0].y = 105;
    {
        ShootPlan p2 = plan_shoot(wm, 1);
        if (!p2.viable || p2.aim_y >= 90.0) {
            printf("FAIL: GK 偏上应瞄下半区 got aim=%.1f viable=%d\n", p2.aim_y, p2.viable);
            return 1;
        }
    }
    wm.opp[0].y = 75;                    // GK 偏下 → 瞄上半区
    {
        ShootPlan p3 = plan_shoot(wm, 1);
        if (!p3.viable || p3.aim_y <= 90.0) {
            printf("FAIL: GK 偏下应瞄上半区 got aim=%.1f\n", p3.aim_y);
            return 1;
        }
    }
    wm.ball.x = 100; wm.opp[0].y = 90;   // 100cm + GK 封中：净开口 ≈ 2·(11.3°−4.6°) ≈ 6.7° < 8°
    for (int i = 1; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 20; }
    {
        ShootPlan p4 = plan_shoot(wm, 1);
        if (p4.viable && !p4.bank) {
            printf("FAIL: GK 封死时 100cm 远射应被拒 (open=%.1f)\n", p4.open_angle);
            return 1;
        }
    }
    wm.opp[0].y = 108;                   // GK 偏上 → 下半开口 ~17° → 放行
    {
        ShootPlan p5 = plan_shoot(wm, 1);
        if (!p5.viable) { printf("FAIL: 开口够大应可远射\n"); return 1; }
        if (p5.open_angle < 8.0) { printf("FAIL: open=%.1f 应≥8\n", p5.open_angle); return 1; }
        if (p5.quality < 0.35) { printf("FAIL: quality=%.2f 应≥0.35\n", p5.quality); return 1; }
        {
            double dgx = ctx.opp_goal_x() - wm.ball.x, dgy = 90.0 - wm.ball.y;
            double dn = std::hypot(dgx, dgy);
            if (dn > 1e-6) { dgx /= dn; dgy /= dn; }
            wm.opp[1].x = wm.ball.x + dgx * 12.0;
            wm.opp[1].y = wm.ball.y + dgy * 12.0;
        }
        ShootPlan p6 = plan_shoot(wm, 1);
        if (p6.viable && !p6.bank) {
            printf("FAIL: 路线被挡的远射不该沿原直线硬射 (lane_blocked=%d)\n", (int)p6.lane_blocked);
            return 1;
        }
        if (p6.bank && p6.bank_quality < 0.45) {
            printf("FAIL: 借墙方案质量应≥0.45 got %.3f\n", p6.bank_quality);
            return 1;
        }
        wm.opp[1].x = 200; wm.opp[1].y = 30;     // 撤走 → 恢复可射
        if (!plan_shoot(wm, 1).viable) { printf("FAIL: 撤走阻挡后应恢复可射\n"); return 1; }
    }
    wm.ball.x = 60;
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 2; wm.opp[i].y = 74.0 + i * 8.0; }
    {
        ShootPlan p7 = plan_shoot(wm, 1);
        if (!p7.viable) { printf("FAIL: ≤70cm 应无条件可射（A/B 校准的主力区）\n"); return 1; }
    }
    wm.ball.x = 92; wm.ball.y = 90; wm.ball.vx = 0; wm.ball.vy = 0;
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 3; wm.opp[i].y = 90; }
    wm.opp[0].y = 90;                    // GK 站中央封门
    {
        wm.in_penalty_exec = true;
        ShootPlan pp = plan_shoot(wm, 1);
        if (!pp.viable || !pp.penalty) {
            printf("FAIL: 点球执行期必须可射（旁路闸门）open=%.1f\n", pp.open_angle);
            return 1;
        }
        if (pp.quality < 0.99) { printf("FAIL: 点球 quality 应置 1 got %.2f\n", pp.quality); return 1; }
        wm.in_penalty_exec = false;
        ShootPlan pn = plan_shoot(wm, 1);
        if (pn.viable && !pn.bank) {
            printf("FAIL: 非点球时 GK 封死的 92cm 不该沿直线硬射 (bank=%d open=%.2f)\n",
                   (int)pn.bank, pn.open_angle);
            return 1;
        }
    }
    printf("shoot plan: OK (近距无条件/dir单位/连续瞄准/远射双闸门/路线阻挡/点球旁路)\n");
    return 0;
}

// 借墙射门：直线被门将封死时改走借墙，反射点用实测各向异性系数（法向 0.45）
static int test_bank_shot() {
    {
        TeamContext ctx{true};
        WorldModel wm;
        wm.ctx = ctx;
        wm.ball.valid = true;
        wm.ball.x = 100; wm.ball.y = 55; wm.ball.vx = 0; wm.ball.vy = 0;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 150; wm.opp[i].y = 20 + i * 35; }
        wm.opp[0].x = 20; wm.opp[0].y = 83;          // 门将压在 (100,55)→(0,90) 连线上
        ShootPlan p = plan_shoot(wm, 1);
        if (!p.viable || !p.bank) {
            printf("FAIL: 直线被门将封死时应改走借墙 (viable=%d bank=%d open=%.1f)\n",
                   (int)p.viable, (int)p.bank, p.open_angle);
            return 1;
        }
        if (p.bank_quality < 0.45) {
            printf("FAIL: 借墙质量应≥0.45 got %.3f\n", p.bank_quality);
            return 1;
        }
        if (p.bank_wall != 0.0) {          // 球在下半场 → 该借底墙
            printf("FAIL: 球在 y=55 应借底墙 got wall=%.0f\n", p.bank_wall);
            return 1;
        }
        const double kTestFric = get_param("shoot.kBankWallFric", 0.81);
        const double kTestRest = get_param("shoot.kBankWallRest", 0.45);
        double ox = p.dir_x * kTestFric, oy = -p.dir_y * kTestRest;
        double tx = ctx.opp_goal_x() - p.bounce_x, ty = p.aim_y - p.bank_wall;
        double cross = ox * ty - oy * tx;
        double sc = std::hypot(ox, oy) * std::hypot(tx, ty);
        if (sc < 1e-9 || fabs(cross) / sc > 1e-3) {
            printf("FAIL: 反射解不自洽 sin=%.2e (bounce=%.2f aim=%.2f)\n",
                   sc > 1e-9 ? fabs(cross) / sc : 9.9, p.bounce_x, p.aim_y);
            return 1;
        }
        double c1m = p.aim_y - p.bank_wall, c2m = p.bank_wall - wm.ball.y;
        double rx_mirror = (c1m * wm.ball.x - c2m * ctx.opp_goal_x()) / (c1m - c2m);
        if (fabs(rx_mirror - p.bounce_x) < 3.0) {
            printf("FAIL: 镜面解(%.2f)与实测解(%.2f)几乎相同 → 系数没生效\n",
                   rx_mirror, p.bounce_x);
            return 1;
        }
        if (p.bounce_x >= wm.ball.x || p.bounce_x <= ctx.opp_goal_x()) {
            printf("FAIL: 反弹点位置不对 %.2f\n", p.bounce_x);
            return 1;
        }
    }
    {
        TeamContext ctx{true};
        WorldModel wm;
        wm.ctx = ctx;
        wm.ball.valid = true;
        wm.ball.x = 25; wm.ball.y = 90; wm.ball.vx = 0; wm.ball.vy = 0;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 3; wm.opp[i].y = 74.0 + i * 8.0; }
        ShootPlan p = plan_shoot(wm, 1);
        if (!p.viable || p.bank) {
            printf("FAIL: 门前 25cm 应走直线射门，不该借墙 (bank=%d)\n", (int)p.bank);
            return 1;
        }
    }
    {
        TeamContext ctx{true};
        WorldModel wm;
        wm.ctx = ctx;
        wm.ball.valid = true;
        wm.ball.x = 3; wm.ball.y = 90; wm.ball.vx = 0; wm.ball.vy = 0;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 20; wm.opp[i].y = 20 + i * 30; }
        ShootPlan p = plan_shoot(wm, 1);
        if (p.viable || p.bank) {
            printf("FAIL: 贴门线应无方案 got viable=%d bank=%d\n", (int)p.viable, (int)p.bank);
            return 1;
        }
    }
    {
        TeamContext ctx{false};
        WorldModel wm;
        wm.ctx = ctx;
        wm.ball.valid = true;
        wm.ball.x = 120; wm.ball.y = 120; wm.ball.vx = 0; wm.ball.vy = 0;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 60; wm.opp[i].y = 20 + i * 35; }
        wm.opp[0].x = 200; wm.opp[0].y = 96;         // 门将压在 (120,120)→(220,90) 连线上
        ShootPlan p = plan_shoot(wm, 1);
        if (!p.viable || !p.bank || fabs(p.bank_wall - 180.0) > 1e-9) {
            printf("FAIL: 黄队上半场应借顶墙 got viable=%d bank=%d wall=%.0f q=%.3f\n",
                   (int)p.viable, (int)p.bank, p.bank_wall, p.bank_quality);
            return 1;
        }
        if (p.bank_quality < 0.45) {
            printf("FAIL: 黄队借顶墙质量应≥0.45 got %.3f\n", p.bank_quality);
            return 1;
        }
    }
    printf("bank shot: OK (反射闭式解/与镜面解有差/不抢直线/贴门线不硬凑/黄队镜像)\n");
    return 0;
}

// 借墙测试档 shoot.kBankForceTest=1：强制出借墙样本，但门前 ≤70cm 仍走直线
static int test_bank_force_test() {
    struct ForceRestore {
        double v;
        ~ForceRestore() { set_param("shoot.kBankForceTest", v); }
    } force_restore{get_param("shoot.kBankForceTest", 0.0)};

    {
        TeamContext ctx{true};                    // 蓝队：守 x=220、攻 x=0
        WorldModel wm;
        wm.ctx = ctx;
        wm.ball.valid = true;
        wm.ball.x = 75; wm.ball.y = 90; wm.ball.vx = 0; wm.ball.vy = 0;   // dgoal=75 → 远射档
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 150; wm.opp[i].y = 20 + i * 30; }
        wm.opp[0].x = 5; wm.opp[0].y = 130;       // 门将离射门线很远 → 直线净开口很大
        set_param("shoot.kBankForceTest", 0.0);
        ShootPlan gated = plan_shoot(wm, 1);
        set_param("shoot.kBankForceTest", 1.0);
        ShootPlan forced = plan_shoot(wm, 1);
        if (!gated.viable || gated.bank) {
            printf("FAIL: 直线是好机会时门槛版不该借墙 (viable=%d bank=%d q=%.3f)\n",
                   (int)gated.viable, (int)gated.bank, gated.quality);
            return 1;
        }
        if (!forced.viable || !forced.bank) {
            printf("FAIL: 测试档应强制借墙 (viable=%d bank=%d)\n",
                   (int)forced.viable, (int)forced.bank);
            return 1;
        }
        if (plan_bank_carry(wm).viable) {
            printf("FAIL: 测试档下 plan_bank_carry 应恒不可行（只留主攻一人借墙）\n");
            return 1;
        }
    }
    {
        TeamContext ctx{true};
        WorldModel wm;
        wm.ctx = ctx;
        wm.ball.valid = true;
        wm.ball.x = 25; wm.ball.y = 90; wm.ball.vx = 0; wm.ball.vy = 0;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 3; wm.opp[i].y = 74.0 + i * 8.0; }
        ShootPlan p = plan_shoot(wm, 1);
        if (!p.viable || p.bank) {
            printf("FAIL: 测试档下门前 25cm 仍应走直线射门 (bank=%d)\n", (int)p.bank);
            return 1;
        }
    }
    printf("bank force test: OK (门槛版走直线/测试档强制借墙/门前70内不被抢/蜂群借墙关)\n");
    return 0;
}

// 走廊拐弯：边路无射门方案时把球往门前中路推，且准备点始终留在场内
static int test_corridor_plan() {
    const double prep_d = get_param("shoot.kCorridorPrepDist", 23.0);
    const double margin = get_param("shoot.kCorridorMargin", 11.0);

    {
        TeamContext ctx{true};               // 蓝队：攻 x=0，对方门 y∈[70,110]
        WorldModel wm;
        wm.ctx = ctx;
        wm.ball.valid = true;
        wm.ball.x = 75; wm.ball.y = 25;      // 贴下边路、离门 99cm
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 150; wm.opp[i].y = 20 + i * 30; }
        wm.opp[0].x = 5; wm.opp[0].y = 90;   // 门将站门中央
        ShootPlan p = plan_corridor(wm, 1);
        if (!p.viable) { printf("FAIL: 边路应出走廊拐弯方案\n"); return 1; }
        if (!p.corridor) { printf("FAIL: corridor 标志未置\n"); return 1; }
        double dl = std::hypot(p.dir_x, p.dir_y);
        if (fabs(dl - 1.0) > 1e-3) { printf("FAIL: corridor dir 非单位 (%.3f)\n", dl); return 1; }
        if (p.dir_x >= 0.0) { printf("FAIL: 应朝对方门推 dir_x=%.2f\n", p.dir_x); return 1; }
        if (p.dir_y <= 0.0) { printf("FAIL: 球在下边路应往上拐 dir_y=%.2f\n", p.dir_y); return 1; }
        double py = wm.ball.y - p.dir_y * prep_d;
        if (py < margin - 0.5 || py > TeamContext::FIELD_WIDTH - margin + 0.5) {
            printf("FAIL: 准备点出界 py=%.1f (margin=%.1f)\n", py, margin); return 1;
        }
        if (fabs(p.aim_rot - angle_to(0, 0, p.dir_x, p.dir_y)) > 1e-6) {
            printf("FAIL: corridor aim_rot 与 dir 不一致\n"); return 1;
        }
        if (p.aim_y < goal_y_low() || p.aim_y > goal_y_high()) {
            printf("FAIL: 走廊门点出框 aim_y=%.1f\n", p.aim_y); return 1;
        }
        // 关键前提：这种边路位置本来不该有直线射门方案
        if (plan_shoot(wm, 1).viable && plan_shoot(wm, 1).open_angle >= get_param("roles.kCorridorShotOpen", 12.0)) {
            printf("FAIL: 边路开口过大，前提不成立 open=%.1f\n", plan_shoot(wm, 1).open_angle); return 1;
        }
    }
    {
        TeamContext ctx{true};
        WorldModel wm;
        wm.ctx = ctx;
        wm.ball.valid = true;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 150; wm.opp[i].y = 20 + i * 30; }
        wm.opp[0].x = 5; wm.opp[0].y = 90;
        wm.ball.x = 75; wm.ball.y = 90;      // 中路：不该拐弯，直接射
        if (plan_corridor(wm, 1).viable) { printf("FAIL: 中路线不该出走廊方案\n"); return 1; }
        wm.ball.x = 170; wm.ball.y = 25;     // 太远（>kCorridorMaxDist）：交给普通推进
        if (plan_corridor(wm, 1).viable) { printf("FAIL: 超距不该出走廊方案\n"); return 1; }
        wm.ball.x = 75; wm.ball.y = 25;
        wm.in_penalty_exec = true;           // 点球执行期不拐弯
        if (plan_corridor(wm, 1).viable) { printf("FAIL: 点球执行期不该出走廊方案\n"); return 1; }
        wm.in_penalty_exec = false;
    }
    {
        TeamContext ctx{false};              // 黄队镜像：守 x=0、攻 x=220
        WorldModel wm;
        wm.ctx = ctx;
        wm.ball.valid = true;
        wm.ball.x = 145; wm.ball.y = 155;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 70; wm.opp[i].y = 20 + i * 30; }
        wm.opp[0].x = 215; wm.opp[0].y = 90;
        ShootPlan p = plan_corridor(wm, 1);
        if (!p.viable) { printf("FAIL: 黄队镜像应出走廊方案\n"); return 1; }
        if (p.dir_x <= 0.0) { printf("FAIL: 黄队应朝 x=220 推 dir_x=%.2f\n", p.dir_x); return 1; }
        if (p.dir_y >= 0.0) { printf("FAIL: 球在上边路应往下拐 dir_y=%.2f\n", p.dir_y); return 1; }
    }
    printf("corridor plan: OK (边路拐弯/方向朝门/准备点在场内/中路与超距不出/点球旁路/黄队镜像)\n");
    return 0;
}

// 走廊拐弯的准备点约束：横向分量被夹住后，多脚推进仍应逐步把球带向中线
static int test_corridor_progress() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 150; wm.opp[i].y = 20 + i * 30; }
    wm.opp[0].x = 5; wm.opp[0].y = 90;

    // 模拟沿推球方向连推 6 脚：球位应同时朝对方门（x 减小）和中线（|y-90| 减小）走
    double bx = 70, by = 20;
    double start_x = bx, start_lat = fabs(by - 90.0);
    int moved = 0;
    for (int step = 0; step < 6; ++step) {
        wm.ball.x = bx; wm.ball.y = by;
        ShootPlan p = plan_corridor(wm, 1);
        if (!p.viable) break;
        bx += p.dir_x * 18.0; by += p.dir_y * 18.0;   // 一脚推进 18cm
        ++moved;
    }
    if (moved < 3) { printf("FAIL: 走廊方案中途断掉 (only %d steps)\n", moved); return 1; }
    if (bx >= start_x) { printf("FAIL: 没朝对方门推进 %.1f→%.1f\n", start_x, bx); return 1; }
    if (fabs(by - 90.0) >= start_lat) {
        printf("FAIL: 没向中线靠拢 |y-90| %.1f→%.1f\n", start_lat, fabs(by - 90.0)); return 1;
    }
    printf("corridor progress: OK (连推 6 脚：x %.0f→%.0f，|y-90| %.0f→%.0f)\n",
           start_x, bx, start_lat, fabs(by - 90.0));
    return 0;
}

// 前场边路：助攻/中场锚点应改成「跟球尾随 + 向中路靠」，中路和远球仍是原来的球前点
static int test_trail_stand() {
    SituationModule sitm;
    const double dist = get_param("situation.kTrailDist", 26.0);
    const double pull = get_param("situation.kTrailPull", 0.45);

    {
        WorldModel wm;
        wm.ctx = TeamContext{true};            // 蓝队：攻 x=0
        wm.ball.valid = true;
        wm.ball.x = 75; wm.ball.y = 25;        // 对方半场且贴下边路 → 该尾随
        wm.team_state = TS_ATTACK;
        wm.threat_level = 0.1;
        wm.assist_x = 200; wm.assist_y = 170;  // 先放到远处，确保滞回会更新
        wm.mid_x = 200; wm.mid_y = 170;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 210; wm.opp[i].y = 175; }
        sitm.update_stand_points(wm);
        double want_ax = wm.ball.x - wm.ctx.attack_dir() * dist;    // 蓝队 ad=-1 → 球后
        double want_ay = wm.ball.y + (90.0 - wm.ball.y) * pull;
        if (fabs(wm.assist_x - want_ax) > 3.0) {
            printf("FAIL: 尾随位 x 应≈%.1f got %.1f\n", want_ax, wm.assist_x); return 1;
        }
        if (fabs(wm.assist_y - want_ay) > 3.0) {
            printf("FAIL: 尾随位 y 应≈%.1f got %.1f\n", want_ay, wm.assist_y); return 1;
        }
        if (wm.assist_y >= wm.ball.y + 40.0 - 1.0) {
            printf("FAIL: 尾随位应比旧的球前点更靠中路 got y=%.1f\n", wm.assist_y); return 1;
        }
    }
    {
        WorldModel wm;
        wm.ctx = TeamContext{true};
        wm.ball.valid = true;
        wm.ball.x = 110; wm.ball.y = 90;       // 正在中路且离对方门 110cm（尾随区之外）→ 保持球前 40cm
        wm.team_state = TS_ATTACK;
        wm.threat_level = 0.1;
        wm.assist_x = 200; wm.assist_y = 170;
        wm.mid_x = 200; wm.mid_y = 170;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 210; wm.opp[i].y = 175; }
        sitm.update_stand_points(wm);
        if (fabs(wm.assist_x - 70.0) > 3.0 || fabs(wm.assist_y - 130.0) > 3.0) {
            printf("FAIL: 中路线应保持球前点 got (%.1f,%.1f)\n", wm.assist_x, wm.assist_y); return 1;
        }
    }
    {
        WorldModel wm;
        wm.ctx = TeamContext{true};
        wm.ball.valid = true;
        wm.ball.x = 175; wm.ball.y = 25;       // 离对方门 >110cm → 不尾随
        wm.team_state = TS_ATTACK;
        wm.threat_level = 0.1;
        wm.assist_x = 200; wm.assist_y = 170;
        wm.mid_x = 200; wm.mid_y = 170;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 210; wm.opp[i].y = 175; }
        sitm.update_stand_points(wm);
        if (fabs(wm.assist_x - 135.0) > 3.0) {
            printf("FAIL: 后场不应尾随 got x=%.1f\n", wm.assist_x); return 1;
        }
    }
    printf("trail stand: OK (前场边路尾随+靠中路/中路不变/后场不变)\n");
    return 0;
}

// ACTIVE 门区停留超限后应撤出门区，而不是继续射门/带球
static int test_active_ga_retreat() {
    TeamContext ctx{true};               // 蓝队：对方门区 x∈[0,50], y∈[75,105]
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.ball.x = 100; wm.ball.y = 90;     // 球远离对方门（>70cm 不可射，排除射门分支）
    wm.ball.vx = 2.0; wm.ball.vy = 0.0;  // 非死球（排除门球早退分支）
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 160; wm.home[i].y = 20 + i * 25; }
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 20; }
    wm.home[1].x = 25; wm.home[1].y = 90; wm.home[1].rot = 0;   // ACTIVE 在对方门区内，面朝 +x
    wm.active_ga_frames = 11;            // 超纯停留限(kActiveGaLimit=10)
    run_active(wm, 1);
    if (!(wm.home[1].vl > 0.0 && wm.home[1].vr > 0.0) ||
        fmax(fabs(wm.home[1].vl), fabs(wm.home[1].vr)) > 300.0) {
        printf("FAIL: 超限应撤出门区(+x) got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }
    printf("active GA limit: OK (超限撤出门区)\n");
    return 0;
}

// 禁止推球区几何单测：四角对称 + 边界半径 35cm
static int test_no_push_zone() {
    const double corners[4][2] = {{0, 0}, {220, 0}, {0, 180}, {220, 180}};
    for (auto &c : corners)
        if (fabs(dist_to_corner(c[0], c[1])) > 1e-9) {
            printf("FAIL: 角点 (%.0f,%.0f) 距离应为 0 got %.3f\n",
                   c[0], c[1], dist_to_corner(c[0], c[1]));
            return 1;
        }
    if (fabs(dist_to_corner(110.0, 90.0) - std::hypot(110.0, 90.0)) > 1e-9) {
        printf("FAIL: 场心距离错 %.3f\n", dist_to_corner(110.0, 90.0));
        return 1;
    }
    if (!in_no_push_zone(20.0, 20.0))  { printf("FAIL: (20,20) 应在禁区内(28.3<35)\n"); return 1; }
    if (in_no_push_zone(28.0, 28.0))   { printf("FAIL: (28,28) 不应在禁区内(39.6>35)\n"); return 1; }
    if (!in_no_push_zone(200.0, 20.0) || !in_no_push_zone(20.0, 160.0) ||
        !in_no_push_zone(200.0, 160.0)) {
        printf("FAIL: 四角不对称（20,20 在区内而其它角不在）\n");
        return 1;
    }
    if (in_no_push_zone(190.0, 150.0)) { printf("FAIL: (190,150) 距角 42.4 不应在区内\n"); return 1; }
    if (in_no_push_zone(110.0, 90.0))  { printf("FAIL: 场心不应在禁区内\n"); return 1; }
    printf("no push zone: OK (四角对称/边界 35cm/场心不在区内)\n");
    return 0;
}

// 角区救球：禁止推球区（半径 35cm）内一律不碰球，区外卡球 >30 帧才推
static int test_active_corner_rescue() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.ball.x = 28; wm.ball.y = 28;      // dist_to_corner = 39.6 > 35 → 允许推
    wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 150; wm.home[i].y = 30 + i * 20; }
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 20; }
    wm.home[1].x = 20; wm.home[1].y = 20; wm.home[1].rot = 45;   // 站球后、对准场心方向
    wm.corner_ball_frames = 31;          // 卡住 >30 帧
    wm.active_ga_frames = 0;
    wm.game_state = PM_PlayOn;
    run_active(wm, 1);
    if (!(wm.home[1].vl > 0.0 && wm.home[1].vr > 0.0) ||
        fmax(fabs(wm.home[1].vl), fabs(wm.home[1].vr)) > 300.0) {
        printf("FAIL: 35cm 外的卡球应朝场内推 got vl=%.1f vr=%.1f\n",
               wm.home[1].vl, wm.home[1].vr);
        return 1;
    }
    if (kNoPushGuardEnabled) {
        wm.ball.x = 26; wm.ball.y = 20;
        wm.home[1].x = 18; wm.home[1].y = 12; wm.home[1].rot = 45;
        wm.home[1].vl = wm.home[1].vr = 0;
        run_active(wm, 1);
        if (fabs(wm.home[1].vl) > 1e-9 || fabs(wm.home[1].vr) > 1e-9) {
            printf("FAIL: 角区内应停住不碰球 got vl=%.1f vr=%.1f\n",
                   wm.home[1].vl, wm.home[1].vr);
            return 1;
        }
        wm.ball.x = 110; wm.ball.y = 90;     // 球在中圈，远离角区
        wm.home[1].x = 100; wm.home[1].y = 90; wm.home[1].rot = 0;
        wm.home[1].vl = wm.home[1].vr = 0;
        wm.game_state = PM_FreeBall_RightBot;    // 平台判了争球
        run_active(wm, 1);
        if (fabs(wm.home[1].vl) > 1e-9 || fabs(wm.home[1].vr) > 1e-9) {
            printf("FAIL: 死球/重启期应停住不推 got vl=%.1f vr=%.1f\n",
                   wm.home[1].vl, wm.home[1].vr);
            return 1;
        }
    }
    wm.game_state = PM_PlayOn;
    wm.ball.x = 60; wm.ball.y = 90;
    wm.home[1].x = 60; wm.home[1].y = 60;
    wm.corner_ball_frames = 0;
    run_active(wm, 1);
    printf("active corner rescue: OK (35cm 外才救/角区内停住/死球期停住/非角区不触发)\n");
    return 0;
}

static int test_segment_circle() {
    CircleObstacle o[2], hit;
    o[0] = {10, 10, 3};
    if (!segment_clear_of_circles(0, 0, 10, 0, o, 1)) { printf("FAIL: 相离圆不应挡\n"); return 1; }
    o[0] = {5, 0, 3};
    if (segment_clear_of_circles(0, 0, 10, 0, o, 1)) { printf("FAIL: 相交圆应挡\n"); return 1; }
    o[0] = {1, 0, 3};
    if (segment_clear_of_circles(0, 0, 10, 0, o, 1)) { printf("FAIL: 端点在圆内应挡\n"); return 1; }
    o[0] = {5, 3, 3};
    if (segment_clear_of_circles(0, 0, 10, 0, o, 1)) { printf("FAIL: 相切应挡\n"); return 1; }
    o[0] = {5, 3, 2};    // 距线段 3 > 2：不挡
    o[1] = {6, 0, 4};    // 挡（穿透 4）
    if (segment_clear_of_circles(0, 0, 10, 0, o, 2, &hit)) { printf("FAIL: 应检测到挡路\n"); return 1; }
    if (fabs(hit.x - 6) > 1e-9 || fabs(hit.r - 4) > 1e-9) { printf("FAIL: hit 应指向挡路圆\n"); return 1; }
    if (!segment_clear_of_circles(0, 0, 10, 0, nullptr, 0)) { printf("FAIL: 无障应恒通\n"); return 1; }
    printf("segment circle: OK (相离/相交/端点在内/相切/多圆取最深/空数组)\n");
    return 0;
}

// 路径规划单测：无障/远处障碍都走直线（不绕路）
static int test_route_straight() {
    RoutePlan p = plan_route(50, 90, 150, 90, nullptr, 0);
    if (!p.found || p.n_wp != 2) { printf("FAIL: 无障应直线两点\n"); return 1; }
    if (fabs(p.length - 100.0) > 0.01) { printf("FAIL: 直线长度 %.1f\n", p.length); return 1; }
    CircleObstacle o = {50, 150, 10};
    p = plan_route(50, 90, 150, 90, &o, 1);
    if (!p.found || p.n_wp != 2) { printf("FAIL: 不挡路的圆不应触发绕行\n"); return 1; }
    printf("route straight: OK (无障直线/远处障碍不绕路)\n");
    return 0;
}

// 路径规划单测：单圆盘绕行（不穿盘、长度 ∈(100,110)）
static int test_route_avoid() {
    CircleObstacle o = {100, 90, 10};        // 挡在 S(50,90)→T(150,90) 正中间
    RoutePlan p = plan_route(50, 90, 150, 90, &o, 1);
    if (!p.found) { printf("FAIL: 单圆盘应可绕行\n"); return 1; }
    double min_d = 1e9;
    for (int i = 0; i < p.n_wp - 1; ++i) {
        double ax = p.wp_x[i], ay = p.wp_y[i], bx = p.wp_x[i + 1], by = p.wp_y[i + 1];
        for (int s = 0; s <= 10; ++s) {
            double t = s / 10.0;
            double x = ax + (bx - ax) * t, y = ay + (by - ay) * t;
            double d = std::hypot(x - 100.0, y - 90.0);
            if (d < min_d) min_d = d;
        }
    }
    if (min_d < 8.0) { printf("FAIL: 路径穿障碍 min_d=%.2f\n", min_d); return 1; }
    if (min_d < 9.5) {
        printf("FAIL: 路径侵入膨胀圈超过 margin(0.5) min_d=%.2f\n", min_d);
        return 1;
    }
    if (p.length <= 100.0 || p.length >= 110.0) {
        printf("FAIL: 绕行长度 %.1f（应 ∈(100,110)，最优≈102）\n", p.length);
        return 1;
    }
    if (fabs(p.wp_x[p.n_wp - 1] - 150.0) > 0.5 || fabs(p.wp_y[p.n_wp - 1] - 90.0) > 0.5) {
        printf("FAIL: 终点错\n"); return 1;
    }
    printf("route avoid: OK (绕行不穿盘/长度∈(100,110)/侵入<=margin/终点精确)\n");
    return 0;
}

// 全局安全契约：plan_route 绝不返回违反任何障碍的路径，否则 found=false
static bool route_hits_any(const RoutePlan &p, const CircleObstacle *obs, int n) {
    if (!p.found) return false;
    for (int i = 0; i < p.n_wp - 1; ++i)
        for (int k = 0; k < n; ++k)
            if (segment_hits_circle(p.wp_x[i], p.wp_y[i], p.wp_x[i + 1], p.wp_y[i + 1],
                                    obs[k].x, obs[k].y, obs[k].r - 0.5))
                return true;
    return false;
}

static int test_route_global_safety() {
    {
        CircleObstacle wall[4] = {{60, 90, 10}, {80, 90, 10}, {100, 90, 10}, {100, 128, 12}};
        RoutePlan p = plan_route(30, 90, 130, 90, wall, 4);
        if (route_hits_any(p, wall, 4)) { printf("FAIL: 墙体+远圆场景路径穿盘\n"); return 1; }
    }
    {
        CircleObstacle cage[4] = {{100, 90, 10}, {100, 110, 10}, {100, 70, 10}, {120, 90, 10}};
        RoutePlan p = plan_route(100, 90, 60, 90, cage, 4);
        if (route_hits_any(p, cage, 4)) { printf("FAIL: 围死场景给了穿盘路径\n"); return 1; }
    }
    uint64_t s = 12345;
    auto rnd = [&s]() { s = s * 6364136223846793005ull + 1442695040888963407ull;
                        return (double)((s >> 33) & 0x7FFFFFFF) / 2147483647.0; };
    int found_n = 0, hit_n = 0;
    for (int trial = 0; trial < 200; ++trial) {
        CircleObstacle obs[5];
        for (int k = 0; k < 5; ++k) {
            obs[k].x = 20.0 + rnd() * 180.0;
            obs[k].y = 20.0 + rnd() * 140.0;
            obs[k].r = 6.0 + rnd() * 12.0;
        }
        double sx = 10.0 + rnd() * 40.0, sy = 20.0 + rnd() * 140.0;
        double tx = 170.0 + rnd() * 40.0, ty = 20.0 + rnd() * 140.0;
        RoutePlan p = plan_route(sx, sy, tx, ty, obs, 5);
        if (p.found) ++found_n;
        if (route_hits_any(p, obs, 5)) {
            ++hit_n;
            if (hit_n == 1)
                printf("FAIL: 随机场景 %d 路径穿盘 S(%.0f,%.0f) T(%.0f,%.0f)\n",
                       trial, sx, sy, tx, ty);
        }
    }
    printf("  random safety: 200 组，可规划 %d 组，穿盘 %d 组\n", found_n, hit_n);
    if (hit_n != 0) return 1;
    {
        CircleObstacle obs[5] = {{100, 90, 10}, {90, 120, 10}, {110, 60, 10}, {130, 90, 10}, {70, 90, 10}};
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 2000; ++i) plan_route(30, 90, 190, 90, obs, 5);
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now() - t0).count();
        printf("  plan_route 最坏配置(5 圆全入图) 平均 %.1f us/次\n", us / 2000.0);
        if (us / 2000.0 > 400.0) { printf("FAIL: plan_route 平均耗时超标\n"); return 1; }
    }
    printf("route global safety: OK (固定场景/围死回退/200 组随机无穿盘)\n");
    return 0;
}

// 路径规划单测：错位栅栏可通 / 围死回退 / 起点圆内
static int test_route_cluster() {
    CircleObstacle bar[3] = {{90, 78, 10}, {90, 102, 10}, {130, 90, 10}};
    RoutePlan p = plan_route(30, 90, 190, 90, bar, 3);
    if (!p.found) { printf("FAIL: 错位栅栏应有通路\n"); return 1; }
    for (int i = 0; i < p.n_wp - 1; ++i) {
        double ax = p.wp_x[i], ay = p.wp_y[i], bx = p.wp_x[i + 1], by = p.wp_y[i + 1];
        for (int k = 0; k < 3; ++k) {
            double d = point_to_segment_dist(bar[k].x, bar[k].y, ax, ay, bx, by);
            if (d < bar[k].r - 1.0) { printf("FAIL: 栅栏路径穿盘 d=%.1f\n", d); return 1; }
        }
    }
    CircleObstacle wall[3] = {{90, 70, 10}, {90, 90, 10}, {90, 110, 10}};
    p = plan_route(30, 90, 170, 90, wall, 3);
    if (p.found) {
        for (int i = 0; i < p.n_wp - 1; ++i)
            for (int k = 0; k < 3; ++k) {
                double d = point_to_segment_dist(wall[k].x, wall[k].y,
                                                 p.wp_x[i], p.wp_y[i], p.wp_x[i + 1], p.wp_y[i + 1]);
                if (d < wall[k].r - 1.0) { printf("FAIL: 夹道场景不应有穿盘路径\n"); return 1; }
            }
    }
    CircleObstacle o = {100, 90, 10};
    p = plan_route(100, 90, 150, 90, &o, 1);   // S == 圆心
    if (p.found) { printf("FAIL: 起点在圆内应不可规划\n"); return 1; }
    printf("route cluster: OK (错位栅栏可通/夹道不穿盘/起点圆内回退)\n");
    return 0;
}

// A* 路径规划：与单圆盘闭式最优解对比（超 ≤12%）+ waypoint 不出界
static double tangent_opt_len(double sx, double sy, double tx, double ty,
                              double cx, double cy, double r) {
    double dS = std::hypot(sx - cx, sy - cy), dT = std::hypot(tx - cx, ty - cy);
    if (dS <= r || dT <= r) return -1.0;
    double bS = std::acos(r / dS), bT = std::acos(r / dT);
    double aS = std::atan2(sy - cy, sx - cx), aT = std::atan2(ty - cy, tx - cx);
    auto wrap = [](double a) {
        const double T2 = 2.0 * 3.14159265358979323846;
        while (a < 0) a += T2;
        while (a >= T2) a -= T2;
        return a;
    };
    double sweep = std::min(wrap((aS - bS) - (aT + bT)), wrap((aT - bT) - (aS + bS)));
    return std::sqrt(dS * dS - r * r) + std::sqrt(dT * dT - r * r) + r * sweep;
}

static int test_route_optimality() {
    {
        CircleObstacle o = {100, 90, 10};
        RoutePlan p = plan_route(50, 90, 150, 90, &o, 1);
        double opt = tangent_opt_len(50, 90, 150, 90, 100, 90, 10);
        if (!p.found) { printf("FAIL: 单圆应可绕行\n"); return 1; }
        printf("  单圆用例：A* %.2f / 解析最优 %.2f（超 %.1f%%）\n",
               p.length, opt, 100.0 * (p.length / opt - 1.0));
        if (p.length > opt * 1.10) { printf("FAIL: A* 比解析最优长 >10%%\n"); return 1; }
        if (p.length < opt - 0.6) { printf("FAIL: A* 比解析最优还短（说明贴进圆里了）\n"); return 1; }
    }
    uint64_t s = 987654321ull;
    auto rnd = [&s]() { s = s * 6364136223846793005ull + 1442695040888963407ull;
                        return (double)((s >> 33) & 0x7FFFFFFF) / 2147483647.0; };
    int n_ok = 0;
    double worst = 0.0;
    for (int trial = 0; trial < 200; ++trial) {
        double r = 6.0 + rnd() * 12.0;
        double sx = 20.0 + rnd() * 40.0, tx = 160.0 + rnd() * 40.0;
        double sy = 40.0 + rnd() * 100.0, ty = 40.0 + rnd() * 100.0;
        double t = 0.35 + rnd() * 0.3;
        double cx = sx + (tx - sx) * t, cy = sy + (ty - sy) * t;
        CircleObstacle ob = {cx, cy, r};
        RoutePlan p = plan_route(sx, sy, tx, ty, &ob, 1);
        if (!p.found) {
            printf("FAIL: 随机单圆 %d 应可绕 S(%.0f,%.0f) T(%.0f,%.0f) r=%.1f\n",
                   trial, sx, sy, tx, ty, r);
            return 1;
        }
        double opt = tangent_opt_len(sx, sy, tx, ty, cx, cy, r);
        double over = p.length / opt - 1.0;
        if (over > worst) worst = over;
        for (int i = 0; i < p.n_wp; ++i)
            if (p.wp_x[i] < -1 || p.wp_x[i] > 221 || p.wp_y[i] < -1 || p.wp_y[i] > 181) {
                printf("FAIL: waypoint 出界 (%.1f,%.1f)\n", p.wp_x[i], p.wp_y[i]);
                return 1;
            }
        if (over > 0.12) { printf("FAIL: 随机单圆 %d 超解析最优 %.1f%%\n", trial, over * 100); return 1; }
        ++n_ok;
    }
    printf("  随机单圆 %d 组：最坏超解析最优 %.1f%%\n", n_ok, worst * 100.0);

    {
        CircleObstacle ob = {100, 12, 10};
        RoutePlan p = plan_route(40, 12, 160, 12, &ob, 1);
        if (!p.found) { printf("FAIL: 贴边障碍应能从上方绕\n"); return 1; }
        for (int i = 0; i < p.n_wp; ++i)
            if (p.wp_x[i] < 2.0 || p.wp_x[i] > 218.0 || p.wp_y[i] < 2.0 || p.wp_y[i] > 178.0) {
                printf("FAIL: 贴边场景 waypoint 出界/贴死边线 (%.1f,%.1f)\n", p.wp_x[i], p.wp_y[i]);
                return 1;
            }
        printf("  贴边障碍：绕行 %.1fcm、%d 个 waypoint，全部在场内\n", p.length, p.n_wp);
    }
    printf("route optimality: OK (≤解析最优+12%% / 不短于最优 / 场边界内)\n");
    return 0;
}

// motion::follow_route 逐段执行单测
static int test_follow_route() {
    RoutePlan rt;
    rt.found = true;
    rt.n_wp = 3;
    rt.wp_x[0] = 0;    rt.wp_y[0] = 0;     // S
    rt.wp_x[1] = 60;   rt.wp_y[1] = 0;     // 中间点
    rt.wp_x[2] = 120;  rt.wp_y[2] = 0;     // T
    RobotState r;
    r.x = 5; r.y = 0; r.rot = 0; r.vl = r.vr = 0;
    int wp_next = 0;
    motion::follow_route(r, rt, wp_next);
    if (wp_next != 1) { printf("FAIL: 起点处应切到段1 got %d\n", wp_next); return 1; }
    if (!(r.vl > 0.0 && r.vr > 0.0)) { printf("FAIL: 第1段应朝+x走 vl=%.1f vr=%.1f\n", r.vl, r.vr); return 1; }
    r.x = 59;
    motion::follow_route(r, rt, wp_next);
    if (wp_next != 2) { printf("FAIL: 到达后应推进到段2 got %d\n", wp_next); return 1; }
    if (!(r.vl > 0.0 && r.vr > 0.0)) { printf("FAIL: 第2段应朝+x走\n"); return 1; }
    wp_next = 0;
    r.x = 118;
    motion::follow_route(r, rt, wp_next);
    if (wp_next != 2) { printf("FAIL: 越过后应推进到段2 got %d\n", wp_next); return 1; }
    if (!(r.vl > 0.0)) { printf("FAIL: 末段应继续朝终点\n"); return 1; }
    RoutePlan bad;
    bad.found = false;
    motion::follow_route(r, bad, wp_next);
    if (fabs(r.vl) > 1e-6 || fabs(r.vr) > 1e-6) { printf("FAIL: found=false 应停车\n"); return 1; }
    printf("follow route: OK (分段推进/到达切段/越过跳段/不可规划停车)\n");
    return 0;
}

// run_active 激进档位冒烟（远射档 / GK 封死 / 追球绕障）

// 禁区变角推射计数：未对准不推球、对准才计次、超 3 次停推、离射程清零
static int test_shoot_push_limit_impl();
// 回滚档（kNoAlignWait=0）的对准门禁验证
static int test_shoot_push_limit() {
    ::simuro5::set_param("roles.kNoAlignWait", 0.0);
    int rc = test_shoot_push_limit_impl();
    ::simuro5::reset_params();
    return rc;
}
static int test_shoot_push_limit_impl() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.ball.x = 40; wm.ball.y = 90;      // 距对方门 40cm（射程内）
    wm.ball.vx = 10.0; wm.ball.vy = 0.0; // 球被推动中（计次前置条件）
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 160 + i * 10; wm.home[i].y = 90; }
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 20; }
    wm.opp[0].x = 5; wm.opp[0].y = 90;   // GK 封门正中 → 开口两侧均 16cm，射门可行

    ShootPlan sp = plan_shoot(wm, 1);
    if (!sp.viable) { printf("FAIL: 门前 40cm 应可射\n"); return 1; }
    double prep_x = wm.ball.x - sp.dir_x * 20.0;   // 与 roles.cpp kPrepDist 一致
    double prep_y = wm.ball.y - sp.dir_y * 20.0;

    wm.home[1].x = prep_x; wm.home[1].y = prep_y;
    wm.home[1].rot = normalize_angle(sp.aim_rot + 40.0);
    wm.home[1].vl = wm.home[1].vr = 0;
    run_active(wm, 1);
    if (wm.shoot_push_count != 0) {
        printf("FAIL: 未对准(偏40°)不该推球 count=%d\n", wm.shoot_push_count);
        return 1;
    }
    if (!(wm.home[1].vl * wm.home[1].vr < 0.0)) {
        printf("FAIL: 未对准应原地转正(vl/vr 反号) got vl=%.1f vr=%.1f\n",
               wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    wm.home[1].x = prep_x; wm.home[1].y = prep_y;
    wm.home[1].rot = sp.aim_rot;
    wm.home[1].vl = wm.home[1].vr = 0;
    wm.shoot_push_cd = 0;
    run_active(wm, 1);
    if (wm.shoot_push_count != 1 || wm.shoot_push_cd <= 0) {
        printf("FAIL: 对准后应推穿计次1 got count=%d cd=%d\n",
               wm.shoot_push_count, wm.shoot_push_cd);
        return 1;
    }
    wm.shoot_push_count = 3; wm.shoot_push_cd = 0;
    wm.home[1].x = prep_x; wm.home[1].y = prep_y; wm.home[1].rot = sp.aim_rot;
    wm.home[1].vl = 0; wm.home[1].vr = 0;
    run_active(wm, 1);
    if (wm.shoot_push_count != 3) {
        printf("FAIL: 超限仍推球 count=%d\n", wm.shoot_push_count);
        return 1;
    }
    wm.ball.x = 160; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[1].x = 180; wm.home[1].y = 90;
    run_active(wm, 1);
    if (wm.shoot_push_count != 0) {
        printf("FAIL: 离射程未清零 count=%d\n", wm.shoot_push_count);
        return 1;
    }
    printf("shoot push limit: OK (未对准不推球/对准才推穿计次/超3次停推/离射程清零)\n");
    return 0;
}

// 运动控制：制动包线 / 停点收敛 / 语义开关 / 数值边界
// 平台动力学近似：v=(vl+vr)/2 cm/s、ω=(vr−vl)/10 rad/s、a_wheel=658 cm/s²（真机 p95）
struct PlantRobot { double x = 0, y = 0, rot = 0, vl = 0, vr = 0; };

static void plant_step(PlantRobot &p, const RobotState &cmd, double a_wheel, double dt,
                       double k_scale) {
    double maxdv = a_wheel * dt;
    auto lim = [maxdv](double from, double to) {
        if (to > from) return std::min(to, from + maxdv);
        return std::max(to, from - maxdv);
    };
    p.vl = lim(p.vl, cmd.vl);
    p.vr = lim(p.vr, cmd.vr);
    double v = (p.vl + p.vr) * 0.5 * k_scale;   // 轮速单位 → cm/s
    double w = (p.vr - p.vl) / 10.0;            // rad/s
    double rad = p.rot * SIMURO5_PI / 180.0;
    p.x += v * std::cos(rad) * dt;
    p.y += v * std::sin(rad) * dt;
    p.rot += w * dt * 180.0 / SIMURO5_PI;
}

// 旧律副本：改 motion.cpp 之前的 position()，冻结作对照基线，勿随新律同步修改
static void legacy_position(RobotState &r, double tx, double ty) {
    double dx = tx - r.x, dy = ty - r.y;
    double de = std::hypot(dx, dy);
    if (de < 1.0) { r.vl = 0.0; r.vr = 0.0; return; }
    double te = angle_diff(angle_to(r.x, r.y, tx, ty), r.rot);
    double vc = 150.0, Ka = 10.0 / 90.0;
    if (de > 100.0)      Ka = 20.0 / 90.0;
    else if (de > 50.0)  Ka = 22.0 / 90.0;
    else if (de > 30.0)  Ka = 24.0 / 90.0;
    else if (de > 20.0)  Ka = 26.0 / 90.0;
    else                 Ka = 28.0 / 90.0;
    double drive = vc * (1.0 / (1.0 + std::exp(-3.0 * de)) - 0.25);
    if (de < 12.0 && std::fabs(te) > 30.0) drive *= 0.25;
    if (te > 95.0 || te < -95.0) {
        te += (te > 0) ? -180.0 : 180.0;
        te = clamp(te, -80.0, 80.0);
        if (de < 5.0 && std::fabs(te) < 40.0) Ka = 0.1;
        r.vr = -drive + Ka * te; r.vl = -drive - Ka * te;
    } else if (te > -85.0 && te < 85.0) {
        if (de < 5.0 && std::fabs(te) < 40.0) Ka = 0.1;
        r.vr = drive + Ka * te; r.vl = drive - Ka * te;
    } else {
        r.vr = 0.17 * te; r.vl = -0.17 * te;
    }
}

// 用同一套平台动力学跑到点停住，返回统计量
struct StopTrace {
    double max_past = 0.0;     // 越过目标点的最大距离(cm，沿 +x)
    double settle_d = 1e9;     // 稳定后最终位置误差
    int    settle_f = 99999;   // 最后一次离开 ±3cm 的帧号（之后一直在带内）
    int    first_2cm = -1;     // 首次进入 2cm 的帧号
};

static StopTrace run_stop_task(double tx, double ty, double rot0, bool use_new_law,
                               double k_scale = 1.0, double a_wheel = 658.0) {
    const double dt = 1.0 / 40.0;
    PlantRobot p;
    p.rot = rot0;
    StopTrace tr;
    for (int f = 0; f < 400; ++f) {
        RobotState cmd;
        cmd.x = p.x; cmd.y = p.y; cmd.rot = p.rot;
        if (use_new_law) motion::position(cmd, tx, ty, motion::TM_STOP);
        else             legacy_position(cmd, tx, ty);
        plant_step(p, cmd, a_wheel, dt, k_scale);
        double d = dist(p.x, p.y, tx, ty);
        tr.max_past = std::max(tr.max_past, p.x - tx);
        if (d < 2.0 && tr.first_2cm < 0) tr.first_2cm = f;
        if (d > 3.0) tr.settle_f = f;      // 记最后一次出带；跑完仍在带内才算稳定
    }
    tr.settle_d = dist(p.x, p.y, tx, ty);
    return tr;
}

// 制动包线：命令速度 ≤ sqrt(2·a·(de−ε))；远距离不减速
static int test_motion_brake_envelope() {
    const double ds[] = {1.6, 2.0, 3.0, 5.0, 8.0, 12.0, 16.0, 17.5, 20.0, 30.0, 60.0, 100.0};
    for (double de : ds) {
        RobotState r;
        r.x = 0; r.y = 90; r.rot = 0;          // 朝 +x
        motion::position(r, de, 90, motion::TM_STOP);
        double v = (r.vl + r.vr) * 0.5;
        double allow = std::sqrt(2.0 * motion::kBrakeAccel * std::max(0.0, de - motion::kStopEps));
        if (v > allow + 1e-6) {
            printf("FAIL: de=%.1f 命令速度 %.1f 超包线 %.1f\n", de, v, allow);
            return 1;
        }
    }
    RobotState r;
    r.x = 0; r.y = 90; r.rot = 0;
    motion::position(r, motion::kStopEps - 0.01, 90, motion::TM_STOP);
    if (std::fabs(r.vl) > 1e-9 || std::fabs(r.vr) > 1e-9) {
        printf("FAIL: 死区内未停车 vl=%.2f vr=%.2f\n", r.vl, r.vr);
        return 1;
    }
    r.x = 0; r.y = 90; r.rot = 0;
    motion::position(r, 60.0, 90, motion::TM_STOP);
    if ((r.vl + r.vr) * 0.5 < 112.0) {
        printf("FAIL: de=60 巡航被降速 %.1f（应保持 112.5）\n", (r.vl + r.vr) * 0.5);
        return 1;
    }
    r.x = 0; r.y = 90; r.rot = 0;
    motion::position(r, 8.0, 90, motion::TM_STOP);
    if ((r.vl + r.vr) * 0.5 > 73.0) {          // sqrt(2*400*6.5)=72.1
        printf("FAIL: de=8 未按包线减速 %.1f\n", (r.vl + r.vr) * 0.5);
        return 1;
    }
    printf("motion brake envelope: OK (命令速度<=sqrt(2as)/死区停车/远距不降速)\n");
    return 0;
}

// 停点收敛：新律不过冲能稳定；侧向目标须 40 帧内触球
static int lateral_touch_frames(double scale, double damp, double *heading_turn) {
    const double s_save = get_param("motion.kWheelScaleSat", 1.0), d_save = get_param("motion.kLatDamp", 0.5);
    set_param("motion.kWheelScaleSat", scale); set_param("motion.kLatDamp", damp);
    RobotState r; r.x = 92.2; r.y = 47.2; r.rot = -63.0;
    const double bx = 86.6, by = 44.3;                   // 车坐标系下 前+0.3 侧-6.6
    int hit = -1;
    for (int f = 0; f < 120 && hit < 0; ++f) {
        motion::position(r, bx, by, motion::TM_PASS);
        double v = 0.5 * (r.vl + r.vr) * 0.025, w = (r.vr - r.vl) / 10.0 * 0.025 * 180.0 / SIMURO5_PI;
        r.rot = normalize_angle(r.rot + w);
        r.x += v * std::cos(r.rot * SIMURO5_PI / 180.0); r.y += v * std::sin(r.rot * SIMURO5_PI / 180.0);
        if (std::hypot(bx - r.x, by - r.y) < 4.5) hit = f;
    }
    *heading_turn = std::fabs(angle_diff(r.rot, -63.0));
    set_param("motion.kWheelScaleSat", s_save); set_param("motion.kLatDamp", d_save);
    return hit;
}
static int test_motion_lateral_target() {
    double turn_new = 0, turn_old = 0;
    int f_new = lateral_touch_frames(1.0, 0.5, &turn_new), f_old = lateral_touch_frames(0.0, 0.0, &turn_old);
    printf("motion lateral: 新 触球帧=%d 车头转了 %.0f° / 旧 触球帧=%d 车头转了 %.0f°（-1=120 帧没碰到）\n",
           f_new, turn_new, f_old, turn_old);
    if (f_new < 0 || f_new > 40) { printf("FAIL: 球在车侧 7cm，新口径 40 帧内未触球\n"); return 1; }
    return 0;
}

static int test_motion_stop_convergence() {
    StopTrace n1 = run_stop_task(120.0, 0.0, 0.0, true);        // 直冲 120cm
    StopTrace o1 = run_stop_task(120.0, 0.0, 0.0, false);       // 旧律对照
    printf("  [new] max_past=%.1fcm settle_err=%.2fcm last_out_frame=%d first_2cm=%d\n",
           n1.max_past, n1.settle_d, n1.settle_f, n1.first_2cm);
    printf("  [old] max_past=%.1fcm settle_err=%.2fcm last_out_frame=%d first_2cm=%d\n",
           o1.max_past, o1.settle_d, o1.settle_f, o1.first_2cm);

    if (n1.max_past > 3.0) {
        printf("FAIL: 新律过冲 %.1fcm > 3cm\n", n1.max_past);
        return 1;
    }
    if (n1.settle_d > 2.0) {
        printf("FAIL: 新律未停在目标 %.2fcm\n", n1.settle_d);
        return 1;
    }
    if (n1.settle_f > 250) {
        printf("FAIL: 新律稳定太慢 last_out_frame=%d\n", n1.settle_f);
        return 1;
    }
    if (o1.max_past <= 3.0) {
        printf("WARN: 旧律在标定物理下未过冲（max_past=%.1f）——docs/18 立论需复核\n", o1.max_past);
    }
    StopTrace n2 = run_stop_task(120.0, 0.0, 25.0, true);
    if (n2.settle_d > 2.0 || n2.settle_f > 300) {
        printf("FAIL: 带 25° 初始偏差未收敛 err=%.2fcm last_out=%d\n", n2.settle_d, n2.settle_f);
        return 1;
    }
    StopTrace n3 = run_stop_task(120.0, 0.0, 0.0, true, 0.9, 300.0);
    printf("  [sim-plant] max_past=%.1fcm settle_err=%.2fcm last_out_frame=%d\n",
           n3.max_past, n3.settle_d, n3.settle_f);
    if (n3.max_past > 5.0 || n3.settle_d > 2.0) {
        printf("FAIL: sim 平台口径残余过冲 %.1fcm / 停止误差 %.2fcm 超预期\n",
               n3.max_past, n3.settle_d);
        return 1;
    }
    printf("motion stop convergence: OK (不过冲/2cm 内停住/带 25° 偏差仍收敛/sim 口径残余<=5cm)\n");
    return 0;
}

// 语义开关：TM_PASS（追球/穿球）保持旧律行为，不被包线减速
static int test_motion_pass_mode() {
    RobotState r;
    r.x = 0; r.y = 90; r.rot = 0;
    motion::position(r, 5.0, 90, motion::TM_PASS);
    if (std::fabs((r.vl + r.vr) * 0.5 - 150.0) > 0.5) {
        printf("FAIL: TM_PASS de=5 应为 150.0 got %.1f\n", (r.vl + r.vr) * 0.5);
        return 1;
    }
    r.x = 0; r.y = 90; r.rot = 0;
    motion::position(r, 2.0, 90, motion::TM_PASS);          // 2cm 处旧律仍满速（bang-bang 原样）
    if ((r.vl + r.vr) * 0.5 < 100.0) {
        printf("FAIL: TM_PASS 应保持旧速律 got %.1f\n", (r.vl + r.vr) * 0.5);
        return 1;
    }
    r.x = 0; r.y = 90; r.rot = 0;
    motion::position(r, 0.5, 90, motion::TM_PASS);          // 0.5cm → 旧律的 de<1 停车
    if (std::fabs(r.vl) > 1e-9 || std::fabs(r.vr) > 1e-9) {
        printf("FAIL: TM_PASS 死区内应停车 vl=%.2f vr=%.2f\n", r.vl, r.vr);
        return 1;
    }
    RobotState c;
    c.x = 0; c.y = 90; c.rot = 0;
    BallState pred; pred.x = 2.0; pred.y = 90;
    motion::chase_ball(c, pred);
    if ((c.vl + c.vr) * 0.5 > 35.0) {
        printf("FAIL: chase_ball 贴球未减速 %.1f\n", (c.vl + c.vr) * 0.5);
        return 1;
    }
    printf("motion pass mode: OK (TM_PASS 不降速/贴球减速保留)\n");
    return 0;
}

// 数值边界：NaN/Inf 目标直接停车；轮速有限且有界
static int test_motion_bounds() {
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double inf_v = std::numeric_limits<double>::infinity();
    double bad[3] = {nan_v, inf_v, -inf_v};
    for (double t : bad) {
        RobotState r;
        r.x = 10; r.y = 90; r.rot = 0;
        motion::position(r, t, 90, motion::TM_STOP);
        if (r.vl != 0.0 || r.vr != 0.0) {
            printf("FAIL: 非法目标(%.0f)未停车 vl=%.1f vr=%.1f\n", t, r.vl, r.vr);
            return 1;
        }
    }
    const double rots[] = {0, 45, 90, 95, 120, 179, -179, -100, -90, -45};
    const double des[] = {0.01, 1.0, 5.0, 40.0, 200.0, 1e6};
    for (double rot : rots) for (double de : des) {
        RobotState r;
        r.x = 0; r.y = 0; r.rot = rot;
        motion::position(r, de, 0.0, motion::TM_STOP);
        if (!std::isfinite(r.vl) || !std::isfinite(r.vr) ||
            std::fabs(r.vl) > motion::kMaxWheel + 1e-9 ||
            std::fabs(r.vr) > motion::kMaxWheel + 1e-9) {
            printf("FAIL: rot=%.0f de=%.2f 轮速越界 vl=%.1f vr=%.1f\n", rot, de, r.vl, r.vr);
            return 1;
        }
    }
    printf("motion bounds: OK (NaN/Inf 停车/扫描输入有界有限)\n");
    return 0;
}

// 到点定向：三段式 + 物理积分收敛到位姿都对
static int test_motion_aligned() {
    const double dt = 1.0 / 40.0;
    const double kPlantAccel = 658.0;
    {
        RobotState r;
        r.x = 60; r.y = 0; r.rot = 180.0;
        bool ok = motion::position_aligned(r, 60.0, 0.0, 180.0);
        if (!ok || r.vl != 0.0 || r.vr != 0.0) {
            printf("FAIL: 已对准应返回 true 且停车 ok=%d vl=%.1f\n", (int)ok, r.vl);
            return 1;
        }
    }
    {
        RobotState r;
        r.x = 60; r.y = 0; r.rot = 140.0;      // 期望 180 → 偏 40°
        bool ok = motion::position_aligned(r, 60.0, 0.0, 180.0);
        if (ok) { printf("FAIL: 偏 40° 不该报就绪\n"); return 1; }
        if (!(r.vl * r.vr < 0.0)) {
            printf("FAIL: 转正应 vl/vr 反号 got vl=%.1f vr=%.1f\n", r.vl, r.vr);
            return 1;
        }
        if (std::fabs(r.vl) > motion::kMaxWheel || std::fabs(r.vr) > motion::kMaxWheel) {
            printf("FAIL: 转正轮速越界 vl=%.1f vr=%.1f\n", r.vl, r.vr);
            return 1;
        }
    }
    {
        RobotState r;
        r.x = 0; r.y = 0; r.rot = 0;
        bool ok = motion::position_aligned(r, 60.0, 0.0, 0.0);
        if (ok) { printf("FAIL: 60cm 外不该报就绪\n"); return 1; }
        if (!(r.vl > 0.0 && r.vr > 0.0)) {
            printf("FAIL: 远处应朝目标前进 got vl=%.1f vr=%.1f\n", r.vl, r.vr);
            return 1;
        }
    }
    {
        PlantRobot p;
        p.x = 10.0; p.y = 0.0; p.rot = 0.0;
        bool ready = false; int ready_frame = -1;
        for (int f = 0; f < 200; ++f) {
            RobotState cmd;
            cmd.x = p.x; cmd.y = p.y; cmd.rot = p.rot;
            if (motion::position_aligned(cmd, 60.0, 0.0, 0.0)) {
                ready = true; ready_frame = f;       // 记录**首次**就绪帧（勿每帧覆盖）
                p.vl = p.vr = 0.0;
                break;
            }
            plant_step(p, cmd, kPlantAccel, dt, 1.0);
        }
        double err_pos = dist(p.x, p.y, 60.0, 0.0);
        double err_ang = std::fabs(angle_diff(0.0, p.rot));
        printf("  [aligned 直进] ready=%d frame=%d 位置误差=%.2fcm 朝向误差=%.1f°\n",
               (int)ready, ready_frame, err_pos, err_ang);
        if (!ready || ready_frame > 120) {
            printf("FAIL: 直进接近应 ≤120 帧就绪 got ready=%d frame=%d\n",
                   (int)ready, ready_frame);
            return 1;
        }
        if (err_pos > 3.0 || err_ang > 8.0) {
            printf("FAIL: 直进收敛不到位姿容差 pos=%.2f ang=%.1f\n", err_pos, err_ang);
            return 1;
        }
    }
    {
        PlantRobot p;
        p.x = 60.0; p.y = 0.0; p.rot = 0.0;
        bool ready = false; int ready_frame = -1;
        for (int f = 0; f < 200; ++f) {
            RobotState cmd;
            cmd.x = p.x; cmd.y = p.y; cmd.rot = p.rot;
            if (motion::position_aligned(cmd, 60.0, 0.0, 180.0)) {
                ready = true; ready_frame = f;       // 首次就绪帧
                p.vl = p.vr = 0.0;
                break;
            }
            plant_step(p, cmd, kPlantAccel, dt, 1.0);
        }
        double err_pos = dist(p.x, p.y, 60.0, 0.0);
        double err_ang = std::fabs(angle_diff(180.0, p.rot));
        printf("  [aligned 掉头] ready=%d frame=%d 位置误差=%.2fcm 朝向误差=%.1f°\n",
               (int)ready, ready_frame, err_pos, err_ang);
        if (!ready || ready_frame > 120) {
            printf("FAIL: 掉头 180° 应 ≤120 帧就绪（≈3s）got ready=%d frame=%d\n",
                   (int)ready, ready_frame);
            return 1;
        }
        if (err_pos > 3.0 || err_ang > 8.0) {
            printf("FAIL: 掉头收敛不到位姿容差 pos=%.2f ang=%.1f\n", err_pos, err_ang);
            return 1;
        }
    }
    printf("motion aligned: OK (就绪判定/原地转正/远处前进/物理积分位姿收敛)\n");
    return 0;
}

// 摆位语义：PM_X_Kick = X 队主罚；罚球人站「球后」（远离被攻球门那侧）
static int test_placement_semantics() {
    Robot r[5];
    Robot former[5] = {};
    TeamContext blue{true}, yellow{false};
    Vector3D ball;
    ball.z = 0;

    ball.x = 39.4; ball.y = 89.8;
    formation_later(blue, PM_PenaltyKick_Blue, former, ball, r);
    if (!(r[1].pos.x > ball.x + 4.0)) {
        printf("FAIL: 蓝队主罚点球，罚球人应站球后(x>%.1f)，实际 x=%.1f\n", ball.x + 4.0, r[1].pos.x);
        return 1;
    }
    if (!(r[0].pos.x > 200.0)) { printf("FAIL: 主罚点球时门将仍应守门(x>200)\n"); return 1; }
    ball.x = 180.6; ball.y = 89.8;
    formation_later(yellow, PM_PenaltyKick_Yellow, former, ball, r);
    if (!(r[1].pos.x < ball.x - 4.0)) {
        printf("FAIL: 黄队主罚点球应站球后(x<%.1f)，实际 x=%.1f\n", ball.x - 4.0, r[1].pos.x);
        return 1;
    }

    for (int i = 0; i < 5; ++i) { r[i].pos.x = -99; r[i].pos.y = -99; }
    formation_former(blue, PM_PenaltyKick_Yellow, r);
    if (!(r[0].pos.x > 200.0)) {
        printf("FAIL: 防守点球时门将必须在门线附近(x>200)，实际 x=%.1f\n", r[0].pos.x);
        return 1;
    }
    for (int i = 1; i < 5; ++i)
        if (!(r[i].pos.x > 110.0)) {
            printf("FAIL: 防守点球时 robot[%d] 必须在我方半场(x>110)，实际 %.1f\n", i, r[i].pos.x);
            return 1;
        }
    for (int i = 0; i < 5; ++i) { r[i].pos.x = -99; r[i].pos.y = -99; }
    formation_former(yellow, PM_PenaltyKick_Blue, r);
    if (!(r[0].pos.x < 20.0)) {
        printf("FAIL: 黄队防守点球时门将应贴 x=0 门线，实际 %.1f\n", r[0].pos.x);
        return 1;
    }
    for (int i = 1; i < 5; ++i)
        if (!(r[i].pos.x < 110.0)) {
            printf("FAIL: 黄队防守点球时 robot[%d] 应在黄队半场(x<110)，实际 %.1f\n", i, r[i].pos.x);
            return 1;
        }

    for (int i = 0; i < 5; ++i) { r[i].pos.x = -99; r[i].pos.y = -99; }
    formation_former(blue, PM_PlaceKick_Blue, r);
    if (!(r[1].pos.x > 110.0)) {
        printf("FAIL: 蓝队开球时开球人应在我方半场(球后 x>110)，实际 x=%.1f\n", r[1].pos.x);
        return 1;
    }
    for (int i = 0; i < 5; ++i) { r[i].pos.x = -99; r[i].pos.y = -99; }
    formation_former(yellow, PM_PlaceKick_Yellow, r);
    if (!(r[1].pos.x < 110.0)) {
        printf("FAIL: 黄队开球时开球人应在黄队半场(x<110)，实际 x=%.1f\n", r[1].pos.x);
        return 1;
    }

    printf("placement semantics: OK (主罚方站球后/防守方守门线+本方半场/开球人在本方半场)\n");
    return 0;
}

// 摆位规则纪律：除门将外不进己方门区；争球人纵向离球 25cm、其余人在争球 1/4 场外
static int test_placement_rule_boxes() {
    TeamContext blue{true}, yellow{false};
    Robot r[5];
    Robot former[5] = {};
    for (int t = 0; t < 2; ++t) {
        const TeamContext ctx = t ? yellow : blue;
        const char *team = ctx.is_blue ? "蓝队" : "黄队";
        for (int gi = PM_FreeBall_LeftTop; gi <= PM_GoalKick_Blue; ++gi) {
            const PlayMode gs = (PlayMode)gi;
            Vector3D ball; ball.z = 0.0; ball.x = 110.0; ball.y = 90.0;
            if (gi <= PM_FreeBall_RightBot) {                       // 争球：1/4 区中心
                ball.x = (gi == PM_FreeBall_RightTop || gi == PM_FreeBall_RightBot) ? 165.0 : 55.0;
                ball.y = (gi == PM_FreeBall_LeftTop || gi == PM_FreeBall_RightTop) ? 135.0 : 45.0;
            } else if (gs == PM_PenaltyKick_Blue || gs == PM_PenaltyKick_Yellow) {
                ball.x = (gs == PM_PenaltyKick_Blue) ? 39.4 : 180.6;  // 罚球点：门前 39.4cm
                ball.y = 89.8;
            }
            for (int pass = 0; pass < 2; ++pass) {
                for (int i = 0; i < 5; ++i) { r[i].pos.x = 0.0; r[i].pos.y = 0.0; r[i].rotation = 0.0; }
                if (pass == 0) formation_former(ctx, gs, r);
                else           formation_later(ctx, gs, former, ball, r);
                const char *who = pass == 0 ? "先摆" : "后摆";

                for (int i = 1; i < 5; ++i) {
                    if (in_goal_area(ctx, r[i].pos.x, r[i].pos.y)) {
                        printf("FAIL: %s %s gs=%d robot[%d] 摆在己方门区里 (%.1f,%.1f)\n",
                               team, who, gi, i, r[i].pos.x, r[i].pos.y);
                        return 1;
                    }
                }
                if (gi <= PM_FreeBall_RightBot) {
                    const bool right = ball.x > 110.0, top = ball.y > 90.0;
                    if (std::fabs(std::fabs(r[1].pos.x - ball.x) - 25.0) > 0.5 ||
                        std::fabs(r[1].pos.y - ball.y) > 0.5) {
                        printf("FAIL: %s %s gs=%d 争球人应离球纵向 25cm，实际 (%.1f,%.1f) 球 (%.1f,%.1f)\n",
                               team, who, gi, r[1].pos.x, r[1].pos.y, ball.x, ball.y);
                        return 1;
                    }
                    for (int i = 2; i < 5; ++i) {
                        const double x = r[i].pos.x, y = r[i].pos.y;
                        const bool in_quarter = ((x > 110.0) == right) && ((y > 90.0) == top);
                        const bool on_line = ((x > 110.0) == right) && (std::fabs(y - 90.0) <= 1.0);
                        if (in_quarter || on_line) {
                            printf("FAIL: %s %s gs=%d robot[%d]=(%.1f,%.1f) 落在争球 1/4 场内"
                                   "（球 (%.1f,%.1f) 该区 x%s110、y%s90）\n",
                                   team, who, gi, i, x, y, ball.x, ball.y,
                                   right ? ">" : "<", top ? ">" : "<");
                            return 1;
                        }
                    }
                }
            }
        }
    }
    printf("placement rule boxes: OK (除门将外不进己方门区/争球人纵向 25cm/其余人在争球 1/4 场外)\n");
    return 0;
}

// 门将开球「先转正再推穿」：对准即推穿，机头偏 90° 须 40 帧内转正
static int test_goalie_clear_push() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                    // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    wm.ball.x = 190.0; wm.ball.y = 89.7; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 150; wm.home[i].y = 90; wm.opp[i].x = 100; wm.opp[i].y = 90;
        wm.role[i] = ROLE_PASSIVE;
    }
    wm.role[0] = ROLE_GOALIE;

    double pdirx = 0.0, pdiry = 0.0;
    gk_restart_direction(wm, 0, wm.ball.x, wm.ball.y, pdirx, pdiry);
    double aim_rot = angle_to(0.0, 0.0, pdirx, pdiry);

    wm.home[0].x = wm.ball.x - pdirx * 10.0;   // 球后 10cm（沿推球方向）
    wm.home[0].y = wm.ball.y - pdiry * 10.0;
    wm.home[0].rot = aim_rot;
    run_goalie(wm, 0);
    double v1 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v1 < 30.0) {
        printf("FAIL: 门将对准后应直线推穿（有速度）v=%.0f\n", v1);
        return 1;
    }
    double rot = normalize_angle(aim_rot - 90.0);
    int conv = -1;
    for (int f = 0; f < 40; ++f) {
        wm.home[0].x = wm.ball.x - pdirx * 10.0;
        wm.home[0].y = wm.ball.y - pdiry * 10.0;
        wm.home[0].rot = rot;
        run_goalie(wm, 0);
        double w = (wm.home[0].vr - wm.home[0].vl) / 10.0;              // rad/s（平台口径）
        rot = normalize_angle(rot + w * 0.025 * 180.0 / SIMURO5_PI);    // dt=1/40s
        if (std::fabs(angle_diff(aim_rot, rot)) <= 20.0) { conv = f; break; }
    }
    if (conv < 0) { printf("FAIL: 门将没能转正到推球方向（仍在死区打转）\n"); return 1; }
    printf("goalie clear push: OK (对准即推穿 v=%.0f / 偏 90° 时 %d 帧内转正)\n", v1, conv);
    return 0;
}

// 贴门线静止球「直线推出」：门球往侧面空当推，被抢时直线远离己门
static int test_goalie_straight_clear() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                    // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 150; wm.home[i].y = 90; wm.opp[i].x = 100; wm.opp[i].y = 90;
        wm.role[i] = ROLE_PASSIVE;
    }
    wm.role[0] = ROLE_GOALIE;

    wm.ball.x = 205.0; wm.ball.y = 89.7; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    double pdirx = 0.0, pdiry = 0.0;
    gk_restart_direction(wm, 0, wm.ball.x, wm.ball.y, pdirx, pdiry);
    if (std::fabs(pdiry) < 0.3) {
        printf("FAIL: 门球出球方向应是侧面(带 y 分量)，实际 dir=(%.2f,%.2f) 仍直线\n", pdirx, pdiry);
        return 1;
    }
    wm.home[0].x = wm.ball.x - pdirx * 10.0;
    wm.home[0].y = wm.ball.y - pdiry * 10.0;
    wm.home[0].rot = angle_to(0.0, 0.0, pdirx, pdiry);
    run_goalie(wm, 0);
    double v1 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v1 < 30.0) {
        printf("FAIL: 门球侧面方向对准后应推穿(有速度) v=%.0f\n", v1);
        return 1;
    }

    wm.ball.x = 183.0; wm.ball.y = 89.7; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.opp[0].x = 152.0; wm.opp[0].y = 89.7;                              // 对手距球 31cm < 40
    wm.home[0].x = 215.0; wm.home[0].y = 89.7; wm.home[0].rot = 180.0;   // 门侧，距球 32cm
    run_goalie(wm, 0);
    double v2 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v2 < 30.0) {
        printf("FAIL: 被抢球门侧应直线推出(抢得过对手) v=%.0f\n", v2);
        return 1;
    }

    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 100; wm.opp[i].y = 90; }   // 先重置
    wm.opp[0].x = 180.0; wm.opp[0].y = 89.7;                              // 对手距球 ~25cm < 40
    wm.ball.x = 205.4; wm.ball.y = 89.7; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[0].x = 214.8; wm.home[0].y = 90.3; wm.home[0].rot = -90.0;
    double rot3 = -90.0;
    int conv3 = -1;
    for (int f = 0; f < 40; ++f) {
        wm.home[0].x = 214.8; wm.home[0].y = 90.3;
        wm.home[0].rot = rot3;
        run_goalie(wm, 0);
        double w = (wm.home[0].vr - wm.home[0].vl) / 10.0;              // rad/s（平台口径）
        rot3 = normalize_angle(rot3 + w * 0.025 * 180.0 / SIMURO5_PI);    // dt=1/40s
        if (std::fabs(angle_diff(180.0, rot3)) <= 20.0) { conv3 = f; break; }
    }
    if (conv3 < 0) {
        printf("FAIL: 被抢球+机头垂直(-90°)时门将没转正到 180°（仍在死区画弧）\n");
        return 1;
    }
    wm.home[0].x = 214.8; wm.home[0].y = 90.3; wm.home[0].rot = 180.0;
    run_goalie(wm, 0);
    double v3 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v3 < 30.0) {
        printf("FAIL: 被抢球转正后应直线推穿 v=%.0f\n", v3);
        return 1;
    }
    printf("goalie straight clear: OK (门球侧面推穿 v1=%.0f / 被抢球直线 v2=%.0f v3=%.0f 转正%df)\n",
           v1, v2, v3, conv3);
    return 0;
}

// 门将发球要推远：准备点带速不刹停（≥135），横偏 13cm 则回摆准备点
static int test_goalie_kick_far() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                    // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 150; wm.home[i].y = 90;
        wm.opp[i].x = 60; wm.opp[i].y = 90;        // 对手 >40cm 不抢 → 走"无人抢"分支
        wm.role[i] = ROLE_PASSIVE;
    }
    wm.role[0] = ROLE_GOALIE;

    wm.ball.x = 220.0 - 40.0; wm.ball.y = 90.0; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    double dx = 0.0, dy = 0.0;
    gk_restart_direction(wm, 0, wm.ball.x, wm.ball.y, dx, dy);
    wm.home[0].x = wm.ball.x - dx * 28.0;
    wm.home[0].y = wm.ball.y - dy * 28.0;
    wm.home[0].rot = angle_to(0.0, 0.0, dx, dy);
    run_goalie(wm, 0);
    double v1 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v1 < 135.0) {
        printf("FAIL: 带速穿球应在准备点前不刹车（v=%.0f，应≥135；旧 TM_STOP 包线只给 ~122）\n", v1);
        return 1;
    }

    double px = wm.ball.x - dx * 8.0, py = wm.ball.y - dy * 8.0;
    wm.home[0].x = px - dy * 13.0;
    wm.home[0].y = py + dx * 13.0;
    wm.home[0].rot = angle_to(0.0, 0.0, dy, -dx);   // 机头对着准备点，避免落进自转死区
    run_goalie(wm, 0);
    double v2 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v2 > 110.0) {
        printf("FAIL: 横向差 13cm 擦不到球，不该高速直冲（v=%.0f，应≤110 回准备点套包线）\n", v2);
        return 1;
    }

    ::simuro5::set_param("roles.kGkPrepPass", 0.0);
    wm.home[0].x = wm.ball.x - dx * 28.0;
    wm.home[0].y = wm.ball.y - dy * 28.0;
    run_goalie(wm, 0);
    double v3 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    ::simuro5::set_param("roles.kGkPrepPass", 1.0);
    if (v3 > 130.0) {
        printf("FAIL: 关掉 kGkPrepPass 后应回到刹停口径（v=%.0f，应≤130 对应包线 ~122）\n", v3);
        return 1;
    }

    printf("goalie kick far: OK (带速准备点 v=%.0f / 横偏13cm 回摆 v=%.0f / 关旋钮 v=%.0f 回旧口径)\n",
           v1, v2, v3);
    return 0;
}

// 门前清道夫：仅当 B4 比球更靠己门时朝球推，避免乌龙
static int test_passive_front_sweep() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                    // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 150; wm.home[i].y = 90;
        wm.opp[i].x = 200; wm.opp[i].y = 91;       // 对手逼近(<100cm)，满足门前协防触发条件
        wm.role[i] = ROLE_PASSIVE;
    }
    wm.role[4] = ROLE_PASSIVE;

    wm.ball.x = 198.0; wm.ball.y = 91.0; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[4].x = 203.0; wm.home[4].y = 91.0; wm.home[4].rot = 180.0;
    run_passive(wm, 4);
    double v1 = 0.5 * (wm.home[4].vl + wm.home[4].vr);
    if (v1 < 10.0) {
        printf("FAIL: 门前清道夫没出手（球门侧 v=%.0f，应 >10 朝 -x 把球顶离己门）\n", v1);
        return 1;
    }

    wm.ball.x = 212.0; wm.ball.y = 91.0;
    wm.home[4].x = 219.0; wm.home[4].y = 91.0; wm.home[4].rot = 180.0;
    run_passive(wm, 4);
    double v1b = 0.5 * (wm.home[4].vl + wm.home[4].vr);
    if (std::fabs(v1b) > 5.0) {
        printf("FAIL: 球在裁判门区内清道夫不该出手（v=%.0f，应 ≈0，交门将）\n", v1b);
        return 1;
    }

    wm.home[4].x = 205.0; wm.home[4].y = 91.0; wm.home[4].rot = 0.0;
    run_passive(wm, 4);
    double v2 = 0.5 * (wm.home[4].vl + wm.home[4].vr);
    if (std::fabs(v2) > 5.0) {
        printf("FAIL: 场侧不该直撞球（v=%.0f，应 ≈0 停轮站定）\n", v2);
        return 1;
    }

    printf("passive front sweep: OK (球门侧主动清球 v1=%.0f / 场侧停轮不撞 v2=%.0f)\n", v1, v2);
    return 0;
}

// 贴门线时「球外侧禁推」护栏：门将在场侧则让开，避免把球顶进自家门
static int test_gk_on_line_no_push() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                    // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 150; wm.home[i].y = 90; wm.opp[i].x = 100; wm.opp[i].y = 90;
        wm.role[i] = ROLE_PASSIVE;
    }
    wm.role[0] = ROLE_GOALIE;

    wm.ball.x = 218.99; wm.ball.y = 75.44; wm.ball.vx = 0.328; wm.ball.vy = -2.021;
    wm.home[0].x = 214.85; wm.home[0].y = 71.31; wm.home[0].rot = -50.8;
    double tx = 0.0, ty = 0.0;
    if (!gk_side_step_point(wm, 0, tx, ty)) {
        printf("FAIL: 球离门线1cm且门将在场侧，'球外侧禁推'护栏没拦下（门将会把球顶进自家门）\n");
        return 1;
    }

    wm.ball.x = 212.0; wm.ball.y = 91.0; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[0].x = 219.0; wm.home[0].y = 91.0;
    if (gk_side_step_point(wm, 0, tx, ty)) {
        printf("FAIL: 门将已在球门侧时不该被让开护栏拦下（会丢掉合法清球能力）\n");
        return 1;
    }

    printf("gk on-line no push: OK (贴门线1cm必让开 / 门将已在门侧仍可清球)\n");
    return 0;
}

// 对方门口盘带：门将上前封角度，而非退锁门线
static int test_goalie_challenge() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                    // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 150; wm.home[i].y = 90; wm.opp[i].x = 100; wm.opp[i].y = 90;
        wm.role[i] = ROLE_PASSIVE;
    }
    wm.role[0] = ROLE_GOALIE;
    wm.ball.x = 192.0; wm.ball.y = 89.7; wm.ball.vx = 0.0; wm.ball.vy = 0.0;  // 离门 28cm
    wm.opp[0].x = 194.0; wm.opp[0].y = 89.7;                                  // 距球 2cm → 持球
    wm.home[0].x = 207.0; wm.home[0].y = 89.7; wm.home[0].rot = 180.0;        // 离球 15cm、门线外 13cm
    run_goalie(wm, 0);
    double v = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v <= 0.0) {
        printf("FAIL: 门口盘带门将应上前封角度(-x 前进)，实际 v=%.0f(倒退回门线)\n", v);
        return 1;
    }
    printf("goalie challenge: OK (门口盘带门将上前封角度 v=%.0f)\n", v);
    return 0;
}

// 松球/即将接球也夹抢：持球判定 8→15cm
static int test_doubleteam_loose_ball() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                    // 蓝队，己方门 x=220
    wm.ball.valid = true;
    wm.threat_level = 0.9;
    wm.sweeper_id = -1;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        wm.opp[i].x = 60; wm.opp[i].y = 90; wm.opp_vx[i] = 0; wm.opp_vy[i] = 0;
        wm.home[i].x = 150; wm.home[i].y = 90; wm.role[i] = ROLE_ASSIST;
    }
    wm.role[2] = ROLE_MIDFIELD;
    wm.ball.x = 180; wm.ball.y = 90;
    wm.opp[0].x = 180; wm.opp[0].y = 102;          // 离球 12cm（旧门槛 8 会拒，15 放行）
    double dx = 0, dy = 0;
    if (!double_team_point(wm, 1, dx, dy)) {
        printf("FAIL: 松球(离球12cm)对手压门前应夹抢（持球判定应 8→15 放行）\n");
        return 1;
    }
    wm.opp[0].y = 110;                             // 离球 20cm
    if (double_team_point(wm, 1, dx, dy)) {
        printf("FAIL: 真散球(离球20cm)不该夹抢\n");
        return 1;
    }
    printf("doubleteam loose ball: OK (松球12cm夹抢 / 散球20cm不夹抢)\n");
    return 0;
}

// 球权来源：纯距离自算，平台 whosBall 只作标定诊断计数（whos_mismatch）
static int test_possession_source() {
    WorldModel wm;
    wm.ctx = TeamContext{true};
    wm.ball.valid = true;
    wm.ball.x = 110; wm.ball.y = 90;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        wm.home[i].x = 200; wm.home[i].y = 90;
        wm.opp[i].x  = 200; wm.opp[i].y  = 90;
    }
    SituationModule sitm;
    wm.whos_ball = 0; wm.home[1].x = 115;
    if (!sitm.analyze(wm).we_have_ball) { printf("FAIL: whos=0 应按距离判我方\n"); return 1; }
    wm.home[1].x = 200; wm.opp[1].x = 115;
    if (sitm.analyze(wm).we_have_ball) { printf("FAIL: whos=0 对方更近应为对方\n"); return 1; }
    wm.home[1].x = 115; wm.opp[1].x = 200; wm.whos_ball = 2;
    Situation s3 = sitm.analyze(wm);
    if (!s3.we_have_ball) { printf("FAIL: 明确控球时不该被平台字段覆盖\n"); return 1; }
    if (!s3.whos_mismatch) { printf("FAIL: 应记录 平台 vs 自算 不一致\n"); return 1; }
    wm.home[1].x = 140; wm.opp[1].x = 140; wm.whos_ball = 1;
    if (sitm.analyze(wm).we_have_ball) { printf("FAIL: 散球时 whos=1 不再该信平台\n"); return 1; }
    wm.home[1].x = 128; wm.opp[1].x = 150; wm.whos_ball = 2;
    if (!sitm.analyze(wm).we_have_ball) { printf("FAIL: 散球时我方更近应自算判我方\n"); return 1; }
    printf("possession source: OK (距离自算/明确控球不覆盖/散球不信平台)\n");
    return 0;
}

// 前场散球逼抢：谁近谁去 + 球在球员前方加分（只进攻三人组）
static int test_presser() {
    Strategy strat;

    {
        WorldModel wm;
        wm.ctx = TeamContext{true};            // 蓝队，对方门 x=0，前场 x<110
        wm.ball.valid = true;
        wm.ball.x = 50; wm.ball.y = 90;        // 前场
        wm.ball.vx = 0; wm.ball.vy = 0;        // 静止
        wm.game_state = PM_PlayOn;
        wm.home[0].x = 210; wm.home[0].y = 90; // 门将
        wm.home[1].x = 150; wm.home[1].y = 90; // ACTIVE 离球 100
        wm.home[2].x = 70;  wm.home[2].y = 90; // ASSIST 离球 20，球在助攻前方(50<70)
        wm.home[3].x = 100; wm.home[3].y = 90; // MID 离球 50
        wm.home[4].x = 120; wm.home[4].y = 90; // PASSIVE 离球 70
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 90; }
        strat.run(wm);
        if (wm.presser_id != 2) {
            printf("FAIL: 前场静止散球应选离球最近的助攻(编号2)，got %d\n", wm.presser_id);
            return 1;
        }
    }

    {
        WorldModel wm;
        wm.ctx = TeamContext{true};
        wm.ball.valid = true;
        wm.ball.x = 50; wm.ball.y = 90;
        wm.ball.vx = 2.0; wm.ball.vy = 0;       // 球在动
        wm.game_state = PM_PlayOn;
        wm.home[0].x = 210; wm.home[0].y = 90;
        wm.home[1].x = 150; wm.home[1].y = 90;
        wm.home[2].x = 70;  wm.home[2].y = 90;
        wm.home[3].x = 100; wm.home[3].y = 90;
        wm.home[4].x = 120; wm.home[4].y = 90;
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 90; }
        wm.opp[0].x = 80; wm.opp[0].y = 90;     // 对手离球 30cm（<40 阈值）
        strat.run(wm);
        if (wm.presser_id != -1) {
            printf("FAIL: 球在动且周围有对方，不应触发逼抢，got %d\n", wm.presser_id);
            return 1;
        }
    }

    {
        WorldModel wm;
        wm.ctx = TeamContext{true};
        wm.ball.valid = true;
        wm.ball.x = 50; wm.ball.y = 90;
        wm.ball.vx = 0; wm.ball.vy = 0;
        wm.game_state = PM_PlayOn;
        wm.home[0].x = 210; wm.home[0].y = 90;
        wm.home[1].x = 150; wm.home[1].y = 90; // ACTIVE 离球 100
        wm.home[2].x = 80;  wm.home[2].y = 90; // ASSIST 离球 30
        wm.home[3].x = 100; wm.home[3].y = 90; // MID 离球 50
        wm.home[4].x = 75;  wm.home[4].y = 90; // PASSIVE 离球 25（最近但应排除）
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 90; }
        strat.run(wm);
        if (wm.presser_id == 4 || wm.presser_id == -1) {
            printf("FAIL: 中卫离球最近也不该被选，应选进攻球员，got %d\n", wm.presser_id);
            return 1;
        }
    }

    printf("presser: OK (最近进攻球员/球周围有对方不触发/中卫不选)\n");
    return 0;
}

// 抢反弹位三合一 + 二抢一封推进方向
static int test_rebound_and_doubleteam() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                  // 蓝队，己方门 x=220
    wm.ball.valid = true;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        wm.opp[i].x = 100 + i; wm.opp[i].y = 30 + i * 20;   // 先都放远处（不是门将）
        wm.home[i].x = 150; wm.home[i].y = 90; wm.role[i] = ROLE_ASSIST;
    }
    wm.ball.x = 150; wm.ball.y = 20; wm.ball.vx = 2.0; wm.ball.vy = -2.0;
    wm.opp[0].x = 218; wm.opp[0].y = 90;         // 门将
    double rx = 0, ry = 0;
    rebound_point(wm, 30.0, rx, ry);
    if (ry < 70.0 || ry > 110.0) { printf("FAIL: 反弹位 y 应在门框内 got %.1f\n", ry); return 1; }
    if (std::fabs(rx - 140.0) > 0.5) { printf("FAIL: 反弹位 x 应为罚球区前缘 140 got %.1f\n", rx); return 1; }
    wm.ball.x = 150; wm.ball.y = 100; wm.ball.vx = 5.0; wm.ball.vy = 0.0;
    double rx2 = 0, ry2 = 0;
    rebound_point(wm, 0.0, rx2, ry2);
    wm.opp[0].y = 90;
    double rx3 = 0, ry3 = 0;
    wm.ball.y = 104;                             // 落点在门将上侧
    rebound_point(wm, 0.0, rx3, ry3);
    wm.ball.y = 76;                              // 落点在门将下侧 → 偏向相反
    double rx4 = 0, ry4 = 0;
    rebound_point(wm, 0.0, rx4, ry4);
    if (!(ry3 > ry4)) { printf("FAIL: 上侧来球反弹位应更高 ry3=%.1f ry4=%.1f\n", ry3, ry4); return 1; }
    (void)ry2; (void)rx2;
    wm.ball.x = 150; wm.ball.y = 100; wm.ball.vx = 5.0; wm.ball.vy = 0.0;
    double a_x = 0, a_y = 0;
    rebound_point(wm, 0.0, a_x, a_y);
    wm.opp[1].x = a_x; wm.opp[1].y = a_y;        // 对手正站我们落点
    double b_x = 0, b_y = 0;
    rebound_point(wm, 0.0, b_x, b_y);
    if (std::fabs(b_y - a_y) < 15.0) { printf("FAIL: 落点被占应让开(差 %.1f)\n", std::fabs(b_y - a_y)); return 1; }

    wm.threat_level = 0.9;
    wm.sweeper_id = -1;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) { wm.opp[i].x = 60; wm.opp[i].y = 90; wm.opp_vx[i] = 0; wm.opp_vy[i] = 0; }
    wm.ball.x = 60; wm.ball.y = 90;              // 持球者=opp[0]，离球 0cm（<15 判定）
    wm.opp[0].x = 60; wm.opp[0].y = 60;          // 离门 160cm ✗ >100 → 不夹抢
    double dx2 = 0, dy2 = 0;
    if (double_team_point(wm, 1, dx2, dy2)) { printf("FAIL: 离门太远不该夹抢\n"); return 1; }
    wm.opp[0].x = 180; wm.opp[0].y = 90;         // 离门 40cm（<45 才允许第二人进禁区协防）
    wm.ball.x = 180; wm.ball.y = 90;
    wm.home[1].x = 150; wm.home[1].y = 90;       // 我去夹（最近）
    wm.home[2].x = 60;  wm.home[2].y = 90;       // 队友更远 → 不抢我的活
    wm.role[2] = ROLE_MIDFIELD;
    wm.opp_vx[0] = 0.0; wm.opp_vy[0] = 3.0;      // 持球者向上推进
    if (!double_team_point(wm, 1, dx2, dy2)) { printf("FAIL: 门前持球应夹抢\n"); return 1; }
    if (!(dy2 > 90.0)) { printf("FAIL: 持球者向上推进时夹抢点应在其前方 y>90 got %.1f\n", dy2); return 1; }
    wm.opp_vx[0] = 0.0; wm.opp_vy[0] = 0.0;      // 静止 → 退回门侧站位
    double ex = 0, ey = 0;
    if (!double_team_point(wm, 1, ex, ey)) { printf("FAIL: 静止持球也应夹抢（门侧）\n"); return 1; }
    if (!(ex > dx2)) { printf("FAIL: 静止时应站门侧（x 更大）got %.1f vs %.1f\n", ex, dx2); return 1; }
    printf("rebound+doubleteam: OK (反射预测/反弹偏向/补射者让位/二抢一封推进方向)\n");
    return 0;
}

// 门将「球外侧禁推」：目标必须落球后 10cm + 侧向，绝不落球的门侧
static int test_goalie_side_step() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                  // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 180; wm.home[i].y = 90; }
    double tx = 0.0, ty = 0.0;

    wm.ball.x = 215; wm.ball.y = 95;
    wm.home[0].x = 205; wm.home[0].y = 95;
    if (!gk_side_step_point(wm, 0, tx, ty)) {
        printf("FAIL: 门将在球外侧且球到门口应触发侧向让开\n");
        return 1;
    }
    if (tx > wm.ball.x + 0.01) {     // 蓝队：门的 x 更大 → 绝不能比球更靠门
        printf("FAIL: 目标点越过了球（会自己把球顶进门）tx=%.1f 球x=%.1f\n", tx, wm.ball.x);
        return 1;
    }
    if (fabs(ty - 106.0) > 0.5 && fabs(ty - 74.0) > 0.5) {
        printf("FAIL: 侧向让开目标 y 应贴到 106/74 边界，实际 %.1f\n", ty);
        return 1;
    }
    wm.home[0].y = 135;   // 侧向已让开 40cm ≥ kGkSideClear(35)
    if (gk_side_step_point(wm, 0, tx, ty)) { printf("FAIL: 已让开就不该再拦\n"); return 1; }
    wm.home[0].y = 95; wm.home[0].x = 218;
    if (gk_side_step_point(wm, 0, tx, ty)) { printf("FAIL: 门将已在门侧不该拦\n"); return 1; }
    wm.ball.x = 150; wm.home[0].x = 140;
    if (gk_side_step_point(wm, 0, tx, ty)) { printf("FAIL: 球还远不该拦\n"); return 1; }
    wm.ctx = TeamContext{false};
    wm.ball.x = 5; wm.ball.y = 95; wm.home[0].x = 15; wm.home[0].y = 95;
    if (!gk_side_step_point(wm, 0, tx, ty)) { printf("FAIL: 黄队镜像应触发\n"); return 1; }
    if (tx < wm.ball.x - 0.01) {
        printf("FAIL: 黄队镜像目标越过了球 tx=%.1f 球x=%.1f\n", tx, wm.ball.x);
        return 1;
    }
    printf("goalie side step: OK (禁推/不越球/让开后放行/已在门侧放行/太远不拦/黄队镜像)\n");
    return 0;
}

// 罚点球射门执行：助跑短、ACTIVE 必须平移（不许纯自转原地磨蹭）
static int test_penalty_shot_prep() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                   // 蓝队攻左门(x=0)
    wm.ball.valid = true;
    wm.ball.x = 39.4; wm.ball.y = 89.8; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.in_penalty_exec = true;
    wm.we_have_ball = true;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 120; wm.home[i].y = 90; wm.home[i].rot = 180.0;
        wm.opp[i].x = 150; wm.opp[i].y = 90;
        wm.role[i] = ROLE_PASSIVE;
    }
    wm.role[1] = ROLE_ACTIVE;
    wm.home[1].x = 43.5; wm.home[1].y = 91.0; wm.home[1].rot = -179.9;

    if (!(shoot_prep_dist(wm) <= 20.0)) {
        printf("FAIL: 罚点球助跑应短(<=20) got %.1f\n", shoot_prep_dist(wm));
        return 1;
    }
    run_active(wm, 1);
    double v = 0.5 * (wm.home[1].vl + wm.home[1].vr);
    double spin = std::fabs(wm.home[1].vl) + std::fabs(wm.home[1].vr);
    if (std::fabs(v) < 10.0 && std::max(std::fabs(wm.home[1].vl), std::fabs(wm.home[1].vr)) < 2.0) {
        printf("FAIL: 罚点球时 ACTIVE 不做任何动作 vl=%.1f vr=%.1f\n",
               wm.home[1].vl, wm.home[1].vr);
        return 1;
    }
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 60; wm.opp[i].y = 90; }
    wm.ball.x = 39.4; wm.ball.y = 89.8; wm.ball.vx = 0; wm.ball.vy = 0;
    wm.in_penalty_exec = true;
    double d0 = -1.0;
    for (int f = 0; f < 5; ++f) {
        wm.home[1].rot = -179.9;
        run_active(wm, 1);
        double v = 0.5 * (wm.home[1].vl + wm.home[1].vr);
        double w = (wm.home[1].vr - wm.home[1].vl) / 10.0;
        wm.home[1].x += v * 0.025 * std::cos(wm.home[1].rot * SIMURO5_PI / 180.0);
        wm.home[1].y += v * 0.025 * std::sin(wm.home[1].rot * SIMURO5_PI / 180.0);
        wm.home[1].rot = normalize_angle(wm.home[1].rot + w * 0.025 * 180.0 / SIMURO5_PI);
        double d = dist(wm.home[1].x, wm.home[1].y, wm.ball.x, wm.ball.y);
        if (d0 < 0) d0 = d;
        if (d > d0 + 1.0) {
            printf("FAIL: 对手逼近时罚点球仍在后退（被截的根因）d=%.1f > d0=%.1f\n", d, d0);
            return 1;
        }
    }
    wm.in_penalty_exec = false;
    wm.ball.x = 60.0;
    const double prep_expect = simuro5::get_param("roles.kPrepDist", 20.0);
    if (std::fabs(shoot_prep_dist(wm) - prep_expect) > 0.01) {
        printf("FAIL: 常规助跑应 %.1f（当前 kPrepDist）got %.1f\n", prep_expect, shoot_prep_dist(wm));
        return 1;
    }

    {
        WorldModel w2;
        w2.ctx = TeamContext{true};                 // 蓝队攻左门(x=0)
        w2.ball.valid = true;
        w2.ball.x = 39.4; w2.ball.y = 89.8; w2.ball.vx = 0; w2.ball.vy = 0;
        w2.in_penalty_exec = true;
        w2.we_have_ball = true;
        for (int i = 0; i < 5; ++i) {
            w2.home[i].x = 120; w2.home[i].y = 90; w2.home[i].rot = 180.0;
            w2.opp[i].x = 150; w2.opp[i].y = 90;
            w2.role[i] = ROLE_PASSIVE;
        }
        w2.role[1] = ROLE_ACTIVE;
        w2.home[1].x = 43.5; w2.home[1].y = 91.0; w2.home[1].rot = -179.9;
        w2.opp[0].x = 5.2; w2.opp[0].y = 90.0;
        double d0 = dist(w2.home[1].x, w2.home[1].y, w2.ball.x, w2.ball.y);
        for (int f = 0; f < 12; ++f) {
            run_active(w2, 1);
            double v = 0.5 * (w2.home[1].vl + w2.home[1].vr);
            double w = (w2.home[1].vr - w2.home[1].vl) / 10.0;
            w2.home[1].x += v * 0.025 * std::cos(w2.home[1].rot * SIMURO5_PI / 180.0);
            w2.home[1].y += v * 0.025 * std::sin(w2.home[1].rot * SIMURO5_PI / 180.0);
            w2.home[1].rot = normalize_angle(w2.home[1].rot + w * 0.025 * 180.0 / SIMURO5_PI);
            double d = dist(w2.home[1].x, w2.home[1].y, w2.ball.x, w2.ball.y);
            if (d > d0 + 1.0) {
                printf("FAIL: 对手 34cm 时罚点球仍在后退（真机重发 3 次的根因）"
                       "d=%.1f > d0=%.1f 帧=%d\n", d, d0, f);
                return 1;
            }
        }
        double lx = w2.pen_dir_x, ly = w2.pen_dir_y, lr = w2.pen_aim_rot, lyy = w2.pen_aim_y;
        w2.opp[0].x = 5.2; w2.opp[0].y = 70.0;      // 门将跑到下角
        w2.ball.x = 39.4; w2.ball.y = 89.8;
        run_active(w2, 1);
        if (std::fabs(w2.pen_dir_x - lx) > 1e-9 || std::fabs(w2.pen_dir_y - ly) > 1e-9 ||
            std::fabs(w2.pen_aim_rot - lr) > 1e-9 || std::fabs(w2.pen_aim_y - lyy) > 1e-9) {
            printf("FAIL: 点球执行期瞄准方向漂移了 (dir %.3f,%.3f→%.3f,%.3f)\n",
                   lx, ly, w2.pen_dir_x, w2.pen_dir_y);
            return 1;
        }
        w2.in_penalty_exec = false;
        run_active(w2, 1);
        if (w2.pen_aim_locked) { printf("FAIL: 点球结束后应解锁\n"); return 1; }
    }
    printf("penalty shot prep: OK (不倒车/对手34cm也不退/瞄准锁存/解锁/常规助跑20cm)\n");
    return 0;
}

// 罚点球瞄准本就偏离门将：球距门 39.4cm ⇒ 门张角 ±26.9°、门将遮挡 ±13.2°
static int test_penalty_aim_offcenter() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队攻左门(x=0)，门 y∈[70,110]
    wm.ball.valid = true;
    wm.ball.x = 39.4; wm.ball.y = 89.8; wm.ball.vx = 0; wm.ball.vy = 0;
    wm.in_penalty_exec = true;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 150; wm.home[i].y = 20 + i * 30; }
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 150; wm.opp[i].y = 20 + i * 30; }

    wm.opp[0].x = 5.2; wm.opp[0].y = 90.0;
    ShootPlan p1 = plan_shoot(wm, 1);
    if (!p1.viable || !p1.penalty) { printf("FAIL: 点球应可射\n"); return 1; }
    if (std::fabs(p1.aim_y - 90.0) < 10.0) {
        printf("FAIL: 居中门将时点球瞄准也须偏离门将，实际 aim_y=%.1f\n", p1.aim_y);
        return 1;
    }
    if (p1.aim_y < goal_y_low() || p1.aim_y > goal_y_high()) {
        printf("FAIL: 点球瞄准出框 aim_y=%.1f\n", p1.aim_y); return 1;
    }
    wm.opp[0].y = 74.0;
    ShootPlan p2 = plan_shoot(wm, 1);
    wm.opp[0].y = 106.0;
    ShootPlan p3 = plan_shoot(wm, 1);
    if (!(p2.aim_y > 90.0 && p3.aim_y < 90.0)) {
        printf("FAIL: 应打离门将远的一侧，实际 门将下 aim=%.1f / 门将上 aim=%.1f\n",
               p2.aim_y, p3.aim_y);
        return 1;
    }
    wm.in_penalty_exec = false;
    wm.ball.x = 60.0; wm.ball.y = 90.0;         // 距门 60cm（≤70 无条件可射区）
    wm.opp[0].x = 5.2; wm.opp[0].y = 90.0;
    ShootPlan p4 = plan_shoot(wm, 1);
    if (!p4.viable || p4.penalty) { printf("FAIL: 非点球应可射且 penalty=false\n"); return 1; }
    if (p4.aim_y < goal_y_low() - 0.1 || p4.aim_y > goal_y_high() + 0.1) {
        printf("FAIL: 非点球瞄准出框 aim_y=%.1f\n", p4.aim_y); return 1;
    }
    printf("penalty aim: OK (点球瞄准本就偏离门将/打远侧/非点球不变)\n");
    return 0;
}

static int test_penalty_spot_detect() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队：攻左门(x=0)，对方罚球点 x≈39.4
    wm.ball.valid = true;
    wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.ball.x = 39.4; wm.ball.y = 89.8;
    if (!we_take_penalty_spot(wm)) { printf("FAIL: 球停在对方罚球点应判我方点球\n"); return 1; }
    wm.ball.x = 180.6; wm.ball.y = 89.8;
    if (we_take_penalty_spot(wm)) { printf("FAIL: 我方罚球点上的球=对方主罚\n"); return 1; }
    wm.ball.x = 39.4; wm.ball.y = 89.8; wm.ball.vx = 2.0;
    if (we_take_penalty_spot(wm)) { printf("FAIL: 球在动不应判摆位期\n"); return 1; }
    wm.ball.vx = 0.0;
    wm.ball.x = 110; wm.ball.y = 90;
    if (we_take_penalty_spot(wm)) { printf("FAIL: 中圈静止球不应判点球\n"); return 1; }
    wm.ctx = TeamContext{false};
    wm.ball.x = 180.6; wm.ball.y = 89.8;
    if (!we_take_penalty_spot(wm)) { printf("FAIL: 黄队镜像应判我方(黄)点球\n"); return 1; }

    printf("penalty spot detect: OK (对方罚球点=我方主罚/我方罚球点/球在动/中圈/黄队镜像)\n");
    return 0;
}

// 配合进攻：按「接球后射门机会质量」选传球目标（+0.15 门槛、只向前传）
static int test_coop_pass() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队攻 x=0
    wm.ball.valid = true;
    wm.ball.x = 90; wm.ball.y = 90; wm.ball.vx = 0; wm.ball.vy = 0;
    wm.assist_x = 60;   wm.assist_y = 65;       // 助攻更靠对方门、且在门区外（向前、可接）
    wm.mid_x = 90;      wm.mid_y = 120;         // 中场在我方一侧（向后 → 不该被选）
    wm.passive_x = 150; wm.passive_y = 90;      // 后卫更靠后 → 不该被选
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 150; wm.home[i].y = 90; }
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 20 + i * 40; }

    if (in_opp_goal_area(wm.ctx, wm.assist_x, wm.assist_y) ||
        hypot(wm.assist_x, wm.assist_y - 90.0) > hypot(wm.ball.x, wm.ball.y - 90.0) - 5.0) {
        printf("FAIL: 测试前提错误：助攻接球点必须在门区外且满足向前传球\n"); return 1;
    }
    CoopPass cp = plan_coop_pass(wm, 1);
    if (!cp.viable || cp.receiver_id != 2) {
        printf("FAIL: 应传给更靠门的助攻(2)，实际 id=%d viable=%d\n",
               cp.receiver_id, (int)cp.viable);
        return 1;
    }
    if (cp.score <= 0.0) { printf("FAIL: 接球后射门分应>0\n"); return 1; }
    printf("coop pass legal forward: OK\n");

    wm.opp[0].x = 80; wm.opp[0].y = 82;         // ② 挡线，但距接球点至少 20cm，单独验证线路规则
    CircleObstacle blocker{wm.opp[0].x, wm.opp[0].y, 8.0};
    if (hypot(wm.opp[0].x - wm.assist_x, wm.opp[0].y - wm.assist_y) < 20.0 ||
        segment_clear_of_circles(wm.ball.x, wm.ball.y, wm.assist_x, wm.assist_y, &blocker, 1)) {
        printf("FAIL: 测试前提错误：对手必须挡线且不贴身\n"); return 1;
    }
    CoopPass cp2 = plan_coop_pass(wm, 1);
    if (cp2.viable && cp2.receiver_id == 2) { printf("FAIL: 线路被挡时不该传\n"); return 1; }
    printf("coop pass blocked line only: OK\n");
    wm.opp[0].x = 60; wm.opp[0].y = 50;         // ③ 距接球点 15cm、线段外 15cm，贴身但不挡线
    blocker = CircleObstacle{wm.opp[0].x, wm.opp[0].y, 8.0};
    if (hypot(wm.opp[0].x - wm.assist_x, wm.opp[0].y - wm.assist_y) >= 20.0 ||
        !segment_clear_of_circles(wm.ball.x, wm.ball.y, wm.assist_x, wm.assist_y, &blocker, 1)) {
        printf("FAIL: 测试前提错误：对手必须贴身且不挡线\n"); return 1;
    }
    CoopPass cp3 = plan_coop_pass(wm, 1);
    if (cp3.viable && cp3.receiver_id == 2) { printf("FAIL: 接球点被贴身时不该传\n"); return 1; }
    printf("coop pass close opponent only: OK\n");
    wm.opp[0].x = 200; wm.opp[0].y = 20;        // ④ 恢复干净局面 → 又能传
    if (!plan_coop_pass(wm, 1).viable) { printf("FAIL: 干净局面应可传\n"); return 1; }
    printf("coop pass restored clear: OK\n");
    printf("coop pass: OK (选最靠门的接球人/线路被挡不传/贴身不传/只向前)\n");
    return 0;
}

// 禁止「过冲后反向穿球」：主攻冲过球再沿瞄准线回推 = 把球顶向自家门
// 配合任务必须决定实际轮速，而不只是保存一份无人执行的坐标
static WorldModel coop_task_scene() {
        WorldModel wm;
        wm.ctx = TeamContext{true};
        wm.game_state = wm.game_state_last = PM_PlayOn;
        wm.ball.valid = true;
        wm.ball.x = 75; wm.ball.y = 150;
        wm.we_have_ball = true; wm.threat_level = 0.1;
        wm.assist_x = 55; wm.assist_y = 90;
        wm.mid_x = 130; wm.mid_y = 90;
        wm.passive_x = 150; wm.passive_y = 40;
        for (int i = 0; i < 5; ++i) {
            wm.home[i].x = 120; wm.home[i].y = 130;
            wm.opp[i].x = 200; wm.opp[i].y = 20 + 30 * i;
        }
        const double len = hypot(20.0, 60.0);
        wm.opp[0].x = 0; wm.opp[0].y = 90;
        wm.opp[1].x = 50; wm.opp[1].y = 134;
        wm.opp[2].x = 67; wm.opp[2].y = 165;
        wm.home[1].x = wm.ball.x + 20.0 / len * 5.0;
        wm.home[1].y = wm.ball.y + 60.0 / len * 5.0;
        wm.home[1].rot = angle_to(75, 150, 55, 90);
        return wm;
}

static int test_coop_pass_task() {
    auto scene = coop_task_scene;
    auto same_wheels = [](const RobotState &a, const RobotState &b) {
        return fabs(a.vl - b.vl) < 1e-8 && fabs(a.vr - b.vr) < 1e-8;
    };
    using Runner = void (*)(WorldModel &, int);
    const Runner runners[] = {run_assist, run_midfield, run_passive};
    set_param("roles.kRecvMeetBall", 0.0);
    set_param("roles.kRecvNoReverse", 0.0);
    for (int receiver = 2; receiver <= 4; ++receiver) {
        WorldModel wm = scene();
        if (receiver != 2) { wm.assist_x = 130; wm.assist_y = 140; }
        if (receiver == 3) { wm.mid_x = 55; wm.mid_y = 90; }
        if (receiver == 4) { wm.passive_x = 55; wm.passive_y = 90; }
        CoopPass cp = plan_coop_pass(wm, 1);
        ShootPlan sp = plan_shoot(wm, 1);
        if (!cp.viable || cp.receiver_id != receiver || cp.score <= sp.quality + 0.15) {
            printf("FAIL: coop task fixture receiver=%d actual=%d pass=%.3f shot=%.3f\n", receiver, cp.receiver_id, cp.score, sp.quality); return 1;
        }
        wm.home[receiver].x = cp.rx; wm.home[receiver].y = cp.ry;
        const int limit = (int)ceil(get_param("roles.kMaxShootPushes", 1.0));
        wm.shoot_push_count = limit;
        wm.ball.vx = cp.dir_x * 6; wm.ball.vy = cp.dir_y * 6;
        RobotState expected_passer = wm.home[1];
        motion::position(expected_passer, wm.ball.x + cp.dir_x * 20, wm.ball.y + cp.dir_y * 20, motion::TM_PASS);
        run_active(wm, 1);
        if (!wm.coop_pass_task.active || wm.coop_pass_task.receiver_id != receiver ||
            wm.coop_pass_task.passer_id != 1 || wm.coop_pass_task.rx != cp.rx || wm.coop_pass_task.ry != cp.ry ||
            wm.shoot_push_count != limit || !same_wheels(wm.home[1], expected_passer)) {
            printf("FAIL: coop task publish/push/count receiver=%d active=%d count=%d\n", receiver, (int)wm.coop_pass_task.active, wm.shoot_push_count); return 1;
        }
        wm.assist_x = wm.mid_x = wm.passive_x = 130;
        wm.assist_y = wm.mid_y = wm.passive_y = 150;
        RobotState expected_receiver = wm.home[receiver];
        motion::position(expected_receiver, cp.rx, cp.ry);
        WorldModel ordinary = wm; ordinary.coop_pass_task.active = false;
        runners[receiver - 2](ordinary, receiver);
        runners[receiver - 2](wm, receiver);
        if (!same_wheels(wm.home[receiver], expected_receiver) ||
            same_wheels(wm.home[receiver], ordinary.home[receiver])) {
            printf("FAIL: coop task receiver target overwritten id=%d\n", receiver); return 1;
        }
        for (int other = 2; other <= 4; ++other) {
            if (other == receiver) continue;
            WorldModel with_task = wm, without_task = wm;
            without_task.coop_pass_task.active = false;
            runners[other - 2](with_task, other); runners[other - 2](without_task, other);
            if (!same_wheels(with_task.home[other], without_task.home[other])) {
                printf("FAIL: coop task changed unrelated teammate id=%d\n", other); return 1;
            }
        }
        wm.we_have_ball = false; wm.no_possession_frames = 4;
        wm.team_state = TS_DEFENSE; wm.threat_level = 0.4;
        wm.ball.x += cp.dir_x * 3; wm.ball.y += cp.dir_y * 3;
        expected_passer = wm.home[1];
        motion::position(expected_passer, wm.ball.x + cp.dir_x * 20, wm.ball.y + cp.dir_y * 20, motion::TM_PASS);
        int left = wm.coop_pass_task.frames_left;
        run_active(wm, 1);
        runners[receiver - 2](wm, receiver);
        if (!wm.coop_pass_task.active || wm.coop_pass_task.rx != cp.rx || wm.coop_pass_task.ry != cp.ry ||
            wm.coop_pass_task.frames_left >= left || !same_wheels(wm.home[receiver], expected_receiver) ||
            !same_wheels(wm.home[1], expected_passer)) {
            printf("FAIL: coop task lost/reselected during loose ball id=%d\n", receiver); return 1;
        }
        wm.coop_pass_task.frames_left = 1;
        run_active(wm, 1);
        ordinary = wm; ordinary.coop_pass_task.active = false;
        runners[receiver - 2](ordinary, receiver);
        runners[receiver - 2](wm, receiver);
        if (wm.coop_pass_task.active || !same_wheels(wm.home[receiver], ordinary.home[receiver])) {
            printf("FAIL: coop task timeout/ordinary recovery id=%d\n", receiver); return 1;
        }
    }
    reset_params();   // 恢复接球会合点新行为（默认开）
    for (int reason = 0; reason < 9; ++reason) {
        WorldModel wm = scene(); run_active(wm, 1);
        if (!wm.coop_pass_task.active) { printf("FAIL: coop cancel fixture\n"); return 1; }
        if (reason == 0) wm.game_state = PM_PlaceKick_Blue;
        if (reason == 1) { wm.whos_ball = 2; wm.we_have_ball = false; wm.opp[0] = wm.home[1]; wm.opp[0].x = wm.ball.x; wm.opp[0].y = wm.ball.y; wm.home[1].x = 120; }
        if (reason == 2) wm.threat_level = 0.6;
        if (reason == 3) wm.in_penalty_exec = true;
        if (reason == 4) { wm.ball.x = 5; wm.ball.y = 5; }
        if (reason == 5) wm.ga_cooldown[2] = 10;
        if (reason == 6) { wm.opp[2].x = 60; wm.opp[2].y = 95; }
        if (reason == 7) { wm.ball.x = 180; wm.ball.y = 90; wm.ball.vx = wm.ball.vy = 0; }
        if (reason == 8) { wm.opp[2].x = 65; wm.opp[2].y = 120; }
        run_active(wm, 1);
        if (wm.coop_pass_task.active) { printf("FAIL: coop task not cancelled reason=%d\n", reason); return 1; }
        const CoopOutcome expected[] = {CoopOutcome::GameState, CoopOutcome::Intercepted,
            CoopOutcome::HighThreat, CoopOutcome::Penalty, CoopOutcome::Corner,
            CoopOutcome::GoalDiscipline, CoopOutcome::ReceiverMarked,
            CoopOutcome::EmergencyDefense, CoopOutcome::LaneBlocked};
        cancel_unsafe_coop_pass(wm);
        if (wm.coop_stats.created != 1 || wm.coop_stats.outcomes[(int)expected[reason]] != 1) {
            printf("FAIL: coop cancellation accounting reason=%d created=%lu expected=%lu active=%d\n", reason,
                   wm.coop_stats.created, wm.coop_stats.outcomes[(int)expected[reason]], (int)wm.coop_pass_task.active); return 1;
        }
    }
    for (int existing = 0; existing < 2; ++existing) {
        WorldModel wm = scene();
        wm.ball.x = existing ? 155 : 175; wm.ball.y = 150;
        wm.assist_x = 65; wm.assist_y = 105;
        for (int i = 1; i < 5; ++i) { wm.opp[i].x = 220; wm.opp[i].y = 0; }
        wm.mid_x = wm.passive_x = 210;
        wm.home[1].x = 195; wm.home[1].y = 150;
        if (existing) {
            run_active(wm, 1);
            if (!wm.coop_pass_task.active) { printf("FAIL: coop late safety fixture\n"); return 1; }
            wm.ball.x = 175;
        }
        CoopPass cp = plan_coop_pass(wm, 1);
        ShootPlan sp = plan_shoot(wm, 1);
        if (!cp.viable || cp.score <= sp.quality + 0.15 ||
            !in_no_push_zone(wm.ball.x - cp.dir_x * shoot_prep_dist(wm), wm.ball.y - cp.dir_y * shoot_prep_dist(wm))) {
            printf("FAIL: coop prep guard fixture pass=%.3f shot=%.3f\n", cp.score, sp.quality); return 1;
        }
        run_active(wm, 1);
        if (wm.coop_pass_task.active || wm.home[1].vl != 0 || wm.home[1].vr != 0) {
            printf("FAIL: coop unsafe prep published/retained task\n"); return 1;
        }
    }
    {
        WorldModel wm = scene();
        CoopPass scheduled = plan_coop_pass(wm, 1);
        wm.home[scheduled.receiver_id].x = scheduled.rx; wm.home[scheduled.receiver_id].y = scheduled.ry;
        run_active(wm, 1);
        CoopPassTask saved = wm.coop_pass_task;
        RobotState expected = wm.home[2]; motion::position(expected, saved.rx, saved.ry);
        Strategy strategy; strategy.run(wm);
        if (!wm.coop_pass_task.active || wm.coop_pass_task.rx != saved.rx || wm.coop_pass_task.ry != saved.ry ||
            !same_wheels(wm.home[2], expected)) {
            printf("FAIL: coop task overwritten by full strategy scheduling\n"); return 1;
        }
        Environment env; init_env(env, 75, 150); env.gameState = PM_PlaceKick_Blue;
        wm.update(&env, TeamContext{true});
        if (wm.coop_pass_task.active) { printf("FAIL: coop task survives game-state update\n"); return 1; }
    }
    printf("coop task: OK (3 receivers/shared target/loose ball/timeout/safety/shoot limit)\n");
    return 0;
}

static int test_coop_lifecycle() {
    using Phase = CoopPassPhase;
    auto frame = [](WorldModel &wm, double x, double y) {
        wm.ball_last = wm.ball;
        wm.ball.vx = x - wm.ball.x; wm.ball.vy = y - wm.ball.y;
        wm.ball.x = x; wm.ball.y = y;
        run_active(wm, 1);
        run_assist(wm, 2); run_midfield(wm, 3); run_passive(wm, 4);
    };
    auto stopped = [](const RobotState &r) { return r.vl == 0.0 && r.vr == 0.0; };
    auto start = [&](int receiver) {
        WorldModel wm = coop_task_scene();
        if (receiver == 3) { wm.assist_x = 130; wm.assist_y = 140; wm.mid_x = 55; wm.mid_y = 90; }
        wm.home[receiver].x = 55; wm.home[receiver].y = 90;
        frame(wm, 75, 150);
        return wm;
    };
    const double dx = -20.0 / hypot(20.0, 60.0), dy = -60.0 / hypot(20.0, 60.0);
    {
        WorldModel wm = start(2);
        for (int i = 0; i < 4; ++i) frame(wm, 75, 150);
        wm.ball.vx = dx * 6; wm.ball.vy = dy * 6; run_active(wm, 1);
        if (!wm.coop_pass_task.active || wm.coop_pass_task.phase != Phase::Preparing || wm.coop_ball_control.active) {
            printf("FAIL: coop lifecycle command/stale velocity mistaken for release\n"); return 1;
        }
        wm.coop_pass_task.frames_left = 1; frame(wm, 75, 150);
        if (wm.coop_pass_task.active) { printf("FAIL: coop unreleased timeout\n"); return 1; }
        cancel_unsafe_coop_pass(wm); cancel_unsafe_coop_pass(wm);
        if (wm.coop_stats.created != 1 || wm.coop_stats.released != 0 ||
            wm.coop_stats.outcomes[(int)CoopOutcome::PrepareTimeout] != 1) {
            printf("FAIL: coop prepare timeout statistics duplicated/missing created=%lu prep=%lu recv=%lu invalid=%lu active=%d\n",
                   wm.coop_stats.created, wm.coop_stats.outcomes[(int)CoopOutcome::PrepareTimeout],
                   wm.coop_stats.outcomes[(int)CoopOutcome::ReceiveTimeout],
                   wm.coop_stats.outcomes[(int)CoopOutcome::InvalidTarget], (int)wm.coop_pass_task.active); return 1;
        }
    }
    for (int mode = 0; mode < 3; ++mode) {
        WorldModel wm = start(2);
        double bx = 75 + dx * 10, by = 150 + dy * 10;
        if (mode == 0) { wm.home[1].x += dx * 10; wm.home[1].y += dy * 10; }
        if (mode == 1) { bx = 75 - dx * 10; by = 150 - dy * 10; }
        if (mode == 2) { bx = 75 - dy * 10; by = 150 + dx * 10; }
        frame(wm, bx, by);
        if (wm.coop_pass_task.phase != Phase::Preparing) { printf("FAIL: coop false release mode=%d\n", mode); return 1; }
    }
    for (int receiver : {2, 3}) {
        WorldModel wm = start(receiver);
        if (!wm.coop_pass_task.active || wm.coop_pass_task.phase != Phase::Preparing) {
            printf("FAIL: coop lifecycle did not start preparing\n"); return 1;
        }
        wm.we_have_ball = false; wm.whos_ball = 0;
        wm.no_possession_frames = 4; wm.threat_level = 0.4;
        frame(wm, 75 + dx * 3, 150 + dy * 3);
        if (wm.coop_pass_task.phase != Phase::Preparing) { printf("FAIL: coop release before separation\n"); return 1; }
        frame(wm, 75 + dx * 7, 150 + dy * 7);
        wm.opp[2].x = (75 + dx * 11 + 55) / 2; wm.opp[2].y = (150 + dy * 11 + 90) / 2;
        frame(wm, 75 + dx * 11, 150 + dy * 11);
        if (!wm.coop_pass_task.active || wm.coop_pass_task.phase != Phase::Receiving || !stopped(wm.home[1])) {
            printf("FAIL: coop observed release not latched/passer repeats push\n"); return 1;
        }
        wm.opp[2].x = (wm.ball.x + 55) / 2; wm.opp[2].y = (wm.ball.y + 90) / 2;
        CircleObstacle blocker{wm.opp[2].x, wm.opp[2].y, 8};
        if (segment_clear_of_circles(wm.ball.x, wm.ball.y, 55, 90, &blocker, 1)) {
            printf("FAIL: coop flight blocked-line fixture\n"); return 1;
        }
        int n = 0;
        for (double threat : {0.3, 0.4, 0.59}) {
            wm.threat_level = threat; ++wm.no_possession_frames;
            wm.assist_x = wm.mid_x = 130; wm.assist_y = wm.mid_y = 150;
            RobotState expected = wm.home[receiver]; motion::position(expected, 55, 90);
            ++n; frame(wm, 75 + dx * (11 + n), 150 + dy * (11 + n));
            if (!wm.coop_pass_task.active || wm.coop_pass_task.phase != Phase::Receiving || !stopped(wm.home[1]) ||
                fabs(wm.home[receiver].vl - expected.vl) > 1e-8 || fabs(wm.home[receiver].vr - expected.vr) > 1e-8) {
                printf("FAIL: coop flight interrupted at threat=%.2f receiver=%d\n", threat, receiver); return 1;
            }
        }
        WorldModel flight = wm;
        if (wm.coop_stats.created != 1 || wm.coop_stats.released != 1) {
            printf("FAIL: coop release statistics duplicated/missing\n"); return 1;
        }
        {
            WorldModel expired = flight;
            expired.coop_pass_task.frames_left = 0;
            cancel_unsafe_coop_pass(expired); cancel_unsafe_coop_pass(expired);
            if (expired.coop_stats.outcomes[(int)CoopOutcome::ReceiveTimeout] != 1) {
                printf("FAIL: coop receive timeout statistics\n"); return 1;
            }
        }
        wm.opp[2].x = 200; wm.opp[2].y = 20;
        wm.home[receiver].x = 55; wm.home[receiver].y = 90;
        frame(wm, 65, 120);
        if (!wm.coop_pass_task.active || wm.coop_ball_control.active) { printf("FAIL: coop target arrival mistaken for reception\n"); return 1; }
        wm.home[receiver].x = 55; wm.home[receiver].y = 85; wm.home[receiver].rot = 90;
        wm.we_have_ball = true;
        frame(wm, 55, 90);
        if (wm.coop_ball_control.active) { printf("FAIL: coop fast fly-by mistaken for possession\n"); return 1; }
        WorldModel contested = wm; contested.we_have_ball = false; contested.whos_ball = 0;
        contested.opp[2].x = 55; contested.opp[2].y = 96;
        for (int i = 0; i < 3; ++i) frame(contested, 55, 90);
        if (!contested.coop_pass_task.active || contested.coop_ball_control.active) {
            printf("FAIL: coop close ball without possession mistaken for reception\n"); return 1;
        }
        frame(wm, 55, 90);
        if (!wm.coop_pass_task.active || wm.coop_ball_control.active) { printf("FAIL: coop reception lacks consecutive evidence\n"); return 1; }
        frame(wm, 55, 90);
        if (wm.coop_pass_task.active || wm.coop_pass_task.phase != Phase::Received || !wm.coop_ball_control.active ||
            wm.coop_ball_control.receiver_id != receiver || stopped(wm.home[receiver]) || !stopped(wm.home[1])) {
            printf("FAIL: coop reception did not hand ball to actual receiver\n"); return 1;
        }
        if (wm.coop_stats.received != 1 || wm.coop_stats.control_entered != 1 ||
            wm.coop_stats.outcomes[(int)CoopOutcome::Success] != 1) {
            printf("FAIL: coop success/control statistics\n"); return 1;
        }
        {
            WorldModel ended = wm;
            ended.coop_control_end(CoopOutcome::LooseBall);
            ended.coop_control_end(CoopOutcome::LooseBall);
            ended.coop_finish(CoopOutcome::Intercepted);
            unsigned long sum = 0;
            for (auto n : ended.coop_stats.outcomes) sum += n;
            if (sum != ended.coop_stats.created || sum != 1 ||
                ended.coop_stats.control_exits[(int)CoopOutcome::LooseBall] != 1) {
                printf("FAIL: coop terminal result counted twice after control loss\n"); return 1;
            }
        }
        RobotState expected = wm.home[receiver]; motion::position(expected, 55, 110, motion::TM_PASS);
        frame(wm, 55, 90);
        if (!wm.coop_ball_control.active || wm.coop_pass_task.active || !stopped(wm.home[1]) ||
            fabs(wm.home[receiver].vl - expected.vl) > 1e-8 || fabs(wm.home[receiver].vr - expected.vr) > 1e-8) {
            printf("FAIL: coop receiver abandoned controlled ball\n"); return 1;
        }
        WorldModel scheduled = wm; Strategy strategy; strategy.run(scheduled);
        if (!scheduled.coop_ball_control.active || !stopped(scheduled.home[1]) ||
            fabs(scheduled.home[receiver].vl - expected.vl) > 1e-8 || fabs(scheduled.home[receiver].vr - expected.vr) > 1e-8) {
            printf("FAIL: coop ball ownership lost through fixed role scheduling\n"); return 1;
        }
        for (int danger = 0; danger < 4; ++danger) {
            WorldModel interrupted = wm;
            if (danger == 0) interrupted.threat_level = 0.6;
            if (danger == 1) interrupted.in_penalty_exec = true;
            if (danger == 2) interrupted.ga_cooldown[receiver] = 5;
            if (danger == 3) { interrupted.opp[2].x = 55; interrupted.opp[2].y = 90; interrupted.home[receiver].x = 85; interrupted.whos_ball = 2; }
            frame(interrupted, 55, 90);
            if (interrupted.coop_ball_control.active) { printf("FAIL: coop receiver ignores safety=%d\n", danger); return 1; }
        }
        WorldModel reset = wm; Environment env; init_env(env, 55, 90); env.gameState = PM_PlaceKick_Blue;
        reset.update(&env, TeamContext{true});
        if (reset.coop_ball_control.active) { printf("FAIL: coop ball ownership survives restart\n"); return 1; }
        WorldModel takeover = wm; takeover.home[1].x = 55; takeover.home[1].y = 90;
        takeover.home[receiver].x = 85;
        frame(takeover, 55, 90);
        if (takeover.coop_ball_control.active) { printf("FAIL: coop owner blocks teammate takeover\n"); return 1; }
        wm.home[receiver].x = 100; wm.home[receiver].y = 130; wm.we_have_ball = false;
        for (int i = 0; i < 3; ++i) frame(wm, 55, 90);
        if (wm.coop_ball_control.active) { printf("FAIL: coop owner persists after losing ball\n"); return 1; }
        for (int cause = 0; cause < 3; ++cause) {
            WorldModel cancelled = flight;
            if (cause == 0) { cancelled.opp[2].x = cancelled.ball.x; cancelled.opp[2].y = cancelled.ball.y; cancelled.whos_ball = 2; }
            if (cause == 1) cancelled.threat_level = 0.6;
            if (cause == 2) cancelled.coop_pass_task.frames_left = 1;
            frame(cancelled, cancelled.ball.x, cancelled.ball.y);
            if (cancelled.coop_pass_task.active || cancelled.coop_ball_control.active) {
                printf("FAIL: coop flight not cancelled cause=%d\n", cause); return 1;
            }
            const CoopOutcome expected[] = {CoopOutcome::Intercepted, CoopOutcome::HighThreat, CoopOutcome::ReceiveTimeout};
            cancel_unsafe_coop_pass(cancelled);
            if (cancelled.coop_stats.outcomes[(int)expected[cause]] != 1) {
                printf("FAIL: coop flight accounting cause=%d\n", cause); return 1;
            }
        }
    }
    {
        WorldModel wm = start(2);
        frame(wm, 75 + dx * 11, 150 + dy * 11);
        if (wm.coop_pass_task.phase != Phase::Receiving) { printf("FAIL: coop goalie-area fixture\n"); return 1; }
        wm.home[1].x = 40; wm.home[1].y = 90;
        wm.ball.x = 45; wm.ball.y = 90; wm.ball.vx = 0; wm.ball.vy = 1;
        wm.active_ga_total = (int)floor(get_param("roles.kActiveGaTotal", 18)) - 1;
        wm.shoot_push_cd = 5;
        const int before = wm.active_ga_total;
        Strategy strategy; strategy.run(wm);
        if (!wm.coop_pass_task.active || wm.active_ga_total != before + 1 || wm.shoot_push_cd != 4) {
            printf("FAIL: coop flight freezes goalie-area/shoot cooldown counters\n"); return 1;
        }
        wm.ball.y += 1; strategy.run(wm);
        if (wm.coop_pass_task.active || wm.ga_retreat_fires == 0) {
            printf("FAIL: coop flight bypasses goalie-area retreat\n"); return 1;
        }
    }
    printf("coop lifecycle: OK (observed release/no release/interception/reception/receiver control/moderate threat/goal-area discipline)\n");
    return 0;
}

// 普通 PassPlan 也必须锁定同一接球点，并复用出球/接稳/临时控球生命周期
static int test_ordinary_pass_task() {
    WorldModel wm = coop_task_scene();
    wm.role[1] = ROLE_ACTIVE; wm.role[2] = ROLE_ASSIST;
    wm.assist_x = 55; wm.assist_y = 90;
    PassPlan pp = plan_pass(wm, 1);
    if (!pp.viable || pp.receiver_id < 2) { printf("FAIL: ordinary PassPlan fixture\n"); return 1; }
    wm.assist_x = 130; wm.assist_y = 150;
    auto &task = wm.coop_pass_task;
    task = {};
    task.active = true; task.passer_id = 1; task.receiver_id = pp.receiver_id;
    task.rx = pp.target_x; task.ry = pp.target_y; task.frames_left = 20;
    task.game_state = wm.game_state; task.kind = PassTaskKind::Ordinary;
    task.observing_push = true; task.push_ball_x = wm.ball.x; task.push_ball_y = wm.ball.y;
    const double len = dist(wm.ball.x, wm.ball.y, task.rx, task.ry);
    task.push_dir_x = (task.rx - wm.ball.x) / len; task.push_dir_y = (task.ry - wm.ball.y) / len;
    set_param("roles.kRecvMeetBall", 0.0);
    set_param("roles.kRecvNoReverse", 0.0);
    run_assist(wm, task.receiver_id);
    RobotState expected = wm.home[task.receiver_id]; motion::position(expected, task.rx, task.ry);
    const bool ordinary_ok = fabs(wm.home[task.receiver_id].vl - expected.vl) <= 1e-8 &&
                             fabs(wm.home[task.receiver_id].vr - expected.vr) <= 1e-8;
    reset_params();
    if (!ordinary_ok) {
        printf("FAIL: ordinary receiver target overwritten\n"); return 1;
    }
    wm.ball_last = wm.ball;
    wm.home[1].x = wm.ball.x - task.push_dir_x * 8.0;
    wm.home[1].y = wm.ball.y - task.push_dir_y * 8.0;
    wm.ball.x += task.push_dir_x * 8.0; wm.ball.y += task.push_dir_y * 8.0;
    wm.ball.vx = task.push_dir_x * 6.0; wm.ball.vy = task.push_dir_y * 6.0;
    run_active(wm, 1);
    if (!wm.coop_pass_task.active || wm.coop_pass_task.phase != CoopPassPhase::Receiving || wm.shoot_push_count != 0) {
        printf("FAIL: ordinary release/count active=%d phase=%d count=%d\n", (int)wm.coop_pass_task.active,
               (int)wm.coop_pass_task.phase, wm.shoot_push_count); return 1;
    }
    wm.ball.x = task.rx; wm.ball.y = task.ry; wm.ball.vx = wm.ball.vy = 0.0; wm.we_have_ball = true;
    wm.home[task.receiver_id].x = task.rx; wm.home[task.receiver_id].y = task.ry;
    run_active(wm, 1); run_active(wm, 1);
    if (wm.coop_pass_task.active || !wm.coop_ball_control.active ||
        wm.coop_ball_control.receiver_id != pp.receiver_id) {
        printf("FAIL: ordinary reception/control\n"); return 1;
    }
    printf("ordinary pass task: OK (locked target/release/no shot count/reception/control)\n");
    return 0;
}

// 普通 PassPlan 与 CoopPass 共用同一 readiness gate：未到位只等，不改锁点
static int test_pass_readiness_gate() {
    auto stopped = [](const RobotState &r) { return r.vl == 0.0 && r.vr == 0.0; };

    {
        WorldModel wm = coop_task_scene();
        auto &task = wm.coop_pass_task;
        task.active = true; task.receiver_id = 2; task.rx = 100; task.ry = 90;
        wm.ball.x = 40; wm.ball.y = 90;
        const double receiver_speed = get_param("pass.RECEIVER_READY_SPEED", 2.0);
        const double ball_speed = get_param("pass.PASS_BALL_SPEED", 6.0);
        const double tolerance = get_param("pass.RECEIVER_READY_TOLERANCE", 5.0);
        const double limit = receiver_speed * (60.0 / ball_speed + tolerance);
        wm.home[2].x = task.rx + limit - 1e-6; wm.home[2].y = task.ry;
        if (!pass_receiver_ready(wm)) { printf("FAIL: pass readiness inside boundary rejected\n"); return 1; }
        wm.home[2].x += 0.1;
        if (pass_receiver_ready(wm)) { printf("FAIL: pass readiness over boundary accepted\n"); return 1; }
    }

    for (PassTaskKind kind : {PassTaskKind::Ordinary, PassTaskKind::Coop}) {
        WorldModel wm = coop_task_scene();
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            wm.opp[i].x = 210; wm.opp[i].y = 10 + 40 * i;
        }
        auto &task = wm.coop_pass_task;
        task = {};
        task.active = true; task.passer_id = 1; task.receiver_id = 2;
        task.rx = 55; task.ry = 90; task.frames_left = 20;
        task.game_state = wm.game_state; task.kind = kind;
        const int receiver = task.receiver_id;
        const double locked_x = task.rx, locked_y = task.ry;

        if (pass_receiver_ready(wm)) {
            printf("FAIL: pass readiness far receiver accepted kind=%d\n", (int)kind); return 1;
        }
        set_param("roles.kRecvMeetBall", 0.0);
        set_param("roles.kRecvNoReverse", 0.0);
        RobotState expected_receiver = wm.home[receiver];
        motion::position(expected_receiver, locked_x, locked_y);
        run_active(wm, 1);
        run_assist(wm, receiver);
        const bool wait_ok = task.active && task.receiver_id == receiver &&
                             task.rx == locked_x && task.ry == locked_y &&
                             !task.observing_push && stopped(wm.home[1]) &&
                             fabs(wm.home[receiver].vl - expected_receiver.vl) <= 1e-8 &&
                             fabs(wm.home[receiver].vr - expected_receiver.vr) <= 1e-8;
        reset_params();
        if (!wait_ok) {
            printf("FAIL: pass readiness WAIT mutated/pushed kind=%d active=%d observing=%d\n",
                   (int)kind, (int)task.active, (int)task.observing_push); return 1;
        }

        wm.home[receiver].x = locked_x; wm.home[receiver].y = locked_y;
        if (!pass_receiver_ready(wm)) {
            printf("FAIL: pass readiness arrived receiver rejected kind=%d\n", (int)kind); return 1;
        }
        run_active(wm, 1);
        if (!task.active || !task.observing_push) {
            printf("FAIL: pass readiness READY did not release kind=%d\n", (int)kind); return 1;
        }
    }

    WorldModel danger = coop_task_scene();
    auto &task = danger.coop_pass_task;
    task.active = true; task.passer_id = 1; task.receiver_id = 2;
    task.rx = 55; task.ry = 90; task.frames_left = 20;
    task.game_state = danger.game_state; task.kind = PassTaskKind::Coop;
    danger.threat_level = 0.6;
    run_active(danger, 1);
    if (task.active || danger.coop_stats.outcomes[(int)CoopOutcome::HighThreat] != 1) {
        printf("FAIL: pass readiness WAIT outranked safety cancellation\n"); return 1;
    }
    printf("pass readiness gate: OK (shared WAIT/READY/locked task/safety priority)\n");
    return 0;
}

// 接球人奔「会合点」迎球；目标在正后方则原地转身不倒车
static int test_receiver_meet_ball() {
    auto scene = [](CoopPassPhase phase, double ball_x, double ball_vx,
                    double receiver_x, double receiver_rot) {
        WorldModel wm = coop_task_scene();
        auto &task = wm.coop_pass_task;
        task = {};
        task.active = true; task.passer_id = 1; task.receiver_id = 2;
        task.rx = 55; task.ry = 90; task.frames_left = 20;
        task.game_state = wm.game_state; task.kind = PassTaskKind::Coop;
        task.phase = phase;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 15 + 35 * i; }
        wm.ball.x = ball_x; wm.ball.y = 90; wm.ball.vx = ball_vx; wm.ball.vy = 0.0;
        wm.we_have_ball = true; wm.whos_ball = 1; wm.threat_level = 0.1;
        wm.home[2].x = receiver_x; wm.home[2].y = 90; wm.home[2].rot = receiver_rot;
        return wm;
    };

    {
        WorldModel wm = scene(CoopPassPhase::Receiving, 120.0, -5.0, 60.0, 0.0);
        double mx = 0, my = 0, aim = 0;
        if (!ball_meeting_point(wm, 60.0, 90.0, get_param("roles.kRecvSpeed", 2.0),
                                get_param("roles.kRecvLead", 6.0), mx, my, aim)) {
            printf("FAIL: 会合点场景本应算得出会合点\n"); return 1;
        }
        RobotState expect = wm.home[2];   // 期望 = 去会合点 + 到位迎球
        motion::arrive_facing(expect, mx, my, aim, get_param("roles.kRecvArriveDist", 8.0),
                              get_param("roles.kPassReceiveAngleTol", 12.0), false);
        RobotState old_way = wm.home[2];  // 旧行为 = 直奔锁点
        motion::position(old_way, 55.0, 90.0);
        run_assist(wm, 2);   // 接球人入口（run_assist 首行即 run_pass_receiver）
        if (!wm.coop_pass_task.active || wm.coop_pass_task.receiver_id != 2) {
            printf("FAIL: 会合点场景任务被取消 active=%d\n", (int)wm.coop_pass_task.active); return 1;
        }
        if (fabs(wm.home[2].vl - expect.vl) > 1e-8 || fabs(wm.home[2].vr - expect.vr) > 1e-8) {
            printf("FAIL: 接球人没走会合点 vl=%.1f/%.1f 期望 %.1f/%.1f（会合点 %.1f,%.1f）\n",
                   wm.home[2].vl, wm.home[2].vr, expect.vl, expect.vr, mx, my); return 1;
        }
        if (fabs(wm.home[2].vl - old_way.vl) <= 1e-8 && fabs(wm.home[2].vr - old_way.vr) <= 1e-8) {
            printf("FAIL: 新行为没生效（仍等于直奔锁点）\n"); return 1;
        }
    }

    {
        WorldModel wm = scene(CoopPassPhase::Preparing, 100.0, 0.0, 70.0, 0.0);
        run_assist(wm, 2);   // 接球人入口（run_assist 首行即 run_pass_receiver）
        const double common = fabs(wm.home[2].vl + wm.home[2].vr);
        if (!(wm.home[2].vl < 0.0 && wm.home[2].vr > 0.0) || common > 1e-9) {
            printf("FAIL: 目标在身后应原地转身 vl=%.1f vr=%.1f\n", wm.home[2].vl, wm.home[2].vr); return 1;
        }
    }

    {
        WorldModel wm = scene(CoopPassPhase::Preparing, 100.0, 0.0, 30.0, 0.0);
        RobotState baseline = wm.home[2];
        motion::position(baseline, 55.0, 90.0);
        run_assist(wm, 2);   // 接球人入口（run_assist 首行即 run_pass_receiver）
        if (fabs(wm.home[2].vl - baseline.vl) > 1e-8 || fabs(wm.home[2].vr - baseline.vr) > 1e-8) {
            printf("FAIL: 停球时应按锁点走位 vl=%.1f/%.1f 应 %.1f/%.1f\n",
                   wm.home[2].vl, wm.home[2].vr, baseline.vl, baseline.vr); return 1;
        }
    }

    {
        WorldModel wm = scene(CoopPassPhase::Preparing, 60.0, -2.0, 40.0, 0.0);
        wm.coop_pass_task.rx = 60.0; wm.coop_pass_task.ry = 90.0;
        double mx = 0, my = 0, aim = 0;
        if (!ball_meeting_point(wm, 40.0, 90.0, get_param("roles.kRecvSpeed", 2.0),
                                get_param("roles.kRecvLead", 6.0), mx, my, aim)) {
            printf("FAIL: 门区场景本应算得出会合点（否则本用例测不到红线）\n"); return 1;
        }
        if (!in_opp_goal_area(wm.ctx, mx, my)) {
            printf("FAIL: 门区场景的会合点应落在对方门区内 (%.1f,%.1f)\n", mx, my); return 1;
        }
        RobotState baseline = wm.home[2];
        motion::position(baseline, 60.0, 90.0);
        run_assist(wm, 2);   // 接球人入口（run_assist 首行即 run_pass_receiver）
        if (fabs(wm.home[2].vl - baseline.vl) > 1e-8 || fabs(wm.home[2].vr - baseline.vr) > 1e-8) {
            printf("FAIL: 会合点落对方门区应回退锁点 vl=%.1f/%.1f 应 %.1f/%.1f\n",
                   wm.home[2].vl, wm.home[2].vr, baseline.vl, baseline.vr); return 1;
        }
    }

    printf("receiver meet ball: OK (会合点迎球、身后原地转身不倒车、停球回退锁点)\n");
    return 0;
}

// 对手会明显更早占住锁点则取消传球；球出脚后不再套用
static int test_pass_opponent_first_cancel() {
    auto scene = [](PassTaskKind kind) {
        WorldModel wm;
        wm.ctx = TeamContext{true};
        wm.game_state = wm.game_state_last = PM_PlayOn;
        wm.ball.valid = true; wm.ball.x = 60; wm.ball.y = 90;
        wm.we_have_ball = true; wm.threat_level = 0.1;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            wm.home[i].x = 180; wm.home[i].y = 30 + 25 * i;
            wm.opp[i].x = 200; wm.opp[i].y = 15 + 35 * i;
            wm.opp_vx[i] = wm.opp_vy[i] = 0.0;
        }
        wm.home[1].x = 55; wm.home[1].y = 90;
        auto &task = wm.coop_pass_task;
        task.active = true; task.passer_id = 1; task.receiver_id = 2;
        task.rx = 120; task.ry = 90; task.frames_left = 40;
        task.game_state = wm.game_state; task.kind = kind;
        wm.opp[0].x = 120; wm.opp[0].y = 120; // 离锁点 30cm，且不挡球到锁点的线路。
        wm.opp_vel_ready = true;
        return wm;
    };

    for (PassTaskKind kind : {PassTaskKind::Ordinary, PassTaskKind::Coop}) {
        WorldModel wm = scene(kind);
        wm.home[2].x = 120; wm.home[2].y = 170; // 接球人还需 80cm，对手明显先到。
        cancel_unsafe_coop_pass(wm);
        if (wm.coop_pass_task.active || wm.coop_stats.outcomes[(int)CoopOutcome::OpponentFirst] != 1) {
            printf("FAIL: obvious opponent-first pass kept kind=%d\n", (int)kind); return 1;
        }
    }

    {
        WorldModel wm = scene(PassTaskKind::Coop);
        wm.home[2].x = 130; wm.home[2].y = 90; // 接球人只差 10cm。
        cancel_unsafe_coop_pass(wm);
        if (!wm.coop_pass_task.active) { printf("FAIL: receiver-first pass cancelled\n"); return 1; }
    }
    {
        WorldModel wm = scene(PassTaskKind::Coop);
        wm.home[2].x = 120; wm.home[2].y = 128; // 两者预计时间接近，安全余量应保留任务。
        cancel_unsafe_coop_pass(wm);
        if (!wm.coop_pass_task.active) { printf("FAIL: near-tie pass cancelled\n"); return 1; }
    }
    {
        WorldModel wm = scene(PassTaskKind::Coop);
        wm.home[2].x = 120; wm.home[2].y = 140;
        wm.opp_vy[0] = 5.0; // 目标在下方，对手高速向上远离。
        cancel_unsafe_coop_pass(wm);
        if (!wm.coop_pass_task.active) { printf("FAIL: fast-away opponent caused cancel\n"); return 1; }
    }
    {
        WorldModel wm = scene(PassTaskKind::Coop);
        wm.home[2].x = 120; wm.home[2].y = 140;
        wm.opp[0].y = 130; wm.opp_vy[0] = 5.0;
        for (int frame = 0; frame < 3; ++frame) {
            run_active(wm, 1); // 真实走三帧 readiness WAIT；远离期间应保持同一任务。
            if (!wm.coop_pass_task.active) {
                printf("FAIL: waiting pass cancelled before opponent turned frame=%d\n", frame); return 1;
            }
        }
        wm.opp_vy[0] = -5.0; // 第四帧转向锁点，变成明显先到。
        run_active(wm, 1);
        if (wm.coop_pass_task.active || wm.coop_stats.outcomes[(int)CoopOutcome::OpponentFirst] != 1) {
            printf("FAIL: waiting pass ignored approaching opponent\n"); return 1;
        }
    }
    {
        WorldModel wm = scene(PassTaskKind::Coop);
        wm.home[2].x = 120; wm.home[2].y = 170;
        wm.coop_pass_task.phase = CoopPassPhase::Receiving;
        cancel_unsafe_coop_pass(wm);
        if (!wm.coop_pass_task.active) { printf("FAIL: Receiving pass cancelled by opponent-first rule\n"); return 1; }
    }
    {
        WorldModel wm = scene(PassTaskKind::Coop);
        wm.home[2].x = std::numeric_limits<double>::infinity();
        if (pass_opponent_arrives_first(wm)) { printf("FAIL: non-finite receiver triggered opponent-first\n"); return 1; }
    }
    {
        WorldModel wm = scene(PassTaskKind::Coop);
        wm.home[2].x = 120; wm.home[2].y = 128;
        wm.opp_vy[0] = -std::numeric_limits<double>::infinity();
        if (pass_opponent_arrives_first(wm)) { printf("FAIL: non-finite opponent velocity triggered opponent-first\n"); return 1; }
    }
    printf("pass opponent-first: OK (shared/cushion/direction/wait transition/Receiving guard)\n");
    return 0;
}

// 出球后接球人近球减速并面向来球；Preparing 与远距 Receiving 保持赶路
static int test_pass_receive_control() {
    auto scene = [](PassTaskKind kind, CoopPassPhase phase) {
        WorldModel wm;
        wm.ctx = TeamContext{true};
        wm.game_state = wm.game_state_last = PM_PlayOn;
        wm.ball.valid = true; wm.ball.x = 80; wm.ball.y = 90;
        wm.we_have_ball = true; wm.threat_level = 0.1;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            wm.home[i].x = 140; wm.home[i].y = 20 + 30 * i;
            wm.opp[i].x = 210; wm.opp[i].y = 15 + 35 * i;
        }
        wm.home[1].x = 70; wm.home[1].y = 90;
        wm.home[2].x = 60; wm.home[2].y = 90; wm.home[2].rot = 0;
        auto &task = wm.coop_pass_task;
        task.active = true; task.passer_id = 1; task.receiver_id = 2;
        task.rx = 100; task.ry = 90; task.frames_left = 40;
        task.game_state = wm.game_state; task.kind = kind; task.phase = phase;
        task.push_dir_x = 1.0; task.push_dir_y = 0.0;
        return wm;
    };
    auto same_wheels = [](const RobotState &a, const RobotState &b) {
        return fabs(a.vl - b.vl) < 1e-8 && fabs(a.vr - b.vr) < 1e-8;
    };
    auto finite_wheels = [](const RobotState &r) { return std::isfinite(r.vl) && std::isfinite(r.vr); };

    {
        WorldModel wm = scene(PassTaskKind::Coop, CoopPassPhase::Preparing);
        RobotState expected = wm.home[2]; motion::position(expected, 100, 90);
        run_assist(wm, 2);
        if (!wm.coop_pass_task.active || !same_wheels(wm.home[2], expected)) {
            printf("FAIL: receive control changed Preparing movement\n"); return 1;
        }
    }
    {
        WorldModel wm = scene(PassTaskKind::Coop, CoopPassPhase::Receiving);
        wm.ball.x = 160; wm.ball.vx = -4;
        RobotState expected = wm.home[2]; motion::position(expected, 100, 90);
        run_assist(wm, 2);
        if (!wm.coop_pass_task.active || !same_wheels(wm.home[2], expected)) {
            printf("FAIL: receive control slowed distant ball approach\n"); return 1;
        }
    }
    {
        WorldModel wm = scene(PassTaskKind::Coop, CoopPassPhase::Receiving);
        wm.home[2].x = 90; wm.home[2].rot = 0;
        wm.ball.x = 110; wm.ball.y = 90; wm.ball.vx = -4; wm.ball.vy = 0;
        RobotState full = wm.home[2]; motion::position(full, 100, 90);
        run_assist(wm, 2);
        double old_drive = fabs((full.vl + full.vr) * 0.5);
        double new_drive = fabs((wm.home[2].vl + wm.home[2].vr) * 0.5);
        if (!(new_drive < old_drive * 0.7) || fabs(wm.home[2].vr - wm.home[2].vl) > 1e-8) {
            printf("FAIL: frontal receive did not slow cleanly old=%.2f new=%.2f vl=%.2f vr=%.2f\n",
                   old_drive, new_drive, wm.home[2].vl, wm.home[2].vr); return 1;
        }
        WorldModel closer = scene(PassTaskKind::Coop, CoopPassPhase::Receiving);
        closer.home[2].x = 90; closer.home[2].rot = 0;
        closer.ball.x = 100; closer.ball.y = 90; closer.ball.vx = -4; closer.ball.vy = 0;
        run_assist(closer, 2);
        double closer_drive = fabs((closer.home[2].vl + closer.home[2].vr) * 0.5);
        if (!(closer_drive < new_drive)) {
            printf("FAIL: receive slowdown is not progressive near=%.2f closer=%.2f\n",
                   new_drive, closer_drive); return 1;
        }
    }
    {
        WorldModel wm = scene(PassTaskKind::Coop, CoopPassPhase::Receiving);
        wm.home[2].x = 100; wm.home[2].rot = 0;
        wm.ball.x = 100; wm.ball.y = 110; wm.ball.vx = 0; wm.ball.vy = -4;
        run_assist(wm, 2);
        if (!(wm.home[2].vl < 0 && wm.home[2].vr > 0)) {
            printf("FAIL: side receive did not turn toward incoming ball vl=%.2f vr=%.2f\n",
                   wm.home[2].vl, wm.home[2].vr); return 1;
        }
        WorldModel off_target = scene(PassTaskKind::Coop, CoopPassPhase::Receiving);
        off_target.home[2].x = 80; off_target.home[2].rot = 0; // 离锁点 20cm，超过 motion 内部近距线。
        off_target.ball.x = 80; off_target.ball.y = 110;
        off_target.ball.vx = 0; off_target.ball.vy = -4;
        run_assist(off_target, 2);
        if (!(off_target.home[2].vl < 0 && off_target.home[2].vr > 0)) {
            printf("FAIL: off-target side receive kept chasing lock point vl=%.2f vr=%.2f\n",
                   off_target.home[2].vl, off_target.home[2].vr); return 1;
        }
    }
    for (int abnormal = 0; abnormal < 2; ++abnormal) {
        WorldModel wm = scene(PassTaskKind::Coop, CoopPassPhase::Receiving);
        wm.home[2].x = 100; wm.home[2].rot = 180;
        wm.ball.x = 80; wm.ball.y = 90;
        if (abnormal == 0) { wm.ball.vx = -1e-9; wm.ball.vy = 0; }
        else { wm.ball.vx = std::numeric_limits<double>::quiet_NaN(); wm.ball.vy = std::numeric_limits<double>::infinity(); }
        run_assist(wm, 2);
        if (!wm.coop_pass_task.active || !finite_wheels(wm.home[2]) ||
            fabs(wm.home[2].vl) > 1e-8 || fabs(wm.home[2].vr) > 1e-8) {
            printf("FAIL: receive direction fallback abnormal=%d vl=%.2f vr=%.2f active=%d\n",
                   abnormal, wm.home[2].vl, wm.home[2].vr, (int)wm.coop_pass_task.active); return 1;
        }
    }
    for (int fallback = 0; fallback < 2; ++fallback) {
        WorldModel wm = scene(PassTaskKind::Coop, CoopPassPhase::Receiving);
        wm.home[2].x = 100; wm.home[2].rot = 180;
        wm.ball.x = 80; wm.ball.y = 90;
        wm.ball.vx = fallback == 0 ? -100.0 : -4.0; // 异常高速或可信但正在远离。
        wm.ball.vy = 0;
        run_assist(wm, 2);
        if (!wm.coop_pass_task.active || !finite_wheels(wm.home[2]) ||
            fabs(wm.home[2].vl) > 1e-8 || fabs(wm.home[2].vr) > 1e-8) {
            printf("FAIL: receive high/away fallback=%d vl=%.2f vr=%.2f\n",
                   fallback, wm.home[2].vl, wm.home[2].vr); return 1;
        }
    }
    {
        WorldModel ordinary = scene(PassTaskKind::Ordinary, CoopPassPhase::Receiving);
        ordinary.home[2].x = 100; ordinary.ball.x = 100; ordinary.ball.y = 110;
        ordinary.ball.vx = 0; ordinary.ball.vy = -4;
        WorldModel coop = ordinary; coop.coop_pass_task.kind = PassTaskKind::Coop;
        run_assist(ordinary, 2); run_assist(coop, 2);
        if (!same_wheels(ordinary.home[2], coop.home[2])) {
            printf("FAIL: Ordinary/Coop receive control diverged\n"); return 1;
        }
    }
    {
        WorldModel received = scene(PassTaskKind::Coop, CoopPassPhase::Receiving);
        received.home[2].x = 100; received.ball.x = 105; received.ball.y = 90;
        received.ball.vx = received.ball.vy = 0; received.coop_pass_task.receive_frames = 1;
        run_active(received, 1);
        if (received.coop_pass_task.active || received.coop_pass_task.phase != CoopPassPhase::Received ||
            !received.coop_ball_control.active || received.coop_ball_control.receiver_id != 2) {
            printf("FAIL: receive control broke Received/control handoff\n"); return 1;
        }
        WorldModel expired = scene(PassTaskKind::Coop, CoopPassPhase::Receiving);
        expired.coop_pass_task.frames_left = 1; run_active(expired, 1);
        if (expired.coop_pass_task.active ||
            expired.coop_stats.outcomes[(int)CoopOutcome::ReceiveTimeout] != 1) {
            printf("FAIL: receive control broke Receiving timeout\n"); return 1;
        }
    }
    printf("pass receive control: OK (Preparing/far/slow/facing/fallback/shared/lifecycle)\n");
    return 0;
}

static int test_no_reverse_through_ball() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队攻 x=0（对方门在左）
    wm.ball.valid = true;
    wm.ball.x = 39.4; wm.ball.y = 89.8; wm.ball.vx = 0; wm.ball.vy = 0;
    wm.in_penalty_exec = true;
    wm.we_have_ball = true;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 200; wm.home[i].y = 20 + i * 30; wm.home[i].rot = 180.0;
        wm.opp[i].x = 5;    wm.opp[i].y = 85 + i;
        wm.role[i] = ROLE_PASSIVE;
    }
    wm.role[1] = ROLE_ACTIVE;
    wm.home[1].x = 33.0; wm.home[1].y = 88.7; wm.home[1].rot = -165.0;
    run_active(wm, 1);
    const double v = 0.5 * (wm.home[1].vl + wm.home[1].vr);
    if (v > 2.0) {
        printf("FAIL: 在球的球门侧时应绕行/转向，不该朝自家门驱动 vl=%.1f vr=%.1f\n",
               wm.home[1].vl, wm.home[1].vr);
        return 1;
    }
    printf("no reverse-through-ball: OK (球门侧只绕行/转向，不朝自家门推)\n");
    return 0;
}

// 对方出脚方向预测：只采信「球静止 + 贴球 + 机头对球」的 rot
static int test_opp_kick_predict() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队：己方门线 x=220，门框 y∈[70,110]
    wm.ball.valid = true;
    wm.ball.x = 180; wm.ball.y = 90; wm.ball.vx = 0; wm.ball.vy = 0;
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 300; wm.opp[i].y = 20 + i * 30; wm.opp[i].rot = 0; }
    wm.opp[0].x = 158; wm.opp[0].y = 89;
    wm.opp[0].rot = angle_to(158, 89, 180, 90);
    double y = 0.0;
    if (!opp_kick_target_y(wm, y)) { printf("FAIL: 贴球且机头对球应给出预测落点\n"); return 1; }
    if (std::fabs(y - 91.8) > 1.5) { printf("FAIL: 预测落点应≈91.8，实际 %.1f\n", y); return 1; }
    wm.ball.vx = 2.0;
    if (opp_kick_target_y(wm, y)) { printf("FAIL: 球在动时不该用朝向预测\n"); return 1; }
    wm.ball.vx = 0.0;
    wm.opp[0].rot = normalize_angle(wm.opp[0].rot + 90.0);
    if (opp_kick_target_y(wm, y)) { printf("FAIL: 机头没对球不该预测\n"); return 1; }
    wm.opp[0].x = 140;
    if (opp_kick_target_y(wm, y)) { printf("FAIL: 对手离球太远不该预测\n"); return 1; }
    printf("opp kick predict: OK (贴球+对球才预测/球在动不用/机头偏不采信/太远不预测)\n");
    return 0;
}

// 我方门区「只能有门将」硬闸
static int test_own_goalarea_guard() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队：己方门线 x=220，门区 x∈[170,220], y∈[75,105]
    wm.ball.valid = true;
    wm.ball.x = 190; wm.ball.y = 90;            // 球就在门区里（也要照样把人顶出去）
    for (int i = 0; i < 5; ++i) { wm.home[i].vl = 0; wm.home[i].vr = 0; wm.home[i].rot = 180.0; }
    wm.home[0].x = 215; wm.home[0].y = 90;      // 门将在门区里（豁免）
    wm.home[1].x = 150; wm.home[1].y = 90;      // 主攻在外面（不该被动）
    wm.home[4].x = 180; wm.home[4].y = 90;      // 后卫挤在门区里（必须被顶出去）

    enforce_own_goal_area(wm);

    if (std::fabs(wm.home[0].vl) > 0.01 || std::fabs(wm.home[0].vr) > 0.01) {
        printf("FAIL: 门将不应被门区硬闸驱动 (vl=%.1f vr=%.1f)\n", wm.home[0].vl, wm.home[0].vr);
        return 1;
    }
    if (std::fabs(wm.home[4].vl) < 1.0 && std::fabs(wm.home[4].vr) < 1.0) {
        printf("FAIL: 门区里的后卫应被顶出去，实际没动作\n");
        return 1;
    }
    if (wm.home[4].vl + wm.home[4].vr <= 0.0) {
        printf("FAIL: 顶出方向应朝场内(−x)，实际 vl=%.1f vr=%.1f\n",
               wm.home[4].vl, wm.home[4].vr);
        return 1;
    }
    if (std::fabs(wm.home[1].vl) > 0.01 || std::fabs(wm.home[1].vr) > 0.01) {
        printf("FAIL: 门区外的主攻不应被动 (vl=%.1f)\n", wm.home[1].vl);
        return 1;
    }
    {
        WorldModel w2;
        w2.ctx = TeamContext{false};
        w2.ball.valid = true;
        w2.ball.x = 30; w2.ball.y = 90;
        for (int i = 0; i < 5; ++i) { w2.home[i].rot = 0.0; }
        w2.home[0].x = 5; w2.home[0].y = 90;
        w2.home[2].x = 40; w2.home[2].y = 90;        // 助攻挤在我方门区（黄队门区 x∈[0,50]）
        enforce_own_goal_area(w2);
        if (std::fabs(w2.home[2].vl) < 1.0 && std::fabs(w2.home[2].vr) < 1.0) {
            printf("FAIL: 黄队镜像下门区内的人也应被顶出去\n");
            return 1;
        }
        if (std::fabs(w2.home[0].vl) > 0.01) {
            printf("FAIL: 黄队门将不应被驱动\n");
            return 1;
        }
    }
    printf("own goalarea guard: OK (门将豁免/区内被顶出/区外不动/黄队镜像)\n");
    return 0;
}

// 己方大禁区人数闸（规则 7.10.4）：非门将 4 人在禁区内直接判点球，顶出离球最远者
static int test_own_penalty_count() {
    auto scene = [](TeamContext ctx, double ball_x, double ball_y) {
        WorldModel wm;
        wm.ctx = ctx;
        wm.ball.valid = true;
        wm.ball.x = ball_x; wm.ball.y = ball_y;
        wm.ball.vx = 0.0; wm.ball.vy = 0.0;
        for (int i = 0; i < 5; ++i) { wm.home[i].vl = 0.0; wm.home[i].vr = 0.0; }
        wm.home[0].x = ctx.our_goal_x() + ctx.attack_dir() * 5.0;   // 门将贴门线（在禁区里）
        wm.home[0].y = 90.0;
        return wm;
    };
    const TeamContext blue{true}, yellow{false};

    WorldModel wm = scene(blue, 150.0, 90.0);
    wm.home[1].x = 212.0; wm.home[1].y = 90.0;      // 距球 62cm（最远）
    wm.home[2].x = 160.0; wm.home[2].y = 90.0;      // 距球 10cm
    wm.home[3].x = 165.0; wm.home[3].y = 80.0;
    wm.home[4].x = 150.0; wm.home[4].y = 100.0;     // 距球 ~10cm
    enforce_own_penalty_count(wm);
    if (std::fabs(wm.home[1].vl) < 1.0 && std::fabs(wm.home[1].vr) < 1.0) {
        printf("FAIL: 大禁区 4 台非门将，离球最远的 1 号应被顶出去（实际没动作）\n");
        return 1;
    }
    {
        const double want = (wm.ctx.our_goal_x() + wm.ctx.attack_dir() * 85.0) - wm.home[1].x;
        if ((wm.home[1].vl + wm.home[1].vr) * want <= 0.0) {
            printf("FAIL: 顶出方向应朝大禁区前缘外(x=135)，实际 vl=%.1f vr=%.1f\n",
                   wm.home[1].vl, wm.home[1].vr);
            return 1;
        }
    }
    for (int i = 2; i < 5; ++i)
        if (std::fabs(wm.home[i].vl) > 0.01 || std::fabs(wm.home[i].vr) > 0.01) {
            printf("FAIL: 只该顶出离球最远的那台，robot[%d] 不该被动\n", i);
            return 1;
        }

    wm = scene(blue, 150.0, 90.0);
    wm.home[1].x = 212.0; wm.home[1].y = 90.0;
    wm.home[2].x = 160.0; wm.home[2].y = 90.0;
    wm.home[3].x = 165.0; wm.home[3].y = 80.0;
    wm.home[4].x = 120.0; wm.home[4].y = 90.0;      // 大禁区外（x<140）
    enforce_own_penalty_count(wm);
    for (int i = 1; i < 5; ++i)
        if (std::fabs(wm.home[i].vl) > 0.01 || std::fabs(wm.home[i].vr) > 0.01) {
            printf("FAIL: 只有 3 台非门将在禁区内不该动任何人，robot[%d] 被动\n", i);
            return 1;
        }

    wm = scene(blue, 150.0, 90.0);
    wm.home[0].x = 218.0; wm.home[0].y = 90.0;      // 门将贴线（也在禁区里，不该被这个闸驱动）
    wm.home[1].x = 212.0; wm.home[1].y = 90.0;
    wm.home[2].x = 160.0; wm.home[2].y = 90.0;
    wm.home[3].x = 165.0; wm.home[3].y = 80.0;
    wm.home[4].x = 150.0; wm.home[4].y = 100.0;
    enforce_own_penalty_count(wm);
    if (std::fabs(wm.home[0].vl) > 0.01 || std::fabs(wm.home[0].vr) > 0.01) {
        printf("FAIL: 门将不该被大禁区人数闸驱动 (vl=%.1f vr=%.1f)\n", wm.home[0].vl, wm.home[0].vr);
        return 1;
    }
    if (std::fabs(wm.home[1].vl) < 1.0 && std::fabs(wm.home[1].vr) < 1.0) {
        printf("FAIL: 门将在内也照样要顶出非门将里离球最远的那台\n");
        return 1;
    }

    wm = scene(yellow, 70.0, 90.0);
    wm.home[1].x = 8.0;  wm.home[1].y = 90.0;       // 距球 62cm（最远）
    wm.home[2].x = 60.0; wm.home[2].y = 90.0;
    wm.home[3].x = 55.0; wm.home[3].y = 80.0;
    wm.home[4].x = 70.0; wm.home[4].y = 100.0;
    enforce_own_penalty_count(wm);
    if (wm.home[1].vl + wm.home[1].vr <= 0.0) {
        printf("FAIL: 黄队镜像下顶出方向应朝 +x，实际 vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }
    wm = scene(yellow, 70.0, 90.0);
    wm.home[1].x = 8.0;  wm.home[1].y = 90.0;
    wm.home[2].x = 60.0; wm.home[2].y = 90.0;
    wm.home[3].x = 55.0; wm.home[3].y = 80.0;
    wm.home[4].x = 100.0; wm.home[4].y = 90.0;      // 大禁区外（x>80）
    enforce_own_penalty_count(wm);
    for (int i = 1; i < 5; ++i)
        if (std::fabs(wm.home[i].vl) > 0.01 || std::fabs(wm.home[i].vr) > 0.01) {
            printf("FAIL: 黄队 3 台不该动任何人，robot[%d] 被动\n", i);
            return 1;
        }

    printf("own penalty count: OK (4 台顶出离球最远者/3 台不动/门将豁免/黄队镜像)\n");
    return 0;
}

// 裁判口径门区：门线内 15cm × y∈[65,115]
static int test_rule_goal_area() {
    TeamContext b{true}, y{false};
    if (!in_goal_area_rule(b, 210, 112)) { printf("FAIL: (210,112) 应在裁判门区（门柱外侧）\n"); return 1; }
    if (in_goal_area_rule(b, 204, 90))   { printf("FAIL: (204,90) 离门线 16cm 不在裁判门区\n"); return 1; }
    if (in_goal_area_rule(b, 210, 116))  { printf("FAIL: (210,116) y 超 115 不在裁判门区\n"); return 1; }
    if (!in_goal_area_rule(b, 225, 108)) { printf("FAIL: (225,108) 球门里应计入\n"); return 1; }
    if (in_goal_area_rule(b, 225, 112))  { printf("FAIL: (225,112) 球门里 y 超 110 不计\n"); return 1; }
    if (!in_goal_area_rule(b, 200, 90, 8.0, 0.0)) { printf("FAIL: 余量 8cm 时 (200,90) 应命中\n"); return 1; }
    if (!in_goal_area_rule(y, 10, 68))   { printf("FAIL: 黄队镜像 (10,68) 应在裁判门区\n"); return 1; }

    WorldModel wm;
    wm.ctx = b;
    wm.ball.valid = true; wm.ball.x = 150; wm.ball.y = 90;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 120; wm.home[i].y = 90; wm.home[i].rot = 180.0; wm.home[i].vl = wm.home[i].vr = 0; }
    wm.home[3].x = 210; wm.home[3].y = 112;
    enforce_own_goal_area(wm);
    if (wm.home[3].vl + wm.home[3].vr <= 1.0) {
        printf("FAIL: 门柱外侧(裁判门区)的队员应被顶向场内，实际 vl=%.1f vr=%.1f\n",
               wm.home[3].vl, wm.home[3].vr);
        return 1;
    }
    printf("rule goal area: OK (裁判门区几何/球门内/余量/黄队镜像/门柱外侧被顶出)\n");
    return 0;
}

// 分道压迫进攻（run_swarm）：推进方向 = 射门方案 → 借墙方案 → 门心
static int test_swarm_attack() {
    auto scene = [](double bx, double by) {
        WorldModel wm;
        wm.ctx = TeamContext{true};                   // 蓝队：攻向 x=0
        wm.game_state = wm.game_state_last = PM_PlayOn;
        wm.ball.valid = true; wm.ball.x = bx; wm.ball.y = by;
        wm.threat_level = 0.1;
        const int roles[5] = {ROLE_GOALIE, ROLE_ACTIVE, ROLE_ASSIST, ROLE_MIDFIELD, ROLE_PASSIVE};
        for (int i = 0; i < 5; ++i) wm.role[i] = roles[i];
        for (int i = 0; i < 5; ++i) {
            wm.home[i].x = 170; wm.home[i].y = 20 + 35 * i; wm.home[i].rot = 180;
            wm.opp[i].x = 200; wm.opp[i].y = 20 + 30 * i;
        }
        wm.opp[0].x = 2; wm.opp[0].y = 90;            // 对方门将
        return wm;
    };
    auto herd_dir = [](const WorldModel &wm, double &ux, double &uy) {
        ShootPlan sp = plan_shoot(wm, 2);
        if (sp.viable) { ux = sp.dir_x; uy = sp.dir_y; return; }
        ShootPlan bk = plan_bank_carry(wm);
        if (bk.viable) { ux = bk.dir_x; uy = bk.dir_y; return; }
        double dx = 0.0 - wm.ball.x, dy = 90.0 - wm.ball.y, len = hypot(dx, dy);
        ux = dx / len; uy = dy / len;
    };
    auto expect = [](const WorldModel &wm, int id, double tx, double ty, const char *what) {
        RobotState e = wm.home[id];
        motion::position(e, tx, ty, motion::TM_PASS);
        if (fabs(e.vl - wm.home[id].vl) > 1e-6 || fabs(e.vr - wm.home[id].vr) > 1e-6) {
            printf("FAIL: swarm %s 目标不对 (vl=%.1f/%.1f vr=%.1f/%.1f)\n", what,
                   wm.home[id].vl, e.vl, wm.home[id].vr, e.vr);
            return false;
        }
        return true;
    };
    struct BankOffGuard {
        double carry_save, minq_save, minopen_save;
        BankOffGuard() {
            carry_save = get_param("shoot.kBankCarryMax", 160.0);
            minq_save  = get_param("shoot.kBankMinQ", 0.42);
            minopen_save = get_param("shoot.kMinOpen", 5.0);
            set_param("shoot.kBankCarryMax", 1.0);   // 借墙推进射程压到 1cm → 恒不可行
            set_param("shoot.kBankMinQ", 2.0);       // 借墙接管门槛拉到 2.0 → 恒不接管
            set_param("shoot.kMinOpen", 90.0);       // 远射闸门拉到 90° → 恒不可行（本条测蜂群几何）
        }
        ~BankOffGuard() {
            set_param("shoot.kBankCarryMax", carry_save);
            set_param("shoot.kBankMinQ", minq_save);
            set_param("shoot.kMinOpen", minopen_save);
        }
    } bank_off_guard;
    (void)bank_off_guard;
    {
        WorldModel wm = scene(100, 120);
        double ux, uy; herd_dir(wm, ux, uy);
        wm.home[2].x = 100 - ux * 8; wm.home[2].y = 120 - uy * 8; wm.home[2].rot = angle_to(0, 0, ux, uy);
        run_assist(wm, 2);
        if (!expect(wm, 2, 100 + ux * 22.0, 120 + uy * 22.0, "推穿")) return 1;
    }
    {
        WorldModel wm = scene(100, 120);
        double ux, uy; herd_dir(wm, ux, uy);
        wm.home[2].x = 100 + ux * 30; wm.home[2].y = 120 + uy * 30;
        run_assist(wm, 2);
        if (!expect(wm, 2, 100 - ux * 4 - uy * 17.0, 120 - uy * 4 + ux * 17.0, "横绕")) return 1;
    }
    {
        WorldModel wm = scene(100, 40);
        run_assist(wm, 2);
        if (!expect(wm, 2, 122, 132, "弱侧跟进")) return 1;
    }
    {
        WorldModel wm = scene(30, 95);
        run_assist(wm, 2); run_midfield(wm, 3);
        if (!expect(wm, 2, 68, 122, "门外等二点(上)") || !expect(wm, 3, 68, 58, "门外等二点(下)")) return 1;
    }
    {
        WorldModel wm = scene(100, 95);
        double ux, uy; herd_dir(wm, ux, uy);
        wm.home[1].x = 100 - ux * 8; wm.home[1].y = 95 - uy * 8;
        wm.home[2].x = 120; wm.home[2].y = 130;
        double side = ((120 - 100) * -uy + (130 - 95) * ux) >= 0 ? 1.0 : -1.0;
        run_assist(wm, 2);
        if (!expect(wm, 2, 100 - ux * 16 - uy * side * 22, 95 - uy * 16 + ux * side * 22, "护送")) return 1;
    }
    {
        WorldModel wm = scene(175, 100);
        wm.threat_level = 0.9;
        WorldModel dead = wm; dead.game_state = PM_PlaceKick_Blue;   // 死球期 = 不接管的参照
        run_assist(wm, 2); run_assist(dead, 2);
        if (fabs(wm.home[2].vl - dead.home[2].vl) > 1e-6 || fabs(wm.home[2].vr - dead.home[2].vr) > 1e-6) {
            printf("FAIL: swarm 球近我方门时不应接管\n"); return 1;
        }
    }
    printf("swarm attack: OK (推穿/横绕不回顶/弱侧跟进/门外等二点/让位护送/近门交回防守)\n");
    return 0;
}

// 门将「门线封堵」：球在门框内轨迹上、马上到线 → 抢门线预测落点
static int test_goalie_line_cover() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队：己方门线 x=220，门框 y∈[70,110]
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 180; wm.home[i].y = 90; }
    double tx = 0.0, ty = 0.0;

    wm.ball.x = 210; wm.ball.y = 95; wm.ball.vx = 2.0; wm.ball.vy = 0.0;
    wm.home[0].x = 195; wm.home[0].y = 95;
    if (!gk_cover_line_point(wm, 0, tx, ty)) {
        printf("FAIL: 球在门框内轨迹上应触发门线封堵\n");
        return 1;
    }
    if (fabs(tx - 217.0) > 0.5 || fabs(ty - 95.0) > 0.5) {
        printf("FAIL: 门线封堵目标应为 (217,95)，实际 (%.1f,%.1f)\n", tx, ty);
        return 1;
    }
    wm.ball.vx = -2.0;
    if (gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 球背离门不应触发\n"); return 1; }
    wm.ball.x = 210; wm.ball.y = 130; wm.ball.vx = 2.0; wm.ball.vy = -1.0;
    if (gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 会偏出的球不应触发\n"); return 1; }
    wm.ball.x = 140; wm.ball.y = 95; wm.ball.vx = 2.0; wm.ball.vy = 0.0;
    if (gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 球还远不应触发\n"); return 1; }
    wm.ball.x = 210; wm.ball.y = 95; wm.ball.vx = 0.5; wm.ball.vy = 0.0;
    if (gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 太慢的球不应触发门线封堵\n"); return 1; }
    wm.ball.vx = 2.0;
    wm.home[0].x = 205; wm.home[0].y = 95;
    if (gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 门将贴球时应让位给清球\n"); return 1; }
    wm.ctx = TeamContext{false};
    wm.ball.x = 10; wm.ball.y = 95; wm.ball.vx = -2.0; wm.ball.vy = 0.0;
    wm.home[0].x = 25; wm.home[0].y = 95;
    if (!gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 黄队镜像应触发\n"); return 1; }
    if (fabs(tx - 3.0) > 0.5) { printf("FAIL: 黄队镜像目标 x 应为 3，实际 %.1f\n", tx); return 1; }

    printf("goalie line cover: OK (门框内轨迹抢落点/背离-偏出-太远-太慢-贴球让位/黄队镜像)\n");
    return 0;
}

// 争抢直冲：对手贴球、主攻在球侧上方 → 直接朝球冲，不绕球后
static int test_contest_charge() {
    TeamContext ctx{true};               // 蓝队：攻向 x=0
    WorldModel wm; wm.ctx = ctx; wm.ball.valid = true; wm.ball_pred.valid = true;
    wm.ball.x = wm.ball_pred.x = 100; wm.ball.y = wm.ball_pred.y = 90;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 200; wm.home[i].y = 20 + i * 30; wm.opp[i].x = 20; wm.opp[i].y = 20 + i * 30; }
    wm.opp[2].x = 85; wm.opp[2].y = 90;                    // 对手贴球 15cm
    wm.home[1].x = 98; wm.home[1].y = 102; wm.home[1].rot = -80;   // 主攻贴在球侧上方 12cm、机头朝球（旧逻辑：绕回球后 20cm）
    run_active(wm, 1);
    const RobotState &r = wm.home[1];
    if (r.vl < 50.0 || r.vr < 50.0 || std::fabs(r.vl - r.vr) > 30.0) {
        printf("FAIL: 争抢时应直冲球 (vl=%.1f vr=%.1f)\n", r.vl, r.vr); return 1;
    }
    printf("contest charge: OK (争抢时安全侧直冲球)\n");
    return 0;
}

// 近球不减速/不原地转正/不等对准；门球静止球门将须 200 帧内把球推出
static int gk_kick_touch_frames(double by, double reach, double across = -1.0, double through = -1.0) {
    const double save_reach   = get_param("roles.kGkKickReach", 1.0);
    const double save_across  = get_param("roles.kGkAlignAcross", 3.0);
    const double save_through = get_param("roles.kGkKickThrough", 30.0);
    ::simuro5::set_param("roles.kGkKickReach", reach);
    if (across  >= 0.0) ::simuro5::set_param("roles.kGkAlignAcross", across);
    if (through >= 0.0) ::simuro5::set_param("roles.kGkKickThrough", through);
    WorldModel wm; wm.ctx = TeamContext{true}; wm.game_state = PM_PlayOn;
    wm.ball.valid = wm.ball_pred.valid = true;
    wm.ball.x = wm.ball_pred.x = 205.1; wm.ball.y = wm.ball_pred.y = by;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 120; wm.home[i].y = 20 + i * 30; wm.opp[i].x = 60; wm.opp[i].y = 20 + i * 30; wm.role[i] = ROLE_PASSIVE; }
    wm.role[0] = ROLE_GOALIE;
    wm.home[0].x = 214.8; wm.home[0].y = 89.9; wm.home[0].rot = -90.0;
    int hit = -1;
    for (int f = 0; f < 200 && hit < 0; ++f) {
        run_goalie(wm, 0);
        RobotState &r = wm.home[0];
        double v = 0.5 * (r.vl + r.vr) * 0.025, w = (r.vr - r.vl) / 10.0 * 0.025 * 180.0 / SIMURO5_PI;
        r.rot = normalize_angle(r.rot + w);
        r.x += v * std::cos(r.rot * SIMURO5_PI / 180.0); r.y += v * std::sin(r.rot * SIMURO5_PI / 180.0);
        double ex = wm.ball.x - r.x, ey = wm.ball.y - r.y, d = std::hypot(ex, ey);
        if (d < 7.0 && d > 1e-6) {                 // 碰到 → 球被顶到车身外沿
            wm.ball.x = r.x + ex / d * 7.0; wm.ball.y = r.y + ey / d * 7.0;
        }
        wm.ball_pred.x = wm.ball.x; wm.ball_pred.y = wm.ball.y;
        if (wm.ball.x > 219.0) { hit = -2; break; }   // 顶进自家门
        if (wm.ball.x < 195.0) hit = f;               // 推出门区一带
    }
    ::simuro5::set_param("roles.kGkKickReach", save_reach);
    ::simuro5::set_param("roles.kGkAlignAcross", save_across);
    ::simuro5::set_param("roles.kGkKickThrough", save_through);
    return hit;
}
static int test_gk_goal_kick_reach() {
    int old72 = gk_kick_touch_frames(72.0, 0.0), new72 = gk_kick_touch_frames(72.0, 1.0),
        new78 = gk_kick_touch_frames(78.0, 1.0), new66 = gk_kick_touch_frames(66.0, 1.0);
    printf("gk goal kick reach: 旧 y72=%d / 新 y72=%d y78=%d y66=%d（帧，-1=没推出 -2=乌龙）\n", old72, new72, new78, new66);
    const double acs[3] = {3.0, 6.0, 8.0};
    const double ths[3] = {30.0, 36.0, 42.0};
    for (int i = 0; i < 3; ++i) {
        printf("  [矩阵] ac=%.0f:", acs[i]);
        for (int j = 0; j < 3; j++) {
            printf("  th=%.0f→%3d/%3d/%3d", ths[j],
                   gk_kick_touch_frames(66.0, 1.0, acs[i], ths[j]),
                   gk_kick_touch_frames(72.0, 1.0, acs[i], ths[j]),
                   gk_kick_touch_frames(78.0, 1.0, acs[i], ths[j]));
        }
        printf("   (y66/72/78)\n");
    }
    if (new72 < 0 || new78 < 0 || new66 < 0) { printf("FAIL: 门球静止球门将 200 帧内未触球\n"); return 1; }
    // 发球必须利落：y78 是 formation_set_ball 实际摆的球门球落点，改前要 130 帧（3.25 秒左右磨蹭）
    if (new78 > 60) { printf("FAIL: 门球 y78 发球拖沓 %d 帧（应 <=60；改前 130 帧，属左右磨蹭回归）\n", new78); return 1; }
    if (new66 > 70 || new72 > 90) { printf("FAIL: 门球 y66/y72 发球拖沓 %d/%d 帧（应 <=70/<=90；改前 83/121）\n", new66, new72); return 1; }
    return 0;
}

static int test_zone_mode() {
    auto base = [](WorldModel &wm, double bx, double by) {
        TeamContext ctx{true};               // 蓝队：己方门 x=220，攻向 -x
        wm = WorldModel(); wm.ctx = ctx; wm.ball.valid = true; wm.ball_pred.valid = true;
        wm.ball.x = wm.ball_pred.x = bx; wm.ball.y = wm.ball_pred.y = by;
        for (int i = 0; i < 5; ++i) { wm.home[i].x = 110; wm.home[i].y = 10 + i * 5; wm.opp[i].x = 20; wm.opp[i].y = 20 + i * 30; }
        wm.role[0] = ROLE_GOALIE; wm.role[1] = ROLE_ACTIVE; wm.role[2] = ROLE_ASSIST; wm.role[3] = ROLE_MIDFIELD; wm.role[4] = ROLE_PASSIVE;
    };
    auto same = [](const RobotState &a, const RobotState &b) { return std::fabs(a.vl - b.vl) < 1e-6 && std::fabs(a.vr - b.vr) < 1e-6; };
    WorldModel wm;
    base(wm, 150, 90); wm.home[4].x = 170; wm.home[4].y = 90; wm.home[4].rot = 180;
    RobotState e = wm.home[4]; motion::position(e, 130, 90, motion::TM_PASS);
    run_zone(wm, 4);
    if (!same(wm.home[4], e) || wm.home[4].vl < 50) { printf("FAIL: 中卫应直冲穿球 vl=%.1f vr=%.1f\n", wm.home[4].vl, wm.home[4].vr); return 1; }
    base(wm, 180, 90); wm.home[4].x = 160; wm.home[4].y = 95; wm.home[4].rot = 0;
    e = wm.home[4]; motion::position(e, 185, 112, motion::TM_PASS);
    run_zone(wm, 4);
    if (!same(wm.home[4], e)) { printf("FAIL: 己方门前站错侧应侧绕\n"); return 1; }
    base(wm, 150, 40); wm.home[2].x = 150; wm.home[2].y = 150; wm.home[2].rot = 0;
    e = wm.home[2]; motion::position(e, 158, 120);
    run_zone(wm, 2);
    if (!same(wm.home[2], e)) { printf("FAIL: 上翼应站 (158,120)\n"); return 1; }
    base(wm, 200, 60); wm.home[4].x = 195; wm.home[4].y = 65; wm.home[3].x = 150; wm.home[3].y = 100; wm.home[3].rot = 0;
    e = wm.home[3]; motion::position(e, 175, 60);
    run_zone(wm, 3);
    if (!same(wm.home[3], e)) { printf("FAIL: 非最近者不许进己方大禁区\n"); return 1; }
    printf("zone mode: OK (中卫直冲/门前侧绕/边翼站位/己方禁区纪律)\n");
    return 0;
}

static int test_no_align_wait() {
    auto run = [](double sw, double &fwd, double &turn) {
        ::simuro5::set_param("roles.kNoAlignWait", sw);
        TeamContext ctx{true};               // 蓝队：攻向 x=0
        WorldModel wm; wm.ctx = ctx; wm.ball.valid = true; wm.ball_pred.valid = true;
        wm.ball.x = wm.ball_pred.x = 100; wm.ball.y = wm.ball_pred.y = 90;
        for (int i = 0; i < 5; ++i) { wm.home[i].x = 200; wm.home[i].y = 20 + i * 30; wm.opp[i].x = 20; wm.opp[i].y = 20 + i * 30; }
        wm.home[1].x = 108; wm.home[1].y = 92; wm.home[1].rot = 120;   // 球后 8cm，机头偏 60°
        run_active(wm, 1);
        fwd = (wm.home[1].vl + wm.home[1].vr) / 2; turn = std::fabs(wm.home[1].vl - wm.home[1].vr);
    };
    double f0, t0, f1, t1;
    run(0.0, f0, t0); run(1.0, f1, t1);
    ::simuro5::reset_params();
    printf("no align wait: old fwd=%.1f turn=%.1f | new fwd=%.1f turn=%.1f\n", f0, t0, f1, t1);
    if (!(std::fabs(f1) > std::fabs(f0) + 20.0)) { printf("FAIL: 新逻辑应直接推穿而非原地转正\n"); return 1; }
    printf("no align wait: OK\n");
    return 0;
}

static int test_opp_box_instant_exit() {
    TeamContext ctx{true};               // 蓝队：对方门 x=0
    WorldModel wm; wm.ctx = ctx; wm.ball.valid = true;
    wm.ball.x = 5; wm.ball.y = 154;
    const double hx[5] = {211, 20, 54, 23, 72}, hy[5] = {103, 100, 153, 63, 113};
    const double ox[5] = {2, 8, 5, 33, 47}, oy[5] = {106, 132, 58, 121, 101};
    for (int i = 0; i < 5; ++i) { wm.home[i].x = hx[i]; wm.home[i].y = hy[i]; wm.opp[i].x = ox[i]; wm.opp[i].y = oy[i]; }
    Strategy strat;
    strat.run(wm);
    if (wm.ga_cooldown[3] <= 0) { printf("FAIL: MID 在对方门区应当帧撤出(cooldown=%d)\n", wm.ga_cooldown[3]); return 1; }
    if (wm.ga_cooldown[1] != 0) { printf("FAIL: ACTIVE 在对方门区应保留 15 帧宽限\n"); return 1; }
    printf("opp box instant exit: OK (非主攻进对方门区当帧撤出，主攻保留宽限)\n");
    return 0;
}

static int test_live_play() {
    WorldModel wm; Environment e; TeamContext ctx{true};
    auto step = [&](double bx, double by, int gs) {
        Environment prev = e; init_env(e, bx, by); e.gameState = gs;
        e.lastBall.pos = prev.currentBall.pos;
        wm.update(&e, ctx);
        return wm.live_play;
    };
    init_env(e, 110, 90);
    if (step(110, 90, PM_FreeBall_LeftBot)) { printf("FAIL: 重启摆球首帧不应算活球\n"); return 1; }
    if (step(112, 90, PM_FreeBall_LeftBot)) { printf("FAIL: 球只动 2cm 不应算活球\n"); return 1; }
    if (!step(118, 91, PM_FreeBall_LeftBot)) { printf("FAIL: 开球后 gameState 不回 PlayOn 也应算活球\n"); return 1; }
    if (!step(140, 70, PM_FreeBall_LeftBot)) { printf("FAIL: 比赛进行中应保持活球\n"); return 1; }
    if (step(55, 30, PM_FreeBall_LeftBot)) { printf("FAIL: 球位跳变(重新摆球)应回到死球\n"); return 1; }
    if (step(55, 30, PM_PlaceKick_Blue)) { printf("FAIL: gameState 变化应回到死球\n"); return 1; }
    if (!step(55, 30, PM_PlayOn)) { printf("FAIL: PlayOn 必须算活球\n"); return 1; }
    printf("live play: OK (真机 gameState 不回 PlayOn 时按球离开摆放点判活球)\n");
    return 0;
}

// 回归：门将封角站位深度必须"越近越贴门线"，且不许站到太外面。
// 旧实现（defense.hpp:goalie_block_depth）写死 max_out=40：
//   球离门 >=52cm 时门将一律站到离门线 40cm。真机实测门将 x 中位 14~16cm、p99 42cm、
//   最大 49.8cm；由于回程太长，丢球时门将 x=8.6~27.7 而球已在门线上（球从门将身后进网）。
static int test_gk_retreat_ball_side() {
    TeamContext ctx{true};                       // 蓝位：己方门 x=220
    const double max_out = 22.0, margin = 6.0, guard = 10.0;
    double d60 = goalie_block_depth(ctx, ctx.our_goal_x() - 60.0, guard, max_out, margin);
    double d20 = goalie_block_depth(ctx, ctx.our_goal_x() - 20.0, guard, max_out, margin);
    double d8  = goalie_block_depth(ctx, ctx.our_goal_x() -  8.0, guard, max_out, margin);
    if (d60 > max_out + 1e-9) {
        printf("FAIL: 门将封角深度 %.1f 超过上限 %.1f（旧写死 40，实测站到 34~49cm 后回不来）\n",
               d60, max_out);
        return 1;
    }
    if (d20 > (20.0 - margin) + 1e-9) {
        printf("FAIL: 球在 20cm 时门将深度 %.1f 没有比球更靠门 %.1fcm（会站到球外侧）\n",
               d20, margin);
        return 1;
    }
    if (!(d60 >= d20 - 1e-9 && d20 >= d8 - 1e-9)) {
        printf("FAIL: 封角深度未随球逼近单调收缩（60cm→%.1f / 20cm→%.1f / 8cm→%.1f）\n",
               d60, d20, d8);
        return 1;
    }
    if (d8 < guard - 1e-9) {
        printf("FAIL: 球贴门线时门将深度 %.1f 跌破常规站位 %.1f\n", d8, guard);
        return 1;
    }
    return 0;
}

static int test_goalie_line_block() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 150; wm.home[i].y = 90; wm.opp[i].x = 100; wm.opp[i].y = 90; }
    double tx = 0.0, ty = 0.0;

    wm.ball.x = 197; wm.ball.y = 88.5; wm.ball.vx = 0.47; wm.ball.vy = -0.04;
    wm.home[0].x = 209; wm.home[0].y = 80; wm.home[0].rot = -110;
    if (!gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 中路慢球应对线\n"); return 1; }
    double y_line = 88.5 - 0.04 * (210.0 - 197.0) / 0.47;
    if (fabs(tx - 210.0) > 0.5 || fabs(ty - y_line) > 0.5) {
        printf("FAIL: 对线目标应为 (210,%.1f)，实际 (%.1f,%.1f)\n", y_line, tx, ty); return 1;
    }
    wm.opp[0].x = 190; wm.opp[0].y = 88.5;
    if (gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 对方贴球不应对线\n"); return 1; }
    wm.opp[0].x = 100; wm.opp[0].y = 90;
    wm.ball.vx = 4.0; wm.ball.vy = 0.0;
    if (gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 快球不应走对线\n"); return 1; }
    wm.ball.x = 197; wm.ball.y = 130; wm.ball.vx = 0.5; wm.ball.vy = 0.0;
    if (gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 偏出的慢球不应对线\n"); return 1; }
    wm.ball.x = 217.5; wm.ball.y = 140; wm.ball.vx = 0.0; wm.ball.vy = -1.0;
    wm.home[0].x = 209; wm.home[0].y = 104;
    if (!gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 贴门线滚球应堵路径\n"); return 1; }
    if (fabs(tx - 217.0) > 0.5 || fabs(ty - 106.0) > 0.5) {
        printf("FAIL: 贴门线堵点应为 (217,106)，实际 (%.1f,%.1f)\n", tx, ty); return 1;
    }
    wm.ball.y = 80; wm.ball.vy = 2.0;
    if (!gk_line_block_point(wm, 0, tx, ty) || fabs(ty - 92.0) > 0.5) {
        printf("FAIL: 门口内贴线滚球应堵在球前 (y=92)，实际 %.1f\n", ty); return 1;
    }
    wm.ball.y = 40; wm.ball.vy = -2.0;
    if (gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 背离门口的贴线球不应接管\n"); return 1; }
    wm.ctx = TeamContext{false};
    wm.ball.x = 23; wm.ball.y = 90; wm.ball.vx = -0.5; wm.ball.vy = 0.0;
    wm.home[0].x = 11; wm.home[0].y = 80;
    if (!gk_line_block_point(wm, 0, tx, ty) || fabs(tx - 10.0) > 0.5 || fabs(ty - 90.0) > 0.5) {
        printf("FAIL: 黄队镜像对线目标应为 (10,90)，实际 (%.1f,%.1f)\n", tx, ty); return 1;
    }
    printf("goalie line block: OK (慢球对线/贴球-快球-偏出不接管/贴门线堵路径/黄队镜像)\n");
    return 0;
}

// 带权匈牙利求解器：手算最优 + 与暴力枚举全排列对拍（n=3/4/5，300 组）
static double brute_force_min(const double *c, int n) {
    int perm[kHungarianMaxN];
    for (int i = 0; i < n; ++i) perm[i] = i;
    double best = 1e18;
    do {
        double s = 0.0;
        for (int i = 0; i < n; ++i) s += c[i * n + perm[i]];
        if (s < best) best = s;
    } while (std::next_permutation(perm, perm + n));
    return best;
}

static int test_hungarian_solver() {
    {
        const double c[9] = {4, 1, 3,
                             2, 0, 5,
                             3, 2, 2};
        int a[3] = {-1, -1, -1};
        if (!hungarian_solve(c, 3, a)) { printf("FAIL: 3x3 应有解\n"); return 1; }
        if (a[0] != 1 || a[1] != 0 || a[2] != 2) {
            printf("FAIL: 3x3 最优应为 (1,0,2)，实际 (%d,%d,%d)\n", a[0], a[1], a[2]);
            return 1;
        }
    }
    {
        const double c[4] = {10, 30,
                             12, 15};
        int a[2] = {-1, -1};
        if (!hungarian_solve(c, 2, a)) { printf("FAIL: 2x2 应有解\n"); return 1; }
        if (a[0] == a[1]) { printf("FAIL: 匈牙利不该让两行配同一列\n"); return 1; }
        if (a[0] != 0 || a[1] != 1) {
            printf("FAIL: 撞车例子最优应为 (0,1)，实际 (%d,%d)\n", a[0], a[1]);
            return 1;
        }
    }
    {
        srand(20260930);
        const int n_list[3] = {3, 4, 5};
        for (int t = 0; t < 300; ++t) {
            const int n = n_list[t % 3];
            double c[kHungarianMaxN * kHungarianMaxN];
            for (int i = 0; i < n * n; ++i) c[i] = (rand() % 2000) / 10.0;   // 0~200cm
            int a[kHungarianMaxN];
            if (!hungarian_solve(c, n, a)) {
                printf("FAIL: 随机 %dx%d 求解失败\n", n, n); return 1;
            }
            bool seen[kHungarianMaxN] = {false};
            double got = 0.0;
            for (int i = 0; i < n; ++i) {
                if (a[i] < 0 || a[i] >= n || seen[a[i]]) {
                    printf("FAIL: 随机 %dx%d 的解不是合法排列\n", n, n); return 1;
                }
                seen[a[i]] = true;
                got += c[i * n + a[i]];
            }
            const double best = brute_force_min(c, n);
            if (fabs(got - best) > 1e-6) {
                printf("FAIL: 随机 %dx%d 解 %.3f != 暴力最优 %.3f\n", n, n, got, best);
                return 1;
            }
        }
    }
    printf("hungarian solver: OK (手算最优/撞车一一匹配/300 组与暴力枚举对拍)\n");
    return 0;
}

// 盯人分配场景脚手架（蓝队、威胁过门槛、2=ASSIST 3=MIDFIELD 4=PASSIVE）
static WorldModel mark_scene() {
    WorldModel wm;
    wm.ctx = TeamContext{true};          // 蓝队：守 x=220、攻 x=0
    wm.ball.valid = true;
    wm.threat_level = 0.8;               // 过 defense.kMarkRelGate(0.55)
    wm.role[0] = ROLE_GOALIE;
    wm.role[1] = ROLE_ACTIVE;
    wm.role[2] = ROLE_ASSIST;
    wm.role[3] = ROLE_MIDFIELD;
    wm.role[4] = ROLE_PASSIVE;
    return wm;
}

// 防抖对照：80 帧中途互换两车位置（省 7.2cm），返回换人次数
static int run_mark_swap_probe(double lambda) {
    const double lam_save   = get_param("defense.kMarkLambda", 25.0);
    const double sharp_save = get_param("defense.kSharpDefense", 0.0);
    set_param("defense.kSharpDefense", 0.0);
    set_param("defense.kMarkLambda", lambda);
    WorldModel wm = mark_scene();
    wm.ball.x = 160; wm.ball.y = 90; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    const double opp_base[5][2] = {{150, 70}, {150, 110}, {60, 20}, {60, 160}, {90, 40}};
    for (int j = 0; j < PLAYERS_PER_SIDE; ++j) {
        wm.opp[j].x = opp_base[j][0];
        wm.opp[j].y = opp_base[j][1];
    }
    wm.home[4].x = 30; wm.home[4].y = 90;
    wm.mark_switch_events = 0;
    for (int f = 0; f < 80; ++f) {
        const bool phase2 = (f >= 40);
        wm.home[2].x = 140; wm.home[2].y = phase2 ? 92 : 88;
        wm.home[3].x = 140; wm.home[3].y = phase2 ? 88 : 92;
        for (int j = 0; j < PLAYERS_PER_SIDE; ++j) {   // 确定性微扰（±0.3cm）
            wm.opp[j].x = opp_base[j][0] + 0.3 * sin(f * 1.7 + j);
            wm.opp[j].y = opp_base[j][1] + 0.3 * cos(f * 2.3 + j);
        }
        assign_marks(wm);
    }
    const int switches = wm.mark_switch_events;
    set_param("defense.kMarkLambda", lam_save);
    set_param("defense.kSharpDefense", sharp_save);
    return switches;
}

// 带权匈牙利盯人分配：一个危险对手只 1 台盯；局面几乎不变时不换人
static int test_mark_assignment() {
    {
        WorldModel wm = mark_scene();
        wm.ball.x = 200; wm.ball.y = 90; wm.ball.vx = -1.0; wm.ball.vy = 0.0;
        wm.opp[0].x = 195; wm.opp[0].y = 90;                 // 贴球 = 最危险
        wm.opp[1].x = 40;  wm.opp[1].y = 30;
        wm.opp[2].x = 40;  wm.opp[2].y = 150;
        wm.opp[3].x = 60;  wm.opp[3].y = 90;
        wm.opp[4].x = 30;  wm.opp[4].y = 60;
        wm.home[2].x = 185; wm.home[2].y = 80;               // 三台都想盯 0 号
        wm.home[3].x = 185; wm.home[3].y = 100;
        wm.home[4].x = 190; wm.home[4].y = 90;               // 最近（5cm）
        const int n = assign_marks(wm);
        int on0 = 0;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) if (wm.mark_assign[i] == 0) ++on0;
        if (!wm.mark_assign_valid) { printf("FAIL: 门槛内 mark_assign_valid 应为 true\n"); return 1; }
        if (on0 != 1) { printf("FAIL: 危险对手应恰好被 1 台盯，实际 %d 台\n", on0); return 1; }
        if (wm.mark_assign[4] != 0) {
            printf("FAIL: 应由最近的 4 号盯 0 号，实际 %d\n", wm.mark_assign[4]); return 1;
        }
        if (n != 1) { printf("FAIL: 只应指派 1 台（其余闲着），实际 %d\n", n); return 1; }
        if (wm.mark_assign[1] != -1 || wm.mark_assign[0] != -1) {
            printf("FAIL: 门将/主攻不参与盯人分配\n"); return 1;
        }
    }
    {
        const int sw_free = run_mark_swap_probe(0.0);
        const int sw_locked = run_mark_swap_probe(25.0);
        if (sw_free < 1) {
            printf("FAIL: λ=0 时几何互换应导致换人（说明本用例的\"裸最优\"确实翻转了），实际 %d\n", sw_free);
            return 1;
        }
        if (sw_locked != 0) {
            printf("FAIL: λ=25 应顶住这次几何互换（不许换人），实际换了 %d 次\n", sw_locked);
            return 1;
        }
    }
    {
        const double save = get_param("defense.kMarkHungarian", 1.0);
        const double gate = get_param("defense.kMarkRelGate", 0.55);
        set_param("defense.kMarkHungarian", 0.0);
        WorldModel wm = mark_scene();
        wm.ball.x = 200; wm.ball.y = 90; wm.ball.vx = -1.0; wm.ball.vy = 0.0;
        for (int j = 0; j < PLAYERS_PER_SIDE; ++j) { wm.opp[j].x = 195; wm.opp[j].y = 80 + j * 5; }
        const int n_off = assign_marks(wm);
        const bool valid_off = wm.mark_assign_valid;
        int any_off = 0;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) if (wm.mark_assign[i] != -1) any_off = 1;
        set_param("defense.kMarkHungarian", save);
        WorldModel wm2 = mark_scene();
        wm2.threat_level = gate - 0.1;
        for (int j = 0; j < PLAYERS_PER_SIDE; ++j) { wm2.opp[j].x = 195; wm2.opp[j].y = 80 + j * 5; }
        const int n_low = assign_marks(wm2);
        if (n_off != 0 || valid_off || any_off) {
            printf("FAIL: 关掉 switch 后应无任何指派 (n=%d valid=%d any=%d)\n",
                   n_off, (int)valid_off, any_off);
            return 1;
        }
        if (n_low != 0 || wm2.mark_assign_valid) {
            printf("FAIL: 威胁低于门槛时不应指派 (n=%d valid=%d)\n", n_low, (int)wm2.mark_assign_valid);
            return 1;
        }
    }
    printf("mark assignment: OK (危险对手只 1 台盯/最近者优先/λ=0 会换 vs λ=25 顶住/开关与门槛可回退)\n");
    return 0;
}

int main(int argc, char **argv) {
    int rc = 0;
    bool coop_pass_only = false;
// --params <文件>：注入参数后再跑全部测试（自动调参的行为护栏）
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--coop-pass-only") == 0) coop_pass_only = true;
        if (strcmp(argv[i], "--params") == 0 && i + 1 < argc) {
            int n = simuro5::apply_param_file(argv[++i]);
            if (n < 0) { printf("offline_test: 无法读取参数文件 %s\n", argv[i]); return 2; }
            printf("=== 已注入 %d 个参数（%s）===\n", n, argv[i]);
        }
    }
    if (coop_pass_only) {
        rc = test_coop_pass();
        rc |= test_coop_pass_task();
        rc |= test_coop_lifecycle();
        rc |= test_ordinary_pass_task();
        rc |= test_pass_readiness_gate();
        rc |= test_receiver_meet_ball();
        rc |= test_pass_opponent_first_cancel();
        rc |= test_pass_receive_control();
        printf(rc ? "=== COOP PASS TEST FAILED ===\n" : "=== COOP PASS TEST PASSED ===\n");
        return rc;
    }
    rc |= test_strategy_run(300);
    rc |= test_formation();
    rc |= test_placement_semantics();
    rc |= test_placement_rule_boxes();     // 摆位规则纪律：门区(7.10/7.17) + 争球 1/4 场(7.15)
    rc |= test_penalty_spot_detect();
    rc |= test_penalty_shot_prep();
    rc |= test_penalty_aim_offcenter();
    rc |= test_own_goalarea_guard();
    rc |= test_own_penalty_count();        // 规则 7.10.4：己方大禁区非门将 ≤3 人
    rc |= test_rule_goal_area();
    rc |= test_swarm_attack();
    rc |= test_no_reverse_through_ball();
    rc |= test_opp_kick_predict();
    rc |= test_coop_pass();
    rc |= test_coop_pass_task();
    rc |= test_coop_lifecycle();
    rc |= test_ordinary_pass_task();
    rc |= test_pass_readiness_gate();
    rc |= test_receiver_meet_ball();
    rc |= test_pass_opponent_first_cancel();
    rc |= test_pass_receive_control();
    rc |= test_goalie_side_step();
    rc |= test_rebound_and_doubleteam();
    rc |= test_possession_source();
    rc |= test_goalie_clear_push();
    rc |= test_goalie_straight_clear();
    rc |= test_goalie_kick_far();
    rc |= test_passive_front_sweep();
    rc |= test_gk_on_line_no_push();
    rc |= test_goalie_challenge();
    rc |= test_doubleteam_loose_ball();
    rc |= test_presser();
    rc |= test_goalie_line_cover();
    rc |= test_goalie_line_block();
    rc |= test_live_play();
    rc |= test_opp_box_instant_exit();
    rc |= test_contest_charge();
    rc |= test_no_align_wait();
    rc |= test_zone_mode();
    rc |= test_gk_goal_kick_reach();
    rc |= test_defense_intercept();
    rc |= test_hungarian_solver();
    rc |= test_mark_assignment();
    rc |= test_goalie_predict();
    rc |= test_defense_reach();
    rc |= test_defense_face_incoming();
    rc |= test_ball_meeting_point();
    rc |= test_goal_cover();
    rc |= test_goal_cover_yellow();
    rc |= test_team_state();
    rc |= test_fixed_roles();
    rc |= test_pass();
    rc |= test_pass_threat_weight();
    rc |= test_goalie_scenarios();
    rc |= test_goalie_og_fixes();
    rc |= test_gk_retreat_ball_side();
    rc |= test_roles_spread();
    rc |= test_segment_circle();
    rc |= test_route_straight();
    rc |= test_route_avoid();
    rc |= test_route_optimality();
    rc |= test_route_cluster();
    rc |= test_route_global_safety();
    rc |= test_follow_route();
    rc |= test_shoot_push_limit();
    rc |= test_shoot_plan();
    rc |= test_bank_shot();
    rc |= test_bank_force_test();
    rc |= test_corridor_plan();
    rc |= test_corridor_progress();
    rc |= test_trail_stand();
    rc |= test_active_ga_retreat();
    rc |= test_active_corner_rescue();
    rc |= test_no_push_zone();
    rc |= test_motion_brake_envelope();
    rc |= test_motion_stop_convergence();
    rc |= test_motion_lateral_target();
    rc |= test_motion_pass_mode();
    rc |= test_motion_bounds();
    rc |= test_motion_aligned();
    printf(rc ? "=== TEST FAILED ===\n" : "=== ALL TESTS PASSED ===\n");
    return rc;
}
