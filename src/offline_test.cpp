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
#include "simuro5/hungarian.hpp"   // 带权匈牙利求解器（第 97 轮盯人分配）
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

    // 边界反弹：球朝底/顶边线滚，直线会出界，反射后应折返到界内。
    // ⚠️ 2026-09-14 修正（docs/06 第 65 轮）：反射**不是理想镜面**——实测法向 0.66、切向 0.81，
    //    出射线比镜面更贴墙：同样的输入镜面会给 40 / 140，实测系数给 32.6 / 147.4。
    //    ⚠️ 2026-09-30：本用例走的是 field_info 的 ball_wall_rest()（仍是 0.66，门将/后卫反弹
    //    预测共用）。第 75 轮补记已判定真机其实是 **0.45**，但**本轮刻意不动这条路径**——
    //    只改了借墙射门（shoot.kBankWallRest），以便真机上把"借墙"单独测出来。
    //    ⇒ 若将来要修这里，期望值 32.6 / 147.4 必须按新系数重算，不能直接套用。
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

// 可达性判断单测：近处采纳截点、远处回退卡位
static int test_defense_reach() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.passive_x = 150; wm.passive_y = 70;   // 回退用的静态站位点（明显区别于截点）

    // 球在 (100,90) 朝门滚 vx=3 → 截点本应在 (170,90)（门区修正后 165,90）
    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 3.0; wm.ball.vy = 0.0;

    // 场景 A：2号就站在截点附近，赶得上 → 应采纳截点（偏离 passive）
    wm.home[1].x = 168; wm.home[1].y = 90;
    DefensePlan a = plan_defense(wm, 1);
    if (fabs(a.target_x - wm.passive_x) < 0.5) {
        printf("FAIL: 近处应采纳截点而非卡位 (%.1f,%.1f)\n", a.target_x, a.target_y); return 1;
    }

    // 场景 B：2号在场地另一头，赶不上 → 应回退卡位（等于 passive）
    wm.home[1].x = 20; wm.home[1].y = 90;
    DefensePlan b = plan_defense(wm, 1);
    if (fabs(b.target_x - wm.passive_x) > 0.5 || fabs(b.target_y - wm.passive_y) > 0.5) {
        printf("FAIL: 远处应回退卡位 (%.1f,%.1f)\n", b.target_x, b.target_y); return 1;
    }

    printf("defense reach: OK (近处截断/远处回退)\n");
    return 0;
}

// 第 103 轮（用户指令："防守不能对准球冲过来的方向"）：
//   断球点落在球来路上时，plan_defense 必须同时给出「迎球朝向」= 球来向的反方向。
static int test_defense_face_incoming() {
    WorldModel wm;
    wm.ctx = TeamContext{true};          // 蓝队守 x=220
    wm.ball.valid = true;
    wm.passive_x = 150; wm.passive_y = 70;

    // ① 正对门滚的球：v=(3,0) → 迎球朝向应 = 180°（机头朝 -x，正对来球）
    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 3.0; wm.ball.vy = 0.0;
    wm.home[1].x = 168; wm.home[1].y = 90;
    DefensePlan a = plan_defense(wm, 1);
    if (!a.face_incoming) { printf("FAIL: 断球点应给出迎球朝向\n"); return 1; }
    if (fabs(angle_diff(a.aim_rot, 180.0)) > 1e-6) {
        printf("FAIL: 正对来球应朝 180°，实际 %.1f\n", a.aim_rot); return 1;
    }

    // ② 斜向球：v=(3,1.5) → 迎球朝向 = 球来向的反方向（不是"我跑过来的方向"）
    wm.ball.x = 100; wm.ball.y = 60; wm.ball.vx = 3.0; wm.ball.vy = 1.5;
    wm.home[1].x = 168; wm.home[1].y = 95;
    DefensePlan b = plan_defense(wm, 1);
    const double want = angle_to(0.0, 0.0, -3.0, -1.5);
    if (!b.face_incoming || fabs(angle_diff(b.aim_rot, want)) > 1e-6) {
        printf("FAIL: 斜向球迎球朝向错 face=%d aim=%.1f 应 %.1f\n",
               (int)b.face_incoming, b.aim_rot, want); return 1;
    }

    // ③ 球停着：不给朝向（方向是噪声），执行方走旧的"只给位置"逻辑
    wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[1].x = 168; wm.home[1].y = 90;
    DefensePlan c = plan_defense(wm, 1);
    if (c.face_incoming) { printf("FAIL: 球停着不该给迎球朝向\n"); return 1; }

    // ④ 执行侧：人已经站在断球点上、但机头朝反了 → 原地转正（左右轮反向、共模≈0，不产生位移）
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

// 第 103 轮：会合点 = 沿球未来轨迹找「我比球早到 lead 帧」的第一个点。
//   为什么不是"球停点"：真机标定球每帧只衰减 0.992~0.994，球几乎不会自己停下。
static int test_ball_meeting_point() {
    WorldModel wm;
    wm.ctx = TeamContext{true};
    wm.ball.valid = true;
    double mx = 0, my = 0, aim = 0;

    // ① 球停着 → 没有会合点（回退静态站位）
    wm.ball.x = 100; wm.ball.y = 90; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    if (ball_meeting_point(wm, 130, 90, 2.0, 6.0, mx, my, aim)) {
        printf("FAIL: 停球不该有会合点\n"); return 1;
    }

    // ② 慢球（低于 kMinBallSpeed）→ 方向不可信，不给会合点
    wm.ball.vx = 0.5;
    if (ball_meeting_point(wm, 130, 90, 2.0, 6.0, mx, my, aim)) {
        printf("FAIL: 慢球不该有会合点\n"); return 1;
    }

    // ③ 正向来球：球从 (100,90) 以 5cm/帧朝 +x 滚 → 会合点必在 y=90 的轨迹上、
    //    在球前方，且「我跑过去的时间 + 提前量」不晚于球到那里的时间。
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

    // ④ 站在场地另一头：追不上 → 不给会合点（避免为一个够不着的球失位）
    if (ball_meeting_point(wm, 20, 20, 2.0, 6.0, mx, my, aim)) {
        printf("FAIL: 追不上的球不该给会合点 (%.1f,%.1f)\n", mx, my); return 1;
    }

    // ⑤ 斜向球：会合点落在斜轨迹上，朝向 = 球来向的反方向
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

// 球-门连线护门点单测（docs 第14轮：参考官方 demo CenterDefender 思想）
static int test_goal_cover() {
    TeamContext ctx{true};   // 蓝队守右门 x=220
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;

    double cx = 0, cy = 0;

    // 分区1：球远（>100cm）→ 站「球→门心」连线、球向门方向 45cm
    //   球 (100,90) → 门 (220,90) 连线水平，护门点应 = (145,90)
    wm.ball.x = 100; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 远球应返回 true\n"); return 1; }
    if (fabs(cx - 145.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: 远球护门点 (%.1f,%.1f) 应 (145,90)\n", cx, cy); return 1;
    }

    // 分区2：球中近（45~100cm）→ 站门前 50cm 拦截线、y 跟球
    //   球 (160,100) 距门 60cm → 护门点 x=170, y=100
    wm.ball.x = 160; wm.ball.y = 100;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 中近球应返回 true\n"); return 1; }
    if (fabs(cx - 170.0) > 0.5 || fabs(cy - 100.0) > 0.5) {
        printf("FAIL: 中近护门点 (%.1f,%.1f) 应 (170,100)\n", cx, cy); return 1;
    }

    // 分区3：球贴门（<45cm）→ 站「球与门之间」球前 8cm 堵推射线（demo 中卫球门侧思想）
    //   但 2026-09-26 起：护门点落进**裁判门区**（门线内 15cm，+8 余量）→ 退到门线前 29cm，
    //   门口球交给门将（非门将进门区计数离开不清零，满 20 帧判点球；见 in_goal_area_rule）。
    //   球 (205,90) 距门 15cm → 原护门点 213 在门区内 → (191,90)
    wm.ball.x = 205; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 贴门球应返回 true\n"); return 1; }
    if (fabs(cx - 191.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: 贴门护门点 (%.1f,%.1f) 应 (191,90)（不进裁判门区）\n", cx, cy); return 1;
    }

    // 斜向：球 (150,130) 距门 ~72.1cm（中近分区）→ 门前 50 线 x=170, y=130 夹回 107.5
    wm.ball.x = 150; wm.ball.y = 130;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 斜向球应返回 true\n"); return 1; }
    if (fabs(cx - 170.0) > 0.5 || fabs(cy - 107.5) > 0.5) {
        printf("FAIL: 斜向护门点 (%.1f,%.1f) 应 (170,107.5)\n", cx, cy); return 1;
    }

    // 极贴门：球 (218,90) 距门 2cm → 原 clamp 到 217 仍在裁判门区 → 同样退到 (191,90)
    wm.ball.x = 218; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 极贴门球应返回 true\n"); return 1; }
    if (fabs(cx - 191.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: 极贴门护门点 (%.1f,%.1f) 应 (191,90)（不进裁判门区）\n", cx, cy); return 1;
    }

    printf("goal cover: OK (远球连线/中近拦截/贴门堵射/斜向clamp)\n");
    return 0;
}

// 黄位镜像单测（2026-10-07 补）：同一函数在黄队口径下必须给出"镜像后的同一个答案"。
//   背景：goal_cover_point 里有一处用**绝对 x**判断"球是否比 door_x 更靠门"（`if (bx > door_x)`），
//   只对蓝队成立；黄队 door_x=3 时"任何 bx>3"都成立 ⇒ 护门点被钉死在 x=3、再被门区闸推到 29，
//   与球位无关（远球本该站到 75）。本用例是它的守卫，全部期望值 = 蓝位用例的 x 镜像（x' = 220 - x）。
static int test_goal_cover_yellow() {
    TeamContext ctx{false};   // 黄队守左门 x=0
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;

    double cx = 0, cy = 0;

    // 分区1 远球：球 (120,90)（离门 120cm）→ 站「球→门心」连线、球向门方向 45cm = (75,90)
    //   ★ 这一条就是那个 bug 的守卫：错版本会给 (29,90)
    wm.ball.x = 120; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 远球应返回 true\n"); return 1; }
    if (fabs(cx - 75.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: [黄] 远球护门点 (%.1f,%.1f) 应 (75,90)（= 镜像蓝队 145）\n", cx, cy); return 1;
    }

    // 分区2 中近球：球 (60,100)（离门 60cm）→ 门前 50cm 拦截线 = (50,100)
    wm.ball.x = 60; wm.ball.y = 100;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 中近球应返回 true\n"); return 1; }
    if (fabs(cx - 50.0) > 0.5 || fabs(cy - 100.0) > 0.5) {
        printf("FAIL: [黄] 中近护门点 (%.1f,%.1f) 应 (50,100)（= 镜像蓝队 170）\n", cx, cy); return 1;
    }

    // 分区3 贴门球：球 (15,90)（离门 15cm）→ 护门点在裁判门区内 → 退到门线前 29cm = (29,90)
    wm.ball.x = 15; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 贴门球应返回 true\n"); return 1; }
    if (fabs(cx - 29.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: [黄] 贴门护门点 (%.1f,%.1f) 应 (29,90)（= 镜像蓝队 191）\n", cx, cy); return 1;
    }

    // 斜向：球 (70,130)（离门 ~80cm，中近分区）→ 门前 50 线 x=50, y 夹回 107.5
    wm.ball.x = 70; wm.ball.y = 130;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 斜向球应返回 true\n"); return 1; }
    if (fabs(cx - 50.0) > 0.5 || fabs(cy - 107.5) > 0.5) {
        printf("FAIL: [黄] 斜向护门点 (%.1f,%.1f) 应 (50,107.5)（= 镜像蓝队 170）\n", cx, cy); return 1;
    }

    // 极贴门：球 (2,90)（离门 2cm）→ 同样退到 (29,90)
    wm.ball.x = 2; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: [黄] 极贴门球应返回 true\n"); return 1; }
    if (fabs(cx - 29.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: [黄] 极贴门护门点 (%.1f,%.1f) 应 (29,90)（= 镜像蓝队 191）\n", cx, cy); return 1;
    }

    // 分区3 深且偏（y 远离门框）：球 (30,130)（离门 30cm，贴门分区；y=130 在裁判门区 y 带 [59,121] 之外）
    //   → 挡射线点 = 球朝门 8cm = (25.2,123.6)，且**不该**被门区闸改成 29。
    //   ★ 这一条才是那个 bug 的真正守卫：错版本（用绝对 x 判断）会给 x=3，再被 kMinX=12 抬到 12
    //     ⇒ 比正确答案浅 13.2cm，且位置与球位无关（球在 (30,130) 却站到门线侧后）。
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

    // 初始：防守态、未持球
    wm.team_state = TS_DEFENSE; wm.possession_frames = 0; wm.no_possession_frames = 0;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 10; wm.home[i].y = 90; wm.opp[i].x = 150; wm.opp[i].y = 90; }
    strat.run(wm);
    if (wm.team_state != TS_DEFENSE) { printf("FAIL: 初始应防守态\n"); return 1; }

    // 我方移到球附近（持球），前 2 帧未达滞回阈值 → 仍防守态
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 150; wm.home[i].y = 90; wm.opp[i].x = 10; wm.opp[i].y = 90; }
    strat.run(wm);
    strat.run(wm);
    if (wm.team_state != TS_DEFENSE) { printf("FAIL: 持球 2 帧不应切进攻（滞回）\n"); return 1; }

    // 第 3 帧达阈值 → 进攻态 + 低威胁
    strat.run(wm);
    if (wm.team_state != TS_ATTACK) { printf("FAIL: 连续持球 3 帧应切进攻态\n"); return 1; }
    if (wm.threat_level != 0.1) { printf("FAIL: 进攻态威胁应为 0.1 got %.2f\n", wm.threat_level); return 1; }

    // 丢球：对手移到球附近，连续 3 帧失球 → 回防守态 + 蓝半场威胁 0.6
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

    // 场景①：同等球门距离、同等传球距离下，接应点有对手 → 应被威胁惩罚、落选。
    // A 接应点(52,69) 旁 20cm 放对手(threat=1)，B 接应点(52,111) 无对手(threat=0)。
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

    // 场景②：接应点越过边界 → 夹回场内，坐标不越界。
    // home[1] 在 x=2，领球偏移 -6 后原始 x=-4，应夹到 FIELD_MARGIN=6。
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

    // 场景③：同等威胁、同等球门距离 → 优先短传。
    // A 接应点(54,70) pass_dist=32.8，B 接应点(54,50) pass_dist=47.7，短传 A 应胜出。
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

    // 场景④：接应点基准联动站位点（assist_pt），不依赖队友本体坐标。
    // ASSIST(home[2]) 本体在 (150,90)（距持球者>60，若用本体则不可选），
    // 但其站位点 assist_pt=(60,70) 在传球距离内 → 应基于站位点选出接应点(54,70)。
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
        // 接应点 = 站位点朝进攻方向前移 pass.OFFSET_BASE（原来是写死的 6cm）
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

    // 场景1：带球者高速前突 + 侧前方接应者 → 守门员后退封门（不崩溃）
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 100 + i * 5; wm.opp[i].y = 90; }
    wm.ball.x = 150; wm.ball.y = 90; wm.ball.vx = 8.0; wm.ball.vy = 0.0;
    wm.opp[0].x = 149; wm.opp[0].y = 90;   // 带球者（离球最近）
    wm.opp[1].x = 165; wm.opp[1].y = 80;   // 接应者（更靠门 + 够近）
    run_goalie(wm, 0);
    if (fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr)) > 300.0) {
        printf("FAIL: 二过一场景轮速异常\n"); return 1;
    }

    // 场景2：远射（球快速朝门、对方无人接应）→ 前压拦截（不崩溃）
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 10 + i * 10; wm.opp[i].y = 90; }
    wm.ball.x = 120; wm.ball.y = 90; wm.ball.vx = 10.0; wm.ball.vy = 0.0;
    run_goalie(wm, 0);
    if (fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr)) > 300.0) {
        printf("FAIL: 远射场景轮速异常\n"); return 1;
    }

    // 场景3：慢球朝门滚（无对方埋伏）→ 贴门不出击（深度≈门线，不冲出去）
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 10 + i * 10; wm.opp[i].y = 90; }
    wm.ball.x = 120; wm.ball.y = 90; wm.ball.vx = 3.0; wm.ball.vy = 0.0;
    run_goalie(wm, 0);
    if (fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr)) > 300.0) {
        printf("FAIL: 慢球场景轮速异常\n"); return 1;
    }

    // 场景4：门前有对方埋伏（罚球区内）+ 快球朝门 → 回缩不出太远（不崩溃）
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 10 + i * 10; wm.opp[i].y = 90; }
    wm.opp[0].x = 170; wm.opp[0].y = 90;   // 埋伏在罚球区内
    wm.ball.x = 120; wm.ball.y = 90; wm.ball.vx = 10.0; wm.ball.vy = 0.0;
    run_goalie(wm, 0);
    if (fmax(fabs(wm.home[0].vl), fabs(wm.home[0].vr)) > 300.0) {
        printf("FAIL: 埋伏回缩场景轮速异常\n"); return 1;
    }

    // 场景5：球在门将脚下（很近）→ 主动解围（不崩溃，且往队友方向清）
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

// ============================================================
// 2026-10-06 真机乌龙三修（docs/06 同日条目）
//   ① 对方罚点球：球静止在我方罚球点 → 门将守门线中央，不去"开门球"
//   ② 门球慢爬（0.31cm/帧背离己门）→ 仍按静止门球处理；朝门慢滚的活球不受影响
//   ③ 贴线站定：门将在门侧但没挡在进门点 → 沿 y 横移补位，不再原地目送
// ============================================================
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
    // ① 点球：门将已在门线中央（217,90）→ 原地守住；旋钮关 → 旧行为会被 restart_kick 拉走
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

    // ② 门球慢爬：背离己门 0.31cm/帧 的门球输出必须与静止门球一致；朝门滚的不一致
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

    // ③ 贴线：球 (216,80) 朝门慢爬（朝门 0.17cm/帧，低于 line_block 的 0.2、高于静止 0.2 合速），
    //   门将 (219,92) 在门侧但离进门点 y≈82 有 10cm → 必须动起来
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

// 传球威胁距离加权单测：半径内近/远敌人惩罚不同（新逻辑）
static int test_pass_threat_weight() {
    TeamContext ctx{true};               // 蓝队：门 x=220，攻向左(对方门 x=0)
    WorldModel wm;
    wm.ctx = ctx;
    // role 保持默认(全 GOALIE=0) → plan_pass 走 default 分支用本体坐标
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

// 无球接应拉开单测：站位点附近有敌人时沿 Y 轴横向躲开（run_assist/run_midfield）
// 验证思路：让机器人朝向 +x(rot=0)，站位点在正前方同 y。
//   motion::position 只写 vl/vr，但目标方向会反映到 desired_angle→te→vl/vr 差：
//   · 无敌人 → 目标=站位点，直走(vl≈vr)
//   · 敌人在站位点上方 → spread_y 让目标往下偏 → 左转(vl>vr)
//   · 敌人在站位点下方 → 目标往上偏 → 右转(vl<vr)
static int test_roles_spread() {
    TeamContext ctx{true};               // 蓝队：门 x=220，攻向左
    WorldModel wm;
    wm.ctx = ctx;
    wm.threat_level = 0.1;               // <=0.3 走进攻分支
    wm.game_state = PM_PlaceKick_Blue;   // 死球期：第 79 轮分道压迫不接管，这里只测站位微调
    wm.live_play = false;                // 第 88 轮：压迫闸门认 live_play

    // 把无关对手放到远处（离站位点 > 威胁半径 30cm），避免干扰最近敌人判断
    auto scatter = [&]() {
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 25; }
    };

    // —— 场景1：无敌人 → 不偏移，直走 ——
    scatter();
    wm.home[1].x = 100; wm.home[1].y = 90; wm.home[1].rot = 0;
    wm.assist_x = 140; wm.assist_y = 90;
    wm.mid_y = 150;                          // 队友错开，避免触发队友回避
    run_assist(wm, 1);
    if (fabs(wm.home[1].vl - wm.home[1].vr) > 0.5 || wm.home[1].vl <= 0.0) {
        printf("FAIL: 无敌人应直走 (vl=%.1f vr=%.1f)\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    // —— 场景2：敌人在站位点上方 → 目标往下偏，左转(vl>vr) ——
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

    // —— 场景3：敌人在站位点下方 → 目标往上偏，右转(vl<vr) ——
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

    // —— 场景4：run_midfield 进攻分支同样拉开 ——
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

    // —— 场景5：站位点贴边 + 敌人往界外推 → clamp 不越界 ——
    //   assist_y=2、敌人在上方会算出 y≈-11，应被 clamp 回 FIELD_MARGIN=6。
    //   机器人放在 y=6：clamp 生效 → 目标(140,6) 与机器人同水平线 → 直走(vl≈vr)；
    //   clamp 失效 → 目标(140,-11) → 明显下偏(te≈-23°)，vl≫vr。
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

    // —— 场景6：队友回避——assist 与 mid 站位点 Y 过近 → assist 沿 Y 推开 ——
    scatter();
    wm.home[1].x = 100; wm.home[1].y = 90; wm.home[1].rot = 0;
    wm.assist_x = 140; wm.assist_y = 90;
    wm.mid_y = 100;                         // |90-100|=10 < 25，触发队友回避
    run_assist(wm, 1);
    // 无敌人(spread_y 返回 90)，队友回避把 ty 推到 mid_y-25=75（目标在 mid 下方）
    // 目标(140,75)，desired_angle<0 → 左转(vl>vr)
    if (!(wm.home[1].vl > wm.home[1].vr + 1.0)) {
        printf("FAIL: assist 队友过近应往下推(左转) got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    // —— 场景7：队友回避——midfield 对称推开 ——
    scatter();
    wm.home[1].x = 100; wm.home[1].y = 90; wm.home[1].rot = 0;
    wm.mid_x = 140; wm.mid_y = 90;
    wm.assist_y = 80;                       // |90-80|=10 < 25，触发队友回避
    run_midfield(wm, 1);
    // 无敌人(spread_y 返回 90)，队友回避把 ty 推到 assist_y+25=105（目标在 assist 上方）
    // 目标(140,105)，desired_angle>0 → 右转(vl<vr)
    if (!(wm.home[1].vl < wm.home[1].vr - 1.0)) {
        printf("FAIL: midfield 队友过近应往上推(右转) got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    printf("roles spread: OK (无敌人直走/上方下躲/下方上躲/midfield拉开/贴边clamp/队友回避)\n");
    return 0;
}

// 射门方案单测（docs/06 第 11 轮：两段式推射配套）：
//   dir 单位向量、指向对方球门、推球点=球后 8cm、开口选 GK 远侧
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
        // 新增字段（docs/18 §8）：瞄准角必须与 dir 一致（执行侧 position_aligned 靠它）
        if (fabs(angle_diff(p.aim_rot, angle_to(0, 0, p.dir_x, p.dir_y))) > 1e-6) {
            printf("FAIL: aim_rot 与 dir 不一致 %.2f vs %.2f\n",
                   p.aim_rot, angle_to(0, 0, p.dir_x, p.dir_y)); return 1;
        }
        if (fabs(p.shot_dist - 25.0) > 0.6) { printf("FAIL: shot_dist=%.1f\n", p.shot_dist); return 1; }
        if (p.aim_y < goal_y_low() - 0.1 || p.aim_y > goal_y_high() + 0.1) {
            printf("FAIL: 瞄准点出框 aim_y=%.1f\n", p.aim_y); return 1;
        }
    }
    // GK 偏上(y=105) → 连续开口中心应落在下半区（不再是固定 74，docs/18 §8 改为连续瞄准）
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
    // —— 远射档（70~110cm）：**已开启**（用户 2026-09-11 决定真机观查；sim A/B 反对，
    //   数据见 docs/06 第 47 轮）。闸门 = 净开口 ≥8° 且 路线无遮挡（docs/03 的 0:3 教训）——
    wm.ball.x = 100; wm.opp[0].y = 90;   // 100cm + GK 封中：净开口 ≈ 2·(11.3°−4.6°) ≈ 6.7° < 8°
    for (int i = 1; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 20; }
    {
        ShootPlan p4 = plan_shoot(wm, 1);
        // 第 79 轮放宽借墙闸门后：直线远射仍须被拒，但允许借墙方案接管
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
        // 射门线 12cm 处横一个非门将防守者 → 直线路线被挡
        //   ⚠️ 2026-09-30 修正一处**测试自身的 bug**：原实现用 `p5.dir_x/dir_y` 放阻挡者，
        //   但本场景（100cm + 球静止）里 p5 **可能是借墙方案**——此时 `dir` 是
        //   "球→墙面"的**入射方向**，不是直线瞄准方向。照它摆，阻挡者被放到墙那一侧，
        //   直线路线其实**没被挡**（lane_blocked=0），后面的断言就失去了意义。
        //   （旧系数 0.66 时 p6 恰好又选中借墙，把这条 bug 掩盖住了；系数修正后暴露。）
        //   改为显式用「球→对方门心」方向摆放：本场景瞄准线只偏离门心方向约 3°，
        //   12cm 处横向偏差 <1cm ≪ kLaneBlockR(8cm) ⇒ 必然挡住瞄准线，断言才真正成立。
        {
            double dgx = ctx.opp_goal_x() - wm.ball.x, dgy = 90.0 - wm.ball.y;
            double dn = std::hypot(dgx, dgy);
            if (dn > 1e-6) { dgx /= dn; dgy /= dn; }
            wm.opp[1].x = wm.ball.x + dgx * 12.0;
            wm.opp[1].y = wm.ball.y + dgy * 12.0;
        }
        ShootPlan p6 = plan_shoot(wm, 1);
        // docs/06 第 65 轮起：直线被封后**允许改走借墙**（借墙是"换个角度"，不是硬射被挡的直线）
        //   所以判据改成：要么不射，要么必须是借墙方案（且质量过阈值）——不许沿被挡的直线硬射。
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
    // —— ≤70cm 无条件可射：即便门框被 5 个防守者完全封死 ——
    wm.ball.x = 60;
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 2; wm.opp[i].y = 74.0 + i * 8.0; }
    {
        ShootPlan p7 = plan_shoot(wm, 1);
        if (!p7.viable) { printf("FAIL: ≤70cm 应无条件可射（A/B 校准的主力区）\n"); return 1; }
    }
    // —— 点球旁路：罚球点距门 92cm，净开口只有 ~7.3°、球静止 quality 低 ——
    //    没有旁路就恒不可行（docs/03：点球 0/3 的第二个根因）
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
        // 非点球时：92cm 在远射档内，GK 封死直线 → **直线必须被闸门拒**。
        //   ⚠️ 2026-09-30 修正：原判据是 `if (pn.viable)`——但第 65 轮起**借墙可以接管**
        //   （借墙是"换个角度"，不是硬射被挡的直线），所以那个判据从第 65 轮起就过强了：
        //   旧系数 0.66 时本场景借墙质量 ~0.419 < kBankMinQ(0.42) 恰好没接管，判据"碰巧"通过；
        //   本轮把借墙系数改成真机实测的 0.45 后，弹点几何变化使 q≈0.4255 ≥ 0.42 → 借墙接管 →
        //   判据失效。改为与 test_shoot_plan 的路线阻挡用例同一口径（docs/06 第 65 轮）：
        //   **要么不射、要么必须是借墙方案**，不许沿被门将封死的直线硬射。
        if (pn.viable && !pn.bank) {
            printf("FAIL: 非点球时 GK 封死的 92cm 不该沿直线硬射 (bank=%d open=%.2f)\n",
                   (int)pn.bank, pn.open_angle);
            return 1;
        }
    }
    printf("shoot plan: OK (近距无条件/dir单位/连续瞄准/远射双闸门/路线阻挡/点球旁路)\n");
    return 0;
}

// ============================================================
// 借墙射门（bank shot，docs/06 第 65 轮）
//   验证四件事：
//   ① 直线被门将封死时，自动改走借墙，且机会质量过阈值（≥0.45）
//   ② 反射点满足**实测各向异性反射**（法向×shoot.kBankWallRest、切向×shoot.kBankWallFric；
//      2026-09-30 起法向默认 **0.45**，见 docs/06 第 75 轮补记），且与"理想镜面"解**明显不同**
//      （镜面解偏 3cm 以上 → 证明系数真的生效了，不是白写）
//   ③ 直线好机会不被抢（≤70cm 无条件可射区仍然返回直线方案）
//   ④ 球贴门线（无借墙几何可用）时不硬凑；门线另一侧（黄队 ctx）同样成立
// ============================================================
static int test_bank_shot() {
    // —— ① + ②：蓝队（守 x=220、攻 x=0），球在 (100,55)，门将站在球门中心线上封死直线 ——
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
        // 出射方向必须指向瞄准点：用**借墙旋钮的当前值**算 out，再与「反弹点→目标」做叉积（应共线）
        //   按旋钮取值而非写死 0.45/0.81 —— 否则自动调参一改旋钮，测试就从"守住行为"变成"禁止调参"
        //   （tunable.hpp:60-63 明确推荐这个写法）。
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
        // 与"理想镜面"解对比：必须差 3cm 以上，否则说明实测系数没接进去
        double c1m = p.aim_y - p.bank_wall, c2m = p.bank_wall - wm.ball.y;
        double rx_mirror = (c1m * wm.ball.x - c2m * ctx.opp_goal_x()) / (c1m - c2m);
        if (fabs(rx_mirror - p.bounce_x) < 3.0) {
            printf("FAIL: 镜面解(%.2f)与实测解(%.2f)几乎相同 → 系数没生效\n",
                   rx_mirror, p.bounce_x);
            return 1;
        }
        // 反弹点必须落在球与对方门之间、离门线 ≥12cm
        if (p.bounce_x >= wm.ball.x || p.bounce_x <= ctx.opp_goal_x()) {
            printf("FAIL: 反弹点位置不对 %.2f\n", p.bounce_x);
            return 1;
        }
    }
    // —— ③：≤70cm 无条件可射区，不允许被借墙方案抢走 ——
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
    // —— ④a：球几乎贴门线（x=3 < kMinShot=5）→ 无借墙几何，也不该硬凑 ——
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
    // —— ④b：黄队 ctx（守 x=0、攻 x=220），球在上半场 → 应借顶墙 ——
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

// 借墙测试档单测（docs/06 第 96 轮：用户指令「真机上直接测借墙射门，一个队员就够」）：
//   测试档 = shoot.kBankForceTest=1.0。默认 0.0（生产），只有平台入口 dll_blue.cpp 会打开它；
//   这里显式打开后守三件事：
//   ① 直线本来是好机会，测试档也改走借墙 —— 这才是"强制出样本"（自然对局 175 场只有 2 个借墙进球）
//   ② 门前 ≤70cm 无条件射区**仍然**走直线 —— 测试档不许吃掉最稳的进球区
//   ③ 测试档下 plan_bank_carry 恒不可行 —— 借墙只由主攻一个人发起（用户要的"一个队员"）
//   用 save/restore 而非 reset_params()：后者会清掉 --params 注入（见 test_shoot_push_limit 的坑）。
static int test_bank_force_test() {
    struct ForceRestore {
        double v;
        ~ForceRestore() { set_param("shoot.kBankForceTest", v); }
    } force_restore{get_param("shoot.kBankForceTest", 0.0)};

    // ① 直线是好机会（门将站在射门线外）→ 门槛版走直线、测试档改借墙
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
    // ② 门前 ≤70cm（无条件可射区）：测试档也不许被借墙抢走
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

// ACTIVE 门区停留时限单测（docs/13 方案 C）：
//   超限(kActiveGaLimit=15)后即使球在门区外也应撤出，而不是继续射门/带球
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
    // 撤退目标 = ogx - ad*60 = 60 → 机器人(25,90) 应朝 +x 移动（vl≈vr>0）
    if (!(wm.home[1].vl > 0.0 && wm.home[1].vr > 0.0) ||
        fmax(fabs(wm.home[1].vl), fabs(wm.home[1].vr)) > 300.0) {
        printf("FAIL: 超限应撤出门区(+x) got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }
    printf("active GA limit: OK (超限撤出门区)\n");
    return 0;
}

// 禁止推球区几何单测（docs/06 第 49 轮）：四角对称 + 边界半径正确
static int test_no_push_zone() {
    // 四角距离为 0
    const double corners[4][2] = {{0, 0}, {220, 0}, {0, 180}, {220, 180}};
    for (auto &c : corners)
        if (fabs(dist_to_corner(c[0], c[1])) > 1e-9) {
            printf("FAIL: 角点 (%.0f,%.0f) 距离应为 0 got %.3f\n",
                   c[0], c[1], dist_to_corner(c[0], c[1]));
            return 1;
        }
    // 场心最远
    if (fabs(dist_to_corner(110.0, 90.0) - std::hypot(110.0, 90.0)) > 1e-9) {
        printf("FAIL: 场心距离错 %.3f\n", dist_to_corner(110.0, 90.0));
        return 1;
    }
    // 半径边界：28.3cm < 35 → 禁区内；39.6cm > 35 → 不在
    if (!in_no_push_zone(20.0, 20.0))  { printf("FAIL: (20,20) 应在禁区内(28.3<35)\n"); return 1; }
    if (in_no_push_zone(28.0, 28.0))   { printf("FAIL: (28,28) 不应在禁区内(39.6>35)\n"); return 1; }
    // 四角对称
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

// 角区救球单测（docs/06 第 11 轮）→ docs/06 第 49 轮修订：
//   新规则 = 球在「禁止推球区（角落黄区，半径 kCornerNoPushR=35cm）」内 **一律不碰球**，
//   只停住（本平台所有让球动的动作都是推球；角区推球 = 犯规，每 4 次 +1 球）。
//   球在 35cm 之外、且卡住 >30 帧 → 仍按原两段式救球（推向场心）。
static int test_active_corner_rescue() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    // —— ① 球在 35cm 之外（距角 39.6cm）但仍在"卡球计数区"(30×30 角框)内 → 合法救球 ——
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
    // —— ②③ 球在禁止推球区内 / 平台处于死球期 → 第 49 轮守卫**开启时**才要求"只停不动" ——
    //   守卫总开关见 roles.hpp 的 kNoPushGuardEnabled（2026-09-12 落库时按"连胜版行为"关闭，
    //   所以这两条断言只在开关为 true 时校验）。
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
        // —— ③ 平台处于死球/重启期（非 PlayOn）→ 同样不许推（任何球位） ——
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
    // —— ④ 球不在角区 → 不触发救球（走正常逻辑，不崩溃） ——
    wm.ball.x = 60; wm.ball.y = 90;
    wm.home[1].x = 60; wm.home[1].y = 60;
    wm.corner_ball_frames = 0;
    run_active(wm, 1);
    printf("active corner rescue: OK (35cm 外才救/角区内停住/死球期停住/非角区不触发)\n");
    return 0;
}

static int test_segment_circle() {
    CircleObstacle o[2], hit;
    // 相离 → 不挡
    o[0] = {10, 10, 3};
    if (!segment_clear_of_circles(0, 0, 10, 0, o, 1)) { printf("FAIL: 相离圆不应挡\n"); return 1; }
    // 相交（圆压在线段上）→ 挡
    o[0] = {5, 0, 3};
    if (segment_clear_of_circles(0, 0, 10, 0, o, 1)) { printf("FAIL: 相交圆应挡\n"); return 1; }
    // 端点在圆内 → 挡
    o[0] = {1, 0, 3};
    if (segment_clear_of_circles(0, 0, 10, 0, o, 1)) { printf("FAIL: 端点在圆内应挡\n"); return 1; }
    // 相切（圆心距线段 == r）→ 挡
    o[0] = {5, 3, 3};
    if (segment_clear_of_circles(0, 0, 10, 0, o, 1)) { printf("FAIL: 相切应挡\n"); return 1; }
    // 多圆：最近挡路者命中更深的那个
    o[0] = {5, 3, 2};    // 距线段 3 > 2：不挡
    o[1] = {6, 0, 4};    // 挡（穿透 4）
    if (segment_clear_of_circles(0, 0, 10, 0, o, 2, &hit)) { printf("FAIL: 应检测到挡路\n"); return 1; }
    if (fabs(hit.x - 6) > 1e-9 || fabs(hit.r - 4) > 1e-9) { printf("FAIL: hit 应指向挡路圆\n"); return 1; }
    // 空障碍数组 → 恒不挡
    if (!segment_clear_of_circles(0, 0, 10, 0, nullptr, 0)) { printf("FAIL: 无障应恒通\n"); return 1; }
    printf("segment circle: OK (相离/相交/端点在内/相切/多圆取最深/空数组)\n");
    return 0;
}

// ============================================================
// docs/15 P0-1：路径规划单测 —— 直线最优（不绕路）
// ============================================================
static int test_route_straight() {
    RoutePlan p = plan_route(50, 90, 150, 90, nullptr, 0);
    if (!p.found || p.n_wp != 2) { printf("FAIL: 无障应直线两点\n"); return 1; }
    if (fabs(p.length - 100.0) > 0.01) { printf("FAIL: 直线长度 %.1f\n", p.length); return 1; }
    // 有障碍但不挡 S-T 直线 → 仍走直线（最优 = 不绕远路）
    CircleObstacle o = {50, 150, 10};
    p = plan_route(50, 90, 150, 90, &o, 1);
    if (!p.found || p.n_wp != 2) { printf("FAIL: 不挡路的圆不应触发绕行\n"); return 1; }
    printf("route straight: OK (无障直线/远处障碍不绕路)\n");
    return 0;
}

// ============================================================
// docs/15 P0-1：路径规划单测 —— 单圆盘绕行（最优性：绕行且不穿盘）
// ============================================================
static int test_route_avoid() {
    CircleObstacle o = {100, 90, 10};        // 挡在 S(50,90)→T(150,90) 正中间
    RoutePlan p = plan_route(50, 90, 150, 90, &o, 1);
    if (!p.found) { printf("FAIL: 单圆盘应可绕行\n"); return 1; }
    // 路径逐段粗采样：离圆心的最小距离 > 8（r=10 已 inflate，留 2cm 判定余量）
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
    // docs/18 顺手优化 ⑤：把 margin 语义写成断言 —— 路径允许侵入膨胀圈(r=10)最多 margin(0.5)，
    //   即 min_d ≥ 9.5。以后有人把 roles.cpp 的 kRouteInflate 调小而忘了 margin 就会在此暴露。
    if (min_d < 9.5) {
        printf("FAIL: 路径侵入膨胀圈超过 margin(0.5) min_d=%.2f\n", min_d);
        return 1;
    }
    // 最优性：真最优 ≈ 2·√(48²+9.8²) + 弦 4.0 ≈ 102.0
    //   docs/18 顺手优化 ④：原界 (100,200] 太松，路径退化 90% 也察觉不到 → 收紧到 (100,110)
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

// ============================================================
// docs/18 顺手优化 ①③：全局安全契约 —— plan_route 返回的路径
//   绝不违反**任何**障碍（含未入图的远圆），否则必须 found=false。
//   这是"邻域裁剪 + 违例驱动补算"的正确性锁：裁剪会漏掉远圆，补算负责找回来。
// ============================================================
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
    // ① 墙体 + 墙外远圆：远圆距 S→T 线段 38cm > 2r，会被邻域裁剪漏掉，
    //    但绕墙外沿的路径可能擦到它 → 补算必须兜住
    {
        CircleObstacle wall[4] = {{60, 90, 10}, {80, 90, 10}, {100, 90, 10}, {100, 128, 12}};
        RoutePlan p = plan_route(30, 90, 130, 90, wall, 4);
        if (route_hits_any(p, wall, 4)) { printf("FAIL: 墙体+远圆场景路径穿盘\n"); return 1; }
    }
    // ② 起终点被围死 → 必须 found=false（绝不能给穿盘路径）
    {
        CircleObstacle cage[4] = {{100, 90, 10}, {100, 110, 10}, {100, 70, 10}, {120, 90, 10}};
        RoutePlan p = plan_route(100, 90, 60, 90, cage, 4);
        if (route_hits_any(p, cage, 4)) { printf("FAIL: 围死场景给了穿盘路径\n"); return 1; }
    }
    // ③ 固定种子随机扫描 200 组（5 圆、半径 6~18、S/T 随机）
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
    // ④ 耗时上限（防以后有人去掉邻域裁剪让每帧变慢）：最坏配置 5 圆全入图
    {
        CircleObstacle obs[5] = {{100, 90, 10}, {90, 120, 10}, {110, 60, 10}, {130, 90, 10}, {70, 90, 10}};
        auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < 2000; ++i) plan_route(30, 90, 190, 90, obs, 5);
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::steady_clock::now() - t0).count();
        printf("  plan_route 最坏配置(5 圆全入图) 平均 %.1f us/次\n", us / 2000.0);
        // 耗时红线（docs/23）：40Hz 一帧 25ms，路径规划每帧最多几次调用，
        //   平均 >400µs 说明有人在网格里做了傻事（比如每格都建可见图）。
        if (us / 2000.0 > 400.0) { printf("FAIL: plan_route 平均耗时超标\n"); return 1; }
    }
    printf("route global safety: OK (固定场景/围死回退/200 组随机无穿盘)\n");
    return 0;
}

// ============================================================
// docs/15 P0-1：路径规划单测 —— 错位栅栏可通 / 围死回退 / 起点圆内
// ============================================================
static int test_route_cluster() {
    // 错位栅栏（三圆不重叠 r=10，绕行需走圆-圆外公切线）→ 应有安全通路
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
    // 竖向密排夹道（相切圆列）：路径不穿盘（可能绕不过 → found=false 回退也算过）
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
    // S 在障碍内 → found=false（调用方回退直线，防 NAN）
    CircleObstacle o = {100, 90, 10};
    p = plan_route(100, 90, 150, 90, &o, 1);   // S == 圆心
    if (p.found) { printf("FAIL: 起点在圆内应不可规划\n"); return 1; }
    printf("route cluster: OK (错位栅栏可通/夹道不穿盘/起点圆内回退)\n");
    return 0;
}

// ============================================================
// docs/23：A* 路径规划单测 —— 与「解析最优解」对比 + 场地边界
//   单圆盘的最短路有闭式解：贴着圆边走 = S切线段 + 圆弧 + T切线段
//     len = √(dS²−r²) + √(dT²−r²) + r·Δθ（Δθ = 两侧切点半径夹角取小）
//   用它当尺子：A*（4cm 网格 + 视线拉直）不该比真最优差太多。
// ============================================================
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
    // ① 教科书用例：S(50,90)→T(150,90)，圆 (100,90) r=10 → 解析最优 ≈ 102.01
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
    // ② 固定种子随机 200 组单圆（圆心取在 S→T 上 ⇒ 直线必被挡），全部对比解析最优
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

    // ③ 场地边界：障碍贴下边线（圆心 y=12 r=10）→ 只能从上方绕，
    //    所有 waypoint 必须留在场内（旧可见图没有边界概念，允许贴线甚至出界）
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

// ============================================================
// docs/15 P0-2：motion::follow_route 逐段执行单测
// ============================================================
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
    // 机器人位于路径起点 S=(0,0) 的 8cm 内 → 自动切到段 1，目标 (60,0) 在 +x → vl≈vr>0
    motion::follow_route(r, rt, wp_next);
    if (wp_next != 1) { printf("FAIL: 起点处应切到段1 got %d\n", wp_next); return 1; }
    if (!(r.vl > 0.0 && r.vr > 0.0)) { printf("FAIL: 第1段应朝+x走 vl=%.1f vr=%.1f\n", r.vl, r.vr); return 1; }
    // 到达段1终点 (60,0)（1cm 内）→ 自动切段 2，目标 (120,0) 仍朝 +x
    r.x = 59;
    motion::follow_route(r, rt, wp_next);
    if (wp_next != 2) { printf("FAIL: 到达后应推进到段2 got %d\n", wp_next); return 1; }
    if (!(r.vl > 0.0 && r.vr > 0.0)) { printf("FAIL: 第2段应朝+x走\n"); return 1; }
    // 越过跳段：重置 wp_next=0 但机器人已在终点附近 → 应经中间段直接跳到末段
    wp_next = 0;
    r.x = 118;
    motion::follow_route(r, rt, wp_next);
    if (wp_next != 2) { printf("FAIL: 越过后应推进到段2 got %d\n", wp_next); return 1; }
    // 终点慢速接近
    if (!(r.vl > 0.0)) { printf("FAIL: 末段应继续朝终点\n"); return 1; }
    // found=false → 兜底停车
    RoutePlan bad;
    bad.found = false;
    motion::follow_route(r, bad, wp_next);
    if (fabs(r.vl) > 1e-6 || fabs(r.vr) > 1e-6) { printf("FAIL: found=false 应停车\n"); return 1; }
    printf("follow route: OK (分段推进/到达切段/越过跳段/不可规划停车)\n");
    return 0;
}

// ============================================================
// docs/15 P0-4：run_active 激进档位冒烟（防崩溃/NaN/轮速有界）
//   A: 70~110 动态区远射档（quality≥0.35 → 进入射门执行体）
//   B: 动态区 GK 封死（开口<8° → 拒绝远射，落围困带离）
//   C: 追球避障路径（直线被挡 → route 绕行，不穿圆盘）
// ============================================================

// 禁区变角推射计数单测（docs/17）+ 到点定向门禁（docs/18 §8）：
//   ① 未对准（机头偏 40°）→ **不许推球**，先原地转正（旧实现 40° 就推 = 射正率 8% 的机制）
//   ② 已对准（机头=瞄准线）→ 立刻推穿并计次
//   ③ 超 3 次不再推；④ 球离开射程计数清零
static int test_shoot_push_limit_impl();
// 第 90 轮起默认不等对准；此单测验证回滚档（kNoAlignWait=0）的对准门禁仍完好
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

    // ① 到点但机头偏 40° → 不推球，输出原地转正（vl/vr 反号）
    wm.home[1].x = prep_x; wm.home[1].y = prep_y;
    wm.home[1].rot = normalize_angle(sp.aim_rot + 40.0);
    wm.home[1].vl = wm.home[1].vr = 0;
    run_active(wm, 1);
    if (wm.shoot_push_count != 0) {
        printf("FAIL: 未对准(偏40°)不该推球 count=%d\n", wm.shoot_push_count);
        return 1;
    }
    // 未对准时应**就地转正**（vl/vr 反号、不前进）——docs/18 §8 启用版
    if (!(wm.home[1].vl * wm.home[1].vr < 0.0)) {
        printf("FAIL: 未对准应原地转正(vl/vr 反号) got vl=%.1f vr=%.1f\n",
               wm.home[1].vl, wm.home[1].vr);
        return 1;
    }

    // ② 到点且机头=瞄准线 → 推穿并计次
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
    // ③ 超 3 次 → 射门分支被跳过（count 不再增长）
    wm.shoot_push_count = 3; wm.shoot_push_cd = 0;
    wm.home[1].x = prep_x; wm.home[1].y = prep_y; wm.home[1].rot = sp.aim_rot;
    wm.home[1].vl = 0; wm.home[1].vr = 0;
    run_active(wm, 1);
    if (wm.shoot_push_count != 3) {
        printf("FAIL: 超限仍推球 count=%d\n", wm.shoot_push_count);
        return 1;
    }
    // ④ 球离开射程 → 计数清零（下一轮进攻重新计）
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

// ============================================================
// docs/18 P1：运动控制 —— 制动包线 / 停点收敛 / 语义开关 / 数值边界
// ============================================================
// 平台动力学近似（真机标定，docs/18 §1；标定工具 tools/py/motion_calib.py）：
//   v = (vl+vr)/2 · 1.0 cm/s （1 轮速单位 ≈ 1 cm/s：命令饱和 112.5 ↔ 实测稳态 p90 115.3）
//   ω = (vr−vl)/10 rad/s     （轮距 10cm，同 sim_bench）
//   轮速变化率受限 |Δwheel| ≤ a_wheel·dt；两轮同向时线性加减速度 = a_wheel
//   a_wheel 取真机实测减速 p95 = 658 cm/s² —— 比控制律设计值 kBrakeAccel=400 更"能刹"，
//   即控制律偏保守：这样都冲过目标就说明包线失效（测试是有意偏向"对旧律有利"的物理）。
struct PlantRobot { double x = 0, y = 0, rot = 0, vl = 0, vr = 0; };

// k_scale：轮速单位 → cm/s（真机实测 ≈1.0；sim_bench 内部用 0.9 近似）
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

// 旧律副本（改 motion.cpp 之前的 position()，冻结作对照基线，勿随新律同步修改）
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

// 用同一套平台动力学跑一条"到点停住"的任务，返回统计量
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

// 1) 包线性质：停点任务的命令速度恒不超过 sqrt(2·a·(de−ε))；远距离不减速（巡航不受影响）
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
    // 死区内停车
    RobotState r;
    r.x = 0; r.y = 90; r.rot = 0;
    motion::position(r, motion::kStopEps - 0.01, 90, motion::TM_STOP);
    if (std::fabs(r.vl) > 1e-9 || std::fabs(r.vr) > 1e-9) {
        printf("FAIL: 死区内未停车 vl=%.2f vr=%.2f\n", r.vl, r.vr);
        return 1;
    }
    // 远距离（de=60）不受包线影响：仍为旧律饱和值 112.5（巡航不降速）
    r.x = 0; r.y = 90; r.rot = 0;
    motion::position(r, 60.0, 90, motion::TM_STOP);
    if ((r.vl + r.vr) * 0.5 < 112.0) {
        printf("FAIL: de=60 巡航被降速 %.1f（应保持 112.5）\n", (r.vl + r.vr) * 0.5);
        return 1;
    }
    // 包线开始起作用的距离 ~= v_max²/(2a)+ε ≈ 17.3cm：16cm 处应已被限制
    r.x = 0; r.y = 90; r.rot = 0;
    motion::position(r, 8.0, 90, motion::TM_STOP);
    if ((r.vl + r.vr) * 0.5 > 73.0) {          // sqrt(2*400*6.5)=72.1
        printf("FAIL: de=8 未按包线减速 %.1f\n", (r.vl + r.vr) * 0.5);
        return 1;
    }
    printf("motion brake envelope: OK (命令速度<=sqrt(2as)/死区停车/远距不降速)\n");
    return 0;
}

// 2) 停点收敛：新律在同一个平台上不过冲、能稳定；旧律必然冲过（对照论证）
// 2026-10-06：侧向目标"直线来回蹭"（真机 G2 B4 帧 1798~1850：球在车侧 7cm，车头 -63° 不变，
//   前进/倒车交替 50 帧不触球）。闭环：目标=球（经过型），车头 -63°、球在左后侧 7cm。
//   新口径（两轮等比缩放 + 侧向收油门）须在 40 帧内碰到球；旧口径复现来回蹭（作对照）。
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
    // 旧律：进入 2cm 后必然再冲出 ±3cm 带（极限环机制），这是本改动的立论依据
    if (o1.max_past <= 3.0) {
        printf("WARN: 旧律在标定物理下未过冲（max_past=%.1f）——docs/18 立论需复核\n", o1.max_past);
    }
    // 带初始朝向偏差（rot=25°）：仍须收敛
    StopTrace n2 = run_stop_task(120.0, 0.0, 25.0, true);
    if (n2.settle_d > 2.0 || n2.settle_f > 300) {
        printf("FAIL: 带 25° 初始偏差未收敛 err=%.2fcm last_out=%d\n", n2.settle_d, n2.settle_f);
        return 1;
    }
    // —— sim 平台口径（kSpeed=0.9、轮速上限 300 单位/s² → 线性 270 cm/s²，比真机弱）——
    // 包线按 kScale=1.0 设计（真机实测就是 1.0），在 sim 里会偏乐观：
    //   有效线性减速度 = kBrakeAccel·kScale² = 400·0.81 = 324 > 270 → 规划制动距离偏短
    //   理论残余过冲 ≈ v²/(2·270) − v²/(2·324) ≈ 3cm。这里把它量化并锁住上界，
    //   避免"sim 看起来没事"掩盖真实差距；真机（k≈1.0）反而更保守。
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

// 3) 语义开关：TM_PASS（追球/穿球）保持旧律行为——不被包线减速（否则抢点变慢）
static int test_motion_pass_mode() {
    RobotState r;
    r.x = 0; r.y = 90; r.rot = 0;
    motion::position(r, 5.0, 90, motion::TM_PASS);          // 第59轮用户指令提速：饱和 150（原 112.5）
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
    // chase_ball 内部走 TM_PASS：贴球减速逻辑保留
    RobotState c;
    c.x = 0; c.y = 90; c.rot = 0;
    BallState pred; pred.x = 2.0; pred.y = 90;
    motion::chase_ball(c, pred);
    if ((c.vl + c.vr) * 0.5 > 35.0) {                       // 2/10 缩放 → ~30（第59轮提速后）
        printf("FAIL: chase_ball 贴球未减速 %.1f\n", (c.vl + c.vr) * 0.5);
        return 1;
    }
    printf("motion pass mode: OK (TM_PASS 不降速/贴球减速保留)\n");
    return 0;
}

// 4) 数值边界：NaN/Inf 目标直接停车；任何输入下轮速有限且有界
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
    // 极端输入扫描：有限且 |v| ≤ kMaxWheel
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

// 5) 到点定向（docs/18 P2）：三段式 + 物理积分收敛到"位姿都对"
static int test_motion_aligned() {
    const double dt = 1.0 / 40.0;
    const double kPlantAccel = 658.0;      // 真机标定减速 p95
    // ① 已到位且已对准 → 立刻返回 true 且停车
    {
        RobotState r;
        r.x = 60; r.y = 0; r.rot = 180.0;
        bool ok = motion::position_aligned(r, 60.0, 0.0, 180.0);
        if (!ok || r.vl != 0.0 || r.vr != 0.0) {
            printf("FAIL: 已对准应返回 true 且停车 ok=%d vl=%.1f\n", (int)ok, r.vl);
            return 1;
        }
    }
    // ② 到位但机头偏 40° → 返回 false，且输出原地旋转（不前进）
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
    // ③ 远处 → 返回 false 且朝目标走（前进命令）
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
    // ④ 物理积分收敛 A：正常射门接近姿态（沿瞄准线从后方开过去，机头已朝瞄准方向）
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
    // ④ 物理积分收敛 B：最坏情况 = 到位后要掉头 180°（纯旋转，无位移漂移）
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

// 定位球摆位语义单测（2026-09-12 复盘 15:29 场：4 次点球、0 次射门）
//   平台约定：**PM_X_Kick = X 队主罚**（证据：官方 demo 的 SetBall 只在 PM_GoalKick_Yellow
//   时把球放黄队门区，而 demo 是黄队；demo 的 SetLaterRobots case 7=PM_PenaltyKick_Yellow
//   摆的是"黄队自己主罚"的阵型，case 8=PM_PenaltyKick_Blue 才是黄队防守）。
//   摆位顺序：开球/任意球/门球 = 主罚方先摆；点球 = 防守方先摆。
//   罚球人位置：平台 HELP 原文 "The kicker shall be placed behind the ball" → 站"球后"
//   = 远离被攻球门那一侧。站反了，机器人一推就把球顶回自己半场（实测帧 4087-4119）。
static int test_placement_semantics() {
    Robot r[5];
    Robot former[5] = {};
    TeamContext blue{true}, yellow{false};
    Vector3D ball;
    ball.z = 0;

    // —— 1. 我方主罚点球：罚球人必须站"球后"，不是站门前 ——
    //    实测罚球点 ≈ 门前 40cm（2026-09-12 rlg 帧 4087：球停在 (39.4, 89.8)）
    ball.x = 39.4; ball.y = 89.8;
    formation_later(blue, PM_PenaltyKick_Blue, former, ball, r);
    if (!(r[1].pos.x > ball.x + 4.0)) {
        printf("FAIL: 蓝队主罚点球，罚球人应站球后(x>%.1f)，实际 x=%.1f\n", ball.x + 4.0, r[1].pos.x);
        return 1;
    }
    if (!(r[0].pos.x > 200.0)) { printf("FAIL: 主罚点球时门将仍应守门(x>200)\n"); return 1; }
    // 黄队镜像：黄队攻右门(x=220)，罚球点在 x=180.6 → 罚球人要站在 x<180.6 一侧
    ball.x = 180.6; ball.y = 89.8;
    formation_later(yellow, PM_PenaltyKick_Yellow, former, ball, r);
    if (!(r[1].pos.x < ball.x - 4.0)) {
        printf("FAIL: 黄队主罚点球应站球后(x<%.1f)，实际 x=%.1f\n", ball.x - 4.0, r[1].pos.x);
        return 1;
    }

    // —— 2. 对方主罚点球（状态名=黄队）→ 我们(蓝)是防守方，平台要我们先摆 ——
    //    防守方要求：门将贴门线、其他人全在我方半场
    //    （平台 HELP："The robots shall be placed wholly on the other side of the half line"）
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
    // 黄队镜像：状态名=蓝队 → 黄队是防守方
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

    // —— 3. 开球：主罚方先摆，开球人必须在自己半场 ——
    //    实测帧 0 / 65.6 / 200.1：我们开球时 ACTIVE 被摆在 (95.7, 90.8) = 对方半场 + 球前面
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

// ============================================================
// 2026-10-05：摆位**规则纪律**属性测试（规则书 7.10.1/7.10.2/7.15/7.17）
//   ① 门区纪律：除 1 号守门员外，**己方门区(A = 门前 50cm × 门宽±15)里不许摆人**。
//      7.10.1 = 门区里"再保持一个机器人"停留 >20 连续周期判点球；7.10.2 = 除门将外
//      门区里 2 个以上直接判；7.17 = 发门球时**只有守门员允许在门区内**（明文摆位规则）。
//      field_info.hpp 还记着平台裁判**计数离开不清零** ⇒ 摆位阶段就先别把人放进去。
//      旧阵三处踩线：防守通用阵 1 号 (190,90)、开球阵 4 号 (185,90)、门球阵 1 号 (190,100)，
//      都落在门区里（x∈[170,220] 且 y∈[75,105]），进场第一帧才被
//      strategy.cpp 的 enforce_own_goal_area 顶出来 —— 摆位那一刻就是超规的。
//   ② 争球纪律（7.15）：每队 1 人放在"沿场地纵向离球 25cm"处，**其余机器人必须在
//      争球所在 1/4 场地之外**。旧阵把其余 3 台固定摆 (150,60)/(150,120)/(185,90)，
//      不看争球在哪个 1/4 区 —— 争球在右上区时 (150,120) 就落在区内。
//   本用例是**属性测试**：遍历 12 种 PlayMode × 蓝/黄 × 先摆/后摆，逐条断言。
// ============================================================
static int test_placement_rule_boxes() {
    TeamContext blue{true}, yellow{false};
    Robot r[5];
    Robot former[5] = {};
    for (int t = 0; t < 2; ++t) {
        const TeamContext ctx = t ? yellow : blue;
        const char *team = ctx.is_blue ? "蓝队" : "黄队";
        for (int gi = PM_FreeBall_LeftTop; gi <= PM_GoalKick_Blue; ++gi) {
            const PlayMode gs = (PlayMode)gi;
            // 平台在该状态会给的球位（先摆那侧接口没有球参数 → 与 formation.cpp 的估计同口径）
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

                // ① 己方门区：除 0 号守门员外任何人不得落在里面
                for (int i = 1; i < 5; ++i) {
                    if (in_goal_area(ctx, r[i].pos.x, r[i].pos.y)) {
                        printf("FAIL: %s %s gs=%d robot[%d] 摆在己方门区里 (%.1f,%.1f)\n",
                               team, who, gi, i, r[i].pos.x, r[i].pos.y);
                        return 1;
                    }
                }
                // ② 争球：争球人纵向 25cm + 其余 3 台在争球 1/4 场外
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
                        // 贴中线（|y-90|≤1cm）时"算不算区内"口径不明 → 一并判不合格（留余量）
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

// ============================================================
// docs/06 第 60 轮：门将开球（门球/清球）"先转正再推穿"单测
//   真机 09-13 09:59 场：门球卡 7.6 秒球不动，门将机头 -100°（该 180°）→
//   position 落进 (85°,95°) 纯自转死区 ⇒ 只蹭不推。断言两条：
//   ① 已对准 → 必须"直线推穿"（有速度）；② 机头偏 90° → 必须转正且 40 帧内收敛到 ±20°。
//   ⚠️ 第 72 轮起：球距门 <20cm 的贴门线静止球改走「直线推出」硬钳位（见
//      test_goalie_straight_clear），本测试把球摆在 30cm（20~45cm 侧面开球区），
//      继续覆盖侧面开球的"先转正再推穿"死区修复。
// ============================================================
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

    // 推球方向（与 run_goalie 同一打分）：本布局队友/对手都在正前方，侧面是空当 →
    //   打分偏向侧面；门将须沿此方向对准才推穿。
    double pdirx = 0.0, pdiry = 0.0;
    gk_restart_direction(wm, 0, wm.ball.x, wm.ball.y, pdirx, pdiry);
    double aim_rot = angle_to(0.0, 0.0, pdirx, pdiry);

    // ① 已对准（机头=推球方向、站在球后沿推球方向）+ 球在门前静止 → 必须有推穿速度（不是蹭）
    wm.home[0].x = wm.ball.x - pdirx * 10.0;   // 球后 10cm（沿推球方向）
    wm.home[0].y = wm.ball.y - pdiry * 10.0;
    wm.home[0].rot = aim_rot;
    run_goalie(wm, 0);
    double v1 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v1 < 30.0) {
        printf("FAIL: 门将对准后应直线推穿（有速度）v=%.0f\n", v1);
        return 1;
    }
    // ② 机头偏 90°（真机实测 -100°）→ 转正，40 帧内收敛到 ±20°（面向推球方向）
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

// ============================================================
// 第 72 轮：贴门线静止球「直线推出」硬钳位（修 Q1 丢球③震荡 + 丢球①绕行竞速）
//   球距门 <20cm（贴门线）或 对手 <40cm 正抢 → 门将站球门侧推出。
//   第 72 轮用户指令（问题2/3）：门球(无人逼抢)不再正前方直线踢（喂中路对手），改往
//   侧面空当推；对手正抢(<40cm)时仍直线远离己门（抢时间，丢球①不回退）。
//   断言三条：① 门球+门侧对齐 → 沿【侧面】方向推穿(有速度，且方向带 y 分量)；
//   ② 被抢球+门侧 → 直线推出(有速度)；③ 被抢球+机头垂直(-90°) → 先转正到 180° 再直线推穿。
// ============================================================
static int test_goalie_straight_clear() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                    // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 150; wm.home[i].y = 90; wm.opp[i].x = 100; wm.opp[i].y = 90;
        wm.role[i] = ROLE_PASSIVE;
    }
    wm.role[0] = ROLE_GOALIE;

    // ① 贴门线(15cm) + 对手远(105cm，不抢) → 应往【侧面】踢（出球方向必须带明显 y 分量）
    wm.ball.x = 205.0; wm.ball.y = 89.7; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    double pdirx = 0.0, pdiry = 0.0;
    gk_restart_direction(wm, 0, wm.ball.x, wm.ball.y, pdirx, pdiry);
    if (std::fabs(pdiry) < 0.3) {
        printf("FAIL: 门球出球方向应是侧面(带 y 分量)，实际 dir=(%.2f,%.2f) 仍直线\n", pdirx, pdiry);
        return 1;
    }
    // 门将沿侧面方向站球后对准 → 应推穿（有速度）
    wm.home[0].x = wm.ball.x - pdirx * 10.0;
    wm.home[0].y = wm.ball.y - pdiry * 10.0;
    wm.home[0].rot = angle_to(0.0, 0.0, pdirx, pdiry);
    run_goalie(wm, 0);
    double v1 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v1 < 30.0) {
        printf("FAIL: 门球侧面方向对准后应推穿(有速度) v=%.0f\n", v1);
        return 1;
    }

    // ② 对手正抢(<40cm) + 球 37cm(非贴门线) + 门将门侧 → 直线推出(抢得过对手)
    //   2026-10-06：原摆 (181,89.7) 恰是真机罚球点 (180.8,89.7)，现被识别为对方点球（门将守线），
    //   挪 2cm 避开罚球点，测试意图不变。
    wm.ball.x = 183.0; wm.ball.y = 89.7; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.opp[0].x = 152.0; wm.opp[0].y = 89.7;                              // 对手距球 31cm < 40
    wm.home[0].x = 215.0; wm.home[0].y = 89.7; wm.home[0].rot = 180.0;   // 门侧，距球 32cm
    run_goalie(wm, 0);
    double v2 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v2 < 30.0) {
        printf("FAIL: 被抢球门侧应直线推出(抢得过对手) v=%.0f\n", v2);
        return 1;
    }

    // ③ 被抢球 + 机头垂直(-90°，真机 09-22 场门球开局残留) → 先转正到 180° 再直线推穿。
    //    复现 f491：球 (205.4,89.7) 静止，门将 (214.8,90.3) 机头 -90°。旧代码直接
    //    motion::position 到球前 30cm，目标恰在机头 90° 后方 → 落进 (85°,95°) 死区自转，
    //    再被「推穿↔先到球后8cm」翻转来回拽成画弧振荡，球一动不动。对手距球 <40cm 走直线，
    //    验证转正兜底在直线分支仍有效（门球无人逼抢时走侧面，无此死区）。
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
    // 转正后下一帧应直线推穿（有速度）
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

// ============================================================
// 2026-10-06（用户指令："门将发球要把球尽可能推远，别轻轻一下推太近"）
//   真机现象：门球后球常停在自家门区边上，对手一步就推进来。
//   机理：球出脚初速 ≈ 触球瞬间车速。两个旋钮：
//     ① `roles.kGkAlignAcross`（3 → 6cm）：让门将"敢出脚"，不再一遍遍冲准备点来回蹭；
//     ② `roles.kGkKickThrough`（30 → 45cm）：出脚后"顶着球跑"更久，球被带得更快。
//   几何前提：真机门球点离门线仅 14.8cm、门将中心最多 ~215 ⇒ 直线助跑封顶 ~10cm，
//    "加长助跑跑满速"在门球上不成立，所以只动上面两个数（+准备点不刹停）。
//   断言（rot = 目标方向，v=(vl+vr)/2 = 车头方向速度）：
//     ① 横向差 5cm（旧 3cm 门槛判"没对准"）→ 该帧必须直接朝前高速出脚（旧代码落进
//        |te|∈(85°,95°) 死区原地打转，v≈0）—— 先红后绿；
//     ② 无人抢、球 40cm 出门线、距准备点 20cm → 带速穿球不该刹停（≥135；
//        旧口径 TM_STOP 被制动包线压到 ~122）—— 先红后绿；
//     ③ 横向差 13cm（>2×门槛，真推也擦不到）→ 必须回去摆准备点（套包线 → ≤110），
//        防止"放宽门槛"变成"闭眼乱冲"。
// ============================================================
// 2026-10-06：本用例由"横向门槛"改判"准备点带速"。
//   原因（`test_gk_goal_kick_reach` 敏感性矩阵实测）：
//     横向门槛 3→6cm 会让门将从"没对准"的位置就冲出去、擦不到球、来回空跑
//     （y78/y66 直接变 -1）⇒ 这条杠杆被证伪，默认必须留在 3cm，本用例不能再拿它当断言。
//   保留有效的那条杠杆：**准备点带速**。无人抢时，旧口径在准备点前 ~17cm 就按制动包线收油，
//     到了是零速再起步，撞球那一下车头速度只剩 ~122；带速走则一路不刹（≥135）。
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

    // ① 球 40cm 出门线（无人抢）：距准备点 20cm、横向对准 → 必须带速穿球（不刹停）
    wm.ball.x = 220.0 - 40.0; wm.ball.y = 90.0; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    double dx = 0.0, dy = 0.0;
    gk_restart_direction(wm, 0, wm.ball.x, wm.ball.y, dx, dy);
    wm.home[0].x = wm.ball.x - dx * 28.0;          // 准备点在球后 8cm ⇒ 距它 20cm
    wm.home[0].y = wm.ball.y - dy * 28.0;
    wm.home[0].rot = angle_to(0.0, 0.0, dx, dy);
    run_goalie(wm, 0);
    double v1 = 0.5 * (wm.home[0].vl + wm.home[0].vr);
    if (v1 < 135.0) {
        printf("FAIL: 带速穿球应在准备点前不刹车（v=%.0f，应≥135；旧 TM_STOP 包线只给 ~122）\n", v1);
        return 1;
    }

    // ② 横向差 13cm（>4×门槛，真推也擦不到球）：绝不高速直冲，回去摆准备点（套制动包线）
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

    // ③ 准备点带速的总开关：关掉后同一局面必须回到旧的"刹停"口径
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

// ============================================================
// 第 74 轮（2026-09-23 真机复盘）：门前清道夫 —— 治"门口无人区"
//   真机证据：球停在自家门前 3~4cm 达 113 帧（2.8s）没人清；18 局 26 个丢球里
//   23 个（88%）是"我方最后触球"。根因是三条防乌龙规则叠加：
//     ① 球在门区不逼抢（交给门将）；② 只从球门侧贴球（场侧不追）；
//     ③ 球贴门线时 B4"停轮站定、绝不碰球" —— 合起来 = 谁都不碰球，球自己滚进门。
//   新增通道只在**双向安全**的前提下出手：仅当 B4 比球更靠己门（已在球门侧）时
//   朝球推过去（从门侧推 ⇒ 球只会被顶离己门）。
//   断言（rot=180 面向 -x = 场内；前进为正）：
//     ① B4 在球门侧 → 必须主动清球（v>0）；
//     ② B4 在场侧   → 仍绝不直撞（保持原"停轮站定"，v≈0），不新增乌龙通道。
// ============================================================
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

    // ① 球门侧（离门更近）→ 应主动推球清出去（rot=180 时前进 = -x = 远离己门）
    //   2026-09-26：球须在裁判门区外（门线 22cm），门区内的球交给门将（见 ①b）
    wm.ball.x = 198.0; wm.ball.y = 91.0; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[4].x = 203.0; wm.home[4].y = 91.0; wm.home[4].rot = 180.0;
    run_passive(wm, 4);
    double v1 = 0.5 * (wm.home[4].vl + wm.home[4].vr);
    if (v1 < 10.0) {
        printf("FAIL: 门前清道夫没出手（球门侧 v=%.0f，应 >10 朝 -x 把球顶离己门）\n", v1);
        return 1;
    }

    // ①b 球在裁判门区内（离门线 8cm）→ 即使在球门侧也不进去清（非门将进门区=计点球）
    wm.ball.x = 212.0; wm.ball.y = 91.0;
    wm.home[4].x = 219.0; wm.home[4].y = 91.0; wm.home[4].rot = 180.0;
    run_passive(wm, 4);
    double v1b = 0.5 * (wm.home[4].vl + wm.home[4].vr);
    if (std::fabs(v1b) > 5.0) {
        printf("FAIL: 球在裁判门区内清道夫不该出手（v=%.0f，应 ≈0，交门将）\n", v1b);
        return 1;
    }

    // ② 场侧（离门更远）→ 绝不直撞球（保留"停轮站定"，否则就是历史乌龙通道）
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

// ============================================================
// 第 75 轮（2026-09-23 真机复盘）：球贴门线时"球外侧禁推"护栏被 6.76cm 门槛放过
//   真机 f726（14:40 场丢球1，黑匣子逐帧还原）：球 (218.99,75.44) 离门线 1.01cm、
//   门将 (214.85,71.31) 在场侧 5.9cm、球只比门将靠门 4.14cm < kGkBehindMargin 6.76
//   ⇒ 护栏判定"球还没明显越过门将（齐平）"而放弃 → 落到"照常清球"分支 → 门将冲球
//   ⇒ 球速 +0.33→+1.33 cm/帧（翻 4 倍）滚进自家门。同一局另 2 球、08:23 那局 3 球同型。
//   断言：① 贴门线 1cm + 门将在场侧 → 护栏必须拦下（返回 true，命令让开）；
//         ② 球贴门线但门将已在球门侧 → 护栏不拦（返回 false，保留合法清球能力）。
// ============================================================
static int test_gk_on_line_no_push() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                    // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) {
        wm.home[i].x = 150; wm.home[i].y = 90; wm.opp[i].x = 100; wm.opp[i].y = 90;
        wm.role[i] = ROLE_PASSIVE;
    }
    wm.role[0] = ROLE_GOALIE;

    // ① 真机 f726 原样复现（黑匣子数值，不做任何近似）
    wm.ball.x = 218.99; wm.ball.y = 75.44; wm.ball.vx = 0.328; wm.ball.vy = -2.021;
    wm.home[0].x = 214.85; wm.home[0].y = 71.31; wm.home[0].rot = -50.8;
    double tx = 0.0, ty = 0.0;
    if (!gk_side_step_point(wm, 0, tx, ty)) {
        printf("FAIL: 球离门线1cm且门将在场侧，'球外侧禁推'护栏没拦下（门将会把球顶进自家门）\n");
        return 1;
    }

    // ② 球同样贴门线，但门将已在球的门侧 → 护栏不拦，保留合法清球能力
    wm.ball.x = 212.0; wm.ball.y = 91.0; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.home[0].x = 219.0; wm.home[0].y = 91.0;
    if (gk_side_step_point(wm, 0, tx, ty)) {
        printf("FAIL: 门将已在球门侧时不该被让开护栏拦下（会丢掉合法清球能力）\n");
        return 1;
    }

    printf("gk on-line no push: OK (贴门线1cm必让开 / 门将已在门侧仍可清球)\n");
    return 0;
}

// ============================================================
// 第 73 轮（问题1 · A）：对方门口盘带 → 门将上前封角度，而非锁门线倒退
//   复现 0:4 复盘：对方门口 (x≈190) 从容盘带，门将退回门线 (x≈210) = 1v1 门洞大开。
//   断言：球离门 28cm、对手贴球 2cm、门将站门线外 13cm(离球 15cm)时，run_goalie 应
//   产生「前进(朝球, -x)」的速度（v>0），而非「倒退(回门线, +x)」（v<0）。
//   ⚠️ 断言用速度方向区分：门将 rot=180°(面向场)，前进=-x、倒退=+x。
// ============================================================
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

// ============================================================
// 第 73 轮（问题1 · B）：松球/即将接球也夹抢——持球判定 8→15cm
//   复现：防守时对方离球 9~15cm（松球/将接）无人上前，全队绕球走。
//   断言：离球 12cm 的对手（松球）压门前应触发夹抢（8→15 放行）；离球 20cm（真散球）不夹抢。
// ============================================================
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
    // 持球者压到门前 40cm（<111 危险距、<53.8 禁区协防距），但离球 12cm（松球）
    wm.ball.x = 180; wm.ball.y = 90;
    wm.opp[0].x = 180; wm.opp[0].y = 102;          // 离球 12cm（旧门槛 8 会拒，15 放行）
    double dx = 0, dy = 0;
    if (!double_team_point(wm, 1, dx, dy)) {
        printf("FAIL: 松球(离球12cm)对手压门前应夹抢（持球判定应 8→15 放行）\n");
        return 1;
    }
    // 离球 20cm（真散球）→ 仍不夹抢
    wm.opp[0].y = 110;                             // 离球 20cm
    if (double_team_point(wm, 1, dx, dy)) {
        printf("FAIL: 真散球(离球20cm)不该夹抢\n");
        return 1;
    }
    printf("doubleteam loose ball: OK (松球12cm夹抢 / 散球20cm不夹抢)\n");
    return 0;
}

// ============================================================
// 球权来源单测（2026-09-28 改：兜底不再信平台 whosBall）
//   判定纯距离自算：明确我方/对方（20cm 内且比对方近 5cm）→ 否则 by_distance（最近<20cm）。
//   whosBall 只作标定诊断计数（whos_mismatch），不参与 we_have_ball。
// ============================================================
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
    // ① 平台未知(0) + 我方最近且 <20cm → 自算"我方球权"
    wm.whos_ball = 0; wm.home[1].x = 115;
    if (!sitm.analyze(wm).we_have_ball) { printf("FAIL: whos=0 应按距离判我方\n"); return 1; }
    // ② 平台未知(0) + 对方更近 → 对方球权
    wm.home[1].x = 200; wm.opp[1].x = 115;
    if (sitm.analyze(wm).we_have_ball) { printf("FAIL: whos=0 对方更近应为对方\n"); return 1; }
    // ③ 明确我方控球（5cm）时，平台字段即使矛盾也不覆盖（防语义标定错误导致全队误判）
    wm.home[1].x = 115; wm.opp[1].x = 200; wm.whos_ball = 2;
    Situation s3 = sitm.analyze(wm);
    if (!s3.we_have_ball) { printf("FAIL: 明确控球时不该被平台字段覆盖\n"); return 1; }
    if (!s3.whos_mismatch) { printf("FAIL: 应记录 平台 vs 自算 不一致\n"); return 1; }
    // ④ 不明确（双方都 30cm 外）→ 不再听平台（whosBall 不可靠），按距离自算：相等 → 判非我方
    wm.home[1].x = 140; wm.opp[1].x = 140; wm.whos_ball = 1;
    if (sitm.analyze(wm).we_have_ball) { printf("FAIL: 散球时 whos=1 不再该信平台\n"); return 1; }
    // ④b 不明确但距离上我方更近（<20cm）→ 自算判我方，即使平台报 2=对方
    wm.home[1].x = 128; wm.opp[1].x = 150; wm.whos_ball = 2;
    if (!sitm.analyze(wm).we_have_ball) { printf("FAIL: 散球时我方更近应自算判我方\n"); return 1; }
    printf("possession source: OK (距离自算/明确控球不覆盖/散球不信平台)\n");
    return 0;
}

// ============================================================
// 第 83 轮：前场散球逼抢——谁近谁去 + 球在球员前方加分
//   断言：① 前场静止散球选离球最近且球在其前方的进攻球员；
//         ② 球在动且球周围有对方 → 不触发；
//         ③ 中卫离球最近也不选（只进攻三人组）。
// ============================================================
static int test_presser() {
    Strategy strat;

    // —— ① 前场静止散球：助攻(编号2)离球 20cm 且球在其前方 → 选助攻 ——
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

    // —— ② 球在动 且 球周围有对方(30cm) → 不触发 ——
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

    // —— ③ 中卫(编号4)离球最近(25cm)，但被排除，选进攻三人组里的助攻(30cm) ——
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

// ============================================================
// 2026-09-14：抢反弹位三合一升级 + 二抢一封推进方向 单测
// ============================================================
static int test_rebound_and_doubleteam() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                  // 蓝队，己方门 x=220
    wm.ball.valid = true;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        wm.opp[i].x = 100 + i; wm.opp[i].y = 30 + i * 20;   // 先都放远处（不是门将）
        wm.home[i].x = 150; wm.home[i].y = 90; wm.role[i] = ROLE_ASSIST;
    }
    // ① 反射版预测：球朝下边墙滚（直线会算到场外 → 反射后应仍在场内）
    wm.ball.x = 150; wm.ball.y = 20; wm.ball.vx = 2.0; wm.ball.vy = -2.0;
    wm.opp[0].x = 218; wm.opp[0].y = 90;         // 门将
    double rx = 0, ry = 0;
    rebound_point(wm, 30.0, rx, ry);
    if (ry < 70.0 || ry > 110.0) { printf("FAIL: 反弹位 y 应在门框内 got %.1f\n", ry); return 1; }
    if (std::fabs(rx - 140.0) > 0.5) { printf("FAIL: 反弹位 x 应为罚球区前缘 140 got %.1f\n", rx); return 1; }
    // ② 偏向：球落点(反射后)在门将上侧 → 落点应比"无偏向"更高
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
    // ③ 对手补射者占住落点 → 让开 20cm
    wm.ball.x = 150; wm.ball.y = 100; wm.ball.vx = 5.0; wm.ball.vy = 0.0;
    double a_x = 0, a_y = 0;
    rebound_point(wm, 0.0, a_x, a_y);
    wm.opp[1].x = a_x; wm.opp[1].y = a_y;        // 对手正站我们落点
    double b_x = 0, b_y = 0;
    rebound_point(wm, 0.0, b_x, b_y);
    if (std::fabs(b_y - a_y) < 15.0) { printf("FAIL: 落点被占应让开(差 %.1f)\n", std::fabs(b_y - a_y)); return 1; }

    // ④ 二抢一：持球者推进方向决定夹抢点（动 → 站他前面；静 → 站门侧）
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

// ============================================================
// docs/06 第 58 轮：门将「球外侧禁推」单测（真机 3 场 6 个丢球的共同机制）
//   球已到门口 + 门将在球的场侧 → 目标必须是"球后 10cm + 侧向 25cm"，
//   **绝不能落在球的门侧**（那等于自己把球往门里推）。
// ============================================================
static int test_goalie_side_step() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                  // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 180; wm.home[i].y = 90; }
    double tx = 0.0, ty = 0.0;

    // ① 球在门口 (215,95)、门将在球后 10cm 同线 → 触发；目标 = 球后 10cm + 侧向让开
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
    // ② 门将已侧向让开 25cm → 不再拦（可以去绕球的门侧推）
    wm.home[0].y = 135;   // 侧向已让开 40cm ≥ kGkSideClear(35)
    if (gk_side_step_point(wm, 0, tx, ty)) { printf("FAIL: 已让开就不该再拦\n"); return 1; }
    // ③ 门将已在球的门侧（比球更靠门）→ 不拦
    wm.home[0].y = 95; wm.home[0].x = 218;
    if (gk_side_step_point(wm, 0, tx, ty)) { printf("FAIL: 门将已在门侧不该拦\n"); return 1; }
    // ④ 球还远（距门 70cm）→ 不拦（走常规防守）
    wm.ball.x = 150; wm.home[0].x = 140;
    if (gk_side_step_point(wm, 0, tx, ty)) { printf("FAIL: 球还远不该拦\n"); return 1; }
    // ⑤ 黄队镜像（己方门线 x=0）
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

// ============================================================
// docs/06 第 57 轮：罚点球射门执行单测（真机 09-13 rlg 帧 2292~2338 复盘）
//   ① 罚点球助跑要比常规长（出球速度 = 撞球瞬间机头速度）
//   ② 罚点球时 ACTIVE **必须平移**：旧实现在球后就地转正 18 帧（vl≈-vr 纯自转，
//      观感"完全不动"）→ 断言 (vl+vr)/2 不能≈0
// ============================================================
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
    wm.home[1].x = 43.5; wm.home[1].y = 91.0; wm.home[1].rot = -179.9;   // 真机摆位：球后 4cm

    if (!(shoot_prep_dist(wm) <= 20.0)) {   // 第61轮：罚点球助跑改短（倒车太久会被截）
        printf("FAIL: 罚点球助跑应短(<=20) got %.1f\n", shoot_prep_dist(wm));
        return 1;
    }
    run_active(wm, 1);
    double v = 0.5 * (wm.home[1].vl + wm.home[1].vr);
    double spin = std::fabs(wm.home[1].vl) + std::fabs(wm.home[1].vr);
    // docs/06 第 66 轮：点球执行期改成"**先就地转正 → 立刻推穿**"（不再倒车助跑），
    //   所以"平移速度"可以为 0（正在原地转正），但**轮子必须有指令**（禁止原地磨蹭没动作）。
    if (std::fabs(v) < 10.0 && std::max(std::fabs(wm.home[1].vl), std::fabs(wm.home[1].vr)) < 2.0) {
        printf("FAIL: 罚点球时 ACTIVE 不做任何动作 vl=%.1f vr=%.1f\n",
               wm.home[1].vl, wm.home[1].vr);
        return 1;
    }
    // ③ 对手逼近（离球 21cm）→ 罚点球**绝不后退**：模拟 5 帧，到球距离不能变大
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
    // 常规射门：助跑距离应等于**当前旋钮值**
    wm.in_penalty_exec = false;
    wm.ball.x = 60.0;
    const double prep_expect = simuro5::get_param("roles.kPrepDist", 20.0);
    if (std::fabs(shoot_prep_dist(wm) - prep_expect) > 0.01) {
        printf("FAIL: 常规助跑应 %.1f（当前 kPrepDist）got %.1f\n", prep_expect, shoot_prep_dist(wm));
        return 1;
    }

    // ============================================================
    // docs/06 第 66 轮（用户真机实测"罚球还是太慢"）：
    //   ① 对手在 **34cm**（真机那次的真实距离）时也必须不后退 —— 旧规则只在 <45 才不倒车，
    //      但真机那次对手 34cm 却仍然倒了车 ⇒ 现在"点球一律不倒车"，这里锁住这个行为。
    //   ② 瞄准方向必须**锁存**：第一帧定下后，即使门将移动导致 plan_shoot 的新方案不同，
    //      执行期用的方向/瞄准点也不能变（否则准备点漂移 → 机器人从球侧上方掠过把球推偏）。
    // ============================================================
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
        // 真机摆位：踢球人在球后 4cm、机头朝左门（-179.9° ≈ 已对准）
        w2.home[1].x = 43.5; w2.home[1].y = 91.0; w2.home[1].rot = -179.9;
        // 门将（demo）站 (5.2, 90)：距球 34.2cm —— 真机那次的真实距离
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
        // ② 瞄准锁存：首帧锁定后，把门将挪走（plan_shoot 会给出不同方案），方向不得变
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
        // 执行结束 → 解锁（下一次点球重新算方向）
        w2.in_penalty_exec = false;
        run_active(w2, 1);
        if (w2.pen_aim_locked) { printf("FAIL: 点球结束后应解锁\n"); return 1; }
    }
    printf("penalty shot prep: OK (不倒车/对手34cm也不退/瞄准锁存/解锁/常规助跑20cm)\n");
    return 0;
}

// ============================================================
// docs/06 第 67 轮：罚点球"贴柱兜底"瞄准单测
//   真机报障"没有偏离、对着门将推球"：点球时门将站 (5.2,90)、距球 34.2cm，
//   遮挡 ±13.2° > 门张角 ±12.3° ⇒ 净开口 = 0 ⇒ 原逻辑回落成"瞄门中心" = 正对门将。
//   现在：净开口 <3° 时改瞄门柱内侧 4cm，门将居中则逐次交替选边。
// ============================================================
// docs/06 第 67 轮（含一处**更正**）：点球瞄准**本来就偏离门将**
//   起因：用户真机报障"没有偏离、对着门将推球"，我最初把门张角算成 ±12.3°
//   （那是球距门 92cm 时的值），据此以为"净开口=0 ⇒ 瞄门中心"。
//   正确数字：点球处球距门只有 39.4cm ⇒ 门张角 **±26.9°**，门将遮挡 ±13.2°
//   ⇒ 两侧各留 13.7° 空隙 ⇒ 原"取空隙中心"的瞄准落在 y≈75 或 104（**离门将约 14cm**）。
//   而且门将遮挡角恒小于门张角的一半（atan2(8,d) < atan2(20,d)）⇒ 净开口**不可能为 0**
//   ⇒ 曾经设想的"贴柱兜底"是死代码，已撤掉。
//   本用例把"正常点球瞄准偏离门将"这个事实锁住，防止以后再被误判成瞄准 bug。
// ============================================================
static int test_penalty_aim_offcenter() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队攻左门(x=0)，门 y∈[70,110]
    wm.ball.valid = true;
    wm.ball.x = 39.4; wm.ball.y = 89.8; wm.ball.vx = 0; wm.ball.vy = 0;
    wm.in_penalty_exec = true;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 150; wm.home[i].y = 20 + i * 30; }
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 150; wm.opp[i].y = 20 + i * 30; }

    // ① 门将站真机实测位置 (5.2,90)（距球 34.2cm）→ 瞄准必须**偏离**门将（约 14cm）
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
    // ② 门将偏下(y=74) → 该瞄上侧；偏上(y=106) → 该瞄下侧（打离门将远的那侧）
    wm.opp[0].y = 74.0;
    ShootPlan p2 = plan_shoot(wm, 1);
    wm.opp[0].y = 106.0;
    ShootPlan p3 = plan_shoot(wm, 1);
    if (!(p2.aim_y > 90.0 && p3.aim_y < 90.0)) {
        printf("FAIL: 应打离门将远的一侧，实际 门将下 aim=%.1f / 门将上 aim=%.1f\n",
               p2.aim_y, p3.aim_y);
        return 1;
    }
    // ③ 非点球（同一几何）行为不变：可射、penalty=false、瞄准在门框内
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
    // ① 球静止在对方罚球点 → 我方主罚
    wm.ball.x = 39.4; wm.ball.y = 89.8;
    if (!we_take_penalty_spot(wm)) { printf("FAIL: 球停在对方罚球点应判我方点球\n"); return 1; }
    // ② 球静止在**我方**罚球点(x≈180.6) → 那是对方主罚
    wm.ball.x = 180.6; wm.ball.y = 89.8;
    if (we_take_penalty_spot(wm)) { printf("FAIL: 我方罚球点上的球=对方主罚\n"); return 1; }
    // ③ 球在对方罚球点但还在滚 → 摆位期已过（比赛进行中）
    wm.ball.x = 39.4; wm.ball.y = 89.8; wm.ball.vx = 2.0;
    if (we_take_penalty_spot(wm)) { printf("FAIL: 球在动不应判摆位期\n"); return 1; }
    wm.ball.vx = 0.0;
    // ④ 球静止在中圈 → 不是点球
    wm.ball.x = 110; wm.ball.y = 90;
    if (we_take_penalty_spot(wm)) { printf("FAIL: 中圈静止球不应判点球\n"); return 1; }
    // ⑤ 黄队镜像：黄队攻右门(x=220)，其对方罚球点 x≈180.6
    wm.ctx = TeamContext{false};
    wm.ball.x = 180.6; wm.ball.y = 89.8;
    if (!we_take_penalty_spot(wm)) { printf("FAIL: 黄队镜像应判我方(黄)点球\n"); return 1; }

    printf("penalty spot detect: OK (对方罚球点=我方主罚/我方罚球点/球在动/中圈/黄队镜像)\n");
    return 0;
}

// ============================================================
// docs/06 第 69 轮：配合进攻（按"接球后射门机会质量"选传球目标：+0.15 门槛、只向前传、安全接球为前提）
//   默认随全量测试运行；--coop-pass-only 可单独复现，保留原场景和断言。
// ============================================================
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

// ============================================================
// docs/06 第 70 轮：禁止"过冲后反向穿球"（主攻冲过球、再沿瞄准线回推 = 把球顶向自家门）
//   真机实证：点球帧 896 起球朝 +x 加速到 +5.6cm/帧，就是因为 1 号已在球的球门侧。
// ============================================================
// 配合任务必须决定实际轮速，而不只是保存一份无人执行的坐标。
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
        // 门将留门前，另一对手封主攻射门线但不封向助攻的传球线。
        wm.opp[0].x = 0; wm.opp[0].y = 90;
        wm.opp[1].x = 50; wm.opp[1].y = 134;
        wm.opp[2].x = 67; wm.opp[2].y = 165;     // 第三人封顶墙借墙线（第 79 轮借墙闸门放宽后）
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
    // 第 103 轮：本段的断言都是"接球人走位 = 直线奔向任务锁点"这条老口径，
    //   先把接球会合点新行为（roles.kRecvMeetBall / kRecvNoReverse）关掉，
    //   新行为由 test_receiver_meet_ball 单独验；本段同时充当"回滚开关有效"的验证。
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
        // 本段验证出球执行；接球人先放到锁点，避免 readiness gate 把场景正确判成 WAIT。
        wm.home[receiver].x = cp.rx; wm.home[receiver].y = cp.ry;
        // 已达到射门上限，球速足够触发原射门计次：配合传球仍须推球且不计射门。
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
        // 普通站位故意换到另一边；任务目标和轮速不能被普通避敌/间距覆盖。
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
        // 空闲球与防守状态切换不能让任务丢失，候选站位已经全换走。
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
        // 到期后恢复普通角色动作。
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
    // 新任务与存量任务都要服从危险/比赛状态；不允许取消后同帧重发。
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
    // 评分通过也不能提前发布：球合法，但准备点落入角区，执行必须停下。
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
    // 真实调度会刷新普通站位，但应保留任务并先执行主攻再执行接球人。
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
    // 多次发出轮速而球不动，不能假报出球；速度字段单独跳高也不算。
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
    // 球跟人一起移动仍是带球，必须真的和传球人分离；倒向/横向移动也不算出脚。
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
        // 出球确认这帧就出现新挡线：必须先识别阶段，再决定是否重查线路。
        wm.opp[2].x = (75 + dx * 11 + 55) / 2; wm.opp[2].y = (150 + dy * 11 + 90) / 2;
        frame(wm, 75 + dx * 11, 150 + dy * 11);
        if (!wm.coop_pass_task.active || wm.coop_pass_task.phase != Phase::Receiving || !stopped(wm.home[1])) {
            printf("FAIL: coop observed release not latched/passer repeats push\n"); return 1;
        }
        // 对手进入新算出的球→接球点线段，但没有截到球；飞行不能因此取消。
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
        // 到目标而球还没到，不是接球成功。
        wm.opp[2].x = 200; wm.opp[2].y = 20;
        wm.home[receiver].x = 55; wm.home[receiver].y = 90;
        frame(wm, 65, 120);
        if (!wm.coop_pass_task.active || wm.coop_ball_control.active) { printf("FAIL: coop target arrival mistaken for reception\n"); return 1; }
        // 球高速掠过脚边也不是控住；接着减速并连续观测己方距离优势。
        wm.home[receiver].x = 55; wm.home[receiver].y = 85; wm.home[receiver].rot = 90;
        wm.we_have_ball = true;
        frame(wm, 55, 90);
        if (wm.coop_ball_control.active) { printf("FAIL: coop fast fly-by mistaken for possession\n"); return 1; }
        // 接球人虽近，但贴身争抢且没有己方球权证据，不能宣布接稳。
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
        // 已接球后仍由接球人处理，不恢复普通分散站位；1号不抢回同一脚球。
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
        // 控球者失球三帧恢复固定角色；球权被1号明确接管则立即释放临时权。
        WorldModel takeover = wm; takeover.home[1].x = 55; takeover.home[1].y = 90;
        takeover.home[receiver].x = 85;
        frame(takeover, 55, 90);
        if (takeover.coop_ball_control.active) { printf("FAIL: coop owner blocks teammate takeover\n"); return 1; }
        wm.home[receiver].x = 100; wm.home[receiver].y = 130; wm.we_have_ball = false;
        for (int i = 0; i < 3; ++i) frame(wm, 55, 90);
        if (wm.coop_ball_control.active) { printf("FAIL: coop owner persists after losing ball\n"); return 1; }
        // 飞行中确实截球/高威胁/超时仍取消，不能被不重算线路的豁免吞掉。
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
    // 飞行中停车不能冻结主攻门区总停留计时（球贴身时调度层存在攻门豁免）。
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

// 普通 PassPlan 也必须锁定同一接球点，并复用出球/接稳/临时控球生命周期。
static int test_ordinary_pass_task() {
    WorldModel wm = coop_task_scene();
    wm.role[1] = ROLE_ACTIVE; wm.role[2] = ROLE_ASSIST;
    wm.assist_x = 55; wm.assist_y = 90;
    PassPlan pp = plan_pass(wm, 1);
    if (!pp.viable || pp.receiver_id < 2) { printf("FAIL: ordinary PassPlan fixture\n"); return 1; }
    // 故意把接球人的普通站位移开；任务坐标必须仍是 PassPlan 的锁定点。
    wm.assist_x = 130; wm.assist_y = 150;
    auto &task = wm.coop_pass_task;
    task = {};
    task.active = true; task.passer_id = 1; task.receiver_id = pp.receiver_id;
    task.rx = pp.target_x; task.ry = pp.target_y; task.frames_left = 20;
    task.game_state = wm.game_state; task.kind = PassTaskKind::Ordinary;
    task.observing_push = true; task.push_ball_x = wm.ball.x; task.push_ball_y = wm.ball.y;
    const double len = dist(wm.ball.x, wm.ball.y, task.rx, task.ry);
    task.push_dir_x = (task.rx - wm.ball.x) / len; task.push_dir_y = (task.ry - wm.ball.y) / len;
    // 第 103 轮：验"普通传球任务的锁点不被覆盖"这条老性质 → 先关掉接球会合点新行为。
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

// 普通 PassPlan 与 CoopPass 共用同一 readiness gate：未到位只等，不改锁点、不开始观察出球。
static int test_pass_readiness_gate() {
    auto stopped = [](const RobotState &r) { return r.vl == 0.0 && r.vr == 0.0; };

    // 公式边界：阈值内 1e-6cm 放行，再远 0.1cm 就等待（避开浮点等号脆弱性）。
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
        // 第 103 轮：本段验"WAIT 时不改锁点/不推球"，接球人走位口径按老的锁点直线比。
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

    // 安全取消仍先于 WAIT：高威胁必须直接取消，不能因接球人未到位而保留任务。
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

// 第 103 轮（用户指令："接球需要提前到达位置，而不是后退、绕一下再接球"）：
//   ① 球在飞 → 接球人奔「会合点」迎球（不再傻站锁点等球滚过来）；
//   ② 目标在正后方 → 原地转身，不倒着走。
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

    // ① 球从远处朝锁点滚（球在 x=120 朝 -x 滚 5cm/帧，接球人在 x=60、面朝 +x）：
    //    应奔"球来路上的会合点"迎上去（测试用同一个公开函数复算期望值），
    //    而不是"直奔锁点 55"。会合点约 (67,90)，离接球人 7cm → 到位即站定迎球。
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

    // ② 目标在正后方（接球人在 x=70 面朝 +x、锁点在 x=55 身后）：原地转身，不倒车。
    {
        WorldModel wm = scene(CoopPassPhase::Preparing, 100.0, 0.0, 70.0, 0.0);
        run_assist(wm, 2);   // 接球人入口（run_assist 首行即 run_pass_receiver）
        const double common = fabs(wm.home[2].vl + wm.home[2].vr);
        if (!(wm.home[2].vl < 0.0 && wm.home[2].vr > 0.0) || common > 1e-9) {
            printf("FAIL: 目标在身后应原地转身 vl=%.1f vr=%.1f\n", wm.home[2].vl, wm.home[2].vr); return 1;
        }
    }

    // ③ 球停着（还没出脚）：仍走锁点（原行为），不做会合点那套。
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

    // ④ 纪律红线（第 103 轮 sim A/B 暴露"门区2+人帧 +40.8"）：会合点落进对方门区 →
    //    弃用会合点、回退锁点（否则接球人会为了抢球踩进对方小禁区 → 罚点球）。
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

// 等待接球人时，如果对手会明显更早占住锁点，就取消；球出脚后不再套用此规则。
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

    // 普通传球和配合传球必须走同一取消入口。
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

// 出球后接球人近球减速并面向来球；Preparing 与远距 Receiving 保持原赶路行为。
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

    // Preparing 必须逐轮保持原 motion::position，不得提前减速或转向迎球。
    {
        WorldModel wm = scene(PassTaskKind::Coop, CoopPassPhase::Preparing);
        RobotState expected = wm.home[2]; motion::position(expected, 100, 90);
        run_assist(wm, 2);
        if (!wm.coop_pass_task.active || !same_wheels(wm.home[2], expected)) {
            printf("FAIL: receive control changed Preparing movement\n"); return 1;
        }
    }
    // Receiving 但球还远：继续正常赶锁点，命令必须与原行为完全相同。
    {
        WorldModel wm = scene(PassTaskKind::Coop, CoopPassPhase::Receiving);
        wm.ball.x = 160; wm.ball.vx = -4;
        RobotState expected = wm.home[2]; motion::position(expected, 100, 90);
        run_assist(wm, 2);
        if (!wm.coop_pass_task.active || !same_wheels(wm.home[2], expected)) {
            printf("FAIL: receive control slowed distant ball approach\n"); return 1;
        }
    }
    // 正面来球：保持正确朝向，但近球时共同前进速度应明显低于原锁点赶路。
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
    // 侧面来球：到锁点后应原地转向球的来向，而不是横着停车等撞。
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
    // 球速过小或异常时用球的相对位置兜底，不能按噪声方向乱转或输出 NaN/Inf。
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
    // Ordinary 与 Coop 必须得到完全相同的近球接应命令。
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
    // 接稳和超时仍沿用原生命周期，不因动作层变化而改判据。
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
    // 1 号已经**冲过球**（在球的球门侧）：球在 39.4，它在 33.0，机头朝 −x
    wm.home[1].x = 33.0; wm.home[1].y = 88.7; wm.home[1].rot = -165.0;
    run_active(wm, 1);
    const double v = 0.5 * (wm.home[1].vl + wm.home[1].vr);
    // 允许原地转向（v≈0），但**绝不允许朝 +x（自家门方向）开**去穿球
    if (v > 2.0) {
        printf("FAIL: 在球的球门侧时应绕行/转向，不该朝自家门驱动 vl=%.1f vr=%.1f\n",
               wm.home[1].vl, wm.home[1].vr);
        return 1;
    }
    printf("no reverse-through-ball: OK (球门侧只绕行/转向，不朝自家门推)\n");
    return 0;
}

// ============================================================
// docs/06 第 71 轮：对方出脚方向预测（只采信"球静止+贴球+机头对球"的 rot）
// ============================================================
static int test_opp_kick_predict() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队：己方门线 x=220，门框 y∈[70,110]
    wm.ball.valid = true;
    wm.ball.x = 180; wm.ball.y = 90; wm.ball.vx = 0; wm.ball.vy = 0;
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 300; wm.opp[i].y = 20 + i * 30; wm.opp[i].rot = 0; }
    // 对手贴球、机头正对球 → 预测落点约 y=91.8（在门框内）
    wm.opp[0].x = 158; wm.opp[0].y = 89;
    wm.opp[0].rot = angle_to(158, 89, 180, 90);
    double y = 0.0;
    if (!opp_kick_target_y(wm, y)) { printf("FAIL: 贴球且机头对球应给出预测落点\n"); return 1; }
    if (std::fabs(y - 91.8) > 1.5) { printf("FAIL: 预测落点应≈91.8，实际 %.1f\n", y); return 1; }
    // ② 球在动 → 不用朝向
    wm.ball.vx = 2.0;
    if (opp_kick_target_y(wm, y)) { printf("FAIL: 球在动时不该用朝向预测\n"); return 1; }
    wm.ball.vx = 0.0;
    // ③ 机头没对着球（转 90°）→ 不采信
    wm.opp[0].rot = normalize_angle(wm.opp[0].rot + 90.0);
    if (opp_kick_target_y(wm, y)) { printf("FAIL: 机头没对球不该预测\n"); return 1; }
    // ④ 没人贴球（>25cm）→ 不预测
    wm.opp[0].x = 140;
    if (opp_kick_target_y(wm, y)) { printf("FAIL: 对手离球太远不该预测\n"); return 1; }
    printf("opp kick predict: OK (贴球+对球才预测/球在动不用/机头偏不采信/太远不预测)\n");
    return 0;
}

// ============================================================
// docs/06 第 68 轮：我方门区"只能有门将"硬闸
//   真机 09-14 20:46 场：我方门区 ≥2 人 674 帧（≈9%），同场被判 9 次点球 ⇒ 病根在此。
// ============================================================
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

    // ① 门将豁免：不该被这个硬闸指挥（速度保持 0）
    if (std::fabs(wm.home[0].vl) > 0.01 || std::fabs(wm.home[0].vr) > 0.01) {
        printf("FAIL: 门将不应被门区硬闸驱动 (vl=%.1f vr=%.1f)\n", wm.home[0].vl, wm.home[0].vr);
        return 1;
    }
    // ② 门区里的后卫必须被驱动（朝 −x 走，即朝门区外/场内方向）
    if (std::fabs(wm.home[4].vl) < 1.0 && std::fabs(wm.home[4].vr) < 1.0) {
        printf("FAIL: 门区里的后卫应被顶出去，实际没动作\n");
        return 1;
    }
    if (wm.home[4].vl + wm.home[4].vr <= 0.0) {
        printf("FAIL: 顶出方向应朝场内(−x)，实际 vl=%.1f vr=%.1f\n",
               wm.home[4].vl, wm.home[4].vr);
        return 1;
    }
    // ③ 门区外的人不动
    if (std::fabs(wm.home[1].vl) > 0.01 || std::fabs(wm.home[1].vr) > 0.01) {
        printf("FAIL: 门区外的主攻不应被动 (vl=%.1f)\n", wm.home[1].vl);
        return 1;
    }
    // ④ 黄队镜像：己方门区变成 x∈[0,50]
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

// ============================================================
// 2026-10-06：己方大禁区人数闸（规则 7.10.4）
//   规则：除守门员外，**4 个机器人在禁区内（A+B，球门前 80×35）防守 → 直接判点球**。
//   （7.10.3 的"3 个在禁区内"只在"在球门区里停留 >20 周期"时才罚，而门区已被
//    enforce_own_goal_area 每帧清空 ⇒ 只要大禁区里非门将 ≤3 人就安全。）
//   为什么需要独立闸：`own_box_clamp`（roles.cpp，"只放离球最近的一个分区球员进大禁区"）
//   挂在 run_zone 上，而 `strategy.kZoneMode` 默认 0（第 93 轮用户指令关闭）⇒ 那条纪律
//   已是死代码；plan_defense 只把断球/护门点推到 85cm 线，管不住 ACTIVE 追球、盯人跟防、
//   护门点回撤同时涌入大禁区。
//   断言：① 4 台在己方大禁区里 → 顶出**离球最远**的那台（其余不动）；
//        ② 只有 3 台 → 一个人都不动；③ 门将豁免（门将也在里面不额外顶人）；
//        ④ 黄队镜像：顶出方向朝 +x（场地中心侧）。
// ============================================================
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

    // ① 4 台非门将都在禁区内（球在中路 x=150 → 离球最远的是贴门线的 1 号）
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
    // 顶出方向必须指向"大禁区前缘外"：蓝队目标 x = 220−85 = 135（在 212 的 −x 侧）。
    //   rot=0 → 前进方向 = +x，轮速和 (vl+vr) 为正表示朝 +x；用投影判方向，不写死符号。
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

    // ② 只有 3 台在禁区内（+门将）→ 谁都不动
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

    // ③ 门将豁免：4 台非门将 + 门将全在禁区内 → 仍只顶出非门将里离球最远的那台
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

    // ④ 黄队镜像：己方门线 x=0 → 禁区 x∈[0,80]；顶出方向朝 +x（场地中心侧）
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
    // ⑤ 黄队只有 3 台 → 不动
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

// ============================================================
// 2026-09-26：裁判口径门区（官方 Judge_PENALTY_KICK：门线内 15cm × y∈[65,115]）
//   旧 in_goal_area（50×30，y∈[75,105]）漏掉门柱两侧 y∈[65,75)∪(105,115]，
//   而裁判计数离开不清零 → magic_rob vs demo 场均 14 个点球的主因。
// ============================================================
static int test_rule_goal_area() {
    TeamContext b{true}, y{false};
    // 几何：蓝队门线 x=220
    if (!in_goal_area_rule(b, 210, 112)) { printf("FAIL: (210,112) 应在裁判门区（门柱外侧）\n"); return 1; }
    if (in_goal_area_rule(b, 204, 90))   { printf("FAIL: (204,90) 离门线 16cm 不在裁判门区\n"); return 1; }
    if (in_goal_area_rule(b, 210, 116))  { printf("FAIL: (210,116) y 超 115 不在裁判门区\n"); return 1; }
    if (!in_goal_area_rule(b, 225, 108)) { printf("FAIL: (225,108) 球门里应计入\n"); return 1; }
    if (in_goal_area_rule(b, 225, 112))  { printf("FAIL: (225,112) 球门里 y 超 110 不计\n"); return 1; }
    if (!in_goal_area_rule(b, 200, 90, 8.0, 0.0)) { printf("FAIL: 余量 8cm 时 (200,90) 应命中\n"); return 1; }
    if (!in_goal_area_rule(y, 10, 68))   { printf("FAIL: 黄队镜像 (10,68) 应在裁判门区\n"); return 1; }

    // 硬闸：非门将站在门柱外侧 (210,112)（旧门区漏判处）→ 必须被顶出去（朝 −x）
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

// ============================================================
// docs/06 第 79 轮：分道压迫进攻（run_assist / run_midfield 前置的 run_swarm）
//   推进方向与 roles.cpp 同口径：射门方案 → 借墙推进方案 → 门心。
//   用 motion::position 对期望目标点算出的轮速做比对（目标点对了轮速就逐位相等）。
// ============================================================
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
    // —— 与射门模块解耦：本用例测的是"蜂群推进几何"，不该被借墙几何牵连 ——
    //   ⚠️ 2026-09-30：撞墙系数修正（shoot.kBankWallRest 0.66→0.45）后，本场景
    //   `plan_shoot` 会返回**借墙方案**，于是 herd_direction 给出"朝墙"的入射方向，
    //   各子用例的期望目标随之变化。这不是蜂群逻辑的问题，而是射门模块的输入变了。
    //   ⇒ 这里临时把**两条借墙路径**关掉，让 herd_direction 稳定退回"朝门心"兜底方向，
    //     用例只考蜂群几何、与射门调参解耦（射门自己的行为由 test_shoot_plan/test_bank_shot 守）。
    //   用 save/restore 而非 reset_params()：后者会清掉 --params 注入（见 test_shoot_push_limit 的坑）。
    //   用 RAII 保证早退路径也复原。
    //   ⚠️ 2026-10-06 补记：远射开口门槛 `shoot.kMinOpen` 9°→5°（见 docs/06 该轮）后，本场景的
    //   `plan_shoot` 由"恒不可行"变成"可行"（净开口落在 5°~9° 之间），而它的方向依赖队员当前位置 ⇒
    //   用例把 (ux,uy) 算在**旧位置**、实现算在**新位置**，② 横绕的期望目标失配（实测 vl 差 4.4）。
    //   这与"蜂群逻辑坏了"无关，是同一类射门耦合 ⇒ 同样把远射闸门临时关死（kMinOpen=90 ⇒ 恒不可行），
    //   退回"朝门心"兜底方向。射门门槛自己的行为由 test_shoot_plan 守。
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
    // ① 人在球后且对准 → 沿推进方向推穿
    {
        WorldModel wm = scene(100, 120);
        double ux, uy; herd_dir(wm, ux, uy);
        wm.home[2].x = 100 - ux * 8; wm.home[2].y = 120 - uy * 8; wm.home[2].rot = angle_to(0, 0, ux, uy);
        run_assist(wm, 2);
        if (!expect(wm, 2, 100 + ux * 22.0, 120 + uy * 22.0, "推穿")) return 1;
    }
    // ② 人在球前面（推进线上）→ 横绕到球侧，绝不直冲球（防往回顶）
    {
        WorldModel wm = scene(100, 120);
        double ux, uy; herd_dir(wm, ux, uy);
        wm.home[2].x = 100 + ux * 30; wm.home[2].y = 120 + uy * 30;
        run_assist(wm, 2);
        if (!expect(wm, 2, 100 - ux * 4 - uy * 17.0, 120 - uy * 4 + ux * 17.0, "横绕")) return 1;
    }
    // ③ 球在下半道 → ASSIST（上半道）弱侧跟进：落后球 22cm、y=90+42
    {
        WorldModel wm = scene(100, 40);
        run_assist(wm, 2);
        if (!expect(wm, 2, 122, 132, "弱侧跟进")) return 1;
    }
    // ④ 球在对方门区附近 → 两人都在门区外沿等二点，不进门区
    {
        WorldModel wm = scene(30, 95);
        run_assist(wm, 2); run_midfield(wm, 3);
        if (!expect(wm, 2, 68, 122, "门外等二点(上)") || !expect(wm, 3, 68, 58, "门外等二点(下)")) return 1;
    }
    // ⑤ 队友已顶住球 → 不挤同一个球，站侧后方护送
    {
        WorldModel wm = scene(100, 95);
        double ux, uy; herd_dir(wm, ux, uy);
        wm.home[1].x = 100 - ux * 8; wm.home[1].y = 95 - uy * 8;
        wm.home[2].x = 120; wm.home[2].y = 130;
        double side = ((120 - 100) * -uy + (130 - 95) * ux) >= 0 ? 1.0 : -1.0;
        run_assist(wm, 2);
        if (!expect(wm, 2, 100 - ux * 16 - uy * side * 22, 95 - uy * 16 + ux * side * 22, "护送")) return 1;
    }
    // ⑥ 球离我方门 <60cm → 不接管（交回原防守逻辑）：目标不能是推穿/落位点
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

// ============================================================
// docs/06 第 55 轮：门将「门线封堵」单测（真机 15:29 场两个丢球的病因）
//   球已在门框内轨迹上、马上到线 → 必须抢门线预测落点（贴线 3cm + 预测落点 y）。
//   其余情形一律让位：背离门 / 会偏出 / 还太远 / 太慢 / 门将已贴球（清球优先）。
// ============================================================
static int test_goalie_line_cover() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队：己方门线 x=220，门框 y∈[70,110]
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 180; wm.home[i].y = 90; }
    double tx = 0.0, ty = 0.0;

    // ① 球朝门飞、落点 95 在门框内、门将在 15cm 外 → 触发；目标 = 门线前 3cm + 预测落点
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
    // ② 球背离门 → 不抢
    wm.ball.vx = -2.0;
    if (gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 球背离门不应触发\n"); return 1; }
    // ③ 预测落点跑到门框外（y=125）→ 不抢
    wm.ball.x = 210; wm.ball.y = 130; wm.ball.vx = 2.0; wm.ball.vy = -1.0;
    if (gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 会偏出的球不应触发\n"); return 1; }
    // ④ 球还远（距门线 80cm）→ 不抢
    wm.ball.x = 140; wm.ball.y = 95; wm.ball.vx = 2.0; wm.ball.vy = 0.0;
    if (gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 球还远不应触发\n"); return 1; }
    // ⑤ 球太慢（朝门 ~0.45cm/帧 = 18cm/s）→ 不抢（站线跟球即可）
    wm.ball.x = 210; wm.ball.y = 95; wm.ball.vx = 0.5; wm.ball.vy = 0.0;
    if (gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 太慢的球不应触发门线封堵\n"); return 1; }
    // ⑥ 门将已贴球（5cm）→ 让位给清球（推出去比站线好）
    wm.ball.vx = 2.0;
    wm.home[0].x = 205; wm.home[0].y = 95;
    if (gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 门将贴球时应让位给清球\n"); return 1; }
    // ⑦ 黄队镜像（己方门线 x=0）
    wm.ctx = TeamContext{false};
    wm.ball.x = 10; wm.ball.y = 95; wm.ball.vx = -2.0; wm.ball.vy = 0.0;
    wm.home[0].x = 25; wm.home[0].y = 95;
    if (!gk_cover_line_point(wm, 0, tx, ty)) { printf("FAIL: 黄队镜像应触发\n"); return 1; }
    if (fabs(tx - 3.0) > 0.5) { printf("FAIL: 黄队镜像目标 x 应为 3，实际 %.1f\n", tx); return 1; }

    printf("goalie line cover: OK (门框内轨迹抢落点/背离-偏出-太远-太慢-贴球让位/黄队镜像)\n");
    return 0;
}

// 第 90 轮：争抢直冲——对手贴球、主攻在球侧上方（撞过去是横推，不朝自家门）→ 直接朝球冲，不绕球后
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

// 第 90 轮：近球不减速/不原地转正/不等对准（官方式）。在球后、朝向偏 60°、无争抢：
//   旧（kNoAlignWait=0）原地转正（前进分量≈0），新（=1）直接前冲穿球。
// 第 91 轮：官方式分区。① 中卫在球后 → 直冲穿球；② 己方门前站在球的进攻侧 → 侧绕不穿球；
//   ③ 上翼（ASSIST）球在下路 → 站 (球fx-8, y=120)。期望动作与直接调 motion 同目标逐位一致。
// 第 92 轮：门球静止球门将够得着 —— 球 y=72 时准备点被 kGkYLo=74 夹住、永远对不准
//   （真机 s53 门将 y 62↔88 来回冲 197+ 帧不触球）。简易运动学闭环：N 帧内必须把球推出 x<195。
// 2026-10-06：reset_params() → save/restore。reset 会清掉 --params 注入和其他旋钮
//   （本文件 L1078 已记过这个坑）；顺带把"门将推远"两个旋钮做成入参，好打印敏感性矩阵。
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
    // 敏感性矩阵（只打印，不改判定）：横向门槛 × 穿球目标，每格 = y66/72/78 的推出帧数。
    //   用途：2026-10-06 加长穿球目标后 y78/y66 变 -1，用矩阵定位"哪个旋钮在哪个球位上翻车"。
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
    // ① 球 (150,90)，中卫在球后 (170,90) 朝 -x → 目标 = 球前 20cm (130,90)
    base(wm, 150, 90); wm.home[4].x = 170; wm.home[4].y = 90; wm.home[4].rot = 180;
    RobotState e = wm.home[4]; motion::position(e, 130, 90, motion::TM_PASS);
    run_zone(wm, 4);
    if (!same(wm.home[4], e) || wm.home[4].vl < 50) { printf("FAIL: 中卫应直冲穿球 vl=%.1f vr=%.1f\n", wm.home[4].vl, wm.home[4].vr); return 1; }
    // ② 球 (180,90) 离己门 40cm，中卫在 (160,95)（球的进攻侧）→ 侧绕 (185,112)，不往自家门撞
    base(wm, 180, 90); wm.home[4].x = 160; wm.home[4].y = 95; wm.home[4].rot = 0;
    e = wm.home[4]; motion::position(e, 185, 112, motion::TM_PASS);
    run_zone(wm, 4);
    if (!same(wm.home[4], e)) { printf("FAIL: 己方门前站错侧应侧绕\n"); return 1; }
    // ③ 球 (150,40)：上翼 ASSIST 不追，站 fx=70-8=62 → (158,120)
    base(wm, 150, 40); wm.home[2].x = 150; wm.home[2].y = 150; wm.home[2].rot = 0;
    e = wm.home[2]; motion::position(e, 158, 120);
    run_zone(wm, 2);
    if (!same(wm.home[2], e)) { printf("FAIL: 上翼应站 (158,120)\n"); return 1; }
    // ④ 己方禁区纪律：球 (200,60)，下翼 MIDFIELD 官方站位 (20,120)→镜像 y=60 落在大禁区；
    //   它不是离球最近的分区球员（中卫在 (195,65)）→ 夹到 fx=45 即 x=175
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

// 第 89 轮：非主攻进对方门区当帧就撤（真机 09-29 帧 4544：MID 在 (23,63) 滞留 26 帧，球在对角 (5,154)）
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

// 第 88 轮：活球判定。真机进行中 gameState 停在重启类型不回 PlayOn（黑匣子 09-28 三场 gs∈{1,2,3,5}，0 帧 PlayOn）
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

// 第 87 轮：对准球的直线提前堵（场景取自 09-28 真机黑匣子）
static int test_goalie_line_block() {
    WorldModel wm;
    wm.ctx = TeamContext{true};                 // 蓝队：己方门线 x=220
    wm.ball.valid = true;
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 150; wm.home[i].y = 90; wm.opp[i].x = 100; wm.opp[i].y = 90; }
    double tx = 0.0, ty = 0.0;

    // ① 中路慢球（0.47cm/帧，y 微降）、门将在门前 → 站直线上门前 10cm
    wm.ball.x = 197; wm.ball.y = 88.5; wm.ball.vx = 0.47; wm.ball.vy = -0.04;
    wm.home[0].x = 209; wm.home[0].y = 80; wm.home[0].rot = -110;
    if (!gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 中路慢球应对线\n"); return 1; }
    double y_line = 88.5 - 0.04 * (210.0 - 197.0) / 0.47;
    if (fabs(tx - 210.0) > 0.5 || fabs(ty - y_line) > 0.5) {
        printf("FAIL: 对线目标应为 (210,%.1f)，实际 (%.1f,%.1f)\n", y_line, tx, ty); return 1;
    }
    // ② 对方贴球带 → 不接管（交给持球硬锁）
    wm.opp[0].x = 190; wm.opp[0].y = 88.5;
    if (gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 对方贴球不应对线\n"); return 1; }
    wm.opp[0].x = 100; wm.opp[0].y = 90;
    // ③ 快球 → 不接管（门线封堵/前压封角处理）
    wm.ball.vx = 4.0; wm.ball.vy = 0.0;
    if (gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 快球不应走对线\n"); return 1; }
    // ④ 会偏出的慢球 → 不接管
    wm.ball.x = 197; wm.ball.y = 130; wm.ball.vx = 0.5; wm.ball.vy = 0.0;
    if (gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 偏出的慢球不应对线\n"); return 1; }
    // ⑤ 贴门线从上往下滚向门口（x=217.5, y=140, vy=-1）→ 站路径 x 上、门口上沿内侧
    wm.ball.x = 217.5; wm.ball.y = 140; wm.ball.vx = 0.0; wm.ball.vy = -1.0;
    wm.home[0].x = 209; wm.home[0].y = 104;
    if (!gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 贴门线滚球应堵路径\n"); return 1; }
    if (fabs(tx - 217.0) > 0.5 || fabs(ty - 106.0) > 0.5) {
        printf("FAIL: 贴门线堵点应为 (217,106)，实际 (%.1f,%.1f)\n", tx, ty); return 1;
    }
    // ⑥ 贴门线从下往上滚、已进门口带（y=80, vy=+2）→ 球前方 12cm
    wm.ball.y = 80; wm.ball.vy = 2.0;
    if (!gk_line_block_point(wm, 0, tx, ty) || fabs(ty - 92.0) > 0.5) {
        printf("FAIL: 门口内贴线滚球应堵在球前 (y=92)，实际 %.1f\n", ty); return 1;
    }
    // ⑦ 贴门线但背离门口滚（y=40 往下）→ 不接管
    wm.ball.y = 40; wm.ball.vy = -2.0;
    if (gk_line_block_point(wm, 0, tx, ty)) { printf("FAIL: 背离门口的贴线球不应接管\n"); return 1; }
    // ⑧ 黄队镜像：中路慢球朝 x=0
    wm.ctx = TeamContext{false};
    wm.ball.x = 23; wm.ball.y = 90; wm.ball.vx = -0.5; wm.ball.vy = 0.0;
    wm.home[0].x = 11; wm.home[0].y = 80;
    if (!gk_line_block_point(wm, 0, tx, ty) || fabs(tx - 10.0) > 0.5 || fabs(ty - 90.0) > 0.5) {
        printf("FAIL: 黄队镜像对线目标应为 (10,90)，实际 (%.1f,%.1f)\n", tx, ty); return 1;
    }
    printf("goalie line block: OK (慢球对线/贴球-快球-偏出不接管/贴门线堵路径/黄队镜像)\n");
    return 0;
}

// ============================================================
// 带权匈牙利求解器单测（第 97 轮，用户指令「使用带权的匈牙利算法」）
//   ① 手算 3×3 的唯一最优解
//   ② "两台车都想盯同一个人"的例子：证明一一匹配的价值（各自贪心会撞车）
//   ③ 随机矩阵 ×300 组与**暴力枚举所有排列**对拍（n=3/4/5；5!=120）——
//      求解器只要错一点，这一条就会抓出来
// ============================================================
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
    // ① 手算：最优 = row0→col1(1) + row1→col0(2) + row2→col2(2) = 5
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
    // ② 撞车例子：（行=两台车，列=两个对手）
    //    两台车都离对手 0 更近（10 / 12 < 30 / 15）⇒ 各自贪心会**双双去盯对手 0**，
    //    对手 1 没人管。匈牙利必须一一匹配：(0→0, 1→1) = 25 < (0→1, 1→0) = 42。
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
    // ③ 暴力对拍（固定种子 ⇒ 可复现）
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

// 盯人分配的场景脚手架（蓝队、威胁过门槛、固定分工 2=ASSIST 3=MIDFIELD 4=PASSIVE）
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

// ② 防抖对照用：跑 80 帧（前 40 帧分工 A、后 40 帧把两台车位置互换 → 裸最优想换成分工 B，
//    省 7.2cm），对手位置带确定性微扰（±0.3cm，模拟真机噪声）。返回"换人次数"。
static int run_mark_swap_probe(double lambda) {
    // λ 对照测的是 kMarkLambda 自身的"锁"行为，须在犀利档关闭下测：
    // kSharpDefense=1 会把 λ 硬覆盖成 sharp_mark_lambda=15，令 set_param 的 λ=0 失效。
    const double lam_save   = get_param("defense.kMarkLambda", 25.0);
    const double sharp_save = get_param("defense.kSharpDefense", 0.0);
    set_param("defense.kSharpDefense", 0.0);
    set_param("defense.kMarkLambda", lambda);
    WorldModel wm = mark_scene();
    wm.ball.x = 160; wm.ball.y = 90; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    // 两个"有价值"的对手（球在它们中间，威胁相同）
    const double opp_base[5][2] = {{150, 70}, {150, 110}, {60, 20}, {60, 160}, {90, 40}};
    for (int j = 0; j < PLAYERS_PER_SIDE; ++j) {
        wm.opp[j].x = opp_base[j][0];
        wm.opp[j].y = opp_base[j][1];
    }
    wm.home[4].x = 30; wm.home[4].y = 90;          // 4 号留在前场（太远 ⇒ 应当闲着）
    wm.mark_switch_events = 0;
    for (int f = 0; f < 80; ++f) {
        // 前 40 帧：2 号在 (140,88)、3 号在 (140,92) —— 分工 A 更优
        // 后 40 帧：两者位置互换 —— 分工 B 反而优 7.2cm
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

// ============================================================
// 带权匈牙利盯人分配单测（第 97 轮）
//   ① 一个危险对手只能被**一台**车盯（旧的各自贪心会让三台都扑上去）
//   ② 局面几乎没变时不许换人（λ/EMA/量化/承诺期四道锁）——
//      并用"把 λ 调成 0 就会换"做对照，证明锁真的在起作用
//   ③ 关掉开关 / 威胁没过门槛 ⇒ 不产生指派（角色函数回退旧逻辑）
// ============================================================
static int test_mark_assignment() {
    // ① 三台车都挤在危险对手旁边：只能有一台被指派盯他，且是最近的那台
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
    // ② 防抖：λ=0（无锁）会换人；默认 λ=25（有锁）必须顶住
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
    // ③ 关掉开关 / 威胁没过门槛 ⇒ 不指派（角色函数各自回退旧逻辑）
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
        // 门槛以下（开关开着也不该指派）
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
    // --params <文件>：注入参数后再跑全部测试。
    // 这是"自动调参的行为护栏"：搜索时对每个候选跑一遍本程序，
    // 跑不过就直接判死（防止优化器靠"关掉防守行为/借墙射门"刷分，见 docs/06 轮次 77）。
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
        rc |= test_receiver_meet_ball();     // 第 103 轮：接球会合点 + 禁止倒车
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
    rc |= test_receiver_meet_ball();     // 第 103 轮：接球会合点 + 禁止倒车
    rc |= test_pass_opponent_first_cancel();
    rc |= test_pass_receive_control();
    rc |= test_goalie_side_step();
    rc |= test_rebound_and_doubleteam();
    rc |= test_possession_source();
    rc |= test_goalie_clear_push();
    rc |= test_goalie_straight_clear();
    rc |= test_goalie_kick_far();          // 2026-10-06：门将发球推远（对齐门槛/穿球目标/带速准备点）
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
    rc |= test_hungarian_solver();     // 第 97 轮：带权匈牙利求解器（含暴力枚举对拍）
    rc |= test_mark_assignment();      // 第 97 轮：盯人一一匹配 + 防抖动四道锁
    rc |= test_goalie_predict();
    rc |= test_defense_reach();
    rc |= test_defense_face_incoming();  // 第 103 轮：防守到位迎球朝向
    rc |= test_ball_meeting_point();     // 第 103 轮：会合点计算
    rc |= test_goal_cover();
    rc |= test_goal_cover_yellow();   // 2026-10-07：黄位镜像用例（此前该函数只被蓝队测过）
    rc |= test_team_state();
    rc |= test_fixed_roles();
    rc |= test_pass();
    rc |= test_pass_threat_weight();
    rc |= test_goalie_scenarios();
    rc |= test_goalie_og_fixes();        // 2026-10-06：点球守线/门球慢爬/贴线横移
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
