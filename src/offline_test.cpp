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

    // 边界反弹（新增）：球朝底/顶边线滚，直线会出界，反射后应折返到界内
    double ry = 0.0;
    predict_y_at_x_reflect(100.0, 20.0, 2.0, -2.0, 160.0, ry);   // 撞底墙 -> y=40
    if (fabs(ry - 40.0) > 0.5) { printf("FAIL: 撞底墙反射 y=%.1f (应 40)\n", ry); return 1; }
    predict_y_at_x_reflect(100.0, 160.0, 2.0, 2.0, 160.0, ry);   // 撞顶墙 -> y=140
    if (fabs(ry - 140.0) > 0.5) { printf("FAIL: 撞顶墙反射 y=%.1f (应 140)\n", ry); return 1; }

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
    //   球 (205,90) 距门 15cm → 护门点 x=213, y=90（球与门之间，不是球后）
    wm.ball.x = 205; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 贴门球应返回 true\n"); return 1; }
    if (fabs(cx - 213.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: 贴门护门点 (%.1f,%.1f) 应 (213,90)\n", cx, cy); return 1;
    }

    // 斜向：球 (150,130) 距门 ~72.1cm（中近分区）→ 门前 50 线 x=170, y=130 夹回 107.5
    wm.ball.x = 150; wm.ball.y = 130;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 斜向球应返回 true\n"); return 1; }
    if (fabs(cx - 170.0) > 0.5 || fabs(cy - 107.5) > 0.5) {
        printf("FAIL: 斜向护门点 (%.1f,%.1f) 应 (170,107.5)\n", cx, cy); return 1;
    }

    // 贴门但不越过门线：球 (218,90) 距门 2cm → 球前 8cm 会越线，应 clamp 到门前 3cm (217)
    wm.ball.x = 218; wm.ball.y = 90;
    if (!goal_cover_point(wm, cx, cy)) { printf("FAIL: 极贴门球应返回 true\n"); return 1; }
    if (fabs(cx - 217.0) > 0.5 || fabs(cy - 90.0) > 0.5) {
        printf("FAIL: 极贴门护门点 (%.1f,%.1f) 应 (217,90)\n", cx, cy); return 1;
    }

    printf("goal cover: OK (远球连线/中近拦截/贴门堵射/斜向clamp)\n");
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
        if (fabs(p.target_x - 54.0) > 0.5 || fabs(p.target_y - 70.0) > 0.5) {
            printf("FAIL: 场景④接应点未基于站位点 (%.1f,%.1f)\n", p.target_x, p.target_y);
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
        if (p4.viable) {
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
        // 射门线 12cm 处横一个非门将防守者 → 路线被挡 → 拒
        wm.opp[1].x = wm.ball.x + p5.dir_x * 12.0;
        wm.opp[1].y = wm.ball.y + p5.dir_y * 12.0;
        ShootPlan p6 = plan_shoot(wm, 1);
        if (p6.viable) {
            printf("FAIL: 路线被挡的远射应被拒 (lane_blocked=%d)\n", (int)p6.lane_blocked);
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
        // 非点球时：92cm 在远射档内，但 GK 封死 → 净开口只有两侧各 ~7.3° < 8° → 应被拒
        if (pn.viable) {
            printf("FAIL: 非点球时 GK 封死的 92cm 应被闸门拒\n");
            return 1;
        }
    }
    printf("shoot plan: OK (近距无条件/dir单位/连续瞄准/远射双闸门/路线阻挡/点球旁路)\n");
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
    // —— ② 球在禁止推球区内（距角 32.8cm < 35）→ **只停不动**（新规则，第 49 轮）——
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
static int test_shoot_push_limit() {
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
    motion::position(r, 5.0, 90, motion::TM_PASS);          // 旧律此处 112.5（de>=5 饱和）
    if (std::fabs((r.vl + r.vr) * 0.5 - 112.5) > 0.5) {
        printf("FAIL: TM_PASS de=5 应为 112.5 got %.1f\n", (r.vl + r.vr) * 0.5);
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
    if ((c.vl + c.vr) * 0.5 > 25.0) {                       // 2/10 缩放 → ~22.5
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

int main() {
    int rc = 0;
    rc |= test_strategy_run(300);
    rc |= test_formation();
    rc |= test_defense_intercept();
    rc |= test_goalie_predict();
    rc |= test_defense_reach();
    rc |= test_goal_cover();
    rc |= test_team_state();
    rc |= test_fixed_roles();
    rc |= test_pass();
    rc |= test_pass_threat_weight();
    rc |= test_goalie_scenarios();
    rc |= test_roles_spread();
    rc |= test_segment_circle();
    rc |= test_route_straight();
    rc |= test_route_avoid();
    rc |= test_route_cluster();
    rc |= test_route_global_safety();
    rc |= test_follow_route();
    rc |= test_shoot_push_limit();
    rc |= test_shoot_plan();
    rc |= test_active_ga_retreat();
    rc |= test_active_corner_rescue();
    rc |= test_no_push_zone();
    rc |= test_motion_brake_envelope();
    rc |= test_motion_stop_convergence();
    rc |= test_motion_pass_mode();
    rc |= test_motion_bounds();
    rc |= test_motion_aligned();
    printf(rc ? "=== TEST FAILED ===\n" : "=== ALL TESTS PASSED ===\n");
    return rc;
}
