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

    // —— P0-2 连续威胁断言：球越靠己方门威胁越高；对方贴球比球孤立威胁高 ——
    {
        // 罚球区外（x∈[120,130]，己方半场）测单调性，避开罚球区 0.9 保底
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 120; wm.opp[i].y = 90; }
        wm.ball.x = 120; wm.ball.y = 90;
        strat.run(wm);
        double t_far = wm.threat_level;
        wm.ball.x = 130; wm.ball.y = 90;            // 距门更近 10cm
        strat.run(wm);
        double t_near = wm.threat_level;
        if (t_near <= t_far) {
            printf("FAIL: 球越靠己方门威胁应越高 (近 %.2f vs 远 %.2f)\n", t_near, t_far);
            return 1;
        }
        // 对手全部远离（球孤立）→ 威胁应明显低于对方贴球同位置
        for (int i = 0; i < 5; ++i) { wm.opp[i].x = 40; wm.opp[i].y = 90; }
        strat.run(wm);
        double t_lone = wm.threat_level;
        if (t_lone >= t_near) {
            printf("FAIL: 球孤立威胁应低于对方贴球 (孤立 %.2f vs 贴球 %.2f)\n", t_lone, t_near);
            return 1;
        }
    }

    printf("team state: OK (滞回防抖/事件标志/威胁分级连续化)\n");
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

    // 场景⑤（P0-1）：持球者被围（周围 25cm ≥2 对手）→ 出球从"最靠前"切"最近安全点"。
    //   持球者(100,90)，demo 上下贴球(100,66)/(100,114)；ASSIST 站位点在前(82,90)
    //   （接应点 76,90，更靠对方门=旧评分胜出方），MIDFIELD 站位点侧后(118,90)
    //   （接应点 112,90，更近）。被围后应选近的 MIDFIELD——侧后短传出球，不再死带。
    wm.role[0] = ROLE_GOALIE;
    wm.role[1] = ROLE_ACTIVE;
    wm.role[2] = ROLE_ASSIST;
    wm.role[3] = ROLE_MIDFIELD;
    wm.role[4] = ROLE_PASSIVE;
    wm.home[0].x = 210; wm.home[0].y = 90;
    wm.home[1].x = 100; wm.home[1].y = 90;   // 持球者(ACTIVE)
    wm.home[2].x = 150; wm.home[2].y = 90;   // ASSIST 本体远，用站位点
    wm.home[3].x = 150; wm.home[3].y = 150;  // MIDFIELD 本体远，用站位点
    wm.home[4].x = 150; wm.home[4].y = 30;
    wm.assist_x = 82;  wm.assist_y = 90;     // 前方接应点(76,90) 距 24cm
    wm.mid_x = 118;    wm.mid_y = 90;        // 侧后接应点(112,90) 距 12cm
    wm.passive_x = 150; wm.passive_y = 30;
    wm.opp[0].x = 100; wm.opp[0].y = 66;     // 上下贴球：swarm=2 → 被围
    wm.opp[1].x = 100; wm.opp[1].y = 114;
    wm.opp[2].x = 200; wm.opp[2].y = 30;
    wm.opp[3].x = 200; wm.opp[3].y = 90;
    wm.opp[4].x = 200; wm.opp[4].y = 150;
    {
        PassPlan p = plan_pass(wm, 1);
        if (!p.viable || p.receiver_id != 3) {
            printf("FAIL: 场景⑤被围应选近侧后点MID(home[3]) got viable=%d recv=%d\n",
                   p.viable, p.receiver_id);
            return 1;
        }
        if (fabs(p.target_x - 112.0) > 0.5 || fabs(p.target_y - 90.0) > 0.5) {
            printf("FAIL: 场景⑤被围接应点应为(112,90) got (%.1f,%.1f)\n", p.target_x, p.target_y);
            return 1;
        }
    }

    printf("pass: OK (威胁惩罚/边界夹取/短传优先/联动站位点/被围近点)\n");
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
    wm.ball.x = 25; wm.ball.y = 90;      // 球在对方门前 25cm（dgoal<70 → 可射）
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
    }
    // GK 偏上(y=105 贴上柱) → 应选下开口 74
    wm.opp[0].y = 105;
    {
        ShootPlan p2 = plan_shoot(wm, 1);
        if (!p2.viable || fabs(p2.aim_y - 74.0) > 0.5) {
            printf("FAIL: GK 偏上应瞄下开口 got aim=%.1f viable=%d\n", p2.aim_y, p2.viable);
            return 1;
        }
    }
    printf("shoot plan: OK (可射/dir单位/推球点/GK远侧开口)\n");
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

// 角区救球单测（docs/06 第 11 轮）：
//   球卡角区外环(距角 22~30cm)且静止 >30 帧 → ACTIVE 两段式救球（直线穿过球推向场心）；
//   球压到角心(禁止推球区，距角 <22cm) → 不救（推球=犯规，等平台判僵局重置）
static int test_active_corner_rescue() {
    TeamContext ctx{true};
    WorldModel wm;
    wm.ctx = ctx;
    wm.ball.valid = true;
    wm.ball.x = 26; wm.ball.y = 20;      // 左下角外环（距角 26cm，22~30cm 之间）
    wm.ball.vx = 0.0; wm.ball.vy = 0.0;  // 静止
    for (int i = 0; i < 5; ++i) { wm.home[i].x = 150; wm.home[i].y = 30 + i * 20; }
    for (int i = 0; i < 5; ++i) { wm.opp[i].x = 200; wm.opp[i].y = 30 + i * 20; }
    wm.home[1].x = 18; wm.home[1].y = 12; wm.home[1].rot = 45;   // 已对准球 → 直线推穿段
    wm.corner_ball_frames = 31;          // 卡住 >30 帧
    wm.active_ga_frames = 0;
    run_active(wm, 1);
    // 推穿目标 = 球 + 场心方向*30 ≈ (49.1, 39.2) → 机器人应朝 +x,+y 移动
    if (!(wm.home[1].vl > 0.0 && wm.home[1].vr > 0.0) ||
        fmax(fabs(wm.home[1].vl), fabs(wm.home[1].vr)) > 300.0) {
        printf("FAIL: 角区外环救球应朝场内推 got vl=%.1f vr=%.1f\n", wm.home[1].vl, wm.home[1].vr);
        return 1;
    }
    // 球压到角心（禁止推球区）→ 不救球（推球=犯规）；正常逻辑是追球（朝球移动，不朝场心推穿）
    wm.ball.x = 8; wm.ball.y = 8;
    wm.home[1].x = 30; wm.home[1].y = 30; wm.home[1].rot = 0;
    run_active(wm, 1);
    double vl_deep = wm.home[1].vl, vr_deep = wm.home[1].vr;
    // 角心不救：不应出现"朝场心(+x,+y)推穿"的运动；正常逻辑是向球(左下)追，vl/vr 应 <0
    bool deep_rescued = (vl_deep > 0.0 && vr_deep > 0.0);
    if (deep_rescued) {
        printf("FAIL: 禁止推球区角心不应救球 got vl=%.1f vr=%.1f\n", vl_deep, vr_deep);
        return 1;
    }
    // 球不在角区 → 不触发救球（走正常逻辑，不崩溃）
    wm.ball.x = 60; wm.ball.y = 90;
    wm.home[1].x = 60; wm.home[1].y = 60;
    run_active(wm, 1);
    printf("active corner rescue: OK (外环静止球被推向场内/角心不救/非角区不触发)\n");
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
    rc |= test_shoot_plan();
    rc |= test_active_ga_retreat();
    rc |= test_active_corner_rescue();
    printf(rc ? "=== TEST FAILED ===\n" : "=== ALL TESTS PASSED ===\n");
    return rc;
}
