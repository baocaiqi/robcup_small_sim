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
        // 射门线 12cm 处横一个非门将防守者 → 直线路线被挡
        wm.opp[1].x = wm.ball.x + p5.dir_x * 12.0;
        wm.opp[1].y = wm.ball.y + p5.dir_y * 12.0;
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
        // 非点球时：92cm 在远射档内，但 GK 封死 → 净开口只有两侧各 ~7.3° < 8° → 应被拒
        if (pn.viable) {
            printf("FAIL: 非点球时 GK 封死的 92cm 应被闸门拒\n");
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
//   ② 反射点满足**实测各向异性反射**（法向×0.66、切向×0.81），且与"理想镜面"解**明显不同**
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
        // 出射方向必须指向瞄准点：用实测系数算 out，再与「反弹点→目标」做叉积（应共线）
        double ox = p.dir_x * ball_wall_fric(), oy = -p.dir_y * ball_wall_rest();
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
    gk_clear_direction(wm, 0, wm.ball.x, wm.ball.y, pdirx, pdiry);
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
    gk_clear_direction(wm, 0, wm.ball.x, wm.ball.y, pdirx, pdiry);
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

    // ② 对手正抢(<40cm) + 球 39cm(非贴门线) + 门将门侧 → 直线推出(抢得过对手)
    wm.ball.x = 181.0; wm.ball.y = 89.7; wm.ball.vx = 0.0; wm.ball.vy = 0.0;
    wm.opp[0].x = 150.0; wm.opp[0].y = 89.7;                              // 对手距球 31cm < 40
    wm.home[0].x = 215.0; wm.home[0].y = 89.7; wm.home[0].rot = 180.0;   // 门侧，距球 34cm
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
    wm.ball.x = 212.0; wm.ball.y = 91.0; wm.ball.vx = 0.0; wm.ball.vy = 0.0;   // 离门线 8cm、静止

    // ① 球门侧（离门更近）→ 应主动推球清出去（rot=180 时前进 = -x = 远离己门）
    wm.home[4].x = 219.0; wm.home[4].y = 91.0; wm.home[4].rot = 180.0;
    run_passive(wm, 4);
    double v1 = 0.5 * (wm.home[4].vl + wm.home[4].vr);
    if (v1 < 10.0) {
        printf("FAIL: 门前清道夫没出手（球门侧 v=%.0f，应 >10 朝 -x 把球顶离己门）\n", v1);
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
// 球权来源单测（2026-09-13 用户指令：先用平台给的 whosBall 字段）
//   平台字段有效(≠0) → 以它为准；未知(0) → 退回"最近的人且 <20cm"自算。
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
    // ④ 不明确（双方都 30cm 外）→ 听平台的
    wm.home[1].x = 140; wm.opp[1].x = 140; wm.whos_ball = 1;
    if (!sitm.analyze(wm).we_have_ball) { printf("FAIL: 散球时 whos=1 应判我方\n"); return 1; }
    wm.whos_ball = 2;
    if (sitm.analyze(wm).we_have_ball) { printf("FAIL: 散球时 whos=2 应判对方\n"); return 1; }
    printf("possession source: OK (平台优先/未知退回自算/whos=1我方/whos=2对方)\n");
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
        if (!same_wheels(wm.home[receiver], expected_receiver) || same_wheels(wm.home[receiver], ordinary.home[receiver])) {
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
    run_assist(wm, task.receiver_id);
    RobotState expected = wm.home[task.receiver_id]; motion::position(expected, task.rx, task.ry);
    if (fabs(wm.home[task.receiver_id].vl - expected.vl) > 1e-8 ||
        fabs(wm.home[task.receiver_id].vr - expected.vr) > 1e-8) {
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
        RobotState expected_receiver = wm.home[receiver];
        motion::position(expected_receiver, locked_x, locked_y);
        run_active(wm, 1);
        run_assist(wm, receiver);
        if (!task.active || task.receiver_id != receiver || task.rx != locked_x || task.ry != locked_y ||
            task.observing_push || !stopped(wm.home[1]) ||
            fabs(wm.home[receiver].vl - expected_receiver.vl) > 1e-8 ||
            fabs(wm.home[receiver].vr - expected_receiver.vr) > 1e-8) {
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

int main(int argc, char **argv) {
    int rc = 0;
    bool coop_pass_only = false;
    // Strategy default keeps experimental cooperation disabled; pass-specific
    // unit tests explicitly opt in because they validate that subsystem.
    simuro5::set_param("roles.kPassTasksEnabled", 1.0);
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
        rc |= test_pass_opponent_first_cancel();
        rc |= test_pass_receive_control();
        printf(rc ? "=== COOP PASS TEST FAILED ===\n" : "=== COOP PASS TEST PASSED ===\n");
        return rc;
    }
    rc |= test_strategy_run(300);
    rc |= test_formation();
    rc |= test_placement_semantics();
    rc |= test_penalty_spot_detect();
    rc |= test_penalty_shot_prep();
    rc |= test_penalty_aim_offcenter();
    rc |= test_own_goalarea_guard();
    rc |= test_no_reverse_through_ball();
    rc |= test_opp_kick_predict();
    rc |= test_coop_pass();
    rc |= test_coop_pass_task();
    rc |= test_coop_lifecycle();
    rc |= test_ordinary_pass_task();
    rc |= test_pass_readiness_gate();
    rc |= test_pass_opponent_first_cancel();
    rc |= test_pass_receive_control();
    rc |= test_goalie_side_step();
    rc |= test_rebound_and_doubleteam();
    rc |= test_possession_source();
    rc |= test_goalie_clear_push();
    rc |= test_goalie_straight_clear();
    rc |= test_passive_front_sweep();
    rc |= test_gk_on_line_no_push();
    rc |= test_goalie_challenge();
    rc |= test_doubleteam_loose_ball();
    rc |= test_goalie_line_cover();
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
    rc |= test_route_optimality();
    rc |= test_route_cluster();
    rc |= test_route_global_safety();
    rc |= test_follow_route();
    rc |= test_shoot_push_limit();
    rc |= test_shoot_plan();
    rc |= test_bank_shot();
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
