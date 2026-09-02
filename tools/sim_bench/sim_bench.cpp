// ============================================================
// sim_bench.cpp — 快速无头仿真器（训练量/回归测试/参数搜索用）
//
// 目标：一场完整比赛（600s×40Hz=24000 帧）压缩到 ~0.2 秒，
//       跑 N 场输出比分/控球/射门统计 —— "大量样本"的地基
//       （参考 refs：FRA-UNIted 动力学重实现 0.137µs/转移、RoboCIn 500 场 CI）
//
// 物理为简化模型（可调，见常量区）：
//   · 机器人：差速轮 vl/vr → 线速度 v=(vl+vr)/2*kSpeed，角速度 w=(vr-vl)/WHEEL_BASE
//     —— 带加速度限制 kAccel（真实平台有惯性，否则追球过冲把球铲偏——本 sim 主要失真源）
//   · 球：摩擦衰减 + 撞墙反弹(垂直速度骤减, 2007 论文怪癖) + 与机器人交互
//   · 球-机器人交互：
//       a) 携带：球在非守门员机器人前方弧区(≤kCarryR 且与速度方向夹角≤kCarryArc)，
//          球速 = 旧速×0.3 + 机器人速度×0.7（推球有动量，射门才可能）
//       b) 挡球：任何机器人(含守门员)与球体相碰 → 径向反弹（守门员是"挡"不是"带"）
//   · 进球：球整体越门线且 y∈[70,110]（蓝守 x=220 / 黄守 x=0）
//   · 僵局：60 帧内球位移 <25cm → 判争球重置中圈（平台同款，防球卡角）
//
// 用法：
//   sim_bench.exe --games 100 --opp scripted   策略 vs 脚本对手（默认）
//   sim_bench.exe --games 100 --opp self       自我博弈（同策略对打）
//   sim_bench.exe --games 20  --frames 6000    每场只跑 6000 帧（快速冒烟）
//   sim_bench.exe --seed 42                    固定随机种子（可复现实验）
// ============================================================
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <chrono>
#include "simuro5/simuro_interface.hpp"
#include "simuro5/team.hpp"
#include "simuro5/world_model.hpp"
#include "simuro5/strategy.hpp"

using namespace simuro5;

// 简单确定性 PRNG（xorshift64*）：可复现、够快
struct Rng {
    uint64_t s = 0x9E3779B97F4A7C15ull;
    explicit Rng(uint64_t seed) { s = seed ? seed : 0x9E3779B97F4A7C15ull; }
    uint64_t next() {
        uint64_t x = s;
        x ^= x >> 12; x ^= x << 25; x ^= x >> 27;
        s = x;
        return x * 0x2545F4914F6CDD1Dull;
    }
    double unit() { return (double)(next() >> 11) / (double)(1ull << 53); }   // [0,1)
    double range(double a, double b) { return a + (b - a) * unit(); }
};

// ==================== 简化物理常量（可调） ====================
static constexpr double kDt        = 1.0 / 40.0;   // 40Hz
static constexpr double kSpeed     = 0.9;          // 轮速→cm/s 缩放
static constexpr double kWheelBase = 10.0;         // 轮距 cm（决定转向灵敏度）
static constexpr double kAccel     = 300.0;        // 轮速最大加速度 cm/s²（每帧 ±7.5）
                                                   //   真实平台有惯性：机器人不能瞬间 0→112，
                                                   //   否则追球过冲把球铲向错误方向（本 sim 主要失真源）
static constexpr double kBallDecay = 0.985;        // 球每帧摩擦衰减（校准: 真实rlg高速段 0.986-0.994/帧，一致）
static constexpr double kWallRest  = 0.45;         // 撞墙反弹恢复系数（校准: 真实rlg x=0.458 y=0.449）
static constexpr double kWallFric  = 0.90;         // 撞墙时平行分量衰减（墙摩擦）：球贴墙滑动会减速
                                                   //   ——否则球沿墙滑 vy 不降，一路滑进角落/门角（2007论文怪癖: 球卡四角）
static constexpr double kContact   = 5.5;          // 球-机器人最小分离 cm（防球嵌进机器人身体）
static constexpr double kCarryR    = 9.0;          // 携带区半径 cm：略大于策略"球后 8cm 推球点"，
                                                   //   机器人站到推球点即可带球；但防止提前携带
                                                   //   （接近途中 9-12cm 就带球会把球斜推偏，射门失效）
static constexpr double kCarryArc  = 40.0;         // 携带区前向半弧（度）：球几乎在正前方才携带，
                                                   //   防止机器人斜向接近时把球"斜推"带偏射门方向
static constexpr double kDeflect   = 0.45;         // 守门员挡球反弹恢复系数（挡不是带）
static constexpr double kRobotR    = 6.0;          // 机器人-机器人最小间距 cm
static constexpr double kGoalLo = 70.0, kGoalHi = 110.0;  // 门宽

struct SimRobot { double x=0, y=0, rot=0, vl=0, vr=0, pl=0, pr=0; };

struct SimState {
    SimRobot blue[5], yellow[5];
    double bx = 110, by = 90, bvx = 0, bvy = 0;
    double p_bx = 110, p_by = 90;      // 上一帧球位（WorldModel 用它算球速）
    int score_blue = 0, score_yellow = 0;
    int frames = 0;
    long poss_blue = 0, poss_yellow = 0;   // 球权帧数（距球<15cm）
    int shots_blue = 0, shots_yellow = 0;  // 射门（球进入对方门区 30cm 内）
    long zone_blue_third = 0, zone_mid = 0, zone_yellow_third = 0;  // 球位分布
    long ga_we_frames = 0, ga_we_episodes = 0, pa_we_frames = 0;   // 禁区纪律(我们)
    bool prev_ga_viol = false;               // 上一帧是否门区违规（片段计数用）
    // 单人停留>20帧（FIRA：门区除门将外停留>20周期 → 罚点球；docs/13 方案 C 场景）
    int solo_cnt[4] = {0};                   // 各非门将机器人在对方门区连续静止停留帧数
    double solo_prevx[5] = {0}, solo_prevy[5] = {0};   // 单人停留统计的上一帧位置（测活动度）
    long ga_solo_frames = 0, ga_solo_episodes = 0;
    bool prev_solo_viol = false;
    long freeball_count = 0;             // 僵局重置次数（球卡角/停死 → 判争球，真实平台 FreeBall 13 次/场）
    long freeball_corner = 0;            // 其中：球在角区(距角<30cm)的重置次数
    long corner_rescue = 0;              // 我方 ACTIVE 角区救球触发次数（world_model 统计）
    double check_x = 110, check_y = 90;   // 僵局检测：每 60 帧对比球位移
    int check_cnt = 0;
    int dbg_contacts = 0;              // 接触事件日志开关（调试用）
};

static double deg2rad(double d) { return d * 3.14159265358979 / 180.0; }

// 初始摆位（简单开局阵型）；seed 用于引入摆位微扰（模拟真实开局差异）
static void init_formation(SimState &s, Rng *rng = nullptr) {
    // 蓝队守 x=220
    double bxs[5] = {215, 185, 150, 150, 120};
    double bys[5] = {90, 90, 60, 120, 90};
    for (int i = 0; i < 5; ++i) {
        s.blue[i].x = bxs[i]; s.blue[i].y = bys[i]; s.blue[i].rot = 180;
        s.blue[i].vl = s.blue[i].vr = 0; s.blue[i].pl = s.blue[i].pr = 0;
    }
    // 黄队守 x=0
    double yxs[5] = {5, 35, 70, 70, 100};
    double yys[5] = {90, 90, 60, 120, 90};
    for (int i = 0; i < 5; ++i) {
        s.yellow[i].x = yxs[i]; s.yellow[i].y = yys[i]; s.yellow[i].rot = 0;
        s.yellow[i].vl = s.yellow[i].vr = 0; s.yellow[i].pl = s.yellow[i].pr = 0;
    }
    s.bx = 110; s.by = 90; s.bvx = s.bvy = 0;
    s.p_bx = 110; s.p_by = 90;
    // 摆位微扰 ±3cm（守门员除外）：让每场比赛过程不完全相同
    if (rng) {
        for (int i = 1; i < 5; ++i) {
            s.blue[i].x += rng->range(-3, 3); s.blue[i].y += rng->range(-3, 3);
            s.yellow[i].x += rng->range(-3, 3); s.yellow[i].y += rng->range(-3, 3);
        }
        s.bx += rng->range(-2, 2); s.by += rng->range(-2, 2);
    }
}

// 把 SimState 填进 Environment（给 WorldModel::update 用）
static void fill_env(Environment &e, const SimState &s, bool blue_side) {
    memset(&e, 0, sizeof(e));
    e.fieldBounds.left = 0; e.fieldBounds.right = 220;
    e.fieldBounds.bottom = 0; e.fieldBounds.top = 180;
    e.goalBounds.left = 0; e.goalBounds.right = 220;
    e.goalBounds.bottom = 70; e.goalBounds.top = 110;
    e.currentBall.pos.x = s.bx; e.currentBall.pos.y = s.by; e.currentBall.pos.z = 0;
    e.lastBall.pos.x = s.p_bx; e.lastBall.pos.y = s.p_by; e.lastBall.pos.z = 0;
    e.predictedBall.pos = e.currentBall.pos;
    e.gameState = PM_PlayOn;
    const SimRobot *home = blue_side ? s.blue : s.yellow;
    const SimRobot *opp  = blue_side ? s.yellow : s.blue;
    for (int i = 0; i < 5; ++i) {
        e.home[i].pos.x = home[i].x; e.home[i].pos.y = home[i].y; e.home[i].rotation = home[i].rot;
        e.opponent[i].pos.x = opp[i].x; e.opponent[i].pos.y = opp[i].y; e.opponent[i].rotation = opp[i].rot;
    }
}

// 一帧物理推进
static void step_physics(SimState &s) {
    // 先记录决策时的速度向量（用 move 前的 rot 计算）——携带推球方向必须基于
    // 策略看到的朝向，否则机器人中途转向站位点时会把球"铲"错方向
    double bvx5[5], bvy5[5], yvx5[5], yvy5[5];
    for (int i = 0; i < 5; ++i) {
        double rad = deg2rad(s.blue[i].rot);
        double v = (s.blue[i].vl + s.blue[i].vr) * 0.5 * kSpeed;
        bvx5[i] = v * std::cos(rad); bvy5[i] = v * std::sin(rad);
        rad = deg2rad(s.yellow[i].rot);
        v = (s.yellow[i].vl + s.yellow[i].vr) * 0.5 * kSpeed;
        yvx5[i] = v * std::cos(rad); yvy5[i] = v * std::sin(rad);
    }
    // 机器人运动（差速轮），带加速度限制（模拟真实平台惯性，防追球过冲铲球）
    auto move = [&](SimRobot &r) {
        // 轮速受加速度限制：每帧最多变化 kAccel * kDt
        double maxdv = kAccel * kDt;
        double nvl = r.vl, nvr = r.vr;
        if (nvl > r.pl) { if (nvl - r.pl > maxdv) nvl = r.pl + maxdv; }
        else           { if (r.pl - nvl > maxdv) nvl = r.pl - maxdv; }
        if (nvr > r.pr) { if (nvr - r.pr > maxdv) nvr = r.pr + maxdv; }
        else           { if (r.pr - nvr > maxdv) nvr = r.pr - maxdv; }
        r.pl = nvl; r.pr = nvr; r.vl = nvl; r.vr = nvr;

        double v = (r.vl + r.vr) * 0.5 * kSpeed;              // cm/s
        double w = (r.vr - r.vl) / kWheelBase;                // rad/s
        double rad = deg2rad(r.rot);
        r.x += v * std::cos(rad) * kDt;
        r.y += v * std::sin(rad) * kDt;
        r.rot += w * kDt * 180.0 / 3.14159265358979;
        // 夹回场内
        if (r.x < kRobotR) r.x = kRobotR; if (r.x > 220 - kRobotR) r.x = 220 - kRobotR;
        if (r.y < kRobotR) r.y = kRobotR; if (r.y > 180 - kRobotR) r.y = 180 - kRobotR;
    };
    for (int i = 0; i < 5; ++i) { move(s.blue[i]); move(s.yellow[i]); }

    // 机器人-机器人分离（防叠位）
    for (int a = 0; a < 5; ++a) for (int b = a + 1; b < 5; ++b) {
        double dx = s.blue[b].x - s.blue[a].x, dy = s.blue[b].y - s.blue[a].y;
        double d = std::hypot(dx, dy);
        if (d < kRobotR && d > 1e-6) {
            double push = (kRobotR - d) * 0.5;
            s.blue[a].x -= dx / d * push; s.blue[a].y -= dy / d * push;
            s.blue[b].x += dx / d * push; s.blue[b].y += dy / d * push;
        }
    }
    for (int a = 0; a < 5; ++a) for (int b = 0; b < 5; ++b) {
        double dx = s.yellow[b].x - s.blue[a].x, dy = s.yellow[b].y - s.blue[a].y;
        double d = std::hypot(dx, dy);
        if (d < kRobotR && d > 1e-6) {
            double push = (kRobotR - d) * 0.5;
            s.blue[a].x -= dx / d * push; s.blue[a].y -= dy / d * push;
            s.yellow[b].x += dx / d * push; s.yellow[b].y += dy / d * push;
        }
    }

    // 球运动
    s.bx += s.bvx * kDt;
    s.by += s.bvy * kDt;
    s.bvx *= kBallDecay; s.bvy *= kBallDecay;

    // 球-墙反弹（垂直分量衰减 + 平行分量摩擦衰减）；门线开口处（y∈门宽）不反弹——球要能进门！
    if (s.bx < 0) {
        if (s.by >= kGoalLo && s.by <= kGoalHi) { /* 进门：交给 check_goal 判定 */ }
        else { s.bx = -s.bx; s.bvx = -s.bvx * kWallRest; s.bvy *= kWallFric; }
    }
    if (s.bx > 220) {
        if (s.by >= kGoalLo && s.by <= kGoalHi) { /* 进门 */ }
        else { s.bx = 440 - s.bx; s.bvx = -s.bvx * kWallRest; s.bvy *= kWallFric; }
    }
    if (s.by < 0) { s.by = -s.by; s.bvy = -s.bvy * kWallRest; s.bvx *= kWallFric; }
    if (s.by > 180) { s.by = 360 - s.by; s.bvy = -s.bvy * kWallRest; s.bvx *= kWallFric; }

    // —— 球-机器人交互 ——
    // 1) 携带：球在「非守门员」机器人前方弧区（距离≤kCarryR 且 |偏角|≤kCarryArc）内，
    //    球速 = 旧速×0.3 + 载体速度×0.7（快推产生动量，射门才可能）。
    //    kCarryR=12 覆盖策略的「球后 8cm 推球点」：机器人停在推球点即进入携带区，带球才发生。
    // 2) 挡球/侧面接触：任何机器人（含守门员）与球体相碰（<kContact）→ 径向反弹，
    //    守门员是"挡"不是"带"，球永远不会被门将吸走。
    int carry_i = -1; bool carry_blue = false; double carry_d = 1e9;
    // 前方弧判断：球相对机器人，位于决策时速度方向前方（运动方向 + 球 才携带，
    // 避免"路过"机器人（向别处移动）把球铲偏）
    auto front_arc = [&](const SimRobot &r, double dx, double dy, double rvx, double rvy) {
        (void)r;
        double mv = std::hypot(rvx, rvy);
        if (mv < 1e-6) return false;                 // 静止不携带
        double dot = (dx * rvx + dy * rvy) / (std::hypot(dx, dy) * mv);
        return dot >= std::cos(deg2rad(kCarryArc));  // 夹角 ≤ kCarryArc
    };
    for (int t = 0; t < 2; ++t) {
        for (int i = 1; i < 5; ++i) {          // 0 号守门员不携带
            const SimRobot &r = t ? s.yellow[i] : s.blue[i];
            double dx = s.bx - r.x, dy = s.by - r.y;
            double d = std::hypot(dx, dy);
            double rvx = t ? yvx5[i] : bvx5[i];
            double rvy = t ? yvy5[i] : bvy5[i];
            if (d < kCarryR && d < carry_d && front_arc(r, dx, dy, rvx, rvy)) {
                carry_d = d; carry_i = i; carry_blue = (t == 0);
            }
        }
    }
    if (carry_i >= 0) {
        // 用决策时的速度向量（机器人 move 前朝向），而不是 move 后 rot——
        // 否则机器人转向站位点时会把球铲向错误方向
        double rvx = carry_blue ? bvx5[carry_i] : yvx5[carry_i];
        double rvy = carry_blue ? bvy5[carry_i] : yvy5[carry_i];
        double rv = std::hypot(rvx, rvy);
        double bv = std::hypot(s.bvx, s.bvy);              // 球当前速度
        // 携带+惯性：球速 = 旧速×0.3 + 机器人速度×0.7。
        // 机器人推球时球获得动量（射门才有"射出感"），脱离接触后自由滚动衰减。
        // 窄携带区(9cm/40°)保证只有"对准球门方向推"时才携带，不会提前斜推。
        if (rv > bv * 0.9) {
            if (s.dbg_contacts) printf("  [接触] %s%d 携带推球 rv%.0f\n",
                                       carry_blue ? "蓝" : "黄", carry_i, rv);
            s.bvx = s.bvx * 0.3 + rvx * 0.7;
            s.bvy = s.bvy * 0.3 + rvy * 0.7;
        }
        // 球保持在机器人前方接触区（防球钻进机器人身体）
        const SimRobot &c = carry_blue ? s.blue[carry_i] : s.yellow[carry_i];
        double dx = s.bx - c.x, dy = s.by - c.y;
        double d = std::hypot(dx, dy);
        if (d > 1e-6 && d < kContact * 0.5) {
            double sep = (kContact * 0.5 - d) * 0.5;
            s.bx += dx / d * sep; s.by += dy / d * sep;
        }
    }
    // 2) 所有机器人径向弹开（守门员挡球 + 侧面碰撞），携带者跳过（已处理）
    for (int t = 0; t < 2; ++t) {
        for (int i = 0; i < 5; ++i) {
            if (carry_i >= 0 && i == carry_i && (t == 0) == carry_blue) continue;
            const SimRobot &r = t ? s.yellow[i] : s.blue[i];
            double dx = s.bx - r.x, dy = s.by - r.y;
            double d = std::hypot(dx, dy);
            if (d >= kContact || d < 1e-6) continue;
            double nx = dx / d, ny = dy / d;
            double rvx = t ? yvx5[i] : bvx5[i];              // 决策时速度向量
            double rvy = t ? yvy5[i] : bvy5[i];
            double rel_vn = (s.bvx - rvx) * nx + (s.bvy - rvy) * ny;   // 球相对机器人沿法线接近速度
            if (rel_vn < 0) {                                          // 球朝机器人运动才反弹
                if (s.dbg_contacts) printf("  [碰撞] %s%d 挡球 v(%.0f,%.0f)→", t ? "黄" : "蓝", i, s.bvx, s.bvy);
                s.bvx -= (1.0 + kDeflect) * rel_vn * nx;
                s.bvy -= (1.0 + kDeflect) * rel_vn * ny;
                if (s.dbg_contacts) printf("(%.0f,%.0f) 球(%.0f,%.0f) 机(%.0f,%.0f)\n",
                                           s.bvx, s.bvy, s.bx, s.by, r.x, r.y);
            }
            double push = kContact - d;                                // 推出身体防钻入
            s.bx += nx * push; s.by += ny * push;
        }
    }
}

// 进球判定 + 重置（带随机摆位）；debug>0 时打印进球详情
static bool check_goal(SimState &s, Rng *rng, int debug) {
    if (s.bx > 220 && s.by >= kGoalLo && s.by <= kGoalHi) {   // 蓝队失球(黄得分)
        s.score_yellow++;
        if (debug) printf("  [失球] 帧%d 蓝失: 球(%.0f,%.0f)v(%.0f,%.0f) 门将(%.0f,%.0f) 蓝1(%.0f,%.0f) 黄近球(%.0f,%.0f)\n",
                          s.frames, s.bx, s.by, s.bvx, s.bvy, s.blue[0].x, s.blue[0].y,
                          s.blue[1].x, s.blue[1].y,
                          s.yellow[0].x, s.yellow[0].y);
        init_formation(s, rng); return true;
    }
    if (s.bx < 0 && s.by >= kGoalLo && s.by <= kGoalHi) {     // 黄队失球(蓝得分)
        s.score_blue++;
        if (debug) printf("  [进球] 帧%d 蓝进: 球(%.0f,%.0f)\n", s.frames, s.bx, s.by);
        init_formation(s, rng); return true;
    }
    return false;
}

// 脚本对手：黄队——0 号守门；1 台追球（最近者），1 台协防，其余站中场阵型
// 避免「4 台全追球」的蜂群压制（会无限触发我方围困检测导致僵局）
// 脚本对手（镜像版）：is_blue=true 时操作蓝队(守 x=220)，false 时操作黄队(守 x=0)。
// 逻辑与攻防方向按守卫侧镜像，保证两侧强度一致（用于测"我们打黄队侧"的半场对称性）。
static void scripted_opponent(SimState &s, bool is_blue, double strength) {
    auto drive = [](SimRobot &r, double tx, double ty, double spd) {
        double dx = tx - r.x, dy = ty - r.y;
        double d = std::hypot(dx, dy);
        if (d < 1e-6) { r.vl = 0; r.vr = 0; return; }
        double want = std::atan2(dy, dx) * 180.0 / 3.14159265358979;
        double te = want - r.rot; while (te > 180) te -= 360; while (te < -180) te += 360;
        double v = spd * (d > 8 ? 1.0 : d / 8.0);
        double ka = 0.8;
        r.vl = v - ka * te; r.vr = v + ka * te;
    };
    SimRobot *R = is_blue ? s.blue : s.yellow;   // 脚本队机器人
    double gx_line = is_blue ? 220.0 : 0.0;      // 守门线
    double gx_in = is_blue ? 212.0 : 8.0;        // 门将 x
    double att_sign = is_blue ? -1.0 : 1.0;      // 进攻方向：蓝守右→攻左(-x)；黄守左→攻右(+x)
    double field_w = 220.0;
    // 守门员：球进本方禁区附近才快速跟球 y；球远时回中待命。
    // 校准自真实 demo 门将：横向峰值 p90≈47cm/s 但有明显失误（真实跟球 y 差 p50=6.4cm、
    // 25% 时间离球>10cm），脚本门将若完美跟球会封死球门导致进球虚低。
    // 基础档 30cm/s + 0.3s 反应延迟（12帧）模拟 demo；strength 放大速度/缩小反应间隔。
    SimRobot &gk = R[0];
    static int gk_react = 0;
    static double gk_target = 90.0;
    // 门将目标：球在本方半场才跟 y（蓝守右 → 球 x>160；黄守左 → 球 x<60）
    bool ball_in_own_half = is_blue ? (s.bx > field_w - 60.0) : (s.bx < 60.0);
    int react_gap = (int)(12.0 / std::min(strength, 3.0) + 0.5);
    if (s.frames % react_gap == 0) {             // 每 0.3s/strength 才重新瞄球
        gk_target = ball_in_own_half ? (s.by < 70 ? 70.0 : (s.by > 110 ? 110.0 : s.by)) : 90.0;
        gk_react = 0;
    }
    double dy = gk_target - gk.y;
    double maxdy = 30.0 * strength / 40.0;       // 30*strength cm/s 横向限速
    if (dy > maxdy) dy = maxdy; else if (dy < -maxdy) dy = -maxdy;
    gk.y += dy;
    gk.x = gx_in;                                // 门线站位固定
    gk.rot = is_blue ? 180.0 : 0.0;              // 面向场内
    gk.vl = gk.vr = 0;                           // 位置直接控制，不走差速
    // 找离球最近的追击手 + 次近协防
    int chaser = 1, support = 2;
    double best = 1e9, second = 1e9;
    for (int i = 1; i < 5; ++i) {
        double d = std::hypot(s.bx - R[i].x, s.by - R[i].y);
        if (d < best) { second = best; support = chaser; best = d; chaser = i; }
        else if (d < second) { second = d; support = i; }
    }
    for (int i = 1; i < 5; ++i) {
        if (i == chaser) {
            double db = std::hypot(s.bx - R[i].x, s.by - R[i].y);
            // 追击手模拟真实 demo：带球质量低（推球点不准+速度慢），球易被碰丢
            //  —— 真实 demo 场均只进 0.8 球；strength↑ → 抖动↓、近球减速↓（带球更稳）
            double wob = 14.0 / std::min(strength, 3.0) * std::sin(s.frames * 0.07 + i * 2.4);
            double spd_near = 30.0 + 25.0 * (strength - 1.0);
            double spd = (db < 20.0) ? spd_near : (80.0 * (0.6 + 0.4 * strength));
            // 站球后推球（朝对方球门方向）：推球点 = 球后方 6cm = 靠己方门一侧。
            drive(R[i], s.bx - att_sign * 6.0 + wob * 0.6, s.by + wob, spd);
        } else if (i == support) {
            // 协防：站到球与己方球门连线 40% 处（截击传球路线），不直接贴球。
            double mx = is_blue ? (s.bx + 220.0) * 0.6 : (s.bx + 0.0) * 0.4;
            double my = (s.by + 90.0) * 0.5;
            drive(R[i], mx, my, 50);
        } else {
            // 阵型站位：中线散开（防反击）；站位点避开球
            double sx = (is_blue ? 125.0 - (i - 1) * 8.0 : 95.0 + (i - 1) * 8.0);
            double sy = (i == 3) ? 50.0 : 130.0;
            double d2b = std::hypot(s.bx - R[i].x, s.by - R[i].y);
            if (d2b < 25.0) {
                double ax = R[i].x - (s.bx - R[i].x);
                double ay = R[i].y - (s.by - R[i].y);
                double al = std::hypot(ax - R[i].x, ay - R[i].y);
                if (al > 1e-6) { sx = R[i].x + (ax - R[i].x) / al * 20.0;
                                 sy = R[i].y + (ay - R[i].y) / al * 20.0; }
                sx = std::min(std::max(sx, 15.0), 205.0); sy = std::min(std::max(sy, 15.0), 165.0);
            }
            drive(R[i], sx, sy, 40);
        }
    }
}

// 一场比赛
// opp_mode: 0=脚本对手打黄队(我们守x=220, 默认)  1=自我博弈  2=脚本对手打蓝队(我们守x=0, 测半场对称)
static void play_match(int frames, int opp_mode, int debug, double opp_strength, Rng &rng, long &r_blue, long &r_yellow,
                       double &r_poss, int &r_shots, long r_zones[3], long &r_ga_frames, long &r_ga_eps,
                       long &r_ga_solo_frames, long &r_ga_solo_eps, long &r_freeball,
                       long &r_freeball_corner, long &r_corner_rescue) {
    SimState s;
    init_formation(s, &rng);
    TeamContext ctx_blue{true}, ctx_yellow{false};
    WorldModel wm_b, wm_y;
    Strategy strat_b, strat_y;
    Environment env_b, env_y;

    for (int f = 0; f < frames; ++f) {
        // 蓝队决策：opp_mode=2 时蓝队是脚本；否则蓝队是我们的策略
        if (opp_mode == 2) {
            scripted_opponent(s, true, opp_strength);
        } else {
            fill_env(env_b, s, true);
            wm_b.update(&env_b, ctx_blue);
            strat_b.run(wm_b);
            for (int i = 0; i < 5; ++i) { s.blue[i].vl = wm_b.home[i].vl; s.blue[i].vr = wm_b.home[i].vr; }
        }

        // 黄队决策：opp_mode=0 时黄队是脚本；否则黄队是我们的策略
        if (opp_mode == 0) {
            scripted_opponent(s, false, opp_strength);
        } else {
            fill_env(env_y, s, false);
            wm_y.update(&env_y, ctx_yellow);
            strat_y.run(wm_y);
            for (int i = 0; i < 5; ++i) { s.yellow[i].vl = wm_y.home[i].vl; s.yellow[i].vr = wm_y.home[i].vr; }
        }

        if (debug && f < debug) {
            s.dbg_contacts = 1;
            printf("f%04d 球(%.0f,%.0f)v(%.1f,%.1f) | 蓝0门(%.0f,%.0f) 蓝1(%.0f,%.0f) 蓝2(%.0f,%.0f) 蓝3(%.0f,%.0f) 蓝4(%.0f,%.0f)\n",
                   f, s.bx, s.by, s.bvx, s.bvy,
                   s.blue[0].x, s.blue[0].y, s.blue[1].x, s.blue[1].y,
                   s.blue[2].x, s.blue[2].y, s.blue[3].x, s.blue[3].y, s.blue[4].x, s.blue[4].y);
            printf("    黄0门(%.0f,%.0f) 黄1(%.0f,%.0f) 黄2(%.0f,%.0f) 黄3(%.0f,%.0f) 黄4(%.0f,%.0f)\n",
                   s.yellow[0].x, s.yellow[0].y,
                   s.yellow[1].x, s.yellow[1].y,
                   s.yellow[2].x, s.yellow[2].y,
                   s.yellow[3].x, s.yellow[3].y,
                   s.yellow[4].x, s.yellow[4].y);
        } else {
            s.dbg_contacts = 0;
        }

        step_physics(s);

        // 记录本帧球位作为下一帧的 lastBall（WorldModel 用差分算球速）
        s.p_bx = s.bx; s.p_by = s.by;

        // 球权统计：谁离球最近算谁控球（阈值 20cm 内；无人区不算）
        double d_b = 1e9, d_y = 1e9;
        for (int i = 0; i < 5; ++i) {
            d_b = std::min(d_b, std::hypot(s.bx - s.blue[i].x, s.by - s.blue[i].y));
            d_y = std::min(d_y, std::hypot(s.bx - s.yellow[i].x, s.by - s.yellow[i].y));
        }
        if (d_b < 20.0 && d_b <= d_y) s.poss_blue++;
        else if (d_y < 20.0 && d_y < d_b) s.poss_yellow++;
        // 射门统计（球进入对方门区 30cm）
        if (s.bx < 30.0 && s.by >= kGoalLo && s.by <= kGoalHi) s.shots_blue++;
        if (s.bx > 190.0 && s.by >= kGoalLo && s.by <= kGoalHi) s.shots_yellow++;

        // 禁区纪律统计（诊断用，不进比分）：
        //   FIRA 规则：进攻方在**对方门区**(球门前50×15)除门将外停留>20帧 或 门区2+人 → 罚点球
        //   ——真实比赛我们场均被罚1.9个点球（禁区聚集），这里看 sim 是否复现同样行为
        {
            // 我们进攻的门区：opp_mode=0/1 我们守x=220攻x=0 → 对方门区在 x≈0
            //   opp_mode=2 我们守x=0攻x=220 → 对方门区在 x≈220
            double opp_ga_x = (opp_mode == 2) ? 220.0 : 0.0;
            // 门区：球门前 50×15（x 以门线为基准向内 50，y 门宽 ±15/2）
            double ga_lo = opp_ga_x == 0.0 ? 0.0 : 220.0 - 50.0;
            double ga_hi = opp_ga_x == 0.0 ? 50.0 : 220.0;
            int blue_in_ga = 0, yellow_in_ga = 0;
            for (int i = 1; i < 5; ++i) {   // 跳过门将(0号)
                double by = s.blue[i].y;
                // 门区判定口径与 field_info.hpp in_goal_area 一致：y ∈ [75,105]（90±15）。
                //   此前用「门宽 70~110 外扩 7.5」的宽框 [62.5,117.5]，会数进门角外
                //   （y<75 或 >105）的非违规帧，且比罚球区还宽，几何上不可能
                //   （官方规则：罚球区 80×35 包含门区 50×15，见 MiroSot Rules 1.1.4/1.1.5）。
                if (s.blue[i].x > ga_lo && s.blue[i].x < ga_hi && by > 75.0 && by < 105.0) blue_in_ga++;
                double yy = s.yellow[i].y;
                if (s.yellow[i].x > ga_lo && s.yellow[i].x < ga_hi && yy > 75.0 && yy < 105.0) yellow_in_ga++;
            }
            // 只有"我们"是 opp_mode==2 ? yellow : blue
            int we_in_ga = opp_mode == 2 ? yellow_in_ga : blue_in_ga;
            bool ga_viol = we_in_ga >= 2;            // 对方门区2+人（会被罚点球）
            if (ga_viol) s.ga_we_frames++;
            if (ga_viol && !s.prev_ga_viol) s.ga_we_episodes++;   // 连续片段计数
            s.prev_ga_viol = ga_viol;

            // 单人停留>20帧（docs/13 方案 C 场景）：我们任一非门将在对方门区
            //   连续停留 >20 帧 → 罚点球（真实平台 8/29 实测 ACTIVE 滞留 21~30 帧被判）
            {
                bool solo_viol = false;
                for (int i = 1; i < 5; ++i) {
                    double sx = (opp_mode == 2) ? s.yellow[i].x : s.blue[i].x;
                    double sy = (opp_mode == 2) ? s.yellow[i].y : s.blue[i].y;
                    if (sx > ga_lo && sx < ga_hi && sy > 75.0 && sy < 105.0) {
                        // 真实平台判"停留"看活动度：高速移动（追球穿过/撤出途中）不算停留。
                        // 实测被判的滞留：位置徘徊 ≤1~2cm/帧（sim 40Hz 帧间）；用 <2cm/帧 过滤。
                        double mdx = sx - s.solo_prevx[i], mdy = sy - s.solo_prevy[i];
                        s.solo_prevx[i] = sx; s.solo_prevy[i] = sy;
                        if (std::hypot(mdx, mdy) < 2.0) {
                            if (++s.solo_cnt[i - 1] > 20) solo_viol = true;
                        } else {
                            s.solo_cnt[i - 1] = 0;   // 快速移动中 = 路过/撤离，重置
                        }
                    } else {
                        s.solo_cnt[i - 1] = 0;
                        s.solo_prevx[i] = sx; s.solo_prevy[i] = sy;
                    }
                }
                if (solo_viol) s.ga_solo_frames++;
                if (solo_viol && !s.prev_solo_viol) s.ga_solo_episodes++;
                s.prev_solo_viol = solo_viol;
            }
        }

        check_goal(s, &rng, debug);
        s.frames++;

        // 僵局规则（平台同款）：60 帧(1.5s)内球位移 <25cm → 判争球重置中圈
        // 防球卡死在墙边/角落（2007 论文怪癖：球常卡进四角）；
        // 仅真正的门线区（y∈门宽）不判僵局（门前混战不算，否则打断进球）；
        // 角落(x<10 或 x>210 且 y<10 或 y>170)照判，否则球卡角永不出来。
        s.check_cnt++;
        if (s.check_cnt >= 60) {
            bool in_goal_mouth = ((s.bx < 40.0 || s.bx > 180.0) && s.by >= kGoalLo && s.by <= kGoalHi);
            // 角落/墙边区：球贴墙缓慢滑动也算卡住（真实平台对卡角判 FreeBall）
            bool in_wall_zone = (s.bx < 15.0 || s.bx > 205.0) || (s.by < 15.0 || s.by > 165.0);
            double stall_dist = in_wall_zone ? 40.0 : 25.0;
            if (!in_goal_mouth && std::hypot(s.bx - s.check_x, s.by - s.check_y) < stall_dist) {
                // 角区卡球（距角 <30cm）：真实平台 FreeBall 主因（右下角卡球无人救）
                if ((s.bx < 30.0 || s.bx > 190.0) && (s.by < 30.0 || s.by > 150.0)) {
                    s.freeball_corner++;
                }
                init_formation(s, &rng);
                s.freeball_count++;        // 计一次"争球重置"（真实平台 FreeBall）
            }
            s.check_x = s.bx; s.check_y = s.by; s.check_cnt = 0;
        }

        // 球位分布（三分之一场：蓝后场/中场/黄后场）——诊断球困在哪
        if (s.bx < 73.0) s.zone_blue_third++;
        else if (s.bx < 147.0) s.zone_mid++;
        else s.zone_yellow_third++;
    }
    // opp_mode=2 时"蓝"=脚本、"黄"=我们：控球/射门统计换边输出
    r_blue = opp_mode == 2 ? s.score_yellow : s.score_blue;
    r_yellow = opp_mode == 2 ? s.score_blue : s.score_yellow;
    double poss_us = opp_mode == 2 ? s.poss_yellow : s.poss_blue;
    double poss_opp = opp_mode == 2 ? s.poss_blue : s.poss_yellow;
    r_poss = poss_us / (poss_us + poss_opp + 1) * 100.0;
    r_shots = opp_mode == 2 ? s.shots_yellow : s.shots_blue;
    // 球位分布：蓝后(x<73)/中/黄后(x>147) 标签不变（与谁是我们无关）
    r_zones[0] = s.zone_blue_third; r_zones[1] = s.zone_mid; r_zones[2] = s.zone_yellow_third;
    r_ga_frames = s.ga_we_frames; r_ga_eps = s.ga_we_episodes;
    r_ga_solo_frames = s.ga_solo_frames; r_ga_solo_eps = s.ga_solo_episodes;
    r_freeball = s.freeball_count;
    r_freeball_corner = s.freeball_corner;
    r_corner_rescue = wm_b.corner_rescue_events + wm_y.corner_rescue_events;
}

int main(int argc, char **argv) {
    int games = 3, frames = 24000;
    int debug = 0;
    int opp_mode = 0;                        // 0=脚本打黄(我们守x=220) 1=自我博弈 2=脚本打蓝(我们守x=0)
    double opp_strength = 1.0;               // 脚本对手强度倍率（1.0=demo 校准档，>1 更强，见 docs/12）
    uint64_t seed = 0;                       // 0 = 用时间种子（每场不同）
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--games" && i + 1 < argc) games = std::atoi(argv[++i]);
        else if (a == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (a == "--strength" && i + 1 < argc) opp_strength = std::atof(argv[++i]);
        else if (a == "--opp" && i + 1 < argc) {
            std::string o = argv[++i];
            if (o == "self") opp_mode = 1;
            else if (o == "yellow") opp_mode = 2;      // 我们打黄队侧(守x=0)，脚本打蓝
            else opp_mode = 0;                          // scripted（默认）
        }
        else if (a == "--debug" && i + 1 < argc) debug = std::atoi(argv[++i]);
        else if (a == "--seed" && i + 1 < argc) seed = (uint64_t)std::atoll(argv[++i]);
        else if (a == "--help") {
            printf("sim_bench: --games N --frames N --opp scripted|self|yellow [--strength X] [--debug N] [--seed N]\n");
            return 0;
        }
    }
    const char *mode_name = opp_mode == 1 ? "自我博弈" : (opp_mode == 2 ? "我方守x=0(黄队侧)" : "脚本对手");
    printf("=== sim_bench: games=%d frames/场=%d 模式=%s 对手强度=%.2f ===\n", games, frames, mode_name, opp_strength);
    long t_blue = 0, t_yellow = 0;
    double t_poss = 0;
    int t_shots = 0;
    long t_ga = 0, t_ga_eps = 0, t_ga_solo = 0, t_ga_solo_eps = 0, t_fb = 0, t_fb_corner = 0, t_rescue = 0;
    auto t0 = std::chrono::steady_clock::now();
    for (int g = 0; g < games; ++g) {
        // 修复：--seed N 时每场要用不同种子（seed + 场次偏移），
        //   否则所有场次开局微扰完全相同 → 100 场其实是同一场的 100 份拷贝，
        //   批次统计(均分/控球/射门)会退化成单样本，掩盖掉 --seed 复现实验的意义。
        uint64_t gs = seed ? (seed + (uint64_t)g * 0x9E3779B97F4A7C15ull)
                           : (uint64_t)std::chrono::steady_clock::now().time_since_epoch().count();
        Rng rng(gs);
        long b, y; double poss; int shots; long zones[3] = {0,0,0};
        long ga_frames = 0, ga_eps = 0, ga_solo = 0, ga_solo_eps = 0, fb = 0, fb_corner = 0, rescue = 0;
        play_match(frames, opp_mode, debug, opp_strength, rng, b, y, poss, shots, zones, ga_frames, ga_eps, ga_solo, ga_solo_eps, fb, fb_corner, rescue);
        // play_match 已按 opp_mode 归一化：返回的 b=我们进球、y=对手进球
        printf("  场%02d: 我们 %ld : %ld 对手   控球率(我们) %.0f%%   射门 %d   球位 %ld%%/%ld%%/%ld%%   禁区2+人 %ld帧/%ld次 单人>20帧 %ld帧/%ld次 争球重置 %ld次(角区%ld) 救球%ld次\n",
               g + 1, b, y, poss, shots,
               zones[0] * 100 / (long)frames, zones[1] * 100 / (long)frames, zones[2] * 100 / (long)frames,
               ga_frames, ga_eps, ga_solo, ga_solo_eps, fb, fb_corner, rescue);
        t_blue += b; t_yellow += y; t_poss += poss; t_shots += shots;
        t_ga += ga_frames; t_ga_eps += ga_eps; t_ga_solo += ga_solo; t_ga_solo_eps += ga_solo_eps; t_fb += fb; t_fb_corner += fb_corner; t_rescue += rescue;
    }
    auto t1 = std::chrono::steady_clock::now();
    double sec = std::chrono::duration<double>(t1 - t0).count();
    printf("=== 汇总: 我们 %ld : %ld 对手 (均 %.1f : %.1f)   平均控球 %.1f%%   平均射门 %.1f\n",
           t_blue, t_yellow, (double)t_blue / games, (double)t_yellow / games,
           t_poss / games, (double)t_shots / games);
    printf("=== 禁区纪律(我们): 门区2+人 均 %.1f 帧/场, %.1f 次/场；单人停留>20帧 均 %.1f 帧/场, %.1f 次/场；争球重置 均 %.1f 次/场(角区 %.1f)，角区救球触发 %.1f 次/场 ===\n",
           (double)t_ga / games, (double)t_ga_eps / games,
           (double)t_ga_solo / games, (double)t_ga_solo_eps / games,
           (double)t_fb / games, (double)t_fb_corner / games,
           (double)t_rescue / games);
    printf("=== 耗时 %.2fs, 场均 %.2fs (%.1f 帧/秒) ===\n", sec, sec / games, games * (double)frames / sec);
    return 0;
}
