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
#include <map>
#include <algorithm>
#include <chrono>
#include "simuro5/simuro_interface.hpp"
#include "simuro5/team.hpp"
#include "simuro5/world_model.hpp"
#include "simuro5/strategy.hpp"
#include "simuro5/shoot.hpp"          // 借墙射门计数（bank_plan_count/bank_frame_count）
#define TUNABLE_PREFIX "sim."
#include "simuro5/tunable.hpp"        // 参数注入（--params / --dump-params，见 docs/work/RL参数搜索规格.md）

using namespace simuro5;

namespace simuro5 { extern const char *g_gk_rule_probe; }   // roles.cpp，SIMURO5_GK_PROBE 下才有

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

// ==================== 物理常量（可注入旋钮，默认值=原来的硬编码值） ====================
// 生活化比喻：这些原来是焊死的螺母，现在换成带刻度的旋钮——
//   默认仍拧在原来的位置，所以"不注入参数"时行为与打补丁前逐位一致。
//   定标脚本（tools/py/calibrate_sim.py）用真机 132 场 rlg 反推它们该拧到哪。
static constexpr double kDt = 1.0 / 40.0;                 // 40Hz（平台定死，不可调）
static constexpr double kGoalLo = 70.0, kGoalHi = 110.0;  // 门宽（规则，不可调）
TUNABLE(kSpeed, 1.12856);            // 轮速→cm/s 缩放（真机实测我方 p99≈154cm/s，现在只有 90）
TUNABLE(kWheelBase, 12.1851);       // 轮距 cm（决定转向灵敏度）
TUNABLE(kAccel, 334.84);          // 轮速最大加速度 cm/s²（真机 p90≈0.209cm/帧² ⇒ 量级正确）
TUNABLE(kBallDecay, 0.9999);      // 球**高速段**每帧衰减（真机实测 0.992~0.994）
TUNABLE(kBallDecaySlow, 0.985);  // 球**低速段**每帧衰减（真机实测 ≈0.9999 几乎不减速）
TUNABLE(kDecayVref, 0.0);        // 速度分档阈值 cm/帧；0 = 关闭两档（默认与旧行为一致）
TUNABLE(kWallRest, 0.448686);        // 撞墙法向恢复（真机新测法实测 0.451/0.449）
TUNABLE(kWallFricX, 0.81);       // x 墙切向保持（真机实测 ≈0.99 几乎无损失）
TUNABLE(kWallFricY, 0.81);       // y 墙切向保持（真机实测 0.78~0.84）
TUNABLE(kContact, 7.15354);          // 球-机器人最小分离 cm（防球嵌进机器人身体）
TUNABLE(kCarryR, 13.9858);           // 携带区半径 cm（略大于策略"球后 8cm 推球点"）
TUNABLE(kCarryArc, 53.4047);        // 携带区前向半弧（度）
TUNABLE(kDeflect, 0.394744);         // 守门员挡球反弹恢复系数
TUNABLE(kRobotR, 10);           // 机器人-机器人最小间距 cm
// —— 新增：真机有、仿真原先没有的两个执行环节 ——
TUNABLE(kActDelay, 0);         // 指令生效延迟帧数（真机至少 1 帧；0=旧行为）
TUNABLE(kWheelDead, 0);        // 轮速死区：小于此值的轮速命令推不动（0=旧行为）
// —— 推球动量（原来硬编码 0.3 / 0.7） ——
TUNABLE(kPushKeep, 0.427589);         // 推球时保留旧球速的比例
TUNABLE(kPushGain, 0.98998);         // 推球时机器人速度注入的比例
// —— 脚本对手速度（原来写死 80/30/50/40；实测"对手慢一半"的根因就在这里） ——
TUNABLE(oppChaseSpeed, 145.712);      // 追击手（远球）
TUNABLE(oppChaseNearSpeed, 10);  // 追击手（近球 20cm 内）
TUNABLE(oppSupportSpeed, 120);    // 协防
TUNABLE(oppFormationSpeed, 37.624);  // 阵型站位
TUNABLE(oppGkSpeed, 77.2724);         // 门将横向 cm/s

struct SimRobot {
    double x=0, y=0, rot=0, vl=0, vr=0, pl=0, pr=0;
    double hist_l[4]={0,0,0,0}, hist_r[4]={0,0,0,0};  // 指令延迟缓冲（见 kActDelay）
    int hidx=0;
};

struct SimState {
    SimRobot blue[5], yellow[5];
    double bx = 110, by = 90, bvx = 0, bvy = 0;
    double p_bx = 110, p_by = 90;      // 上一帧球位（WorldModel 用它算球速）
    int score_blue = 0, score_yellow = 0;
    // 触球方归属（诊断，2026-10-05）：touch_blue/touch_yellow（位掩码，上面已声明）记为"本帧碰过球"，
    //   下面 last_touch_* 记"最近一次触球是谁"。用来给每个失球定性：
    //   最后触球是蓝队(我们) = 乌龙（真机 10-04 实测 88% 失球是乌龙）。
    int last_touch_team = -1, last_touch_id = -1;   // 0=蓝(我们), 1=黄(对手)
    int own_goals_by[5] = {0,0,0,0,0};             // 乌龙按"最后触球那台的编号"分类（0=门将 … 4=后卫）
    int own_goals = 0, own_goals_gk = 0;            // 乌龙数 / 其中门将顶进去的（按"我们"归一，两侧都算）
    int own_goals_y = 0, own_goals_gk_y = 0;        // opp_mode=2（我们守 x=0）时的乌龙
    long gk_own_episodes = 0;                       // 门将乌龙"片段"数（连续触球算一次）
    bool gk_own_prev = false;
    // 乌龙定性（诊断）：蓝队最后一次触球时的"当时在干什么"——门将=命中的门将规则名，场上=角色名；
    //   lt_dv = 这次触球让球"朝 x=220（蓝门）"的速度增加了多少 cm/s（>40 ≈ >1cm/帧 = 主动往自家门送）
    std::string lt_rule;
    double lt_dv = 0.0;
    int lt_frame = 0;
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
    // 以下只记录物理过程，不参与动力学或策略。
    int reset_epoch = 0, carry_team = -1, carry_id = -1;
    unsigned touch_blue = 0, touch_yellow = 0;
    bool push_applied = false;
};

#include "coop_observer.hpp"

#ifdef SIMURO5_BRANCH_TRACE
// ============================================================
// 争抢犹豫追踪（sim_trace 诊断构建）：对方贴球(≤20cm)时，我方离球最近的非门将
//   (8~60cm) 若"朝球方向的指令速度"< kHesSpeed → 记一次犹豫，归到下指令的源码行。
// ============================================================
#define SIMURO5_TRACE_NO_MACROS
#include "simuro5/branch_trace.hpp"
#include <map>
#include <algorithm>
static std::map<std::string, long> g_hes_by_line, g_all_by_line;
static long g_hes_n = 0, g_con_n = 0;
static long g_loose_n = 0, g_loose_hes = 0;
static std::map<std::string, long> g_loose_all, g_loose_hes_by;
static void hes_sample(const WorldModel &wm, const SimState &s) {
    const double kHesSpeed = 40.0;
    double od = 1e9;
    for (int j = 0; j < 5; ++j) od = std::min(od, std::hypot(s.yellow[j].x - s.bx, s.yellow[j].y - s.by));
    {   // 无主球：双方最近者都 >15cm，记我方最近场上队员的分支与朝球指令速度
        int bi = -1; double bdd = 1e9;
        for (int i = 1; i < 5; ++i) {
            double d = std::hypot(wm.home[i].x - s.bx, wm.home[i].y - s.by);
            if (d < bdd) { bdd = d; bi = i; }
        }
        if (bi >= 0 && bdd > 15.0 && od > 15.0 && bdd < 100.0) {
            const RobotState &r = wm.home[bi];
            double a2 = std::atan2(s.by - r.y, s.bx - r.x) - r.rot * 3.14159265358979 / 180.0;
            double v2 = 0.5 * (r.vl + r.vr) * std::cos(a2);
            const trace::Mark *m = trace::find(&r);
            std::string key = "R" + std::to_string(wm.role[bi]) + " ";
            if (m && m->file) {
                const char *f = std::strrchr(m->file, '\\'); if (!f) f = std::strrchr(m->file, '/');
                key += std::string(f ? f + 1 : m->file) + ":" + std::to_string(m->line);
            } else key += "(unmarked)";
            ++g_loose_n; ++g_loose_all[key];
            if (v2 < kHesSpeed) { ++g_loose_hes; ++g_loose_hes_by[key]; }
        }
    }
    if (od > 20.0) return;
    int best = -1; double bd = 1e9;
    for (int i = 1; i < 5; ++i) {
        double d = std::hypot(wm.home[i].x - s.bx, wm.home[i].y - s.by);
        if (d < bd) { bd = d; best = i; }
    }
    if (best < 0 || bd < 8.0 || bd > 60.0) return;
    const RobotState &r = wm.home[best];
    double a = std::atan2(s.by - r.y, s.bx - r.x) - r.rot * 3.14159265358979 / 180.0;
    double v = 0.5 * (r.vl + r.vr) * std::cos(a);
    const trace::Mark *m = trace::find(&r);
    std::string key = "R" + std::to_string(wm.role[best]) + " ";
    if (m && m->file) {
        const char *f = std::strrchr(m->file, '\\'); if (!f) f = std::strrchr(m->file, '/');
        key += std::string(f ? f + 1 : m->file) + ":" + std::to_string(m->line);
    } else key += "(unmarked)";
    ++g_con_n; ++g_all_by_line[key];
    if (v < kHesSpeed) { ++g_hes_n; ++g_hes_by_line[key]; }
}
static void hes_report() {
    printf("\n[无主球追踪] 帧 %ld，最近者不朝球 %ld（%.1f%%）\n", g_loose_n, g_loose_hes, 100.0 * g_loose_hes / std::max(1L, g_loose_n));
    {
        std::vector<std::pair<long, std::string>> v;
        for (auto &kv : g_loose_all) v.push_back({g_loose_hes_by[kv.first], kv.first});
        std::sort(v.rbegin(), v.rend());
        for (size_t i = 0; i < v.size() && i < 25; ++i)
            printf("  %-28s 不朝球 %6ld / 帧 %6ld  (%.0f%%)\n", v[i].second.c_str(), v[i].first,
                   g_loose_all[v[i].second], 100.0 * v[i].first / std::max(1L, g_loose_all[v[i].second]));
    }
    printf("\n[犹豫追踪] 争抢帧 %ld，犹豫 %ld（%.1f%%）\n", g_con_n, g_hes_n, 100.0 * g_hes_n / std::max(1L, g_con_n));
    std::vector<std::pair<long, std::string>> v;
    for (auto &kv : g_all_by_line) v.push_back({g_hes_by_line[kv.first], kv.first});
    std::sort(v.rbegin(), v.rend());
    for (size_t i = 0; i < v.size() && i < 80; ++i)
        printf("  %-28s 犹豫 %6ld / 争抢 %6ld  (%.0f%%)\n", v[i].second.c_str(), v[i].first,
               g_all_by_line[v[i].second], 100.0 * v[i].first / std::max(1L, g_all_by_line[v[i].second]));
}
#endif

static double deg2rad(double d) { return d * 3.14159265358979 / 180.0; }

// ============================================================
// 轨迹导出（docs/18 验证用）：--traj out.csv 把第一场的逐帧位置写成 CSV，
// 列名与真机 rlg 导出的 traj CSV 完全一致 → 可用 tools/py/motion_calib.py
// 在同一口径下量 sim 与真机（速度/加速度/静止帧占比/低速来回蹭）。
// ============================================================
static FILE *g_traj = nullptr;

static void traj_write_header(FILE *fp) {
    for (int i = 0; i < 5; ++i) fprintf(fp, "b%d_x,b%d_y,", i, i);
    for (int i = 0; i < 5; ++i) fprintf(fp, "y%d_x,y%d_y%s", i, i, i == 4 ? "" : ",");
    fprintf(fp, ",ball_x,ball_y\n");
}

static void traj_write_row(FILE *fp, const SimState &s) {
    for (int i = 0; i < 5; ++i) fprintf(fp, "%.2f,%.2f,", s.blue[i].x, s.blue[i].y);
    for (int i = 0; i < 5; ++i)
        fprintf(fp, "%.2f,%.2f%s", s.yellow[i].x, s.yellow[i].y, i == 4 ? "" : ",");
    fprintf(fp, ",%.2f,%.2f\n", s.bx, s.by);
}

// 门球演练（诊断，2026-10-05）：--goalkick-drill 时除开场外每次复位都摆成「我方门球」，
//   摆位抄真机 rlg 实测（球 (205,78)，门将 (215,90) 朝 -90°，对手在 80~100cm 外）。
//   真机 15 场 114 次门球 113 次 1 秒后球没离开 15cm —— 原 sim 从不复位成门球，测不到。
static bool g_gk_drill = false;
static long g_gkd_n = 0, g_gkd_out1s = 0, g_gkd_out2s = 0, g_gkd_conceded = 0;
static std::map<std::string, long> g_gkd_rules;   // 门球后前 80 帧门将命中的规则

// 初始摆位（简单开局阵型）；seed 用于引入摆位微扰（模拟真实开局差异）
static void init_formation(SimState &s, Rng *rng = nullptr) {
    ++s.reset_epoch;
    s.carry_team = s.carry_id = -1; s.touch_blue = s.touch_yellow = 0;
    s.push_applied = false;
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
    // 补充实验：复位不冒充球速。默认模式保持历史行为。
    if (g_correct_ball_history) { s.p_bx = s.bx; s.p_by = s.by; }
    if (g_gk_drill && s.reset_epoch > 1) {
        const double gbx[5] = {215, 190, 170, 150, 130}, gby[5] = {90, 100, 65, 40, 130};
        const double gyx[5] = {5, 81, 81, 100, 100},     gyy[5] = {90, 49, 131, 59, 91};
        for (int i = 0; i < 5; ++i) {
            s.blue[i].x = gbx[i]; s.blue[i].y = gby[i]; s.blue[i].rot = i == 0 ? -90 : 180;
            s.yellow[i].x = gyx[i]; s.yellow[i].y = gyy[i]; s.yellow[i].rot = 0;
            if (rng && i > 0) { s.yellow[i].x += rng->range(-3, 3); s.yellow[i].y += rng->range(-3, 3); }
        }
        s.bx = s.p_bx = 205.2; s.by = s.p_by = 77.7;
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
    s.carry_team = s.carry_id = -1; s.touch_blue = s.touch_yellow = 0;
    s.push_applied = false;
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
        // 指令延迟：真机上本帧算出的轮速，要过 kActDelay 帧才生效
        int D = (int)(kActDelay + 0.5); if (D > 3) D = 3; if (D < 0) D = 0;
        r.hist_l[r.hidx] = r.vl; r.hist_r[r.hidx] = r.vr;
        // D=0 → 读本帧刚写入那格（无延迟，保持旧行为）；D=1 → 读上一帧的命令
        int rd = ((r.hidx - D) % 4 + 4) % 4;
        r.hidx = (r.hidx + 1) & 3;
        double cmd_l = r.hist_l[rd];
        double cmd_r = r.hist_r[rd];
        // 轮速死区：真机小轮速推不动（0 = 关闭，保持旧行为）
        if (std::fabs(cmd_l) < kWheelDead) cmd_l = 0;
        if (std::fabs(cmd_r) < kWheelDead) cmd_r = 0;
        double maxdv = kAccel * kDt;
        double nvl = cmd_l, nvr = cmd_r;
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
    // 两档衰减：真机实测低速段几乎不减速(0.9999)，高速段 0.992~0.994
    { double bspd = std::hypot(s.bvx, s.bvy);
      double dec = (kDecayVref > 0.0 && bspd < kDecayVref) ? kBallDecaySlow : kBallDecay;
      s.bvx *= dec; s.bvy *= dec; }

    // 球-墙反弹（垂直分量衰减 + 平行分量摩擦衰减）；门线开口处（y∈门宽）不反弹——球要能进门！
    if (s.bx < 0) {
        if (s.by >= kGoalLo && s.by <= kGoalHi) { /* 进门：交给 check_goal 判定 */ }
        else { s.bx = -s.bx; s.bvx = -s.bvx * kWallRest; s.bvy *= kWallFricX; }
    }
    if (s.bx > 220) {
        if (s.by >= kGoalLo && s.by <= kGoalHi) { /* 进门 */ }
        else { s.bx = 440 - s.bx; s.bvx = -s.bvx * kWallRest; s.bvy *= kWallFricX; }
    }
    if (s.by < 0) { s.by = -s.by; s.bvy = -s.bvy * kWallRest; s.bvx *= kWallFricY; }
    if (s.by > 180) { s.by = 360 - s.by; s.bvy = -s.bvy * kWallRest; s.bvx *= kWallFricY; }

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
        s.carry_team = carry_blue ? 0 : 1; s.carry_id = carry_i;
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
            s.push_applied = true;
            (carry_blue ? s.touch_blue : s.touch_yellow) |= 1u << carry_i;   // 携带推球也算触球
            if (s.dbg_contacts) printf("  [接触] %s%d 携带推球 rv%.0f\n",
                                       carry_blue ? "蓝" : "黄", carry_i, rv);
            s.bvx = s.bvx * kPushKeep + rvx * kPushGain;
            s.bvy = s.bvy * kPushKeep + rvy * kPushGain;
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
            (t ? s.touch_yellow : s.touch_blue) |= 1u << i;
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

// 乌龙定性统计（诊断，只统计蓝队=我们的乌龙，即 scripted/wall 模式）
static std::map<std::string, std::pair<long, long>> g_og_tags;   // 标签 -> (个数, 其中主动)
static long g_og_active = 0, g_og_age[3] = {0, 0, 0};   // 主动送进；触球后 <10 / 10~40 / >40 帧进门
static void og_record(const SimState &s) {
    const bool active = s.lt_dv > 40.0;
    g_og_active += active;
    const int age = s.frames - s.lt_frame;
    ++g_og_age[age < 10 ? 0 : (age <= 40 ? 1 : 2)];
    auto &t = g_og_tags[s.lt_rule];
    ++t.first; t.second += active;
}

// 进球判定 + 重置（带随机摆位）；debug>0 时打印进球详情
static bool check_goal(SimState &s, Rng *rng, int debug) {
    if (s.bx > 220 && s.by >= kGoalLo && s.by <= kGoalHi) {   // 蓝队失球(黄得分)
        s.score_yellow++;
        // 乌龙判定：本帧或近几帧最后触球是我方（touch_* 每帧清空，所以用 last_touch_*）
        if (s.last_touch_team == 0) {
            ++s.own_goals;
            if (s.last_touch_id >= 0 && s.last_touch_id < 5) ++s.own_goals_by[s.last_touch_id];
            if (s.last_touch_id == 0) ++s.own_goals_gk;
            og_record(s);
        }
        if (debug) printf("  [失球] 帧%d 蓝失: 球(%.0f,%.0f)v(%.0f,%.0f) 门将(%.0f,%.0f) 蓝1(%.0f,%.0f) 黄近球(%.0f,%.0f) 最后触球=%s%d\n",
                          s.frames, s.bx, s.by, s.bvx, s.bvy, s.blue[0].x, s.blue[0].y,
                          s.blue[1].x, s.blue[1].y,
                          s.yellow[0].x, s.yellow[0].y,
                          s.last_touch_team < 0 ? "无" : (s.last_touch_team == 0 ? "蓝" : "黄"), s.last_touch_id);
        init_formation(s, rng); return true;
    }
    if (s.bx < 0 && s.by >= kGoalLo && s.by <= kGoalHi) {     // 黄队失球(蓝得分)
        s.score_blue++;
        if (s.last_touch_team == 1) {                          // opp_mode=2 时我们执黄队
            ++s.own_goals_y;
            if (s.last_touch_id == 0) ++s.own_goals_gk_y;
        }
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
    double maxdy = oppGkSpeed * strength / 40.0;  // oppGkSpeed*strength cm/s 横向限速
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
            double spd_near = oppChaseNearSpeed * (0.6 + 0.4 * strength);
            double spd = (db < 20.0) ? spd_near : (oppChaseSpeed * (0.6 + 0.4 * strength));
            // 站球后推球（朝对方球门方向）：推球点 = 球后方 6cm = 靠己方门一侧。
            drive(R[i], s.bx - att_sign * 6.0 + wob * 0.6, s.by + wob, spd);
        } else if (i == support) {
            // 协防：站到球与己方球门连线 40% 处（截击传球路线），不直接贴球。
            double mx = is_blue ? (s.bx + 220.0) * 0.6 : (s.bx + 0.0) * 0.4;
            double my = (s.by + 90.0) * 0.5;
            drive(R[i], mx, my, oppSupportSpeed);
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
            drive(R[i], sx, sy, oppFormationSpeed);
        }
    }
}

// demo 式门线堆人墙（参考真机 12:28 铁证：demo 5 人堆门线防我们贴线推射，
//   自研实现）——只防守不主动进攻，专测"我们进攻 vs 5 人墙"的终结能力：
//   0=GK 门线慢横移跟球；1-3 贴线 y 墙(55/78/102/145 覆盖门宽上下+翼侧)；
//   4=机动：球压到本方门前 60cm → 贴球关门（球朝中场 8cm，挡我们推线），
//   球远 → 回门口 (60,90)。任何墙员距球<13 且球 x<30 → 把球推回中场（解围）。
static void wall_opponent(SimState &s) {
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
    SimRobot *R = s.yellow;
    // GK：门线跟球（30cm/s 限速、12 帧反应）——同 scripted 校准档
    SimRobot &gk = R[0];
    static int gk_react = 0;
    static double gk_target = 90.0;
    bool ball_in_own_half = s.bx < 60.0;
    if (s.frames % 12 == 0) {
        gk_target = ball_in_own_half ? (s.by < 70 ? 70.0 : (s.by > 110 ? 110.0 : s.by)) : 90.0;
        gk_react = 0;
    }
    double dy = gk_target - gk.y;
    double maxdy = 30.0 / 40.0;
    if (dy > maxdy) dy = maxdy; else if (dy < -maxdy) dy = -maxdy;
    gk.y += dy;
    gk.x = 8.0; gk.rot = 0.0; gk.vl = gk.vr = 0;
    double wy[4] = {55.0, 78.0, 102.0, 145.0};
    // 双层深度（9/7 真机 demo 战术：球压前场时双层弧 x20-60 y60-150，
    //   见 docs/06 第34轮热区 x40-60 y90-120 最密 417）——按球 x 动态前压：
    //   球 x<40 贴线关门（原单层）；x40-110 双层弧前压；x>110 回中圈
    double dline[4] = {24.0, 36.0, 48.0, 55.0};   // 双层弧各员 x 深度
    for (int i = 1; i < 5; ++i) {
        double db = std::hypot(s.bx - R[i].x, s.by - R[i].y);
        // 解围优先：贴线者距球<13 且球贴门线(x<30) → 推球回中场
        if (db < 13.0 && s.bx < 30.0) { drive(R[i], s.bx + 10.0, s.by, 90.0); continue; }
        double tx, ty;
        if (i == 4) {
            if (s.bx < 60.0) { tx = s.bx + 8.0; ty = s.by; }   // 机动关门（挡我们推线）
            else if (s.bx < 110.0) { tx = 70.0; ty = (s.by > 90 ? 40.0 : 140.0); }  // 中场边路截击
            else { tx = 95.0; ty = 90.0; }
        } else {
            if (s.bx >= 40.0) {
                tx = dline[i - 1] + (s.bx > 80.0 ? 6.0 : 0.0);   // 双层弧前压
            } else {
                tx = 10.0 + (i - 1) * 4.0;                        // 贴线深度（球已压到门前）
            }
            ty = wy[i - 1];                                       // 墙位
            if (s.bx < 40.0 && std::fabs(s.by - wy[i - 1]) < 25.0) {
                tx = s.bx + 6.0; ty = s.by;                       // 球压近 → 关门位
            }
        }
        drive(R[i], tx, ty, 55.0);
    }
}

// 主攻交接计数（诊断，2026-10-05）：只数比赛进行中我方 active_id 的变化，
//   死球/点球时 RoleAssignment 复位到 1 号不算交接。对齐真机 role_dynamics.py 的"交接"口径。
static long g_active_swaps = 0;
static long g_og_by_id[5] = {0,0,0,0,0};
// 控球拆解（诊断，2026-10-05）：20cm 内 只有我方 / 只有对方 / 双方都在(争抢) / 都不在 的帧数，
//   争抢帧里再分谁更近。用来看 34% 控球是"人不在球边"还是"挤在一起输了距离"。
static long g_pz_us = 0, g_pz_opp = 0, g_pz_both = 0, g_pz_none = 0, g_pz_both_us = 0;
static long g_pz_opp_gk = 0, g_pz_opp_zone[3] = {0, 0, 0};   // 仅对方帧：对方门将 / 球在对方门前40·对方半场·我方半场
static void count_active_swap(const WorldModel &wm, int &prev_id, bool &prev_live) {
    const bool running = wm.runtime_phase == RuntimePhase::Running;
    if (prev_live && running && wm.active_id != prev_id) ++g_active_swaps;
    prev_id = wm.active_id; prev_live = running;
}

// 一场比赛
// opp_mode: 0=脚本对手打黄队(我们守x=220, 默认)  1=自我博弈  2=脚本对手打蓝队(我们守x=0, 测半场对称)
static void play_match(int frames, int opp_mode, int debug, double opp_strength, Rng &rng, long &r_blue, long &r_yellow,
                       double &r_poss, int &r_shots, long r_zones[3], long &r_ga_frames, long &r_ga_eps,
                       long &r_ga_solo_frames, long &r_ga_solo_eps, long &r_freeball,
                       long &r_freeball_corner, long &r_corner_rescue,
                       long &r_own_goals, long &r_own_goals_gk, long &r_gk_own_episodes) {
    SimState s;
    SimState last_decision_state;
    init_formation(s, &rng);
    TeamContext ctx_blue{true}, ctx_yellow{false};
    WorldModel wm_b, wm_y;
    g_coop_state = &s;
    g_coop_tail[0] = g_coop_tail[1] = 0;
    wm_b.coop_observer = wm_y.coop_observer = coop_csv_row;
    Strategy strat_b, strat_y;
    Environment env_b, env_y;
    int swap_prev_id = 1; bool swap_prev_live = false;
    int drill_t = -1, drill_epoch = s.reset_epoch;   // 门球演练：门球后第几帧（-1 = 不在演练窗口）

    for (int f = 0; f < frames; ++f) {
        g_coop_frame = f;
        const int old_epoch = s.reset_epoch;
        if (f == frames - 1) last_decision_state = s;
        // 蓝队决策：opp_mode=2 时蓝队是脚本；否则蓝队是我们的策略
        if (opp_mode == 2) {
            scripted_opponent(s, true, opp_strength);
        } else {
            fill_env(env_b, s, true);
            wm_b.update(&env_b, ctx_blue);
#ifdef SIMURO5_BRANCH_TRACE
            trace::reset();
#endif
            strat_b.run(wm_b);
#ifdef SIMURO5_BRANCH_TRACE
            hes_sample(wm_b, s);
#endif
            coop_sample(wm_b);
            if (drill_t >= 0 && drill_t < 80) ++g_gkd_rules[g_gk_rule_probe];
            count_active_swap(wm_b, swap_prev_id, swap_prev_live);
            for (int i = 0; i < 5; ++i) { s.blue[i].vl = wm_b.home[i].vl; s.blue[i].vr = wm_b.home[i].vr; }
        }

        // 黄队决策：opp_mode=0 时黄队是脚本；3=门线堆人墙；否则黄队是我们的策略
        if (opp_mode == 0) {
            scripted_opponent(s, false, opp_strength);
        } else if (opp_mode == 3) {
            wall_opponent(s);
        } else {
            fill_env(env_y, s, false);
            wm_y.update(&env_y, ctx_yellow);
            strat_y.run(wm_y);
            coop_sample(wm_y);
            if (opp_mode == 2) count_active_swap(wm_y, swap_prev_id, swap_prev_live);
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

        if (g_correct_ball_history) { s.p_bx = s.bx; s.p_by = s.by; }
        s.touch_blue = 0; s.touch_yellow = 0;
        const double pre_bvx = s.bvx;
        step_physics(s);

        // 记录本帧球位作为下一帧的 lastBall（WorldModel 用差分算球速）
        if (!g_correct_ball_history) { s.p_bx = s.bx; s.p_by = s.by; }

        // 更新"最近一次触球"（乌龙判定用；双方同帧触球时取离球更近的一方）
        if (s.touch_blue || s.touch_yellow) {
            int bt = -1, yt = -1;
            for (int i = 0; i < 5; ++i) {           // 取触球方位掩码里编号最小的一台（不依赖 __builtin_ctz）
                if (bt < 0 && (s.touch_blue & (1u << i))) bt = i;
                if (yt < 0 && (s.touch_yellow & (1u << i))) yt = i;
            }
            bool take_blue = false;
            if (bt >= 0 && yt >= 0) {
                double db = std::hypot(s.bx - s.blue[bt].x, s.by - s.blue[bt].y);
                double dy = std::hypot(s.bx - s.yellow[yt].x, s.by - s.yellow[yt].y);
                take_blue = db <= dy;
            } else take_blue = (bt >= 0);
            s.last_touch_team = take_blue ? 0 : 1;
            s.last_touch_id = take_blue ? bt : yt;
            if (take_blue) {
                static const char *const kRoleName[] = {"场上:GOALIE", "场上:ACTIVE", "场上:PASSIVE", "场上:ASSIST", "场上:MIDFIELD"};
                const int role = (opp_mode == 2) ? -1 : wm_b.role[bt];
                s.lt_rule = opp_mode == 2 ? "脚本" : (bt == 0 ? g_gk_rule_probe
                            : (role >= 0 && role < 5 ? kRoleName[role] : "场上:?"));
#ifdef SIMURO5_BRANCH_TRACE
                if (opp_mode != 2) {   // sim_trace：再附上这台本帧运动指令来自哪一行源码
                    const trace::Mark *m = trace::find(&wm_b.home[bt]);
                    if (m && m->file) {
                        const char *fn = std::strrchr(m->file, '\\'); if (!fn) fn = std::strrchr(m->file, '/');
                        s.lt_rule += std::string(" ") + (fn ? fn + 1 : m->file) + ":" + std::to_string(m->line);
                    }
                }
#endif
                s.lt_dv = s.bvx - pre_bvx;
                s.lt_frame = s.frames;
            }
            bool gk_now = take_blue && bt == 0;
            if (gk_now && !s.gk_own_prev) ++s.gk_own_episodes;
            s.gk_own_prev = gk_now;
        } else {
            s.gk_own_prev = false;   // 连续触球才叫一个"片段"，断一次就算新的
        }

        if (g_traj) traj_write_row(g_traj, s);   // docs/18：轨迹导出（仅第一场）

        // 球权统计：谁离球最近算谁控球（阈值 20cm 内；无人区不算）
        double d_b = 1e9, d_y = 1e9;
        for (int i = 0; i < 5; ++i) {
            d_b = std::min(d_b, std::hypot(s.bx - s.blue[i].x, s.by - s.blue[i].y));
            d_y = std::min(d_y, std::hypot(s.bx - s.yellow[i].x, s.by - s.yellow[i].y));
        }
        if (d_b < 20.0 && d_b <= d_y) s.poss_blue++;
        else if (d_y < 20.0 && d_y < d_b) s.poss_yellow++;
        {
            double d_us = opp_mode == 2 ? d_y : d_b, d_op = opp_mode == 2 ? d_b : d_y;
            bool nu = d_us < 20.0, no = d_op < 20.0;
            if (nu && no) { ++g_pz_both; if (d_us <= d_op) ++g_pz_both_us; }
            else if (nu) ++g_pz_us;
            else if (no) {
                ++g_pz_opp;
                const SimRobot *O = opp_mode == 2 ? s.blue : s.yellow;
                int k = 0; double dk = 1e9;
                for (int i = 0; i < 5; ++i) { double d = std::hypot(s.bx - O[i].x, s.by - O[i].y); if (d < dk) { dk = d; k = i; } }
                if (k == 0) ++g_pz_opp_gk;
                double depth = opp_mode == 2 ? 220.0 - s.bx : s.bx;   // 球离"对方门线"
                ++g_pz_opp_zone[depth < 40.0 ? 0 : depth < 110.0 ? 1 : 2];
            }
            else ++g_pz_none;
        }
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

        const int pre_sy = s.score_yellow;
        check_goal(s, &rng, debug);
        s.frames++;
        if (g_gk_drill && opp_mode != 2) {   // 门球演练统计（只在我们守 x=220 时）
            if (drill_t >= 0) {
                ++drill_t;
                double mv = std::hypot(s.bx - 205.2, s.by - 77.7);
                if (drill_t == 40 && mv > 15.0) ++g_gkd_out1s;
                if (drill_t == 80 && mv > 15.0) ++g_gkd_out2s;
                if (s.score_yellow > pre_sy && drill_t <= 200) ++g_gkd_conceded;
                if (drill_t > 200 || s.reset_epoch != drill_epoch) drill_t = -1;
            }
            if (s.reset_epoch != drill_epoch && s.reset_epoch > 1) {
                drill_epoch = s.reset_epoch; drill_t = 0; ++g_gkd_n;
            }
        }

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
        if (g_coop && s.reset_epoch != old_epoch)
            printf("SIM_EVENT game=%d frame=%d reset=%d score_blue=%d score_yellow=%d\n",
                g_coop_game, f, s.reset_epoch, s.score_blue, s.score_yellow);
    }
    g_coop_frame = frames;
    // 最后决策的 WorldModel 必须配同一时点物理快照；截止帧没有新的决策。
    g_coop_state = &last_decision_state;
    if (opp_mode != 2) coop_print_result(wm_b);
    if (opp_mode == 1 || opp_mode == 2) coop_print_result(wm_y);
    g_coop_state = nullptr;
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
    for (int i = 0; i < 5; ++i) g_og_by_id[i] += s.own_goals_by[i];
    r_own_goals = opp_mode == 2 ? s.own_goals_y : s.own_goals;
    r_own_goals_gk = opp_mode == 2 ? s.own_goals_gk_y : s.own_goals_gk;
    r_gk_own_episodes = s.gk_own_episodes;
    r_freeball_corner = s.freeball_corner;
    r_corner_rescue = wm_b.corner_rescue_events + wm_y.corner_rescue_events;
}

int main(int argc, char **argv) {
    int games = 3, frames = 24000;
    int debug = 0;
    int opp_mode = 0;                        // 0=脚本打黄(我们守x=220) 1=自我博弈 2=脚本打蓝(我们守x=0)
    double opp_strength = 1.0;               // 脚本对手强度倍率（1.0=demo 校准档，>1 更强，见 docs/12）
    uint64_t seed = 0;                       // 0 = 用时间种子（每场不同）
    const char *traj_path = nullptr;         // --traj out.csv：导出第一场逐帧轨迹（docs/18）
    const char *params_path = nullptr;       // --params in.txt：注入策略参数（离线搜索用）
    const char *dump_path = nullptr;         // --dump-params out.txt：导出参数表
    const char *coop_path = nullptr;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--games" && i + 1 < argc) games = std::atoi(argv[++i]);
        else if (a == "--frames" && i + 1 < argc) frames = std::atoi(argv[++i]);
        else if (a == "--coop-csv" && i + 1 < argc) coop_path = argv[++i];
        else if (a == "--correct-ball-history") g_correct_ball_history = true;
        else if (a == "--goalkick-drill") g_gk_drill = true;
        else if (a == "--strength" && i + 1 < argc) opp_strength = std::atof(argv[++i]);
        else if (a == "--opp" && i + 1 < argc) {
            std::string o = argv[++i];
            if (o == "self") opp_mode = 1;
            else if (o == "yellow") opp_mode = 2;      // 我们打黄队侧(守x=0)，脚本打蓝
            else if (o == "wall") opp_mode = 3;        // 我们打 demo 式门线堆人墙
            else opp_mode = 0;                          // scripted（默认）
        }
        else if (a == "--debug" && i + 1 < argc) debug = std::atoi(argv[++i]);
        else if (a == "--seed" && i + 1 < argc) seed = (uint64_t)std::atoll(argv[++i]);
        else if (a == "--traj" && i + 1 < argc) traj_path = argv[++i];
        // 参数注入（离线搜索用；默认不注入 ⇒ 行为与编译期常量完全一致）
        else if (a == "--params" && i + 1 < argc) params_path = argv[++i];
        else if (a == "--dump-params" && i + 1 < argc) dump_path = argv[++i];
        else if (a == "--help") {
            printf("sim_bench: --games N --frames N --opp scripted|self|yellow|wall [--strength X] [--debug N] [--seed N] [--traj out.csv]\n");
            printf("           --coop-csv out.csv（配合生命周期观测） --correct-ball-history（独立球速输入修正实验）\n");
            printf("           --params in.txt（注入参数） --dump-params out.txt（导出全部可调参数及默认值）\n");
            return 0;
        }
    }
    if (dump_path) {
        int n = simuro5::dump_params(dump_path);
        printf("=== 已导出 %d 个可调参数 → %s ===\n", n, dump_path);
        return 0;
    }
    if (params_path) {
        std::vector<std::string> unknown;
        int n = simuro5::apply_param_file(params_path, &unknown);
        if (n < 0) { printf("sim_bench: 无法读取参数文件 %s\n", params_path); return 1; }
        printf("=== 已注入 %d 个参数（来自 %s）", n, params_path);
        if (!unknown.empty()) {
            printf("；⚠️ 未识别 %d 个:", (int)unknown.size());
            for (size_t k = 0; k < unknown.size() && k < 8; ++k) printf(" %s", unknown[k].c_str());
        }
        printf(" ===\n");
    }
    if (traj_path) {
        g_traj = fopen(traj_path, "w");
        if (!g_traj) { printf("sim_bench: 无法写入轨迹文件 %s\n", traj_path); return 1; }
        traj_write_header(g_traj);
    }
    const char *mode_name = opp_mode == 1 ? "自我博弈" : (opp_mode == 2 ? "我方守x=0(黄队侧)" : (opp_mode == 3 ? "门线堆人墙" : "脚本对手"));
    if (coop_path) {
        g_coop = fopen(coop_path, "w");
        if (!g_coop) { fprintf(stderr, "Cannot open coop CSV: %s\n", coop_path); return 1; }
        coop_csv_header();
    }
    printf("=== COOP observation: ball_history=%s ===\n", g_correct_ball_history ? "corrected_experiment" : "original");
    printf("=== sim_bench: games=%d frames/场=%d 模式=%s 对手强度=%.2f ===\n", games, frames, mode_name, opp_strength);
    long t_blue = 0, t_yellow = 0;
    double t_poss = 0;
    int t_shots = 0;
    long t_ga = 0, t_ga_eps = 0, t_ga_solo = 0, t_ga_solo_eps = 0, t_fb = 0, t_fb_corner = 0, t_rescue = 0;
    long t_og = 0, t_og_gk = 0, t_gk_ep = 0;   // 乌龙 / 门将乌龙 / 门将触球片段
    auto t0 = std::chrono::steady_clock::now();
    for (int g = 0; g < games; ++g) {
        // 修复：--seed N 时每场要用不同种子（seed + 场次偏移），
        //   否则所有场次开局微扰完全相同 → 100 场其实是同一场的 100 份拷贝，
        //   批次统计(均分/控球/射门)会退化成单样本，掩盖掉 --seed 复现实验的意义。
        uint64_t gs = seed ? (seed + (uint64_t)g * 0x9E3779B97F4A7C15ull)
                           : (uint64_t)std::chrono::steady_clock::now().time_since_epoch().count();
        Rng rng(gs);
        g_coop_game = g + 1; g_coop_seed = gs;
        long b, y; double poss; int shots; long zones[3] = {0,0,0};
        long ga_frames = 0, ga_eps = 0, ga_solo = 0, ga_solo_eps = 0, fb = 0, fb_corner = 0, rescue = 0;
        long og = 0, og_gk = 0, gk_ep = 0;
        play_match(frames, opp_mode, debug, opp_strength, rng, b, y, poss, shots, zones, ga_frames, ga_eps, ga_solo, ga_solo_eps, fb, fb_corner, rescue, og, og_gk, gk_ep);
        t_og += og; t_og_gk += og_gk; t_gk_ep += gk_ep;
        if (g_traj) { fclose(g_traj); g_traj = nullptr; }   // 轨迹只导第一场
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
    printf("=== 主攻交接(我们): 均 %.1f 次/场, %.2f 次/秒 ===\n",
           (double)g_active_swaps / games, (double)g_active_swaps / ((double)games * frames / 40.0));
    printf("=== 乌龙归属(我们，编号1主攻/2助攻/3中场/4后卫): %.2f / %.2f / %.2f / %.2f 个/场 ===\n",
           (double)g_og_by_id[1] / games, (double)g_og_by_id[2] / games,
           (double)g_og_by_id[3] / games, (double)g_og_by_id[4] / games);
    printf("=== 乌龙(我们): 均 %.2f 个/场 (占总失球 %.0f%%)，其中门将 %.2f 个/场；门将门前触球片段 %.1f 次/场 ===\n",
           (double)t_og / games, 100.0 * t_og / (t_yellow + 1e-9), (double)t_og_gk / games, (double)t_gk_ep / games);
    {
        long n = 0;
        std::vector<std::pair<long, std::string>> order;
        for (const auto &kv : g_og_tags) { n += kv.second.first; order.emplace_back(-kv.second.first, kv.first); }
        std::sort(order.begin(), order.end());
        if (n > 0) {
            printf("=== 乌龙定性(仅蓝=我们时有效): 主动送进(球朝自家门加速>1cm/帧) %.0f%%；触球后进门 <10帧 %.0f%% / 10~40帧 %.0f%% / >40帧 %.0f%% ===\n",
                   100.0 * g_og_active / n, 100.0 * g_og_age[0] / n, 100.0 * g_og_age[1] / n, 100.0 * g_og_age[2] / n);
            for (const auto &o : order) {
                const auto &t = g_og_tags[o.second];
                if (t.first * 100 < n) continue;   // <1% 的不列
                printf("    最后触球时在做[%s]: %.2f 个/场 (%.0f%%)，其中主动 %.0f%%\n", o.second.c_str(),
                       (double)t.first / games, 100.0 * t.first / n, 100.0 * t.second / (t.first + 1e-9));
            }
        }
    }
    if (g_gk_drill && g_gkd_n > 0) {
        printf("=== 门球演练: %ld 次门球；1 秒后球离开 15cm %.0f%%，2 秒后 %.0f%%；门球后 5 秒内失球 %.0f%% ===\n",
               g_gkd_n, 100.0 * g_gkd_out1s / g_gkd_n, 100.0 * g_gkd_out2s / g_gkd_n, 100.0 * g_gkd_conceded / g_gkd_n);
        long tot = 0;
        for (const auto &kv : g_gkd_rules) tot += kv.second;
        for (const auto &kv : g_gkd_rules)
            printf("    门球后 2 秒内门将规则 [%s] %.0f%%\n", kv.first.c_str(), 100.0 * kv.second / (tot + 1e-9));
    }
    {
        double tot = (double)(g_pz_us + g_pz_opp + g_pz_both + g_pz_none) + 1e-9;
        printf("=== 控球拆解: 仅我方 %.1f%%  仅对方 %.1f%%  争抢 %.1f%%(其中我方更近 %.1f%%)  无人 %.1f%% ===\n",
               100.0 * g_pz_us / tot, 100.0 * g_pz_opp / tot, 100.0 * g_pz_both / tot,
               100.0 * g_pz_both_us / (g_pz_both + 1e-9), 100.0 * g_pz_none / tot);
        double po = (double)g_pz_opp + 1e-9;
        printf("=== 仅对方帧里: 对方门将 %.1f%%；球在 对方门前40cm %.1f%% / 对方半场 %.1f%% / 我方半场 %.1f%% ===\n",
               100.0 * g_pz_opp_gk / po, 100.0 * g_pz_opp_zone[0] / po,
               100.0 * g_pz_opp_zone[1] / po, 100.0 * g_pz_opp_zone[2] / po);
    }
    printf("=== 耗时 %.2fs, 场均 %.2fs (%.1f 帧/秒) ===\n", sec, sec / games, games * (double)frames / sec);
    // 借墙射门（docs/06 第 65 轮）：机会次数（连续采纳算 1 次）+ 采纳帧数
    printf("=== 借墙射门: 机会 %.1f 次/场, 采纳 %ld 帧 (%.1f 帧/场) ===\n",
           (double)simuro5::bank_plan_count() / games,
           simuro5::bank_frame_count(), (double)simuro5::bank_frame_count() / games);
    // 机器可读汇总（参数搜索/脚本解析专用，格式稳定，勿改字段顺序）
    // 字段：net 净胜球/场、gf/ga 场均进球、poss 控球%、shots 场均射门、
    //       gaf 门区2+人帧/场、fb 争球重置/场、sec 耗时
    printf("FIT games=%d net=%.4f gf=%.4f ga=%.4f poss=%.2f shots=%.2f gaf=%.3f fb=%.2f sec=%.2f\n",
           games, (double)(t_blue - t_yellow) / games, (double)t_blue / games, (double)t_yellow / games,
           t_poss / games, (double)t_shots / games, (double)t_ga / games, (double)t_fb / games, sec);
    if (g_coop) fclose(g_coop);
#ifdef SIMURO5_BRANCH_TRACE
    hes_report();
#endif
    return 0;
}
