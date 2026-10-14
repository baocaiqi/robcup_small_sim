// local_planner.cpp — 无球跑位局部避让 + DWA 影子（参考 [TAC] 势场 / DWA 思想，自研实现）
//
// 做法（人话）：角色函数已经决定"去哪"，这里只管"怎么绕"。
//   合力 F = 指向目标的单位引力 + 近身对手/队友/边墙的线性斥力 + （我方控球时）威胁场斥力 −k·grad
//   前视点 = 车位 + F 方向 × min(到目标距离, 前视长度)，再交给 motion::position。
//   偏转太小（<最小偏角）就不改 ⇒ 空旷场地上行为与原来逐帧一致；偏转封顶 ⇒ 不会掉头乱跑。
// 和对手的区别：对手直接把合力换算成轮速，我们只改目标点，轮速仍由已标定的 motion.cpp 出；
//   常量全是我们按自己场地口径定的起点，不是对手的观察值。还没真机验证。
#include "simuro5/local_planner.hpp"
#include "simuro5/threat_field.hpp"
#include "simuro5/role_assignment.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/motion.hpp"
#include "simuro5/blackbox.hpp"
#define TUNABLE_PREFIX "plan."
#include "simuro5/tunable.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

// 总开关：0=关 1=影子（只算+日志）2=转向生效
TUNABLE(kPlanMode, 2.0);
// 斥力：半径 R(cm) 内线性增长，贴身时等于 K（引力恒为 1）
TUNABLE(kPlanOppR, 22.0);
TUNABLE(kPlanOppK, 1.2);
TUNABLE(kPlanMateR, 16.0);
TUNABLE(kPlanMateK, 0.8);
TUNABLE(kPlanWallR, 10.0);
TUNABLE(kPlanWallK, 1.0);
// 威胁场斥力系数（只在我方控球时加，让无球队员往"对手少的空当"偏）；0 = 不用威胁场
TUNABLE(kPlanThreatK, 40.0);
// 前视长度 / 离目标多近就不再干预 / 目标离球多近视为"去抢球"不干预（cm）
TUNABLE(kPlanLook, 30.0);
TUNABLE(kPlanMinDist, 20.0);
TUNABLE(kPlanBallSkip, 25.0);
// 偏角：小于最小值不改目标；大于最大值截断（度）
TUNABLE(kPlanMinDeflect, 5.0);
TUNABLE(kPlanMaxDeflect, 60.0);
// DWA 影子：轮距(cm，取 sim 标定口径)、前瞻 5 步×0.1s、线速度窗口 ±40、出界余量 4cm
TUNABLE(kPlanWheelBase, 10.0);
TUNABLE(kPlanDwaLogEvery, 200.0);

int local_planner_mode() { return (int)kPlanMode; }

namespace {

struct PlanStats {
    long considered = 0, steered = 0;
    long dwa_total = 0, dwa_agree = 0;
    double sum_deflect = 0.0;
};
PlanStats &stats() { static PlanStats s; return s; }

bool steered_this_frame[PLAYERS_PER_SIDE] = {false, false, false, false, false};

inline void add_repulse(double px, double py, double ox, double oy, double R, double K,
                        double &fx, double &fy) {
    const double dx = px - ox, dy = py - oy;
    const double d = std::hypot(dx, dy);
    if (d >= R || d < 1e-6) return;
    const double m = K * (1.0 - d / R);
    fx += m * dx / d;
    fy += m * dy / d;
}

}  // namespace

void plan_steer(WorldModel &wm, int id) {
    steered_this_frame[id] = false;
    if (kPlanMode <= 0.0) return;
    if (id <= 0 || id >= PLAYERS_PER_SIDE) return;
    if (wm.runtime_phase != RuntimePhase::Running || wm.in_penalty_exec) return;
    const int role = wm.role[id];
    if (role != ROLE_PASSIVE && role != ROLE_ASSIST && role != ROLE_MIDFIELD) return;
    RobotState &r = wm.home[id];
    if (r.vl == 0.0 && r.vr == 0.0) return;                       // 角色主动站定/让位：不碰
    const double tx = r.tgt_x, ty = r.tgt_y;
    if (!std::isfinite(tx) || !std::isfinite(ty)) return;
    const double de = std::hypot(tx - r.x, ty - r.y);
    if (de < kPlanMinDist) return;                                 // 快到了：保证到位精度
    if (dist(tx, ty, wm.ball.x, wm.ball.y) < kPlanBallSkip) return; // 去抢球：不绕
    ++stats().considered;

    // 引力：指向目标的单位向量
    const double ax = (tx - r.x) / de, ay = (ty - r.y) / de;
    double fx = ax, fy = ay;
    // 对手斥力；目标点旁边那个对手（盯人对象）不算，否则盯人永远贴不上去
    for (int j = 0; j < PLAYERS_PER_SIDE; ++j) {
        const double ox = wm.opp[j].x, oy = wm.opp[j].y;
        if (dist(ox, oy, tx, ty) < kPlanOppR) continue;
        add_repulse(r.x, r.y, ox, oy, kPlanOppR, kPlanOppK, fx, fy);
    }
    for (int j = 0; j < PLAYERS_PER_SIDE; ++j) {
        if (j == id) continue;
        add_repulse(r.x, r.y, wm.home[j].x, wm.home[j].y, kPlanMateR, kPlanMateK, fx, fy);
    }
    // 边墙：离墙 R 内往场内推
    const double W = TeamContext::FIELD_LENGTH, H = TeamContext::FIELD_WIDTH;
    if (r.x < kPlanWallR)     fx += kPlanWallK * (1.0 - r.x / kPlanWallR);
    if (r.x > W - kPlanWallR) fx -= kPlanWallK * (1.0 - (W - r.x) / kPlanWallR);
    if (r.y < kPlanWallR)     fy += kPlanWallK * (1.0 - r.y / kPlanWallR);
    if (r.y > H - kPlanWallR) fy -= kPlanWallK * (1.0 - (H - r.y) / kPlanWallR);
    // 威胁场：只在我方控球时让无球队员避开对手密集区（防守时我们本来就该往危险处去）
    if (kPlanThreatK > 0.0 && wm.we_have_ball && wm.threat.valid) {
        double gx = 0, gy = 0;
        threat_grad(wm.threat, r.x, r.y, &gx, &gy);
        fx -= kPlanThreatK * gx;
        fy -= kPlanThreatK * gy;
    }

    const double fn = std::hypot(fx, fy);
    if (fn < 1e-6) return;
    // 合力与引力的夹角（带符号）
    double ang = std::atan2(ax * fy - ay * fx, ax * fx + ay * fy) * 180.0 / SIMURO5_PI;
    if (std::fabs(ang) < kPlanMinDeflect) return;                  // 偏得少：保持原样
    ang = clamp(ang, -kPlanMaxDeflect, kPlanMaxDeflect);
    const double rad = ang * SIMURO5_PI / 180.0;
    const double dx = ax * std::cos(rad) - ay * std::sin(rad);
    const double dy = ax * std::sin(rad) + ay * std::cos(rad);
    const double L = std::min(de, (double)kPlanLook);
    double nx = clamp(r.x + dx * L, 6.0, W - 6.0);
    double ny = clamp(r.y + dy * L, 6.0, H - 6.0);
    // 前视点落进任一门区就放弃改动，交给原有纪律层
    if (in_goal_area_rule(wm.ctx, nx, ny, 8.0, 6.0) || in_opp_goal_area(wm.ctx, nx, ny)) return;

    stats().sum_deflect += std::fabs(ang);
    if (kPlanMode < 1.5) return;                                   // 影子：只统计
    const int mode = r.tgt_mode;
    motion::position(r, nx, ny, mode == (int)motion::TM_PASS ? motion::TM_PASS : motion::TM_STOP);
    r.tgt_x = tx; r.tgt_y = ty; r.tgt_mode = mode;                 // 保留"真目标"，下游纪律层仍按它判断
    steered_this_frame[id] = true;
    ++stats().steered;
}

// DWA 影子：在当前轮速附近试 7×9 组 (v,w)，各往前推 0.5s，按"接近目标/朝向/离对手/速度"打分，
//   看最优解和实际下发的轮速是否同向（转向同号且线速度差 ≤30）。只计数，不驱动。
void plan_frame_end(const WorldModel &wm) {
    if (kPlanMode <= 0.0 || !bb::kEnabled) return;
    PlanStats &s = stats();
    const double L = std::max(1.0, (double)kPlanWheelBase);
    for (int id = 1; id < PLAYERS_PER_SIDE; ++id) {
        const RobotState &r = wm.home[id];
        const double tx = r.tgt_x, ty = r.tgt_y;
        const double d0 = std::hypot(tx - r.x, ty - r.y);
        if (d0 < kPlanMinDist || !std::isfinite(d0)) continue;
        const double v_cmd = 0.5 * (r.vl + r.vr), w_cmd = (r.vr - r.vl) / L;
        double best = -1e9, bv = 0, bw = 0;
        for (int i = 0; i < 7; ++i) {
            const double v = clamp(v_cmd - 40.0 + i * (80.0 / 6.0), -150.0, 150.0);
            for (int j = 0; j < 9; ++j) {
                const double w = w_cmd + (j - 4) * 1.5;            // rad/s，±6
                double x = r.x, y = r.y, th = r.rot * SIMURO5_PI / 180.0, clear = 1e9;
                bool out = false;
                for (int k = 0; k < 5; ++k) {
                    th += w * 0.1;
                    x += std::cos(th) * v * 0.1;
                    y += std::sin(th) * v * 0.1;
                    if (x < 4 || x > 216 || y < 4 || y > 176) out = true;
                    for (int o = 0; o < PLAYERS_PER_SIDE; ++o)
                        clear = std::min(clear, dist(x, y, wm.opp[o].x, wm.opp[o].y));
                }
                const double prog = clamp((d0 - std::hypot(tx - x, ty - y)) / 60.0, -1.0, 1.0);
                const double want = std::atan2(ty - y, tx - x);
                double he = std::fabs(std::remainder(want - th, 2.0 * SIMURO5_PI));
                he = std::min(he, SIMURO5_PI - he);                  // 差速轮可倒车：车头或车尾对准都算
                const double score = 0.4 * prog + 0.25 * (1.0 - he / (0.5 * SIMURO5_PI)) +
                                     0.25 * (out ? 0.0 : std::min(clear / 25.0, 1.0)) +
                                     0.1 * std::fabs(v) / 150.0;
                if (score > best) { best = score; bv = v; bw = w; }
            }
        }
        ++s.dwa_total;
        const bool same_turn = (bw * w_cmd >= 0.0) || std::fabs(bw - w_cmd) < 1.5;
        if (same_turn && std::fabs(bv - v_cmd) <= 30.0) ++s.dwa_agree;
    }
    const long every = std::max(1L, (long)kPlanDwaLogEvery);
    if ((bb::st().frame % every) == 0) {
        if (std::FILE *f = bb::fp()) {
            std::fprintf(f, "PL,%ld,%d,%ld,%ld,%.1f,%ld,%ld\n", bb::st().frame, (int)kPlanMode,
                         s.considered, s.steered,
                         s.considered ? s.sum_deflect / s.considered : 0.0,
                         s.dwa_total, s.dwa_agree);
        }
    }
}

}  // namespace simuro5
