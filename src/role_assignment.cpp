#include "simuro5/role_assignment.hpp"
#define TUNABLE_PREFIX "roles."
#include "simuro5/tunable.hpp"
#include <cmath>
#include <utility>

namespace simuro5 {

// 动态主攻：每帧算各队员到球代价，明显更优者接任 ACTIVE（含防抖滞回）；
// 距离类参数单位 cm，kActDynamic<=0 关闭
TUNABLE(kActDynamic, 1.0);
TUNABLE(kActSwapMargin, 20.0);
TUNABLE(kActSwapFrames, 8.0);
TUNABLE(kActSwapHold, 25.0);
TUNABLE(kActTurnCost, 12.0);
TUNABLE(kActAheadCost, 30.0);
TUNABLE(kActSwapPassive, 1.0);
// 分区冻结：球在己方半场(危险区)时换人后冻结 kActSwapHoldDanger 帧，否则用 Attack 帧
TUNABLE(kActDangerDepth, 110.0);
TUNABLE(kActSwapHoldDanger, 70.0);
TUNABLE(kActSwapHoldAttack, -1.0);

static const int kFixedRole[PLAYERS_PER_SIDE] = {ROLE_GOALIE, ROLE_ACTIVE, ROLE_ASSIST, ROLE_MIDFIELD, ROLE_PASSIVE};

static double ball_cost(const WorldModel &wm, int i) {
    const RobotState &r = wm.home[i];
    double dx = wm.ball.x - r.x, dy = wm.ball.y - r.y, d = std::hypot(dx, dy);
    double a = std::fabs(std::remainder(std::atan2(dy, dx) * 180.0 / 3.14159265358979 - r.rot, 360.0));
    if (a > 90.0) a = 180.0 - a;   // 差速车可倒车
    double ahead = (wm.ball.x - r.x) * wm.ctx.attack_dir();
    return d + kActTurnCost * a / 90.0 + (ahead < -5.0 ? kActAheadCost : 0.0);
}

// 死球/重启期"立刻换人"（2026-10-11）：见下方 assign() 里的说明。
// ⚠️ 默认 0 = 关。实测（40 局 × 3 种子 × scripted/wall，`--restart-gamestate` 口径）：
//   "只保留角色表" net 3.975 / 自己门区2+人 8.1 帧；再叠加"立刻换人" → 3.192；
//   再把"重启抢球"也开上 → 3.083。⇒ 越激进越差，**只留保留角色表那一条**。
TUNABLE(kActDeadInstant, 0.0);
// 额外的"球静止"判定（cm/帧）：默认 0 = 关。只用 runtime_phase 判重启 —— 实测（sim 默认口径球速恒 0、
// 真机慢滚球也常见）一旦拿"球速小"当条件，本规则会整场都在免确认换人，把滞回/冻结全废掉。
TUNABLE(kActDeadStill, 0.0);
// 重启期是否保留原角色表：1 = 保留（新，默认）；0 = 回滚成"每帧重置成固定档"（HEAD 行为，A/B 对照用）
TUNABLE(kActKeepRoleDead, 1.0);
// 重启期(球还钉在摆放点)换人要多快：>0 时用这个更短的确认帧数、且不受 active_hold 冻结；
//   0 = 关（与活球期同口径：8 帧确认 + 球在我方 110cm 内时 70 帧冻结）
TUNABLE(kActDeadConfirm, 0.0);

void RoleAssignment::assign(WorldModel &wm) {
    const bool table_ok = wm.active_id >= 1 && wm.active_id < PLAYERS_PER_SIDE &&
                          wm.role[0] == ROLE_GOALIE && wm.role[wm.active_id] == ROLE_ACTIVE;
    // ⚠️ 合并取舍（2026-10-12）：队员 d9b1225 在 frozen 里加了 `runtime_phase != Running`，
    //   效果等于"重启期每帧重置成固定档" —— 这正是我们第 72 轮用 kActKeepRoleDead 修掉的行为
    //   （240 局 A/B：净胜 3.383→3.975，自家门区"2+ 人"犯规风险 −41%）。
    //   故这里保留我们的旋钮口径；想退回那个行为把 kActKeepRoleDead 设成 0 即可。
    const bool old_freeze = wm.runtime_phase != RuntimePhase::Running && kActKeepRoleDead < 0.5;
    // 队员 ff2e542：重复 ACTIVE / 未初始化 / 被外部改写时恢复唯一固定初始角色
    int active_count = 0;
    for (int i = 1; i < PLAYERS_PER_SIDE; ++i)
        if (wm.role[i] == ROLE_ACTIVE) ++active_count;
    if (kActDynamic < 0.5 || wm.in_penalty_exec || !wm.ball.valid || old_freeze || !table_ok ||
        active_count != 1) {   // 未初始化、重复主攻或被外部改写
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) wm.role[i] = kFixedRole[i];
        wm.active_id = 1; wm.active_cand = -1; wm.active_cand_frames = 0; wm.active_hold = 0;
        return;
    }
    // ★ 重启/死球：球还没离开摆放点(runtime_phase=RestartSetup)或基本静止 → **本帧就把主攻交给离球最近的人**，
    //   不等 kActSwapFrames(8 帧) 确认、也不受 active_hold(球在我方 110cm 内时 70 帧) 冻结。
    //   为什么必须立刻：逐帧取证（sim_trace --target-csv）——复位那一帧离球 7.5cm 的那台是"抢球"，
    //   49cm 外的那台才是"主攻"；7 帧(0.18s)后才换过来，而这 0.18 秒球已经被推走 16~21cm。
    //   球一旦滚起来（live_play=true 且球速起来）就退回原来的滞回口径，避免整场都在换人。
    const bool ball_dead = wm.runtime_phase != RuntimePhase::Running ||
                           (kActDeadStill > 0.0 && std::hypot(wm.ball.vx, wm.ball.vy) <= kActDeadStill);
    const bool dead = wm.runtime_phase != RuntimePhase::Running;   // 球还钉在重启摆放点 = 重启窗口
    if (kActDeadInstant > 0.5 && ball_dead) {
        const int cur = wm.active_id;
        int best = -1;
        double best_c = ball_cost(wm, cur) - kActSwapMargin;
        for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {
            if (i == cur || wm.ga_cooldown[i] > 0) continue;
            double c = ball_cost(wm, i);
            if (c < best_c) { best_c = c; best = i; }
        }
        if (best >= 0) {
            std::swap(wm.role[cur], wm.role[best]);
            wm.active_id = best;
            wm.shoot_align_frames = 0; wm.shoot_push_count = 0;
            wm.active_ga_frames = 0; wm.active_ga_total = 0;
        }
        wm.active_cand = -1; wm.active_cand_frames = 0; wm.active_hold = 0;
        return;
    }
    if (wm.active_hold > 0 && !(dead && kActDeadConfirm > 0.0)) { --wm.active_hold; return; }
    if (wm.coop_pass_task.active || wm.coop_ball_control.active) { wm.active_cand_frames = 0; return; }
    const int cur = wm.active_id;
    int best = -1;
    double best_c = ball_cost(wm, cur) - kActSwapMargin;
    for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {
        if (i == cur || wm.ga_cooldown[i] > 0) continue;
        if (wm.role[i] == ROLE_PASSIVE && kActSwapPassive < 0.5) continue;
        double c = ball_cost(wm, i);
        if (c < best_c) { best_c = c; best = i; }
    }
    if (best < 0) { wm.active_cand = -1; wm.active_cand_frames = 0; return; }
    wm.active_cand_frames = (best == wm.active_cand) ? wm.active_cand_frames + 1 : 1;
    wm.active_cand = best;
    if (wm.active_cand_frames < ((dead && kActDeadConfirm > 0.0) ? kActDeadConfirm : kActSwapFrames)) return;
    std::swap(wm.role[cur], wm.role[best]);
    wm.active_id = best; wm.active_cand = -1; wm.active_cand_frames = 0;
    const double depth = (wm.ball.x - wm.ctx.our_goal_x()) * wm.ctx.attack_dir();
    const double zone_hold = depth < kActDangerDepth ? kActSwapHoldDanger : kActSwapHoldAttack;
    wm.active_hold = static_cast<int>(zone_hold < 0.0 ? kActSwapHold : zone_hold);
    wm.shoot_align_frames = 0; wm.shoot_push_count = 0;
    wm.active_ga_frames = 0; wm.active_ga_total = 0;
}

}  // namespace simuro5
