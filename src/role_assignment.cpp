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

// 球权中心化换人口径（见 roles.cpp update_our_possession）：
//   无球 = 全队抢球，谁离球近谁当 ACTIVE——不给现任留 20cm 让位余量、不攒 8 帧、换完不冻结；
//   有球 = 球在我们脚下，谁拿着谁继续，一帧都不换。
//   kActBallCentricSwap 关掉即整段回到旧的让位/确认/冻结三件套。
TUNABLE(kActBallCentricSwap, 1.0);
// 2 帧（=50ms）不是犹豫，纯粹是滤掉 40Hz 下的单帧抖动；0 让位余量 / 0 冻结
TUNABLE(kActBallCentricMargin, 0.0);
TUNABLE(kActBallCentricFrames, 2.0);

static const int kFixedRole[PLAYERS_PER_SIDE] = {ROLE_GOALIE, ROLE_ACTIVE, ROLE_ASSIST, ROLE_MIDFIELD, ROLE_PASSIVE};

static double ball_cost(const WorldModel &wm, int i) {
    const RobotState &r = wm.home[i];
    double dx = wm.ball.x - r.x, dy = wm.ball.y - r.y, d = std::hypot(dx, dy);
    double a = std::fabs(std::remainder(std::atan2(dy, dx) * 180.0 / 3.14159265358979 - r.rot, 360.0));
    if (a > 90.0) a = 180.0 - a;   // 差速车可倒车
    double ahead = (wm.ball.x - r.x) * wm.ctx.attack_dir();
    return d + kActTurnCost * a / 90.0 + (ahead < -5.0 ? kActAheadCost : 0.0);
}

void RoleAssignment::assign(WorldModel &wm) {
    const bool frozen = kActDynamic < 0.5 || !wm.live_play || wm.in_penalty_exec ||
                        !wm.ball.valid;
    if (frozen || wm.active_id < 1 || wm.active_id >= PLAYERS_PER_SIDE ||
        wm.role[0] != ROLE_GOALIE || wm.role[wm.active_id] != ROLE_ACTIVE) {   // 未初始化或被外部改写
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) wm.role[i] = kFixedRole[i];
        wm.active_id = 1; wm.active_cand = -1; wm.active_cand_frames = 0; wm.active_hold = 0;
        return;
    }
    if (wm.active_hold > 0) { --wm.active_hold; return; }
    if (wm.coop_pass_task.active || wm.coop_ball_control.active) { wm.active_cand_frames = 0; return; }
    // 有球模式：球在我们脚下，持球人就是 ACTIVE——换人 = 交接那一下把球白送给对手
    const bool bc = (kActBallCentricSwap >= 0.5);
    if (bc && wm.our_possession != 0) { wm.active_cand_frames = 0; wm.active_hold = 0; return; }
    const int cur = wm.active_id;
    int best = -1;
    // 无球模式：0 让位余量，只要比现任更近就换（挑战者不需要先比现任近 20cm）
    const double margin = bc ? kActBallCentricMargin : kActSwapMargin;
    const double need_frames = bc ? kActBallCentricFrames : kActSwapFrames;
    double best_c = ball_cost(wm, cur) - margin;
    for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {
        if (i == cur || wm.ga_cooldown[i] > 0) continue;
        if (wm.role[i] == ROLE_PASSIVE && kActSwapPassive < 0.5) continue;
        double c = ball_cost(wm, i);
        if (c < best_c) { best_c = c; best = i; }
    }
    if (best < 0) { wm.active_cand = -1; wm.active_cand_frames = 0; return; }
    wm.active_cand_frames = (best == wm.active_cand) ? wm.active_cand_frames + 1 : 1;
    wm.active_cand = best;
    if (wm.active_cand_frames < need_frames) return;
    std::swap(wm.role[cur], wm.role[best]);
    wm.active_id = best; wm.active_cand = -1; wm.active_cand_frames = 0;
    if (bc) {
        wm.active_hold = 0;   // 无球抢球不冻结：下一帧球更近的人立刻能接班
    } else {
        const double depth = (wm.ball.x - wm.ctx.our_goal_x()) * wm.ctx.attack_dir();
        const double zone_hold = depth < kActDangerDepth ? kActSwapHoldDanger : kActSwapHoldAttack;
        wm.active_hold = static_cast<int>(zone_hold < 0.0 ? kActSwapHold : zone_hold);
    }
    wm.shoot_align_frames = 0; wm.shoot_push_count = 0;
    wm.active_ga_frames = 0; wm.active_ga_total = 0;
}

}  // namespace simuro5
