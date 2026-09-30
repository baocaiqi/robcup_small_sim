#include "simuro5/role_assignment.hpp"
#define TUNABLE_PREFIX "roles."
#include "simuro5/tunable.hpp"
#include <cmath>
#include <utility>

namespace simuro5 {

// 动态主攻（第 82 轮）：原先固定 1 号 = ACTIVE。真机黑匣子：无主球时我方离球最近的
//   队员 77% 不是 ACTIVE（PASSIVE/ASSIST/MID 在站位），ACTIVE 离球中位 67~72cm 还在远离球，
//   无主球争夺 22 次我方只先到 1 次。这里每帧估算各场上队员"到球代价"，明显更优者接任 ACTIVE，
//   原 ACTIVE 接手它的角色（其余分工与阵型不变）。
//   防抖（第 8 轮废弃距离贪心的原因）：代价优势 > kActSwapMargin 且连续 kActSwapFrames 帧，
//   换人后 kActSwapHold 帧内不再换；配合传球/控球、点球、死球期间冻结。
TUNABLE(kActDynamic, 1.0);      // 第 94 轮用户指令开启真机试（sim 犯规 ×5，盯点球）；0 = 回滚
TUNABLE(kActSwapMargin, 20.0);   // cm 等效
TUNABLE(kActSwapFrames, 4.0);
TUNABLE(kActSwapHold, 25.0);
TUNABLE(kActTurnCost, 12.0);     // 每 90° 转向折算 cm
TUNABLE(kActAheadCost, 30.0);    // 人在球的进攻方向前面（要绕回球后）折算 cm
TUNABLE(kActSwapPassive, 1.0);   // PASSIVE 能否接任（0=只在 ASSIST/MID 间换）

static const int kFixedRole[PLAYERS_PER_SIDE] = {ROLE_GOALIE, ROLE_ACTIVE, ROLE_ASSIST, ROLE_MIDFIELD, ROLE_PASSIVE};

static double ball_cost(const WorldModel &wm, int i) {
    const RobotState &r = wm.home[i];
    double dx = wm.ball.x - r.x, dy = wm.ball.y - r.y, d = std::hypot(dx, dy);
    double a = std::fabs(std::remainder(std::atan2(dy, dx) * 180.0 / 3.14159265358979 - r.rot, 360.0));
    if (a > 90.0) a = 180.0 - a;   // 差速车可倒车
    double ahead = (wm.ball.x - r.x) * wm.ctx.attack_dir();   // <0：人在球前
    return d + kActTurnCost * a / 90.0 + (ahead < -5.0 ? kActAheadCost : 0.0);
}

void RoleAssignment::assign(WorldModel &wm) {
    const bool frozen = kActDynamic < 0.5 || !wm.live_play || wm.in_penalty_exec ||
                        !wm.ball.valid;
    if (frozen || wm.active_id < 1 || wm.active_id >= PLAYERS_PER_SIDE ||
        wm.role[0] != ROLE_GOALIE || wm.role[wm.active_id] != ROLE_ACTIVE) {   // 未初始化 / 被外部改写
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) wm.role[i] = kFixedRole[i];
        wm.active_id = 1; wm.active_cand = -1; wm.active_cand_frames = 0; wm.active_hold = 0;
        return;
    }
    if (wm.active_hold > 0) { --wm.active_hold; return; }
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
    if (wm.active_cand_frames < kActSwapFrames) return;
    std::swap(wm.role[cur], wm.role[best]);
    wm.active_id = best; wm.active_cand = -1; wm.active_cand_frames = 0;
    wm.active_hold = static_cast<int>(kActSwapHold);
    wm.shoot_align_frames = 0; wm.shoot_push_count = 0;
    wm.active_ga_frames = 0; wm.active_ga_total = 0;
}

}  // namespace simuro5
