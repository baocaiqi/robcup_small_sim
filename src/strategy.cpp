#include "simuro5/strategy.hpp"
#include "simuro5/roles.hpp"
#include "simuro5/motion.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/defense.hpp"
#include <cmath>

namespace simuro5 {

// ============================================================
// 攻防状态机（队员 A：全局调度）
//
// 给全队加「记忆 + 滞回 + 事件」：
//   1. 滞回：持球/失球需连续 kStateHysteresisFrames 帧才翻转状态，
//      消除球在双方都能抢到时进攻/防守的每帧横跳（治"角色/阵型抖动"）；
//   2. 事件：state_transition 标记本帧切换，供后续模块做即时响应
//      （丢球→立即收缩，夺球→立即前插，无需等下帧重算）；
//   3. 威胁等级改由「状态 + 球位」稳定输出，不再随单帧球权抖动。
// ============================================================
static const int kStateHysteresisFrames = 3;   // 滞回帧数（可调，见 docs/06）

void Strategy::run(WorldModel &wm) {
    // 1. 局势分析（球权/半场/禁区）
    Situation sit = sit_.analyze(wm);
    wm.we_have_ball = sit.we_have_ball;

    // 2. 攻防状态机（滞回 + 事件 + 威胁）
    update_team_state(wm);

    // 3. 站位参考点（状态感知：进攻锚点 vs 防守锚点）
    sit_.update_stand_points(wm);

    // 4. 角色分配（固定角色：0=GK 1=ACTIVE 2=ASSIST 3=MID 4=PASSIVE）
    ra_.assign(wm);

    // 5. 按角色执行（薄壳调度）
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        switch (wm.role[i]) {
            case ROLE_GOALIE:   run_goalie(wm, i); break;
            case ROLE_ACTIVE:   run_active(wm, i); break;
            case ROLE_PASSIVE:  run_passive(wm, i); break;
            case ROLE_ASSIST:   run_assist(wm, i); break;
            case ROLE_MIDFIELD: run_midfield(wm, i); break;
            default:            motion::stop(wm.home[i]); break;
        }
    }
}

void Strategy::update_team_state(WorldModel &wm) {
    // 滞回计数
    if (wm.we_have_ball) { ++wm.possession_frames; wm.no_possession_frames = 0; }
    else                 { ++wm.no_possession_frames; wm.possession_frames = 0; }

    TeamState prev = wm.team_state;
    if (wm.team_state == TS_DEFENSE && wm.possession_frames >= kStateHysteresisFrames)
        wm.team_state = TS_ATTACK;
    else if (wm.team_state == TS_ATTACK && wm.no_possession_frames >= kStateHysteresisFrames)
        wm.team_state = TS_DEFENSE;
    wm.state_transition = (wm.team_state != prev);

    // 威胁等级：由「状态 + 球位」稳定输出（不再随单帧球权抖动）
    wm.threat_level = threat_from_state(wm);
}

double Strategy::threat_from_state(const WorldModel &wm) const {
    const TeamContext &ctx = wm.ctx;
    if (wm.team_state == TS_ATTACK) return 0.1;   // 我方控球：低威胁
    // 防守态：按球的位置分级
    if (in_penalty_area(ctx, wm.ball.x, wm.ball.y)) return 1.0;   // 球在己方罚球区
    bool our_half = ctx.attack_dir() > 0 ? (wm.ball.x < 110.0) : (wm.ball.x > 110.0);
    double threat = our_half ? 0.6 : 0.4;

    // 球速方向加成（team-level danger）：球快速朝门滚时提前升档，让全队早回防。
    //   danger = 球朝己方门的速度分量（defense.hpp 点积投影），横滚/背离=0，不会误判。
    //   朝门且快 → 对方半场 0.4→0.6（提前触发人盯人）、己方半场 0.6→0.8（预留更高档）。
    //   下游阈值：>0.3 assist/mid 回防、>=0.6 passive 人盯人——升 0.6 是真正的提前回防收益。
    const double kThreatDangerSpeed = 6.0;   // cm/帧：朝门有效速度阈值（同 kDribbleSpeed 量级，可调）
    if (ball_danger_speed(wm) > kThreatDangerSpeed) {
        threat = our_half ? 0.8 : 0.6;
    }
    return threat;
}

}  // namespace simuro5
