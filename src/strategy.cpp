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

    // 4.5 清道夫指派（球在防守三区拉边时，抽一个区域防守者钉中路封远门柱/横传）
    update_sweeper(wm);

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

void Strategy::update_sweeper(WorldModel &wm) {
    wm.sweeper_id = -1;
    if (wm.team_state != TS_DEFENSE) return;

    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;

    // 触发：球在本方防守三区（离门 1/3 场以内）且明显拉边（|y-90|>30）。
    //   这种局面球-门连线的静态站位会把所有区域防守者都拽到球侧（Y 同侧），
    //   中路/远门柱真空，横传或内切一打就穿——抽一个区域防守者回收中路兜底。
    const double third = TeamContext::FIELD_LENGTH / 3.0;
    bool our_third = (ctx.attack_dir() > 0) ? (bx < third)
                                            : (bx > TeamContext::FIELD_LENGTH - third);
    if (!our_third || std::fabs(by - 90.0) <= 30.0) return;

    // 从两个区域防守者(ASSIST/MIDFIELD)里挑「离球更远」的那个做清道夫：
    //   离球近的继续压上/断球，离球远的回收中路（少跑路、也正好在远侧）。
    int ids[2] = { -1, -1 };
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (wm.role[i] == ROLE_ASSIST)        ids[0] = i;
        else if (wm.role[i] == ROLE_MIDFIELD) ids[1] = i;
    }
    double d0 = (ids[0] >= 0) ? dist(bx, by, wm.home[ids[0]].x, wm.home[ids[0]].y) : -1.0;
    double d1 = (ids[1] >= 0) ? dist(bx, by, wm.home[ids[1]].x, wm.home[ids[1]].y) : -1.0;
    if (d0 < 0.0 && d1 < 0.0) return;
    wm.sweeper_id = (d0 >= d1) ? ids[0] : ids[1];

    // 清道夫站位：罚球区前缘外侧(gx+85)、中路 y=90。
    //   y=90 封中路与远门柱；x=85 尊重「非门将不进己方罚球区」纪律(见 situation.cpp)。
    wm.sweeper_x = ctx.our_goal_x() + ctx.attack_dir() * 85.0;
    wm.sweeper_y = 90.0;
}

}  // namespace simuro5
