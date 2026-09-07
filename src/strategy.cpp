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

// 反击快攻窗口帧数（docs/13 攻击强化 方案 A）：
//   断球瞬间起 30 帧（≈0.75s）内，assist/midfield 豁免回防条件立即前插接应，
//   让 ACTIVE 断球后有传球选择；窗口过后恢复正常回防逻辑。
//   30 帧约等于 demo 就地反抢到位所需时间——窗口内把球传/带过半场即成功。
static const int kCounterWindowFrames = 30;

void Strategy::run(WorldModel &wm) {
    // 1. 局势分析（球权/半场/禁区）
    Situation sit = sit_.analyze(wm);
    wm.we_have_ball = sit.we_have_ball;

    // 1.5 我方主罚点球执行期标志（供 roles 区分"对方门球"vs"我方点球"：
    //   两者都是"球静止在对方门区"，但点球必须去踢，门球要等对方开出）
    {
        bool we_take = (wm.ctx.is_blue && wm.game_state == PM_PenaltyKick_Yellow) ||
                       (!wm.ctx.is_blue && wm.game_state == PM_PenaltyKick_Blue);
        if (we_take) {
            wm.in_penalty_exec = true;
        } else if (wm.in_penalty_exec) {
            // 球离开罚球点（被踢出/被碰走）或 PlayMode 已切走 → 执行期结束
            bool ball_leaves = std::hypot(wm.ball.vx, wm.ball.vy) > 3.0 ||
                               std::fabs(wm.ball.x - wm.ctx.opp_goal_x()) > 60.0 ||
                               std::fabs(wm.ball.y - 90.0) > 30.0;
            if (wm.game_state != PM_PlayOn || ball_leaves) wm.in_penalty_exec = false;
        }
    }

    // 2. 攻防状态机（滞回 + 事件 + 威胁）
    update_team_state(wm);

    // 3. 站位参考点（状态感知：进攻锚点 vs 防守锚点）
    sit_.update_stand_points(wm);

    // 4. 角色分配（固定角色：0=GK 1=ACTIVE 2=ASSIST 3=MID 4=PASSIVE）
    ra_.assign(wm);

    // 4.5 清道夫指派（球在防守三区拉边时，抽一个区域防守者钉中路封远门柱/横传）
    update_sweeper(wm);

    // 4.6 多人盯人分配（威胁高时，PASSIVE/ASSIST/MIDFIELD 贴防对方多核，链式防守）
    assign_marks(wm);

    // 5. 按角色执行（薄壳调度）
    //    冷却期门区禁令（docs/13 方案 C 扩展）：撤出刚触发 30 帧内，本角色若还在
    //    对方门区（且非攻门作业/点球执行），直接指令门外、**跳过角色函数**——
    //    让角色再跑一帧会把 chase/站位目标与撤出目标交替覆盖 vl/vr，机器人
    //    原地抖振卡在门区（实测蓝1 滞留 45 帧的根因）。
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (wm.role[i] == ROLE_GOALIE) { run_goalie(wm, i); continue; }
        if (wm.ga_cooldown[i] > 0) {
            --wm.ga_cooldown[i];
            bool in_ga = in_opp_goal_area(wm.ctx, wm.home[i].x, wm.home[i].y);
            bool ball_in_ga = in_opp_goal_area(wm.ctx, wm.ball.x, wm.ball.y);
            bool shooting_work = ball_in_ga &&
                dist(wm.home[i].x, wm.home[i].y, wm.ball.x, wm.ball.y) <= 25.0;
            bool penalty = wm.in_penalty_exec && wm.role[i] == ROLE_ACTIVE;
            if (in_ga && !shooting_work && !penalty) {
                double ogx = wm.ctx.opp_goal_x(), ad = wm.ctx.attack_dir();
                motion::position(wm.home[i], ogx - ad * 70.0, clamp(wm.home[i].y, 72.5, 107.5));
                continue;   // 冷却期禁令：跳过角色函数
            }
        }
        switch (wm.role[i]) {
            case ROLE_ACTIVE:   run_active(wm, i); break;
            case ROLE_PASSIVE:  run_passive(wm, i); break;
            case ROLE_ASSIST:   run_assist(wm, i); break;
            case ROLE_MIDFIELD: run_midfield(wm, i); break;
            default:            motion::stop(wm.home[i]); break;
        }
    }

    // 5.5 全角色对方门区停留时限兜底（docs/13 方案 C 扩展）：
    //   平台判罚看**实际位置**，clamp 只约束站位点，挡不住追球/锚点振荡实际进区
    //   （sim 实测：ACTIVE 追角区球路径穿门区滞留 36 帧、ASSIST 锚点停门区边缘）。
    //   ACTIVE 的 run_active 内方案 C 撤出照旧，这里是兜底：豁免"球在门区且自己
    //   贴球(≤25cm)"——门前争抢/补射/带球攻门不算滞留；其余（含追球穿区）连续
    //   >15 帧 → 强制撤到门区前缘外 70cm + 冷却 30 帧（冷却由 5 的调度前置拦截执行）。
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (wm.role[i] == ROLE_GOALIE) continue;
        double ogx = wm.ctx.opp_goal_x(), ad = wm.ctx.attack_dir();
        double hold_x = ogx - ad * 70.0, hold_y = clamp(wm.home[i].y, 72.5, 107.5);
        bool in_ga = in_opp_goal_area(wm.ctx, wm.home[i].x, wm.home[i].y);
        if (in_ga) {
            bool ball_in_ga = in_opp_goal_area(wm.ctx, wm.ball.x, wm.ball.y);
            bool shooting_work = ball_in_ga &&
                dist(wm.home[i].x, wm.home[i].y, wm.ball.x, wm.ball.y) <= 25.0;
            if (!shooting_work && ++wm.ga_overstay[i] > 15) {
                wm.ga_overstay[i] = 0;
                wm.ga_cooldown[i] = 30;
                motion::position(wm.home[i], hold_x, hold_y);
            }
        } else {
            wm.ga_overstay[i] = 0;
        }
    }
}

void Strategy::update_team_state(WorldModel &wm) {
    // 滞回计数
    if (wm.we_have_ball) { ++wm.possession_frames; wm.no_possession_frames = 0; }
    else                 { ++wm.no_possession_frames; wm.possession_frames = 0; }

    // 反击快攻窗口（docs/13 方案 A）：
    //   失球→持球转换帧 = 断球成功，置窗口让 assist/mid 立即前插（不等滞回切进攻态）；
    //   窗口每帧递减，归零后恢复正常回防。
    if (wm.we_have_ball && !wm.prev_we_have_ball && wm.game_state == PM_PlayOn) {
        wm.counter_attack_frames = kCounterWindowFrames;
    }
    if (wm.counter_attack_frames > 0) --wm.counter_attack_frames;
    wm.prev_we_have_ball = wm.we_have_ball;

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
    if (wm.team_state == TS_ATTACK) return 0.1;   // 我方控球：低威胁（下游阈值 0.1 精确比较）

    // —— 连续威胁函数（docs/14 #2，P0-2 真机驱动改造）——
    //   原实现 = 半场二值(0.6/0.4) + 球速一档升(→0.8/0.6)，球位在阈值附近会跳变；
    //   本版 = 空间连续 × 侧偏 × 持球 × 球速 四因子连续调制，语义对齐下游阈值：
    //     >0.3 assist/mid 回防、>=0.6 passive 人盯人、罚球区≈1.0 全员回缩。
    const double gx = ctx.our_goal_x();
    const double bx = wm.ball.x, by = wm.ball.y;
    const double dist_x = std::fabs(bx - gx);          // 到己方门线 x 距离 0..220

    // 1) 空间因子（取代半场二值）：门线=1.0 → 罚球区边缘(80cm)≈0.86 → 中线=0.62 →
    //    对方门线≈0.28。分段线性、单调无跳变；己方半场内越靠门威胁越高。
    double spatial;
    if (dist_x <= 110.0) spatial = 1.0 - 0.38 * (dist_x / 110.0);
    else                 spatial = 0.62 - 0.34 * ((dist_x - 110.0) / 110.0);

    // 2) 侧偏因子：球偏离门心 y=90 越远，直接射门角度越小 → 微降（贴边最多 -15%）。
    //    治原实现的盲点：球贴着边线纵深推进时威胁与中路同档，回防过度。
    const double side = std::min(1.0, std::fabs(by - 90.0) / 90.0);
    const double side_f = 1.0 - 0.15 * side;

    // 3) 持球因子（仅己方半场生效）：对方贴球(≤15cm)能立刻起脚/推进 → ×1.15；
    //    球孤立(≥40cm)大概率被我方夺回 → ×0.8；线性过渡。
    //    对方半场不乘：球没过中线前不因贴球提前人盯人（原语义：只有快速推进才升档）。
    double press_f = 1.0;
    if (dist_x <= 110.0) {
        double opp_dmin = 1e9;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i)
            opp_dmin = std::min(opp_dmin, dist(bx, by, wm.opp[i].x, wm.opp[i].y));
        press_f = 1.15;
        if (opp_dmin > 15.0)
            press_f = 1.15 - 0.35 * std::min(1.0, (opp_dmin - 15.0) / 25.0);
    }

    // 4) 球速因子（team-level danger，连续取代原 >6 单档）：
    //    danger = 球朝己方门的速度分量（defense.hpp 点积投影，横滚/背离=0）。
    //    0 → ×1.0；12cm/帧(≈480cm/s 高速射门) → ×1.2；24 → ×1.4（封顶）。
    const double danger = ball_danger_speed(wm);
    double speed_f = 1.0 + 0.0167 * danger;
    if (speed_f > 1.4) speed_f = 1.4;

    double t = spatial * side_f * press_f * speed_f;
    // 罚球区保底：球进己方罚球区（含门区）→ ≥0.9（原恒 1.0，语义保留为"接近全员回缩"）
    if (in_penalty_area(ctx, bx, by)) t = std::max(t, 0.9);
    return std::max(0.15, std::min(1.0, t));
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
