// ============================================================
// world_model.hpp — 世界模型包装
// 平台上帝视角数据的内部化封装
// 小型组是上帝视角，平台每周期直接把全场数据塞进 Environment，
// 这里只做「搬运 + 派生量计算」(球速/预测/球权)。
// ============================================================
#ifndef SIMURO5_WORLD_MODEL_HPP
#define SIMURO5_WORLD_MODEL_HPP

#include "simuro5/simuro_interface.hpp"
#include <cmath>
#include "simuro5/team.hpp"

namespace simuro5 {

// 团队攻防状态（Strategy 状态机写入，滞回防抖）
enum TeamState {
    TS_ATTACK = 0,    // 进攻态：我方稳定控球（连续 N 帧持球才进入）
    TS_DEFENSE = 1    // 防守态：对方控球 / 球权未定（连续 N 帧失球才进入）
};

struct RobotState {
    double x = 0, y = 0;        // 位置
    double rot = 0;             // 朝向(度)
    double vl = 0, vr = 0;      // 差速轮速(仅己方有效)
};

struct BallState {
    double x = 0, y = 0;
    double vx = 0, vy = 0;      // 速度(由 current-last 差分)
    bool valid = false;
};

struct WorldModel {
    TeamContext ctx;

    BallState ball;          // 当前球
    BallState ball_last;     // 上一帧球
    BallState ball_pred;     // 平台预测球(直接用)
    RobotState home[PLAYERS_PER_SIDE];
    RobotState opp[PLAYERS_PER_SIDE];
    RobotState opp_last[PLAYERS_PER_SIDE];   // 上一帧对方位置（差分求速用）
    double opp_vx[PLAYERS_PER_SIDE] = {0};   // 对方速度(cm/帧，current-last 差分，瞬移清零)
    double opp_vy[PLAYERS_PER_SIDE] = {0};
    bool opp_vel_ready = false;              // 首帧只记位置不求速（防初始(0,0)跳变）

    Bounds field;            // 场地边界(平台给)
    Bounds goal;             // 球门边界(平台给)
    int game_state = 0;      // PlayMode
    long whos_ball = 0;      // 球权(0=未知/1=我们? 以平台为准)
    double threat_level = 0.0; // 威胁等级 0~1（状态机输出：状态+球位稳定计算）
    bool we_have_ball = false; // 球权是否在我方（简版判断）

    // —— 攻防状态机（strategy.cpp 每帧写入）——
    TeamState team_state = TS_DEFENSE; // 当前攻防状态
    int possession_frames = 0;         // 连续持球帧数（滞回计数）
    int no_possession_frames = 0;      // 连续失球帧数
    bool state_transition = false;     // 本帧是否刚发生攻防切换（事件标志，供即时响应）

    // 角色分配结果（由 RoleAssignment 填写）
    int role[PLAYERS_PER_SIDE] = {0, 0, 0, 0, 0};   // 见 roles.hpp 的 Roles 枚举
    // 人盯人目标（上一帧选中的对方球员下标，-1=无；供滞回防抖用）
    int mark_target = -1;

    // 站位参考点（由 SituationModule 填写）
    double passive_x = 0, passive_y = 90;
    double assist_x = 0, assist_y = 90;
    double mid_x = 110, mid_y = 90;

    // D1 纪律：ACTIVE 在对方门区连续停留帧数（roles.cpp run_active 维护，
    // 规则 7.10.1 单人停留 >20 周期判点球，≥18 帧强制撤离）
    int active_goal_area_frames[PLAYERS_PER_SIDE] = {0, 0, 0, 0, 0};

    // 每周期从平台环境刷新
    void update(const Environment *env, const TeamContext &ctx_);

    // 距离辅助
    double ball_opp_goal_dist() const { return std::fabs(ball.x - ctx.opp_goal_x()); }
    double ball_our_goal_dist() const { return std::fabs(ball.x - ctx.our_goal_x()); }
};

}  // namespace simuro5
#endif
