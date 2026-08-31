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
    int game_state_last = 0; // 上一帧 PlayMode（点球/定位球执行期识别：PenaltyKick→PlayOn 过渡）
    long whos_ball = 0;      // 球权(0=未知/1=我们? 以平台为准)
    // 我方主罚点球执行中（strategy.cpp 每帧维护）：球静止在罚球点、我方必须去踢。
    // roles 用它区分"对方门球"（不抢）vs"我方点球"（必须射门）——两者都是球静止在对方门区。
    bool in_penalty_exec = false;
    // 全角色对方门区停留时限（strategy.cpp 调度后兜底，docs/13 方案 C 扩展）：
    //   平台判罚看实际位置，clamp 站位点挡不住 ASSIST/MID/PASSIVE 追球/振荡进区；
    //   连续停留 >15 帧强制撤出 + 冷却（防撤出-回区拉锯）。
    int ga_overstay[PLAYERS_PER_SIDE] = {0};
    int ga_cooldown[PLAYERS_PER_SIDE] = {0};
    // 死球等待计时（run_active）：球静止在对方门区（对方门球/卡死）的连续帧数。
    //   超时（>100 帧）→ 判定"对方不开球/球卡死"，ACTIVE 主动去推球（真实 8/31 镜像
    //   内战 0:0 根因：球卡对方门区 135 秒，进攻方死球等待永不超时、门将清球不穿过）。
    int dead_ball_frames = 0;
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

    // ACTIVE 在对方门区停留计数（docs/13 方案 C：防"门区单人停留>20 周期"罚点球）
    // roles.cpp run_active 每帧更新；超限强制撤出（射门/传球/带球出区）。
    //   active_ga_frames ：纯停留帧数（人在门区 且 球不在门区或不在脚下>25cm）——主判据
    //   active_ga_total  ：门区总时长（含带球推射）——兜底：球被门将挡回反复推
    //                      也是真实平台罚点球场景（8/29 实测 21~30 帧被罚），不能无限续
    //   ga_retreat_fires ：超限撤出触发次数（sim_bench 诊断用）
    int active_ga_frames = 0;
    int active_ga_total = 0;
    int ga_retreat_fires = 0;

    // 角区救球触发次数（sim_bench 统计用：验证"FreeBall 13 次/场"角区卡球是否被救）
    int corner_rescue_events = 0;
    // 球在角区且静止的连续帧数（>30 帧才算"真卡住"，防路过/刚弹到角的球误触发救球）
    int corner_ball_frames = 0;

    // 清道夫（远侧覆盖）：球在防守三区拉边时，指定一个区域防守者钉中路封远门柱/横传。
    //   strategy.cpp 每帧写入；-1=无清道夫（正常防守站位）。
    int sweeper_id = -1;
    double sweeper_x = 0, sweeper_y = 90;   // 清道夫站位点（罚球区前缘外侧、中路）

    // 站位参考点（由 SituationModule 填写）
    double passive_x = 0, passive_y = 90;
    double assist_x = 0, assist_y = 90;
    double mid_x = 110, mid_y = 90;

    // 每周期从平台环境刷新
    void update(const Environment *env, const TeamContext &ctx_);

    // 距离辅助
    double ball_opp_goal_dist() const { return std::fabs(ball.x - ctx.opp_goal_x()); }
    double ball_our_goal_dist() const { return std::fabs(ball.x - ctx.our_goal_x()); }
};

}  // namespace simuro5
#endif
