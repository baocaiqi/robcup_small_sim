// world_model.hpp — 平台 Environment 的内部封装（搬运 + 派生量）
#ifndef SIMURO5_WORLD_MODEL_HPP
#define SIMURO5_WORLD_MODEL_HPP

#include "simuro5/simuro_interface.hpp"
#include <cmath>
#include "simuro5/team.hpp"

namespace simuro5 {

enum TeamState {
    TS_ATTACK = 0,
    TS_DEFENSE = 1
};

enum class RuntimePhase {
    RestartSetup,
    Running
};

// 运动战球权只表达场上空间关系；定位球归属仍由 raw game_state 单独识别。
enum class Possession { Unknown, Ours, Opponent, Loose };

struct RobotState {
    double x = 0, y = 0;
    double rot = 0;
    double vl = 0, vr = 0;      // 差速轮速（仅己方有效）
};

struct BallState {
    double x = 0, y = 0;
    double vx = 0, vy = 0;      // 速度 cm/帧（current-last 差分）
    bool valid = false;
};

enum class CoopPassPhase { Preparing, Receiving, Received };
enum class PassTaskKind { Coop, Ordinary };

// 结算原因，只用于记账
enum class CoopOutcome {
    Success, PrepareTimeout, ReceiveTimeout, PhaseInterrupted, InvalidBall, Penalty,
    HighThreat, Corner, Intercepted, EmergencyDefense, IncomingShot,
    InvalidTarget, GoalDiscipline, ReceiverMarked, LaneBlocked, PushForbidden,
    DeadBall, DegenerateTarget, PrepPoint, MatchEnd, LooseBall, TeammateTakeover,
    CarryPoint, OpponentFirst, Count
};
// 结算原因名（统计用）
const char *coop_outcome_name(CoopOutcome result);
struct CoopPassStats {
    unsigned long created = 0, released = 0, received = 0, control_entered = 0;
    unsigned long outcomes[(int)CoopOutcome::Count] = {};
    unsigned long control_exits[(int)CoopOutcome::Count] = {};
};

struct CoopPassTask {
    bool active = false;
    int passer_id = -1, receiver_id = -1;
    double rx = 0.0, ry = 0.0;
    int frames_left = 0;
    CoopPassPhase phase = CoopPassPhase::Preparing;
    // 推球指令只开观察窗口，之后的球位/速度/人球分离才是出球证据
    bool observing_push = false;
    double push_ball_x = 0.0, push_ball_y = 0.0;
    double push_dir_x = 0.0, push_dir_y = 0.0;
    int receive_frames = 0;
    PassTaskKind kind = PassTaskKind::Coop;
};

struct CoopBallControl {
    bool active = false;
    int receiver_id = -1;
    int loose_frames = 0;
};

struct WorldModel {
    CoopPassTask coop_pass_task;
    CoopBallControl coop_ball_control;
    CoopPassStats coop_stats;
    // 可为空；仿真器可装只读记录器，比赛 DLL 不写文件
    void (*coop_observer)(const WorldModel &, const char *, CoopOutcome) = nullptr;
    // 传球任务与临时控球的计数/结算
    void coop_created();
    void coop_released();
    void coop_finish(CoopOutcome result);
    void coop_control_entered();
    void coop_control_end(CoopOutcome reason);
    TeamContext ctx;

    BallState ball;
    BallState ball_last;
    BallState ball_pred;
    RobotState home[PLAYERS_PER_SIDE];
    RobotState opp[PLAYERS_PER_SIDE];
    RobotState opp_last[PLAYERS_PER_SIDE];
    double opp_vx[PLAYERS_PER_SIDE] = {0};   // 对方速度 cm/帧（差分，复位跳变清零）
    double opp_vy[PLAYERS_PER_SIDE] = {0};
    bool opp_vel_ready = false;

    Bounds field;
    Bounds goal;
    int game_state = 0;      // raw PlayMode: referee events/restart type, not normal-action permission
    int game_state_last = 0;
    // RuntimePhase is the sole normal-play action gate; update() derives it from PlayMode and restart-ball motion.
    RuntimePhase runtime_phase = RuntimePhase::Running;
    bool restart_armed = false;
    double restart_x = 0.0, restart_y = 0.0;
    long whos_ball = 0;      // raw platform possession hint; retained for diagnostics, not our possession authority
    int whos_disagree = 0;
    // 我方点球执行中：roles 用它区分"对方门球(不抢)"与"我方点球(必须射门)"
    bool in_penalty_exec = false;
    int ga_overstay[PLAYERS_PER_SIDE] = {0};
    int ga_cooldown[PLAYERS_PER_SIDE] = {0};
    int dead_ball_frames = 0;  // 球静止在对方门区的连续帧数（>100 判卡死）
    double threat_level = 0.0; // 0~1
    int threat_hold_frames = 0;
    int goalie_opp_hold = 0;
    int goalie_serve_phase = 0;   // 发球承诺：0=先精确到位并转正机头, 1=已承诺，一气推穿不回头
    bool we_have_ball = false;  // in-play possession inferred by Situation; separate from phase/restart ownership
    Possession possession = Possession::Unknown;  // Situation 的唯一球权结果；bool 字段仅为兼容投影

    TeamState team_state = TS_DEFENSE;
    int possession_frames = 0;
    int no_possession_frames = 0;
    bool state_transition = false;
    bool prev_we_have_ball = false;   // 反击窗口：断球瞬间置位，窗口内 assist/mid 立即前插
    int counter_attack_frames = 0;

    int role[PLAYERS_PER_SIDE] = {0, 0, 0, 0, 0};   // 见 Roles 枚举
    // 动态主攻：当前 ACTIVE 下标 + 换人滞回状态
    int active_id = 1;
    int active_cand = -1, active_cand_frames = 0, active_hold = 0;
    int mark_target = -1;

    // 带权匈牙利盯人：mark_assign[i]=机器人 i 本帧被指派的对手（-1=不盯，回退区域防守）
    int mark_assign[PLAYERS_PER_SIDE] = {-1, -1, -1, -1, -1};
    int mark_prev_assign[PLAYERS_PER_SIDE] = {-1, -1, -1, -1, -1};
    int mark_commit[PLAYERS_PER_SIDE] = {0, 0, 0, 0, 0};
    double mark_cost_ema[PLAYERS_PER_SIDE][PLAYERS_PER_SIDE] = {{0.0}};
    bool mark_ema_ready = false;
    bool mark_assign_valid = false;   // false 时角色函数回退旧逻辑
    int mark_switch_events = 0;

    int stealer_id = -1;   // 抢断唯一竞标：只让 EV 最高的一台上抢（-1=无人）

    int active_ga_frames = 0;
    int active_ga_total = 0;
    int ga_retreat_fires = 0;

    int corner_rescue_events = 0;
    int corner_ball_frames = 0;

    int sweeper_id = -1;
    double sweeper_x = 0, sweeper_y = 90;
    int presser_id = -1;
    int route_wp_next[PLAYERS_PER_SIDE] = {0, 0, 0, 0, 0};
    int shoot_push_count = 0;   // 变角推射：已推球次数（cd 为计次冷却）
    int shoot_push_cd = 0;
    int shoot_push_last_side = 0;
    int shoot_align_frames = 0;
    bool   pen_aim_locked = false;   // 点球执行期锁定瞄准方向（rot 度 / 方向向量 / 瞄准点 y）
    double pen_aim_rot = 0.0;
    double pen_dir_x = 1.0, pen_dir_y = 0.0;
    double pen_aim_y = 90.0;

    double passive_x = 0, passive_y = 90;
    double assist_x = 0, assist_y = 90;
    double mid_x = 110, mid_y = 90;

    // 每周期从平台环境刷新
    void update(const Environment *env, const TeamContext &ctx_);
};

}  // namespace simuro5
#endif
