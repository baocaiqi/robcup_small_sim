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
    // 本帧最后被命令的目标点（由 motion::position 写入）。用途：门将要"用新驱动律重发同一个目标"
    //   —— 决策用旧口径算、驱动用新口径发（见 run_goalie）；顺带也是排障时最有用的一个字段。
    double tgt_x = 0, tgt_y = 0;
    int    tgt_mode = 0;        // motion::TargetMode（0=TM_STOP 1=TM_PASS）
};

struct BallState {
    double x = 0, y = 0;
    double vx = 0, vy = 0;      // 速度 cm/帧（current-last 差分）
    bool valid = false;
};

enum class CoopPassPhase { Preparing, Receiving, Received };
enum class PassTaskKind { Coop, Ordinary };
// 普通传球人自己的准备进度；不改变 CoopPass 的生命周期阶段。
enum class OrdinaryPassPrepPhase { GoPrepSide, GoPrepPoint, Align, Prepared };

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
    // 普通传球首次合法 Push 时固定穿球目标；出球或任务结束后清除。
    bool push_target_locked = false;
    double push_start_ball_x = 0.0, push_start_ball_y = 0.0;
    double push_start_dir_x = 0.0, push_start_dir_y = 0.0;
    double locked_push_target_x = 0.0, locked_push_target_y = 0.0;
    int receive_frames = 0;
    PassTaskKind kind = PassTaskKind::Coop;
    OrdinaryPassPrepPhase ordinary_prep_phase = OrdinaryPassPrepPhase::GoPrepPoint;
    int ordinary_prep_side = 0;
    void clear_push_target_lock() {
        push_target_locked = false;
        push_start_ball_x = push_start_ball_y = 0.0;
        push_start_dir_x = push_start_dir_y = 0.0;
        locked_push_target_x = locked_push_target_y = 0.0;
    }
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
    // 门球发球的**锚点**（2026-10-13）："本次门球第一次进 restart_kick"时的球位。
    //   为什么必须锚住：准备点是 `px = 球位 - 推球方向×8cm`，原来每帧按**当前**球位重算，
    //   而门将一推球球就在动 ⇒ 准备点跟着球走 = **追自己的尾巴**，`at_ready`（停在 3cm 内）
    //   永远不成立 ⇒ 承诺段那脚猛推（TM_PASS 150~200cm/s）从不触发，只剩 19cm/s 的身体慢推。
    //   真机取证（20261013193005 帧 3817~3862）：球从门球点 (205,77) 以 19cm/s 被推走 11cm，
    //   门将 y 在 70~85 之间来回摆，对手从 85cm 外赶到捅进 → 该场 4/5 丢球都是这个形状。
    //   `ok=false` = 还没锚（下一帧进规则时重新锚）；球被平台重新摆放/规则不再适用时清零。
    bool   goalie_serve_anchor_ok = false;
    double goalie_serve_ax = 0.0, goalie_serve_ay = 0.0;
    // 门将上一帧位置（run_goalie 自己记）：用来量门将**真实**移动量（平台上报的 vl/vr 只是上一帧命令），
    //   和认出"平台把门将瞬移回门口"（重新开球）。goalie_last_id=-1 表示还没有上一帧。
    int    goalie_last_id = -1;
    double goalie_last_x = 0.0, goalie_last_y = 0.0;
    double goalie_moved = 0.0;    // 本帧量到的门将上一帧位移（cm/帧）
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
