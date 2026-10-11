#include "simuro5/pass.hpp"
#include "simuro5/shoot.hpp"
#include "simuro5/defense.hpp"
#include "simuro5/field_info.hpp"
#include <cmath>
#include <algorithm>

#include "simuro5/geometry.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/shoot.hpp"
#include "simuro5/role_assignment.hpp"
#define TUNABLE_PREFIX "pass."
#include "simuro5/tunable.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

namespace {
// 调参常量：长度 cm，速度 cm/帧，时间 帧
TUNABLE(PASS_MAX_DIST, 69.4267);
TUNABLE(PASS_MIN_DIST, 8.0);
TUNABLE(BLOCK_THRESHOLD, 7.4329);
TUNABLE(OFFSET_BASE, 10.1);
TUNABLE(THREAT_RADIUS, 34.1917);
TUNABLE(FIELD_MARGIN, 6.0);
TUNABLE(RECEIVER_READY_SPEED, 2.0);
TUNABLE(PASS_BALL_SPEED, 6.0);
TUNABLE(RECEIVER_READY_TOLERANCE, 5.0);
TUNABLE(OPPONENT_REACH_SPEED, 2.0);
TUNABLE(OPPONENT_MOTION_HORIZON, 3.0);
TUNABLE(OPPONENT_FIRST_MARGIN, 3.0);

bool route_blocked(const WorldModel &wm, double sx, double sy, double tx, double ty) {
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = point_to_segment_dist(wm.opp[i].x, wm.opp[i].y, sx, sy, tx, ty);
        if (d < BLOCK_THRESHOLD) {
            return true;
        }
    }
    return false;
}

// 距离加权威胁度：贴脸≈1.0，THREAT_RADIUS 边缘≈0.0
double count_near_opponent(const WorldModel &wm, double x, double y)
{
    const double r2 = THREAT_RADIUS * THREAT_RADIUS;
    double threat = 0.0;
    for(int i=0;i<PLAYERS_PER_SIDE;i++)
    {
        double dx = wm.opp[i].x - x;
        double dy = wm.opp[i].y - y;
        double d2 = dx*dx + dy*dy;
        if(d2 < r2)
        {
            threat += 1.0 - d2 / r2;
        }
    }
    return threat;
}

int count_front_opponent(const WorldModel &wm, double x, double y, double ad) {
    int cnt = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double dx = wm.opp[i].x - x, dy = wm.opp[i].y - y;
        if (dx*dx + dy*dy < THREAT_RADIUS * THREAT_RADIUS) {
            if (ad * (wm.opp[i].x - x) > 0.0) cnt++;
        }
    }
    return cnt;
}

// 接应点夹回场内（含避开对方罚球区）
void clamp_receive_point(const TeamContext &ctx, double &x, double &y) {
    x = clamp(x, FIELD_MARGIN, TeamContext::FIELD_LENGTH - FIELD_MARGIN);
    y = clamp(y, FIELD_MARGIN, TeamContext::FIELD_WIDTH - FIELD_MARGIN);
    if (in_opp_penalty_area(ctx, x, y)) {
        double gx = ctx.opp_goal_x();
        x = (ctx.attack_dir() > 0) ? gx - 80.0 - FIELD_MARGIN : gx + 80.0 + FIELD_MARGIN;
    }
}

// 传球评分权重：长度 cm，速度 cm/帧
constexpr double SHOOT_BONUS      = 60.0;
constexpr double ARRIVE_PENALTY   = 40.0;
constexpr double SPEED_THREAT_W   = 10.0;
constexpr double OUR_ARRIVE_SPEED = 2.0;
constexpr double OPP_MIN_SPEED    = 1.0;
constexpr double ARRIVE_MARGIN    = 1.0;
constexpr double SPEED_BASE       = 2.0;
constexpr double ORDINARY_MOVE_COST_W = 0.25;
constexpr double ORDINARY_OWNERSHIP_W = 0.35;
constexpr double ORDINARY_LOCAL_STEP = 15.0;
constexpr double ORDINARY_ANCHOR_DEVIATION_W = 0.5;

// 假球位问 plan_shoot：只看球位/门将，即可判接应点有无射门开口
bool receive_point_can_shoot(const WorldModel &wm, double tx, double ty) {
    WorldModel wm2 = wm;
    wm2.ball.x = tx; wm2.ball.y = ty;
    return plan_shoot(wm2, 0).viable;
}

// 对手最快到达接应点的帧数；静止对手按 OPP_MIN_SPEED 兜底
double opp_arrive_time(const WorldModel &wm, double tx, double ty) {
    double t_min = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = dist(wm.opp[i].x, wm.opp[i].y, tx, ty);
        double spd = std::max(std::hypot(wm.opp_vx[i], wm.opp_vy[i]), OPP_MIN_SPEED);
        t_min = std::min(t_min, d / spd);
    }
    return t_min;
}

double speed_threat(const WorldModel &wm, double x, double y) {
    const double r2 = THREAT_RADIUS * THREAT_RADIUS;
    double threat = 0.0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double dx = wm.opp[i].x - x, dy = wm.opp[i].y - y;
        double d2 = dx * dx + dy * dy;
        if (d2 < r2 && d2 > 1e-6) {
            double d = std::sqrt(d2);
            double approach = -(wm.opp_vx[i] * dx + wm.opp_vy[i] * dy) / d;
            if (approach > 0.0) {
                threat += (1.0 - d2 / r2) * (approach / SPEED_BASE);
            }
        }
    }
    return threat;
}

// 普通传球排序项：低行程成本、且己方相对最近对手更容易占点时得分更低。
double ordinary_receiver_score_delta(const WorldModel &wm, int receiver_id,
                                     double target_x, double target_y) {
    if (receiver_id < 0 || receiver_id >= PLAYERS_PER_SIDE ||
        !std::isfinite(target_x) || !std::isfinite(target_y) ||
        !std::isfinite(wm.home[receiver_id].x) || !std::isfinite(wm.home[receiver_id].y)) return 0.0;
    const RobotState &receiver = wm.home[receiver_id];
    const double move_cost = dist(receiver.x, receiver.y, target_x, target_y);
    double nearest_opp_dist = 1e9;
    bool has_opponent = false;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (!std::isfinite(wm.opp[i].x) || !std::isfinite(wm.opp[i].y)) continue;
        const double opponent_dist = dist(wm.opp[i].x, wm.opp[i].y, target_x, target_y);
        if (!has_opponent || opponent_dist < nearest_opp_dist) {
            nearest_opp_dist = opponent_dist;
            has_opponent = true;
        }
    }
    if (!has_opponent) nearest_opp_dist = move_cost;
    const double ownership_margin = nearest_opp_dist - move_cost;
    return ORDINARY_MOVE_COST_W * move_cost - ORDINARY_OWNERSHIP_W * ownership_margin;
}

// Ordinary 候选沿用 Admission 已有的目标合法区、接球点净空和球到目标线路规则。
bool ordinary_pass_target_safe(const WorldModel &wm, double target_x, double target_y) {
    if (!std::isfinite(target_x) || !std::isfinite(target_y) ||
        target_x < 0.0 || target_x > TeamContext::FIELD_LENGTH ||
        target_y < 0.0 || target_y > TeamContext::FIELD_WIDTH ||
        in_opp_goal_area(wm.ctx, target_x, target_y) ||
        in_goal_area(wm.ctx, target_x, target_y) || in_no_push_zone(target_x, target_y)) return false;

    CircleObstacle obstacles[PLAYERS_PER_SIDE];
    int obstacle_count = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (!std::isfinite(wm.opp[i].x) || !std::isfinite(wm.opp[i].y)) continue;
        if (dist(target_x, target_y, wm.opp[i].x, wm.opp[i].y) < 20.0) return false;
        obstacles[obstacle_count++] = {wm.opp[i].x, wm.opp[i].y, 8.0};
    }
    if (wm.ball.valid) {
        if (!std::isfinite(wm.ball.x) || !std::isfinite(wm.ball.y) ||
            !segment_clear_of_circles(wm.ball.x, wm.ball.y, target_x, target_y,
                                      obstacles, obstacle_count)) return false;
    }
    return true;
}

// Ordinary 创建前候选筛选与活动任务复查共用同一套 ETA 证据和阈值。
bool opponent_arrives_first_for_target(const WorldModel &wm, int receiver_id,
                                      double target_x, double target_y) {
    if (receiver_id < 0 || receiver_id >= PLAYERS_PER_SIDE ||
        !std::isfinite(target_x) || !std::isfinite(target_y) ||
        !std::isfinite(RECEIVER_READY_SPEED) || !std::isfinite(OPPONENT_REACH_SPEED) ||
        !std::isfinite(OPPONENT_MOTION_HORIZON) || !std::isfinite(OPPONENT_FIRST_MARGIN) ||
        RECEIVER_READY_SPEED <= 1e-6 || OPPONENT_REACH_SPEED <= 1e-6 ||
        OPPONENT_MOTION_HORIZON < 0.0 || OPPONENT_FIRST_MARGIN < 0.0) return false;

    const RobotState &receiver = wm.home[receiver_id];
    if (!std::isfinite(receiver.x) || !std::isfinite(receiver.y)) return false;
    const double receiver_time = dist(receiver.x, receiver.y, target_x, target_y) / RECEIVER_READY_SPEED;
    double opponent_time = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        const double dx = target_x - wm.opp[i].x, dy = target_y - wm.opp[i].y;
        const double distance = std::hypot(dx, dy);
        if (!std::isfinite(distance)) continue;

        double approach = 0.0;
        if (wm.opp_vel_ready && distance > 1e-6 &&
            std::isfinite(wm.opp_vx[i]) && std::isfinite(wm.opp_vy[i])) {
            approach = (wm.opp_vx[i] * dx + wm.opp_vy[i] * dy) / distance;
            if (!std::isfinite(approach)) approach = 0.0;
        }
        double eta = 0.0;
        if (approach > 1e-6 && distance / approach <= OPPONENT_MOTION_HORIZON) {
            eta = distance / approach;
        } else {
            const double remaining = std::max(0.0, distance - approach * OPPONENT_MOTION_HORIZON);
            eta = OPPONENT_MOTION_HORIZON + remaining / OPPONENT_REACH_SPEED;
        }
        opponent_time = std::min(opponent_time, eta);
    }
    return opponent_time + OPPONENT_FIRST_MARGIN < receiver_time;
}

}  // anonymous namespace

bool pass_receiver_ready(const WorldModel &wm) {
    const CoopPassTask &task = wm.coop_pass_task;
    if (!task.active || task.receiver_id < 0 || task.receiver_id >= PLAYERS_PER_SIDE ||
        !std::isfinite(task.rx) || !std::isfinite(task.ry) ||
        RECEIVER_READY_SPEED <= 1e-6 || PASS_BALL_SPEED <= 1e-6) return false;

    const RobotState &receiver = wm.home[task.receiver_id];
    const double t_receiver = dist(receiver.x, receiver.y, task.rx, task.ry) / RECEIVER_READY_SPEED;
    const double t_ball = dist(wm.ball.x, wm.ball.y, task.rx, task.ry) / PASS_BALL_SPEED;
    return t_receiver <= t_ball + RECEIVER_READY_TOLERANCE;
}

bool pass_opponent_arrives_first(const WorldModel &wm) {
    const CoopPassTask &task = wm.coop_pass_task;
    if (!task.active || task.phase != CoopPassPhase::Preparing) return false;
    return opponent_arrives_first_for_target(wm, task.receiver_id, task.rx, task.ry);
}

static PassPlan plan_pass_impl(const WorldModel &wm, int passer_id, int receiver_filter,
                               double max_receiver_move = -1.0) {
    PassPlan plan{};
    if (passer_id < 1 || passer_id >= PLAYERS_PER_SIDE) return plan;
    const TeamContext &ctx = wm.ctx;

    double px = wm.home[passer_id].x;
    double py = wm.home[passer_id].y;
    double ad = ctx.attack_dir();

    int best = -1;
    double best_score = 1e9;
    double best_move = 1e9;
    double best_tx = 0, best_ty = 0;

    const double local_offsets[5][2] = {
        {0.0, 0.0}, {ORDINARY_LOCAL_STEP, 0.0}, {-ORDINARY_LOCAL_STEP, 0.0},
        {0.0, ORDINARY_LOCAL_STEP}, {0.0, -ORDINARY_LOCAL_STEP}
    };

    for (int id = 1; id < PLAYERS_PER_SIDE; ++id) {
        if (id == passer_id) continue;
        if (receiver_filter >= 1 && id != receiver_filter) continue;

        // 基准点：优先角色站位锚点，否则本体坐标
        double base_x = wm.home[id].x, base_y = wm.home[id].y;
        switch (wm.role[id]) {
            case ROLE_ASSIST:   base_x = wm.assist_x;  base_y = wm.assist_y;  break;
            case ROLE_MIDFIELD: base_x = wm.mid_x;     base_y = wm.mid_y;     break;
            case ROLE_PASSIVE:  base_x = wm.passive_x; base_y = wm.passive_y; break;
            default: break;
        }

        double center_x = base_x + ad * OFFSET_BASE;
        double center_y = base_y;
        clamp_receive_point(ctx, center_x, center_y);
        if (!std::isfinite(center_x) || !std::isfinite(center_y)) continue;

        double seen_x[5] = {};
        double seen_y[5] = {};
        int seen_count = 0;
        for (int p = 0; p < 5; ++p) {
            double tx = center_x + ad * local_offsets[p][0];
            double ty = center_y + local_offsets[p][1];
            clamp_receive_point(ctx, tx, ty);
            if (!std::isfinite(tx) || !std::isfinite(ty) ||
                dist(center_x, center_y, tx, ty) > ORDINARY_LOCAL_STEP + 1e-6) continue;

            bool duplicate = false;
            for (int d = 0; d < seen_count; ++d)
                duplicate = duplicate || (std::fabs(tx - seen_x[d]) <= 1e-6 &&
                                          std::fabs(ty - seen_y[d]) <= 1e-6);
            if (duplicate) continue;
            seen_x[seen_count] = tx; seen_y[seen_count] = ty; ++seen_count;

            const double pass_dist = dist(px, py, tx, ty);
            if (!std::isfinite(pass_dist) || pass_dist <= PASS_MIN_DIST || pass_dist >= PASS_MAX_DIST) continue;
            if (route_blocked(wm, px, py, tx, ty)) continue;
            if (!ordinary_pass_target_safe(wm, tx, ty)) continue;
            if (opponent_arrives_first_for_target(wm, id, tx, ty)) continue;
            const double receiver_move = dist(wm.home[id].x, wm.home[id].y, tx, ty);
            if (max_receiver_move >= 0.0 &&
                (receiver_move > max_receiver_move ||
                 route_blocked(wm, wm.home[id].x, wm.home[id].y, tx, ty))) continue;

            double threat = count_near_opponent(wm, tx, ty);
            int front_threat = count_front_opponent(wm, tx, ty, ad);

            bool can_shoot = receive_point_can_shoot(wm, tx, ty);
            double t_opp = opp_arrive_time(wm, tx, ty);
            double t_our = dist(wm.home[id].x, wm.home[id].y, tx, ty) / OUR_ARRIVE_SPEED;
            bool opp_first = t_opp < t_our * ARRIVE_MARGIN;
            double spd_threat = speed_threat(wm, tx, ty);

            double goal_dist = std::fabs(tx - ctx.opp_goal_x());
            double score = goal_dist + threat * 20.0 + front_threat * 12.0 + pass_dist * 0.5;
            if (can_shoot) score -= SHOOT_BONUS;
            if (opp_first) score += ARRIVE_PENALTY;
            score += spd_threat * SPEED_THREAT_W;
            score += ordinary_receiver_score_delta(wm, id, tx, ty);
            score += ORDINARY_ANCHOR_DEVIATION_W * dist(center_x, center_y, tx, ty);
            if ((max_receiver_move < 0.0 && score < best_score) ||
                (max_receiver_move >= 0.0 &&
                 (receiver_move < best_move - 1e-6 ||
                  (std::fabs(receiver_move - best_move) <= 1e-6 && score < best_score)))) {
                best_score = score;
                best_move = receiver_move;
                best = id;
                best_tx = tx;
                best_ty = ty;
            }
        }
    }

    if (best < 0) {
        return plan;
    }

    plan.viable = true;
    plan.receiver_id = best;
    plan.target_x = best_tx;
    plan.target_y = best_ty;
    return plan;
}

PassPlan plan_pass(const WorldModel &wm, int passer_id) {
    return plan_pass_impl(wm, passer_id, -1);
}

// 临时职责必须与当前首段任务绑定；第二段重新创建后不能沿用旧目标。
static bool third_man_task_matches(const WorldModel &wm, const ThirdManSupport &support) {
    const CoopPassTask &task = wm.coop_pass_task;
    return support.active && task.active && task.phase != CoopPassPhase::Received &&
           wm.runtime_phase == RuntimePhase::Running && wm.possession != Possession::Opponent &&
           task.passer_id >= 1 && task.passer_id < PLAYERS_PER_SIDE &&
           task.receiver_id >= 1 && task.receiver_id < PLAYERS_PER_SIDE &&
           task.passer_id != task.receiver_id && wm.active_id == task.passer_id &&
           wm.role[task.passer_id] == ROLE_ACTIVE &&
           support.first_passer_id == task.passer_id &&
           support.first_receiver_id == task.receiver_id &&
           support.first_rx == task.rx && support.first_ry == task.ry &&
           support.task_created == wm.coop_stats.created;
}

static bool third_man_runner_eligible(const WorldModel &wm, int id) {
    const CoopPassTask &task = wm.coop_pass_task;
    if (id < 1 || id >= PLAYERS_PER_SIDE || id == task.passer_id ||
        id == task.receiver_id || id == wm.active_id ||
        id == wm.presser_id || id == wm.sweeper_id || id == wm.stealer_id ||
        wm.ga_cooldown[id] > 0 || wm.ga_overstay[id] >= 15 ||
        in_goal_area(wm.ctx, wm.home[id].x, wm.home[id].y) ||
        in_opp_goal_area(wm.ctx, wm.home[id].x, wm.home[id].y) ||
        (wm.mark_assign_valid && wm.mark_assign[id] >= 0)) return false;
    const int role = wm.role[id];
    if (role == ROLE_ASSIST || role == ROLE_MIDFIELD) return true;
    return role == ROLE_PASSIVE && wm.team_state == TS_ATTACK &&
           wm.possession == Possession::Ours;
}

// 球飞行短暂成为 Loose 时保留计划；如正常角色已需回防，本帧交还控制权。
static bool third_man_defense_priority(const WorldModel &wm) {
    return (wm.threat_level > 0.3 || wm.no_possession_frames >= 2) &&
           wm.counter_attack_frames <= 0;
}

static bool third_man_target_safe(const WorldModel &projected, int passer_id, int runner_id,
                                  double tx, double ty) {
    const double d = dist(projected.home[passer_id].x, projected.home[passer_id].y, tx, ty);
    return std::isfinite(d) && d > PASS_MIN_DIST && d < PASS_MAX_DIST &&
           !route_blocked(projected, projected.ball.x, projected.ball.y, tx, ty) &&
           !route_blocked(projected, projected.home[runner_id].x,
                          projected.home[runner_id].y, tx, ty) &&
           ordinary_pass_target_safe(projected, tx, ty) &&
           !opponent_arrives_first_for_target(projected, runner_id, tx, ty);
}

bool third_man_support_ready(const WorldModel &wm, int runner_id) {
    const ThirdManSupport &support = wm.third_man_support;
    return third_man_task_matches(wm, support) && support.runner_id == runner_id &&
           third_man_runner_eligible(wm, runner_id) && !third_man_defense_priority(wm);
}

void update_third_man_support(WorldModel &wm) {
    ThirdManSupport &support = wm.third_man_support;
    const CoopPassTask &task = wm.coop_pass_task;
    if (!task.active || task.phase == CoopPassPhase::Received ||
        wm.runtime_phase != RuntimePhase::Running || wm.possession == Possession::Opponent ||
        task.frames_left <= 0 || task.passer_id < 1 || task.passer_id >= PLAYERS_PER_SIDE ||
        task.receiver_id < 1 || task.receiver_id >= PLAYERS_PER_SIDE ||
        task.passer_id == task.receiver_id || wm.active_id != task.passer_id ||
        wm.role[task.passer_id] != ROLE_ACTIVE) {
        support = {};
        return;
    }
    if (support.active && !third_man_task_matches(wm, support)) support = {};

    // Preparing 还需等 B 就位；Receiving 只剩球到锁点的飞行时间。
    const double ball_eta = dist(wm.ball.x, wm.ball.y, task.rx, task.ry) / PASS_BALL_SPEED;
    const double receiver_eta = dist(wm.home[task.receiver_id].x, wm.home[task.receiver_id].y,
                                     task.rx, task.ry) / RECEIVER_READY_SPEED;
    const double estimated = ball_eta + RECEIVER_READY_TOLERANCE +
                             (task.phase == CoopPassPhase::Preparing ? receiver_eta : 0.0);
    const double available = std::min(static_cast<double>(task.frames_left), estimated);
    if (!std::isfinite(available) || available <= 0.0 ||
        !std::isfinite(task.rx) || !std::isfinite(task.ry)) {
        support = {};
        return;
    }

    WorldModel projected = wm;
    projected.ball.x = task.rx; projected.ball.y = task.ry;
    projected.ball.vx = projected.ball.vy = 0.0;
    projected.home[task.receiver_id].x = task.rx;
    projected.home[task.receiver_id].y = task.ry;
    if (support.active && third_man_runner_eligible(wm, support.runner_id) &&
        third_man_target_safe(projected, task.receiver_id, support.runner_id,
                              support.target_x, support.target_y) &&
        dist(wm.home[support.runner_id].x, wm.home[support.runner_id].y,
             support.target_x, support.target_y) <= available * RECEIVER_READY_SPEED) {
        support.available_frames = available;
        return;
    }
    support = {};
    if (third_man_defense_priority(wm)) return;

    double best_move = 1e9, best_depth = -1e9;
    for (int id = 1; id < PLAYERS_PER_SIDE; ++id) {
        if (!third_man_runner_eligible(wm, id)) continue;
        const PassPlan point = plan_pass_impl(projected, task.receiver_id, id,
                                             available * RECEIVER_READY_SPEED);
        if (!point.viable ||
            !third_man_target_safe(projected, task.receiver_id, id,
                                   point.target_x, point.target_y)) continue;
        const double move = dist(wm.home[id].x, wm.home[id].y, point.target_x, point.target_y);
        if (!std::isfinite(move) || move > available * RECEIVER_READY_SPEED) continue;
        const double depth = wm.ctx.dist_our_goal(point.target_x);
        if (move < best_move - 1e-6 ||
            (std::fabs(move - best_move) <= 1e-6 && depth > best_depth)) {
            best_move = move; best_depth = depth;
            support.active = true;
            support.runner_id = id;
            support.first_passer_id = task.passer_id;
            support.first_receiver_id = task.receiver_id;
            support.first_rx = task.rx; support.first_ry = task.ry;
            support.target_x = point.target_x; support.target_y = point.target_y;
            support.available_frames = available;
            support.task_created = wm.coop_stats.created;
        }
    }
}

CoopPass plan_coop_pass(const WorldModel &wm, int passer_id) {
    CoopPass best;
    if (passer_id < 1 || passer_id >= PLAYERS_PER_SIDE) return best;
    const TeamContext &ctx = wm.ctx;
    const double bx = wm.ball.x, by = wm.ball.y;
    const double ogx = ctx.opp_goal_x();
    const double d_goal = dist(bx, by, ogx, 90.0);

    for (int i = 1; i < PLAYERS_PER_SIDE; ++i) {
        if (i == passer_id) continue;
        double rx = 0.0, ry = 0.0;
        switch (wm.role[i]) {
            case ROLE_ASSIST: rx = wm.assist_x; ry = wm.assist_y; break;
            case ROLE_MIDFIELD: rx = wm.mid_x; ry = wm.mid_y; break;
            case ROLE_PASSIVE: rx = wm.passive_x; ry = wm.passive_y; break;
            default: continue;
        }
        if (in_opp_goal_area(ctx, rx, ry)) continue;
        if (dist(rx, ry, ogx, 90.0) > d_goal - 5.0) continue;
        if (dist(bx, by, rx, ry) > 120.0) continue;

        CircleObstacle obs[PLAYERS_PER_SIDE];
        for (int j = 0; j < PLAYERS_PER_SIDE; ++j) {
            obs[j].x = wm.opp[j].x; obs[j].y = wm.opp[j].y; obs[j].r = 8.0;
        }
        if (!segment_clear_of_circles(bx, by, rx, ry, obs, PLAYERS_PER_SIDE)) continue;
        double opp_min = 1e9;
        for (int j = 0; j < PLAYERS_PER_SIDE; ++j)
            opp_min = std::min(opp_min, dist(rx, ry, wm.opp[j].x, wm.opp[j].y));
        if (opp_min < 20.0) continue;

        WorldModel tmp = wm;
        tmp.ball.x = rx; tmp.ball.y = ry; tmp.ball.vx = 0.0; tmp.ball.vy = 0.0;
        tmp.in_penalty_exec = false;
        ShootPlan ps = plan_shoot(tmp, i);
        double q = ps.viable ? ps.quality : 0.0;
        if (!ps.viable) {
            const double dg = dist(rx, ry, ogx, 90.0);
            q = 0.25 * clamp((160.0 - dg) / 160.0, 0.0, 1.0);
        }
        if (q <= best.score) continue;
        best.viable = true; best.receiver_id = i; best.rx = rx; best.ry = ry; best.score = q;
        double dx = rx - bx, dy = ry - by;
        const double L = std::hypot(dx, dy);
        if (L > 1e-6) { best.dir_x = dx / L; best.dir_y = dy / L; }
        best.aim_rot = angle_to(0.0, 0.0, best.dir_x, best.dir_y);
    }
    return best;
}

} // namespace simuro5
