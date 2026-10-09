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
    if (!task.active || task.phase != CoopPassPhase::Preparing ||
        task.receiver_id < 0 || task.receiver_id >= PLAYERS_PER_SIDE ||
        !std::isfinite(task.rx) || !std::isfinite(task.ry) ||
        !std::isfinite(RECEIVER_READY_SPEED) || !std::isfinite(OPPONENT_REACH_SPEED) ||
        !std::isfinite(OPPONENT_MOTION_HORIZON) || !std::isfinite(OPPONENT_FIRST_MARGIN) ||
        RECEIVER_READY_SPEED <= 1e-6 || OPPONENT_REACH_SPEED <= 1e-6 ||
        OPPONENT_MOTION_HORIZON < 0.0 || OPPONENT_FIRST_MARGIN < 0.0) return false;

    const RobotState &receiver = wm.home[task.receiver_id];
    if (!std::isfinite(receiver.x) || !std::isfinite(receiver.y)) return false;
    const double receiver_time = dist(receiver.x, receiver.y, task.rx, task.ry) / RECEIVER_READY_SPEED;
    double opponent_time = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        const double dx = task.rx - wm.opp[i].x, dy = task.ry - wm.opp[i].y;
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

PassPlan plan_pass(const WorldModel &wm, int passer_id) {
    PassPlan plan{};
    const TeamContext &ctx = wm.ctx;

    double px = wm.home[passer_id].x;
    double py = wm.home[passer_id].y;
    double ad = ctx.attack_dir();

    int best = -1;
    double best_score = 1e9;
    double best_tx = 0, best_ty = 0;

    for (int id = 0; id < PLAYERS_PER_SIDE; ++id) {
        if (id == passer_id) continue;

        // 基准点：优先角色站位锚点，否则本体坐标
        double base_x = wm.home[id].x, base_y = wm.home[id].y;
        switch (wm.role[id]) {
            case ROLE_ASSIST:   base_x = wm.assist_x;  base_y = wm.assist_y;  break;
            case ROLE_MIDFIELD: base_x = wm.mid_x;     base_y = wm.mid_y;     break;
            case ROLE_PASSIVE:  base_x = wm.passive_x; base_y = wm.passive_y; break;
            default: break;
        }

        double tx = base_x + ad * OFFSET_BASE;
        double ty = base_y;
        clamp_receive_point(ctx, tx, ty);

        double pass_dist = dist(px, py, tx, ty);
        if (pass_dist <= PASS_MIN_DIST || pass_dist >= PASS_MAX_DIST) {
            continue;
        }

        if (route_blocked(wm, px, py, tx, ty)) {
            continue;
        }

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

        if (score < best_score) {
            best_score = score;
            best = id;
            best_tx = tx;
            best_ty = ty;
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

CoopPass plan_coop_pass(const WorldModel &wm, int passer_id) {
    CoopPass best;
    const TeamContext &ctx = wm.ctx;
    const double bx = wm.ball.x, by = wm.ball.y;
    const double ogx = ctx.opp_goal_x();
    const double d_goal = dist(bx, by, ogx, 90.0);

    const int cand[3] = {2, 3, 4};
    const double ax[3] = {wm.assist_x, wm.mid_x, wm.passive_x};
    const double ay[3] = {wm.assist_y, wm.mid_y, wm.passive_y};

    for (int k = 0; k < 3; ++k) {
        const int i = cand[k];
        if (i == passer_id) continue;
        const double rx = ax[k], ry = ay[k];
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
