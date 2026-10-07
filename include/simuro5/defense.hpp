// defense.hpp — 区域防守：断球点计算工具箱（原点左下角 cm；蓝队守 x=220 的右门）
#ifndef SIMURO5_DEFENSE_HPP
#define SIMURO5_DEFENSE_HPP

#include "simuro5/world_model.hpp"
#include "simuro5/geometry.hpp"
#include "simuro5/field_info.hpp"

namespace simuro5 {

// 球速大小 / 朝己方门心速度分量 / 朝某点(px,py)速度分量（cm/帧；后两者只取投影，背离为 0）
inline double ball_speed(double vx, double vy) {
    return std::hypot(vx, vy);
}

inline double ball_danger_speed(const WorldModel &wm) {
    double gx = wm.ctx.our_goal_x();
    double dx = gx - wm.ball.x, dy = 90.0 - wm.ball.y;
    double d = std::hypot(dx, dy);
    if (d < 1e-6) return 0.0;
    return std::max(0.0, (wm.ball.vx * dx + wm.ball.vy * dy) / d);
}

inline double ball_approach_speed(const WorldModel &wm, double px, double py) {
    double dx = px - wm.ball.x, dy = py - wm.ball.y;
    double d = std::hypot(dx, dy);
    if (d < 1e-6) return 0.0;
    return std::max(0.0, (wm.ball.vx * dx + wm.ball.vy * dy) / d);
}

// 球匀速直线到竖线 x=target_x 时的 y；false = |vx|≈0 或球背离（t<0）
inline bool predict_y_at_x(double x, double y, double vx, double vy,
                           double target_x, double &out_y) {
    if (std::fabs(vx) < 1e-9) return false;
    double t = (target_x - x) / vx;
    if (t < 0.0) return false;
    out_y = y + vy * t;
    return true;
}

// 同上但含一次 y 边墙反弹（实测非镜面：法向×0.66、切向×0.81；只算一次反射）
inline bool predict_y_at_x_reflect(double x, double y, double vx, double vy,
                                   double target_x, double &out_y) {
    if (std::fabs(vx) < 1e-9) return false;
    double t_total = (target_x - x) / vx;
    if (t_total < 0.0) return false;
    double y_end = y + vy * t_total;
    if (y_end >= 0.0 && y_end <= 180.0) {
        out_y = y_end;
        return true;
    }
    const double kRest = ball_wall_rest(), kFric = ball_wall_fric();
    double wall_y = (y_end < 0.0) ? 0.0 : 180.0;
    double t_wall = (wall_y - y) / vy;
    double x_wall = x + vx * t_wall;
    double vx2 = vx * kFric, vy2 = -vy * kRest;
    if (std::fabs(vx2) < 1e-9) return false;
    double t_rem = (target_x - x_wall) / vx2;
    if (t_rem < 0.0) return false;
    out_y = wall_y + vy2 * t_rem;
    return true;
}

// 对方射门是否在门框内且朝门（门宽 40 → y∈[70,110]）
inline bool shot_on_target(const WorldModel &wm) {
    double y_at_goal = 90.0;
    if (!predict_y_at_x(wm.ball.x, wm.ball.y, wm.ball.vx, wm.ball.vy,
                        wm.ctx.our_goal_x(), y_at_goal))
        return false;
    const double half = TeamContext::GOAL_WIDTH / 2.0;
    return y_at_goal >= 90.0 - half && y_at_goal <= 90.0 + half;
}

// 离球最近的对方球员下标(0~4，回填距离 cm) / 该距离(cm)
inline int nearest_opp_to_ball(const WorldModel &wm, double *dmin = nullptr) {
    int best = -1;
    double bd = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = dist(wm.ball.x, wm.ball.y, wm.opp[i].x, wm.opp[i].y);
        if (d < bd) { bd = d; best = i; }
    }
    if (dmin) *dmin = bd;
    return best;
}

inline double opp_clear_dist(const WorldModel &wm) {
    double dmin = 1e9;
    nearest_opp_to_ball(wm, &dmin);
    return dmin;
}

// 球门在球的哪一侧：+1 = 门在球 +x 侧（门将判断站门侧还是场侧）
inline double ball_goal_side(const TeamContext &ctx, double bx) {
    return (ctx.our_goal_x() > bx) ? 1.0 : -1.0;
}

// 门将封角度深度(cm)：球-门连线上球前 12cm，夹在 [guard_dist, 40]
inline double goalie_block_depth(const TeamContext &ctx, double bx, double guard_dist) {
    return std::min(40.0, std::max(guard_dist, ctx.dist_our_goal(bx) - 12.0));
}

// 门前抢反弹位：罚球区前缘、预测入球点 ±y_side（+30 上侧 / -30 下侧），
//   带边墙反射预测 + 按门将站位偏向 + 落点被对手占住则让开 20cm；最小朝门速度 8cm/帧
inline void rebound_point(const WorldModel &wm, double y_side, double &x, double &y) {
    const TeamContext &ctx = wm.ctx;
    double y_at_goal = 90.0;
    if (!predict_y_at_x_reflect(wm.ball.x, wm.ball.y, wm.ball.vx, wm.ball.vy,
                                ctx.our_goal_x(), y_at_goal))
        y_at_goal = wm.ball.y;
    double gk_y = 90.0, gk_best = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = std::fabs(wm.opp[i].x - ctx.our_goal_x());
        if (d < gk_best) { gk_best = d; gk_y = wm.opp[i].y; }
    }
    double miss = y_at_goal - gk_y;
    double bias = (std::fabs(miss) < 5.0) ? 0.0 : ((miss > 0.0) ? 1.0 : -1.0);
    x = ctx.our_goal_x() + ctx.attack_dir() * 80.0;
    y = clamp(y_at_goal + y_side + bias * 8.0, 72.5, 107.5);
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (dist(wm.opp[i].x, wm.opp[i].y, x, y) < 25.0) {
            y = clamp((wm.opp[i].y >= y) ? y - 20.0 : y + 20.0, 72.5, 107.5);
        }
    }
}

inline double rebound_min_danger() { return 8.0; }

// 对方出脚方向预测：仅「球静止 + 对手贴球 ≤25cm + 机头对球 ≤35°」才采信
inline bool opp_kick_direction(const WorldModel &wm, double &dx, double &dy, int &who) {
    who = -1;
    if (!wm.ball.valid) return false;
    if (std::hypot(wm.ball.vx, wm.ball.vy) > 1.0) return false;       // ① 球在动
    double dmin = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        const double d = dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y);
        if (d < dmin) { dmin = d; who = i; }
    }
    if (who < 0 || dmin > 25.0) return false;                         // ② 没人贴球
    const double to_ball = angle_to(wm.opp[who].x, wm.opp[who].y, wm.ball.x, wm.ball.y);
    if (std::fabs(angle_diff(wm.opp[who].rot, to_ball)) > 35.0) return false;
    const double ra = wm.opp[who].rot * SIMURO5_PI / 180.0;
    dx = std::cos(ra); dy = std::sin(ra);
    return true;
}

inline constexpr bool kOppKickPredict = true;

// 由预测方向推球从门线哪个 y 进（开关 kOppKickPredict 单常量可回退）；仅门框内且门前 120cm 内
inline bool opp_kick_target_y(const WorldModel &wm, double &y_at_goal) {
    if (!kOppKickPredict) return false;
    double dx = 0.0, dy = 0.0; int who = -1;
    if (wm.ctx.dist_our_goal(wm.ball.x) > 120.0) return false;
    if (!opp_kick_direction(wm, dx, dy, who)) return false;
    if (!predict_y_at_x(wm.ball.x, wm.ball.y, dx, dy, wm.ctx.our_goal_x(), y_at_goal))
        return false;
    return y_at_goal >= goal_y_low() && y_at_goal <= goal_y_high();
}

// 防守计划：断球点 cm / 是否朝己方门逼近 / 球速 cm/帧；face_incoming=true 时 aim_rot(-v 方向) 有效
struct DefensePlan {
    double target_x = 0, target_y = 90;
    bool   approaching = false;
    double ball_spd = 0;

    double aim_rot = 0.0;
    bool   face_incoming = false;
};

// 主入口：算 defender_id 的断球点（含可达性判断与静态点回退）
DefensePlan plan_defense(const WorldModel &wm, int defender_id);

// 球-门连线护门点：球距门 >100cm 站球后 45cm / 45~100 站门前 50cm / <45 站球前 8cm
bool goal_cover_point(const WorldModel &wm, double &out_x, double &out_y);

// 断球点 = 球轨迹 ∩ 球门前 line_dist 处拦截线；false = 球没朝门滚（调用方回退）
bool intercept_point(const WorldModel &wm, double line_dist,
                     double &ix, double &iy);

// 主动截球：沿球轨迹扫采样点，找最早「我比球先到」的点（补横传/斜传盲区）
bool early_intercept_point(const WorldModel &wm, int defender_id,
                           double &out_x, double &out_y);

// 会合点：沿球未来轨迹逐帧外推（含衰减），找「我比球早到 lead_frames 帧」的点；out_aim=迎球朝向
bool ball_meeting_point(const WorldModel &wm, double px, double py,
                        double my_speed, double lead_frames,
                        double &out_x, double &out_y, double &out_aim);

// 犀利进攻防守档：读总开关（defense.kSharpDefense）；收紧值 威胁升档 4cm/帧、贴距 12cm、预测 5 帧、λ 15cm
bool sharp_defense_on();

inline double sharp_danger_speed() { return 4.0; }
inline double sharp_mark_dist()    { return 12.0; }
inline double sharp_mark_lead()    { return 5.0; }
inline double sharp_mark_lambda()  { return 15.0; }

// 盯人贴距(cm)：太近(<8cm)判推球犯规，取 16（犀利档 12）；预测帧数：按被盯者速度外推再站位，太大易超调
inline double mark_dist() { return sharp_defense_on() ? sharp_mark_dist() : 16.0; }

inline double mark_lead() { return sharp_defense_on() ? sharp_mark_lead() : 3.0; }

// 堵传球线距离 / 盯人危险门限(cm)：离球或离门够近才值得贴，否则回区域防守
inline double mark_pass_lane_dist() { return 40.0; }

inline double mark_engage_ball_dist() { return 40.0; }
inline double mark_engage_goal_dist() { return 40.0; }

// 威胁打分：越该被盯分越高（d_ball/d_goal 为 cm，approach/danger 为 cm/帧）
double mark_threat(double d_ball, double d_goal,
                   double approach_speed, double danger_speed, bool is_dribbler);

// 选威胁最大的对手下标(0~4)，含滞回与危险门限；-1 = 无人值得盯
int pick_mark_target(const WorldModel &wm, int current_target);

// 带权匈牙利盯人分配（自研）：全队最优一一匹配，写回 mark_assign，返回被指派台数
int assign_marks(WorldModel &wm);

// 抢断唯一竞标：每帧只让 EV 最高且 >0 的一人抢球，写回 stealer_id（-1 = 无人抢）
void steal_decide(WorldModel &wm);

// 二抢一夹抢站位点；false = 本轮不夹抢（该防守者留区域）
bool double_team_point(const WorldModel &wm, int defender_id,
                       double &out_x, double &out_y);

}  // namespace simuro5
#endif
