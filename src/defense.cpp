// ============================================================
// defense.cpp — 区域防守：断球点计算（队员 D 负责）
//
// 见 defense.hpp 头注释：核心是用球速 (vx, vy) 的方向角做三角外推，
// 把断球点从「球-门连线静态点」升级成「球真实运动轨迹上的点」。
// ============================================================
#include "simuro5/defense.hpp"
#include "simuro5/field_info.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

// ============================================================
// 可调参数（改动后记得同步 docs/06-调参记录.md）
// ============================================================

// 拦截线距球门线的距离(cm)：球门前方多远处开始断球。
//   越近 → 站位越靠门，堵门更稳，但断球更晚、留给反应的时间更少；
//   越远 → 断球更早，但离门远、一旦被变向绕过就回不来。
//   默认 50cm（沿用旧版「球-门连线 50cm」的口径）。
static constexpr double kInterceptLineDist = 50.0;

// 球速阈值(cm/帧)：低于此值视为「球基本停着」，外推方向噪声大、
// 无断球价值，直接回退到静态球-门连线站位。
static constexpr double kMinBallSpeed = 3.0;

// 断球点夹取范围，防止站出场地或贴死边线。
static constexpr double kMinX = 12.0, kMaxX = 208.0;
static constexpr double kMinY = 15.0, kMaxY = 165.0;

// 可达性判断：防守队员最大直线速度(cm/帧)。保守值，需用 rlg 复盘实测标定。
//   （motion 基准车速 vc=100 是轮速量级，换算成每帧位移要现场标定，这里先取 2.0）
static constexpr double kMySpeed = 2.0;

// 可达余量倍数：我到达时间 ≤ 球到达时间 × 该系数 才认为追得上。
//   >1 给自己留缓冲（比如 1.2 = 多留 20% 时间余量）。
static constexpr double kReachMargin = 1.2;

// ============================================================
// 纯函数：球轨迹 ∩ 球门前拦截线
// ============================================================
bool intercept_point(const WorldModel &wm, double line_dist,
                     double &ix, double &iy) {
    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;
    double vx = wm.ball.vx, vy = wm.ball.vy;

    // 拦截线 x：球门线向场内退 line_dist。
    //   attack_dir() 指进攻方向（蓝队 -1，黄队 +1），
    //   所以「球门线 + attack_dir * line_dist」正好是向场内方向。
    //   蓝队：220 + (-1)*50 = 170；黄队：0 + (+1)*50 = 50。
    double line_x = ctx.our_goal_x() + ctx.attack_dir() * line_dist;

    // 三角函数外推：球沿 (vx,vy) 方向到达拦截线时的 y（见 defense.hpp）。
    double y_at_line = 0.0;
    if (!predict_y_at_x(bx, by, vx, vy, line_x, y_at_line)) {
        return false;   // vx≈0 或球背离球门 → 没有「朝门滚」的断球点
    }

    ix = clamp(line_x, kMinX, kMaxX);
    iy = clamp(y_at_line, kMinY, kMaxY);
    return true;
}

// ============================================================
// 主入口：2 号防守队员的断球点
// ============================================================
DefensePlan plan_defense(const WorldModel &wm, int defender_id) {
    DefensePlan plan;
    const TeamContext &ctx = wm.ctx;
    plan.ball_spd = ball_speed(wm.ball.vx, wm.ball.vy);

    // 球是否朝己方球门滚：
    //   用「速度 x 分量」与「球→门」的 x 方向是否同号判断。
    //   蓝队门在 +x 端(220)，球在门左侧，朝门 = vx>0；
    //   黄队门在 -x 端(0)，  球在门右侧，朝门 = vx<0。
    //   统一写成 vx * (goal_x - ball.x) > 0，蓝黄通用。
    double toward_goal = wm.ball.vx * (ctx.our_goal_x() - wm.ball.x);
    plan.approaching = toward_goal > 0.0;

    // 有威胁（朝门滚 + 球速够快）→ 用三角函数算真实断球点；
    // 否则（球慢 / 背离 / 无有效交点）→ 回退到 situation 给的
    // 球-门连线静态点 wm.passive_xy。
    double ix = 0.0, iy = 0.0;
    if (plan.approaching && plan.ball_spd >= kMinBallSpeed &&
        intercept_point(wm, kInterceptLineDist, ix, iy)) {
        // 可达性判断：算出的断球点，我赶不赶得上？
        //   t_ball = 球到截点的时间 = dist(球, 截点) / 球速
        //   t_me   = 我到截点的时间 = dist(我, 截点) / kMySpeed
        //   若 t_me > t_ball × 余量 → 追不上，回退卡位：
        //   避免为追一个够不着的球而失位、把身后空档让给对方。
        double t_ball = dist(wm.ball.x, wm.ball.y, ix, iy) / plan.ball_spd;
        double t_me   = dist(wm.home[defender_id].x, wm.home[defender_id].y, ix, iy) / kMySpeed;
        if (t_me <= t_ball * kReachMargin) {
            plan.target_x = ix;
            plan.target_y = iy;
        } else {
            plan.target_x = wm.passive_x;
            plan.target_y = wm.passive_y;
        }
    } else {
        plan.target_x = wm.passive_x;
        plan.target_y = wm.passive_y;
    }

    // 规则红线：断球点不得进入己方门区（只有守门员能进）。
    //   蓝队门区 x∈[170,220]；拦截线本身 x=170 正好压门区前缘，
    //   若 y 也落在门宽范围内则再往外推 5cm，避免踩线犯规。
    if (in_goal_area(ctx, plan.target_x, plan.target_y)) {
        plan.target_x = clamp(ctx.our_goal_x() + ctx.attack_dir() * 55.0, kMinX, kMaxX);
        plan.target_y = clamp(wm.ball.y, 75.0, 105.0);
    }

    // 兑底（本地第4轮）：防守点若落入己方大禁区则推出（防堆叠送点球）
    if (in_penalty_area(ctx, plan.target_x, plan.target_y)) {
        plan.target_x = ctx.our_goal_x() + ctx.attack_dir() * 85.0;
        plan.target_y = 90.0;
    }
    // 防推球犯规（本地第7轮）：防守点距球保持 >= 8cm（球周围不挤球不推球）
    double db = dist(plan.target_x, plan.target_y, wm.ball.x, wm.ball.y);
    if (db < 8.0) {
        double ang = atan2(wm.ball.y - plan.target_y, wm.ball.x - plan.target_x);
        plan.target_x = wm.ball.x - 8.0 * cos(ang);
        plan.target_y = wm.ball.y - 8.0 * sin(ang);
        // 推球点可能又被推回罚球区（球贴罚球区前缘时），再夹一次防送点球
        if (in_penalty_area(ctx, plan.target_x, plan.target_y)) {
            plan.target_x = ctx.our_goal_x() + ctx.attack_dir() * 85.0;
            plan.target_y = 90.0;
        }
    }
    return plan;
}

// ============================================================
// 人盯人：威胁打分 + 目标选择（见 defense.hpp 注释）
// ============================================================
double mark_threat(double d_ball, double d_goal,
                   double approach_speed, double danger_speed, bool is_dribbler) {
    const double K = 10.0;                  // 距离平滑：避免贴脸分数爆表/每帧抖动
    double s = 50.0 / (d_ball + K);         // ① 控球威胁（主导）：越靠近球越危险
    s      += 25.0 / (d_goal + K);          // ② 位置威胁：越靠门越危险
    // ③ 接球威胁：球正朝该球员冲（快传/直塞），按离球距离加权——
    //    离球近的人（拿得到球）才吃这个加成；球越快朝他、人越近球 → 越危险。
    //    用倒数衰减而非硬阈值，避免阈值附近每帧抖动。
    double reach = 1.0 / (1.0 + d_ball / 20.0);   // d_ball=0→1, 20→0.5, 40→0.33
    s += 0.4 * approach_speed * reach;
    // ④ 持球突破威胁：持球者带球朝门冲，球越快朝门越要贴。
    //    （持球者脚下 d_ball≈0，③ 项 reach 虽≈1 但 approach_speed≈0，靠此项兜住）
    if (is_dribbler) s += 0.3 * danger_speed;
    return s;
}

int pick_mark_target(const WorldModel &wm, int current_target) {
    const TeamContext &ctx = wm.ctx;
    double danger = ball_danger_speed(wm);   // 球朝己方门速度（持球突破威胁用）

    // 持球者 = 离球最近的对方球员
    int dribbler = -1;
    double dmin = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = dist(wm.ball.x, wm.ball.y, wm.opp[i].x, wm.opp[i].y);
        if (d < dmin) { dmin = d; dribbler = i; }
    }

    // 逐人打分取 argmax
    int best = -1;
    double best_score = -1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d_ball = dist(wm.ball.x, wm.ball.y, wm.opp[i].x, wm.opp[i].y);
        double d_goal = ctx.dist_our_goal(wm.opp[i].x);
        double appr = ball_approach_speed(wm, wm.opp[i].x, wm.opp[i].y);
        double score = mark_threat(d_ball, d_goal, appr, danger, i == dribbler);
        if (score > best_score) { best_score = score; best = i; }
    }

    // 滞回：新目标分没超过当前目标 10% 就不换（防每帧换人原地转圈；
    //   20% 太强，实测「选错人」占未贴住 19~22%——危险换人常被滞回挡掉）
    if (current_target >= 0 && current_target < PLAYERS_PER_SIDE &&
        best != current_target) {
        double cur_d_ball = dist(wm.ball.x, wm.ball.y,
                                 wm.opp[current_target].x, wm.opp[current_target].y);
        double cur_d_goal = ctx.dist_our_goal(wm.opp[current_target].x);
        double cur_appr = ball_approach_speed(wm, wm.opp[current_target].x,
                                              wm.opp[current_target].y);
        double cur_score = mark_threat(cur_d_ball, cur_d_goal, cur_appr, danger,
                                       current_target == dribbler);
        if (best_score <= cur_score * 1.1) best = current_target;
    }

    // 危险门限：最终目标必须离球近(持球/抢点) 或 离门近(门前埋伏) 才贴；
    //   否则返回 -1 回区域防守——复盘未贴住帧里 44~48% 被盯者离球 >40cm，
    //   追不危险的对手白费体力还丢区域。
    if (best >= 0) {
        double d_ball = dist(wm.ball.x, wm.ball.y, wm.opp[best].x, wm.opp[best].y);
        double d_goal = ctx.dist_our_goal(wm.opp[best].x);
        if (d_ball > mark_engage_ball_dist() && d_goal > mark_engage_goal_dist()) {
            return -1;
        }
    }
    return best;
}

}  // namespace simuro5
