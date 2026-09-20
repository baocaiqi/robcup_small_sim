#include "simuro5/pass.hpp"
#include "simuro5/shoot.hpp"        // 复用射门质量（把球挪到接球点问一次）
#include "simuro5/defense.hpp"      // segment_clear_of_circles / CircleObstacle
#include "simuro5/field_info.hpp"   // in_opp_goal_area
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
// 调参常量
TUNABLE(PASS_MAX_DIST, 69.4267);  // 最大传球距离 cm
TUNABLE(PASS_MIN_DIST, 8.0);  // 最小传球距离，避免贴脸传球
TUNABLE(BLOCK_THRESHOLD, 7.4329);  // 传球线路阻挡阈值 cm
TUNABLE(OFFSET_BASE, 10.1);  // 接应点向前的领球偏移 cm
TUNABLE(THREAT_RADIUS, 34.1917);  // 接应点周围敌方威胁半径 cm
TUNABLE(FIELD_MARGIN, 6.0);  // 接应点离边线的最小距离 cm

// 路线 (sx,sy)->(tx,ty) 是否被某个对手机器人挡住
bool route_blocked(const WorldModel &wm, double sx, double sy, double tx, double ty) {
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = point_to_segment_dist(wm.opp[i].x, wm.opp[i].y, sx, sy, tx, ty);
        if (d < BLOCK_THRESHOLD) {
            return true;
        }
    }
    return false;
}

// 统计目标点周围敌方机器人的「距离加权威胁度」：越近权重越高，
// 贴脸(d=0)≈1.0、半径边缘(d=THREAT_RADIUS)≈0.0，区分「被紧盯」与「附近路过」。
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
            threat += 1.0 - d2 / r2;   // 贴脸≈1.0，边缘≈0.0
        }
    }
    return threat;
}

// 统计目标点周围「前方威胁」：比目标点更靠对方球门的对手
int count_front_opponent(const WorldModel &wm, double x, double y, double ad) {
    int cnt = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double dx = wm.opp[i].x - x, dy = wm.opp[i].y - y;
        if (dx*dx + dy*dy < THREAT_RADIUS * THREAT_RADIUS) {
            if (ad * (wm.opp[i].x - x) > 0.0) cnt++;   // 前方 = 对方球门方向
        }
    }
    return cnt;
}

// 把接应点夹回场内，并避免深入对方罚球区（带球/接应不该进禁区）。
void clamp_receive_point(const TeamContext &ctx, double &x, double &y) {
    x = clamp(x, FIELD_MARGIN, TeamContext::FIELD_LENGTH - FIELD_MARGIN);
    y = clamp(y, FIELD_MARGIN, TeamContext::FIELD_WIDTH - FIELD_MARGIN);
    if (in_opp_penalty_area(ctx, x, y)) {
        // 落在对方罚球区内：沿 x 推到禁区外沿（朝持球者一侧）
        double gx = ctx.opp_goal_x();
        x = (ctx.attack_dir() > 0) ? gx - 80.0 - FIELD_MARGIN : gx + 80.0 + FIELD_MARGIN;
    }
}

// ============================================================
// D2：传球收益三步升级（P0，2026-09 校赛前）
//   ① 射门收益：接应点能否形成射门（复用 plan_shoot 预判）
//   ② 到达时间：对手是否比接应队友更快到点（含速度）
//   ③ 速度威胁：正在逼近接应点的对手加权（用 opp_vx/opp_vy）
// ============================================================
constexpr double SHOOT_BONUS      = 60.0;  // 接应点能射门 → 评分奖励（goal_dist 量级）
constexpr double ARRIVE_PENALTY   = 40.0;  // 对手明显先到 → 惩罚
constexpr double SPEED_THREAT_W   = 10.0;  // 速度威胁权重
constexpr double OUR_ARRIVE_SPEED = 2.0;   // 己方到点速度 cm/帧（与 defense kMySpeed 口径一致）
constexpr double OPP_MIN_SPEED    = 1.0;   // 对手速度下限 cm/帧（静止兜底，防除零）
constexpr double ARRIVE_MARGIN    = 1.0;   // 对手到达时间 < 己方×该系数 → 判对手先到
constexpr double SPEED_BASE       = 2.0;   // 速度投影归一化基准 cm/帧

// ① 接应点能否形成射门：拷贝世界模型、把球放到接应点，调 plan_shoot 预判。
//    plan_shoot 只读 wm.ball 位置 + 对方门将 y + 球距门距离（不用射手位置），
//    因此「假球位」即可预判「球推到接应点是否具备射门开口」。
bool receive_point_can_shoot(const WorldModel &wm, double tx, double ty) {
    WorldModel wm2 = wm;
    wm2.ball.x = tx; wm2.ball.y = ty;
    return plan_shoot(wm2, 0).viable;
}

// ② 对手到接应点的最短到达时间（帧）。静止对手按 OPP_MIN_SPEED 兜底。
double opp_arrive_time(const WorldModel &wm, double tx, double ty) {
    double t_min = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = dist(wm.opp[i].x, wm.opp[i].y, tx, ty);
        double spd = std::max(std::hypot(wm.opp_vx[i], wm.opp_vy[i]), OPP_MIN_SPEED);
        t_min = std::min(t_min, d / spd);
    }
    return t_min;
}

// ③ 速度威胁：威胁半径内、正在朝接应点逼近的对手，按接近速度加权。
//    静态威胁 × (接近速度/基准)；静止或背离的对手不额外加权。
double speed_threat(const WorldModel &wm, double x, double y) {
    const double r2 = THREAT_RADIUS * THREAT_RADIUS;
    double threat = 0.0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double dx = wm.opp[i].x - x, dy = wm.opp[i].y - y;
        double d2 = dx * dx + dy * dy;
        if (d2 < r2 && d2 > 1e-6) {
            double d = std::sqrt(d2);
            double approach = -(wm.opp_vx[i] * dx + wm.opp_vy[i] * dy) / d;  // dx、dy 从接应点指向对手，取负使逼近为正
            if (approach > 0.0) {
                threat += (1.0 - d2 / r2) * (approach / SPEED_BASE);
            }
        }
    }
    return threat;
}


}  // anonymous namespace

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

        // —— 接应点基准：优先用 A 输出的角色站位点（联动），其余回退本体坐标 ——
        double base_x = wm.home[id].x, base_y = wm.home[id].y;
        switch (wm.role[id]) {
            case ROLE_ASSIST:   base_x = wm.assist_x;  base_y = wm.assist_y;  break;
            case ROLE_MIDFIELD: base_x = wm.mid_x;     base_y = wm.mid_y;     break;
            case ROLE_PASSIVE:  base_x = wm.passive_x; base_y = wm.passive_y; break;
            default: break;   // GOALIE / 未分配：无站位点，用本体坐标
        }

        // —— 选点：在站位点前方领出一个接应点（向前推进），再夹回场内 ——
        double tx = base_x + ad * OFFSET_BASE;
        double ty = base_y;
        clamp_receive_point(ctx, tx, ty);

        // 传球距离校验
        double pass_dist = dist(px, py, tx, ty);
        if (pass_dist <= PASS_MIN_DIST || pass_dist >= PASS_MAX_DIST) {
            continue;
        }

        // 路线被挡则排除
        if (route_blocked(wm, px, py, tx, ty)) {
            continue;
        }

        // —— 威胁评估：在接应点（不是队友当前位置）按距离加权统计对手盯防 ——
        double threat = count_near_opponent(wm, tx, ty);
        int front_threat = count_front_opponent(wm, tx, ty, ad);   // 前方威胁（比目标点更靠对方球门）

        // —— D2：射门收益 + 到达时间 + 速度威胁 ——
        bool can_shoot = receive_point_can_shoot(wm, tx, ty);      // ① 接到即可射门
        double t_opp = opp_arrive_time(wm, tx, ty);                // ② 对手最快到达时间
        double t_our = dist(wm.home[id].x, wm.home[id].y, tx, ty) / OUR_ARRIVE_SPEED;
        bool opp_first = t_opp < t_our * ARRIVE_MARGIN;            // 对手先到 → 危险
        double spd_threat = speed_threat(wm, tx, ty);              // ③ 逼近中对手

        // —— 评分：越靠前越好 + 威胁越低越好 + 传球越短越稳 + 射门收益 - 对手先到 - 速度威胁 ——
        double goal_dist = std::fabs(tx - ctx.opp_goal_x());
        double score = goal_dist + threat * 20.0 + front_threat * 12.0 + pass_dist * 0.5;
        if (can_shoot) score -= SHOOT_BONUS;
        if (opp_first) score += ARRIVE_PENALTY;
        score += spd_threat * SPEED_THREAT_W;

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

// ============================================================
// 配合进攻：选"接球后射门机会最好"的队友（docs/06 第 69 轮，用户 2026-09-15 指令）
//   打分 = 把球挪到该队友的接球点、问一次射门模块得到的 quality（复用同一套净开口几何）；
//   若从那里根本射不了（>110cm 射程外），给一个随距离衰减的底分，避免所有远点同为 0 分。
//   安全性（用户要求的"可以安全接受"）：
//     ① 球→接球点 线段上无对手（对手按半径 8cm 圆盘）
//     ② 接球点 20cm 内无对手（被贴身不算安全）
//     ③ 传球距离 ≤120cm（再长容易被对方中场截）
//   只向前：接球点到对方门的距离必须比球更近 5cm 以上；接球点不得在对方门区（纪律）。
// ============================================================
CoopPass plan_coop_pass(const WorldModel &wm, int passer_id) {
    CoopPass best;
    const TeamContext &ctx = wm.ctx;
    const double bx = wm.ball.x, by = wm.ball.y;
    const double ogx = ctx.opp_goal_x();
    const double d_goal = dist(bx, by, ogx, 90.0);

    const int cand[3] = {2, 3, 4};                     // 助攻 / 中场 / 后卫（门将不参与）
    const double ax[3] = {wm.assist_x, wm.mid_x, wm.passive_x};
    const double ay[3] = {wm.assist_y, wm.mid_y, wm.passive_y};

    for (int k = 0; k < 3; ++k) {
        const int i = cand[k];
        if (i == passer_id) continue;
        const double rx = ax[k], ry = ay[k];
        if (in_opp_goal_area(ctx, rx, ry)) continue;                       // 纪律：不进对方门区
        if (dist(rx, ry, ogx, 90.0) > d_goal - 5.0) continue;              // 只向前传
        if (dist(bx, by, rx, ry) > 120.0) continue;                        // 太远不传

        CircleObstacle obs[PLAYERS_PER_SIDE];
        for (int j = 0; j < PLAYERS_PER_SIDE; ++j) {
            obs[j].x = wm.opp[j].x; obs[j].y = wm.opp[j].y; obs[j].r = 8.0;
        }
        if (!segment_clear_of_circles(bx, by, rx, ry, obs, PLAYERS_PER_SIDE)) continue;   // ① 线路
        double opp_min = 1e9;
        for (int j = 0; j < PLAYERS_PER_SIDE; ++j)
            opp_min = std::min(opp_min, dist(rx, ry, wm.opp[j].x, wm.opp[j].y));
        if (opp_min < 20.0) continue;                                      // ② 接球点被贴身

        WorldModel tmp = wm;                                               // 借一份世界模型问射门
        tmp.ball.x = rx; tmp.ball.y = ry; tmp.ball.vx = 0.0; tmp.ball.vy = 0.0;
        tmp.in_penalty_exec = false;
        ShootPlan ps = plan_shoot(tmp, i);
        double q = ps.viable ? ps.quality : 0.0;
        if (!ps.viable) {                                                  // 射程外：按距离给底分
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
