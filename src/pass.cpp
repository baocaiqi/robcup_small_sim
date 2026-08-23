#include "simuro5/pass.hpp"
#include "simuro5/geometry.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/role_assignment.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

namespace {
// 调参常量
constexpr double PASS_MAX_DIST     = 60.0;    // 最大传球距离 cm
constexpr double PASS_MIN_DIST     = 8.0;     // 最小传球距离，避免贴脸传球
constexpr double BLOCK_THRESHOLD   = 15.0;    // 传球线路阻挡阈值 cm
constexpr double OFFSET_BASE       = 6.0;     // 接应点向前的领球偏移 cm
constexpr double THREAT_RADIUS     = 30.0;    // 接应点周围敌方威胁半径 cm
constexpr double FIELD_MARGIN      = 6.0;     // 接应点离边线的最小距离 cm

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

        // —— 评分：越靠前越好 + 威胁越低越好 + 传球越短越稳 ——
        double goal_dist = std::fabs(tx - ctx.opp_goal_x());
        double score = goal_dist + threat * 20.0 + pass_dist * 0.5;

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

} // namespace simuro5
