#include "simuro5/pass.hpp"
#include "simuro5/geometry.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>



namespace simuro5 {


namespace {
// 调参常量
constexpr double PASS_MAX_DIST     = 60.0;    // 最大传球距离 cm
constexpr double PASS_MIN_DIST     = 8.0;     // 最小传球距离，避免贴脸传球
constexpr double BLOCK_THRESHOLD   = 15.0;    // 传球线路阻挡阈值 cm
constexpr double OFFSET_BASE       = 5.0;     // 基础向前偏移
constexpr double THREAT_RADIUS     = 25.0;    // 接应点周围敌方威胁半径


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


// 统计目标点周围敌方机器人数量（威胁度）
int count_near_opponent(const WorldModel &wm, double x, double y)
{
    int cnt = 0;
    for(int i=0;i<PLAYERS_PER_SIDE;i++)
    {
        double dx = wm.opp[i].x - x;
        double dy = wm.opp[i].y - y;
        if(dx*dx + dy*dy < THREAT_RADIUS * THREAT_RADIUS)
        {
            cnt++;
        }
    }
    return cnt;
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
    int best_threat = 0;


    for (int id = 0; id < PLAYERS_PER_SIDE; ++id) {
        if (id == passer_id) continue;
        const auto &rob = wm.home[id];

        double raw_dist = dist(px, py, rob.x, rob.y);
        printf("[RAW_DIST] id:%d raw dist:%.2f  min:%.2f max:%.2f\n",
                id, raw_dist, PASS_MIN_DIST, PASS_MAX_DIST);

        int threat = count_near_opponent(wm, rob.x, rob.y);
        double dynamic_offset = OFFSET_BASE * std::max(0.2, 1.0 - 0.25 * threat);

        double rx2 = rob.x + ad * dynamic_offset;
        double ry2 = rob.y;

        // 重点：判断偏移之后接应点的距离（业务真实逻辑）
        double pass_dist = dist(px, py, rx2, ry2);
        if (pass_dist <= PASS_MIN_DIST || pass_dist >= PASS_MAX_DIST) {
            continue;
        }

        bool blocked = route_blocked(wm, px, py, rx2, ry2);
        if (blocked) {
            continue;
        }

        double goal_dist = std::fabs(rob.x - ctx.opp_goal_x());
        double score = goal_dist + threat * 12.0;

        if (score < best_score) {
            best_score = score;
            best = id;
            best_tx = rx2;
            best_ty = ry2;
            best_threat = threat;
        }
    }


    if (best < 0) {
        printf("[PASS_DEBUG] passer:%d no valid receiver\n", passer_id);
        return plan;
    }

    plan.viable = true;
    plan.receiver_id = best;
    plan.target_x = best_tx;
    plan.target_y = best_ty;
    printf("[PASS_DEBUG] passer:%d -> recv:%d threat:%d target(%.2f,%.2f)\n",
           passer_id, best, best_threat, best_tx, best_ty);


    return plan;
}


} // namespace simuro5