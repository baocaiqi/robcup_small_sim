#include "simuro5/situation.hpp"
#include "simuro5/field_info.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

Situation SituationModule::analyze(const WorldModel &wm) {
    Situation sit;
    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;

    // 球权：简版 = 最近的人是否是自己人（距离阈值）
    double our_min = 1e9, opp_min = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        our_min = std::min(our_min, dist(bx, by, wm.home[i].x, wm.home[i].y));
        opp_min = std::min(opp_min, dist(bx, by, wm.opp[i].x, wm.opp[i].y));
    }
    sit.we_have_ball = (our_min < opp_min) && (our_min < 20.0);

    // 半场/禁区判断（参数化）
    sit.ball_in_our_half = ctx.attack_dir() > 0 ? (bx < 110.0) : (bx > 110.0);
    sit.ball_in_our_penalty = in_penalty_area(ctx, bx, by);
    sit.ball_in_opp_penalty = in_opp_penalty_area(ctx, bx, by);

    // 威胁等级
    if (sit.ball_in_our_penalty && !sit.we_have_ball)      sit.threat_level = 1.0;
    else if (sit.ball_in_our_half && !sit.we_have_ball)    sit.threat_level = 0.6;
    else if (sit.we_have_ball)                             sit.threat_level = 0.1;
    else                                                   sit.threat_level = 0.3;
    return sit;
}

void SituationModule::update_stand_points(WorldModel &wm) {
    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;
    if (!wm.ball.valid) return;

    double ad = ctx.attack_dir();   // +1 或 -1
    double ogx = ctx.our_goal_x();

    // 助攻站位：球前方偏侧（和持球人错开）
    // 截图前拉：助攻点球前 40cm（原 25cm 太保守）
    wm.assist_x = clamp(bx + ad * 40.0, 15.0, 205.0);
    // 宽度课题：助攻点偏球上方(y+40)，中场点偏球下方(y-40)，两台天然分上下两侧（原来都拤中路）
    wm.assist_y = clamp(by + 40.0, 20.0, 160.0);

    // 中场站位：中线附近，另一侧
    // 增大前压系数 0.3->0.6：球前压时中场也跟着前移
    wm.mid_x = clamp(110.0 + (bx - 110.0) * 0.6, 15.0, 205.0);
    wm.mid_y = clamp(by - 40.0, 20.0, 160.0);

    // 防守站位：球-己方球门连线，距球门约 50cm
    double gx = ogx, gy = 90.0;
    double d = dist(bx, by, gx, gy);
    if (d > 30) {
        double t = 50.0 / d;
        wm.passive_x = clamp(bx + t * (gx - bx), 10.0, 210.0);
        wm.passive_y = clamp(by + t * (gy - by), 10.0, 170.0);
    } else {
        wm.passive_x = clamp(gx + ad * 50.0, 10.0, 210.0);
        wm.passive_y = 90.0;
    }

    // 高位防守：球在对方半场时，防守线前压（防全缩后场）
    bool ball_opp_half = (ad > 0) ? (bx > 110.0) : (bx < 110.0);
    if (ball_opp_half) {
        wm.passive_x = (ad > 0) ? std::max(wm.passive_x, 65.0)
                                 : std::min(wm.passive_x, 155.0);
    }
    // 禁区修正：防守点不得进入己方门区/罚球区
    // 罚球区（大禁区）堆叠 4 人 = 送点球！    if (in_goal_area(ctx, wm.passive_x, wm.passive_y))
        wm.passive_x = clamp(gx + ad * 55.0, 10.0, 210.0);
    if (in_penalty_area(ctx, wm.passive_x, wm.passive_y))
        wm.passive_x = clamp(gx + ad * 85.0, 10.0, 210.0);
    // 助攻点避开己方罚球区（球在禁区时外拉，防堆叠送点）
    if (in_penalty_area(ctx, wm.assist_x, wm.assist_y))
        wm.assist_x = clamp(gx + ad * 85.0, 15.0, 205.0);
}

}  // namespace simuro5
