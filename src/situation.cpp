#include "simuro5/situation.hpp"
#include "simuro5/field_info.hpp"
#include <cmath>
#include <algorithm>
#define TUNABLE_PREFIX "situation."
#include "simuro5/tunable.hpp"

namespace simuro5 {

// 尾随接应（前场边路）：球后 TrailDist cm、向中路走廊靠 TrailPull；离对方门 TrailZone cm 内且偏离中线 TrailLateral 才启用
TUNABLE(kTrailEnabled, 1.0);
TUNABLE(kTrailZone, 110.0);
TUNABLE(kTrailLateral, 22.0);
TUNABLE(kTrailDist, 26.0);
TUNABLE(kTrailPull, 0.45);

// 门前捡漏（tap-in）：球进对方门前 kTapInZone cm 内时，assist 不撤、改蹲门前正面当捡漏点
//   kTapInDepth=离门多远；y 死站门中心（接横传/回做，不跟着球上下偏，避免半吊子站位）
TUNABLE(kTapInZone, 55.0);
TUNABLE(kTapInDepth, 25.0);

Situation SituationModule::analyze(const WorldModel &wm) {
    Situation sit;
    const TeamContext &ctx = wm.ctx;
    double bx = wm.ball.x, by = wm.ball.y;

    // 球权判定：20cm 内且比对方近 5cm 才算明确控球，否则按最近距离兜底
    double our_min = 1e9, opp_min = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        our_min = std::min(our_min, dist(bx, by, wm.home[i].x, wm.home[i].y));
        opp_min = std::min(opp_min, dist(bx, by, wm.opp[i].x, wm.opp[i].y));
    }
    bool by_distance = (our_min < opp_min) && (our_min < 20.0);
    // 平台 whosBall 语义不可靠，只留作标定计数（whos_mismatch）
    bool near_ours   = (our_min < 20.0) && (our_min + 5.0 < opp_min);
    bool near_theirs = (opp_min < 20.0) && (opp_min + 5.0 < our_min);
    if (near_ours) {
        sit.we_have_ball = true;
    } else if (near_theirs) {
        sit.we_have_ball = false;
    } else {
        sit.we_have_ball = by_distance;
    }
    if (wm.whos_ball != 0) sit.whos_mismatch = ((wm.whos_ball == 1) != by_distance);

    sit.ball_in_our_half = ctx.attack_dir() > 0 ? (bx < 110.0) : (bx > 110.0);
    sit.ball_in_our_penalty = in_penalty_area(ctx, bx, by);
    sit.ball_in_opp_penalty = in_opp_penalty_area(ctx, bx, by);

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

    double ad = ctx.attack_dir();
    double gx = ctx.our_goal_x();
    bool attack = (wm.team_state == TS_ATTACK);

    // 防守锚点：球-己方门连线，距门约 50cm（攻防共用）
    double gy = 90.0;
    double d = dist(bx, by, gx, gy);
    if (d > 30) {
        double t = 50.0 / d;
        wm.passive_x = clamp(bx + t * (gx - bx), 10.0, 210.0);
        wm.passive_y = clamp(by + t * (gy - by), 10.0, 170.0);
    } else {
        wm.passive_x = clamp(gx + ad * 50.0, 10.0, 210.0);
        wm.passive_y = 90.0;
    }

    double opp_box_edge = ctx.opp_goal_x() - ad * 85.0;

    double ax, ay, mx, my;
    bool trail = false;
    if (attack) {
        ax = clamp(bx + ad * 40.0, 15.0, 205.0);
        ay = clamp(by + 40.0, 20.0, 160.0);
        // 前场边路：助攻改成「跟球尾随 + 向中路靠」——球脱脚时有人接，回做也有人抢点
        //   原来的「球前 40cm 偏上」在这种局面落在人堆/边线外，前点后点都不占
        if (kTrailEnabled >= 0.5 && ctx.dist_opp_goal(bx) < kTrailZone && std::fabs(by - 90.0) > kTrailLateral) {
            ax = clamp(bx - ad * kTrailDist, 15.0, 205.0);
            ay = clamp(by + (90.0 - by) * kTrailPull, 20.0, 160.0);
            trail = true;
        }
        if (in_opp_penalty_area(ctx, ax, ay)) ax = opp_box_edge;
        mx = clamp(110.0 + (bx - 110.0) * 0.5, 15.0, 205.0);
        my = clamp(by - 40.0, 20.0, 160.0);
        if (in_opp_penalty_area(ctx, mx, my)) mx = opp_box_edge;
        // 门前捡漏：球进对方罚球区或门线 kTapInZone cm 内时，assist 不再撤到禁区外沿，
        //   改蹲门前正面（离门 kTapInDepth）当捡漏点，y 死站门中心接横传/回做
        //   （不跟着球上下偏——球在底角时偏三成会把 assist 卡在球和门之间半吊子，够不着球也接不到横传）
        //   尾随位不撤：它正是回做要用的那个接应点
        if (!trail && (in_opp_penalty_area(ctx, bx, by) || ctx.dist_opp_goal(bx) < kTapInZone)) {
            ax = ctx.opp_goal_x() - ad * kTapInDepth;
            ay = 90.0;
            mx = 110.0;
        }
    } else {
        // 防守态：威胁 <0.6 时 assist/mid 站中线偏前 30cm 当反击支点，否则回收中线
        if (wm.threat_level < 0.6) {
            ax = 110.0 + ad * 30.0;
            mx = 110.0 + ad * 30.0;
        } else {
            ax = 110.0;
            mx = 110.0;
        }
        ay = clamp(by + 40.0, 20.0, 160.0);
        my = clamp(by - 40.0, 20.0, 160.0);
    }

    // 跑位前瞻：目标点附近有对手时，y 往空档侧挪开（躲人半径 25cm、位移 30cm）
    const double kAvoidRadius = 25.0;
    const double kAvoidShift  = 30.0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (dist(wm.opp[i].x, wm.opp[i].y, ax, ay) < kAvoidRadius) {
            ay = (wm.opp[i].y >= ay) ? ay - kAvoidShift : ay + kAvoidShift;
            ay = clamp(ay, 20.0, 160.0);
        }
        if (dist(wm.opp[i].x, wm.opp[i].y, mx, my) < kAvoidRadius) {
            my = (wm.opp[i].y >= my) ? my - kAvoidShift : my + kAvoidShift;
            my = clamp(my, 20.0, 160.0);
        }
    }

    // 锚点滞回 15cm：变化小于此不更新，防追着每帧移动的锚点频繁变向（防守锚点不设）
    const double kAnchorHysteresis = 15.0;
    if (dist(ax, ay, wm.assist_x, wm.assist_y) >= kAnchorHysteresis) { wm.assist_x = ax; wm.assist_y = ay; }
    if (dist(mx, my, wm.mid_x, wm.mid_y) >= kAnchorHysteresis)       { wm.mid_x = mx;  wm.mid_y = my; }

    bool ball_opp_half = (ad > 0) ? (bx > 110.0) : (bx < 110.0);
    if (ball_opp_half) {
        wm.passive_x = (ad > 0) ? std::max(wm.passive_x, 65.0)
                                 : std::min(wm.passive_x, 155.0);
    }
    // 防守堆叠纪律：三台区域防守者两两最小间距 20cm，沿连线各推一半（2 轮）
    {
        const double kDefenseMinSep = 20.0;
        for (int it = 0; it < 2; ++it) {
            double ax[3] = { wm.passive_x, wm.assist_x, wm.mid_x };
            double ay[3] = { wm.passive_y, wm.assist_y, wm.mid_y };
            for (int a = 0; a < 3; ++a) {
                for (int b = a + 1; b < 3; ++b) {
                    double dx = ax[b] - ax[a], dy = ay[b] - ay[a];
                    double d = std::hypot(dx, dy);
                    if (d >= kDefenseMinSep) continue;
                    if (d < 1e-6) { dx = 0.0; dy = 1.0; d = 1.0; }
                    double push = (kDefenseMinSep - d) * 0.5;
                    double ux = dx / d, uy = dy / d;
                    ax[a] -= ux * push; ay[a] -= uy * push;
                    ax[b] += ux * push; ay[b] += uy * push;
                }
            }
            wm.passive_x = clamp(ax[0], 10.0, 210.0);
            wm.passive_y = clamp(ay[0], 10.0, 170.0);
            wm.assist_x  = clamp(ax[1], 15.0, 205.0);
            wm.assist_y  = clamp(ay[1], 20.0, 160.0);
            wm.mid_x     = clamp(ax[2], 15.0, 205.0);
            wm.mid_y     = clamp(ay[2], 20.0, 160.0);
        }
    }
    // 己方禁区纪律：三个锚点不得进己方门区/罚球区（防堆叠送点）
    if (in_goal_area(ctx, wm.passive_x, wm.passive_y))
        wm.passive_x = clamp(gx + ad * 55.0, 10.0, 210.0);
    if (in_penalty_area(ctx, wm.passive_x, wm.passive_y))
        wm.passive_x = clamp(gx + ad * 85.0, 10.0, 210.0);
    if (in_penalty_area(ctx, wm.assist_x, wm.assist_y))
        wm.assist_x = clamp(gx + ad * 85.0, 15.0, 205.0);
    if (in_penalty_area(ctx, wm.mid_x, wm.mid_y))
        wm.mid_x = clamp(gx + ad * 85.0, 15.0, 205.0);
    // 防守锚点也不进对方门前 65cm 以内（对方门区深 50 + 15cm 余量）
    if (ad < 0) {
        wm.passive_x = std::max(wm.passive_x, 65.0);     // 蓝方守 x=220：不进 x<65
        wm.assist_x  = std::max(wm.assist_x, 65.0);
        wm.mid_x     = std::max(wm.mid_x, 65.0);
    } else {
        wm.passive_x = std::min(wm.passive_x, 155.0);    // 黄方守 x=0：不进 x>155
        wm.assist_x  = std::min(wm.assist_x, 155.0);
        wm.mid_x     = std::min(wm.mid_x, 155.0);
    }
}

}  // namespace simuro5
