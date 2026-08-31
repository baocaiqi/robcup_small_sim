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
    double gx = ctx.our_goal_x();
    bool attack = (wm.team_state == TS_ATTACK);

    // —— 防守锚点（PASSIVE/中卫）：球-己方球门连线，距门约 50cm（攻防共用）——
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

    // 对方罚球区外沿（进攻方站位不进入对方禁区，留 5cm 余量）
    double opp_box_edge = ctx.opp_goal_x() - ad * 85.0;

    // —— 助攻/中场目标点（先算局部变量，走滞回后再写入）——
    double ax, ay, mx, my;
    if (attack) {
        // 进攻态：助攻球前 40cm 偏上、中场中线前压 0.5 偏下，均不进入对方禁区
        ax = clamp(bx + ad * 40.0, 15.0, 205.0);
        ay = clamp(by + 40.0, 20.0, 160.0);
        if (in_opp_penalty_area(ctx, ax, ay)) ax = opp_box_edge;
        mx = clamp(110.0 + (bx - 110.0) * 0.5, 15.0, 205.0);
        my = clamp(by - 40.0, 20.0, 160.0);
        if (in_opp_penalty_area(ctx, mx, my)) mx = opp_box_edge;
        // 门前半撤：球攻进对方罚球区时——assist 留禁区外沿当近端短传出球点
        // （治"主攻被围无近端接应"，参考 2008 心得「禁区内能安全倒开球是关键」），
        // mid 回撤中线防反击。
        // 触发条件除罚球区外，还包括球在对方门线附近 55cm 内（含门角外的球：
        //   球滚到门角 y<72.5 或 >107.5 时不在罚球区判定内，但锚点会贴到门区线
        //   附近 x≈55-59，落位振荡即踩线送点球——sim 诊断 f6861/f15621 实测）。
        if (in_opp_penalty_area(ctx, bx, by) || ctx.dist_opp_goal(bx) < 55.0) {
            ax = opp_box_edge; mx = 110.0;
        }
    } else {
        // 防守态：助攻/中场回收中线两侧（保持出球点 + 防守纵深）
        ax = 110.0;
        ay = clamp(by + 40.0, 20.0, 160.0);
        mx = 110.0;
        my = clamp(by - 40.0, 20.0, 160.0);
    }

    // —— 跑位前瞻修正：目标点附近有对手时，y 往空档侧挪开（别跑到别人怀里）——
    //    参考 2008 战术心得「跑位要考虑对方会不会先到」，自研实现；
    //    与现有约束合并：在滞回之前算，禁区外沿/门前回撤/己方禁区纪律仍生效。
    const double kAvoidRadius = 25.0;    // 对手距目标点多近需要躲（cm）
    const double kAvoidShift  = 30.0;    // 躲开的 y 位移（cm）
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

    // —— 锚点滞回：目标变化 < kAnchorHysteresis 不更新 ——
    //    均速落后 demo 的主因之一：追着每帧移动的锚点频繁变向（差速轮转向慢吃速度）。
    //    目标点冻结 → 机器人跑直线、到位等待，少无效转向。防守锚点不设滞回（要响应快）。
    const double kAnchorHysteresis = 15.0;   // cm（可调，见 docs/06）
    if (dist(ax, ay, wm.assist_x, wm.assist_y) >= kAnchorHysteresis) { wm.assist_x = ax; wm.assist_y = ay; }
    if (dist(mx, my, wm.mid_x, wm.mid_y) >= kAnchorHysteresis)       { wm.mid_x = mx;  wm.mid_y = my; }

    // 高位防守：球在对方半场时，防守线前压（防全缩后场，攻防均适用）
    bool ball_opp_half = (ad > 0) ? (bx > 110.0) : (bx < 110.0);
    if (ball_opp_half) {
        wm.passive_x = (ad > 0) ? std::max(wm.passive_x, 65.0)
                                 : std::min(wm.passive_x, 155.0);
    }
    // 己方禁区纪律：防守点/助攻点/中场点不得进入己方门区与罚球区（防堆叠送点）
    if (in_goal_area(ctx, wm.passive_x, wm.passive_y))
        wm.passive_x = clamp(gx + ad * 55.0, 10.0, 210.0);
    if (in_penalty_area(ctx, wm.passive_x, wm.passive_y))
        wm.passive_x = clamp(gx + ad * 85.0, 10.0, 210.0);
    if (in_penalty_area(ctx, wm.assist_x, wm.assist_y))
        wm.assist_x = clamp(gx + ad * 85.0, 15.0, 205.0);
    if (in_penalty_area(ctx, wm.mid_x, wm.mid_y))
        wm.mid_x = clamp(gx + ad * 85.0, 15.0, 205.0);
    // 对方门区防护带（禁区纪律-进攻侧）：防守锚点也不得进入对方门前 65cm 以内。
    //   球被压到对方门前时，球-门连线 50cm 锚点会落在门区线正上方（x≈50~60），
    //   落位振荡即踩线送点球（sim 诊断 f15621/f23607：passive 在真门区 x≈47-50）。
    //   镜像己方纪律：与对方门线保持 15cm 余量（65 = 门区深 50 + 余量 15，
    //   与 clamp_out_opp_goal_area 口径一致；10cm 余量实测违规回升且得分无改善）。
    if (ad < 0) {
        wm.passive_x = std::max(wm.passive_x, 65.0);     // 蓝方守 x=220：不进 x<65
        // 助攻/中场同样受门区纪律约束（sim 诊断 f16376：ASSIST 落位/穿行对方
        //   门区 x≈32-47 连续 21+ 帧送点球；进攻站 x=85 不受影响）
        wm.assist_x  = std::max(wm.assist_x, 65.0);
        wm.mid_x     = std::max(wm.mid_x, 65.0);
    } else {
        wm.passive_x = std::min(wm.passive_x, 155.0);    // 黄方守 x=0：不进 x>155
        wm.assist_x  = std::min(wm.assist_x, 155.0);
        wm.mid_x     = std::min(wm.mid_x, 155.0);
    }
}

}  // namespace simuro5
