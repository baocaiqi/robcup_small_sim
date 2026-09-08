#include "simuro5/roles.hpp"
#include "simuro5/role_assignment.hpp"
#include "simuro5/motion.hpp"
#include "simuro5/shoot.hpp"
#include "simuro5/pass.hpp"
#include "simuro5/defense.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/route.hpp"
#include <cmath>

namespace simuro5 {

namespace {
// ============================================================
// docs/15 P0-4：避障移动 helper（追球/移动避障路径层接线，纯路径无射门依赖）
// ============================================================
constexpr double kRouteInflate = 10.0;   // 对方机器人避障半径（本体 6 + 净空 4）

// 避障移动：从 (r.x,r.y) 向 (tx,ty)，对方 5 机器人作圆盘障碍。
//   直线通 → 直线（最短即最优）；直线被挡 → 可见图+Dijkstra 绕行；
//   目标被障碍吞 / 无通路 → 回退直线（plan_route found=false）。
//   decel=true 时（追球场景）接近目标 10cm 内线性减速，防冲过头把球铲偏。
void move_avoiding(WorldModel &wm, RobotState &r, int id,
                   double tx, double ty, bool decel = false) {
    CircleObstacle obs[PLAYERS_PER_SIDE];
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        obs[i].x = wm.opp[i].x;
        obs[i].y = wm.opp[i].y;
        obs[i].r = kRouteInflate;
    }
    RoutePlan rt = plan_route(r.x, r.y, tx, ty, obs, PLAYERS_PER_SIDE);
    if (rt.found && rt.n_wp > 2) {
        wm.route_wp_next[id] = 0;      // 每帧重置：路径变了不沿用旧段（follow_route 自动跳段）
        motion::follow_route(r, rt, wm.route_wp_next[id]);
    } else {
        motion::position(r, tx, ty);
    }
    if (decel) {
        double dg = dist(r.x, r.y, tx, ty);
        if (dg < 10.0) { r.vl *= dg / 10.0; r.vr *= dg / 10.0; }   // 同 chase_ball 防冲
    }
}

// ============================================================
// 无球接应拉开（队员C）：站位点附近有敌方机器人时，沿 Y 轴横向躲开。
//   · 只微调 Y，保留 A 输出的原始 X（全局跑位点由 situation.cpp 决定，这里只做局部微调）
//   · 判距用 dx*dx+dy*dy，风格对齐 pass.cpp 的 count_near_opponent
//   · 返回微调后的 y；威胁半径内无敌人则原样返回 by
// ============================================================
// 追球目标校验（docs/13 防守修复 F2）：平台预测(ball_pred)在进球/FreeBall/定位球
//   重置期会冻结在错误位置（真实 9/2 丢球 2：ACTIVE 追幻影 40 帧脱位、
//   4v5 被 demo 运动战破门）——预测点离实际球 >80cm 即视为不可信，改用实际球位。
BallState chase_target(const WorldModel &wm) {
    BallState t = wm.ball_pred;
    if (std::hypot(t.x - wm.ball.x, t.y - wm.ball.y) > 80.0) {
        t.x = wm.ball.x; t.y = wm.ball.y;
    }
    return t;
}

double spread_y(const WorldModel &wm, double bx, double by,
                double threat_radius, double max_offset) {
    double best_d2 = threat_radius * threat_radius;
    double nearest_dy = 0.0;
    bool found = false;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double dx = wm.opp[i].x - bx;
        double dy = wm.opp[i].y - by;
        double d2 = dx * dx + dy * dy;
        if (d2 < best_d2) {      // 半径内最近的敌人
            best_d2 = d2;
            nearest_dy = dy;
            found = true;
        }
    }
    if (!found) return by;
    // 越近偏移越大（d=0→max_offset，d=radius→0），朝敌人反方向横向躲
    double frac = 1.0 - std::sqrt(best_d2) / threat_radius;
    double sign = (nearest_dy >= 0.0) ? -1.0 : 1.0;   // 敌人在上 → 往下躲
    return by + sign * max_offset * clamp(frac, 0.0, 1.0);
}

}  // anonymous namespace

void run_goalie(WorldModel &wm, int id) {
    const TeamContext &ctx = wm.ctx;
    RobotState &r = wm.home[id];
    // 点球：守门员锁定球门线中央（不出小禁区，防调度失位）
    if (wm.game_state == PM_PenaltyKick_Blue || wm.game_state == PM_PenaltyKick_Yellow) {
        motion::position(r, ctx.our_goal_x() + ctx.attack_dir() * 3.0, 90.0);
        return;
    }

    // ============================================================
    // 可调参数（改动后记得同步 docs/06-调参记录.md）
    // ============================================================
    // 门前站位：球门中心前方 10cm；y 跟踪范围 [76,104]（门宽内侧留余量）
    const double kGuardDist = 10.0;
    const double kTrackYLo  = 76.0, kTrackYHi = 104.0;
    // 近距扑球阈值：
    //   kMaxTTA   ：球到门线时间上限(帧)，超过它=远期威胁，不扑
    //   kMaxReach ：守门员够球距离上限(cm)
    //   kMinSpeed ：球速下限(cm/帧)，太慢的球不用出击
    const double kMaxTTA   = 15.0;
    const double kMaxReach = 30.0;
    const double kMinSpeed = 5.0;
    // 二过一检测阈值：
    //   kDribbleSpeed：球速超过它视为「对方带球高速前突」
    //   kSupportDist ：接应者判定距离（离带球者够近 且 比带球者更靠门）
    const double kDribbleSpeed = 6.0;
    const double kSupportDist  = 60.0;
    // 出击深度阈值（第 1、2 条动态深度用）：
    //   kFastShotSpeed：球速达到它则前压到罚球区前缘（远射封角度）
    //   kOppPullback  ：罚球区内每个对方球员让出击深度回缩的距离（防埋伏回敲）
    const double kFastShotSpeed = 12.0;
    const double kOppPullback   = 20.0;
    // 解围阈值：
    //   kClearDist：球进到这个距离内，守门员主动解围（清球）
    //   kPushDist ：推球点离球距离（站在球后面推，跟 shoot.cpp 一致）
    //   kLateral  ：球夹在门将和门之间时，绕弧线的侧向偏移距离（防乌龙）
    const double kClearDist = 20.0;
    const double kPushDist  = 8.0;
    const double kLateral   = 15.0;

    double bx = wm.ball.x, by = wm.ball.y;
    double vx = wm.ball.vx, vy = wm.ball.vy;
    double danger = ball_danger_speed(wm);   // 球朝己方门的速度分量（横滚≈0、背离=0，才是真威胁）
    double db = dist(r.x, r.y, bx, by);      // 守门员到球的当前距离
    // 门球/定位球重启：球停在我方门前 → 门将主动沿中线穿过球把它推出去。
    //   否则球静止时门将只停在球后 8cm 或退到门线上，球被推/滚到门线外又触发门球，
    //   形成连续门球死循环（复盘实测一局循环 17+ 次）。
    // 门前抢球：球距门<45cm 且（球近静止 或 对方持球者离球<25cm 正在推/控）→ 门将
    //   主动冲球推穿。参考官方 demo Goalie 的 aggressive 出击（ball.x<12 直接压球，
    //   不看球速），自研实现。
    //   第15轮复盘修正：旧分支要求 spd<2——Y5 一碰球 spd>2，门将立即退回门线，
    //   球被推入（rlg f3304：B1 距球 8cm 却挡不住）。加"对方持球"条件后，
    //   对方碰球期间门将保持冲球姿态，不再中途放弃。
    bool ball_still = std::hypot(vx, vy) < 2.0;
    double opp_dmin_door = 1e9;
    if (!ball_still) {
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            double d = dist(bx, by, wm.opp[i].x, wm.opp[i].y);
            if (d < opp_dmin_door) opp_dmin_door = d;
        }
    }
    if (ctx.dist_our_goal(bx) < 45.0 && ball_still) {
        // 穿过球把球推出门区（直线推穿，与 run_active 射门同款）：
        //   已贴球(≤25cm) **且 y 与球对准(≤3cm)** → 目标=球前 20cm（场侧），直线穿过
        //   球把球推向场中央；未对准 → 先到球后（门侧）8cm 对准，下一帧再穿。
        //   旧版（8/31 14:18）直接 position 到球前 30cm：motion 弧线绕行碰不到球。
        //   第15轮修复：只判 dbg<25 就推穿仍会斜线绕球（rlg f3304：B1 在 (206,82)、
        //   球 (207,90) 距 8cm，目标 (185,90)，B1 斜线过去路径不经过球心，又没碰到球）。
        //   必须 y 对准（门将、球、目标三点同一直线）才可能直线穿球。
        double dbg = dist(r.x, r.y, bx, by);
        double aligned = std::fabs(r.y - by) <= 3.0;
        double px, py;
        if (dbg < 25.0 && aligned) {
            px = bx + ctx.attack_dir() * 20.0;
            py = clamp(by, 78.0, 102.0);
        } else {
            px = bx - ctx.attack_dir() * 8.0;
            py = clamp(by, 78.0, 102.0);
        }
        clamp_goalie_area(ctx, px, py);
        motion::position(r, px, py);
        return;
    }
    // 对方持球压门（球距门<45 且对方离球<25）→ 不冲球，封球-门连线：
    //   真机丢球复盘（12:03 场下角两球）：demo 高速带球到门前时，门将冲球
    //   推穿目标在球身上，demo 变向一推球就换侧进门；改为站在球与门心
    //   连线上、球前 12cm 深度处封角度（深度 ≤40cm 且不低于 kGuardDist——
    //   第15轮教训：对方碰球期间门将不能退到门线）。
    if (ctx.dist_our_goal(bx) < 45.0 && opp_dmin_door < 25.0) {
        double back = std::max(0.0, ctx.dist_our_goal(bx) - 12.0);
        double depth2 = std::min(40.0, std::max(kGuardDist, back));
        double px2 = ctx.our_goal_x() + ctx.attack_dir() * depth2;
        double py2 = 90.0 + (by - 90.0) * (back / ctx.dist_our_goal(bx));
        py2 = clamp(py2, 78.0, 102.0);
        clamp_goalie_area(ctx, px2, py2);
        motion::position(r, px2, py2);
        return;
    }
    // 门将站位深度：球贴近门线(<15cm)后撤贴门(3cm)，防球沿门线/身后滚过；否则 10cm 封角度
    double gx = ctx.our_goal_x() + ctx.attack_dir() *
                (std::fabs(bx - ctx.our_goal_x()) < 15.0 ? 3.0 : kGuardDist);

    // ============================================================
    // 基础预判：球会不会进球 + 多久到门
    // ============================================================
    double y_at_goal = 90.0;
    // 球会不会到达门线：predict_y_at_x 返回 false = 球背离门或只有 y 向运动
    bool heading_goal = predict_y_at_x(bx, by, vx, vy, ctx.our_goal_x(), y_at_goal);
    // 到达门线时 y 落在门宽内 → 这球会进球（不是偏出/打墙）
    bool on_target = heading_goal &&
                     y_at_goal >= goal_y_low() && y_at_goal <= goal_y_high();
    // 到门线时间 TTA（帧）：距离 / 朝门速度分量
    double tta = 1e9;
    if (std::fabs(vx) > 1e-9)
        tta = std::fabs(ctx.our_goal_x() - bx) / std::fabs(vx);

    // ============================================================
    // 带球者识别：离球最近的对方球员（球在谁脚下）
    // ============================================================
    int dribbler = -1;
    double dmin = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = dist(bx, by, wm.opp[i].x, wm.opp[i].y);
        if (d < dmin) { dmin = d; dribbler = i; }
    }
    bool opp_has_ball = (dmin < 15.0);
    // 慢速盘带压门标志：对方持球且已进门前 50cm（球速低时 danger 测不出，靠位置兜底）
    bool dribble_press = opp_has_ball && ctx.dist_our_goal(bx) < 50.0;

    // ============================================================
    // 二过一检测（第 3 条）：带球者高速前突时，侧前方是否有接应者。
    //   启发式：接应者 = 离带球者够近 + 比带球者更靠门的对方球员。
    //   可能误判，联调时用 rlg 复盘校准 kSupportDist。
    // ============================================================
    bool has_support = false;
    if (opp_has_ball && danger > kDribbleSpeed) {
        double dg_dribbler = ctx.dist_our_goal(wm.opp[dribbler].x);
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            if (i == dribbler) continue;
            double d = dist(wm.opp[dribbler].x, wm.opp[dribbler].y,
                            wm.opp[i].x, wm.opp[i].y);
            // 接应者：够近 + 比带球者更靠门（在带球者前方）
            if (d < kSupportDist && ctx.dist_our_goal(wm.opp[i].x) < dg_dribbler - 5.0) {
                has_support = true; break;
            }
        }
    }

    // ============================================================
    // 出击深度（动态，第 1、2 条核心）：不再固定压到罚球区前缘，
    // 而是按「球速」和「门前对方人数」动态算该出来多远：
    //   · 球越快 → 越该前压封角度（远射提前拦截）
    //   · 球越慢 → 越该留后贴门（慢球不用出那么远，避免失位/不回防）
    //   · 罚球区内对方越多 → 越该留后（防埋伏接应者一脚回敲打穿）
    //   深度在 [门前站位距离, 罚球区深度 80cm] 之间，最后 clamp 回罚球区。
    // ============================================================
    // 对方埋伏在罚球区（等横传）的人数
    int opp_in_box = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (in_penalty_area(ctx, wm.opp[i].x, wm.opp[i].y))
            ++opp_in_box;
    }
    // 朝门球速线性映射出击深度：kMinSpeed→贴门，kFastShotSpeed→罚球区前缘(80cm)
    double frac = clamp((danger - kMinSpeed) / (kFastShotSpeed - kMinSpeed), 0.0, 1.0);
    double depth = kGuardDist + frac * (80.0 - kGuardDist);
    // 门前有对方埋伏 → 回缩，别出那么远
    depth = std::max(kGuardDist, depth - opp_in_box * kOppPullback);
    // 慢速盘带压门(#1)：dribble_press 且球速低 → 前压深度改用持球人位置，
    //   始终站在球与门之间(球前 12cm)封角度，夹 [kGuardDist, 40]，不过度上抢。
    if (dribble_press && danger < kMinSpeed) {
        depth = std::min(40.0, std::max(kGuardDist, ctx.dist_our_goal(bx) - 12.0));
    }
    double out_x = ctx.our_goal_x() + ctx.attack_dir() * (std::fabs(bx - ctx.our_goal_x()) < 15.0 ? 3.0 : depth);

    // 拦截点 y：球运动轨迹在 out_x 竖线处的 y（封射门角度）
    double iy = 90.0;
    if (!predict_y_at_x(bx, by, vx, vy, out_x, iy)) {
        // 球速不可用/球已越过 out_x → 兜底用球-门连线与 out_x 交点
        if (std::fabs(bx - ctx.our_goal_x()) > 1e-6) {
            double t = (out_x - ctx.our_goal_x()) / (bx - ctx.our_goal_x());
            t = std::max(0.0, std::min(1.0, t));  // 球越过站位线时外推 t>1 → 截断到门线上
            iy = 90.0 + t * (by - 90.0);
        }
    }
    // 封堵上角（真机复盘：demo 多次打门柱内侧 y∈[100,105] 上角，门将按轨迹
    // 站位线拦截不够贴门）：预测球会进门且进门点比站位线点更靠上/下角时，
    // 改用进门点 y。
    if (on_target && std::fabs(y_at_goal - 90.0) > std::fabs(iy - 90.0)) {
        iy = y_at_goal;
    }
    clamp_goalie_area(ctx, out_x, iy);

    // ============================================================
    // 解围（最高优先级）：球在脚下很近时，主动把球清走，避免乌龙 + 清给对方。
    //   · 清球方向 = 往「最空」队友（离对方最近球员最远），且该队友须比球
    //     更远离己方球门 —— 保证不往门边清、也不往对方球员脚下清。
    //   · 推球点 = 球后方 kPushDist（站门侧推球，推球方向 = 清球方向）。
    //   · 防乌龙：球夹在门将和门之间时，直线去推球点会穿球顶进自家门，
    //     给推球点加侧向偏移，弧线绕到门侧再推。
    // ============================================================
    double clear_x = 0.0, clear_y = 0.0;
    bool clearing = false;
    if (db < kClearDist) {
        int best_id = -1;
        double best_open = -1e9;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            if (i == id) continue;                       // 跳过守门员自己
            double tx = wm.home[i].x, ty = wm.home[i].y;
            if (ctx.dist_our_goal(tx) < ctx.dist_our_goal(bx)) continue;  // 队友比球靠门，不选
            double min_opp = 1e9;
            for (int j = 0; j < PLAYERS_PER_SIDE; ++j)
                min_opp = std::min(min_opp, dist(tx, ty, wm.opp[j].x, wm.opp[j].y));
            // 边路解围偏好(#4)：沿边线(Y<10 或 >170)的队友即便稍近也优先——
            //   中路纵深传球线路长、易被对方中场断成单刀反击，边路踢出界风险低。
            double score = min_opp + ((ty < 10.0 || ty > 170.0) ? 30.0 : 0.0);
            if (score > best_open) { best_open = score; best_id = i; }
        }
        double dirx = 0.0, diry = 0.0;
        if (on_target) {
            // 球会进己方门（在门宽内）→ 沿“球-门连线”水平推向中场解围，
            //   门将始终贴住球-门连线，慢滚球不会从身侧漏过；
            //   推向空位队友会有 y 偏移，把球门让给慢球（复盘丢球根因）。
            dirx = ctx.attack_dir();   // 远离己门方向
            diry = 0.0;
        } else if (best_id >= 0) {
            dirx = wm.home[best_id].x - bx;
            diry = wm.home[best_id].y - by;
        } else {
            dirx = ctx.opp_goal_x() - bx;               // 兜底：往对方球门沿 x 清
            diry = 0.0;
        }
        double len = std::hypot(dirx, diry);
        if (len < 1e-6) { dirx = ctx.opp_goal_x() - bx; diry = 0.0; len = std::hypot(dirx, diry); }
        if (len < 1e-6) { dirx = 0.0; diry = 1.0; len = 1.0; }
        dirx /= len; diry /= len;

        clear_x = bx - dirx * kPushDist;                // 推球点：球后方（门侧）
        clear_y = by - diry * kPushDist;

        if (ctx.dist_our_goal(bx) < ctx.dist_our_goal(r.x)) {   // 球夹在门将和门之间 → 弧线绕
            double nx = -diry, ny = dirx;
            double side = (r.x - bx) * nx + (r.y - by) * ny;
            double s = (side >= 0.0) ? 1.0 : -1.0;
            clear_x += s * nx * kLateral;
            clear_y += s * ny * kLateral;
        }
        clamp_goalie_area(ctx, clear_x, clear_y);
        clearing = true;
    }

    // ============================================================
    // 决策（按优先级从高到低）
    // ============================================================
    if (clearing) {
        motion::position(r, clear_x, clear_y);
    } else if (has_support) {
        // 二过一威胁：不贸然前压（会被一脚直塞打穿），后退封门，
        //   站门前跟预测入球点，封住接应者可能的射门角度。
        motion::position(r, gx, clamp(y_at_goal, kTrackYLo, kTrackYHi));
    } else if (on_target && tta < kMaxTTA && db < kMaxReach && danger > kMinSpeed) {
        // 近距扑球：球会进球且马上到，扑向门线内侧预测点。
        double aim_x = ctx.our_goal_x() + ctx.attack_dir() * 3.0;   // 门线前 3cm
        double aim_y = clamp(y_at_goal, 74.0, 106.0);
        // 截球半径极限封堵(#2)：预测入球点横向偏差超过够球距离(reach)时，
        //   硬追球路两头都够不着(近角真空)，改赌近门柱——封球真正飞向的那一侧、放远角。
        if (std::fabs(y_at_goal - r.y) > kMaxReach) {
            aim_y = (y_at_goal < 90.0) ? 74.0 : 106.0;
        }
        motion::position(r, aim_x, aim_y);
    } else if (on_target || (opp_has_ball && danger > kMinSpeed) || dribble_press) {
        // 远射 / 带球威胁：按动态深度前压封角度（慢球贴门，快球到罚球区前缘）。
        motion::position(r, out_x, iy);
    } else {
        // 无威胁 / 球慢：常规门前站位，y 跟球（夹在门区内）。
        motion::position(r, gx, clamp(by, kTrackYLo, kTrackYHi));
    }
}

// 对方门区停留红线（docs/13 方案 C）：规则"门区除门将外停留 >20 周期 → 罚点球"。
//   纯停留限 8 帧：人在门区且（球不在门区 或 球不在脚下>25cm）= 明显滞留。
//   必须留足"撤出时间"：实测撤出 30~40cm 要 10~12 帧，红线 20 帧内必须离开；
//   限 10 帧时 11+12=23 仍越线（sim 实测 3.4 次/场），8+12=20 刚好卡线。
//   带球推射（球在门区且贴脚≤25cm）不累计，限 8 不影响攻门。
//   总时长兜底 20 帧：带球反复推不进（真实平台 8/29 实测 21~30 帧被判 3 点球）
//   也不能无限续命，规则红线就是 20，宁可放弃机会不送点球。
// —— 补射跟进（docs/17 讲解稿规划）：射门被挡后球在门前低速反弹 → 追进去补一脚 ——
//   判据：球在门前锥形区内且球速低于可抢阈值 + 我方就近（距离限制保证追进门区
//   途中 4~8 帧内贴球，"纯停留"帧计数不超限）；贴球(≤25cm)后 active_ga_frames
//   清零（攻门作业不累计），走带球/推射链（plan_shoot 每帧重评，球距门<70cm 即射）；
//   追不到/超时由 kActiveGaLimit 撤出分支兜底，不会赖在门区送判罚。
constexpr double kReboundRushSpeed = 8.0;   // cm/帧：反弹球可抢速度阈值（GK扑出/挡回典型 <10）
constexpr double kReboundRushDist  = 90.0;  // cm：我方距球超过此值不冲（就近补，防全场狂奔）
static const int kActiveGaLimit  = 8;
static const int kActiveGaTotal  = 18;  // 在门区总时长兜底：平台 20 周期判罚红线，留 2 帧余量
                                        // （8/29 实测被判滞留 21~30 帧；太紧会打断合法带球攻门 10~15 帧）

void run_active(WorldModel &wm, int id) {
    RobotState &r = wm.home[id];
    const TeamContext &ctx = wm.ctx;

    // 对方门区停留计数（docs/13 方案 C）：每帧更新，离开门区清零。
    //   口径：人在门区，若 球不在门区 或 球不在脚下(>25cm) → 计"纯停留"。
    //   —— 球在门区且贴着(≤25cm) = 正在攻门作业（直线推射 10 帧内完成），不累计；
    //   —— 其余（含追球沿边线穿过门区——实测违例主因：球沿左边线上行，ACTIVE
    //       直线追球正好切穿门区 30+ 帧）一律累计，超限撤出。
    //   总时长 active_ga_total 兜底：带球在门区反复推不进（真实平台 8/29 实测
    //   21~30 帧被判 3 点球）也不能无限续命。
    if (in_opp_goal_area(ctx, r.x, r.y)) {
        ++wm.active_ga_total;
        bool ball_in_ga = in_opp_goal_area(ctx, wm.ball.x, wm.ball.y);
        if (!ball_in_ga || dist(r.x, r.y, wm.ball.x, wm.ball.y) > 25.0) ++wm.active_ga_frames;
    } else {
        wm.active_ga_frames = 0;
        wm.active_ga_total = 0;
    }

    // 角区卡球计时：球在角区(距角 <30cm)且基本静止(速度<1cm/帧) → 连续帧数+1；
    //   否则清零。（>30 帧才认为"真卡住"：路过/刚弹到角的球不算，避免救球喧宾夺主。）
    if ((wm.ball.x < 30.0 || wm.ball.x > 190.0) &&
        (wm.ball.y < 30.0 || wm.ball.y > 150.0) &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 1.0) {
        ++wm.corner_ball_frames;
    } else {
        wm.corner_ball_frames = 0;
    }

    // 对方门球/定位球重启：球停死在对方门区(球门前 50cm) → 别冲进去抢。
    //   球是死球，冲进去射门/追球会横穿全场撞进对方门区，冲撞对方门将(门区受保护)
    //   或在门区停留犯规；实测自博弈每局门球 ACTIVE 都从己方半场直冲对方门区。
    //   等球被开出(球速起来)再正常进攻；先站对方罚球区外沿外 5cm 等反击。
    //   ⚠️ 例外：我方主罚点球（in_penalty_exec，strategy.cpp 维护）——球同样静止在
    //   对方罚球点，但必须去踢！真实平台 8/29 点球 0/3 全丢的根因就是这里拦截：
    //   逐帧复盘 ACTIVE 助跑后只停在门区外沿 (85,90)，从不执行射门。
    if (!wm.in_penalty_exec &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 1.0 &&
        std::fabs(wm.ball.x - ctx.opp_goal_x()) < 50.0 &&
        std::fabs(wm.ball.y - 90.0) < 20.0) {
        // 等待超时：对方迟迟不开球（球卡死/对方无人处理）→ 主动去推/射，把球带出门区。
        //   真实 8/31 镜像内战 0:0 根因：球静止对方门区 (205,90) 135 秒无人处理，
        //   进攻方死球等待永不超时、门将清球又不穿过球 → 双方僵持。
        //   正常门球对方 2 秒内会来开（球动起来）→ 100 帧超时不会误伤。
        if (++wm.dead_ball_frames > 100) {
            wm.dead_ball_frames = 0;
            // 不 return —— 落到 plan_shoot/带球逻辑：进门区作业（方案 C 攻门豁免），
            // 直线推穿把球推出对方门区或直接射门
        } else {
            double hold_x = ctx.opp_goal_x() - ctx.attack_dir() * 85.0;
            double hold_y = clamp(wm.ball.y, 72.5, 107.5);
            motion::position(r, hold_x, hold_y);
            return;
        }
    } else {
        wm.dead_ball_frames = 0;
    }

    // —— 对方门区停留时限（docs/13 方案 C，放在射门之前）——
    // 规则：门区除门将外单人停留 >20 周期 → 罚点球。真实平台 8/29 实测：
    // ACTIVE 带球在门区滞留 21~30 帧即被判 3 个点球（0/3 全丢）。
    // 必须在射门前检查：否则"射门优先"会让 ACTIVE 一直站在门区推球，
    // 停留帧数无限累积（sim 实测单人>20帧 1.9 次/场降不下去）。
    // 超限后不恋战：传球给禁区外沿接应，无传球则撤到门区前缘外（x=60cm 线，
    //   出区即清零，球留在门区由对方清掉，也比送点球划算）。
    if (wm.active_ga_frames > kActiveGaLimit || wm.active_ga_total > kActiveGaTotal) {
        ++wm.ga_retreat_fires;   // 诊断用（sim_bench 验证超限撤出触发）
        PassPlan pp_ga = plan_pass(wm, id);
        if (pp_ga.viable) { motion::position(r, pp_ga.target_x, pp_ga.target_y); return; }
        double ogx = ctx.opp_goal_x(), ad = ctx.attack_dir();
        motion::position(r, ogx - ad * 60.0, clamp(wm.ball.y, 72.5, 107.5));
        return;
    }

    // —— 射门：直线推穿（治真实平台"带球射门系统性偏下"）——
    // 原单点推球：高速冲到球后 8cm 推球点时还边转边铲，推球方向 = 接近轨迹
    //   方向（被出发点带偏），球被斜推偏出（实测球 y 90→65 偏出门柱，助跑 175cm 太长）。
    // 现改为"贴球后直线推穿"：已贴在球后(≤22cm)且朝向大致对准(≤40°)时，
    //   目标=球前 20cm，直线加速穿过球，推球方向=瞄准线，方向不再被带偏；
    //   还远/在侧面时先绕到球后沿瞄准线的站位点（球后 20cm）对准再推。
    ShootPlan sp = plan_shoot(wm, id);
    if (sp.viable) {
        double bx = wm.ball.x, by = wm.ball.y;
        double db = dist(r.x, r.y, bx, by);
        double te_ball = angle_diff(angle_to(r.x, r.y, bx, by), r.rot);
        if (db < 22.0 && std::fabs(te_ball) < 40.0) {
            motion::position(r, bx + sp.dir_x * 20.0, by + sp.dir_y * 20.0);
        } else {
            motion::position(r, bx - sp.dir_x * 20.0, by - sp.dir_y * 20.0);
        }
        return;
    }

    // —— 角区救球（治"FreeBall 13 次/场"：球卡四角无人救 → 判争球）——
    // 仅救"角区外环"(距角 22~30cm)的卡球：该环在平台禁止推球区(四角黄区)之外，
    // 推球合法；球压到角心(距角 <22cm)则**不救**——规则"禁止推球区推球 =
    // 犯规(每4次+1球)+判争球"，深角球等平台判僵局重置，比送犯规划算。
    // 救球执行：贴近时直线穿过球推向场心方向，让球离开墙角继续比赛；
    // 还远/在侧面时先绕到球后(角落侧)对准再推。
    if (wm.corner_ball_frames > 30) {
        double db = dist(r.x, r.y, wm.ball.x, wm.ball.y);
        if (db < 80.0) {
            bool deep = (wm.ball.x < 22.0 || wm.ball.x > 198.0) &&
                        (wm.ball.y < 22.0 || wm.ball.y > 158.0);
            if (!deep) {
                ++wm.corner_rescue_events;   // 统计用（sim_bench 验证救球触发）
                double ex = 110.0 - wm.ball.x, ey = 90.0 - wm.ball.y;
                double elen = std::hypot(ex, ey);
                if (elen > 1e-6) { ex /= elen; ey /= elen; }
                double te_ball = angle_diff(angle_to(r.x, r.y, wm.ball.x, wm.ball.y), r.rot);
                if (db < 22.0 && std::fabs(te_ball) < 40.0) {
                    motion::position(r, wm.ball.x + ex * 30.0, wm.ball.y + ey * 30.0);
                } else {
                    motion::position(r, wm.ball.x - ex * 8.0, wm.ball.y - ey * 8.0);
                }
                return;
            }
        }
    }

    PassPlan pp = plan_pass(wm, id);
    if (pp.viable) { motion::position(r, pp.target_x, pp.target_y); return; }

    double db = dist(r.x, r.y, wm.ball.x, wm.ball.y);
    // 找离球最近的对方防守者（含守门员）：决定带球开口侧 + 判断球权是否在我
    double opp_d = 1e9, opp_y = 90.0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y);
        if (d < opp_d) { opp_d = d; opp_y = wm.opp[i].y; }
    }

    if (db < 15.0 && db < opp_d) {
        // —— 围困检测（治"带球有进无退"）——
        // 球周围 25cm 内 ≥2 个 demo，或最近 demo <12cm → 不硬带：
        //   优先回传/横传（此时 assist 已在禁区外沿近端接应），无传球选择则带球离场。
        int swarm = 0, near_i = -1;
        double opp_near = 1e9;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            double d = dist(wm.opp[i].x, wm.opp[i].y, wm.ball.x, wm.ball.y);
            if (d < 25.0) swarm++;
            if (d < opp_near) { opp_near = d; near_i = i; }
        }
        if (swarm >= 2 || opp_near < 12.0) {
            PassPlan pp2 = plan_pass(wm, id);
            if (pp2.viable) { motion::position(r, pp2.target_x, pp2.target_y); return; }
            // 无传球选择：把球带离最近的围困者（向空档方向推，直线推穿不逗留）
            if (near_i >= 0 && opp_near > 1e-6) {
                double dx = wm.ball.x - wm.opp[near_i].x;
                double dy = wm.ball.y - wm.opp[near_i].y;
                double len = std::hypot(dx, dy);
                if (len > 1e-6) { dx /= len; dy /= len; }
                double te_e = angle_diff(angle_to(r.x, r.y, wm.ball.x, wm.ball.y), r.rot);
                if (db < 12.0 && std::fabs(te_e) < 40.0) {
                    motion::position(r, wm.ball.x + dx * 25.0, wm.ball.y + dy * 25.0);
                } else {
                    motion::position(r, wm.ball.x - dx * 20.0, wm.ball.y - dy * 20.0);
                }
                return;
            }
        }
        // —— 正常带球推进：球在我方脚下控制范围（我方是离球最近的人）——
        // 目标方向指向门柱开口（开口选「离最近防守者远」的一侧，避免直线撞进防守怀里）。
        // 执行方式与射门同款"直线推穿"：已贴在球后(≤12cm)且朝向对准(≤40°)时
        //   目标=球前 20cm 直线穿过球——sim 的 carry 在穿过瞬间生效，不会像原来
        //   停在球后 8cm 推球点那样死锁（死锁 → 球静止 → 判僵局 FreeBall 重置，
        //   sim 实测 158 次/场的主因之一）；还远/在侧面时先绕到球后 20cm 站位点对准。
        double ogx = ctx.opp_goal_x();
        double aim_y = (opp_y > 90.0) ? 106.0 : 74.0;   // 防守者偏下 → 带上柱口
        double dirx = (ogx + ctx.attack_dir() * 5.0) - wm.ball.x;
        double diry = aim_y - wm.ball.y;
        double len = std::hypot(dirx, diry);
        if (len > 1e-6) { dirx /= len; diry /= len; }
        double te_d = angle_diff(angle_to(r.x, r.y, wm.ball.x, wm.ball.y), r.rot);
        if (db < 12.0 && std::fabs(te_d) < 40.0) {
            motion::position(r, wm.ball.x + dirx * 20.0, wm.ball.y + diry * 20.0);
        } else {
            motion::position(r, wm.ball.x - dirx * 20.0, wm.ball.y - diry * 20.0);
        }
    } else {
        // 球不在脚下 / 争抢中：追预测球位（带减速防冲过头）
        // 对方罚球区防线（docs/13 攻击强化补充）：预测位在对方罚球区内
        //   （对手射门被挡/门将扑救后球滞留门区）→ 追到罚球区外沿等球弹出，
        //   不冲进门区——sim 实测 ACTIVE 追球穿门区 + MID 站位 = 2+ 人违规
        //   16 帧/场（debug f9109：球已弹到 x=59 球速 22，两人还滞留门区 13 帧）。
        BallState chased = chase_target(wm);
        // 门前 100cm 锥形区（dist_opp_goal<100 且 |y-90|<45）：直线追球路径必穿门区
        //   （sim debug：球射偏出门线 y=41，ACTIVE 追球穿门区 16 帧 = 2+ 人违规主因），
        //   追到门区外沿等球弹出；射门分支优先不受影响。
        if (wm.ctx.dist_opp_goal(chased.x) < 100.0 && std::fabs(chased.y - 90.0) < 45.0) {
            // 补射跟进：球在门前低速且我方就近 → 豁免 clamp、追进球区补射
            //   （球被 GK 扑出/防守者挡回的反弹是第二落点机会，错过了就白射；
            //   球高速滚向门 / 距球太远时不豁免——追不过去、也不值得长途进场）。
            double bspeed = std::hypot(wm.ball.vx, wm.ball.vy);
            bool rebound_rush = (bspeed < kReboundRushSpeed) &&
                                dist(r.x, r.y, chased.x, chased.y) < kReboundRushDist;
            if (!rebound_rush) {
                chased.x = wm.ctx.opp_goal_x() - wm.ctx.attack_dir() * 85.0;
                chased.y = clamp(chased.y, 72.5, 107.5);
            }
        }
        // docs/15 P0-4：追球避障——直线被对方挡才绕行（plan_route 2r 邻域裁剪
        //   后开销约全图 1/10）；decel=true 保留接近减速防冲过头。
        move_avoiding(wm, r, id, chased.x, chased.y, true);
    }
}

void run_passive(WorldModel &wm, int id) {
    // —— 门前协防（docs/03 第14轮，参考官方 demo CenterDefender 球-门连线思想，自研实现）——
    // 根因（真 vs demo 2:7×2、0:6 复盘）：demo 把球控停我方门前 (205,90) 静止 1.5~2.2s，
    //   其追击手 Y5 从 100cm 外高速直冲抢点推射；原防守 double_team 球进罚球区
    //   return false（禁区纪律）、盯人站位被罚球区纪律推出 → 门前只剩门将 1v1。
    // 对策：球在我方门前 80cm 内、近静止、且对方已逼近(<100cm，正来抢/控这颗球)时，
    //   本角色钉到「球-门连线」护门点（球远站球后45、球近站球前8cm 堵推射线，见
    //   defense.cpp goal_cover_point）——正好卡在对方从弧顶/中场直冲门前的路径上，
    //   与门将形成双人包夹。对方离球远（我方门球/清球场景）不触发，不拉离防区。
    //   阈值 60→100（2026-08-31 第15轮）：rlg 复盘 B5 从后场到门前需 40+ 帧，
    //   Y5 从 60cm 冲球仅 ~24 帧——60cm 触发永远晚到；100cm 提前出发才有拦截余量。
    if (wm.ctx.dist_our_goal(wm.ball.x) < 80.0 &&
        std::hypot(wm.ball.vx, wm.ball.vy) < 3.0) {
        double opp_dmin = 1e9;
        for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
            double d = dist(wm.ball.x, wm.ball.y, wm.opp[i].x, wm.opp[i].y);
            if (d < opp_dmin) opp_dmin = d;
        }
        if (opp_dmin < 100.0) {
            // —— 门线球站定防乌龙（docs/06 第37轮）——
            // 真机 2 乌龙铁证（9/7 23:40 场 @124.8 门角 y35、@138.6 门线中央）：
            //   B4 护门点站位每帧随威胁 position 微调，而 B4 贴球(6-9cm) → 移动
            //   时把球拖着走（球 y110→101 随 B4 y106→96 = 拖进门内）。
            // 对策：球贴我方门线(<15cm)且本角色贴球(<14cm) → 停轮站定，用身体
            //   封 demo 推射角，绝不碰球不拖动（demo 推球用身体挡，不主动清）。
            double dbp = dist(wm.ball.x, wm.ball.y, wm.home[id].x, wm.home[id].y);
            if (wm.ctx.dist_our_goal(wm.ball.x) < 15.0 && dbp < 14.0) {
                wm.home[id].vl = 0.0;
                wm.home[id].vr = 0.0;
                return;
            }
            double cx = 0.0, cy = 0.0;
            goal_cover_point(wm, cx, cy);
            motion::position(wm.home[id], cx, cy);
            return;
        }
    }
    // 人盯人：威胁高时，盯住威胁最大的对方球员，站在他与己方球门之间。
    //   威胁分已计入球速（接球威胁 approach + 持球突破 danger，见 defense.cpp mark_threat），
    //   球越快越该贴住危险的进攻点。
    if (wm.threat_level >= 0.6) {
        int t = pick_mark_target(wm, wm.mark_target);
        wm.mark_target = t;
        if (t >= 0) {
            // 对方门区聚集犯规防护（C 模块方案，余量并入 A/B 15cm 调参）：
            //   被盯者缩在对方门区且球不在门区时不追进去（门区 2+ 人 / 停留>20帧 → 罚点球），
            //   改站对方门区前缘外 15cm 等反击。
            if (in_opp_goal_area(wm.ctx, wm.opp[t].x, wm.opp[t].y) &&
                !in_opp_goal_area(wm.ctx, wm.ball.x, wm.ball.y)) {
                double fx = 0.0, fy = 0.0;
                opp_goal_area_front(wm.ctx, wm.ball.y, fx, fy);
                motion::position(wm.home[id], fx, fy);
                return;
            }
            // E2 盯人转逼抢（docs/13 攻击强化）：被盯者离球 <25cm（控球/即将接球）时
            //   放弃站连线、直接冲球贴身逼抢（抢下或破坏对手组织，配合 E1 双人夹抢）。
            double d_opp_ball = dist(wm.ball.x, wm.ball.y, wm.opp[t].x, wm.opp[t].y);
            // 球在我方门区内不逼抢：门前交给门将（否则与门将 2+ 人违规，
            //   sim 实测 E2 首版 2+人 0.5→2.3 次/场），站门区外等解围。
            if (d_opp_ball < 25.0 && !in_goal_area(wm.ctx, wm.ball.x, wm.ball.y)) {
                motion::chase_ball(wm.home[id], chase_target(wm));
                return;
            }
            // 站位：速度前馈预测被盯者未来位置（改「追着跑」为「截击」，
            //   同速追逐追不上移动目标），站到「被盯者→球门」连线上、离其 mark_dist 处，
            //   封住他的传/射路线。
            double gx = wm.ctx.our_goal_x(), gy = 90.0;
            double px = wm.opp[t].x + wm.opp_vx[t] * mark_lead();
            double py = wm.opp[t].y + wm.opp_vy[t] * mark_lead();
            // 站位方向选择：
            //   · 被盯者是接球者（非持球者）且离球够近 → 传球随时发生，
            //     站「球→被盯者」连线上（堵传球线，提前掐断传球）；
            //   · 否则（持球者 / 离球远）→ 站「被盯者→球门」连线上（封射门/再传）。
            double d_ball = dist(wm.ball.x, wm.ball.y, wm.opp[t].x, wm.opp[t].y);
            double ref_x = gx, ref_y = gy;                       // 默认 goal-side
            if (d_ball > 15.0 && d_ball < mark_pass_lane_dist()) {
                ref_x = wm.ball.x; ref_y = wm.ball.y;            // 堵传球线
            }
            double dx = ref_x - px, dy = ref_y - py;
            double len = std::hypot(dx, dy);
            if (len > 1e-6) { dx /= len; dy /= len; }
            double mx = px + dx * mark_dist();
            double my = py + dy * mark_dist();
            // 站位点若落入己方罚球区（只有门将能进）→ 推到罚球区前缘 5cm
            if (in_penalty_area(wm.ctx, mx, my)) {
                mx = wm.ctx.our_goal_x() + wm.ctx.attack_dir() * 85.0;
                my = clamp(py, 72.5, 107.5);
            }
            mx = clamp(mx, 0.0, TeamContext::FIELD_LENGTH);
            my = clamp(my, 0.0, TeamContext::FIELD_WIDTH);
            // 对方门区禁入（docs/13 方案 A）：盯人站位不得进入对方门区（防 2+ 人违规判点球）
            clamp_out_opp_goal_area(wm.ctx, mx, my);
            motion::position(wm.home[id], mx, my);
            return;
        }
    }
    DefensePlan dp = plan_defense(wm, id);
    motion::position(wm.home[id], dp.target_x, dp.target_y);
}

void run_assist(WorldModel &wm, int id) {
    // 威胁高：回防但分散站位（封上侧射门线，不与 passive 挤一点）
    // 反击快攻窗口（docs/13 方案 A）：断球后 counter_attack_frames>0 时豁免回防、立即前插接应，
    //   让 ACTIVE 断球有传球选择（治反击前场真空，真实 9/2 vs demo 0 射门威胁）。
    if ((wm.threat_level > 0.3 || wm.no_possession_frames >= 2) && wm.counter_attack_frames <= 0) {
        // E1 双人逼抢（docs/13 攻击强化）：球在对方半场且对手控球时，本角色放弃
        //   防守站位压上追球（与 ACTIVE 双人夹抢，抢下即反击，治"1 人追球断球率低"）。
        //   球进对方罚球区不追（禁区纪律，非门将不进对方禁区）；离球 >130cm 不追（防失位）。
        double bbx = wm.ball.x;
        bool ball_opp_half = (wm.ctx.attack_dir() > 0) ? (bbx > 110.0) : (bbx < 110.0);
        // 球在门前 100cm 锥形区不追（直线追球路径穿门区 = 2+ 人违规，同 run_active）
        if (ball_opp_half && !wm.we_have_ball &&
            !(wm.ctx.dist_opp_goal(bbx) < 100.0 && std::fabs(wm.ball.y - 90.0) < 45.0) &&
            dist(wm.home[id].x, wm.home[id].y, bbx, wm.ball.y) < 130.0) {
            // 追球目标同样夹在门前锥形区外（预测位可能滚进门区，直接追会穿门区）
            double pxx = wm.ball_pred.x, pyy = wm.ball_pred.y;
            if (wm.ctx.dist_opp_goal(pxx) < 100.0 && std::fabs(pyy - 90.0) < 45.0) {
                pxx = wm.ctx.opp_goal_x() - wm.ctx.attack_dir() * 85.0;
                pyy = clamp(pyy, 72.5, 107.5);
            }
            motion::position(wm.home[id], pxx, pyy);
            return;
        }

        // 清道夫(远侧覆盖)：球在防守三区拉边时，本角色被 strategy.cpp 指派为清道夫，
        //   钉中路封远门柱/横传（触发与站位见 update_sweeper）。
        if (id == wm.sweeper_id) {
            motion::position(wm.home[id], wm.sweeper_x, wm.sweeper_y);
            return;
        }
        // 对方射门在门框内且够快 → 抢上侧反弹位（防补射）
        if (shot_on_target(wm) && ball_danger_speed(wm) > rebound_min_danger()) {
            double rx = 0.0, ry = 0.0;
            rebound_point(wm, +30.0, rx, ry);
            motion::position(wm.home[id], rx, ry);
            return;
        }
        // 二抢一：持球者带球推进到门前危险区时，补一个防守者上前与 passive 包夹
        double dtx = 0.0, dty = 0.0;
        if (double_team_point(wm, id, dtx, dty)) {
            motion::position(wm.home[id], dtx, dty);
            return;
        }
        DefensePlan dp = plan_defense(wm, id);
        double ty = clamp(dp.target_y + 30.0, 20.0, 160.0);
        motion::position(wm.home[id], dp.target_x, ty);
        return;
    }
    // —— 进攻分支：站 A 的助攻点；先躲敌人，再和队友 Y 轴互相推开 ——
    constexpr double THREAT_RADIUS = 30.0;   // 敌方威胁检测半径 cm
    constexpr double MAX_OFFSET     = 20.0;   // 最大允许横向偏移 cm
    constexpr double FIELD_MARGIN   = 6.0;    // 目标点离边线最小距离 cm
    constexpr double TEAM_SPACING   = 25.0;   // 与中场的最小 Y 间距（防挤堆）
    double tx = wm.assist_x;                  // X 保持 A 原始输出
    double ty = spread_y(wm, wm.assist_x, wm.assist_y, THREAT_RADIUS, MAX_OFFSET);   // ① 躲敌人（优先级高）
    // ② 队友推开：离中场站位点 Y 太近时，朝远离方向错开，X 不动
    if (fabs(ty - wm.mid_y) < TEAM_SPACING) {
        ty = wm.mid_y + ((ty >= wm.mid_y) ? TEAM_SPACING : -TEAM_SPACING);
    }
    tx = clamp(tx, FIELD_MARGIN, TeamContext::FIELD_LENGTH - FIELD_MARGIN);
    ty = clamp(ty, FIELD_MARGIN, TeamContext::FIELD_WIDTH  - FIELD_MARGIN);
    // 对方门区禁入（docs/13 方案 A）：助攻站位不得进入对方门区（防 2+ 人违规判点球）
    clamp_out_opp_goal_area(wm.ctx, tx, ty);
    motion::position(wm.home[id], tx, ty);
}

void run_midfield(WorldModel &wm, int id) {
    // 威胁高：回防但分散站位（封下侧射门线）
    // 反击快攻窗口（docs/13 方案 A）：断球后 counter_attack_frames>0 时豁免回防、立即前插接应，
    //   让 ACTIVE 断球有传球选择（治反击前场真空，真实 9/2 vs demo 0 射门威胁）。
    if ((wm.threat_level > 0.3 || wm.no_possession_frames >= 2) && wm.counter_attack_frames <= 0) {
        // 清道夫(远侧覆盖)：球在防守三区拉边时，本角色被 strategy.cpp 指派为清道夫，
        //   钉中路封远门柱/横传（触发与站位见 update_sweeper）。
        if (id == wm.sweeper_id) {
            motion::position(wm.home[id], wm.sweeper_x, wm.sweeper_y);
            return;
        }
        // 对方射门在门框内且够快 → 抢下侧反弹位（防补射）
        if (shot_on_target(wm) && ball_danger_speed(wm) > rebound_min_danger()) {
            double rx = 0.0, ry = 0.0;
            rebound_point(wm, -30.0, rx, ry);
            motion::position(wm.home[id], rx, ry);
            return;
        }
        // 二抢一：持球者带球推进到门前危险区时，补一个防守者上前与 passive 包夹
        double dtx = 0.0, dty = 0.0;
        if (double_team_point(wm, id, dtx, dty)) {
            motion::position(wm.home[id], dtx, dty);
            return;
        }
        // 球在对方半场（对手解围/球权争夺中）：不去对方门前截球——球-门连线
        //   截点落对方门前/门区，会把 MID 拉进门区（sim debug f9109 实测
        //   MID 滞留门区 13 帧、与 ACTIVE 2+ 人违规 2.2 次/场），站中线等球弹回。
        double bbx2 = wm.ball.x;
        bool ball_opp_half2 = (wm.ctx.attack_dir() > 0) ? (bbx2 > 110.0) : (bbx2 < 110.0);
        if (ball_opp_half2) {
            motion::position(wm.home[id], 110.0, clamp(wm.home[id].y, 20.0, 160.0));
            return;
        }
        DefensePlan dp = plan_defense(wm, id);
        double ty = clamp(dp.target_y - 30.0, 20.0, 160.0);
        motion::position(wm.home[id], dp.target_x, ty);
        return;
    }
    // —— 进攻分支：站 A 的中场点；先躲敌人，再和队友 Y 轴互相推开 ——
    constexpr double THREAT_RADIUS = 30.0;   // 敌方威胁检测半径 cm
    constexpr double MAX_OFFSET     = 20.0;   // 最大允许横向偏移 cm
    constexpr double FIELD_MARGIN   = 6.0;    // 目标点离边线最小距离 cm
    constexpr double TEAM_SPACING   = 25.0;   // 与助攻的最小 Y 间距（防挤堆）
    double tx = wm.mid_x;                     // X 保持 A 原始输出
    double ty = spread_y(wm, wm.mid_x, wm.mid_y, THREAT_RADIUS, MAX_OFFSET);   // ① 躲敌人（优先级高）
    // ② 队友推开：离助攻站位点 Y 太近时，朝远离方向错开，X 不动
    if (fabs(ty - wm.assist_y) < TEAM_SPACING) {
        ty = wm.assist_y + ((ty >= wm.assist_y) ? TEAM_SPACING : -TEAM_SPACING);
    }
    tx = clamp(tx, FIELD_MARGIN, TeamContext::FIELD_LENGTH - FIELD_MARGIN);
    ty = clamp(ty, FIELD_MARGIN, TeamContext::FIELD_WIDTH  - FIELD_MARGIN);
    // 对方门区禁入（docs/13 方案 A）：中场站位不得进入对方门区（防 2+ 人违规判点球）
    clamp_out_opp_goal_area(wm.ctx, tx, ty);
    motion::position(wm.home[id], tx, ty);
}

}  // namespace simuro5
