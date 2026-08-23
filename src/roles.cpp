#include "simuro5/roles.hpp"
#include "simuro5/role_assignment.hpp"
#include "simuro5/motion.hpp"
#include "simuro5/shoot.hpp"
#include "simuro5/pass.hpp"
#include "simuro5/defense.hpp"
#include "simuro5/field_info.hpp"
#include <cmath>

namespace simuro5 {

namespace {
// ============================================================
// 无球接应拉开（队员C）：站位点附近有敌方机器人时，沿 Y 轴横向躲开。
//   · 只微调 Y，保留 A 输出的原始 X（全局跑位点由 situation.cpp 决定，这里只做局部微调）
//   · 判距用 dx*dx+dy*dy，风格对齐 pass.cpp 的 count_near_opponent
//   · 返回微调后的 y；威胁半径内无敌人则原样返回 by
// ============================================================
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

    double gx = ctx.our_goal_x() + ctx.attack_dir() * kGuardDist;
    double bx = wm.ball.x, by = wm.ball.y;
    double vx = wm.ball.vx, vy = wm.ball.vy;
    double speed = ball_speed(vx, vy);
    double db = dist(r.x, r.y, bx, by);      // 守门员到球的当前距离

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

    // ============================================================
    // 二过一检测（第 3 条）：带球者高速前突时，侧前方是否有接应者。
    //   启发式：接应者 = 离带球者够近 + 比带球者更靠门的对方球员。
    //   可能误判，联调时用 rlg 复盘校准 kSupportDist。
    // ============================================================
    bool has_support = false;
    if (opp_has_ball && speed > kDribbleSpeed) {
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
    // 球速线性映射出击深度：kMinSpeed→贴门，kFastShotSpeed→罚球区前缘(80cm)
    double frac = clamp((speed - kMinSpeed) / (kFastShotSpeed - kMinSpeed), 0.0, 1.0);
    double depth = kGuardDist + frac * (80.0 - kGuardDist);
    // 门前有对方埋伏 → 回缩，别出那么远
    depth = std::max(kGuardDist, depth - opp_in_box * kOppPullback);
    double out_x = ctx.our_goal_x() + ctx.attack_dir() * depth;

    // 拦截点 y：球运动轨迹在 out_x 竖线处的 y（封射门角度）
    double iy = 90.0;
    if (!predict_y_at_x(bx, by, vx, vy, out_x, iy)) {
        // 球速不可用/球已越过 out_x → 兜底用球-门连线与 out_x 交点
        if (std::fabs(bx - ctx.our_goal_x()) > 1e-6) {
            double t = (out_x - ctx.our_goal_x()) / (bx - ctx.our_goal_x());
            iy = 90.0 + t * (by - 90.0);
        }
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
            if (min_opp > best_open) { best_open = min_opp; best_id = i; }
        }
        double dirx = 0.0, diry = 0.0;
        if (best_id >= 0) {
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
    } else if (on_target && tta < kMaxTTA && db < kMaxReach && speed > kMinSpeed) {
        // 近距扑球：球会进球且马上到，扑向门线内侧预测点。
        double aim_x = ctx.our_goal_x() + ctx.attack_dir() * 3.0;   // 门线前 3cm
        motion::position(r, aim_x, clamp(y_at_goal, 74.0, 106.0));
    } else if (on_target || (opp_has_ball && speed > kMinSpeed)) {
        // 远射 / 带球威胁：按动态深度前压封角度（慢球贴门，快球到罚球区前缘）。
        motion::position(r, out_x, iy);
    } else {
        // 无威胁 / 球慢：常规门前站位，y 跟球（夹在门区内）。
        motion::position(r, gx, clamp(by, kTrackYLo, kTrackYHi));
    }
}

void run_active(WorldModel &wm, int id) {
    RobotState &r = wm.home[id];
    const TeamContext &ctx = wm.ctx;

    ShootPlan sp = plan_shoot(wm, id);
    if (sp.viable) { motion::position(r, sp.target_x, sp.target_y); return; }

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
            // 无传球选择：把球带离最近的围困者（向空档方向推）
            if (near_i >= 0 && opp_near > 1e-6) {
                double dx = wm.ball.x - wm.opp[near_i].x;
                double dy = wm.ball.y - wm.opp[near_i].y;
                double len = std::hypot(dx, dy);
                if (len > 1e-6) { dx /= len; dy /= len; }
                motion::position(r, wm.ball.x - dx * 8.0, wm.ball.y - dy * 8.0);
                return;
            }
        }
        // —— 正常带球推进：球在我方脚下控制范围（我方是离球最近的人）——
        // 目标 = 球后方 8cm 推球点，方向指向门柱开口；
        // 开口选「离最近防守者远」的一侧，避免直线撞进防守怀里。
        double ogx = ctx.opp_goal_x();
        double aim_y = (opp_y > 90.0) ? 106.0 : 74.0;   // 防守者偏下 → 带上柱口
        double dirx = (ogx + ctx.attack_dir() * 5.0) - wm.ball.x;
        double diry = aim_y - wm.ball.y;
        double len = std::hypot(dirx, diry);
        if (len > 1e-6) { dirx /= len; diry /= len; }
        // 球后方 8cm 推球点（和 shoot 同款推球机制）
        motion::position(r, wm.ball.x - dirx * 8.0, wm.ball.y - diry * 8.0);
    } else {
        // 球不在脚下 / 争抢中：追预测球位（带减速防冲过头）
        motion::chase_ball(r, wm.ball_pred);
    }
}

void run_passive(WorldModel &wm, int id) {
    DefensePlan dp = plan_defense(wm, id);
    motion::position(wm.home[id], dp.target_x, dp.target_y);
}

void run_assist(WorldModel &wm, int id) {
    // 威胁高：回防但分散站位（封上侧射门线，不与 passive 挤一点）
    if (wm.threat_level > 0.3) {
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
    motion::position(wm.home[id], tx, ty);
}

void run_midfield(WorldModel &wm, int id) {
    // 威胁高：回防但分散站位（封下侧射门线）
    if (wm.threat_level > 0.3) {
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
    motion::position(wm.home[id], tx, ty);
}

}  // namespace simuro5
