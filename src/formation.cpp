#include "simuro5/formation.hpp"
#include "simuro5/geometry.hpp"
#define TUNABLE_PREFIX "form."
#include "simuro5/tunable.hpp"
#include <cmath>

namespace simuro5 {

namespace {

// 蓝队坐标 → 黄队坐标（水平镜像，镜像轴 x=110）；朝向：门将面场内、队员面进攻方向
inline double M(const TeamContext &c, double x) { return c.is_blue ? x : 220.0 - x; }
inline double goalie_rot(const TeamContext &c) { return c.is_blue ? -90.0 : 90.0; }
inline double field_rot(const TeamContext &c)  { return c.is_blue ? 180.0 : 0.0; }

inline void put(Robot *r, int i, double x, double y, double rot) {
    r[i].pos.x = x; r[i].pos.y = y; r[i].rotation = rot;
}

// 争球点估计（1/4 场中心，对应四个 FreeBall 状态）
double estimate_freeball_x(int gs) { return (gs == PM_FreeBall_RightTop || gs == PM_FreeBall_RightBot) ? 165.0 : 55.0; }
double estimate_freeball_y(int gs) { return (gs == PM_FreeBall_LeftTop || gs == PM_FreeBall_RightTop) ? 135.0 : 45.0; }

// 门区纪律站位线 165：己方门区(门前50 × 门宽±15)内除 0 号门将任何人不许摆（规则 7.10.1/7.10.2/7.17）
constexpr double kFrontLineX = 165.0;

// 争球 1/4 区纪律（规则 7.15）：入参为绝对坐标，区内或贴中线(|y-90|≤5cm)都要推到另一半
constexpr double kFbQuarterClear = 5.0;
void keep_out_of_fb_quarter(double &x, double &y, bool fb_right, bool fb_top) {
    if ((x > 110.0) != fb_right) return;
    const bool in_quarter = (y > 90.0) == fb_top;
    const bool straddling = std::fabs(y - 90.0) <= kFbQuarterClear;
    if (!in_quarter && !straddling) return;
    y = fb_top ? 90.0 - kFbQuarterClear : 90.0 + kFbQuarterClear;
}


// ═══ 我方门球：门将摆位时机头就对准"门将→球"那条线（2026-10-14 晚）═══
//   真机取证（今天 4 场 rlg，黄位 7 次门球）：平台把门将摆在 (5.1,90.1) 机头 90°（我们给的 goalie_rot），
//   球在 (14.7,77.7)，"门将→球"方向 ≈ −52°；快发球要先原地转 ~38°（车尾对准）。真机原地转起步很慢
//   （命令 ±70 → 前 9 帧只转 20°，还往后溜 7cm），实测出球 35~45 帧 + 再 10~25 帧等球静止，
//   离线闭环的 9~17 帧在真机上没兑现。摆位时直接把机头摆到这条线上 = 一帧不用转，开球就冲。
//   参考对手门球"门将摆好就直接斜着顶穿"思想，自研实现（对手门将摆位机头 180° 是观察值）。
//   kGkKickFaceBall=0 回滚成老的 goalie_rot。球位用真机观察值（平台会把 SetBall 的 (10,78) 挪到 (14.7,77.7)）。
TUNABLE(kGkKickFaceBall, 1.0);
TUNABLE(kGkKickBallDx, 14.7);   // 门球球位离己门线（观察值）
TUNABLE(kGkKickBallY, 77.7);
TUNABLE(kGkKickGkY, 90.1);      // 平台摆好后门将 y（观察值）
double goalkick_goalie_rot(const TeamContext &c) {
    if (kGkKickFaceBall < 0.5) return goalie_rot(c);
    const double gx = M(c, 215), bx = c.our_goal_x() + c.attack_dir() * kGkKickBallDx;
    return angle_to(gx, kGkKickGkY, bx, kGkKickBallY);
}

// ═══ 我方门球站位（2026-10-14 晚，参考对手"门球前队友已分散站好接应位"思想，自研实现）═══
//   对手自己门球时 4 人站 (190,70)(190,115)(150,40)(130,130)（蓝位坐标，观察值，不是我们照抄的值）。
//   我们的快发球固定把球斜着捅向球所在那侧边墙（球 y=77.7 → 下边墙，方向约 −52°，
//   直线在 x≈75 撞墙）。老站位 3 号 (70,40) 在这条线场内侧 ~15cm，球滚到 y≈25 时它还得横插；
//   真机今天 22 次门球里 8 次 4 秒内丢球，形状都是"球慢慢滚出去 → 对手先到 → 顶回门"。
//   新站位：3 号直接站到出球线前方靠墙（kGkKickRecvX/Y），1 号收到门前中路偏上挡回传路线，
//   2/4 号不动。kGkKickNewShape=0 回滚老站位。还没真机验证。
TUNABLE(kGkKickNewShape, 1.0);
TUNABLE(kGkKickRecvX, 150.0);   // 3 号接应点（蓝位坐标，x=150 ⇔ 黄位 70）
TUNABLE(kGkKickRecvY, 25.0);
TUNABLE(kGkKickGuardX, 175.0);  // 1 号门前护位（蓝位坐标，x=175 ⇔ 黄位 45）
TUNABLE(kGkKickGuardY, 100.0);
void goalkick_formation(const TeamContext &c, Robot *r) {
    put(r, 0, M(c, 215), 90, goalkick_goalie_rot(c));
    if (kGkKickNewShape < 0.5) {
        put(r, 1, M(c, kFrontLineX), 100, field_rot(c));
        put(r, 3, M(c, 150), 40, field_rot(c));
    } else {
        put(r, 1, M(c, kGkKickGuardX), kGkKickGuardY, field_rot(c));
        put(r, 3, M(c, kGkKickRecvX), kGkKickRecvY, field_rot(c));
    }
    put(r, 2, M(c, 170), 65, field_rot(c));
    put(r, 4, M(c, 130), 130, field_rot(c));
}

void defense_formation(const TeamContext &c, Robot *r) {
    put(r, 0, M(c, 215), 90, goalie_rot(c));
    put(r, 1, M(c, kFrontLineX), 90, field_rot(c));
    put(r, 2, M(c, 150), 90, field_rot(c));
    put(r, 3, M(c, 130), 60, field_rot(c));
    put(r, 4, M(c, 130), 120, field_rot(c));
}

// 进攻开球阵型：ACTIVE(id1) 站球后 10cm（原放 id4 导致发球没人踢）
void kickoff_formation(const TeamContext &c, Robot *r) {
    put(r, 0, M(c, 215), 90, goalie_rot(c));
    put(r, 1, M(c, 120), 90, field_rot(c));   // ACTIVE：球(110)后 10cm（球后=远离被攻球门侧）
    put(r, 2, M(c, 150), 60, field_rot(c));
    put(r, 3, M(c, 150), 120, field_rot(c));
    put(r, 4, M(c, kFrontLineX), 90, field_rot(c));
}

void freeball_formation(const TeamContext &c, Robot *r, double bx, double by) {
    put(r, 0, M(c, 215), 90, goalie_rot(c));
    const bool fb_right = bx > 110.0;
    const bool fb_top   = by > 90.0;
    double dx = fb_right ? 25.0 : -25.0;
    put(r, 1, bx + dx, by, field_rot(c));
    const double xs[3] = {150.0, 150.0, kFrontLineX};
    const double ys[3] = { 60.0, 120.0, 90.0};
    for (int k = 0; k < 3; ++k) {
        double ax = M(c, xs[k]), ay = ys[k];
        keep_out_of_fb_quarter(ax, ay, fb_right, fb_top);
        put(r, 2 + k, ax, ay, field_rot(c));
    }
}

}  // namespace

void formation_former(const TeamContext &c, PlayMode gs, Robot robots[]) {
    switch (gs) {
        // 开球：进攻方先摆
        case PM_PlaceKick_Blue:
        case PM_PlaceKick_Yellow:
            kickoff_formation(c, robots);
            break;

        // 争球：先摆方按 1/4 区中心估计球位
        case PM_FreeBall_LeftTop:
        case PM_FreeBall_LeftBot:
        case PM_FreeBall_RightTop:
        case PM_FreeBall_RightBot:
            freeball_formation(c, robots, estimate_freeball_x(gs), estimate_freeball_y(gs));
            break;

        // 点球：防守方先摆；平台约定 PM_PenaltyKick_X = X 队主罚（不是"X 队被罚"）
        case PM_PenaltyKick_Blue:
            if (!c.is_blue) { put(robots, 0, M(c,215), 90, goalie_rot(c)); put(robots, 1, M(c,130), 60, field_rot(c)); put(robots, 2, M(c,130), 120, field_rot(c)); put(robots, 3, M(c,150), 90, field_rot(c)); put(robots, 4, M(c,kFrontLineX), 90, field_rot(c)); }
            break;
        case PM_PenaltyKick_Yellow:
            if (c.is_blue) { put(robots, 0, M(c,215), 90, goalie_rot(c)); put(robots, 1, M(c,130), 60, field_rot(c)); put(robots, 2, M(c,130), 120, field_rot(c)); put(robots, 3, M(c,150), 90, field_rot(c)); put(robots, 4, M(c,kFrontLineX), 90, field_rot(c)); }
            break;

        // 任意球：进攻方先摆（罚球人 ACTIVE 球后 10cm，其余己方半场）
        case PM_FreeKick_Blue:
            if (c.is_blue) {
                put(robots, 0, M(c,215), 90, goalie_rot(c));
                put(robots, 1, M(c,65), 90, field_rot(c));
                put(robots, 2, M(c,150), 60, field_rot(c));
                put(robots, 3, M(c,150), 120, field_rot(c));
                put(robots, 4, M(c,kFrontLineX), 90, field_rot(c));
            }
            break;
        case PM_FreeKick_Yellow:
            if (!c.is_blue) {
                put(robots, 0, M(c,215), 90, goalie_rot(c));
                put(robots, 1, M(c,155), 90, field_rot(c));
                put(robots, 2, M(c,150), 60, field_rot(c));
                put(robots, 3, M(c,150), 120, field_rot(c));
                put(robots, 4, M(c,kFrontLineX), 90, field_rot(c));
            }
            break;

        case PM_GoalKick_Blue:
            if (c.is_blue) goalkick_formation(c, robots);
            break;
        case PM_GoalKick_Yellow:
            if (!c.is_blue) goalkick_formation(c, robots);
            break;

        default:
            break;
    }
}

void formation_later(const TeamContext &c, PlayMode gs,
                     Robot formerRobots[], Vector3D ball, Robot laterRobots[]) {
    (void)formerRobots;
    switch (gs) {
        case PM_PlaceKick_Blue:
        case PM_PlaceKick_Yellow:
            defense_formation(c, laterRobots);
            break;

        case PM_FreeBall_LeftTop:
        case PM_FreeBall_LeftBot:
        case PM_FreeBall_RightTop:
        case PM_FreeBall_RightBot:
            freeball_formation(c, laterRobots, ball.x, ball.y);
            break;

        case PM_PenaltyKick_Blue:
        case PM_PenaltyKick_Yellow: {
            bool we_take = (c.is_blue && gs == PM_PenaltyKick_Blue) ||
                           (!c.is_blue && gs == PM_PenaltyKick_Yellow);
            if (!we_take) { defense_formation(c, laterRobots); break; }
            put(laterRobots, 0, M(c,215), 90, goalie_rot(c));
            put(laterRobots, 2, M(c,150), 60, field_rot(c));
            put(laterRobots, 3, M(c,150), 120, field_rot(c));
            put(laterRobots, 4, M(c,kFrontLineX), 90, field_rot(c));
            put(laterRobots, 1, ball.x - c.attack_dir() * 10.0, ball.y, field_rot(c));
            break;
        }

        case PM_FreeKick_Blue:
        case PM_FreeKick_Yellow:
            defense_formation(c, laterRobots);
            break;

        case PM_GoalKick_Blue:
        case PM_GoalKick_Yellow:
            defense_formation(c, laterRobots);
            break;

        default:
            break;
    }
}

void formation_set_ball(const TeamContext &c, PlayMode gs, Vector3D *pBall) {
    if ((gs == PM_GoalKick_Blue && c.is_blue) ||
        (gs == PM_GoalKick_Yellow && !c.is_blue)) {
        pBall->x = c.our_goal_x() + c.attack_dir() * 10.0;
        // 落点 y=78：太靠中路会被断球后正对球门；下限受门柱线 70 与门将站位下限 74 约束
        pBall->y = 78.0;
        pBall->z = 0.0;
    }
}

}  // namespace simuro5
