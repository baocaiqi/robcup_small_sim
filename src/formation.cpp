#include "simuro5/formation.hpp"
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
            if (c.is_blue) { put(robots, 0, M(c,215), 90, goalie_rot(c)); put(robots, 1, M(c,kFrontLineX), 100, field_rot(c)); put(robots, 2, M(c,170), 65, field_rot(c)); put(robots, 3, M(c,150), 40, field_rot(c)); put(robots, 4, M(c,130), 130, field_rot(c)); }
            break;
        case PM_GoalKick_Yellow:
            if (!c.is_blue) { put(robots, 0, M(c,215), 90, goalie_rot(c)); put(robots, 1, M(c,kFrontLineX), 100, field_rot(c)); put(robots, 2, M(c,170), 65, field_rot(c)); put(robots, 3, M(c,150), 40, field_rot(c)); put(robots, 4, M(c,130), 130, field_rot(c)); }
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
