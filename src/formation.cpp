#include "simuro5/formation.hpp"
#include <cmath>

namespace simuro5 {

namespace {

// 蓝队坐标 → 黄队坐标（水平镜像，镜像轴 x=110）
inline double M(const TeamContext &c, double x) { return c.is_blue ? x : 220.0 - x; }
// 朝向：蓝队守门员面场内(-x)=-90，队员面进攻方向=180；黄队镜像(90/0)
inline double goalie_rot(const TeamContext &c) { return c.is_blue ? -90.0 : 90.0; }
inline double field_rot(const TeamContext &c)  { return c.is_blue ? 180.0 : 0.0; }

inline void put(Robot *r, int i, double x, double y, double rot) {
    r[i].pos.x = x; r[i].pos.y = y; r[i].rotation = rot;
}

// 争球点估计（1/4 场中心）：LeftTop/LeftBot/RightTop/RightBot
double estimate_freeball_x(int gs) { return (gs == PM_FreeBall_RightTop || gs == PM_FreeBall_RightBot) ? 165.0 : 55.0; }
double estimate_freeball_y(int gs) { return (gs == PM_FreeBall_LeftTop || gs == PM_FreeBall_RightTop) ? 135.0 : 45.0; }

// 默认防守阵型（蓝队坐标）：守门员门前 + 己方半场散开
void defense_formation(const TeamContext &c, Robot *r) {
    put(r, 0, M(c, 215), 90, goalie_rot(c));
    put(r, 1, M(c, 190), 90, field_rot(c));
    put(r, 2, M(c, 150), 90, field_rot(c));
    put(r, 3, M(c, 130), 60, field_rot(c));
    put(r, 4, M(c, 130), 120, field_rot(c));
}

// 进攻开球阵型（蓝队坐标）：守门员门前 + ACTIVE(id1) 站球后 10cm 负责开球推进
// 修复：原罚球人放 id4(PASSIVE 只防守不追球) → 发球没人踢；改 id1(ACTIVE) 带球推进天然覆盖
void kickoff_formation(const TeamContext &c, Robot *r) {
    put(r, 0, M(c, 215), 90, goalie_rot(c));
    put(r, 1, M(c, 120), 90, field_rot(c));   // ACTIVE：球(110)后 10cm（球后=远离被攻球门那侧）
    put(r, 2, M(c, 150), 60, field_rot(c));
    put(r, 3, M(c, 150), 120, field_rot(c));
    put(r, 4, M(c, 185), 90, field_rot(c));
}

// 争球摆位：1 人球侧 25cm，其余人散开（1/4 区外）
void freeball_formation(const TeamContext &c, Robot *r, double bx, double by) {
    put(r, 0, M(c, 215), 90, goalie_rot(c));
    // 争球人：球沿场地纵向 25cm 处
    double dx = (bx > 110.0) ? 25.0 : -25.0;   // 球在右半场→往左 25cm，反之往右
    put(r, 1, bx + dx, by, field_rot(c));
    put(r, 2, M(c, 150), 60, field_rot(c));
    put(r, 3, M(c, 150), 120, field_rot(c));
    put(r, 4, M(c, 185), 90, field_rot(c));
}

}  // namespace

void formation_former(const TeamContext &c, PlayMode gs, Robot robots[]) {
    switch (gs) {
        // 开球：进攻方先摆
        case PM_PlaceKick_Blue:
        case PM_PlaceKick_Yellow:
            kickoff_formation(c, robots);
            break;

        // 争球：先摆方（用 1/4 区中心估计球位）
        case PM_FreeBall_LeftTop:
        case PM_FreeBall_LeftBot:
        case PM_FreeBall_RightTop:
        case PM_FreeBall_RightBot:
            freeball_formation(c, robots, estimate_freeball_x(gs), estimate_freeball_y(gs));
            break;

        // 点球：防守方先摆
        //   ⚠️ 平台约定 PM_PenaltyKick_X = **X 队主罚**（不是"X 队被罚"）。证据：
        //   官方 demo 的 SetBall 只在 PM_GoalKick_Yellow 时把球放黄队门区，而 demo 是黄队；
        //   demo 的 SetLaterRobots case 7=PM_PenaltyKick_Yellow 摆的是黄队自己主罚的阵型、
        //   case 8=PM_PenaltyKick_Blue 才是黄队防守。所以"状态名是对方"时才轮到我们先摆。
        case PM_PenaltyKick_Blue:   // 蓝队主罚 → 只有黄队是防守方，黄队先摆
            if (!c.is_blue) { put(robots, 0, M(c,215), 90, goalie_rot(c)); put(robots, 1, M(c,130), 60, field_rot(c)); put(robots, 2, M(c,130), 120, field_rot(c)); put(robots, 3, M(c,150), 90, field_rot(c)); put(robots, 4, M(c,180), 90, field_rot(c)); }
            break;
        case PM_PenaltyKick_Yellow: // 黄队主罚 → 我们(蓝)是防守方，先摆
            if (c.is_blue) { put(robots, 0, M(c,215), 90, goalie_rot(c)); put(robots, 1, M(c,130), 60, field_rot(c)); put(robots, 2, M(c,130), 120, field_rot(c)); put(robots, 3, M(c,150), 90, field_rot(c)); put(robots, 4, M(c,180), 90, field_rot(c)); }
            break;

        // 任意球：进攻方先摆（罚球人=ACTIVE(id1) 球后 10cm，其他人己方半场）
        case PM_FreeKick_Blue:
            if (c.is_blue) {
                // FK 点在对方半场中部附近估计 (55,90)；罚球人在球后(离球门远侧)
                put(robots, 0, M(c,215), 90, goalie_rot(c));
                put(robots, 1, M(c,65), 90, field_rot(c));    // ACTIVE：球(55)后 10cm
                put(robots, 2, M(c,150), 60, field_rot(c));
                put(robots, 3, M(c,150), 120, field_rot(c));
                put(robots, 4, M(c,185), 90, field_rot(c));
            }
            break;
        case PM_FreeKick_Yellow:
            if (!c.is_blue) {
                put(robots, 0, M(c,215), 90, goalie_rot(c));
                put(robots, 1, M(c,155), 90, field_rot(c));   // ACTIVE：球(165)后 10cm
                put(robots, 2, M(c,150), 60, field_rot(c));
                put(robots, 3, M(c,150), 120, field_rot(c));
                put(robots, 4, M(c,185), 90, field_rot(c));
            }
            break;

        // 门球：发球方先摆（守门员门区，队友门区外）
        case PM_GoalKick_Blue:
            if (c.is_blue) { put(robots, 0, M(c,215), 90, goalie_rot(c)); put(robots, 1, M(c,190), 100, field_rot(c)); put(robots, 2, M(c,170), 65, field_rot(c)); put(robots, 3, M(c,150), 40, field_rot(c)); put(robots, 4, M(c,130), 130, field_rot(c)); }
            break;
        case PM_GoalKick_Yellow:
            if (!c.is_blue) { put(robots, 0, M(c,215), 90, goalie_rot(c)); put(robots, 1, M(c,190), 100, field_rot(c)); put(robots, 2, M(c,170), 65, field_rot(c)); put(robots, 3, M(c,150), 40, field_rot(c)); put(robots, 4, M(c,130), 130, field_rot(c)); }
            break;

        default:
            break;   // PlayOn 或与我无关：保持平台默认
    }
}

void formation_later(const TeamContext &c, PlayMode gs,
                     Robot formerRobots[], Vector3D ball, Robot laterRobots[]) {
    (void)formerRobots;
    switch (gs) {
        // 开球：防守方后摆（自己半场除中圈）
        case PM_PlaceKick_Blue:
        case PM_PlaceKick_Yellow:
            defense_formation(c, laterRobots);
            break;

        // 争球：后摆方（平台给了球位）
        case PM_FreeBall_LeftTop:
        case PM_FreeBall_LeftBot:
        case PM_FreeBall_RightTop:
        case PM_FreeBall_RightBot:
            freeball_formation(c, laterRobots, ball.x, ball.y);
            break;

        // 点球：后摆（我们主罚时摆进攻阵；对方主罚时我们已先摆过，这里只兜底防守阵）
        case PM_PenaltyKick_Blue:
        case PM_PenaltyKick_Yellow: {
            bool we_take = (c.is_blue && gs == PM_PenaltyKick_Blue) ||
                           (!c.is_blue && gs == PM_PenaltyKick_Yellow);
            if (!we_take) { defense_formation(c, laterRobots); break; }
            put(laterRobots, 0, M(c,215), 90, goalie_rot(c));
            put(laterRobots, 2, M(c,150), 60, field_rot(c));
            put(laterRobots, 3, M(c,150), 120, field_rot(c));
            put(laterRobots, 4, M(c,180), 90, field_rot(c));
            // 罚球人：ACTIVE(id1) 站在"球后"10cm —— 球后 = 远离被攻球门那一侧
            //   （平台 HELP 原文 "The kicker shall be placed behind the ball"）。
            //   2026-09-12 真机：原式 ball.x+dir*10（dir 按球在哪个半场定）站到了球门前侧，
            //   结果 4 次点球一次没射门，反而把球顶回自己半场（rlg 帧 4087-4119，球从
            //   39.4 被推到 105 且时速 130cm/s）。
            put(laterRobots, 1, ball.x - c.attack_dir() * 10.0, ball.y, field_rot(c));
            break;
        }

        // 任意球：防守方后摆
        case PM_FreeKick_Blue:
        case PM_FreeKick_Yellow:
            defense_formation(c, laterRobots);
            break;

        // 门球：对方发门球后摆
        case PM_GoalKick_Blue:
        case PM_GoalKick_Yellow:
            defense_formation(c, laterRobots);
            break;

        default:
            break;
    }
}

void formation_set_ball(const TeamContext &c, PlayMode gs, Vector3D *pBall) {
    // 发门球：球放门区内（球门前 10cm）
    if ((gs == PM_GoalKick_Blue && c.is_blue) ||
        (gs == PM_GoalKick_Yellow && !c.is_blue)) {
        pBall->x = c.our_goal_x() + c.attack_dir() * 10.0;
        pBall->y = 90.0;
        pBall->z = 0.0;
    }
}

}  // namespace simuro5
