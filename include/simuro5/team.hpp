// ============================================================
// team.hpp — 队伍上下文（蓝/黄镜像参数化）
//
// 设计：内核只写一套逻辑，蓝黄通过本结构区分。
// 平台给双方同一个绝对坐标系：蓝队守右门(x=220)、黄队守左门(x=0)。
// ============================================================
#include <cmath>

#ifndef SIMURO5_TEAM_HPP
#define SIMURO5_TEAM_HPP

namespace simuro5 {

struct TeamContext {
    bool is_blue = true;

    // 场地与球门常量（cm，来自规则 PDF）
    static constexpr double FIELD_LENGTH = 220.0;   // x 方向
    static constexpr double FIELD_WIDTH  = 180.0;   // y 方向
    static constexpr double GOAL_WIDTH   = 40.0;    // 球门宽(y 方向)
    static constexpr double CENTER_RADIUS = 25.0;   // 中圈半径

    double our_goal_x() const { return is_blue ? FIELD_LENGTH : 0.0; }  // 己方球门线 x
    double opp_goal_x() const { return is_blue ? 0.0 : FIELD_LENGTH; }  // 对方球门线 x
    double attack_dir() const { return is_blue ? -1.0 : +1.0; }         // 进攻方向(+x 或 -x)

    // 距己方球门线的距离
    double dist_our_goal(double x) const { return fabs(x - our_goal_x()); }
    // 距对方球门线的距离
    double dist_opp_goal(double x) const { return fabs(x - opp_goal_x()); }
};

}  // namespace simuro5
#endif
