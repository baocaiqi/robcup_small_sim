// team.hpp — 队伍上下文：一套逻辑蓝/黄镜像；场地 220×180cm、球门宽 40cm，蓝守右门(x=220)、黄守左门(x=0)
#include <cmath>

#ifndef SIMURO5_TEAM_HPP
#define SIMURO5_TEAM_HPP

namespace simuro5 {

struct TeamContext {
    bool is_blue = true;

    static constexpr double FIELD_LENGTH = 220.0;
    static constexpr double FIELD_WIDTH  = 180.0;
    static constexpr double GOAL_WIDTH   = 40.0;

    double our_goal_x() const { return is_blue ? FIELD_LENGTH : 0.0; }
    double opp_goal_x() const { return is_blue ? 0.0 : FIELD_LENGTH; }
    double attack_dir() const { return is_blue ? -1.0 : +1.0; }

    double dist_our_goal(double x) const { return fabs(x - our_goal_x()); }
    double dist_opp_goal(double x) const { return fabs(x - opp_goal_x()); }
};

}  // namespace simuro5
#endif
