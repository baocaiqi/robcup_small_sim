// situation.hpp — 局势分析与站位参考点
#ifndef SIMURO5_SITUATION_HPP
#define SIMURO5_SITUATION_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

struct Situation {
    Possession possession = Possession::Unknown;
    bool we_have_ball = false;
    bool whos_mismatch = false;  // 平台球权与自算不一致（标定用）
    bool ball_in_our_half = true;
    bool ball_in_our_penalty = false;
    bool ball_in_opp_penalty = false;
    double threat_level = 0.0;

    double passive_x = 0, passive_y = 90;
    double assist_x = 0,  assist_y = 90;
    double mid_x = 110,   mid_y = 90;
};

class SituationModule {
public:
    // 分析球权/半场/禁区/威胁
    Situation analyze(const WorldModel &wm);
    // 更新站位参考点
    void update_stand_points(WorldModel &wm);
};

}  // namespace simuro5
#endif
