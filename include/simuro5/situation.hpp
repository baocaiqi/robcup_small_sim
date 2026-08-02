// ============================================================
// situation.hpp — 局势分析与站位参考点
// 移植自中型组 situation_module.cpp（数值按 220×180 场地重标定）
// ============================================================
#ifndef SIMURO5_SITUATION_HPP
#define SIMURO5_SITUATION_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

struct Situation {
    bool we_have_ball = false;   // 球权是否在我方（简版：最近的人持球）
    bool ball_in_our_half = true;
    bool ball_in_our_penalty = false;
    bool ball_in_opp_penalty = false;
    double threat_level = 0.0;   // 0~1 威胁等级

    // 站位参考点
    double passive_x = 0, passive_y = 90;   // 防守站位
    double assist_x = 0,  assist_y = 90;    // 助攻站位
    double mid_x = 110,   mid_y = 90;       // 中场站位
};

class SituationModule {
public:
    Situation analyze(const WorldModel &wm);
    void update_stand_points(WorldModel &wm);
};

}  // namespace simuro5
#endif
