#include "simuro5/defense.hpp"
#include "simuro5/field_info.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

DefensePlan plan_defense(const WorldModel &wm) {
    DefensePlan plan;
    const TeamContext &ctx = wm.ctx;
    plan.target_x = wm.passive_x;
    plan.target_y = wm.passive_y;
    return plan;
}

}  // namespace simuro5
