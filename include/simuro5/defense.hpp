// ============================================================
// defense.hpp — 区域防守
// 移植自中型组 defense_module 思想：
//   站到「球-己方球门连线」的拦截点（situation 的 passive_pt），
//   面向球，球逼近时前压。
// ============================================================
#ifndef SIMURO5_DEFENSE_HPP
#define SIMURO5_DEFENSE_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

struct DefensePlan {
    double target_x = 0, target_y = 90;   // 拦截站位点
};

// 计算 2 号防守队员的拦截点（基于球-门连线）
DefensePlan plan_defense(const WorldModel &wm);

}  // namespace simuro5
#endif
