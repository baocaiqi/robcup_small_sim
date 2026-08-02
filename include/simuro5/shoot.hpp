// ============================================================
// shoot.hpp — 射门决策
// 移植自中型组 shoot_module 思想（瞄准开口+避开守门员）。
// 注意：FIRA 5v5 无独立踢球动作，射门 = 带球冲向对方球门。
// 本模块只输出「射门路径上的目标点」，由角色壳驱动 motion 执行。
// ============================================================
#ifndef SIMURO5_SHOOT_HPP
#define SIMURO5_SHOOT_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

struct ShootPlan {
    bool viable = false;
    double target_x = 0, target_y = 90;   // 带球目标点（球门开口侧前方）
    double aim_y = 90;                    // 瞄准的开口 y（调试用）
};

// 计算 4 号前锋的最佳射门方案
ShootPlan plan_shoot(const WorldModel &wm, int shooter_id);

}  // namespace simuro5
#endif
