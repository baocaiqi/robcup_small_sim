// ============================================================
// shoot.hpp — 射门决策
// 射门决策（瞄准开口+避开守门员）。
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
    double aim_y = 90;                    // 瞄准的开口 y（调试/变角推射分侧用）
    double dir_x = 0, dir_y = 1;          // 推球方向（球→开口的单位向量，两段式推射站位用）
    // —— docs/18 §8：机会质量（70~110cm 动态远射区用；≤70cm 维持无条件可射）——
    double aim_rot = 0.0;        // 瞄准方向角(度)：position_aligned 用它做末端朝向
    double shot_dist = 0.0;      // 球到对方门线距离 cm
    double open_angle = 0.0;     // 净开口角(度)：门张角扣掉门将遮挡后的最大连续空隙
    bool   lane_blocked = false; // 球→开口 30cm 路线是否被非门将防守者挡住
    bool   penalty = false;      // 点球执行期（旁路闸门：白送的射门机会不能被距离/开口拒掉）
    double quality = 0.0;        // 机会质量 ∈[0,1]（开口/距离/球速加权）
};

// 计算 4 号前锋的最佳射门方案
ShootPlan plan_shoot(const WorldModel &wm, int shooter_id);

}  // namespace simuro5
#endif
