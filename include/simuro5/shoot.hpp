// ============================================================
// shoot.hpp — 射门决策（docs/15 P0-3：机会质量驱动的动态射程）
//
// 注意：FIRA 5v5 无独立踢球动作，射门 = 带球冲向对方球门。
// 本模块只输出「射门路径上的目标点 + 机会质量」，由角色壳驱动 motion 执行。
//
// 角度遮挡几何模型（自研，参考通用射门开口理论）：
//   · 门柱对球位张角  → 门内可瞄准的角区间；
//   · 对方守门员(GK) 对球位构成「遮挡角」= 2·atan(Rgk / dist)（出击越近遮挡越大）；
//   · 有效开口角 = 门张角 \ 遮挡角 的最大连续空隙；
//   · 射门路线 30cm 段被 GK 之外防守者挡住（圆盘碰撞）→ lane_blocked，不硬射。
// 动态射程：不是一刀切 70cm，而是「开口 ≥ 8° 且 距门 ≤ 110cm 且 路线通」才可射；
//   quality ∈ [0,1] 供上层分档：≥0.45 立即射 / 0.25~0.45 带球逼近 / <0.25 传或绕。
// ============================================================
#ifndef SIMURO5_SHOOT_HPP
#define SIMURO5_SHOOT_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

struct ShootPlan {
    bool viable = false;
    double target_x = 0, target_y = 90;   // 带球目标点（球后 8cm 推球点，两段式推射站位用）
    double aim_y = 90;                    // 瞄准的开口中心（连续值，调试/参考用）
    double dir_x = 0, dir_y = 1;          // 推球方向（球→最大净开口中心，单位向量）
    double quality = 0;                   // 机会质量 0~1（docs/15 §2.3 公式）
    double open_angle = 0;                // 有效开口角（度，>0 表示 GK 没封死）
    bool lane_blocked = false;            // 射门路线 30cm 段被 GK 之外的防守者挡住
    double shot_dist = 0;                 // 球→门线距离 cm
};

// 计算持球者的最佳射门方案（含机会质量与线路检测）
ShootPlan plan_shoot(const WorldModel &wm, int shooter_id);

}  // namespace simuro5
#endif