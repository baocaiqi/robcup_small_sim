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
    // —— 借墙射门（bank shot，docs/06 第 65 轮，用户 2026-09-14 指令）——
    //   平台没有踢球动作，射门 = 沿 dir 推穿；借墙只是把 dir 换成「球→墙面反弹点」方向，
    //   所以执行体（roles.cpp）不需要改。
    bool   bank = false;         // 本方案是借墙反射，不是直线射门
    double bank_wall = 0.0;      // 借哪面墙：0 = 底墙 y=0，180 = 顶墙 y=180
    double bounce_x = 0.0;       // 反弹点的 x（y 就是 bank_wall）
    double bank_quality = 0.0;   // 借墙机会质量（遮挡/距离/反射点/球速 四项加权）
    double path_len = 0.0;       // 总路程 |球→反弹点| + |反弹点→目标|
};

// 计算 4 号前锋的最佳射门方案
ShootPlan plan_shoot(const WorldModel &wm, int shooter_id);

// 蜂群推进用的借墙方案（docs/06 第 79 轮）：只算借墙，射程放宽到 kBankCarryMax，
//   不与直线比较、不计统计；推进者据此决定"往哪面墙推"。viable=false → 没有可用反弹线。
ShootPlan plan_bank_carry(const WorldModel &wm);

// 借墙方案统计（纯统计：sim 汇总/复盘用；不影响任何决策）
long bank_plan_count();    // 机会次数（连续采纳算 1 次）
long bank_frame_count();   // 采纳帧数

}  // namespace simuro5
#endif
