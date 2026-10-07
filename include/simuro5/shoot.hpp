// shoot.hpp — 射门决策：输出瞄准点（球门开口侧前方），由 roles/motion 执行推球
#ifndef SIMURO5_SHOOT_HPP
#define SIMURO5_SHOOT_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

// ShootPlan：推球目标点 + 瞄准 y/角(度) + 方向单位向量 + 射程 cm + 净开口角(度) + 路线是否被挡 + 点球旁路 + 机会质量∈[0,1]
struct ShootPlan {
    bool viable = false;
    double target_x = 0, target_y = 90;
    double aim_y = 90;
    double dir_x = 0, dir_y = 1;
    double aim_rot = 0.0;
    double shot_dist = 0.0;
    double open_angle = 0.0;
    bool   lane_blocked = false;
    bool   penalty = false;
    double quality = 0.0;
    // 借墙方案（bank=true）：bank_wall 0/180 = 底/顶墙，bounce_x 反弹点 x，path_len 总路程 cm
    bool   bank = false;
    double bank_wall = 0.0;
    double bounce_x = 0.0;
    double bank_quality = 0.0;
    double path_len = 0.0;
    // 走廊拐弯方案（corridor=true）：不是射门，是把球从边路往门前中路走廊推
    bool   corridor = false;
    double gate_y = 90.0;
};

// 算最佳射门方案（直线 vs 借墙）；viable=false 表示没机会
ShootPlan plan_shoot(const WorldModel &wm, int shooter_id);

// 蜂群推进专用：只算借墙，射程放宽到 kBankCarryMax
ShootPlan plan_bank_carry(const WorldModel &wm);

// 走廊拐弯：边路没射门机会时把球往门前中路推（viable=false 表示该走别的分支）
ShootPlan plan_corridor(const WorldModel &wm, int shooter_id);

// 借墙统计：机会次数 / 采纳帧数
long bank_plan_count();
long bank_frame_count();

}  // namespace simuro5
#endif
