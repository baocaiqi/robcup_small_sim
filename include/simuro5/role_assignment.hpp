// ============================================================
// role_assignment.hpp — 固定角色分配
//
// 角色(Roles):
//   GOALIE=0  1号守门员(固定)
//   ACTIVE=1  主攻(追球/带球/射门)
//   PASSIVE=2 中卫(区域防守/拦截)
//   ASSIST=3  助攻(左路接应)
//   MIDFIELD=4 中场(右路/中路衔接)
//
// 说明：从「距离贪心动态分配」改为「固定角色分工」
// （参考官方 demo 固定分工思想，自研实现）。
// 角色固定 → 机器人不再每帧换角色 → 消除无效转向（有效速度↑）、
// 宽度/阵型结构保持；「谁负责哪片区域」由各角色行为实现。
// ============================================================
#ifndef SIMURO5_ROLE_ASSIGNMENT_HPP
#define SIMURO5_ROLE_ASSIGNMENT_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

enum Roles {
    ROLE_GOALIE    = 0,
    ROLE_ACTIVE    = 1,
    ROLE_PASSIVE   = 2,
    ROLE_ASSIST    = 3,
    ROLE_MIDFIELD  = 4,
    ROLE_NONE      = 9
};

class RoleAssignment {
public:
    // 固定角色分工：0=GK 1=ACTIVE 2=ASSIST 3=MID 4=PASSIVE，结果写回 wm.role[]
    void assign(WorldModel &wm);
};

}  // namespace simuro5
#endif
