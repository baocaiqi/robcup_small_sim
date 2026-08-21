#include "simuro5/role_assignment.hpp"

namespace simuro5 {

void RoleAssignment::assign(WorldModel &wm) {
    // 固定角色分工（参考官方 demo「固定角色」思想，自研实现）：
    //   0 号守门员；1 号主攻（追球/带球/射门）；2 号助攻（左路接应）；
    //   3 号中场（右路/中路衔接）；4 号中卫（区域防守/拦截）。
    // 角色固定 → 机器人不每帧换角色 → 少无效转向（有效速度↑）、
    // 宽度/阵型保持；区域触发由各角色行为（roles.cpp）实现。
    wm.role[0] = ROLE_GOALIE;
    wm.role[1] = ROLE_ACTIVE;
    wm.role[2] = ROLE_ASSIST;
    wm.role[3] = ROLE_MIDFIELD;
    wm.role[4] = ROLE_PASSIVE;
}

}  // namespace simuro5
