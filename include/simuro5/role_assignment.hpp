// ============================================================
// role_assignment.hpp — 效用函数角色分配
// 移植自中型组 role_assignment.cpp（集中式版：一次分配全部 4 个角色）
//
// 角色(Roles，与中型组一致):
//   GOALIE=0  1号守门员(固定)
//   ACTIVE=1  追球者(离球最近/效用最高)
//   PASSIVE=2 防守站位
//   ASSIST=3  助攻跑位
//   MIDFIELD=4 中场衔接
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
    // 给 1..4 号机器人分配角色（0 号恒为守门员），结果写回 wm.role[]
    void assign(WorldModel &wm);

private:
    int _last_active_idx = 1;          // 滞后：避免追球角色抖动
    double _utility(int id, const WorldModel &wm, int role) const;
};

}  // namespace simuro5
#endif
