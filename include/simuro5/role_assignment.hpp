// role_assignment.hpp — 角色分配（0=GK 1=ACTIVE 2=ASSIST 3=MID 4=PASSIVE）
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
    // 固定角色分工，结果写回 wm.role[]
    void assign(WorldModel &wm);
};

}  // namespace simuro5
#endif
