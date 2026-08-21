// ============================================================
// strategy.hpp — 主策略调度（RunStrategy 核心）
// 每周期：更新世界模型 → 分析局势 → 算站位 → 分配角色 → 角色执行
// ============================================================
#ifndef SIMURO5_STRATEGY_HPP
#define SIMURO5_STRATEGY_HPP

#include "simuro5/world_model.hpp"
#include "simuro5/situation.hpp"
#include "simuro5/role_assignment.hpp"

namespace simuro5 {

class Strategy {
public:
    // 一周期决策（只读 wm 输入，决策写入 wm.home[i].vl/vr）
    void run(WorldModel &wm);

private:
    SituationModule sit_;
    RoleAssignment ra_;

    // 攻防状态机：滞回计数 + 状态翻转 + 事件标志 + 威胁分级
    void update_team_state(WorldModel &wm);
    double threat_from_state(const WorldModel &wm) const;
};

}  // namespace simuro5
#endif
