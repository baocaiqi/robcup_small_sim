// strategy.hpp — 主策略调度
#ifndef SIMURO5_STRATEGY_HPP
#define SIMURO5_STRATEGY_HPP

#include "simuro5/world_model.hpp"
#include "simuro5/situation.hpp"
#include "simuro5/role_assignment.hpp"

namespace simuro5 {

// 是否我方主罚点球（球静止在对方罚球点）
bool we_take_penalty_spot(const WorldModel &wm);

class Strategy {
public:
    // 一周期决策，结果写机器人轮速
    void run(WorldModel &wm);

private:
    SituationModule sit_;
    RoleAssignment ra_;

    // 攻防状态机：滞回 + 事件 + 威胁
    void update_team_state(WorldModel &wm);
    // 由状态与球位算威胁等级 0~1
    double threat_from_state(const WorldModel &wm) const;

    // 清道夫指派（球拉边时封中路）
    void update_sweeper(WorldModel &wm);

    // 前场散球逼抢者指派
    void update_presser(WorldModel &wm);
};

// 除 0 号门将外，进我方门区就顶出
void enforce_own_goal_area(WorldModel &wm);

// 己方大禁区非门将最多 3 人（规则 7.10.4）
void enforce_own_penalty_count(WorldModel &wm);

}  // namespace simuro5
#endif
