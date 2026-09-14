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

// ============================================================
// 我方是否正在主罚点球？（docs/06 第 56 轮，2026-09-12 真机 16:52 场复盘）
// ------------------------------------------------------------
// ⚠️ 不能只看 gameState：真机实测**执行期平台报的不是点球态**（否则平台不会调用
//    RunStrategy）→ 原来"比对 PM_PenaltyKick_*"的判据永远为假：9 次点球里 ACTIVE 被
//    "死球别推"守卫支到 (85,90) 干等 2.5 秒，球一次没碰（rlg 帧 1316~1358 铁证）。
// 改用可观测量：**球静止在对方罚球点上**。真机实测罚球点 = 门前 39.4cm、正中央
//    （两场共 14 次摆球全部落在 (39.4,89.8) 或镜像 (180.8,89.7)），容差 ±3cm。
// 返回 true = 我方主罚（roles 的"我方点球必须去踢"例外生效）。
// ============================================================
bool we_take_penalty_spot(const WorldModel &wm);

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

    // 清道夫(远侧覆盖)指派：球在防守三区拉边时挑一个区域防守者钉中路封远门柱/横传
    void update_sweeper(WorldModel &wm);
};

// 我方门区"只能有门将"硬闸（docs/06 第 68 轮）：除 0 号门将外，任何人进我方门区
//   都被顶到门区前缘外 8cm。独立成函数是为了能单测（不依赖角色决策）。
void enforce_own_goal_area(WorldModel &wm);

}  // namespace simuro5
#endif
