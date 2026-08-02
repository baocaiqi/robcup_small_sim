// ============================================================
// formation.hpp — 死球摆位（SetFormer/SetLater/SetBall 实现）
// 规则依据：官方规则 PDF 7.6/7.10~7.17
//   · 开球：进攻方先摆(中圈+自己半场)，防守方后摆(自己半场除中圈)
//   · 争球：先摆方1人放球侧25cm，后摆方按平台给的球位布置，其余人1/4区外
//   · 点球：防守方先摆(守门员门线+队友中线另一边)，进攻方后摆(罚球人球后)
//   · 任意球：进攻方先摆(罚球人球后10cm)，防守方后摆(门前+自己半场)
//   · 门球：发球方先摆(守门员门区+队友门区外)，球位由 SetBall 定
// ============================================================
#ifndef SIMURO5_FORMATION_HPP
#define SIMURO5_FORMATION_HPP

#include "simuro5/simuro_interface.hpp"
#include "simuro5/team.hpp"

namespace simuro5 {

// 先摆方摆位（无球位信息，争球用 1/4 区中心估计）
void formation_former(const TeamContext &ctx, PlayMode gs, Robot robots[]);

// 后摆方摆位（有对方先摆位置 + 球位）
void formation_later(const TeamContext &ctx, PlayMode gs,
                     Robot formerRobots[], Vector3D ball, Robot laterRobots[]);

// 发门球时设置球位置（门区内）
void formation_set_ball(const TeamContext &ctx, PlayMode gs, Vector3D *pBall);

}  // namespace simuro5
#endif
