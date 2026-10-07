// formation.hpp — 死球摆位（SetFormer/SetLater/SetBall）
#ifndef SIMURO5_FORMATION_HPP
#define SIMURO5_FORMATION_HPP

#include "simuro5/simuro_interface.hpp"
#include "simuro5/team.hpp"

namespace simuro5 {

// 先摆方摆位（无球位，争球按 1/4 区中心估计）
void formation_former(const TeamContext &ctx, PlayMode gs, Robot robots[]);

// 后摆方摆位（有对方先摆位置 + 球位）
void formation_later(const TeamContext &ctx, PlayMode gs,
                     Robot formerRobots[], Vector3D ball, Robot laterRobots[]);

// 发门球时设置球位（门区内）
void formation_set_ball(const TeamContext &ctx, PlayMode gs, Vector3D *pBall);

}  // namespace simuro5
#endif
