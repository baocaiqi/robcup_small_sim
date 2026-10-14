// local_planner.hpp — 无球跑位的局部避让（势场转向）+ DWA 影子对照（原点左下角 cm，颜色无关）
// 参考对手 [TAC] 势场规划 / DWA 思想（见 _re/spec/TAC-势场规划器-规格.md、DWA-规格.md），自研实现：
//   不碰轮速公式，只把"直奔目标"的目标点换成"绕开障碍后的前视点"，再交回 motion::position 执行。
//   DWA 只做影子：自己挑一组最优轮速，跟实际下发的比对一致率，写黑匣子，不驱动轮子。
#ifndef SIMURO5_LOCAL_PLANNER_HPP
#define SIMURO5_LOCAL_PLANNER_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {

// 角色函数下发完目标后调用：符合条件就把目标改成避让前视点（mode 2）；mode 1 只算不改
void plan_steer(WorldModel &wm, int id);

// 每帧末尾调用一次：DWA 影子对照 + 计数日志（只在 HNNU_TRACE 编译时写盘）
void plan_frame_end(const WorldModel &wm);

// 0=关 1=影子 2=转向生效
int local_planner_mode();

}  // namespace simuro5
#endif
