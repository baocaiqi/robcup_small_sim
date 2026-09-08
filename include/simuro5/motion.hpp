// ============================================================
// motion.hpp — 差速轮运动控制
// 基底：官方 demo 的 Position()（已验证能跑的算法），
// 接口风格：目标点+朝向，输出 vl/vr。
// 控制量直接写机器人 velocityLeft/velocityRight。
// ============================================================
#ifndef SIMURO5_MOTION_HPP
#define SIMURO5_MOTION_HPP

#include "simuro5/world_model.hpp"
#include "simuro5/route.hpp"

namespace simuro5 {
namespace motion {

// 走到目标点 (tx,ty)。核心：官方 Position() 的 sigmoid 速度 + 角度误差比例修正。
void position(RobotState &r, double tx, double ty);

// 追球：追平台的预测球位（球速外推），到附近减速。
void chase_ball(RobotState &r, const BallState &pred);

// 立即停车
void stop(RobotState &r);

// —— docs/15 P0：沿避障路径逐段执行 ——
// 目标 = rt.wp[wp_next]；到位(≤8cm)或已越过时自动推进到下一段；
// wp_next 为跨帧状态（角色层持有，路径重算后重置为 0）。
// 注意：rt.found == false 时调用方必须先回退直线，本函数只停车兜底。
void follow_route(RobotState &r, const RoutePlan &rt, int &wp_next);

}  // namespace motion
}  // namespace simuro5
#endif
