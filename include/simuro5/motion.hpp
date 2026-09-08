// ============================================================
// motion.hpp — 差速轮运动控制
// 基底：官方 demo 的 Position()/Angle()（已验证能跑的算法），
// 接口风格：目标点+朝向，输出 vl/vr。
// 控制量直接写机器人 velocityLeft/velocityRight。
// ============================================================
#ifndef SIMURO5_MOTION_HPP
#define SIMURO5_MOTION_HPP

#include "simuro5/world_model.hpp"

namespace simuro5 {
namespace motion {

// 原地转向到 desired_angle(度)。vl=-v, vr=+v 旋转，速率随角度差。
void angle(RobotState &r, double desired_angle);

// 走到目标点 (tx,ty)。核心：官方 Position() 的 sigmoid 速度 + 角度误差比例修正。
void position(RobotState &r, double tx, double ty);

// 追球：追平台的预测球位（球速外推），到附近减速。
void chase_ball(RobotState &r, const BallState &pred);

// 立即停车
void stop(RobotState &r);

}  // namespace motion
}  // namespace simuro5
#endif
