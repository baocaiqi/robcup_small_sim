// ============================================================
// motion.hpp — 差速轮运动控制
// 基底：官方 demo 的 Position()/Angle()（已验证能跑的算法），
// 接口风格：目标点+朝向，输出 vl/vr。
// 控制量直接写机器人 velocityLeft/velocityRight。
// ============================================================
#ifndef SIMURO5_MOTION_HPP
#define SIMURO5_MOTION_HPP

#include "simuro5/world_model.hpp"
#include "simuro5/route.hpp"

namespace simuro5 {
namespace motion {

// ============================================================
// 目标语义（docs/18 P1）：决定要不要在该点"停住"
//   TM_STOP：必须精确停在该点（站位锚点/防守点/推球准备点）→ 套制动包线，提前减速
//   TM_PASS：只是经过（追球/穿球推射/绕障段）→ 不减速，保持原速律（只做饱和）
// 为什么不一律减速：追球/抢点是"经过型"任务，减速是净损失（会抢不到点）。
// ============================================================
enum TargetMode { TM_STOP = 0, TM_PASS = 1 };

// —— 制动包线参数（真机标定：tools/py/motion_calib.py，见 docs/18）——
// 真机实测减速能力 p95 = 658 cm/s²（静止帧同口径噪声底噪仅 18 → 信号可信）；
// 设计值取更保守的 400 留余量：包线偏保守只会让到点稍慢，偏乐观就会冲过目标。
// 离线用例 test_motion_stop_convergence 断言"到达时间不超基线 +10%"，调过头就回来改这里。
// 2026-09-12 讨论记录：真机实测物理减速 p95=658，理论上可把这里提到 600（末段快约 40%）。
//   但 **sim 口径的物理减速只有 270**（docs/18 §P1：k=0.9、减速 270），600 会让
//   `test_motion_stop_convergence`（带 25° 偏差收敛）与包线断言直接失败 → 过冲打转。
//   结论：主分支保持 400（与团队标定/sim A/B 闭环一致）；要用 600 必须先重标 sim 侧测试，
//   见 tools/py/patch_gk_wall.py 的 p5（含 600 的版本，作为真机实验用）。
constexpr double kBrakeAccel = 400.0;   // cm/s²：制动设计加速度（v ≤ sqrt(2·a·s)）
constexpr double kStopEps    = 1.5;     // cm：到位死区（≥ 单帧位移量级，吸收离散量化）
constexpr double kMaxWheel   = 150.0;   // 轮速命令安全上限（自然命令上限≈139：te≈±85、Ka=28/90）

// 原地转向到 desired_angle(度)。vl=-v, vr=+v 旋转，速率随角度差。
// ⚠️ 目前无调用方（僵尸函数，docs/03 记录过删除、回退时恢复）——保留仅为接口兼容。
void angle(RobotState &r, double desired_angle);

// 走到目标点 (tx,ty)。sigmoid 速度 + 角度误差比例修正；
// mode=TM_STOP 时速度再受制动包线约束（v ≤ sqrt(2·kBrakeAccel·(de−kStopEps))），
// 到 de<kStopEps 停车——根治"满速冲到目标点、物理刹不住 → 过冲 → 倒车 → 极限环"（docs/18）。
void position(RobotState &r, double tx, double ty, TargetMode mode = TM_STOP);

// ============================================================
// 到点定向（docs/18 P2）：走到 (tx,ty) **并且**把机头转到 desired_rot（度）
// 返回 true = 已到位(pos_tol 内) 且 已对准(ang_tol 内) → 调用方可执行后续动作
//            （如射门直线推穿）；返回 false = 本帧仍在走位或转正中（vl/vr 已写好）。
//
// 为什么需要它：position() 只管"停在哪"，不管"停下来时脸朝哪"；而本平台没有踢球动作，
//   射门 = 用身体把球推出去 ⇒ **球的出射方向 ≈ 撞球瞬间的机头方向**。
//   实测真机射正率仅 8%（对手 48%），根因之一就是"到达时机头朝向是随机的"。
// 三段式（差速轮可原地转，所以对准不必边走边对）：
//   ① 走位  de > pos_tol      ：沿用 position(TM_STOP) 制动包线停准；朝向误差 >60° 时限速，
//                               免得斜着高速冲进准备点（到了还得原地转很久）
//   ② 转正  de ≤ pos_tol 且 |te|>ang_tol：原地差速旋转（不前进、不碰球）
//   ③ 完成  两者都满足        ：停车并返回 true
// ============================================================
bool position_aligned(RobotState &r, double tx, double ty, double desired_rot,
                      double pos_tol = 3.0, double ang_tol = 8.0);

// 追球：追平台的预测球位（球速外推），到附近减速。
void chase_ball(RobotState &r, const BallState &pred);

// 立即停车
void stop(RobotState &r);

// —— docs/15 P0：沿避障路径逐段执行 ——
// 目标 = rt.wp[wp_next]；到位(≤8cm)或已越过时自动推进到下一段；
// wp_next 为跨帧状态（角色层持有，路径重算后重置为 0）。
// 注意：rt.found == false 时调用方必须先回退直线，本函数只停车兜底。
// mode 只作用于**最后一段**（末点才是真正的目的地）；中间 waypoint 一律 TM_PASS
// ——绕障途中每段都减速会让机器人一顿一顿（docs/18 P3 才做拐点限速）。
void follow_route(RobotState &r, const RoutePlan &rt, int &wp_next,
                  TargetMode mode = TM_PASS);

}  // namespace motion
}  // namespace simuro5
#endif
