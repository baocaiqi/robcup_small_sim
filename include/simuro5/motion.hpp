// motion.hpp — 差速轮运动控制；输入目标点 + 朝向，输出 vl/vr（cm/s）
#ifndef SIMURO5_MOTION_HPP
#define SIMURO5_MOTION_HPP

#include "simuro5/world_model.hpp"
#include "simuro5/route.hpp"

namespace simuro5 {
namespace motion {

// TM_STOP = 必须精确停在该点（套制动包线）；TM_PASS = 只是经过（不减速）
enum TargetMode { TM_STOP = 0, TM_PASS = 1 };

// 制动包线：保守取 400（真机实测减速 p95=658，sim 口径仅 270）
constexpr double kBrakeAccel = 400.0;   // cm/s²，v ≤ sqrt(2·a·s)
constexpr double kStopEps    = 1.5;     // cm：到位死区（吸收单帧量化）
constexpr double kMaxWheel   = 150.0;   // 轮速命令上限（自然命令约 139）

// 走到 (tx,ty)：sigmoid 速度 + 角度误差比例修正；TM_STOP 时受制动包线约束
void position(RobotState &r, double tx, double ty, TargetMode mode = TM_STOP);

#ifdef SIMURO5_GK_PROBE
// 门将目标点探针（诊断专用，仅 sim_bench/sim_trace 编译）：
// position() 记下本次目标点，门将每帧取回。取回的是"夹进活动范围之后"的目标，
// 所以打印出来的就是门将这一帧真正要去的 y —— 不是规则的中间量。
struct GkTargetProbe {
    bool  on  = false;
    double tx = 0.0, ty = 0.0;
    // 本帧 GkView 的几个关键量，由 roles.cpp 在 gk_view() 之后填（sim_bench 打印用）
    double y_at_goal = 90.0;
    double ball_goal = 999.0;   // 球到己门线距离
    double db = 999.0;          // 门将到球距离
    double opp_dmin = 999.0;    // 最近对手到球距离
    int    heading_goal = 0, on_target = 0;
    int    rule_idx = -1;       // 命中规则在 kGoalieRules 里的下标（-1=前置规则或未命中）
    void begin() { on = true; }
    void finish() { on = false; }
    void note(double x, double y) {
        if (!on) return;
        tx = x; ty = y;
    }
};
extern GkTargetProbe g_gk_probe;   // motion.cpp 定义（per-(dll) 一份）
#endif

// 作用域内 position() 用旧口径（单轮截断、不收侧向油门）——门将门球按旧手感标定
struct LegacyDriveScope {
    LegacyDriveScope();
    ~LegacyDriveScope();
};

// 走到 (tx,ty) 并把机头转到 desired_rot（度）；到位且对准才返回 true
bool position_aligned(RobotState &r, double tx, double ty, double desired_rot,
                      double pos_tol = 3.0, double ang_tol = 8.0);

// 到位迎球：不动朝向限速，到位后原地转正到 aim_rot（allow_reverse=false 不倒车）
bool arrive_facing(RobotState &r, double tx, double ty, double aim_rot,
                   double arrive_dist = 8.0, double ang_tol = 12.0,
                   bool allow_reverse = true);

// 追平台的预测球位（球速外推），到附近减速
void chase_ball(RobotState &r, const BallState &pred);

// 立即停车
void stop(RobotState &r);

// 沿避障路径逐段执行（wp_next 为跨帧状态，到位≤8cm 或越过即推进）
// mode 只作用于最后一段；rt.found==false 时本函数只停车兜底，调用方须回退直线
void follow_route(RobotState &r, const RoutePlan &rt, int &wp_next,
                  TargetMode mode = TM_PASS);

}  // namespace motion
}  // namespace simuro5
#endif
