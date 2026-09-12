#include "simuro5/motion.hpp"
#include "simuro5/geometry.hpp"
#include <cmath>

namespace simuro5 {
namespace motion {

void stop(RobotState &r) { r.vl = 0.0; r.vr = 0.0; }

void angle(RobotState &r, double desired_angle) {
    double te = angle_diff(desired_angle, r.rot);   // (-180,180]
    double w = 0.0;
    if (te > 50.0 || te < -50.0)      w = 9.0 / 90.0 * te;
    else if (te > 20.0 || te < -20.0) w = 11.0 / 90.0 * te;
    r.vl = -w;
    r.vr =  w;
}

void position(RobotState &r, double tx, double ty, TargetMode mode) {
    // —— 移植官方 demo Position()：以 sigmoid(d) 控制车速 + Ka*theta_e 修正转向 ——
    // docs/18 P1 改造：TM_STOP 目标加"制动包线"，见函数末注释。
    double dx = tx - r.x, dy = ty - r.y;
    double de = std::hypot(dx, dy);
    // 数值防护：目标点含 NaN/Inf（上游算崩）时直接停车，绝不把 NaN 写进平台速度。
    if (!std::isfinite(de)) { stop(r); return; }
    if (mode == TM_STOP) {
        // 停点任务：进入死区即停（不附加朝向条件——目标点已在脚下时
        // atan2(dy,dx) 方向是噪声，拿它判朝向会让机器人在锚点上乱转）
        if (de < kStopEps) { stop(r); return; }
    } else if (de < 1.0) {
        stop(r); return;   // 经过型任务：保持原到位判定
    }

    double desired_angle = angle_to(r.x, r.y, tx, ty);
    double te = angle_diff(desired_angle, r.rot);
    // vc：基准车速。复盘实测 demo 均速 97-100 cm/s，我们 55-88 → 再拉上限
    //   drive = vc*(sigmoid-0.25) 饱和值 = 0.75*vc；vc=150 → 上限 112（与 demo 追平）
    //   ⚠️ docs/18 真机复标（2026-09-10，build/traj_before.csv）：稳态速度我们
    //   p50/p90 = 79.8/115.3，demo = 67.9/97.9 —— 我们已不慢；瓶颈是"停不住"。
    double vc = 150.0, Ka = 10.0 / 90.0;

    // Ka 分段整体 +3：转向修正更积极，缩短差速轮重新瞄准的时间
    if (de > 100.0)      Ka = 20.0 / 90.0;
    else if (de > 50.0)  Ka = 22.0 / 90.0;
    else if (de > 30.0)  Ka = 24.0 / 90.0;
    else if (de > 20.0)  Ka = 26.0 / 90.0;
    else                 Ka = 28.0 / 90.0;

    double drive = vc * (1.0 / (1.0 + std::exp(-3.0 * de)) - 0.25);

    // —— 制动包线（docs/18 P1，核心改动）——
    // 双积分器（ṡ=v, v̇=u, |u|≤a）从静止到静止的时间最优控制就是 bang-bang，
    // 其切换曲线为 v = sqrt(2·a·s)：按它限速 = 恰好停在目标点且用时最短（不牺牲到达时间）。
    //   · 旧律的问题：drive 在 de≥5cm 就饱和（sigmoid 系数 3.0 太陡），唯一的减速条款
    //     `de<12 && |te|>30` 在"正对目标直冲"时不触发 → 到 de<1cm 才命令停车。
    //   · 真机标定：减速 p95=658 cm/s² → 从 115cm/s 停下需 10cm，1cm 内停住所允许速度只有
    //     3.6cm/s —— 旧律在 de=1cm 处命令 ~115cm/s，超标 30 倍 → 过冲必然 → te 翻 180°
    //     → 落进倒车分支 → 极限环（真机实测：低速来回蹭 2.3 次/机器人/分钟，demo 0.9；
    //     静止帧占比 13.4%，demo 3.5%）。
    //   · 包线把"减速"从物理兜底变成控制律的一部分：de<~17cm 起按曲线收油，
    //     最后 1.5cm 以低速蹭进死区停住。
    if (mode == TM_STOP) {
        double v_allow = std::sqrt(2.0 * kBrakeAccel * std::max(0.0, de - kStopEps));
        if (drive > v_allow) drive = v_allow;
    }
    // 注：旧条款 `if (de < 12.0 && fabs(te) > 30.0) drive *= 0.25;` 已删除——
    //     它要求"朝向没对准才减速"，恰好与停点任务相反，且与包线叠加会双重减速。

    if (te > 95.0 || te < -95.0) {
        // 目标在正后方：倒着走
        te += (te > 0) ? -180.0 : 180.0;
        te = clamp(te, -80.0, 80.0);
        if (de < 5.0 && std::fabs(te) < 40.0) Ka = 0.1;
        r.vr = (-drive + Ka * te);
        r.vl = (-drive - Ka * te);
    } else if (te > -85.0 && te < 85.0) {
        if (de < 5.0 && std::fabs(te) < 40.0) Ka = 0.1;
        r.vr = (drive + Ka * te);
        r.vl = (drive - Ka * te);
    } else {
        r.vr = (0.17 * te);
        r.vl = (-0.17 * te);
    }
    // 安全网：夹取只防异常数值（自然命令上限 ≈139），不参与调速。
    r.vl = clamp(r.vl, -kMaxWheel, kMaxWheel);
    r.vr = clamp(r.vr, -kMaxWheel, kMaxWheel);
}

// 到点定向（docs/18 P2）：走位 + 末端原地转正，返回"已到位且已对准"。
// 为什么用"原地转"而不是"边走边对"：差速轮可原地旋转，转正是确定性的；
//   而"边走边对"要同时满足位置与朝向两个约束，路径会绕大圈且到点朝向仍靠运气。
namespace {
constexpr double kAlignedGain = 0.22;   // 轮速命令/度
constexpr double kMaxRotW     = 14.0;   // 转正轮速上限（≈160°/s，防大角度命令爆表）
constexpr double kMinRotW     = 2.5;    // 转正轮速下限（≈29°/s，防最后几度爬行）
constexpr double kNearDist    = 12.0;   // cm：进入"近距相位"的半径（先转正、再微调位置）
constexpr double kCreepMax    = 20.0;   // cm/s：近距平移速度上限（对准后小步靠近）
}
bool position_aligned(RobotState &r, double tx, double ty, double desired_rot,
                      double pos_tol, double ang_tol) {
    double de = std::hypot(tx - r.x, ty - r.y);
    if (!std::isfinite(de) || !std::isfinite(desired_rot)) { stop(r); return false; }
    double te_head = angle_diff(desired_rot, r.rot);   // 机头 vs 期望朝向

    if (de <= pos_tol && std::fabs(te_head) <= ang_tol) { stop(r); return true; }  // 完成

    // ① 远距：正常走位（复用制动包线停得准）；朝向误差大时限速，
    //    免得斜着高速冲进准备点（到了还得原地转半天）
    if (de > kNearDist) {
        position(r, tx, ty, TM_STOP);
        if (std::fabs(te_head) > 60.0) { r.vl *= 0.4; r.vr *= 0.4; }
        return false;
    }

    // ② 近距 + 朝向没正：**原地**旋转（不产生位移，所以不会把位置甩出容差）
    //    ⚠️ 相位抖动（旧版实测 300 帧才就绪）的根源是"位置超一点点差就跳回全速走位相位"，
    //    现在用 kNearDist 距离门限把两个相位隔开：12cm 外=走位，12cm 内=转正+微调。
    //    这里不设额外滞回带——设了会在带内变成"什么都不做"的死区（实测卡在 11.7°）。
    if (std::fabs(te_head) > ang_tol) {
        double w = kAlignedGain * te_head;
        w = clamp(w, -kMaxRotW, kMaxRotW);
        if (std::fabs(w) < kMinRotW) w = (te_head > 0.0) ? kMinRotW : -kMinRotW;
        r.vl = clamp(-w, -kMaxWheel, kMaxWheel);
        r.vr = clamp( w, -kMaxWheel, kMaxWheel);
        return false;
    }

    // ③ 近距 + 朝向已可接受：小速度平移补掉剩下的位置差（此时机头已朝期望方向，
    //    前进是安全的；用 position 的转向修正保证走直线）
    position(r, tx, ty, TM_STOP);
    double v = (r.vl + r.vr) * 0.5;
    double creep = clamp(0.8 * (de - pos_tol * 0.5), 3.0, kCreepMax);
    if (v > creep && v > 1e-6) {
        double s = creep / v;
        r.vl *= s; r.vr *= s;
    }
    return false;
}

void chase_ball(RobotState &r, const BallState &pred) {    // 追预测点；快到球时减速避免冲过头。经过型任务 → TM_PASS（不套制动包线，
    //   否则抢点会变慢；减速仍由下面的 10cm 线性缩放负责）
    position(r, pred.x, pred.y, TM_PASS);
    double db = dist(r.x, r.y, pred.x, pred.y);
    if (db < 10.0) {
        r.vl *= db / 10.0;
        r.vr *= db / 10.0;
    }
}

void follow_route(RobotState &r, const RoutePlan &rt, int &wp_next, TargetMode mode) {
    if (!rt.found || rt.n_wp < 2) { stop(r); return; }   // 不可规划：调用方回退直线
    if (wp_next < 0) wp_next = 0;
    if (wp_next > rt.n_wp - 1) wp_next = rt.n_wp - 1;

    // 段推进：到达当前 wp（≤8cm）或已明显越过（离下一点更近 8cm+）→ 切下一段
    while (wp_next < rt.n_wp - 1) {
        double d_cur  = dist(r.x, r.y, rt.wp_x[wp_next], rt.wp_y[wp_next]);
        double d_next = dist(r.x, r.y, rt.wp_x[wp_next + 1], rt.wp_y[wp_next + 1]);
        if (d_cur < 8.0 || d_next < d_cur - 8.0) ++wp_next;
        else break;
    }
    // 只有最后一段用调用方给的语义（末点才是目的地）；中间段一律 TM_PASS
    TargetMode m = (wp_next == rt.n_wp - 1) ? mode : TM_PASS;
    position(r, rt.wp_x[wp_next], rt.wp_y[wp_next], m);
}

}  // namespace motion
}  // namespace simuro5
