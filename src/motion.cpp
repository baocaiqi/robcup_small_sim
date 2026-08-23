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

void position(RobotState &r, double tx, double ty) {
    // —— 移植官方 demo Position()：以 sigmoid(d) 控制车速 + Ka*theta_e 修正转向 ——
    double dx = tx - r.x, dy = ty - r.y;
    double de = std::hypot(dx, dy);
    if (de < 1.0) { stop(r); return; }

    double desired_angle = angle_to(r.x, r.y, tx, ty);
    double te = angle_diff(desired_angle, r.rot);
    // vc：基准车速。复盘实测 demo 均速 97-100 cm/s，我们 55-88 → 再拉上限
    //   drive = vc*(sigmoid-0.25) 饱和值 = 0.75*vc；vc=150 → 上限 112（与 demo 追平）
    double vc = 150.0, Ka = 10.0 / 90.0;

    // Ka 分段整体 +3：转向修正更积极，缩短差速轮重新瞄准的时间
    if (de > 100.0)      Ka = 20.0 / 90.0;
    else if (de > 50.0)  Ka = 22.0 / 90.0;
    else if (de > 30.0)  Ka = 24.0 / 90.0;
    else if (de > 20.0)  Ka = 26.0 / 90.0;
    else                 Ka = 28.0 / 90.0;

    double drive = vc * (1.0 / (1.0 + std::exp(-3.0 * de)) - 0.25);

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
}

void chase_ball(RobotState &r, const BallState &pred) {
    // 追预测点；快到球时减速避免冲过头
    position(r, pred.x, pred.y);
    double db = dist(r.x, r.y, pred.x, pred.y);
    if (db < 10.0) {
        r.vl *= db / 10.0;
        r.vr *= db / 10.0;
    }
}

}  // namespace motion
}  // namespace simuro5
