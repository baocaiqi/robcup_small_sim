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
    // vc：基准车速。复盘实测我们平均 54-63 cm/s，demo 60-68 → 提高上限
    //   drive = vc*(sigmoid-0.3) 饱和值 = 0.7*vc=70；改 -0.25 且 vc=120 → 饱和 90（+29%）
    double vc = 120.0, Ka = 10.0 / 90.0;

    if (de > 100.0)      Ka = 17.0 / 90.0;
    else if (de > 50.0)  Ka = 19.0 / 90.0;
    else if (de > 30.0)  Ka = 21.0 / 90.0;
    else if (de > 20.0)  Ka = 23.0 / 90.0;
    else                 Ka = 25.0 / 90.0;

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
