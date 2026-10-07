#include "simuro5/motion.hpp"
#include "simuro5/geometry.hpp"
#define TUNABLE_PREFIX "motion."
#include "simuro5/tunable.hpp"
#include <cmath>

namespace simuro5 {
namespace motion {

void stop(RobotState &r) { r.vl = 0.0; r.vr = 0.0; }

namespace {
TUNABLE(kWheelScaleSat, 1.0);
TUNABLE(kLatDamp, 0.5);
int g_legacy_drive = 0;
}

LegacyDriveScope::LegacyDriveScope() { ++g_legacy_drive; }
LegacyDriveScope::~LegacyDriveScope() { --g_legacy_drive; }

void position(RobotState &r, double tx, double ty, TargetMode mode) {
    double dx = tx - r.x, dy = ty - r.y;
    double de = std::hypot(dx, dy);
    // 目标点含 NaN/Inf（上游算崩）→ 直接停车，绝不把 NaN 写进平台速度
    if (!std::isfinite(de)) { stop(r); return; }
    if (mode == TM_STOP) {
        if (de < kStopEps) { stop(r); return; }
    } else if (de < 1.0) {
        stop(r); return;
    }

    double desired_angle = angle_to(r.x, r.y, tx, ty);
    double te = angle_diff(desired_angle, r.rot);
    double vc = (mode == TM_PASS) ? 200.0 : 150.0;   // 冲球/经过型满速（平台上限≈150cm/s）
    double Ka = 10.0 / 90.0;

    if (de > 100.0)      Ka = 20.0 / 90.0;
    else if (de > 50.0)  Ka = 22.0 / 90.0;
    else if (de > 30.0)  Ka = 24.0 / 90.0;
    else if (de > 20.0)  Ka = 26.0 / 90.0;
    else                 Ka = 28.0 / 90.0;

    double drive = vc * (1.0 / (1.0 + std::exp(-3.0 * de)) - 0.25);

    // 制动包线：按 v = sqrt(2·a·s) 限速 ⇒ 恰好停在目标点，且末段才收油
    if (mode == TM_STOP) {
        double v_allow = std::sqrt(2.0 * kBrakeAccel * std::max(0.0, de - kStopEps));
        if (drive > v_allow) drive = v_allow;
    }

    // 目标在侧面：先转后冲（差速轮直冲会沿车头来回越过侧向目标）
    auto lat_damp = [&](double t) {
        if (kLatDamp > 0.0 && g_legacy_drive == 0) drive *= 1.0 - kLatDamp + kLatDamp * std::cos(t * SIMURO5_PI / 180.0);
    };
    if (te > 95.0 || te < -95.0) {
        te += (te > 0) ? -180.0 : 180.0;
        te = clamp(te, -80.0, 80.0);
        lat_damp(te);
        if (de < 5.0 && std::fabs(te) < 40.0) Ka = 0.1;
        r.vr = (-drive + Ka * te);
        r.vl = (-drive - Ka * te);
    } else if (te > -85.0 && te < 85.0) {
        lat_damp(te);
        if (de < 5.0 && std::fabs(te) < 40.0) Ka = 0.1;
        r.vr = (drive + Ka * te);
        r.vl = (drive - Ka * te);
    } else {
        r.vr = (0.17 * te);
        r.vl = (-0.17 * te);
    }
    const double vmax = std::max(std::fabs(r.vl), std::fabs(r.vr));
    if (kWheelScaleSat > 0.5 && g_legacy_drive == 0 && vmax > kMaxWheel) { r.vl *= kMaxWheel / vmax; r.vr *= kMaxWheel / vmax; }
    r.vl = clamp(r.vl, -kMaxWheel, kMaxWheel);
    r.vr = clamp(r.vr, -kMaxWheel, kMaxWheel);
}

// 到点定向：走位 + 末端原地转正；返回"已到位且已对准"
namespace {
TUNABLE(kAlignedGain, 0.22);
TUNABLE(kMaxRotW, 14.0);
TUNABLE(kMinRotW, 2.5);
TUNABLE(kNearDist, 12.0);  // cm：进入近距相位的半径（先转正、再微调位置）
TUNABLE(kCreepMax, 20.0);
}
bool position_aligned(RobotState &r, double tx, double ty, double desired_rot,
                      double pos_tol, double ang_tol) {
    double de = std::hypot(tx - r.x, ty - r.y);
    if (!std::isfinite(de) || !std::isfinite(desired_rot)) { stop(r); return false; }
    double te_head = angle_diff(desired_rot, r.rot);

    if (de <= pos_tol && std::fabs(te_head) <= ang_tol) { stop(r); return true; }

    if (de > kNearDist) {
        position(r, tx, ty, TM_STOP);
        if (std::fabs(te_head) > 60.0) { r.vl *= 0.4; r.vr *= 0.4; }
        return false;
    }

    if (std::fabs(te_head) > ang_tol) {
        double w = kAlignedGain * te_head;
        w = clamp(w, -kMaxRotW, kMaxRotW);
        if (std::fabs(w) < kMinRotW) w = (te_head > 0.0) ? kMinRotW : -kMinRotW;
        r.vl = clamp(-w, -kMaxWheel, kMaxWheel);
        r.vr = clamp( w, -kMaxWheel, kMaxWheel);
        return false;
    }

    position(r, tx, ty, TM_STOP);
    double v = (r.vl + r.vr) * 0.5;
    double creep = clamp(0.8 * (de - pos_tol * 0.5), 3.0, kCreepMax);
    if (v > creep && v > 1e-6) {
        double s = creep / v;
        r.vl *= s; r.vr *= s;
    }
    return false;
}

// 到位迎球：赶路不因朝向没对准而限速；到位后原地转正迎球（不倒车）
bool arrive_facing(RobotState &r, double tx, double ty, double aim_rot,
                   double arrive_dist, double ang_tol, bool allow_reverse) {
    const double de = std::hypot(tx - r.x, ty - r.y);
    if (!std::isfinite(de) || !std::isfinite(aim_rot)) { stop(r); return false; }

    const double te_head = angle_diff(aim_rot, r.rot);
    if (de <= arrive_dist && std::fabs(te_head) <= ang_tol) { stop(r); return true; }

    if (de > arrive_dist) {
        if (!allow_reverse) {
            const double te_go = angle_diff(angle_to(r.x, r.y, tx, ty), r.rot);
            if (std::fabs(te_go) > 95.0) {
                double w = clamp(kAlignedGain * te_go, -kMaxRotW, kMaxRotW);
                if (std::fabs(w) < kMinRotW) w = (te_go > 0.0) ? kMinRotW : -kMinRotW;
                r.vl = clamp(-w, -kMaxWheel, kMaxWheel);
                r.vr = clamp( w, -kMaxWheel, kMaxWheel);
                return false;
            }
        }
        position(r, tx, ty, TM_STOP);
        return false;
    }

    double w = clamp(kAlignedGain * te_head, -kMaxRotW, kMaxRotW);
    if (std::fabs(w) < kMinRotW) w = (te_head > 0.0) ? kMinRotW : -kMinRotW;
    r.vl = clamp(-w, -kMaxWheel, kMaxWheel);
    r.vr = clamp( w, -kMaxWheel, kMaxWheel);
    return false;
}

void chase_ball(RobotState &r, const BallState &pred) {
    position(r, pred.x, pred.y, TM_PASS);
    double db = dist(r.x, r.y, pred.x, pred.y);
    if (db < 10.0) {
        r.vl *= db / 10.0;
        r.vr *= db / 10.0;
    }
}

void follow_route(RobotState &r, const RoutePlan &rt, int &wp_next, TargetMode mode) {
    if (!rt.found || rt.n_wp < 2) { stop(r); return; }
    if (wp_next < 0) wp_next = 0;
    if (wp_next > rt.n_wp - 1) wp_next = rt.n_wp - 1;

    // 段推进：到位(≤8cm)或已明显越过（离下一点近 8cm+）→ 切下一段
    while (wp_next < rt.n_wp - 1) {
        double d_cur  = dist(r.x, r.y, rt.wp_x[wp_next], rt.wp_y[wp_next]);
        double d_next = dist(r.x, r.y, rt.wp_x[wp_next + 1], rt.wp_y[wp_next + 1]);
        if (d_cur < 8.0 || d_next < d_cur - 8.0) ++wp_next;
        else break;
    }
    // 仅最后一段用调用方给的语义（末点才是目的地），中间段一律 TM_PASS
    TargetMode m = (wp_next == rt.n_wp - 1) ? mode : TM_PASS;
    position(r, rt.wp_x[wp_next], rt.wp_y[wp_next], m);
}

}  // namespace motion
}  // namespace simuro5
