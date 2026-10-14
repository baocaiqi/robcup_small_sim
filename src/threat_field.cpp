// threat_field.cpp — 对手威胁场（参考"高斯威胁势场"思想，自研实现）
// 场 phi(x,y) = Σ t_k·exp(−r_k²/(2σ_k²))，梯度取解析式；不存网格，任意点现算（每次 6 个 exp，微秒级）。
// 与对手实现的区别（见 _re/spec/ThreatField-规格.md 末节）：
//   ① 亮度 t 来自我们自己的 mark_threat()，用 s/(s+kThreatTRef) 压到 [0,1)；
//   ② 光圈 σ = r0 + |v|·τ，含义是"对手 τ 帧内能跑到的范围"，速度用 cm/帧；
//   ③ 球门位置一律从 ctx 取，黄蓝两队共用。
// 下面所有初值都是我们自己拍的起点，等影子数据标定；没有一个取自对手二进制。还没真机验证。
#include "simuro5/threat_field.hpp"
#include "simuro5/world_model.hpp"
#include "simuro5/defense.hpp"
#include "simuro5/blackbox.hpp"
#define TUNABLE_PREFIX "threat."
#include "simuro5/tunable.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

// 总开关：0=关 1=影子（默认，只算不用）2=门线站位生效
TUNABLE(kThreatFieldMode, 1.0);
// 门线站位里威胁场 y 的混合比例（mode 2 才用；保守起步）
TUNABLE(kThreatFieldGkBlend, 0.3);
// 亮度归一化参考分：mark_threat 打分等于它时 t=0.5
TUNABLE(kThreatTRef, 2.0);
// 对手光圈：静止半径 r0(cm) + 速度(cm/帧)×τ(帧)；速度先截到 kThreatVCap，防复位残留把光圈撑爆
TUNABLE(kThreatSigmaR0, 20.0);
TUNABLE(kThreatSigmaTau, 10.0);
TUNABLE(kThreatVCap, 5.0);
// 球项：朝我门速度达到 kThreatBallVRef(cm/帧) 时亮度满 1，光圈固定；kThreatBallOn=0 关掉球项
TUNABLE(kThreatBallOn, 1.0);
TUNABLE(kThreatBallVRef, 10.0);
TUNABLE(kThreatBallSigma, 12.0);
// 影子日志间隔（帧）；只在 HNNU_TRACE 编译时写盘
TUNABLE(kThreatLogEvery, 10.0);

int    threat_field_mode() { return (int)kThreatFieldMode; }
double threat_gk_blend()   { return kThreatFieldGkBlend; }

namespace {

inline double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }

inline void set_source(ThreatSource &s, double x, double y, double t, double sigma) {
    s.x = x; s.y = y; s.t = t;
    s.sigma = std::max(1.0, sigma);
    s.inv2s2 = 1.0 / (2.0 * s.sigma * s.sigma);
}

// 一次性自检：中心差分 vs 解析梯度，最大误差写黑匣子 TC 行（只在 HNNU_TRACE 编译时有输出）
void self_check_once(const ThreatField &tf) {
    static bool done = false;
    if (done || !bb::kEnabled) return;
    done = true;
    const double h = 0.01;
    double worst = 0.0;
    for (int i = 0; i < 25; ++i) {
        double x = 20.0 + 45.0 * (i % 5), y = 20.0 + 35.0 * (i / 5);
        double gx = 0, gy = 0;
        threat_grad(tf, x, y, &gx, &gy);
        double nx = (threat_phi(tf, x + h, y) - threat_phi(tf, x - h, y)) / (2.0 * h);
        double ny = (threat_phi(tf, x, y + h) - threat_phi(tf, x, y - h)) / (2.0 * h);
        worst = std::max(worst, std::max(std::fabs(nx - gx), std::fabs(ny - gy)));
    }
    if (std::FILE *f = bb::fp()) std::fprintf(f, "TC,%ld,%.3e\n", bb::st().frame, worst);
}

}  // namespace

void threat_build(ThreatField &tf, const WorldModel &wm) {
    tf.valid = false;
    tf.n = 0;
    tf.top = -1;
    tf.goal_D = 0; tf.line_max_y = 90; tf.line_max_v = 0;
    if (kThreatFieldMode <= 0.0 || !wm.ball.valid) return;

    // 对手灯泡：亮度沿用盯人打分（离球近/离门近/接球中/带球突破），所以"灯泡"和"盯谁"是同一把尺子
    const double danger = ball_danger_speed(wm);
    const int dribbler = nearest_opp_to_ball(wm);
    double best_t = -1.0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        const double ox = wm.opp[i].x, oy = wm.opp[i].y;
        if (!std::isfinite(ox) || !std::isfinite(oy)) continue;
        const double d_ball = dist(wm.ball.x, wm.ball.y, ox, oy);
        const double d_goal = wm.ctx.dist_our_goal(ox);
        const double appr = ball_approach_speed(wm, ox, oy);
        const double s = mark_threat(d_ball, d_goal, appr, danger, i == dribbler);
        const double t = s / (s + std::max(1e-6, kThreatTRef));
        const double v = std::min(std::hypot(wm.opp_vx[i], wm.opp_vy[i]), kThreatVCap);
        set_source(tf.src[tf.n++], ox, oy, t, kThreatSigmaR0 + v * kThreatSigmaTau);
        if (t > best_t) { best_t = t; tf.top = i; }
    }
    // 球灯泡：只看"朝我门飞多快"，背离为 0；静止球给个小底亮度，表示"球本身就是争夺点"
    if (kThreatBallOn > 0.0 && std::isfinite(wm.ball.x) && std::isfinite(wm.ball.y)) {
        const double t = clamp01(0.1 + 0.9 * danger / std::max(1e-6, kThreatBallVRef));
        set_source(tf.src[tf.n++], wm.ball.x, wm.ball.y, t, kThreatBallSigma);
    }
    tf.valid = tf.n > 0;
    if (!tf.valid) return;

    // 门线汇总：沿己门线（门框内）积分 + 找最危险的 y，每 2cm 一个点
    const double gx = wm.ctx.our_goal_x();
    tf.goal_D = threat_line_integral(tf, gx, goal_y_low(), goal_y_high(), 2.0);
    tf.line_max_v = threat_line_max(tf, gx, goal_y_low(), goal_y_high(), 2.0, &tf.line_max_y);

    // 影子日志 TF 行：帧,mode,D,门线最危险y,该点phi,top,然后每个灯泡的 t,sigma
    if (bb::kEnabled) {
        self_check_once(tf);
        const long every = std::max(1L, (long)kThreatLogEvery);
        if ((bb::st().frame % every) == 0) {
            if (std::FILE *f = bb::fp()) {
                std::fprintf(f, "TF,%ld,%d,%.3f,%.1f,%.3f,%d", bb::st().frame,
                             (int)kThreatFieldMode, tf.goal_D, tf.line_max_y, tf.line_max_v, tf.top);
                for (int k = 0; k < tf.n; ++k)
                    std::fprintf(f, ",%.3f,%.1f", tf.src[k].t, tf.src[k].sigma);
                std::fprintf(f, "\n");
            }
        }
    }
}

double threat_phi(const ThreatField &tf, double x, double y) {
    if (!tf.valid) return 0.0;
    double phi = 0.0;
    for (int k = 0; k < tf.n; ++k) {
        const ThreatSource &s = tf.src[k];
        const double dx = x - s.x, dy = y - s.y;
        phi += s.t * std::exp(-(dx * dx + dy * dy) * s.inv2s2);
    }
    return phi;
}

// 高斯求导：∂/∂x [t·exp(−r²/2σ²)] = −t·exp(…)·(x−cx)/σ²，所以梯度指向更亮（更危险）的一侧
double threat_grad(const ThreatField &tf, double x, double y, double *gx, double *gy) {
    double phi = 0.0, sx = 0.0, sy = 0.0;
    if (tf.valid) {
        for (int k = 0; k < tf.n; ++k) {
            const ThreatSource &s = tf.src[k];
            const double dx = x - s.x, dy = y - s.y;
            const double g = s.t * std::exp(-(dx * dx + dy * dy) * s.inv2s2);
            const double c = -2.0 * s.inv2s2 * g;      // = −g/σ²
            phi += g;
            sx += c * dx;
            sy += c * dy;
        }
    }
    if (gx) *gx = sx;
    if (gy) *gy = sy;
    return phi;
}

void threat_phi_batch(const ThreatField &tf, const double *xs, const double *ys, int n, double *out) {
    for (int i = 0; i < n; ++i) out[i] = threat_phi(tf, xs[i], ys[i]);
}

double threat_line_integral(const ThreatField &tf, double x, double y0, double y1, double step) {
    if (!tf.valid || y1 <= y0 || step <= 0.0) return 0.0;
    const int m = std::max(1, (int)std::ceil((y1 - y0) / step));
    const double h = (y1 - y0) / m;
    double sum = 0.5 * (threat_phi(tf, x, y0) + threat_phi(tf, x, y1));
    for (int i = 1; i < m; ++i) sum += threat_phi(tf, x, y0 + h * i);
    return sum * h;
}

double threat_line_max(const ThreatField &tf, double x, double y0, double y1, double step, double *out_y) {
    double best_v = 0.0, best_y = 0.5 * (y0 + y1);
    if (tf.valid && y1 >= y0 && step > 0.0) {
        const int m = std::max(1, (int)std::ceil((y1 - y0) / step));
        const double h = (y1 - y0) / m;
        best_v = -1.0;
        for (int i = 0; i <= m; ++i) {
            const double y = y0 + h * i;
            const double v = threat_phi(tf, x, y);
            if (v > best_v) { best_v = v; best_y = y; }
        }
    }
    if (out_y) *out_y = best_y;
    return best_v;
}

}  // namespace simuro5
