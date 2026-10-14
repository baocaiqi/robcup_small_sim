// threat_field.hpp — 对手威胁场：场上任意一点"对我方球门有多危险"（原点左下角 cm，颜色无关）
// 参考"高斯威胁势场"思想（对手 Strategy4Blue.dll，见 _re/spec/ThreatField-规格.md），自研实现：
//   亮度用我们自己的 mark_threat() 打分归一化，光圈 = r0 + 速度×τ（cm/帧口径），不沿用对手的分量与权重。
// 每帧由 WorldModel::update 末尾构建一次；调用方只读 wm.threat。valid=false 时所有查询返回 0。
#ifndef SIMURO5_THREAT_FIELD_HPP
#define SIMURO5_THREAT_FIELD_HPP

#include "simuro5/simuro_interface.hpp"

namespace simuro5 {

struct WorldModel;

// 一个"灯泡"：中心 (x,y)、亮度 t∈[0,1]、光圈 sigma(cm)；inv2s2 = 1/(2σ²) 预先算好
struct ThreatSource {
    double x = 0, y = 0;
    double t = 0, sigma = 1;
    double inv2s2 = 0.5;
};

struct ThreatField {
    bool valid = false;
    int  n = 0;                                    // 有效灯泡数：对手 5 个 + 球 1 个（球项可关）
    ThreatSource src[PLAYERS_PER_SIDE + 1];
    int  top = -1;                                 // 亮度最高的对手下标（0~4）
    // 门线汇总（影子日志与门将用）：D = 门线积分，line_max_y/v = 门线上最危险的 y 和该点 phi
    double goal_D = 0, line_max_y = 90, line_max_v = 0;
};

// 每帧构建（WorldModel::update 末尾调用）；kThreatFieldMode=0 时只置 valid=false
void threat_build(ThreatField &tf, const WorldModel &wm);

// 场值 / 解析梯度（返回 phi）/ 批量场值
double threat_phi (const ThreatField &tf, double x, double y);
double threat_grad(const ThreatField &tf, double x, double y, double *gx, double *gy);
void   threat_phi_batch(const ThreatField &tf, const double *xs, const double *ys, int n, double *out);

// 竖线 x 上 y∈[y0,y1] 的梯形积分 / 最大值（out_y 回填最危险的 y），步长 step cm
double threat_line_integral(const ThreatField &tf, double x, double y0, double y1, double step);
double threat_line_max(const ThreatField &tf, double x, double y0, double y1, double step, double *out_y);

// 0=关 1=影子(只算+写黑匣子，默认) 2=门线站位生效（门将另有 roles.kGkThreatBlend 直接读 line_max_y）
int    threat_field_mode();
double threat_gk_blend();

}  // namespace simuro5
#endif
