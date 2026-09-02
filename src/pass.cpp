#include "simuro5/pass.hpp"
#include "simuro5/geometry.hpp"
#include "simuro5/field_info.hpp"
#include "simuro5/role_assignment.hpp"
#include <cmath>
#include <algorithm>

namespace simuro5 {

namespace {
// ============================================================
// 调参常量（docs/14 图论链规划 方案 A+B；改动后同步 docs/06-调参记录.md）
// ============================================================
constexpr double PASS_MAX_DIST     = 60.0;    // 单段接力推球最大距离 cm
constexpr double PASS_MIN_DIST     = 8.0;     // 单段最小距离，避免贴脸
constexpr double BLOCK_THRESHOLD   = 15.0;    // 车道硬剔除阈值 cm（安全口径不变）
constexpr double OFFSET_BASE       = 6.0;     // 第一档领球偏移 cm
constexpr double DEPTH_STEP        = 15.0;    // 深度档距 cm（d1/d2 = 向前空间阶段点）
constexpr int    MAX_DEPTH         = 2;       // 档 0..2
constexpr double THREAT_RADIUS     = 30.0;    // 终点威胁判定半径 cm
constexpr double FIELD_MARGIN      = 6.0;     // 阶段点离边线最小距离 cm

// —— 图论链规划权重（初值：与旧打分同量级，A/B 调参见 docs/06）——
constexpr double K_LEN         = 0.5;    // 距离代价（对齐旧 pass_dist×0.5）
constexpr double K_RISK        = 12.0;   // 车道连续风险权重（方案 B）
constexpr double K_RISK_CAP    = 4.0;    // 单对手风险封顶
constexpr double K_END         = 20.0;   // 终点威胁权重（对齐旧 threat×20）
constexpr double K_FRONT       = 12.0;   // 终点前方威胁权重（对齐旧 front_threat×12）
constexpr double K_PROG        = 0.3;    // 向门推进收益：每段 Δgoal_dist 的权重
constexpr double GAMMA         = 0.8;    // 链折扣：偏好更早把球送入射门区
// 实验旋钮（docs/06 第 19 轮）：0 = 关链（隔离实验，已跑 graphnohops 批）；
// ≥1 = 开链。当前提交值 = 3（v1.1 完整链配置；sim 净效果为负 → 分支暂存待调）。
constexpr int    LOOKAHEAD_HOPS = 3;     // 阶段点剩余可接力跳数（3×60cm 覆盖中场→射程）
constexpr double SHOOT_RANGE   = 70.0;   // 镜像 plan_shoot 射程
constexpr double kInf          = 1e6;    // 不可达 / 死胡同

// 路线 (sx,sy)->(tx,ty) 是否被某个对手硬挡住（点-线段距离 < 阈值）
bool route_blocked(const WorldModel &wm, double sx, double sy, double tx, double ty) {
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = point_to_segment_dist(wm.opp[i].x, wm.opp[i].y, sx, sy, tx, ty);
        if (d < BLOCK_THRESHOLD) {
            return true;
        }
    }
    return false;
}

// 统计目标点周围敌方机器人的「距离加权威胁度」：越近权重越高，
// 贴脸(d=0)≈1.0、半径边缘(d=THREAT_RADIUS)≈0.0，区分「被紧盯」与「附近路过」。
double count_near_opponent(const WorldModel &wm, double x, double y)
{
    const double r2 = THREAT_RADIUS * THREAT_RADIUS;
    double threat = 0.0;
    for(int i=0;i<PLAYERS_PER_SIDE;i++)
    {
        double dx = wm.opp[i].x - x;
        double dy = wm.opp[i].y - y;
        double d2 = dx*dx + dy*dy;
        if(d2 < r2)
        {
            threat += 1.0 - d2 / r2;   // 贴脸≈1.0，边缘≈0.0
        }
    }
    return threat;
}

// 统计目标点周围「前方威胁」：比目标点更靠对方球门的对手
int count_front_opponent(const WorldModel &wm, double x, double y, double ad) {
    int cnt = 0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double dx = wm.opp[i].x - x, dy = wm.opp[i].y - y;
        if (dx*dx + dy*dy < THREAT_RADIUS * THREAT_RADIUS) {
            if (ad * (wm.opp[i].x - x) > 0.0) cnt++;   // 前方 = 对方球门方向
        }
    }
    return cnt;
}

// 把阶段点夹回场内，并避免深入对方罚球区（带球/接应不该进禁区）。
void clamp_receive_point(const TeamContext &ctx, double &x, double &y) {
    x = clamp(x, FIELD_MARGIN, TeamContext::FIELD_LENGTH - FIELD_MARGIN);
    y = clamp(y, FIELD_MARGIN, TeamContext::FIELD_WIDTH - FIELD_MARGIN);
    if (in_opp_penalty_area(ctx, x, y)) {
        // 落在对方罚球区内：沿 x 推到禁区外沿（朝持球者一侧）
        double gx = ctx.opp_goal_x();
        x = (ctx.attack_dir() > 0) ? gx - 80.0 - FIELD_MARGIN : gx + 80.0 + FIELD_MARGIN;
    }
}

// ============================================================
// docs/14 方案 B：车道连续风险
// 旧 route_blocked 是二值（<15cm 直接丢），可行路线之间没有优劣之分，
// 会随机选中「擦着对手 16cm 过」的险线。本函数给每条可行线一个连续风险：
//   risk(d) = (BLOCK/d)^2，d=15→1.0、d=30→0.25、d=60→0.06，单对手封顶 K_RISK_CAP。
// 硬剔除仍在 hop_cost 里保留（<15cm 不成边），这里只给可行边排序，不放大冒险。
// ============================================================
double lane_risk(const WorldModel &wm, double sx, double sy, double tx, double ty) {
    double risk = 0.0;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = point_to_segment_dist(wm.opp[i].x, wm.opp[i].y, sx, sy, tx, ty);
        if (d >= BLOCK_THRESHOLD) {
            double r = BLOCK_THRESHOLD / std::max(d, 1e-6);
            risk += std::min(r * r, K_RISK_CAP);
        }
    }
    return risk;
}

// 单段接力 u→v 的代价；不可行（距离越限 / 车道被硬挡）返回 kInf。
//   代价 = 距离 + 车道风险 + 终点威胁 + 前方威胁 + 推进惩罚（越靠近对方门越省）。
double hop_cost(const WorldModel &wm, double ux, double uy, double vx, double vy) {
    double d = dist(ux, uy, vx, vy);
    if (d < PASS_MIN_DIST || d >= PASS_MAX_DIST) return kInf;
    const TeamContext &ctx = wm.ctx;
    double ad = ctx.attack_dir();
    if (route_blocked(wm, ux, uy, vx, vy)) return kInf;
    double cost = K_LEN * d
                + K_RISK * lane_risk(wm, ux, uy, vx, vy)
                + K_END   * count_near_opponent(wm, vx, vy)
                + K_FRONT * static_cast<double>(count_front_opponent(wm, vx, vy, ad))
                + K_PROG  * (std::fabs(vx - ctx.opp_goal_x()) - std::fabs(ux - ctx.opp_goal_x()));
    return cost;
}

// 球位 (x,y) 是否已进入「可射门」状态（接力链终结条件，镜像 plan_shoot 口径）：
//   射程内（距门 ≤ SHOOT_RANGE）且 球→门柱开口的车道净空达标。
bool shootable_from(const WorldModel &wm, double x, double y) {
    const TeamContext &ctx = wm.ctx;
    double ogx = ctx.opp_goal_x();
    double ad = ctx.attack_dir();
    double dgoal = dist(x, y, ogx, 90.0);
    if (dgoal > SHOOT_RANGE || dgoal < 5.0) return false;
    // 找对方守门员（离门最近者）定开口（同 plan_shoot）
    double gk_y = 90.0, gk_best = 1e9;
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        double d = std::fabs(wm.opp[i].x - ogx);
        if (d < gk_best) { gk_best = d; gk_y = wm.opp[i].y; }
    }
    double half = TeamContext::GOAL_WIDTH / 2.0;
    double aim_up   = 90.0 + half - 4.0;    // 106 上柱内侧
    double aim_down = 90.0 - half + 4.0;    // 74  下柱内侧
    double aim_y = (std::fabs(aim_up - gk_y) >= std::fabs(aim_down - gk_y)) ? aim_up : aim_down;
    double gx = ogx + ad * 5.0;             // 门线上前 5cm（与 shoot.cpp 推球方向同口径）
    for (int i = 0; i < PLAYERS_PER_SIDE; ++i) {
        if (point_to_segment_dist(wm.opp[i].x, wm.opp[i].y, x, y, gx, aim_y) < BLOCK_THRESHOLD)
            return false;
    }
    return true;
}

// 接力阶段点：球可以被推到的位置（含 d0 队友接应位与 d1/d2 向前空间点）
struct RelayNode {
    double x, y;
    int home_id;      // 归属队友（d0 的接收者；d1/d2 空间点归最近的队友档位）
    bool hop1;        // 是否允许作为第一跳目标（只放行 d0，见 docs/14 §3）
};

// 建图：每个可接应队友（除持球者）沿进攻方向生成 0..MAX_DEPTH 档阶段点，
// 全部过 clamp_receive_point（场内 + 对方罚球区外）+ 对方门区禁入（站位纪律镜像）。
// 同点去重（夹取可能使多档/多人重合）；若重合点来自某队友 d0，则保留 hop1 资格。
void build_relay_nodes(const WorldModel &wm, int passer_id, RelayNode *nodes, int &n) {
    const TeamContext &ctx = wm.ctx;
    double ad = ctx.attack_dir();
    n = 0;
    for (int id = 0; id < PLAYERS_PER_SIDE; ++id) {
        if (id == passer_id) continue;
        // —— 阶段点基准：优先用 A 输出的角色站位点（联动），其余回退本体坐标 ——
        double base_x = wm.home[id].x, base_y = wm.home[id].y;
        switch (wm.role[id]) {
            case ROLE_ASSIST:   base_x = wm.assist_x;  base_y = wm.assist_y;  break;
            case ROLE_MIDFIELD: base_x = wm.mid_x;     base_y = wm.mid_y;     break;
            case ROLE_PASSIVE:  base_x = wm.passive_x; base_y = wm.passive_y; break;
            default: break;   // GOALIE / 未分配：无站位点，用本体坐标（天然被距离滤除）
        }
        for (int d = 0; d <= MAX_DEPTH; ++d) {
            double tx = base_x + ad * (OFFSET_BASE + d * DEPTH_STEP);
            double ty = base_y;
            clamp_receive_point(ctx, tx, ty);
            // 对方门区禁入：阶段点是「机器人要去接球的位置」，同样受站位纪律约束
            // （球可以进对方门区去射门，但阶段点不站人）。
            if (in_opp_goal_area(ctx, tx, ty))
                opp_goal_area_front(ctx, ty, tx, ty);
            bool dup = false;
            for (int k = 0; k < n; ++k) {
                if (std::fabs(nodes[k].x - tx) < 0.01 && std::fabs(nodes[k].y - ty) < 0.01) {
                    dup = true;
                    if (d == 0) nodes[k].hop1 = true;   // 后到的 d0 与已有阶段点重合
                    break;
                }
            }
            if (dup) continue;
            nodes[n].x = tx; nodes[n].y = ty;
            nodes[n].home_id = id;
            nodes[n].hop1 = (d == 0);
            ++n;
        }
    }
}

}  // anonymous namespace

// ============================================================
// plan_pass — 多跳接力链规划（docs/14 方案 A+B）
//
// 把「这次把球推到哪」建成加权有向接力图：
//   · 源   ：球当前位置（物理真实起点；无球数据时回退持球者位置）
//   · 节点 ：每个队友 3 档深度阶段点（d0=现状接应位，d1/d2=向前空间）
//   · 边   ：单段推球可行（8~60cm + 车道净空 ≥15cm + 阶段点纪律）
//   · 代价 ：距离 + 车道连续风险(方案B) + 终点威胁 + 推进惩罚
//   · 价值 ：预算式价值迭代 R[k][u] = 剩余 k 跳内把球送到「可射门」的最小代价；
//           终结 R[0]=0（可射门），预算耗尽仍不可达 = 死胡同(kInf)，
//           跳数预算天然排除回环（循环无法在预算内到达射门区）。
// 输出仍只有第一跳 d0 目标（receiver_id/target），roles.cpp 调用点零改动。
// ============================================================
PassPlan plan_pass(const WorldModel &wm, int passer_id) {
    PassPlan plan{};
    const TeamContext &ctx = wm.ctx;

    // 接力源 = 球位置（比旧「持球者本体坐标」更接近物理真实；无球数据回退）
    double sx, sy;
    if (wm.ball.valid) { sx = wm.ball.x; sy = wm.ball.y; }
    else               { sx = wm.home[passer_id].x; sy = wm.home[passer_id].y; }

    RelayNode node[1 + PLAYERS_PER_SIDE * (MAX_DEPTH + 1)];
    int n = 0;
    build_relay_nodes(wm, passer_id, node, n);
    if (n == 0) return plan;

    // —— 预算式价值迭代：R[k][u] = 从 u 出发、剩余 k 跳内到「可射门」的最小代价 ——
    // 注意：kInf 是哨兵，γ·kInf < kInf 会把死胡同"算成有限"——凡 R 为 INF 的选项
    // 必须显式跳过（不参与求和），见下三处 `>= kInf/2` 守卫。
    const int K = LOOKAHEAD_HOPS;
    double R[LOOKAHEAD_HOPS + 1][1 + PLAYERS_PER_SIDE * (MAX_DEPTH + 1)];
    for (int i = 0; i < n; ++i)
        R[0][i] = shootable_from(wm, node[i].x, node[i].y) ? 0.0 : kInf;
    for (int k = 1; k <= K; ++k) {
        for (int i = 0; i < n; ++i) {
            if (shootable_from(wm, node[i].x, node[i].y)) { R[k][i] = 0.0; continue; }
            double best = R[k - 1][i];            // 少跳一步的既有可达性兜底
            for (int j = 0; j < n; ++j) {
                if (i == j) continue;
                if (R[k - 1][j] >= kInf / 2) continue;   // 死胡同不参与求和
                double c = hop_cost(wm, node[i].x, node[i].y, node[j].x, node[j].y);
                if (c >= kInf / 2) continue;
                double t = c + GAMMA * R[k - 1][j];
                if (t < best) best = t;
            }
            R[k][i] = best;
        }
    }

    // —— 决策：第一跳只从 d0 阶段点里选（传给队友当前接应位，安全口径与旧版一致）——
    // 择优分两级（docs/14 风险表修订，v1.1）：
    //   ① 优先选「活链」d0（链能在预算内把球送入射门区）；活链之间按总代价比。
    //   ② 若所有可行 d0 都是死胡同（R=∞）→ **回退旧版行为**：仍传最优 d0（保底回传/
    //      安全推进，不把 ACTIVE 逼成"只能闷头前压"——v1.0 因此致失球与门区停留上升，
    //      sim A/B 实测 4.9:6.5、单人停留 4.2→9.0 次/场，见 docs/06 第 19 轮）。
    int best = -1, legacy = -1;
    double best_total = kInf, legacy_c = kInf;
    for (int i = 0; i < n; ++i) {
        if (!node[i].hop1) continue;
        double c = hop_cost(wm, sx, sy, node[i].x, node[i].y);
        if (c >= kInf / 2) continue;
        if (c < legacy_c) { legacy_c = c; legacy = i; }      // 保底候选（不看链价值）
        if (R[K][i] >= kInf / 2) continue;                    // 死胡同不进活链择优
        double total = c + GAMMA * R[K][i];
        if (total < best_total) { best_total = total; best = i; }
    }
    if (best < 0) best = legacy;            // 全死胡同 → 回退旧行为
    if (best < 0) {
        return plan;   // 无任何可行 d0 → 不传（沿用旧行为：带球/撤退）
    }
    const bool fallback = (best_total >= kInf / 2);   // 全死胡同 → 保底 d0（旧行为）

    // —— 链回溯（调试/测试信息：chain_value/chain_len/hop2_id）——
    // 沿 R 的最优路径走：若多给一跳没有改进（R[b] == R[b-1]），最优路径不消耗本层
    // 预算 → 纯降预算重查；有改进则取达到 R[b][cur] 的最优下一跳。
    int cur = best, budget = K, hops = 1, hop2 = -1;
    while (budget > 0 && !shootable_from(wm, node[cur].x, node[cur].y)) {
        if (R[budget][cur] >= R[budget - 1][cur] - 1e-9) { --budget; continue; }
        int nxt = -1;
        double nc = R[budget][cur] + 1e-9;   // 允许取到 == R[b][cur] 的最优下一跳
        for (int j = 0; j < n; ++j) {
            if (j == cur) continue;
            if (R[budget - 1][j] >= kInf / 2) continue;
            double c = hop_cost(wm, node[cur].x, node[cur].y, node[j].x, node[j].y);
            if (c >= kInf / 2) continue;
            double t = c + GAMMA * R[budget - 1][j];
            if (t < nc) { nc = t; nxt = j; }
        }
        if (nxt < 0) break;
        if (hop2 < 0) hop2 = node[nxt].home_id;
        cur = nxt;
        --budget;
        ++hops;
    }

    plan.viable = true;
    plan.receiver_id = node[best].home_id;
    plan.target_x = node[best].x;
    plan.target_y = node[best].y;
    plan.chain_value = fallback ? kInf : best_total;
    plan.chain_len = fallback ? 1 : hops;
    plan.hop2_id = fallback ? -1 : hop2;
    return plan;
}

} // namespace simuro5
