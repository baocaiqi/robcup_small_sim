# -*- coding: utf-8 -*-
"""sim_stats.py — 从 sim_bench 的轨迹 CSV 里算出"和真机同一口径"的统计量。

为什么需要：定标的本质是"让仿真的统计量贴近真机的统计量"。
真机那一侧由 tools/py/calib_probe.py 产出（口径见下），
仿真这一侧必须用**完全相同的口径**算，否则比的是两码事。

统计量（与 calib_probe.py 逐条对应）：
  ① 球自由滚动衰减：球离开所有机器人 ≥20cm，且 v1≥2cm/帧 → 按速度分档给比值中位
  ② 撞墙：贴墙(x<2.5/x>217.5 或 y<2.5/y>177.5) + 垂直分量变号 + 入射 ≥1.5cm/帧 + 20cm 内无机器人
     → 法向恢复系数、切向保持系数
  ③ 机器人速度分布（我方=blue，对手=yellow）：中位/p90/p95/p99（cm/帧）
  ④ 机器人加速度分布：p90/p95（cm/帧²）

用法：
    sim_bench.exe --games 1 --frames 24000 --traj build/calib_sim.csv
    python tools\\py\\sim_stats.py build/calib_sim.csv --out build/calib_sim.json
"""
import argparse
import csv
import json
import math
import sys

sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def load(path):
    """读 sim_bench --traj 的 CSV → 与 rlg_analyzer.parse_rlg 同结构。"""
    frames = []
    with open(path, newline="", encoding="utf-8") as f:
        rd = csv.reader(f)
        header = next(rd)
        # 表头形如 b0_x,b0_y,...,y4_x,y4_y,ball_x,ball_y（注意最后一个机器人后面有个空列）
        idx = {name.strip(): k for k, name in enumerate(header) if name.strip()}
        for row in rd:
            if len(row) < 22 or not row[0].strip():
                continue
            try:
                fr = {"blue": [], "yellow": [], "ball": {}}
                for i in range(5):
                    fr["blue"].append({"x": float(row[idx["b%d_x" % i]]), "y": float(row[idx["b%d_y" % i]])})
                    fr["yellow"].append({"x": float(row[idx["y%d_x" % i]]), "y": float(row[idx["y%d_y" % i]])})
                fr["ball"] = {"x": float(row[idx["ball_x"]]), "y": float(row[idx["ball_y"]])}
                frames.append(fr)
            except (ValueError, KeyError):
                continue
    return frames


def med(a):
    return sorted(a)[len(a) // 2] if a else float("nan")


def pct(a, p):
    s = sorted(a)
    return s[min(len(s) - 1, int(len(s) * p))] if a else float("nan")


def wall_bounce_fit(frames, near=4.0, robot_free=15.0, goal_margin=6.0,
                    wall_x=220.0, wall_y=180.0):
    """用"反弹帧前后各 2 帧"估撞墙恢复系数（比老的"位移变号法"可靠）。

    为什么必须换方法（实测教训）：
      仿真里球是**同一帧内**被弹回的（`s.bx = -s.bx`），位置序列**不变号**，
      所以"位移变号"探测器几乎测不到仿真样本，量出 0.294 而参数其实是 0.66 ——
      这是**测量假象**，不是物理差异。若拿假象去定标，优化器会把 kWallRest 越调越低。

    做法：找球在法向坐标上的**局部极小/极大**（= 贴墙那一帧 j），
      入射 = (p[j-1] - p[j-3]) / 2     出射 = (p[j+3] - p[j+1]) / 2
      恢复系数 = |出射法向| / |入射法向|（切向同理）
    自检：对当前仿真（kWallRest=0.66）应能还原出 ≈0.66，否则说明测法还有问题。
    """
    rest_x, rest_y, fric_x, fric_y = [], [], [], []
    n = len(frames)
    for j in range(3, n - 3):
        b = frames[j]["ball"]
        robots = frames[j]["blue"] + frames[j]["yellow"]
        if min(math.hypot(r["x"] - b["x"], r["y"] - b["y"]) for r in robots) < robot_free:
            continue
        x, y = b["x"], b["y"]
        # —— x 墙（左/右）：避开球门开口（y∈门宽±余量），否则会把"进球"当成反弹
        if not (65.0 <= y <= 115.0) and (x < near or x > wall_x - near):
            xm1, xm3 = frames[j - 1]["ball"]["x"], frames[j - 3]["ball"]["x"]
            xp1, xp3 = frames[j + 1]["ball"]["x"], frames[j + 3]["ball"]["x"]
            if (x - xm1) * (xp1 - x) <= 0:      # x 在相邻两帧间是局部极值（贴墙那一帧）
                vin = (xm1 - xm3) / 2.0
                vout = (xp3 - xp1) / 2.0
                yin = (frames[j - 1]["ball"]["y"] - frames[j - 3]["ball"]["y"]) / 2.0
                yout = (frames[j + 3]["ball"]["y"] - frames[j + 1]["ball"]["y"]) / 2.0
                if abs(vin) > 0.8 and vin * vout < 0:
                    rest_x.append(abs(vout) / abs(vin))
                    if abs(yin) > 0.5:
                        fric_x.append(abs(yout) / abs(yin))
        # —— y 墙（下/上）
        if y < near or y > wall_y - near:
            ym1, ym3 = frames[j - 1]["ball"]["y"], frames[j - 3]["ball"]["y"]
            yp1, yp3 = frames[j + 1]["ball"]["y"], frames[j + 3]["ball"]["y"]
            if (y - ym1) * (yp1 - y) <= 0:
                vin = (ym1 - ym3) / 2.0
                vout = (yp3 - yp1) / 2.0
                xin = (frames[j - 1]["ball"]["x"] - frames[j - 3]["ball"]["x"]) / 2.0
                xout = (frames[j + 3]["ball"]["x"] - frames[j + 1]["ball"]["x"]) / 2.0
                if abs(vin) > 0.8 and vin * vout < 0:
                    rest_y.append(abs(vout) / abs(vin))
                    if abs(xin) > 0.5:
                        fric_y.append(abs(xout) / abs(xin))
    return dict(rest_x=med(rest_x), rest_y=med(rest_y),
                fric_x=med(fric_x), fric_y=med(fric_y),
                n_rest_x=len(rest_x), n_rest_y=len(rest_y),
                _raw=dict(rest_x=rest_x, rest_y=rest_y, fric_x=fric_x, fric_y=fric_y))


def compute(frames):
    decay = []
    ball_spd = []
    spd = {"blue": [], "yellow": []}
    acc = {"blue": [], "yellow": []}
    n = len(frames)
    for i in range(2, n):
        b0, b1, b2 = frames[i - 2]["ball"], frames[i - 1]["ball"], frames[i]["ball"]
        dx1, dy1 = b1["x"] - b0["x"], b1["y"] - b0["y"]
        dx2, dy2 = b2["x"] - b1["x"], b2["y"] - b1["y"]
        v1 = math.hypot(dx1, dy1)
        v2 = math.hypot(dx2, dy2)
        ball_spd.append(v1)
        robots = frames[i - 1]["blue"] + frames[i - 1]["yellow"]
        near = min(math.hypot(r["x"] - b1["x"], r["y"] - b1["y"]) for r in robots)
        if near >= 20.0 and v1 >= 2.0 and v2 >= 1.0:
            decay.append((v1, v2 / v1))
    for i in range(2, n):
        for side in ("blue", "yellow"):
            for j in range(5):
                p2, p1, p0 = frames[i - 2][side][j], frames[i - 1][side][j], frames[i][side][j]
                d1 = math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])
                d2 = math.hypot(p2["x"] - p1["x"], p2["y"] - p1["y"])
                spd[side].append(d1)
                if d1 > 0.2 or d2 > 0.2:
                    acc[side].append(d1 - d2)
    wall = wall_bounce_fit(frames)      # 用"反弹帧前后拟合"测撞墙（老方法在仿真上测不到）
    out = {
        "decay_all": med([r for _, r in decay]),
        "decay_2_3": med([r for v, r in decay if 2 <= v < 3]),
        "decay_3_5": med([r for v, r in decay if 3 <= v < 5]),
        "decay_5p": med([r for v, r in decay if v >= 5]),
        "rest_x": wall["rest_x"], "rest_y": wall["rest_y"],
        "fric_x": wall["fric_x"], "fric_y": wall["fric_y"],
        "n_rest_x": wall["n_rest_x"], "n_rest_y": wall["n_rest_y"], "n_decay": len(decay),
        "frames": n,
    }
    for side in ("blue", "yellow"):
        out[side + "_spd_p50"] = pct(spd[side], 0.5)
        out[side + "_spd_p90"] = pct(spd[side], 0.9)
        out[side + "_spd_p95"] = pct(spd[side], 0.95)
        out[side + "_spd_p99"] = pct(spd[side], 0.99)
        out[side + "_acc_p90"] = pct(acc[side], 0.9)
        out[side + "_acc_p95"] = pct(acc[side], 0.95)
    out["ball_spd_p50"] = pct(ball_spd, 0.5)
    out["ball_spd_p90"] = pct(ball_spd, 0.9)
    out["ball_spd_p95"] = pct(ball_spd, 0.95)
    out["ball_spd_p99"] = pct(ball_spd, 0.99)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("traj")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    frames = load(a.traj)
    st = compute(frames)
    print(f"帧数 {st['frames']}  衰减样本 {st['n_decay']}  撞墙样本 x={st['n_rest_x']} y={st['n_rest_y']}")
    print(f"球衰减: 全体={st['decay_all']:.4f}  [2,3)={st['decay_2_3']:.4f}  "
          f"[3,5)={st['decay_3_5']:.4f}  [5,∞)={st['decay_5p']:.4f}")
    print(f"撞墙: 法向 x={st['rest_x']:.3f} y={st['rest_y']:.3f}  切向 x={st['fric_x']:.3f} y={st['fric_y']:.3f}")
    for side, label in (("blue", "我方(sim)"), ("yellow", "对手(sim)")):
        print(f"{label} 速度 中位={st[side+'_spd_p50']:.2f} p90={st[side+'_spd_p90']:.2f} "
              f"p95={st[side+'_spd_p95']:.2f} p99={st[side+'_spd_p99']:.2f} | "
              f"加速度 p90={st[side+'_acc_p90']:.3f} p95={st[side+'_acc_p95']:.3f}")
    if a.out:
        with open(a.out, "w", encoding="utf-8") as f:
            json.dump(st, f, ensure_ascii=False, indent=1)
        print("→", a.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
