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


def compute(frames):
    decay = []
    ball_spd = []
    rest_x, rest_y, fric_x, fric_y = [], [], [], []
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
        free = near >= 20.0
        if free and v1 >= 2.0 and v2 >= 1.0:
            decay.append((v1, v2 / v1))
        if free and (b1["x"] < 2.5 or b1["x"] > 217.5) and dx1 * dx2 < 0 and abs(dx1) >= 1.5:
            rest_x.append(abs(dx2) / abs(dx1))
            fric_x.append(abs(dy2) / max(abs(dy1), 1e-9))
        if free and (b1["y"] < 2.5 or b1["y"] > 177.5) and dy1 * dy2 < 0 and abs(dy1) >= 1.5:
            rest_y.append(abs(dy2) / abs(dy1))
            fric_y.append(abs(dx2) / max(abs(dx1), 1e-9))
    for i in range(2, n):
        for side in ("blue", "yellow"):
            for j in range(5):
                p2, p1, p0 = frames[i - 2][side][j], frames[i - 1][side][j], frames[i][side][j]
                d1 = math.hypot(p1["x"] - p0["x"], p1["y"] - p0["y"])
                d2 = math.hypot(p2["x"] - p1["x"], p2["y"] - p1["y"])
                spd[side].append(d1)
                if d1 > 0.2 or d2 > 0.2:
                    acc[side].append(d1 - d2)
    out = {
        "decay_all": med([r for _, r in decay]),
        "decay_2_3": med([r for v, r in decay if 2 <= v < 3]),
        "decay_3_5": med([r for v, r in decay if 3 <= v < 5]),
        "decay_5p": med([r for v, r in decay if v >= 5]),
        "rest_x": med(rest_x), "rest_y": med(rest_y),
        "fric_x": med(fric_x), "fric_y": med(fric_y),
        "n_rest_x": len(rest_x), "n_rest_y": len(rest_y), "n_decay": len(decay),
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
