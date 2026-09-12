"""motion_calib.py — 从 rlg 轨迹 CSV 标定平台运动学常数（运动控制优化用）

用途：制动曲线 v <= sqrt(2*a*s) 需要平台的加减速度上限 a；"轮速单位 -> cm/s" 的换算
      需要稳态速度与命令值的对应。这些常数不能拿 sim_bench 的近似值凑（docs/03 教训：
      sim 数字与真机方向可能相反），必须从真机 rlg 轨迹量出来。

用法：
    python tools/py/motion_calib.py build/traj_before.csv [--side b|y|both]

输入 CSV 列名（build/traj_before.csv 同款，40Hz，cm）：
    b0_x,b0_y,...,b4_x,b4_y, y0_x,y0_y,...,y4_x,y4_y, ball_x,ball_y

估计方法与可信度：
  1) 速度：v[k] = |p_k - p_{k-1}|/DT。机器人正常速度 <= ~4cm/帧，跳变门限取
     JUMP=6cm/帧（240cm/s）——比"物理探测脚本"的 20cm/帧 严得多，否则开球/定位球
     重置的瞬移（可达 20cm/帧）会污染速度与加速度统计（实测 max 798cm/s 就是这么来的）。
  2) 加速度：单帧 a=Δv/DT 的二阶差分噪声底噪 ≈ sqrt(2)·σ_x/DT²，40Hz 下可达数百
     cm/s²，不可用。改为 0.1s 尺度（4 帧跨度）的中心差分：
        a4[k] = (vs[k+2] - vs[k-2]) / (4·DT)，vs = 3 点滑动平均速度
     并**用静止帧做噪声底噪对照**：若运动帧的 |a4| 明显高于静止帧底噪，才可信。
     报告的是"实测可持续加速度"的下界（不知道当时是否命令了极限）。
  3) 制动能力：a4 的负向分位数（减速）。
  4) 来回蹭(limit cycle 代理)：低速段位移矢量方向翻转次数 + 静止带内累计路程/净位移。

输出全部 ASCII，避免 GBK 控制台乱码。
"""
import argparse
import csv
import math
import sys

DT = 1.0 / 40.0
JUMP = 6.0         # cm/帧：机器人跳变门限（正常 <=~4cm/帧 = 160cm/s）
V_REST = 5.0       # cm/s：静止带
V_MOVE = 20.0      # cm/s：运动帧判定
REV_LO = 0.5       # cm：翻转两段的各自最小位移
REV_HI = 6.0       # cm：两段位移之和上限（超过算正常行驶，不算蹭）


def pct(vals, p):
    if not vals:
        return 0.0
    s = sorted(vals)
    return s[min(len(s) - 1, int(len(s) * p))]


def load(path):
    with open(path, encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def series(rows, key):
    """位置序列 -> (pts, v, ok)。v[k] = k-1 -> k 帧速度；越界/跳变处置 ok=False。"""
    pts = [(float(r[f"{key}_x"]), float(r[f"{key}_y"])) for r in rows]
    v = [0.0] * len(pts)
    ok = [False] * len(pts)
    for k in range(1, len(pts)):
        d = math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1])
        if d <= JUMP:
            v[k] = d / DT
            ok[k] = True
    return pts, v, ok


def smooth(v, ok, w=1):
    """3 点滑动平均；窗口内任一帧无效则该点无效。"""
    out = [0.0] * len(v)
    good = [False] * len(v)
    for k in range(w, len(v) - w):
        if all(ok[j] for j in range(k - w, k + w + 1)):
            out[k] = sum(v[j] for j in range(k - w, k + w + 1)) / (2 * w + 1)
            good[k] = True
    return out, good


def accel_stats(vs, good):
    """0.1s 尺度加速度：a4[k] = (vs[k+2]-vs[k-2])/(4DT)。按静止/运动分组。"""
    a_moving, a_rest = [], []
    for k in range(2, len(vs) - 2):
        if not all(good[j] for j in range(k - 2, k + 3)):
            continue
        a = (vs[k + 2] - vs[k - 2]) / (4 * DT)
        if max(abs(vs[j]) for j in range(k - 2, k + 3)) < V_REST:
            a_rest.append(abs(a))
        elif min(abs(vs[j]) for j in range(k - 2, k + 3)) > V_MOVE:
            a_moving.append(a)
    return a_moving, a_rest


def reversals(pts, ok):
    """低速来回蹭：3 帧位移矢量与下一帧位移矢量夹角 >90°（点积<0），
    两段位移各自 >=REV_LO、之和 <=REV_HI。"""
    cnt = 0
    for k in range(4, len(pts)):
        if not (ok[k] and ok[k - 1] and ok[k - 3]):
            continue
        ax, ay = pts[k - 1][0] - pts[k - 3][0], pts[k - 1][1] - pts[k - 3][1]
        bx, by = pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1]
        la, lb = math.hypot(ax, ay), math.hypot(bx, by)
        if la < REV_LO or lb < REV_LO or la + lb > REV_HI:
            continue
        if ax * bx + ay * by < 0.0:
            cnt += 1
    return cnt


def hunting(pts, v, ok):
    """静止带内连续 >=8 帧片段：累计路程 / 净位移 / 帧数。"""
    dist = net = 0.0
    frames = 0
    start = None
    for k in range(1, len(v)):
        if ok[k] and v[k] < V_REST:
            if start is None:
                start = k - 1
            continue
        if start is not None and k - start >= 8:
            dist += sum(math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
                        for i in range(start + 1, k) if ok[i])
            net += math.hypot(pts[k - 1][0] - pts[start][0], pts[k - 1][1] - pts[start][1])
            frames += k - start
        start = None
    return dist, net, frames


def report(rows, side, label):
    keys = [f"{side}{i}" for i in range(5)]
    all_v, a_moving, a_rest = [], [], []
    h_dist = h_net = 0.0
    h_frames = 0
    rev = 0
    for key in keys:
        pts, v, ok = series(rows, key)
        all_v.extend(x for x, o in zip(v, ok) if o)
        vs, good = smooth(v, ok)
        am, ar = accel_stats(vs, good)
        a_moving.extend(am)
        a_rest.extend(ar)
        d, n, f = hunting(pts, v, ok)
        h_dist += d
        h_net += n
        h_frames += f
        rev += reversals(pts, ok)

    print(f"--- {label} ({side}) ---")
    print(f"  speed cm/s   : p50={pct(all_v,.5):6.1f} p90={pct(all_v,.9):6.1f} "
          f"p99={pct(all_v,.99):6.1f} max={max(all_v) if all_v else 0:7.1f}")
    print(f"  accel p95/p99 (0.1s scale): moving={pct(a_moving,.95):7.0f}/{pct(a_moving,.99):7.0f}  "
          f"noise-floor(rest)={pct(a_rest,.95):6.0f}/{pct(a_rest,.99):6.0f} cm/s^2  "
          f"n_move={len(a_moving)} n_rest={len(a_rest)}")
    neg = [-x for x in a_moving if x < 0]
    if neg:
        print(f"  deceleration (|a|, negative side): p50={pct(neg,.5):6.0f} p90={pct(neg,.9):6.0f} "
              f"p95={pct(neg,.95):6.0f} cm/s^2")
    if h_frames > 0:
        ratio = (h_dist / h_net) if h_net > 1e-6 else float("inf")
        print(f"  rest band |v|<{V_REST:.0f} >=8 frames: frames={h_frames} "
              f"path={h_dist:.0f}cm net={h_net:.0f}cm path/net={ratio:.2f}")
    print(f"  low-speed direction reversals: n={rev}  "
          f"({rev/5/(len(rows)*DT/60.0):.1f} per robot per minute)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--side", choices=["b", "y", "both"], default="both")
    args = ap.parse_args()
    rows = load(args.csv)
    print(f"===== motion_calib: {args.csv}, {len(rows)} frames ~ {len(rows)*DT:.0f}s =====")
    if args.side in ("b", "both"):
        report(rows, "b", "ours(blue)")
    if args.side in ("y", "both"):
        report(rows, "y", "opp(yellow)")


if __name__ == "__main__":
    sys.exit(main())
