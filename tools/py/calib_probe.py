# -*- coding: utf-8 -*-
"""calib_probe.py — 定标数据的"体检"：先确认测量可信，再拿去改仿真。

背景（为什么需要这个脚本）：sim_calib.py 量出"球自由滚动衰减中位 = 1.0000"，
即球在真机上**完全不减速**——这在物理上不可能，最可能是 **.rlg 里的球坐标被量化了**
（比如整数 cm），低速时球每帧刚好走 1cm，比值恰好 = 1.0，把中位数拉到 1.0000。

本脚本做四件事：
  1. 量化体检：球/机器人坐标的小数部分分布（0.00 扎堆 ⇒ 量化确认）
  2. 高速段衰减：只用球速 ≥2cm/帧 的样本，做 log(速度) 线性回归求每帧衰减
  3. 严格撞墙：要求"贴墙 + 垂直分量变号 + 入射垂直分量 ≥1.5cm/帧 + 附近 20cm 内无机器人"
  4. 速度/加速度分布（我方 vs demo 黄队），并换算成 cm/s

用法：python tools\\py\\calib_probe.py [日志目录] [--limit 40]
"""
import sys, os, math, glob, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rlg_analyzer import parse_rlg
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

LOGDIR = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith('--') else r'C:\Strategy'
LIMIT = 40
if '--limit' in sys.argv:
    LIMIT = int(sys.argv[sys.argv.index('--limit') + 1])

LOGS = [p for p in sorted(glob.glob(os.path.join(LOGDIR, '*.rlg'))) if os.path.getsize(p) > 10000]
HNNU = [p for p in LOGS if 'Hnnu' in os.path.basename(p)]
DEMO = [p for p in LOGS if 'Hnnu' not in os.path.basename(p)]
print(f"日志总数 {len(LOGS)}（我方 Hnnu {len(HNNU)}，其他/demo {len(DEMO)}），本次最多读 {LIMIT} 场")


def frac(v):
    return abs(v - round(v))


def load(paths, limit):
    out = []
    for p in paths[:limit]:
        try:
            out.append((os.path.basename(p), parse_rlg(p)))
        except Exception as e:
            print(f"  skip {os.path.basename(p)}: {e}")
    return out


def probe(logs, tag):
    fracs_ball, fracs_rob = [], []
    decay_pairs = []          # (v_in, ratio) 只用高速段
    ball_spd = []             # 球每帧位移（全体，用于球速分布）
    rest_x, rest_y, fric_x, fric_y = [], [], [], []
    spd = {'blue': [], 'yellow': []}
    acc = {'blue': [], 'yellow': []}

    for name, frames in logs:
        n = len(frames)
        # ① 量化体检
        for f in frames[::7]:
            fracs_ball.append(frac(f['ball']['x']))
            fracs_rob.append(frac(f['blue'][1]['x']))
            fracs_rob.append(frac(f['yellow'][1]['y']))
        for i in range(2, n):
            b0, b1, b2 = frames[i-2]['ball'], frames[i-1]['ball'], frames[i]['ball']
            dx1, dy1 = b1['x']-b0['x'], b1['y']-b0['y']
            dx2, dy2 = b2['x']-b1['x'], b2['y']-b1['y']
            v1 = math.hypot(dx1, dy1)
            v2 = math.hypot(dx2, dy2)
            ball_spd.append(v1)
            robots = frames[i-1]['blue'] + frames[i-1]['yellow']
            near = min(math.hypot(r['x']-b1['x'], r['y']-b1['y']) for r in robots)
            free = near >= 20.0
            # ② 高速段衰减（球离开所有机器人 20cm 以上）
            if free and v1 >= 2.0 and v2 >= 1.0:
                decay_pairs.append((v1, v2 / v1))
            # ③ 严格撞墙
            if free and (b1['x'] < 2.5 or b1['x'] > 217.5) and dx1 * dx2 < 0 and abs(dx1) >= 1.5:
                rest_x.append(abs(dx2) / abs(dx1))
                if abs(dx1) > 1e-6:
                    fric_x.append(abs(dy2) / max(abs(dy1), 1e-9))
            if free and (b1['y'] < 2.5 or b1['y'] > 177.5) and dy1 * dy2 < 0 and abs(dy1) >= 1.5:
                rest_y.append(abs(dy2) / abs(dy1))
                if abs(dy1) > 1e-6:
                    fric_y.append(abs(dx2) / max(abs(dx1), 1e-9))
        # ④ 速度/加速度（3 帧差分）
        for i in range(2, n):
            for side in ('blue', 'yellow'):
                for j in range(5):
                    p2, p1, p0 = frames[i-2][side][j], frames[i-1][side][j], frames[i][side][j]
                    d1 = math.hypot(p1['x']-p0['x'], p1['y']-p0['y'])
                    d2 = math.hypot(p2['x']-p1['x'], p2['y']-p1['y'])
                    spd[side].append(d1)
                    if d1 > 0.2 or d2 > 0.2:      # 静止帧不计加速度（量化噪声）
                        acc[side].append(d1 - d2)

    def med(a): return sorted(a)[len(a)//2] if a else float('nan')
    def pct(a, p):
        s = sorted(a); return s[min(len(s)-1, int(len(s)*p))] if a else float('nan')

    print(f"\n========== {tag} ==========")
    fb = [x for x in fracs_ball if x > 1e-6]
    fr = [x for x in fracs_rob if x > 1e-6]
    print(f"① 量化体检: 球坐标非整数占比 {100.0*len(fb)/max(len(fracs_ball),1):.1f}%（样本 {len(fracs_ball)}）; "
          f"机器人非整数占比 {100.0*len(fr)/max(len(fracs_rob),1):.1f}%")
    if decay_pairs:
        vs = [v for v, _ in decay_pairs]
        rs = [r for _, r in decay_pairs]
        # log 速度对帧号回归的斜率 → 每帧衰减因子（只在同一段连续帧上才严谨，这里用比值中位做粗略口径）
        print(f"② 高速段(≥2cm/帧)衰减: 样本={len(rs)} v1中位={med(vs):.2f}cm/帧 "
              f"比值中位={med(rs):.4f} p25={pct(rs,0.25):.4f} p75={pct(rs,0.75):.4f}")
        for lo, hi in ((2, 3), (3, 5), (5, 99)):
            sel = [r for v, r in decay_pairs if lo <= v < hi]
            if sel:
                print(f"     v1∈[{lo},{hi})cm/帧: n={len(sel)} 比值中位={med(sel):.4f}")
    print(f"③ 严格撞墙: x 法向恢复={med(rest_x):.3f}(n={len(rest_x)}) y={med(rest_y):.3f}(n={len(rest_y)})"
          f" | 切向保持 x={med(fric_x):.3f} y={med(fric_y):.3f}")
    for side, label in (('blue', '我方'), ('yellow', 'demo黄队')):
        s, a = spd[side], acc[side]
        print(f"④ {label} 速度(帧cm): 中位={med(s):.2f} p90={pct(s,0.9):.2f} p95={pct(s,0.95):.2f} "
              f"p99={pct(s,0.99):.2f} 最大={max(s):.2f}  → p99={pct(s,0.99)*40:.0f} cm/s")
        print(f"   {label} 加速度(帧cm²): p90={pct(a,0.9):.3f} p95={pct(a,0.95):.3f} p99={pct(a,0.99):.3f} "
              f"→ p95={pct(a,0.95)*1600:.0f} cm/s²")
    # 返回字段名必须与 tools/py/sim_stats.py 完全一致（定标脚本靠这个对齐两边）
    def dec(lo, hi):
        return med([r for v, r in decay_pairs if lo <= v < hi])
    out = dict(
        frames=sum(len(f) for _, f in logs),
        n_decay=len(decay_pairs), n_rest_x=len(rest_x), n_rest_y=len(rest_y),
        decay_all=med([r for _, r in decay_pairs]),
        decay_2_3=dec(2, 3), decay_3_5=dec(3, 5), decay_5p=dec(5, 99),
        rest_x=med(rest_x), rest_y=med(rest_y), fric_x=med(fric_x), fric_y=med(fric_y),
    )
    for side in ('blue', 'yellow'):
        s, a = spd[side], acc[side]
        out[side + '_spd_p50'] = pct(s, 0.5)
        out[side + '_spd_p90'] = pct(s, 0.9)
        out[side + '_spd_p95'] = pct(s, 0.95)
        out[side + '_spd_p99'] = pct(s, 0.99)
        out[side + '_acc_p90'] = pct(a, 0.9)
        out[side + '_acc_p95'] = pct(a, 0.95)
    out['ball_spd_p50'] = pct(ball_spd, 0.5)
    out['ball_spd_p90'] = pct(ball_spd, 0.9)
    out['ball_spd_p95'] = pct(ball_spd, 0.95)
    out['ball_spd_p99'] = pct(ball_spd, 0.99)
    return out


res = {}
res['Hnnu'] = probe(load(HNNU, LIMIT), "我方日志（Hnnu，17 场量级）")
if DEMO:
    res['DEMO'] = probe(load(DEMO, LIMIT), "官方 demo 日志（校准脚本对手用）")
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calib_probe_result.json'), 'w') as f:
    json.dump(res, f, indent=2, ensure_ascii=False)
print("\n→ tools/py/calib_probe_result.json")
