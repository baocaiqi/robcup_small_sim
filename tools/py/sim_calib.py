# -*- coding: utf-8 -*-
"""
sim_calib.py — 从真实 rlg 比赛日志反推 sim_bench 物理参数（校准用）

提取：
  1. 球自由滚动衰减率 kBallDecay（球脱离所有机器人接触、只受地面摩擦的帧）
  2. 撞墙反弹系数 kWallRest（x 左/右墙、y 上下墙分开）
  3. 机器人速度/加速度分布（校准 kSpeed、kAccel）
  4. 蓝队(我方)与黄队(demo)门将横向移动速度（脚本对手门将参照 demo 标定）

用法：
    python tools/py/sim_calib.py [日志目录]
"""
import sys, os, math, glob, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rlg_analyzer import parse_rlg
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

LOGDIR = sys.argv[1] if len(sys.argv) > 1 else r'C:\Strategy'
LOGS = sorted(glob.glob(os.path.join(LOGDIR, '*MyTeam*.rlg')))
print(f"日志 {len(LOGS)} 场")

CONTACT = 15.0      # 机器人-球接触判定距离 cm
DECAY_VLO = 0.6     # 采样衰减所需最小球速 cm/帧

def is_free(robots, ball):
    return all(math.hypot(r['x']-ball['x'], r['y']-ball['y']) >= CONTACT for r in robots)

decays, bounce_x, bounce_y = [], [], []
blue_speeds, blue_accs = [], []          # 我方所有机器人
demo_speeds, demo_accs = [], []
gk_y_blue, gk_y_demo = [], []            # 门将横向速度(帧位移)

for LOG in LOGS:
    try:
        frames = parse_rlg(LOG)
    except Exception as e:
        print(f"  skip {os.path.basename(LOG)}: {e}")
        continue
    n = len(frames)
    for i in range(2, n):
        b0, b1, b2 = frames[i-2]['ball'], frames[i-1]['ball'], frames[i]['ball']
        v1 = math.hypot(b1['x']-b0['x'], b1['y']-b0['y'])
        v2 = math.hypot(b2['x']-b1['x'], b2['y']-b1['y'])
        # 自由滚动（脱离接触 + 都在动 + 缓慢变化）
        if v1 >= DECAY_VLO and v2 > 0.3 and 0.3 < v2/v1 < 1.2 \
           and is_free(frames[i-1]['blue'], b1) and is_free(frames[i-1]['yellow'], b1):
            decays.append(v2/v1)
        # 撞墙 x/y（贴近墙 + 垂直分量变号）
        vx1 = b1['x']-b0['x']; vx2 = b2['x']-b1['x']
        if (b1['x'] < 3 or b1['x'] > 217) and vx1*vx2 < 0 and abs(vx1) > 0.6:
            bounce_x.append(abs(vx2)/abs(vx1))
        vy1 = b1['y']-b0['y']; vy2 = b2['y']-b1['y']
        if (b1['y'] < 3 or b1['y'] > 177) and vy1*vy2 < 0 and abs(vy1) > 0.6:
            bounce_y.append(abs(vy2)/abs(vy1))
    # 机器人速度/加速度（用相邻 3 帧差分，跳变帧会污染加速度，取分布高分位）
    for i in range(2, n):
        for side in ('blue', 'yellow'):
            for j in range(5):
                p2, p1, p0 = frames[i-2][side][j], frames[i-1][side][j], frames[i][side][j]
                d1 = math.hypot(p1['x']-p0['x'], p1['y']-p0['y'])          # v[i]
                d2 = math.hypot(p2['x']-p1['x'], p2['y']-p1['y'])          # v[i-1]
                a = d1 - d2
                if side == 'blue':
                    blue_speeds.append(d1); blue_accs.append(a)
                else:
                    demo_speeds.append(d1); demo_accs.append(a)
        # 门将横向（蓝0号守 x=220, 黄0号守 x=0；横向=y）
        gk_y_blue.append(abs(frames[i]['blue'][0]['y'] - frames[i-1]['blue'][0]['y']))
        gk_y_demo.append(abs(frames[i]['yellow'][0]['y'] - frames[i-1]['yellow'][0]['y']))

def med(a): return sorted(a)[len(a)//2] if a else float('nan')
def pct(a, p):
    s = sorted(a); return s[min(len(s)-1, int(len(s)*p))] if a else float('nan')

print("\n===== 校准提取结果 =====")
print(f"① 自由滚动衰减 kBallDecay: 样本={len(decays)} 中位={med(decays):.4f} "
      f"p25={pct(decays,0.25):.4f} (sim 当前 0.985)")
print(f"   换算每秒保留 = {med(decays)**40:.3f}")
print(f"② 撞墙反弹: x={med(bounce_x):.3f}(n={len(bounce_x)}) y={med(bounce_y):.3f}(n={len(bounce_y)}) "
      f"(sim 当前 0.55)")
print(f"③ 我方机器人速度(帧cm): 中位={med(blue_speeds):.2f} p90={pct(blue_speeds,0.9):.2f} "
      f"p95={pct(blue_speeds,0.95):.2f} p99={pct(blue_speeds,0.99):.2f}")
print(f"   加速度(帧cm差): p90={pct(blue_accs,0.9):.2f} p95={pct(blue_accs,0.95):.2f} "
      f"p99={pct(blue_accs,0.99):.2f} (sim kAccel=300cm/s² = 7.5/帧)")
print(f"   demo机器人速度: 中位={med(demo_speeds):.2f} p90={pct(demo_speeds,0.9):.2f} "
      f"p95={pct(demo_speeds,0.95):.2f} p99={pct(demo_speeds,0.99):.2f}")
print(f"④ 我方门将横向(帧cm): 中位={med(gk_y_blue):.2f} p90={pct(gk_y_blue,0.9):.2f} "
      f"p99={pct(gk_y_blue,0.99):.2f}")
print(f"   demo门将横向(帧cm): 中位={med(gk_y_demo):.2f} p90={pct(gk_y_demo,0.9):.2f} "
      f"p99={pct(gk_y_demo,0.99):.2f}")

out = dict(
    kBallDecay=med(decays), kWallRest_x=med(bounce_x), kWallRest_y=med(bounce_y),
    blue_speed_p90=pct(blue_speeds,0.9), blue_speed_p95=pct(blue_speeds,0.95),
    blue_speed_p99=pct(blue_speeds,0.99),
    blue_acc_p90=pct(blue_accs,0.9), blue_acc_p95=pct(blue_accs,0.95),
    demo_speed_p90=pct(demo_speeds,0.9), demo_speed_p95=pct(demo_speeds,0.95),
    demo_speed_p99=pct(demo_speeds,0.99),
    gk_blue_p90=pct(gk_y_blue,0.9), gk_demo_p90=pct(gk_y_demo,0.9),
    gk_demo_p99=pct(gk_y_demo,0.99),
    samples=dict(decay=len(decays), bx=len(bounce_x), by=len(bounce_y)),
)
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sim_calib_result.json'), 'w') as f:
    json.dump(out, f, indent=2, ensure_ascii=False)
print(f"\n结果已存 tools/py/sim_calib_result.json")
