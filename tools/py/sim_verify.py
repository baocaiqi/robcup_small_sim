# -*- coding: utf-8 -*-
"""
sim_verify.py — 真实平台 vs sim_bench 校准对比（验证 B 级可靠：策略排序一致）

输入：
  1. 真实比赛：官方 SimuroSot5.log（权威比分）+ 完整 .rlg（运动学/行为）
  2. 仿真比赛：sim_bench.exe --games N 的输出（--csv 导出的逐场统计）

输出：
  真实 vs 仿真 的 比分 / 控球 / 球位 / 射门 / 运动学 对比表 + 判定

用法：
    python tools/py/sim_verify.py [日志目录= C:\\Strategy]
"""
import sys, os, math, glob, json, re
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rlg_analyzer import parse_rlg
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

LOGDIR = sys.argv[1] if len(sys.argv) > 1 else r'C:\Strategy'

# ---------- 1. 官方 log 权威比分 ----------
def official_scores(logdir):
    log = os.path.join(logdir, 'SimuroSot5.log')
    if not os.path.exists(log): return []
    data = open(log, 'rb').read().decode('utf-8', errors='replace')
    pat = re.compile(r'^(.+?)\s*--\s*\((\d+)\s*:\s*(\d+)\)\s*Time\s*:\s*(\d+)\.\s*$')
    matches, cur = [], None
    for ln in data.splitlines():
        m = pat.match(ln)
        if not m: continue
        state, b, y, t = m.group(1).strip(), int(m.group(2)), int(m.group(3)), int(m.group(4))
        if state.startswith('Place Kick') and b == 0 and y == 0 and t > 290:
            if cur: matches.append(cur)
            cur = {'b': b, 'y': y, 'tmax': t, 'tmin': t}
        elif cur:
            cur['b'] = b; cur['y'] = y
            cur['tmin'] = min(cur['tmin'], t)
    if cur: matches.append(cur)
    return [m for m in matches if m['tmax'] - m['tmin'] > 100]   # 过滤不完整场

# ---------- 2. 完整 rlg 的统计 ----------
def rlg_stats(rlg_path):
    frames = parse_rlg(rlg_path)
    n = len(frames)
    if n < 5000: return None   # 不完整(<125s)跳过
    # 进球(聚类)
    blue = yellow = 0; last = None
    # 控球/球位
    poss_us = poss_demo = 0
    zones = [0,0,0]
    shots_us = shots_demo = 0
    for i, f in enumerate(frames):
        bx, by = f['ball']['x'], f['ball']['y']
        if bx > 220 and 70 <= by <= 110: side = 'demo'
        elif bx < 0 and 70 <= by <= 110: side = 'us'
        else: side = None
        if side and side != last:
            if side == 'us': blue += 1
            else: yellow += 1
        last = side
        du = min(math.hypot(r['x']-bx, r['y']-by) for r in f['blue'])
        dd = min(math.hypot(r['x']-bx, r['y']-by) for r in f['yellow'])
        if du < 20 and du <= dd: poss_us += 1
        elif dd < 20 and dd < du: poss_demo += 1
        if bx < 73: zones[0] += 1
        elif bx < 147: zones[1] += 1
        else: zones[2] += 1
        if bx < 30 and 70 <= by <= 110: shots_us += 1
        if bx > 190 and 70 <= by <= 110: shots_demo += 1
    return dict(
        seconds=n/40, blue=blue, yellow=yellow,
        poss_us=poss_us/(poss_us+poss_demo)*100,
        zones=[z*100/n for z in zones], shots_us=shots_us, shots_demo=shots_demo)

def main():
    # 真实侧
    scores = official_scores(LOGDIR)
    print("="*70)
    print("真实平台（官方 SimuroSot5.log）")
    print("="*70)
    if scores:
        n = len(scores)
        tb = sum(s['b'] for s in scores); ty = sum(s['y'] for s in scores)
        wins = sum(1 for s in scores if s['b'] > s['y'])
        print(f"完整场次 {n}  平均 {tb/n:.1f} : {ty/n:.1f}  赢{wins}场")
        score_strs = [str(s['b']) + ':' + str(s['y']) for s in scores]
        print(f"各场比分: {score_strs}")
    else:
        print("未找到完整场次（SimuroSot5.log 缺失或无 >100s 场次）")

    # rlg 侧
    rlgs = sorted(glob.glob(os.path.join(LOGDIR, '*MyTeam*.rlg')))
    print("\n" + "="*70)
    print("真实平台（完整 rlg 统计，n<5000帧=不完整跳过）")
    print("="*70)
    valid = 0
    agg = dict(blue=0, yellow=0, poss=0, zones=[0,0,0], shots=0)
    for rl in rlgs:
        st = rlg_stats(rl)
        if not st: continue
        valid += 1
        agg['blue'] += st['blue']; agg['yellow'] += st['yellow']
        agg['poss'] += st['poss_us']
        agg['zones'] = [a+b for a,b in zip(agg['zones'], st['zones'])]
        agg['shots'] += st['shots_us']
        print(f"  {os.path.basename(rl)[:18]} {st['seconds']:.0f}s  "
              f"{st['blue']}:{st['yellow']} 控球{st['poss_us']:.0f}% "
              f"球位{st['zones'][0]:.0f}/{st['zones'][1]:.0f}/{st['zones'][2]:.0f}")
    if valid:
        print(f"  —— 合计 {valid} 场 平均 {agg['blue']/valid:.1f}:{agg['yellow']/valid:.1f} "
              f"控球{agg['poss']/valid:.1f}% "
              f"球位{agg['zones'][0]/valid:.0f}/{agg['zones'][1]/valid:.0f}/{agg['zones'][2]/valid:.0f}")

    print("\n" + "="*70)
    print("⚠️ 重要：rlg 统计仅供参考，不可作比分依据！")
    print("="*70)
    print("已确认 rlg 与官方 log 比分矛盾（rlg 统计 0.9:3.9 vs 官方 2.7:0.8）：")
    print("  - rlg 每场只录部分时段（166-230s/300s），进球事件不完整")
    print("  - rlg gameState 字段全 0（解析偏移或平台不写），无法对齐官方事件")
    print("  - 权威比分 = 官方 SimuroSot5.log（平台自己记分）")
    print("  - rlg 仅用于运动学（球速/衰减/反弹）与球位分布参考")

    print("\n" + "="*70)
    print("对比判定（B 级：策略排序一致）")
    print("="*70)
    print("把 sim_bench --games 30 的汇总行贴到这里对比：")
    print("  sim 均分 : 黄均分   |  控球率   |  球位(黄后/中/蓝后)")
    print("判定标准：")
    print("  ① 方向一致：真实赢→sim 也赢（必须）")
    print("  ② 量级接近：sim 比分与真实 ±1~2 球（良好）")
    print("  ③ 行为一致：控球/球位分布趋势相同（良好）")

    print("\n" + "="*70)
    print("一键契合度判定（输入 sim 均值）")
    print("="*70)
    sim_in = input("输入 sim 汇总行（如 '3.0 : 1.4 控球34%'）: ").strip()
    m = re.match(r'([\d.]+)\s*:\s*([\d.]+)', sim_in)
    if m and scores:
        s_b, s_y = float(m.group(1)), float(m.group(2))
        r_b = tb/n; r_y = ty/n
        print(f"\n  真实 {r_b:.1f}:{r_y:.1f}  vs  sim {s_b:.1f}:{s_y:.1f}")
        dir_ok = (r_b > r_y) == (s_b > s_y)
        print(f"  ① 方向一致: {'✅' if dir_ok else '❌ 真实与sim胜负相反，不可用'}")
        if dir_ok:
            db = abs(s_b - r_b); dy = abs(s_y - r_y)
            print(f"  ② 进球偏差 {db:.1f} 球 {'✅±2内' if db<=2 else '⚠️偏高'}")
            print(f"     失球偏差 {dy:.1f} 球 {'✅±2内' if dy<=2 else '⚠️偏高'}")
            if db <= 2 and dy <= 2:
                print("  ✅ B级可用：策略排序可与真实一致")
            else:
                print("  ⚠️ 接近但量级有偏差：可用于相对排序，绝对数值不可信")
    else:
        print("  输入格式不对（需含 'x : y'）或缺少真实数据")

if __name__ == '__main__':
    main()
