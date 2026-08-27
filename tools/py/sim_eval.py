# -*- coding: utf-8 -*-
"""
sim_eval.py — sim_bench 标准 A/B 评估流程（批量评估线，参考 refs 4.2 #1）
================================================================
用法（改代码前后各跑一次，对比指标）：

    # 改前（基线）：
    python tools/py/sim_eval.py baseline --games 50 --seed 1

    # 改后（新代码）：
    python tools/py/sim_eval.py new --games 50 --seed 1

    # 对比两个结果：
    python tools/py/sim_eval.py compare baseline.json new.json

说明：
    - 每批用相同 --seed（现在每场种子 = seed+场次*黄金比例，批次内各场独立、批间可复现）
    - 默认 50 场（约 20 秒），追求精度可 100 场
    - 输出 JSON：均分/控球/射门/球位/禁区纪律 + 每场明细
    - compare 输出差异 + 判定（方向/量级）
================================================================
"""
import sys, os, json, subprocess, re, argparse
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

SIM = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build', 'Release', 'sim_bench.exe')

def run_batch(games=50, frames=24000, seed=1, opp='scripted', exe=None):
    exe = exe or os.path.abspath(SIM)
    if not os.path.exists(exe):
        raise SystemExit(f"找不到 sim_bench.exe: {exe}（先 cmake --build build --config Release --target sim_bench）")
    cmd = [exe, '--games', str(games), '--frames', str(frames), '--opp', opp, '--seed', str(seed)]
    r = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', errors='replace')
    if r.returncode != 0:
        raise SystemExit(f"sim_bench 运行失败:\n{r.stderr}")
    out = r.stdout
    # 解析汇总行
    m = re.search(r'=== 汇总: 我们 (\d+) : (\d+) 对手 \(均 ([\d.]+) : ([\d.]+)\)\s+平均控球 ([\d.]+)%\s+平均射门 ([\d.]+)', out)
    if not m:
        raise SystemExit(f"无法解析汇总行:\n{out[-500:]}")
    ga = re.search(r'禁区纪律\(我们\): 门区2\+人 均 ([\d.]+) 帧/场, ([\d.]+) 次/场', out)
    result = dict(
        games=games, frames=frames, opp=opp, seed=seed,
        us_mean=float(m.group(3)), opp_mean=float(m.group(4)),
        us_total=int(m.group(1)), opp_total=int(m.group(2)),
        poss=float(m.group(5)), shots=float(m.group(6)),
        ga_frames=float(ga.group(1)) if ga else None,
        ga_episodes=float(ga.group(2)) if ga else None,
    )
    return result

def compare(a, b):
    print("=" * 64)
    print(f"{'指标':<12} {'基线A':>12} {'新代码B':>12} {'Δ':>10} {'判定':<8}")
    print("-" * 64)
    rows = [
        ('我们场均', a['us_mean'], b['us_mean'], '↑好'),
        ('对手场均', a['opp_mean'], b['opp_mean'], '↓好'),
        ('净胜球', a['us_mean']-a['opp_mean'], b['us_mean']-b['opp_mean'], '↑好'),
        ('控球率%', a['poss'], b['poss'], '↑好'),
        ('射门', a['shots'], b['shots'], '↑好'),
        ('门区违规/场', a.get('ga_episodes'), b.get('ga_episodes'), '↓好'),
    ]
    for name, va, vb, good in rows:
        if va is None or vb is None:
            continue
        d = vb - va
        verdict = ''
        if abs(d) < 0.5:
            verdict = '≈持平'
        elif (d > 0) == (good == '↑好'):
            verdict = '✅ 变好'
        else:
            verdict = '⚠️ 变差'
        print(f"{name:<12} {va:>12.2f} {vb:>12.2f} {d:>+10.2f} {verdict:<8}")
    print("-" * 64)
    print("注意：50 场均值波动约 ±0.5 球，Δ<0.5 视为噪声；")
    print("     重大改动建议 100 场，并配合 --debug 抽查关键场次。")

def main():
    ap = argparse.ArgumentParser(description='sim_bench 批量 A/B 评估')
    sub = ap.add_subparsers(dest='cmd', required=True)
    p_run = sub.add_parser('run', help='跑一批并存 JSON')
    p_run.add_argument('name', help='批次名（如 baseline/new）')
    p_run.add_argument('--games', type=int, default=50)
    p_run.add_argument('--frames', type=int, default=24000)
    p_run.add_argument('--seed', type=int, default=1)
    p_run.add_argument('--opp', default='scripted', choices=['scripted', 'yellow', 'self'])
    p_cmp = sub.add_parser('compare', help='对比两个批次 JSON')
    p_cmp.add_argument('a', help='基线 JSON 路径')
    p_cmp.add_argument('b', help='新代码 JSON 路径')
    args = ap.parse_args()

    if args.cmd == 'run':
        r = run_batch(args.games, args.frames, args.seed, args.opp)
        path = f"sim_eval_{args.name}.json"
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(r, f, indent=2, ensure_ascii=False)
        print(f"批次 {args.name}: 我们 {r['us_mean']:.1f} : {r['opp_mean']:.1f} "
              f"控球 {r['poss']:.1f}% 射门 {r['shots']:.0f} 门区违规 {r['ga_episodes']:.1f}次/场")
        print(f"已存 {path}")
    elif args.cmd == 'compare':
        a = json.load(open(args.a, encoding='utf-8'))
        b = json.load(open(args.b, encoding='utf-8'))
        compare(a, b)

if __name__ == '__main__':
    main()
