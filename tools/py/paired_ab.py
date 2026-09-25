"""同种子配对 A/B：对若干参数配置跑 sim_bench，按场配对算差值与 95% CI。

用法：
  python tools/py/paired_ab.py --arms base=  off=roles.kPassTasksEnabled=0 on=roles.kPassTasksEnabled=1 \
      --games 200 --seeds 1,101 --opp scripted --strength 2.0

每个 arm 形如 名字=参数1=值,参数2=值（空 = 默认参数）。第一个 arm 是对照组。
同一 (seed, 场次) 在各 arm 间初始条件相同，所以差值是配对的。
"""
import argparse
import concurrent.futures as cf
import math
import os
import re
import subprocess
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BENCH = os.path.join(ROOT, "build", "Release", "sim_bench.exe")

GAME_RE = re.compile(
    r"场(\d+): 我们 (\d+) : (\d+) 对手\s+控球率\(我们\) (\d+)%\s+射门 (\d+).*?"
    r"禁区2\+人 (\d+)帧/(\d+)次 单人>20帧 (\d+)帧/(\d+)次")
COOP_RE = re.compile(r"COOP game=(\d+) .*?created=(\d+) released=(\d+) received=(\d+) .*?success=(\d+)")

METRICS = ["net", "gf", "ga", "poss", "gaf_frames", "gaf_events", "solo_frames", "solo_events",
           "pass_created", "pass_success", "score"]


def parse_arm(spec):
    name, _, rest = spec.partition("=")
    params = []
    for kv in filter(None, rest.split(",")):
        k, _, v = kv.partition("=")
        params.append((k, v))
    return name, params


def run_chunk(params, seed, games, args):
    cmd = [BENCH, "--games", str(games), "--frames", str(args.frames), "--opp", args.opp,
           "--seed", str(seed), "--strength", str(args.strength)]
    pfile = None
    if params:
        fd, pfile = tempfile.mkstemp(suffix=".txt", text=True)
        with os.fdopen(fd, "w", encoding="ascii") as f:
            for k, v in params:
                f.write(f"{k} {v}\n")
        cmd += ["--params", pfile]
    try:
        out = subprocess.run(cmd, capture_output=True, cwd=ROOT).stdout.decode("utf-8", "replace")
    finally:
        if pfile:
            os.remove(pfile)
    coop = {int(m[1]): (int(m[2]), int(m[5])) for m in COOP_RE.finditer(out)}
    rows = []
    for m in GAME_RE.finditer(out):
        g = int(m[1])
        gf, ga = int(m[2]), int(m[3])
        created, success = coop.get(g, (0, 0))
        r = dict(key=(seed, g), gf=gf, ga=ga, net=gf - ga, poss=int(m[4]),
                 gaf_frames=int(m[6]), gaf_events=int(m[7]),
                 solo_frames=int(m[8]), solo_events=int(m[9]),
                 pass_created=created, pass_success=success)
        # 交接文档建议评分：进球-失球-0.5×门区违规（sim_bench 无点球计数，此处略）
        r["score"] = r["net"] - 0.5 * r["gaf_events"]
        rows.append(r)
    if len(rows) != games:
        raise RuntimeError(f"seed={seed} 期望 {games} 场，解析到 {len(rows)} 场\n{out[-2000:]}")
    return rows


def mean_ci(xs):
    n = len(xs)
    m = sum(xs) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1)) if n > 1 else 0.0
    return m, 1.96 * sd / math.sqrt(n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arms", nargs="+", required=True)
    ap.add_argument("--games", type=int, default=200, help="每个 seed 的场数")
    ap.add_argument("--seeds", default="1,101")
    ap.add_argument("--chunk", type=int, default=25)
    ap.add_argument("--frames", type=int, default=12000)
    ap.add_argument("--opp", default="scripted")
    ap.add_argument("--strength", type=float, default=2.0)
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()

    arms = [parse_arm(s) for s in args.arms]
    # 分块：块基种子 = seed*100000 + 块号，保证各 arm 用同一组块
    blocks = []
    for s in map(int, args.seeds.split(",")):
        for b in range(0, args.games, args.chunk):
            blocks.append((s * 100000 + b, min(args.chunk, args.games - b)))

    results = {name: {} for name, _ in arms}
    with cf.ThreadPoolExecutor(args.jobs) as ex:
        futs = {ex.submit(run_chunk, params, bs, n, args): name
                for name, params in arms for bs, n in blocks}
        for f in cf.as_completed(futs):
            for r in f.result():
                results[futs[f]][r["key"]] = r

    base_name = arms[0][0]
    base = results[base_name]
    keys = sorted(base)
    print(f"opp={args.opp} strength={args.strength} frames={args.frames} 配对场数={len(keys)}")
    print(f"{'指标':<14}" + "".join(f"{n:>16}" for n, _ in arms))
    for met in METRICS:
        line = f"{met:<14}"
        for name, _ in arms:
            m, ci = mean_ci([results[name][k][met] for k in keys])
            line += f"{m:>9.3f}±{ci:<6.3f}"
        print(line)
    print(f"\n配对差值（arm − {base_name}），95% CI；* = CI 不含 0")
    for name, _ in arms[1:]:
        print(f"-- {name}")
        for met in METRICS:
            d = [results[name][k][met] - base[k][met] for k in keys]
            m, ci = mean_ci(d)
            flag = "*" if abs(m) > ci else " "
            print(f"   {met:<14}{m:>+9.3f} ± {ci:.3f} {flag}")


if __name__ == "__main__":
    main()
