# -*- coding: utf-8 -*-
"""sim_ab_pair.py — 两组 sim_bench 日志的**配对 A/B** 分析（同种子逐场比较）

为什么要它：`sim_bench` 的输出是逐场一行 + 一行 FIT 汇总。看过 A、B 两份日志的人
往往只比 FIT 的净胜球，**丢掉了"同一批对手随机序列逐场配对"这个最有价值的信息**——
配对能把对手随机性消掉，灵敏度比两组独立均值高得多（docs/06 轮次 77 的教训：
50 场独立噪声 ±0.5 球，同种子配对能看出 ±0.1 量级）。

用法：
  build\\Release\\sim_bench.exe --games 50 --opp scripted --seed 1 > on.txt
  build\\Release\\sim_bench.exe --games 50 --opp scripted --seed 1 --params off.txt > off.txt
  python tools/py/sim_ab_pair.py on.txt off.txt

输出：各项指标的"关 / 开 / 配对差(开−关) / 95%CI"，以及逐场胜负计数。
约定：第一个文件 = 对照组（关），第二个文件 = 实验组（开）。

⚠️ 读结论的三条纪律（R-D5）：
  1. 置信区间跨 0 ⇒ 只能说"没看出差别"，不能说"更好/更差"；
  2. 本工具只回答"同一对手同档下 A 是否比 B 好"，**不能**回答"打得过官方 demo 吗"；
  3. 区间不跨 0 也只是一条证据——单项出带（比如救球 +4.3）要单独去看是不是副作用。
"""
import re
import statistics as st
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

GAME = re.compile(
    r"场\s*(\d+): 我们\s*(\d+)\s*:\s*(\d+)\s*对手.*?射门\s*(\d+)"
    r".*?禁区2\+人\s*(\d+)帧/(\d+)次.*?单人>20帧\s*(\d+)帧/(\d+)次"
    r".*?争球重置\s*(\d+)次\(角区\d+\).*?救球(\d+)次")

FIELDS = (("net", "净胜球"), ("gf", "进球"), ("ga", "失球"), ("shots", "射门次数"),
          ("gaf", "门区2+人帧"), ("solo", "单人滞留帧"), ("fb", "争球重置"), ("save", "救球次数"))


def parse(path):
    out = {}
    for line in open(path, encoding="utf-8", errors="replace"):
        m = GAME.search(line)
        if m:
            g = int(m.group(1))
            out[g] = dict(gf=int(m.group(2)), ga=int(m.group(3)), shots=int(m.group(4)),
                          gaf=int(m.group(5)), gae=int(m.group(6)), solo=int(m.group(7)),
                          soloe=int(m.group(8)), fb=int(m.group(9)), save=int(m.group(10)))
    return out


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    base, exp = parse(sys.argv[1]), parse(sys.argv[2])
    games = sorted(set(base) & set(exp))
    if not games:
        print("没解析到逐场数据（确认两份日志是 --games 相同的同种子跑法）")
        return
    print(f"配对场次 {len(games)}（对照 = {sys.argv[1]}，实验 = {sys.argv[2]}）\n")
    print(f"{'指标':<14}{'对照':>10}{'实验':>10}{'配对差(实验-对照)':>20}{'95%CI':>20}")
    for key, label in FIELDS:
        a, b = [], []
        for g in games:
            if key == "net":
                a.append(base[g]["gf"] - base[g]["ga"])
                b.append(exp[g]["gf"] - exp[g]["ga"])
            else:
                a.append(base[g][key])
                b.append(exp[g][key])
        d = [y - x for x, y in zip(a, b)]
        mean_d = st.mean(d)
        sd = st.stdev(d) if len(d) > 1 else 0.0
        ci = 1.96 * sd / (len(d) ** 0.5)
        flag = "  ← 出噪声带" if (mean_d - ci) * (mean_d + ci) > 0 else ""
        print(f"{label:<14}{st.mean(a):>10.2f}{st.mean(b):>10.2f}"
              f"{mean_d:>+20.2f}{f'[{mean_d-ci:+.2f}, {mean_d+ci:+.2f}]':>20}{flag}")
    wins = sum(1 for g in games if (exp[g]["gf"] - exp[g]["ga"]) > (base[g]["gf"] - base[g]["ga"]))
    loss = sum(1 for g in games if (exp[g]["gf"] - exp[g]["ga"]) < (base[g]["gf"] - base[g]["ga"]))
    print(f"\n逐场比较：实验更好 {wins} 场 / 更差 {loss} 场 / 持平 {len(games) - wins - loss} 场")


main()
