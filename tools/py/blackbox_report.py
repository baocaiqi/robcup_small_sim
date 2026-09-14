"""黑匣子报告（跑完 Match 1 后运行）—— 把 hnnu_blackbox.csv 翻译成 5 个问题的答案。

用法：python tools/py/blackbox_report.py [csv路径]
回答：
  ⑤ 执行期 gameState / whosBall 的真实值（以及是否只在摆位回调里给真信息）
  ① 我们写进去的踢球人位置（P 行）—— 与 rlg 实际保留值对比
  ② 我们写进去的门将位置（P 行）
  ③ 点球从"球被摆好"到"球第一次被推动"的耗时分布
  球权标定：平台 whosBall 非 0 时，与我们自算球权的**一致率**（判断 1=我方 还是 1=蓝方）
"""
import os
import sys
from collections import Counter, defaultdict

CSV = sys.argv[1] if len(sys.argv) > 1 else r"C:\Strategy\hnnu_blackbox.csv"
STATE = {0: "PlayOn", 1: "FreeBall_LT", 2: "FreeBall_LB", 3: "FreeBall_RT", 4: "FreeBall_RB",
         5: "PlaceKick_Y", 6: "PlaceKick_B", 7: "Penalty_Y", 8: "Penalty_B",
         9: "FreeKick_Y", 10: "FreeKick_B", 11: "GoalKick_Y", 12: "GoalKick_B"}

if not os.path.exists(CSV):
    print(f"✗ 找不到 {CSV}（先跑一场黑匣子版，或确认平台与策略同目录）")
    sys.exit(1)

F, P, B = [], [], []
for ln in open(CSV, encoding="utf-8", errors="replace"):
    t = ln.strip().split(",")
    if not t or t[0] not in ("F", "P", "B"):
        continue
    try:
        if t[0] == "F":
            F.append(dict(fr=int(t[1]), gs=int(t[2]), whos=int(t[3]), bx=float(t[4]), by=float(t[5]),
                          vx=float(t[6]), vy=float(t[7]), ours=int(t[8]), pen=int(t[9]),
                          gk=float(t[10]), act=float(t[11])))
        elif t[0] == "P":
            P.append(dict(fr=int(t[1]), tag=t[2], gs=int(t[3]),
                          xy=[(float(t[4 + 2 * i]), float(t[5 + 2 * i])) for i in range(5)]))
        else:
            B.append(dict(fr=int(t[1]), x=float(t[2]), y=float(t[3]), gs=int(t[4])))
    except (IndexError, ValueError):
        continue

print(f"=== 读入 {CSV}：F {len(F)} 帧 / P {len(P)} 次摆位 / B {len(B)} 次摆球 ===\n")

# ---------- ⑤ gameState / whosBall 的真实分布 ----------
print("【⑤ gameState / whosBall 实测分布】")
c = Counter((f["gs"], f["whos"]) for f in F)
for (gs, whos), n in sorted(c.items(), key=lambda kv: -kv[1]):
    print(f"   gameState={gs:<3}({STATE.get(gs, '?'):<13}) whosBall={whos:<3} → {n:>6} 帧")
print(f"   whosBall 出现过的取值: {sorted(set(f['whos'] for f in F))}")
pen = [f for f in F if f["pen"]]
if pen:
    print(f"\n   我方点球执行期(in_penalty=1) 共 {len(pen)} 帧，其 gameState/whos 组合：")
    for (gs, whos), n in Counter((f["gs"], f["whos"]) for f in pen).most_common(5):
        print(f"      gameState={gs}({STATE.get(gs, '?')}) whosBall={whos} → {n} 帧"
              + ("   ← 执行期确实不是点球态" if gs == 0 else ""))
# 摆位回调时的 gameState（=平台在摆位期给的真值）
if P:
    print("\n   摆位回调时的 gameState（P 行）：")
    for (gs, tag), n in Counter((p["gs"], p["tag"]) for p in P).most_common(8):
        print(f"      {tag:<7} gameState={gs}({STATE.get(gs, '?')}) → {n} 次")

# ---------- 球权标定 ----------
nz = [f for f in F if f["whos"] != 0]
print(f"\n【球权标定】平台 whosBall 非 0 的帧：{len(nz)} / {len(F)}")
if nz:
    agree1 = sum(1 for f in nz if (f["whos"] == 1) == bool(f["ours"]))
    agree2 = sum(1 for f in nz if (f["whos"] == 1) != bool(f["ours"]))
    print(f"   假设A「1=我方, 2=对方」与自算一致率: {100.0*agree1/len(nz):.1f}%")
    print(f"   假设B「1=对方, 2=我方」与自算一致率: {100.0*agree2/len(nz):.1f}%")
    print("   → 一致率高的那个就是真实语义（自算判据有 20cm 阈值，只在明确局面才准）")
    for (whos, ours), n in Counter((f["whos"], f["ours"]) for f in nz).most_common(6):
        print(f"      whos={whos} 自算={ours} → {n} 帧")
else:
    print("   ⚠️ whosBall 全程为 0 ⇒ 这个字段在本平台**没有可用信息**，球权只能自算（可据此定性）")

# ---------- ① ② 我们写进去的摆位 ----------
print("\n【①② 我们写进去的摆位（P 行，取前 6 次）】")
print("   说明：GK=index0、踢球人(ACTIVE)=index1；与 rlg 里摆位后第一帧对比，即得平台挪动量")
for p in P[:6]:
    print(f"   帧{p['fr']:>6} {p['tag']:<7} gs={p['gs']}({STATE.get(p['gs'],'?')})  "
          f"GK=({p['xy'][0][0]:.1f},{p['xy'][0][1]:.1f})  ACTIVE=({p['xy'][1][0]:.1f},{p['xy'][1][1]:.1f})")
if B:
    print(f"   摆球(B 行) 前 4 次: " + " | ".join(f"gs={b['gs']} ({b['x']:.1f},{b['y']:.1f})" for b in B[:4]))

# ---------- ③ 点球执行耗时 ----------
print("\n【③ 点球执行耗时（球被摆好 → 第一次被推动）】")
runs, i = [], 0
while i < len(F):
    if F[i]["pen"]:
        j = i
        while j + 1 < len(F) and F[j + 1]["pen"]:
            j += 1
        # 该段内球速首次超过 0.05cm/帧 的帧号
        moved = next((k for k in range(i, j + 1)
                      if abs(F[k]["vx"]) > 0.05 or abs(F[k]["vy"]) > 0.05), None)
        dur = (j - i + 1) / 40.0
        delay = (moved - i) / 40.0 if moved else None
        runs.append((F[i]["fr"], dur, delay))
        i = j + 1
    else:
        i += 1
if runs:
    ds = [d for _, _, d in runs if d is not None]
    print(f"   共 {len(runs)} 段点球执行期（平均 {sum(r[1] for r in runs)/len(runs):.2f}s）")
    for fr, dur, delay in runs[:8]:
        print(f"      起始帧{fr:>6} 持续{dur:>5.2f}s  " +
              (f"出脚耗时 {delay:.2f}s" if delay is not None else "⚠️ 全程没推动球"))
    if ds:
        print(f"   出脚耗时：中位 {sorted(ds)[len(ds)//2]:.2f}s，最大 {max(ds):.2f}s")
        print("   → 若最大耗时远小于平台打断阈值，说明**执行期没有硬计时**（或阈值很宽）")
else:
    print("   本场没有捕捉到我方点球执行期（in_penalty=1 从未出现）")
