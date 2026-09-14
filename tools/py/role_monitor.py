# -*- coding: utf-8 -*-
"""role_monitor.py — 真机测试用的**实时战术监视窗**：把我方 5 台机器人标上角色名。

为什么需要它：平台本身**没有任何画字/贴标签的接口**（只有 5 个导出函数，唯一能写的字符串是队名），
场上 5 台外观一样、连编号都看不出来。所以"显示角色名"只能在平台之外做：
  读黑匣子 CSV（每帧有我方 5 台的坐标 + 角色、对手 5 台坐标、球位）→ 自己画一张带标签的俯视图。

用法（测试时开着这个窗口即可）：
    python tools\\py\\role_monitor.py                 # 实时（1 秒刷几次，自动跟随最新数据）
    python tools\\py\\role_monitor.py --snapshot x.png  # 只画一帧存图（自检/截图用）
    python tools\\py\\role_monitor.py --csv <路径>      # 换 CSV 位置

界面会显示：我方 5 台（门将/主攻/助攻/中场/后卫，各自颜色）+ 对手（黄圈）+ 球 + 帧号/球权；
**我方门区里出现非门将的第二个人时会红框报警**（这条是真机 9 次点球的根因，见 docs/06 第 68 轮）。
"""
import argparse
import io
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_video as MV                      # 复用画场地/机器人/球/字体
from PIL import Image, ImageDraw

CSV_DEF = r"C:\Strategy\hnnu_blackbox.csv"
STATE = {0: "PlayOn", 1: "FreeBall_LT", 2: "FreeBall_LB", 3: "FreeBall_RT", 4: "FreeBall_RB",
         5: "PlaceKick_Y", 6: "PlaceKick_B", 7: "点球:黄方主罚", 8: "点球:我方主罚",
         9: "FreeKick_Y", 10: "FreeKick_B", 11: "GoalKick_Y", 12: "GoalKick_B"}
W, H = 980, 720
FIELD_W = 760


def read_latest(csv_path):
    """取最新 session 里最后一条 F 行 + 它对应的 R/O 行"""
    try:
        lines = io.open(csv_path, encoding="utf-8", errors="replace").read().splitlines()
    except OSError:
        return None
    if not lines:
        return None
    start = 0
    for k in range(len(lines) - 1, -1, -1):
        if lines[k].startswith("#"):
            start = k
            break
    F = R = O = None
    for l in reversed(lines[start:]):
        t = l.split(",")
        if t[0] == "F" and F is None:
            F = t
        elif t[0] == "R" and R is None:
            R = t
        elif t[0] == "O" and O is None:
            O = t
        if F and R and O:
            break
    if not F or not R:
        return None
    try:
        ours = [(float(R[3 + 4 * i]), float(R[4 + 4 * i]), int(R[6 + 4 * i])) for i in range(5)]
        opps = []
        if O:
            opps = [(float(O[2 + 3 * i]), float(O[3 + 3 * i])) for i in range(5)]
        return dict(frame=int(F[1]), gs=int(F[2]), whos=int(F[3]),
                    ball=(float(F[4]), float(F[5])), ours=ours, opps=opps,
                    ours_ball=int(F[8]), pen=int(F[9]))
    except (ValueError, IndexError):
        return None


def draw(d, s):
    """把一帧画到 980x720 的图上"""
    MV.FIELD_W = FIELD_W                     # 借用 make_video 的坐标换算
    d.rectangle([0, 0, FIELD_W, H], fill=MV.FIELD_BG)
    MV.draw_field(d)
    for (x, y) in s["opps"]:
        if 0 <= x <= MV.FW and 0 <= y <= MV.FH:
            MV.draw_robot(d, x, y, False, 0)
    # 我方门区违规预警（非门将进小禁区）——真机 9 次点球的根因
    our_gx = 220.0 if True else 0.0          # 监视窗默认按蓝队（守 x=220）；黄队时改这里
    bad = [(i, x, y) for i, (x, y, _r) in enumerate(s["ours"][1:], start=1)
           if our_gx - 50 <= x <= our_gx and 75 <= y <= 105]
    for i, (x, y, _r) in enumerate(s["ours"]):
        if 0 <= x <= MV.FW and 0 <= y <= MV.FH:
            MV.draw_robot(d, x, y, True, i)
    MV.draw_ball(d, s["ball"][0], s["ball"][1])
    if bad:
        d.rectangle([6, 6, FIELD_W - 6, H - 6], outline=(255, 60, 60), width=6)
        d.text((20, 20), "⚠️ 我方门区里非门将 2 人以上 → 会被判点球！",
               font=MV.font(22, True), fill=(255, 90, 90))
    # 我方两人挤在一起（<12cm）预警：挤在一起会互相挡路、也更容易连带犯规
    near = [(i, j) for i in range(5) for j in range(i + 1, 5)
            if MV.dist(s["ours"][i][0], s["ours"][i][1],
                       s["ours"][j][0], s["ours"][j][1]) < 12.0]
    if near:
        d.text((20, H - 52), "⚠️ 我方 " + "、".join(
            f"{MV.ROLE_NAME[i]}+{MV.ROLE_NAME[j]}" for i, j in near) + " 挤在一起（<12cm）",
            font=MV.font(20, True), fill=(255, 190, 90))
    # 右侧信息栏
    x0 = FIELD_W + 20
    d.text((x0, 30), "Hnnu 实时监视", font=MV.font(28, True), fill=MV.TXT)
    rows = [f"帧号 {s['frame']}",
            f"状态 {STATE.get(s['gs'], s['gs'])}",
            f"球权 {'我方' if s['ours_ball'] else '对方/中立'}",
            f"球位 ({s['ball'][0]:.1f}, {s['ball'][1]:.1f})",
            ""]
    for i, (_x, _y, role) in enumerate(s["ours"]):
        rows.append(f"{MV.ROLE_NAME[i]:<3}（{i} 号）")
    d.text((x0 + 10, 80), "\n".join(rows), font=MV.font(19), fill=MV.DIM, spacing=8)
    for i in range(5):
        d.ellipse([x0 + 8, 84 + 19 * (5 + i) - 6, x0 + 20, 84 + 19 * (5 + i) + 6],
                  fill=MV.ROLE_COLOR[i])
    d.text((20, H - 26), "数据源：C:\\Strategy\\hnnu_blackbox.csv（逐帧）  "
                         "蓝=我方(带角色名)  黄圈=对手", font=MV.font(16), fill=MV.DIM)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=CSV_DEF)
    ap.add_argument("--snapshot", default=None, help="只画一帧存成 PNG（自检用）")
    ap.add_argument("--hz", type=float, default=4.0)
    a = ap.parse_args()

    s = read_latest(a.csv)
    if a.snapshot:
        img = Image.new("RGB", (W, H), MV.BG)
        d = ImageDraw.Draw(img)
        if s:
            draw(d, s)
        else:
            d.text((30, 30), "读不到黑匣子数据", font=MV.font(24, True), fill=(255, 120, 120))
        img.save(a.snapshot)
        print("已存", a.snapshot, "（数据:", "有" if s else "无", "）")
        return 0

    import tkinter as tk
    root = tk.Tk()
    root.title("Hnnu 角色实时监视")
    canvas = tk.Canvas(root, width=W, height=H, highlightthickness=0)
    canvas.pack()
    img_id = canvas.create_image(0, 0, anchor="nw")
    st = {"t": 0.0}

    def tick():
        s2 = read_latest(a.csv)
        img = Image.new("RGB", (W, H), MV.BG)
        d = ImageDraw.Draw(img)
        if s2:
            draw(d, s2)
            st["t"] = time.time()
        else:
            d.text((30, 30), "等待黑匣子数据…（先启动平台并让策略 DLL 跑起来）",
                   font=MV.font(22, True), fill=(255, 200, 120))
        if st["t"] and time.time() - st["t"] > 3.0:
            d.text((30, 70), "⚠️ 数据超过 3 秒没更新", font=MV.font(20, True), fill=(255, 120, 120))
        canvas.image = _to_tk(img, tk)
        canvas.itemconfig(img_id, image=canvas.image)
        root.after(int(1000 / max(a.hz, 0.5)), tick)

    def _to_tk(im, tkmod):
        from PIL import ImageTk
        return ImageTk.PhotoImage(im)

    tick()
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
