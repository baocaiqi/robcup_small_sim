# -*- coding: utf-8 -*-
"""replay_player.py — 赛后回放器：读黑匣子 CSV（或 .rlg）里**记录下来的整段数据**，
自己画一张带角色名的俯视图，可以**自动播放**（带倍速）也可以**手动**拖时间轴/逐帧看。

为什么用它代替实时窗：实时窗要"边跑边抓文件"，天然有延迟；回放器把整段数据一次性读进内存，
拖动和播放都是本地渲染，**没有延迟**。

用法（在工作区根目录）：
    python tools\\py\\replay_player.py                    # 回放最新一段（黑匣子 CSV 最后一个 session）
    python tools\\py\\replay_player.py --session 2        # 倒数第 2 段
    python tools\\py\\replay_player.py --rlg "C:\\Strategy\\xxx.rlg"   # 没有 CSV 时用 rlg（无角色信息）
    python tools\\py\\replay_player.py --check            # 不开窗口，只渲 3 张图自检
    python tools\\py\\replay_player.py --player 3         # 直接以第 3 台为"主角"视角？(保留参数)

界面操作：
    ▶/⏸ 播放·暂停（或空格）    ←/→ 单帧后退·前进    Shift+←/→ 退/进 1 秒
    时间轴滑块 = 手动拖到任意帧    倍速：0.25× / 0.5× / 1× / 2×
    「下一个死球」按钮 = 跳到下一次摆位/判罚（读 CSV 里的 P 行）
"""
import argparse
import collections
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_video as MV                     # 复用画场地/机器人/球/字体
from PIL import Image, ImageDraw

CSV_DEF = r"C:\Strategy\hnnu_blackbox.csv"
W, H = 980, 760
FIELD_W = 760
STATE = {0: "PlayOn", 1: "FreeBall_LT", 2: "FreeBall_LB", 3: "FreeBall_RT", 4: "FreeBall_RB",
         5: "开球-黄", 6: "开球-我方", 7: "点球-黄方主罚", 8: "点球-我方主罚",
         9: "任意球-黄", 10: "任意球-我方", 11: "门球-黄", 12: "门球-我方"}


class Rec:
    """一段录制：逐帧数据 + 摆位/判罚事件"""

    def __init__(self):
        self.frames = []          # (frame, ball, ours[(x,y,role)], opps[(x,y)], gs, ours_ball)
        self.events = []          # (frame, 说明)


def load_csv(path, session_from_end=1):
    """读黑匣子 CSV 的某一段（默认最后一段）。R/O 行给位置，F 行给球/状态。"""
    ls = io.open(path, encoding="utf-8", errors="replace").read().splitlines()
    marks = [k for k, l in enumerate(ls) if l.startswith("#")]
    if not marks:
        return None
    i = max(0, len(marks) - session_from_end)
    seg = ls[marks[i]:]
    rec = Rec()
    F, R, O = {}, {}, {}
    order = []
    for l in seg:
        t = l.split(",")
        try:
            if t[0] == "F":
                fn = int(t[1]); F[fn] = t
                if fn not in order:
                    order.append(fn)
            elif t[0] == "R":
                R[int(t[1])] = [float(v) for v in t[3:]]
            elif t[0] == "O":
                O[int(t[1])] = [float(v) for v in t[1:]]
            elif t[0] == "P":
                rec.events.append((int(t[1]), f"摆位 {t[2]} 状态{int(t[3])}"
                                              f"({STATE.get(int(t[3]),'?')})"))
        except (ValueError, IndexError):
            continue
    for fn in order:
        f = F.get(fn)
        if not f:
            continue
        try:
            ours = []
            if fn in R:
                r = R[fn]
                # R 行每组 4 个数：x, y, rot, role
                ours = [(r[4 * k], r[4 * k + 1], int(r[4 * k + 3]), r[4 * k + 2])
                        for k in range(5)]
            opps = []
            if fn in O:
                o = O[fn]
                # O 行每组 3 个数：x, y, rot
                opps = [(o[1 + 3 * k], o[2 + 3 * k], o[3 + 3 * k]) for k in range(5)]
            rec.frames.append((fn, (float(f[4]), float(f[5])), ours, opps,
                               int(f[2]), int(f[8])))
        except (ValueError, IndexError):
            continue
    rec.events.sort()
    return rec if rec.frames else None


def load_rlg(path):
    """没有 CSV 时用 .rlg（有球+双方位置，但没有角色与状态）"""
    rec = Rec()
    for fr in MV.parse_rlg(path):
        blue, yel, ball, gs = fr
        rec.frames.append((len(rec.frames), ball, [(x, y, i, 0.0) for i, (x, y) in enumerate(blue)],
                           yel, gs, 0))
    return rec if rec.frames else None


def arrow(d, x, y, rot_deg, color, length=26, width=3):
    """从机器人中心朝 rot 方向画箭头（场地 y 向上 → 屏幕 y 要取反）"""
    import math
    a = math.radians(rot_deg)
    x1 = MV.field_px(x, y)[0], MV.field_px(x, y)[1]
    ex, ey = x1[0] + math.cos(a) * length, x1[1] - math.sin(a) * length
    d.line([x1[0], x1[1], ex, ey], fill=color, width=width)
    for sgn in (+1, -1):                      # 箭头两撇
        b = a + math.pi + sgn * 0.42
        d.line([ex, ey, ex + math.cos(b) * 9, ey - math.sin(b) * 9], fill=color, width=width)


def draw_frame(d, fr, idx, total, extra=""):
    fn, ball, ours, opps, gs, ours_ball = fr
    MV.FIELD_W = FIELD_W
    d.rectangle([0, 0, FIELD_W, H], fill=MV.FIELD_BG)
    MV.draw_field(d)
    for (x, y, rot) in opps:                       # 对手（黄圈 + 细箭头）
        if 0 <= x <= MV.FW and 0 <= y <= MV.FH:
            MV.draw_robot(d, x, y, False, 0)
            arrow(d, x, y, rot, (240, 220, 90), length=22, width=2)
    bad = []
    for i, (x, y, _r, rot) in enumerate(ours):     # 我方（实心 + 角色色箭头）
        if 0 <= x <= MV.FW and 0 <= y <= MV.FH:
            MV.draw_robot(d, x, y, True, i)
            arrow(d, x, y, rot, MV.ROLE_COLOR.get(i, (255, 255, 255)), length=28, width=3)
        if i >= 1 and 170 <= x <= 220 and 75 <= y <= 105:
            bad.append(i)
    MV.draw_ball(d, ball[0], ball[1])
    if bad:
        d.rectangle([6, 6, FIELD_W - 6, H - 6], outline=(255, 60, 60), width=6)
        d.text((20, 20), "⚠️ 我方门区里非门将 2 人以上 → 会被判点球！",
               font=MV.font(22, True), fill=(255, 90, 90))
    x0 = FIELD_W + 20
    d.text((x0, 24), "赛后回放", font=MV.font(26, True), fill=MV.TXT)
    lines = [f"帧 {fn}   ({idx+1}/{total})",
             f"时间 {fn/40.0:6.1f} s",
             f"状态 {STATE.get(gs, gs)}",
             f"球权 {'我方' if ours_ball else '对方/中立'}",
             f"球位 ({ball[0]:.1f}, {ball[1]:.1f})",
             ""]
    for i in range(5):
        lines.append(f"{MV.ROLE_NAME[i]:<3}（{i} 号）")
    d.text((x0 + 10, 70), "\n".join(lines), font=MV.font(19), fill=MV.DIM, spacing=8)
    for i in range(5):
        d.line([x0 + 8, 70 + 27 * (6 + i) - 6, x0 + 20, 70 + 27 * (6 + i) + 6],
               fill=MV.ROLE_COLOR[i], width=8)
    if extra:
        d.text((20, H - 30), extra, font=MV.font(18), fill=(255, 210, 130))
    d.text((20, H - 56), "箭头 = 机头朝向（角色的颜色）", font=MV.font(16), fill=MV.DIM)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=CSV_DEF)
    ap.add_argument("--rlg", default=None)
    ap.add_argument("--session", type=int, default=1, help="倒数第几段（默认最新一段）")
    ap.add_argument("--check", action="store_true", help="不开窗口，只渲 3 张图自检")
    ap.add_argument("--out", default="build/replay")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--export", default=None,
                    help="导出 mp4（整段渲染，不用开窗口；例如 --export 回放.mp4）")
    ap.add_argument("--export-fps", type=int, default=20)
    a = ap.parse_args()

    rec = load_rlg(a.rlg) if a.rlg else load_csv(a.csv, a.session)
    if not rec:
        print("✗ 读不到数据（检查 --csv / --rlg 路径）")
        return 2
    print(f"读入 {len(rec.frames)} 帧（{len(rec.frames)/40.0:.1f}s 比赛时间），"
          f"摆位/判罚事件 {len(rec.events)} 个")
    for fn, txt in rec.events[:8]:
        print(f"   帧 {fn:>6}  {txt}")

    if a.check:
        os.makedirs(a.out, exist_ok=True)
        for k, name in ((0, "a"), (len(rec.frames) // 2, "b"), (len(rec.frames) - 1, "c")):
            img = Image.new("RGB", (W, H), MV.BG)
            d = ImageDraw.Draw(img)
            draw_frame(d, rec.frames[k], k, len(rec.frames), "后台自检")
            p = os.path.join(a.out, f"check_{name}.png")
            img.save(p)
            print("  已存", p)
        return 0

    if a.export:                      # —— 导出 mp4：整段渲染，平时直接看这个 ——
        import subprocess
        ff = MV._find_ffmpeg()
        if not ff:
            print("✗ 没有 ffmpeg（pip install imageio-ffmpeg）")
            return 3
        step = max(1, int(40 / max(a.export_fps, 1)))     # 数据 40 帧/秒 → 目标帧率
        proc = subprocess.Popen(
            [ff, "-y", "-f", "image2pipe", "-vcodec", "mjpeg", "-framerate", str(a.export_fps),
             "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
             "-preset", "veryfast", a.export],
            stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        n = 0
        for k in range(0, len(rec.frames), step):
            img = Image.new("RGB", (W, H), MV.BG)
            d = ImageDraw.Draw(img)
            nxt = next((t for t in rec.events if t[0] > rec.frames[k][0]), None)
            extra = f"下一个事件：帧 {nxt[0]} {nxt[1]}" if nxt else "（本段结束）"
            draw_frame(d, rec.frames[k], k, len(rec.frames), extra)
            img.save(proc.stdin, format="JPEG", quality=88)
            n += 1
            if n % 400 == 0:
                print(f"  已渲染 {n} 帧…", flush=True)
        proc.stdin.close()
        proc.wait(timeout=600)
        import os as _os
        if _os.path.exists(a.export):
            print(f"✓ 导出完成：{a.export}（{n} 帧 / {_os.path.getsize(a.export)/1048576:.1f} MB）")
            return 0
        print("✗ 导出失败")
        return 4

    import tkinter as tk
    from PIL import ImageTk
    root = tk.Tk()
    root.title("Hnnu 赛后回放")
    canvas = tk.Canvas(root, width=W, height=H, highlightthickness=0)
    canvas.pack()
    img_id = canvas.create_image(0, 0, anchor="nw")
    st = {"i": 0, "play": False, "speed": a.speed, "ev": 0}

    def render():
        i = max(0, min(st["i"], len(rec.frames) - 1))
        img = Image.new("RGB", (W, H), MV.BG)
        d = ImageDraw.Draw(img)
        nxt = next((t for t in rec.events if t[0] > rec.frames[i][0]), None)
        extra = f"下一个事件：帧 {nxt[0]} {nxt[1]}" if nxt else "（本段结束）"
        draw_frame(d, rec.frames[i], i, len(rec.frames), extra)
        canvas.image = ImageTk.PhotoImage(img)
        canvas.itemconfig(img_id, image=canvas.image)

    bar = tk.Frame(root)
    bar.pack(fill="x")
    tk.Button(bar, text="⏮", command=lambda: step(-1)).pack(side="left")
    tk.Button(bar, text="⏪1s", command=lambda: step(-40)).pack(side="left")
    btn_play = tk.Button(bar, text="▶ 播放")
    btn_play.pack(side="left")
    tk.Button(bar, text="1s⏩", command=lambda: step(40)).pack(side="left")
    tk.Button(bar, text="⏭", command=lambda: step(1)).pack(side="left")
    tk.Button(bar, text="下一个死球", command=jump_event).pack(side="left")
    tk.Button(bar, text="⏱ 前进5秒", command=lambda: step(200)).pack(side="left")
    scale = tk.Scale(bar, from_=0, to=len(rec.frames) - 1, orient="horizontal",
                     length=430, label="时间轴（拖动=手动）",
                     command=lambda v: (st.update(i=int(float(v))), render()))
    scale.pack(side="left")
    tk.Label(bar, text="倍速").pack(side="left")
    for sp in (0.25, 0.5, 1.0, 2.0):
        tk.Button(bar, text=f"{sp}×", command=lambda s=sp: st.update(speed=s)).pack(side="left")

    def step(dn):
        st["i"] = max(0, min(st["i"] + dn, len(rec.frames) - 1))
        scale.set(st["i"])
        render()

    def jump_event():
        for t in rec.events:
            if t[0] > rec.frames[st["i"]][0]:
                st["i"] = next((k for k, f in enumerate(rec.frames) if f[0] >= t[0]), st["i"])
                scale.set(st["i"]); render(); return
        print("（没有更多事件）")

    def toggle():
        st["play"] = not st["play"]
        btn_play.config(text="⏸ 暂停" if st["play"] else "▶ 播放")

    btn_play.config(command=toggle)
    root.bind("<space>", lambda e: toggle())
    root.bind("<Left>", lambda e: step(-1))
    root.bind("<Right>", lambda e: step(1))
    root.bind("<Shift-Left>", lambda e: step(-40))
    root.bind("<Shift-Right>", lambda e: step(40))

    def tick():
        if st["play"]:
            step(max(1, int(40 / 40 * 2)))          # 每 25ms 前进 2 帧 ≈ 2×；倍速由间隔调
            if st["i"] >= len(rec.frames) - 1:
                st["play"] = False
                btn_play.config(text="▶ 播放")
        root.after(int(25 / max(st["speed"], 0.05)), tick)

    render()
    tick()
    print("窗口已打开：空格=播放/暂停，←/→=逐帧，Shift+←/→=1 秒，滑块=手动拖")
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
