# -*- coding: utf-8 -*-
"""make_goal_clip.py — 把"我方进球"从 .rlg 里精确找出来，渲染成回放片段（mp4）。

为什么需要它：录屏那 125 秒只覆盖了比赛前 ~42 秒，我方两个进球都在录屏结束之后
（详见与用户的对话：墙上时间 ≈ 3 × 纯比赛时间）。所以改用数据渲染——位置是真机真实数据，
画面是我们画的（带角色标注），进球时刻由数据精确定位。

用法：
  python tools/py/make_goal_clip.py --rlg "C:\\Strategy\\xxx.rlg" --out build/goals.mp4
"""
import argparse
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_video as MV          # 复用画场地/机器人/球/信息栏的函数
from PIL import Image, ImageDraw

FPS = 20
PRE_S, POST_S = 7.0, 3.0         # 每个进球：前面 7 秒铺垫 + 后面 3 秒
GOAL_X = 0.0                     # 我方（蓝队）攻 x=0；球越过门线且在门框内 = 我方进球


def find_our_goals(frames):
    """我方进球帧：球贴到对方门线(x≈0)且 y∈门框[70,110]，随后位置瞬移（平台复位）"""
    out = []
    for i in range(1, len(frames)):
        b0 = frames[i - 1][2]
        b1 = frames[i][2]
        if b0[0] < 6.0 and 70.0 <= b0[1] <= 110.0 and abs(b1[0] - b0[0]) > 25.0:
            if not out or i - out[-1] > 400:      # 同一个进球只记一次
                out.append(i - 1)
    return out


def render(frames, idx, out_path, ffmpeg, title):
    """渲染 idx-PRE .. idx+POST 这段（20fps 输出），画场地/双方/球/角色标注"""
    n_out = int((PRE_S + POST_S) * FPS)
    proc = subprocess.Popen(
        [ffmpeg, "-y", "-f", "image2pipe", "-vcodec", "mjpeg", "-framerate", str(FPS),
         "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
         "-preset", "slow", out_path],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    start = int(idx - PRE_S * 40)                 # .rlg 是 40 帧/秒
    for k in range(n_out):
        fi = start + int(k * 2)                   # 40fps → 20fps：隔一帧取一
        fi = max(0, min(fi, len(frames) - 1))
        blue, yel, ball, _gs = frames[fi]
        img = Image.new("RGB", (MV.W, MV.H), MV.BG)
        d = ImageDraw.Draw(img)
        MV.draw_field(d)
        for j, (x, y) in enumerate(yel):
            if 0 <= x <= MV.FW and 0 <= y <= MV.FH:
                MV.draw_robot(d, x, y, False, j)
        for j, (x, y) in enumerate(blue):
            if 0 <= x <= MV.FW and 0 <= y <= MV.FH:
                MV.draw_robot(d, x, y, True, j)
        MV.draw_ball(d, ball[0], ball[1])
        t = k / FPS - PRE_S
        near = (abs(fi - idx) <= FPS)             # 进球前后 1 秒 → 打 GOAL 横幅
        MV.panel(d, [f"真机数据回放（第 {fi} 帧）", f"球位：({ball[0]:.0f}, {ball[1]:.0f})",
                     "", "蓝=我方（标注角色）  黄圈=对手"],
                  title=title, caption="画面：真机 .rlg 逐帧真实坐标")
        if near:
            d.rectangle([MV.FIELD_W // 2 - 150, 40, MV.FIELD_W // 2 + 150, 110],
                        fill=(200, 40, 40))
            d.text((MV.FIELD_W // 2 - 92, 58), "GOAL!", font=MV.font(40, True), fill=(255, 255, 255))
        d.text((24, 16), f"t = {t:+.1f}s", font=MV.font(20, True), fill=(255, 220, 120))
        img.save(proc.stdin, format="JPEG", quality=90)
    proc.stdin.close()
    proc.wait(timeout=120)
    return os.path.exists(out_path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rlg", required=True)
    ap.add_argument("--out", default="build/goals.mp4")
    a = ap.parse_args()

    import make_video as _mv
    ff = _mv._find_ffmpeg()
    if not ff:
        print("✗ 没有 ffmpeg")
        return 2
    frames = MV.parse_rlg(a.rlg)
    goals = find_our_goals(frames)
    print(f"读入 {len(frames)} 帧；找到我方进球 {len(goals)} 个：{goals}（≈{['%.0fs' % (g/40) for g in goals]} 纯比赛时间）")
    if not goals:
        print("✗ 没找到我方进球")
        return 1

    parts = []
    for k, g in enumerate(goals[:2], 1):          # 最多取 2 个
        p = f"build/goal_{k}.mp4"
        if render(frames, g, p, ff, f"我方进球 #{k}"):
            parts.append(p)
            print(f"  ✓ {p}（{os.path.getsize(p)/1048576:.1f} MB）")
    # 拼接（两段用 concat demuxer，避免重编码损失）
    # 拼接：用 concat **滤镜**（两个 -i 直接给路径），避免中文路径在 concat demuxer 里转义出错
    cmd = [ff, "-y"]
    for p in parts:
        cmd += ["-i", p]
    cmd += ["-filter_complex", "".join(f"[{k}:v]" for k in range(len(parts)))
            + f"concat=n={len(parts)}:v=1:a=0[v]", "-map", "[v]",
            "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", "-preset", "slow", a.out]
    subprocess.run(cmd, capture_output=True, check=False)
    if os.path.exists(a.out):
        print(f"✓ 合成：{a.out}（{os.path.getsize(a.out)/1048576:.1f} MB）")
        return 0
    print("✗ 合成失败")
    return 1


if __name__ == "__main__":
    sys.exit(main())
