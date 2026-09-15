# -*- coding: utf-8 -*-
"""burn_subs.py — 把字幕烧进已有成片（**不覆盖原片**，另存 `Hnnu策略视频_带字幕.mp4`）。

两种做法自动选：
  ① ffmpeg 带 subtitles(libass) 滤镜 → 一条命令直接压（几十秒）；
  ② 没有 libass（imageio-ffmpeg 的精简版常常没有）→ 兜底：解码帧 → PIL 画字 → 重新编码。
字幕内容 = docs/work/字幕稿.md v2 的 27 句（650 字，按成片段落对齐，约 4 字/秒）。
"""
import glob
import os
import shutil
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, "Hnnu策略视频.mp4")
OUT = os.path.join(ROOT, "Hnnu策略视频_带字幕.mp4")
WORK = os.path.join(ROOT, "build", "subs")

# (起, 止, 文字)  单位：秒 —— 与 docs/work/字幕稿.md 一一对应
CUES = [
    (0, 5, "各位评委好，我们是 Hnnu 队。"),
    (5, 9, "接下来两分半，介绍我们的 5v5 策略系统。"),
    (9, 12, "先说设计框架，再看真机比赛画面。"),
    (12, 18, "平台每秒把全场状态交给我们 40 次，一帧 25 毫秒。"),
    (18, 24, "我们回一次五个机器人左右轮的速度，这就是全部控制量。"),
    (24, 31, "每帧只做三件事：读世界、判局势、派活。"),
    (31, 38, "读世界，是拿到球和双方十台机器人的位置与朝向。"),
    (38, 44, "判局势，是算球权、算威胁，再切换攻防状态。"),
    (44, 50, "派活最关键：门将、主攻、助攻、中场、后卫，角色固定、各守其责。"),
    (50, 55, "进攻第一件事：判断这一脚到底能不能射。"),
    (55, 60, "我们把球门看成一个扇面，先算出门张角。"),
    (60, 66, "再扣掉门将挡住的那一块，剩下的叫净开口，瞄它最宽的地方。"),
    (66, 71, "射不进去，就借边墙——球先撞墙，反弹进门。"),
    (71, 77, "注意，这个平台的反弹不是镜面反射。"),
    (77, 82, "我们实测 118 场真机录像：垂直墙的速度只剩 66%，反弹点是解方程算出来的。"),
    (82, 88, "定位球要自己摆位：平台把比赛分成 12 种状态。"),
    (88, 94, "开球、任意球、门球由主罚方先摆，点球由防守方先摆。"),
    (94, 102, "点球执行期平台不报状态，我们靠“球静止停在罚球点”识别，就地转正、立刻出脚。"),
    (102, 108, "这两段，是我们真机比赛里打进的球。"),
    (108, 114, "画面不是平台录屏，是用 .rlg 里逐帧真机数据渲染的动画。"),
    (114, 122, "机器人身上标的是它当时的角色：主攻、助攻、中场、后卫、门将。"),
    (122, 128, "这就是我们自研的回放工具：把比赛数据变成能直接看的画面。"),
    (128, 134, "复盘时对着它调站位、调射门时机，比看录像快得多。"),
    (134, 142, "它也是我们队的一个特色：策略、复盘、演示用同一套数据。"),
    (142, 147, "最后一条容易被忽略，但很值钱：纪律。"),
    (147, 152, "门区聚集、在角区推球、死球期间碰球，都会被判罚。"),
    (152, 158, "所以它们是写进策略里的硬约束：该不碰球的时候，绝不碰。"),
]


def find_ffmpeg():
    cands = ["ffmpeg", r"C:\ffmpeg\bin\ffmpeg.exe"]
    try:
        import imageio_ffmpeg
        cands.insert(0, imageio_ffmpeg.get_ffmpeg_exe())
    except Exception:
        pass
    for c in cands:
        try:
            subprocess.run([c, "-version"], capture_output=True, check=True)
            return c
        except Exception:
            pass
    return None


def has_libass(ff):
    r = subprocess.run([ff, "-hide_banner", "-filters"], capture_output=True, text=True,
                       errors="replace")
    return " subtitles " in r.stdout


def wrap(s, n=24):
    return [s[i:i + n] for i in range(0, len(s), n)] or [""]


def font(size):
    for name in ("msyhbd.ttc", "msyh.ttc", "simhei.ttf"):
        p = os.path.join(r"C:\Windows\Fonts", name)
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except OSError:
                pass
    return ImageFont.load_default()


def burn_pil(ff):
    """兜底：解码 → PIL 画字 → 重新编码（不覆盖原片）"""
    fr = os.path.join(WORK, "frames")
    shutil.rmtree(WORK, ignore_errors=True)
    os.makedirs(fr, exist_ok=True)
    print("① 解码成帧…")
    subprocess.run([ff, "-y", "-i", SRC, os.path.join(fr, "%05d.png")],
                   capture_output=True, check=False)
    files = sorted(glob.glob(os.path.join(fr, "*.png")))
    print(f"   共 {len(files)} 帧")
    fps = 20.0
    f = font(28)
    for k, p in enumerate(files):
        t = k / fps
        cur = next((c for c in CUES if c[0] <= t < c[1]), None)
        if not cur:
            continue
        im = Image.open(p).convert("RGB")
        d = ImageDraw.Draw(im)
        lines = wrap(cur[2])
        lh = 40
        y0 = im.height - 56 - lh * (len(lines) - 1)
        for i, ln in enumerate(lines):
            w = d.textlength(ln, font=f)
            d.text(((im.width - w) / 2, y0 + i * lh), ln, font=f, fill=(255, 255, 255),
                   stroke_width=4, stroke_fill=(0, 0, 0))
        im.save(p)
        if k % 400 == 0:
            print(f"   画字幕 {k}/{len(files)}…", flush=True)
    print("③ 重新编码…")
    r = subprocess.run([ff, "-y", "-framerate", str(fps), "-i", os.path.join(fr, "%05d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "21",
                        "-preset", "veryfast", "-maxrate", "500k", "-bufsize", "1M", OUT],
                       capture_output=True, check=False)
    return r.returncode == 0 and os.path.exists(OUT)


def burn_libass(ff):
    os.makedirs(WORK, exist_ok=True)
    srt = os.path.join(WORK, "subs.srt")          # 放在 ASCII 目录，避免滤镜路径转义问题

    def ts(x):
        return "00:%02d:%02d,%03d" % (int(x) // 60, int(x) % 60, int(x * 1000) % 1000)

    with open(srt, "w", encoding="utf-8-sig") as f:
        for i, (a, b, txt) in enumerate(CUES, 1):
            f.write(f"{i}\n{ts(a)} --> {ts(b)}\n{txt}\n\n")
    cmd = [ff, "-y", "-i", SRC, "-vf",
           "subtitles=subs.srt:force_style='FontName=Microsoft YaHei,FontSize=20,"
           "PrimaryColour=&H00FFFFFF,OutlineColour=&H00000000,Outline=2,Shadow=0,"
           "MarginV=28'",
           "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "21", "-preset", "veryfast",
           "-maxrate", "500k", "-bufsize", "1M", OUT]
    r = subprocess.run(cmd, cwd=WORK, capture_output=True, text=True, errors="replace")
    if r.returncode != 0:
        print(r.stderr[-600:])
    return r.returncode == 0 and os.path.exists(OUT)


def main():
    ff = find_ffmpeg()
    if not ff:
        print("✗ 没有 ffmpeg")
        return 2
    if not os.path.exists(SRC):
        print("✗ 找不到原片", SRC)
        return 3
    ok = burn_libass(ff) if has_libass(ff) else burn_pil(ff)
    print(("✓ " if ok else "✗ ") + ("已生成 " + OUT if ok else "字幕烧录失败"))
    if ok:
        print(f"   {os.path.getsize(OUT)/1048576:.1f} MB（原片未改动）")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
