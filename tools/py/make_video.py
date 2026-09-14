# -*- coding: utf-8 -*-
"""make_video.py — 用真机 .rlg 数据 + 卡片动画，生成提交用视频的帧（并尽量直接编码成 mp4）。

为什么这样做：工作区里没有现成的比赛录像文件，但 `.rlg` 里记录着**每一帧 10 台机器人 + 球的真实坐标**
（352B/帧 = 44 个 double），所以可以把真机比赛"重放"成俯视战术动画——数据是真的，只是画法是我们画的。

输出（默认 build/video/）：
  frames/00000.jpg ...      10 fps 的帧序列（1280x720，JPEG）
  Hnnu策略视频.mp4          若系统里有 ffmpeg，则自动编码成 ≤10MB 的 mp4
  encode.bat                没 ffmpeg 时生成，装好后双击即可编码
  preview.gif               没 ffmpeg 时的低帧率预览（PIL 直接写，不需要外部程序）

用法：
  python tools/py/make_video.py --rlg "C:\\Strategy\\xxx.rlg"           # 全片 2:50
  python tools/py/make_video.py --rlg ... --limit 60                    # 只渲前 60 帧（调试）
  python tools/py/make_video.py --rlg ... --fps 10 --font msyh
"""
import argparse
import glob
import os
import struct
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
FW, FH = 220.0, 180.0              # 场地 cm
GOAL_HALF = 20.0                   # 球门半宽（y ∈ [70,110]）
W, H = 1280, 720                   # 视频分辨率
FIELD_W = 790                      # 场地绘制区宽度
FPS_DEF = 20

# 角色（按编号固定，见 src/role_assignment.cpp:11-15）
ROLE_NAME = {0: "门将", 1: "主攻", 2: "助攻", 3: "中场", 4: "后卫"}
ROLE_COLOR = {0: (255, 170, 60), 1: (255, 80, 80), 2: (80, 220, 120),
              3: (90, 170, 255), 4: (200, 130, 255)}

BG = (12, 18, 30)
FIELD_BG = (18, 32, 26)
LINE = (120, 160, 140)
TXT = (235, 240, 250)
DIM = (150, 165, 190)


def font(size, bold=False):
    for name in (("msyhbd.ttc", "msyh.ttc") if bold else ("msyh.ttc",)):
        p = os.path.join(r"C:\Windows\Fonts", name)
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except OSError:
                pass
    return ImageFont.load_default()


def field_px(x, y):
    """场地坐标(cm) → 像素。x 向右、y 向上 → 屏幕 y 翻转。"""
    m = 40
    s = min((FIELD_W - 2 * m) / FW, (H - 2 * m) / FH)
    ox = m + (FIELD_W - 2 * m - FW * s) / 2
    oy = H - m - (H - 2 * m - FH * s) / 2
    return ox + x * s, oy - y * s, s


def draw_field(d):
    """底图：场地 + 门区/罚球区/中圈/球门"""
    d.rectangle([0, 0, FIELD_W, H], fill=FIELD_BG)
    _, _, s = field_px(0, 0)
    x0, y0, _ = field_px(0, FH)
    x1, y1, _ = field_px(FW, 0)
    d.rectangle([x0, y0, x1, y1], outline=LINE, width=2)
    mx, _, _ = field_px(FW / 2, 0)
    d.line([mx, y0, mx, y1], fill=LINE, width=2)
    cx, cy, _ = field_px(FW / 2, FH / 2)
    d.ellipse([cx - 25 * s, cy - 25 * s, cx + 25 * s, cy + 25 * s], outline=LINE, width=2)
    # 两侧门区 50x30 / 罚球区 80x35
    for gx, ad in ((0.0, 1.0), (FW, -1.0)):
        for dep, half in ((50.0, 15.0), (80.0, 17.5)):
            a = field_px(gx, 90 - half)
            b = field_px(gx + ad * dep, 90 + half)
            d.rectangle([min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])],
                        outline=LINE, width=1)
        a = field_px(gx, 90 - GOAL_HALF)
        b = field_px(gx, 90 + GOAL_HALF)
        d.line([a[0], a[1], b[0], b[1]], fill=(120, 170, 255), width=6)


def draw_robot(d, x, y, ours, idx, label=True):
    px, py, s = field_px(x, y)
    r = max(7.0, 3.0 * s)
    if ours:
        col = ROLE_COLOR.get(idx, (120, 200, 255))
        d.ellipse([px - r, py - r, px + r, py + r], fill=col, outline=(10, 12, 18), width=2)
        if label:
            f = font(15, True)
            t = ROLE_NAME.get(idx, "?")
            d.text((px + r + 2, py - r - 6), t, font=f, fill=col,
                   stroke_width=3, stroke_fill=(8, 10, 16))
    else:
        d.ellipse([px - r, py - r, px + r, py + r], outline=(240, 220, 90), width=2)


def draw_ball(d, x, y):
    px, py, s = field_px(x, y)
    r = max(4.0, 1.6 * s)
    d.ellipse([px - r, py - r, px + r, py + r], fill=(255, 255, 255))


def wrap(s, n=24):
    """中文按字数折行（右侧信息栏只有约 440px，不折行会被截断）"""
    out, cur = [], ""
    for ch in s:
        cur += ch
        if len(cur) >= n:
            out.append(cur)
            cur = ""
    if cur:
        out.append(cur)
    return out


def panel(d, lines, title=None, score=None, caption=None, progress=None):
    """右侧信息栏 + 底部字幕/进度条"""
    x = FIELD_W + 24
    yy = 40
    if title:
        d.text((x, yy), title, font=font(30, True), fill=TXT)
        yy += 46
    if score:
        d.text((x, yy), score, font=font(22, True), fill=(255, 220, 120))
        yy += 40
    for ln in lines:
        for w in (wrap(ln) if ln else [""]):
            d.text((x, yy), w, font=font(18), fill=DIM)
            yy += 26
    if caption:
        d.rectangle([0, H - 62, W, H], fill=(8, 12, 20))
        d.text((24, H - 50), caption, font=font(21, True), fill=TXT)
    if progress is not None:
        d.rectangle([0, H - 4, W, H], fill=(40, 50, 70))
        d.rectangle([0, H - 4, int(W * progress), H], fill=(90, 170, 255))


# ============================================================
# .rlg 解析（快：直接 struct，不建 dict）
# ============================================================
def parse_rlg(path, off=0.0):
    """off 必须与 tools/py/constants.py 的 LOG_OFFSET_X 一致（当前 0.0：rlg 里的 x 已是 [0,220]）
    ⚠️ 别自作主张加偏移：我第一版写了 off=192，结果整场数据右移 192cm、全员出界。"""
    data = open(path, "rb").read()
    n = len(data) // 352
    frames = []
    for i in range(n):
        v = struct.unpack_from("<44d", data, i * 352)
        blue = [(v[0 + 4 * k] + off, v[1 + 4 * k]) for k in range(5)]
        yel = [(v[20 + 4 * k] + off, v[21 + 4 * k]) for k in range(5)]
        ball = (v[40] + off, v[41])
        frames.append((blue, yel, ball, int(v[42])))
    return frames


def pick_highlights(frames, seconds, fps, our_goal=220.0):
    """挑"我们在前场有威胁"的连续片段（球在对方半场且我们最近的人离球近）"""
    need = int(seconds * fps)
    best, best_score = 0, -1e9
    step = 8
    for start in range(0, max(1, len(frames) - need * step), need * step // 2):
        sc = 0.0
        for i in range(start, min(start + need * step, len(frames)), 24):
            _b, _y, ball, _gs = frames[i]
            if ball[0] < our_goal / 2:                 # 球在对方半场（我们是蓝队，攻 x=0）
                sc += 1.0
            dmin = min(((r[0] - ball[0]) ** 2 + (r[1] - ball[1]) ** 2) ** 0.5
                       for r in frames[i][0])
            sc += max(0.0, 1.0 - dmin / 60.0)
        if sc > best_score:
            best_score, best = sc, start
    return best, step


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rlg", default=None, help="真机录像（用于实战回放段）")
    ap.add_argument("--out", default=os.path.join(ROOT, "build", "video"))
    ap.add_argument("--fps", type=int, default=FPS_DEF)
    ap.add_argument("--limit", type=int, default=0, help="只渲前 N 帧（调试用）")
    ap.add_argument("--formation", default=os.path.join(ROOT, "docs", "work", "formation12.png"))
    a = ap.parse_args()

    fr_dir = os.path.join(a.out, "frames")
    os.makedirs(fr_dir, exist_ok=True)
    for f in glob.glob(os.path.join(fr_dir, "*.jpg")):
        os.remove(f)

    frames_rlg = []
    if a.rlg and os.path.exists(a.rlg):
        frames_rlg = parse_rlg(a.rlg)
        print(f"读入 {os.path.basename(a.rlg)}：{len(frames_rlg)} 帧真机数据")
    form_img = Image.open(a.formation).convert("RGB").resize((FIELD_W, H)) \
        if os.path.exists(a.formation) else None

    total_sec = 170.0
    total = int(total_sec * a.fps)
    if a.limit:
        total = min(total, a.limit)

    # —— 各段（秒）：与 docs/work/视频讲解稿.md 的时间轴一致 ——
    SEG = [
        ("title",   0.0,  12.0),
        ("frame",  12.0,  38.0),
        ("open",   50.0,  16.0),
        ("bank",   66.0,  16.0),
        ("form",   82.0,  20.0),
        ("play1", 102.0,  20.0),
        ("play2", 122.0,  20.0),
        ("disc",  142.0,  16.0),
        ("end",   158.0,  12.0),
    ]

    def seg_at(t):
        for name, st, dur in SEG:
            if st <= t < st + dur:
                return name, (t - st) / dur
        return "end", 1.0

    hl_start, hl_step = (0, 1)
    if frames_rlg:
        hl_start, hl_step = pick_highlights(frames_rlg, 20.0, a.fps)

    print(f"渲染 {total} 帧 @ {a.fps}fps …")
    for i in range(total):
        t = i / a.fps
        name, u = seg_at(t)
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        prog = i / max(1, total - 1)

        if name == "title":
            d.text((60, 150), "Hnnu 队 · 策略系统功能展示", font=font(52, True), fill=TXT)
            d.text((60, 240), "FIRA SimuroSot 5v5 仿真组", font=font(28), fill=DIM)
            d.text((60, 320), "① 总体设计框架   ② 进攻与防守策略   ③ 真机比赛回放",
                   font=font(26), fill=(140, 190, 255))
            d.text((60, 400), "全片 2 分 50 秒 · 画面为真机 .rlg 逐帧数据回放",
                   font=font(20), fill=DIM)
        elif name == "frame":
            draw_field(d)
            steps = [("① 读世界", "球 + 双方 10 台机器人的位置/朝向（40 帧/秒 = 每帧 25 毫秒）"),
                     ("② 判局势", "球权、威胁等级、攻防状态（连续 3 帧防抖）"),
                     ("③ 派活", "门将 0 / 主攻 1 / 助攻 2 / 中场 3 / 后卫 4，各自决策"),
                     ("④ 输出", "写出 5 台机器人左右轮速，由平台推进比赛")]
            k = int(u * len(steps)) if u < 1 else len(steps) - 1
            lines = [f"{s}：{txt}" for s, txt in steps[:k + 1]]
            panel(d, lines, title="总体设计框架", caption="每帧只做三件事：读世界 → 判局势 → 派活")
        elif name == "open":
            draw_field(d)
            # 净开口几何动画：球在对方门前，门将被"扫"着移动，开口随之变化
            bx, by = 100.0, 90.0
            gky = 90.0 + 14.0 * __import__("math").sin(u * 6.28)
            px, py, s = field_px(bx, by)
            gx, gy, _ = field_px(0.0, gky)
            d.line([px, py, gx, gy], fill=(255, 90, 90), width=2)
            for ty, col in ((70.0, (90, 200, 255)), (110.0, (90, 200, 255))):
                tx, ty2, _ = field_px(0.0, ty)
                d.line([px, py, tx, ty2], fill=col, width=1)
            draw_robot(d, bx, by, True, 1)
            draw_robot(d, 0.0, gky, False, 0)
            panel(d, ["门张角 = 球→两门柱的夹角",
                      "门将遮挡 = 门将半径 8cm 在该距离的张角",
                      "净开口 = 门张角 − 遮挡，取最宽处中心瞄准",
                      "",
                      "⇒ 门将挡住正中间时，我们瞄的是两侧空隙"],
                  title="进攻①：净开口几何", caption="把球门当扇面，只瞄门将挡不住的那块")
        elif name == "bank":
            draw_field(d)
            import math
            bx, by = 120.0, 30.0
            kRest, kFric = 0.66, 0.81          # 实测（118 场 rlg）
            ty = 90.0
            c1, c2 = kFric * ty, kRest * (0.0 - by)
            rx = (c1 * bx - c2 * 0.0) / (c1 - c2)
            px, py, s = field_px(bx, by)
            qx, qy, _ = field_px(rx, 0.0)
            gx, gy, _ = field_px(0.0, ty)
            d.line([px, py, qx, qy], fill=(255, 190, 80), width=3)
            d.line([qx, qy, gx, gy], fill=(255, 190, 80), width=3)
            draw_robot(d, bx, by, True, 1)
            panel(d, ["平台撞墙不是镜面反射：实测 118 场真机录像",
                      "  法向只保留 0.66、切向保留 0.81",
                      "⇒ 反弹点用方程解出（不是把球门镜像）",
                      f"本帧：球({bx:.0f},{by:.0f}) → 反弹点({rx:.1f},0) → 门内(y={ty:.0f})",
                      "",
                      "质量因子 = 0.30 遮挡 + 0.25 距离 + 0.25 反射点 + 0.20 球速"],
                  title="进攻②：借墙射门", caption="直线被封时，借边墙反弹换角度")
        elif name == "form":
            if form_img:
                img.paste(form_img, (0, 0))
                d.rectangle([0, H - 62, W, H], fill=(8, 12, 20))
                d.text((24, H - 50), "12 种比赛状态的摆位：开球/任意球/门球由主罚方先摆，点球由防守方先摆",
                       font=font(21, True), fill=TXT)
            else:
                panel(d, ["（未找到 docs/work/formation12.png）"], title="12 态摆位")
        elif name in ("play1", "play2"):
            if frames_rlg:
                base = hl_start + (0 if name == "play1" else int(20.0 * hl_step))
                fi = int(base + u * 20.0 * hl_step)
                blue, yel, ball, gs = frames_rlg[min(fi, len(frames_rlg) - 1)]
                draw_field(d)
                for k, (x, y) in enumerate(yel):
                    if 0 <= x <= FW and 0 <= y <= FH:
                        draw_robot(d, x, y, False, k)
                for k, (x, y) in enumerate(blue):
                    if 0 <= x <= FW and 0 <= y <= FH:
                        draw_robot(d, x, y, True, k)
                draw_ball(d, ball[0], ball[1])
                panel(d, [f"真机录像逐帧回放（第 {fi} 帧）",
                          f"球位：({ball[0]:.0f}, {ball[1]:.0f})",
                          "", "蓝=我方（标注角色）  黄圈=对手"],
                      title="实时比赛", caption="画面来自真机 .rlg 的真实坐标数据")
            else:
                panel(d, ["（未提供 --rlg，实战段留空）",
                          "用法：python tools/py/make_video.py --rlg <真机.rlg>"],
                      title="实时比赛")
        elif name == "disc":
            d.text((60, 60), "纪律：不是注意事项，是硬约束", font=font(38, True), fill=TXT)
            rows = [("对方门区内 2 人以上 / 单人停留 > 20 周期", "→ 判对方点球", "站位点一律不进对方门区"),
                    ("四角禁止推球区内推球", "→ 每 4 次判给对方 1 球", "球在角区不主动推球"),
                    ("死球（摆位）期间推球", "→ 判犯规", "死球期间不接触球"),
                    ("门区外僵持 100 周期无进展", "→ 判争球", "球长时间静止则主动处理")]
            y = 150
            for a1, a2, a3 in rows:
                d.text((60, y), a1, font=font(21), fill=(255, 200, 120))
                d.text((640, y), a2, font=font(21), fill=(255, 120, 120))
                d.text((60, y + 30), "我们的约束：" + a3, font=font(19), fill=DIM)
                y += 78
        else:
            d.text((60, 250), "把每次机会的把握率做高一点，", font=font(40, True), fill=TXT)
            d.text((60, 320), "把每次判罚的代价降到零。", font=font(40, True), fill=TXT)
            d.text((60, 420), "Hnnu 队 · 谢谢各位评委", font=font(26), fill=(140, 190, 255))

        panel(d, [], progress=prog)
        img.save(os.path.join(fr_dir, f"{i:05d}.jpg"), quality=88)

    print(f"帧已写入 {fr_dir}")

    # —— 编码 ——
    ff = None
    cands = ["ffmpeg", r"C:\ffmpeg\bin\ffmpeg.exe"]
    try:                       # imageio-ffmpeg 自带的静态 ffmpeg（本机就靠它，pip 装的）
        import imageio_ffmpeg
        cands.insert(0, imageio_ffmpeg.get_ffmpeg_exe())
    except Exception:
        pass
    for c in cands:
        try:
            subprocess.run([c, "-version"], capture_output=True, check=True)
            ff = c
            break
        except Exception:
            pass
    mp4 = os.path.join(ROOT, "Hnnu策略视频.mp4")   # 成片放工作区根目录（与策略说明书一致）
    gif = os.path.join(a.out, "preview.gif")
    if ff:
        cmd = [ff, "-y", "-framerate", str(a.fps), "-i", os.path.join(fr_dir, "%05d.jpg"),
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
               "-maxrate", "420k", "-bufsize", "840k", "-preset", "slow", mp4]
        print("编码中：", " ".join(cmd[:4]), "…")
        subprocess.run(cmd, check=False)
        if os.path.exists(mp4):
            print(f"✓ 视频：{mp4}（{os.path.getsize(mp4)/1048576:.1f} MB）")
    else:
        bat = os.path.join(a.out, "encode.bat")
        with open(bat, "w", encoding="utf-8") as f:
            f.write(f'ffmpeg -y -framerate {a.fps} -i "{fr_dir}\\%05d.jpg" -c:v libx264 '
                    f'-pix_fmt yuv420p -crf 30 -maxrate 420k -bufsize 840k "{mp4}"\n')
        print("系统里没有 ffmpeg → 已生成编码脚本：")
        print("   ", bat)
        print("   装好 ffmpeg 后双击它即可得到 ≤10MB 的 mp4")
        # 预览 GIF（每 4 帧取 1，缩小到 1/2）
        try:
            idx = list(range(0, total, 4))
            ims = [Image.open(os.path.join(fr_dir, f"{k:05d}.jpg")).resize((W // 2, H // 2))
                   for k in idx[:120]]
            if ims:
                ims[0].save(gif, save_all=True, append_images=ims[1:], duration=int(4000 / a.fps),
                            loop=0, optimize=True)
                print(f"   预览动图：{gif}（{os.path.getsize(gif)/1048576:.1f} MB，仅预览用）")
        except Exception as e:
            print("   预览动图生成失败：", e)
    return 0


if __name__ == "__main__":
    sys.exit(main())
