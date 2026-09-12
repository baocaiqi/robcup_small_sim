# -*- coding: utf-8 -*-
"""
formation_diagram.py — 生成「12 态摆位示意图」（PNG）

用法：
    python tools/py/formation_diagram.py                 # 输出 docs/work/formation12.png
    python tools/py/formation_diagram.py --check          # 顺便把 src/formation.cpp 里的坐标抄出来核对

口径（蓝队视角：我们守 x=220 右门、攻 x=0 左门；黄队时镜像）：
  · 每格 = 一个"摆位分支"（谁先摆/谁后摆、我方 5 人站哪、球在哪）
  · 红色虚线区 = 对方门区（进攻方 2+ 人 / 单人>20 周期 → 判点球）→ **禁入**
  · 蓝色虚线区 = 我方门区（除门将外不得停留）→ **只有门将**
  · 队友编号 = 角色 id：1=门将 2=抢球ACTIVE 3=助攻ASSIST 4=中场MID 5=拖后PASSIVE
数据来源：src/formation.cpp（formation_former / formation_later / formation_set_ball），
        坐标与代码保持一致；改动摆位后重跑本脚本即可刷新图。
"""
import argparse
import os
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
L, W = 220.0, 180.0          # 场地 cm
SCALE = 3.2                  # px/cm
PAD = 26


def load_font(size):
    """必须用中文字体，否则中文标注会渲染成方块/乱码"""
    for p in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf",
              r"C:\Windows\Fonts\simsun.ttc", r"C:\Windows\Fonts\arial.ttf"):
        try:
            return ImageFont.truetype(p, size)
        except OSError:
            continue
    return ImageFont.load_default()


FONT_T = load_font(30)   # 标题
FONT_S = load_font(20)   # 副标题/备注/号码

# (标题, 副标题, 我方5人[(id,x,y,rot)], 球(x,y), 备注)
# 坐标 = formation.cpp 里 M(c,x)/y 的值（蓝队）；rot: 门将 -90 / 队员 180
GOALIE = (0, 215, 90, -90)
F = 180.0                    # field_rot

PANELS = [
    ("开球 · 我方开球（先摆）", "formation_former → kickoff_formation",
     [GOALIE, (1, 100, 90, F), (2, 150, 60, F), (3, 150, 120, F), (4, 185, 90, F)],
     (110, 90), "注释写“球后 10cm”，但代码给的是球前 10cm(100<110) → 见问答"),
    ("开球 · 对方开球（后摆）", "formation_later → defense_formation",
     [GOALIE, (1, 190, 90, F), (2, 150, 90, F), (3, 130, 60, F), (4, 130, 120, F)],
     (110, 90), "后摆方能看到对方摆位+球位（我们现在没用这个信息）"),
    ("争球 · 先摆（4 个角区通用）", "formation_former → freeball_formation",
     [GOALIE, (1, 55 + 25, 135, F), (2, 150, 60, F), (3, 150, 120, F), (4, 185, 90, F)],
     (55, 135), "球位是**估计**的 1/4 场中心（先摆方拿不到真实球位）"),
    ("争球 · 后摆（拿得到球位）", "formation_later → freeball_formation",
     [GOALIE, (1, 55 + 25, 135, F), (2, 150, 60, F), (3, 150, 120, F), (4, 185, 90, F)],
     (55, 135), "后摆方用平台给的 ball.x/y（本例仍是 (55,135)）"),
    ("点球 · 我方被罚（防守方先摆）", "formation_former → PM_PenaltyKick_Blue",
     [GOALIE, (1, 130, 60, F), (2, 130, 120, F), (3, 150, 90, F), (4, 180, 90, F)],
     (128, 90), "球点=我方门前 92cm；除门将外别进小禁区"),
    ("点球 · 我方主罚（后摆）", "formation_later → PM_PenaltyKick_*",
     [GOALIE, (1, 92 - 10, 90, F), (2, 150, 60, F), (3, 150, 120, F), (4, 180, 90, F)],
     (92, 90), "主罚者=ACTIVE，站球后 10cm（离对方门远侧）"),
    ("任意球 · 我方主罚（先摆）", "formation_former → PM_FreeKick_Blue",
     [GOALIE, (1, 65, 90, F), (2, 150, 60, F), (3, 150, 120, F), (4, 185, 90, F)],
     (55, 90), "FK 点也是**估计**（真实点=犯规地，先摆方看不到）"),
    ("任意球 · 对方主罚（后摆）", "formation_later → PM_FreeKick_*",
     [GOALIE, (1, 190, 90, F), (2, 150, 90, F), (3, 130, 60, F), (4, 130, 120, F)],
     (165, 90), "用默认防守阵型（同样没用上球位信息）"),
    ("门球 · 我方发球（先摆 + SetBall）", "formation_former → PM_GoalKick_Blue；SetBall 放球",
     [GOALIE, (1, 190, 100, F), (2, 170, 65, F), (3, 150, 40, F), (4, 130, 130, F)],
     (210, 90), "球由 SetBall 放门前 10cm、y=90（修掉“门柱线死循环”那个坑）"),
    ("门球 · 对方发球（后摆）", "formation_later → PM_GoalKick_*",
     [GOALIE, (1, 190, 90, F), (2, 150, 90, F), (3, 130, 60, F), (4, 130, 120, F)],
     (10, 90), "默认防守阵型；注意对方球位由对方 SetBall 决定"),
    ("默认防守阵型（后摆兜底）", "defense_formation（多种状态复用）",
     [GOALIE, (1, 190, 90, F), (2, 150, 90, F), (3, 130, 60, F), (4, 130, 120, F)],
     None, "门将门线 + 190/150/130 梯次散开"),
    ("PlayOn / 其它状态", "formation_* 的 default 分支",
     [GOALIE, (1, 60, 90, F), (2, 60, 90, F), (3, 60, 90, F), (4, 60, 90, F)],
     None, "**不摆位**（default: break）→ 保持平台/上一帧的位置；比赛中的跑位靠 roles/situation"),
]


def draw_panel(d, ox, oy, title, sub, robots, ball, note):
    def X(x):
        return ox + PAD + x * SCALE

    def Y(y):
        return oy + PAD + (W - y) * SCALE

    def box(x0, y0, x1, y1):
        """把场地坐标转成 PIL 需要的 (左,上,右,下)（Y 是反的 → 必须排序）"""
        xa, xb = sorted((X(x0), X(x1)))
        ya, yb = sorted((Y(y0), Y(y1)))
        return [xa, ya, xb, yb]

    fw, fh = L * SCALE, W * SCALE
    # 场地
    d.rectangle([X(0), Y(W), X(L), Y(0)], fill=(30, 110, 45), outline=(255, 255, 255), width=2)
    # 中线 / 中圈
    d.line([X(110), Y(0), X(110), Y(W)], fill=(255, 255, 255), width=1)
    d.ellipse(box(110 - 25, 90 - 25, 110 + 25, 90 + 25), outline=(255, 255, 255))
    # 球门
    d.line([X(0), Y(70), X(0), Y(110)], fill=(255, 230, 120), width=5)
    d.line([X(L), Y(70), X(L), Y(110)], fill=(255, 230, 120), width=5)
    # 门区 / 罚球区（左=对方门，右=我方门）
    for gx, sgn in ((0.0, 1), (L, -1)):
        for depth, half, color in ((50, 15, (255, 120, 120)), (80, 35, (255, 200, 120))):
            d.rectangle(box(gx, 90 - half, gx + sgn * depth, 90 + half),
                        outline=color, width=1)
    # 我方门区填色（除门将禁入）
    d.rectangle(box(L - 50, 75, L, 105), outline=(120, 170, 255), width=2)
    # 球
    if ball:
        bx, by = ball
        d.ellipse([X(bx) - 5, Y(by) - 5, X(bx) + 5, Y(by) + 5], fill=(255, 255, 255), outline=(0, 0, 0))
    # 我方机器人
    for i, x, y, rot in robots:
        col = (255, 255, 0) if i == 0 else (120, 200, 255)
        d.ellipse([X(x) - 7, Y(y) - 7, X(x) + 7, Y(y) + 7], fill=col, outline=(0, 0, 0), width=2)
        d.text((X(x) - 4, Y(y) - 8), str(i + 1), fill=(0, 0, 0), font=FONT_S)
    # 标题/备注
    d.text((ox + 6, oy + 2), title, fill=(255, 255, 255), font=FONT_T)
    d.text((ox + 6, oy + 26), sub, fill=(190, 220, 190), font=FONT_S)
    d.text((ox + 6, oy + PAD + fh + 2), note[:78], fill=(230, 230, 160), font=FONT_S)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join("docs", "work", "formation12.png"))
    ap.add_argument("--check", action="store_true", help="把 formation.cpp 里的 put(...) 数出来核对")
    args = ap.parse_args()

    if args.check:
        txt = open(os.path.join(ROOT, "src", "formation.cpp"), encoding="utf-8").read()
        print("formation.cpp 里 put(...) 出现次数：", txt.count("put(robots") + txt.count("put(laterRobots"))
        print("（本脚本的坐标表按 2026-09-12 的 formation.cpp 手抄；若改过摆位请同步表格）")

    cols, rows = 3, 4
    pw = int(L * SCALE) + 2 * PAD
    ph = int(W * SCALE) + 2 * PAD + 18
    img = Image.new("RGB", (pw * cols, ph * rows), (18, 24, 18))
    d = ImageDraw.Draw(img)
    for k, p in enumerate(PANELS):
        ox, oy = (k % cols) * pw, (k // cols) * ph
        draw_panel(d, ox, oy, *p)
    out = os.path.join(ROOT, args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    img.save(out)
    print(f"已生成 {args.out}（{img.width}×{img.height}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
