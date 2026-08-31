# -*- coding: utf-8 -*-
# 一次性脚本：在 run_goalie 中插入「死角封堵」判断。
# roles.cpp 是 UTF-16 LE + LF，用 io.open 读写，newline='' 保持 LF。
import io

PATH = 'src/roles.cpp'

block = (
    "\n    // ============================================================\n"
    "    // 死角封堵：球贴门线且逼近门柱（近柱/远柱）时，优先封死该侧角度。\n"
    "    //   复盘 0:2 第1球：球沿底线从 y=67 滚进 y=73（下柱），门将却走\n"
    "    //   「绕到门侧」解围站到 y=87，漏掉下柱角度被打进。这里在解围之前\n"
    "    //   优先站位贴柱封堵，防止球沿底线溜进门。\n"
    "    // ============================================================\n"
    "    const double kDeadZoneDist = 40.0;   // 球离门线多近算「贴门线」(cm)\n"
    "    const double kPostBand     = 12.0;   // 门柱附近带宽：球/落点落此带视为死角威胁\n"
    "    bool near_line = ctx.dist_our_goal(bx) < kDeadZoneDist;\n"
    "    bool near_post = (by < goal_y_low() + kPostBand) || (by > goal_y_high() - kPostBand);\n"
    "    bool heading_post = heading_goal &&\n"
    "        (y_at_goal < goal_y_low() + kPostBand || y_at_goal > goal_y_high() - kPostBand);\n"
    "    if (near_line && (near_post || heading_post)) {\n"
    "        double post_y = (by < 90.0) ? goal_y_low() : goal_y_high();\n"
    "        double block_y = clamp(by, post_y - 6.0, post_y + 6.0);\n"
    "        block_y = clamp(block_y, goal_y_low(), goal_y_high());\n"
    "        motion::position(r, ctx.our_goal_x() + ctx.attack_dir() * 6.0, block_y);\n"
    "        return;\n"
    "    }\n"
)

anchor = "clamp_goalie_area(ctx, out_x, iy);\n"

t = io.open(PATH, encoding='utf-16').read()
assert t.count(anchor) == 1, f'anchor count = {t.count(anchor)}'
t = t.replace(anchor, anchor + block, 1)
io.open(PATH, 'w', encoding='utf-16', newline='').write(t)
print('inserted deadzone block OK')
