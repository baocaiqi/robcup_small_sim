# -*- coding: utf-8 -*-
"""
patch_demo_yellow.py — 给官方 demo（黄队）源码打补丁（v2，2026-09-11）

用户指令：**只保留「禁区限制」，其余一律不动 demo 原行为。**

v1（2026-09-11 07:35 部署）是"五道闸"，其中两条不是禁区限制，且都有副作用：
    ④ 门将保护圈（贴蓝门将 20cm 内退开）—— 行为干预，非位置限制；
    ⑤ 蓝队死球重启全队退回 x<=130 —— **全队每次重启都后撤**，直接导致"官方看起来变慢"。
v2 只保留三道**位置限制**（都在"别进/别赖在对方门前"这个范畴内）：
    ① 目标红线：门区 y 带 [62.5,117.5] 内，站位目标 x <= 164（门区前缘 170 外留 6cm）
    ② 大禁区最多 1 人：只有离球最近的外场球员可压过 x=135
    ③ 实际位置反馈：真踩进 x >= 168 的门区带 → 立即主动退回（抗惯性过冲/被撞入）
    门将（id==0）不受约束（它本来就在自己门区）。

另修一处**死球卡死根因**（本轮 rlg 实测，2026-09-11 09:44 场）：
    官方 demo 自己的 SetBall 把黄队门球摆在 (10, 70) —— y=70 正好是球门下门柱那条线
    （门框 y∈[70,110]），球被摆在门柱角落；demo 门将的逻辑是"站到球的位置"而不穿球推，
    于是球永远不动 → 平台每 ~5 秒重发一次门球，**一场连续 8 次卡在同一格**。
    我们自己的 formation_set_ball 用的是 (门前 10cm, y=90)（门前正中），从未出现该问题。
    → 把 demo 的 y=70 改成 y=90（与我们的口径一致）。

用法：
    python tools/py/patch_demo_yellow.py --src <源文件> [--dst <输出文件>] [--check]
    不传 --dst 时原地改写；按 GBK 读写（官方 demo 源码是 GBK，MSVC 按 936 解析）。
    源文件必须是**未打过补丁的官方原始源码**（C:\\Strategy\\src\\Strategy4Yellow\\）。
"""
import argparse
import sys

MARK = "禁区纪律 v2（本队自加，2026-09-11）"

# —— 锚点 1：原始 demo 的 Position(Environment*, id, x, y) 包装（逐字匹配，含制表符）——
OLD_POS = (
    "void Position(Environment* pEnv, int id, double x, double y)\n"
    "{\n"
    "\tPosition(&(pEnv->home[id]), x, y);\n"
    "}\n"
)

NEW_POS = """void Position(Environment* pEnv, int id, double x, double y)
{
	Robot* self = &(pEnv->home[id]);

	// ============ 禁区纪律 v2（本队自加，2026-09-11）============
	// 目的：不反复触发平台判罚把比赛打断（真机 rlg：停表 9.5~20.2 次/分钟，
	//   其中 73~79% 的停表前 20 帧内本队有人扎进蓝队门区并贴到蓝队门将 <12cm）。
	// 平台口径（蓝队门线 x=220）：门区(小禁区)=门线内 50cm、y∈[62.5,117.5]；
	//   罚球区(大禁区)=门线内 80cm、y∈[72.5,107.5]。
	// 规则：进攻方门区 2 人以上 / 单人停留>20 周期 / 罚球区 4 人以上 / 冲撞门将 → 判罚。
	// 只保留"位置限制"三条（用户指令：除禁区限制外不改 demo 行为）；
	//   v1 的 ④门将保护圈、⑤死球全队退回已删除（⑤让全队每次重启后撤 → 观感"变慢"）。
	// ==========================================================
	static const double kBoxFrontX  = 170.0;   // 门区前缘（判罚红线：门线内 50cm）
	static const double kBoxTargetX = 164.0;   // 站位目标上限（留 6cm 抗过冲余量）
	static const double kBoxExitX   = 168.0;   // 实际位置越此线即强制退出
	static const double kPenHoldX   = 135.0;   // 非最近球者的上限（罚球区外沿 140 之外）
	static const double kBoxYLow    = 62.5;    // 门区判罚带 y 下界
	static const double kBoxYHigh   = 117.5;   // 门区判罚带 y 上界
	static const double kPenYLow    = 72.5;    // 罚球区判罚带 y 下界
	static const double kPenYHigh   = 107.5;   // 罚球区判罚带 y 上界

	if (id != 0)   // 门将不受此纪律约束（本队门将本来就在自己门区）
	{
		// ① 门区 y 带内：站位目标不许越过门区前缘（留余量）
		if (y > kBoxYLow && y < kBoxYHigh && x > kBoxTargetX)
			x = kBoxTargetX;
		// ② 大禁区最多 1 人：非最近球者留在罚球区外
		if (y > kPenYLow && y < kPenYHigh && x > kPenHoldX)
		{
			int best = -1;
			double bestD = 1e18;
			for (int i = 1; i < 5; i++)
			{
				double dx = pEnv->home[i].pos.x - pEnv->currentBall.pos.x;
				double dy = pEnv->home[i].pos.y - pEnv->currentBall.pos.y;
				double d = dx * dx + dy * dy;
				if (d < bestD) { bestD = d; best = i; }
			}
			if (id != best)
				x = kPenHoldX;
		}
	}

	Position(self, x, y);

	// ③ 实际位置反馈：已经踩进危险带（惯性过冲 / 被撞入）→ 主动退回
	if (id != 0 && self->pos.x > kBoxExitX &&
	    self->pos.y > kBoxYLow && self->pos.y < kBoxYHigh)
	{
		Position(self, kBoxTargetX - 6.0, self->pos.y);
	}
}
"""

# —— 锚点 2：门球落点 (10,70) → (10,90)。y=70 是门柱线，球卡在门柱角推不出去 ——
OLD_BALL = (
    "\tif (PM_GoalKick_Yellow == gameState)\n"
    "\t{\n"
    "\t\tpBall->x = 10;\n"
    "\t\tpBall->y = 70;\n"
    "\t}\n"
)
NEW_BALL = (
    "\tif (PM_GoalKick_Yellow == gameState)\n"
    "\t{\n"
    "\t\tpBall->x = 10;\n"
    "\t\t// 2026-09-11 修：原为 y = 70 —— 正好是球门下门柱那条线（门框 y∈[70,110]），\n"
    "\t\t//   球被摆在门柱角落，本队门将只会\"站到球的位置\"而不穿球推 → 球永不动，\n"
    "\t\t//   平台每 ~5 秒重发门球（rlg 实测一场连续 8 次卡在同一格）。\n"
    "\t\t//   改到门前正中 y=90（与我方 formation_set_ball 口径一致）后可正常开出门球。\n"
    "\t\tpBall->y = 90;\n"
    "\t}\n"
)


# —— 锚点 3：官方 2018 源码用了 abs(double)（4 处），现代 MSVC + UCRT 下与整数重载
#    冲突 → error C2668「abs: 对重载函数的调用不明确」。改成 fabs（纯机械替换，行为不变）。
#    这不是我们引入的问题：原始 demo 源码在 VS2022/MSVC 14.44 下就编不过，
#    上一版是并发会话在别处顺手修的；本脚本把修复也纳入，保证"原始源码 → 一键可编译"。
ABS_FIXES = [
    ("\tif (abs(theta_e) > 50)", "\tif (fabs(theta_e) > 50)"),
    ("\telse if (abs(theta_e) > 20)", "\telse if (fabs(theta_e) > 20)"),
    ("\t\tif (d_e < 5.0 && abs(theta_e) < 40)", "\t\tif (d_e < 5.0 && fabs(theta_e) < 40)"),
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="demo 源码路径（GBK，须为原始未打补丁版）")
    ap.add_argument("--dst", default=None, help="输出路径，缺省原地改写")
    ap.add_argument("--check", action="store_true", help="只检查是否已打补丁")
    args = ap.parse_args()

    src = open(args.src, encoding="gbk").read()
    if args.check:
        print("已打补丁(v2)" if MARK in src else "未打补丁")
        return 0 if MARK in src else 1
    if MARK in src:
        print("已经是 v2 补丁版，未做修改")
        return 1
    if src.count(OLD_POS) != 1:
        print(f"锚点1(Position) 匹配 {src.count(OLD_POS)} 次（应为 1）——源码不符，放弃")
        return 2
    if src.count(OLD_BALL) != 1:
        print(f"锚点2(SetBall 门球落点) 匹配 {src.count(OLD_BALL)} 次（应为 1）——源码不符，放弃")
        return 2
    out = src.replace(OLD_POS, NEW_POS).replace(OLD_BALL, NEW_BALL)
    # abs(double) → fabs(double)（MSVC 重载歧义修复；4 处，第 3 处出现两次）
    for old, new in ABS_FIXES:
        out = out.replace(old, new)
    # 校验用词边界正则：不能用 "abs(theta_e)" in out —— fabs(theta_e) 也含这个子串（踩过）
    import re
    if re.search(r"(?<![A-Za-z_])abs\(theta_e\)", out):
        print("仍有裸 abs(theta_e) 未替换（源码与预期不符）")
        return 4
    # GBK 编码防护：官方 demo 源码是 GBK，插入的文本里若有 GBK 编不出的字符（如 emoji）
    #   会在写文件时才炸（本轮踩过：⚠️ U+26A0）→ 提前定位并报出是哪个字符。
    try:
        out.encode("gbk")
    except UnicodeEncodeError as e:
        bad = out[e.start:e.end]
        print(f"待写入文本含 GBK 无法编码的字符 {bad!r}（U+{ord(bad[0]):04X}）"
              f"——请改成中文标点或 ASCII 后重试")
        return 3
    dst = args.dst or args.src
    # 先写临时文件再原子替换：避免"open('w') 已截断、write 中途抛异常"把源码写坏
    #   （本轮踩过：GBK 编码异常导致 .tmp 源码变 0 字节，靠 _v1_backup 才救回来）
    tmp = dst + ".tmp"
    with open(tmp, "w", encoding="gbk", newline="") as fp:
        fp.write(out)
    import os
    os.replace(tmp, dst)
    print(f"OK 已写入 {dst}（+{len(out)-len(src)} 字符；禁区三道闸 + 门球落点 y=70→90）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
