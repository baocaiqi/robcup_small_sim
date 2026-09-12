// test_discipline.cpp — 陪练黄队「禁区位置限制」单元测试（对齐 patch_demo_yellow.py v2）
//
// v2 只保留三条**位置限制**（用户指令：除禁区限制外不改 demo 行为）：
//   ① 门区 y 带 [62.5,117.5] 内，站位目标 x ≤ 164
//   ② 大禁区最多 1 人：只有离球最近的外场球员可压过 x=135
//   ③ 实际位置踩进 x ≥ 168 的门区带 → 立即主动退回
//   （v1 的 ④门将保护圈、⑤死球全队退回已删除 → 本测试不再覆盖那两条）
//
// 判据：机器人 rotation=0 时，等效前进速度 forward = (velocityLeft + velocityRight)/2；
//       forward < 0 = 后退（远离蓝队球门），forward > 0 = 前进（压向蓝队球门）。
#include "Strategy4Yellow.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static Environment env;
static int fails = 0, total = 0;

static void setup(long gs, double ballx, double bally)
{
	memset(&env, 0, sizeof(env));
	env.gameState = gs;
	env.currentBall.pos.x = ballx;
	env.currentBall.pos.y = bally;
	env.predictedBall.pos = env.currentBall.pos;
	env.home[0].pos.x = 5; env.home[0].pos.y = 90; env.home[0].rotation = 0;      // 黄队门将
	for (int i = 1; i < 5; i++) { env.home[i].pos.x = 60; env.home[i].pos.y = 90; env.home[i].rotation = 0; }
	env.opponent[0].pos.x = 214.8; env.opponent[0].pos.y = 89.9;                  // 蓝队门将
}

// 把 id 放到 (rx,ry)，按纪律出口下发给 (tx,ty)，返回等效前进速度
static double forward_speed(int id, double rx, double ry, double tx, double ty)
{
	env.home[id].pos.x = rx; env.home[id].pos.y = ry; env.home[id].rotation = 0;
	env.home[id].velocityLeft = 0; env.home[id].velocityRight = 0;
	Position(&env, id, tx, ty);
	return (env.home[id].velocityLeft + env.home[id].velocityRight) / 2.0;
}

static void check(const char *name, double vx, bool want_backward)
{
	total++;
	bool ok = want_backward ? (vx < -1.0) : (vx > 1.0);
	printf("[%s] %-46s forward=%+7.2f  期望=%s\n", ok ? "PASS" : "FAIL", name, vx,
	       want_backward ? "后退(<0)" : "前进(>0)");
	if (!ok) fails++;
}

int main()
{
	// ① 目标红线：球在蓝队门前 (205,90)，机器人已在门区带内 x=167 → 必须停在 164 外
	setup(PM_PlayOn, 205, 90);
	check("① 门区带内 x=167 不许再压（目标裁到 164）", forward_speed(1, 167, 90, 205, 90), true);
	// ① 对照：门区带外（y=40）同一目标 → 照常追球
	setup(PM_PlayOn, 205, 40);
	check("① 对照 y=40（带外）照常追", forward_speed(1, 167, 40, 205, 40), false);

	// ③ 实际位置反馈：已经过冲到 x=175（踩进门区）→ 主动退回
	setup(PM_PlayOn, 205, 90);
	check("③ 过冲到 x=175 → 主动退回", forward_speed(1, 175, 90, 205, 90), true);

	// ② 大禁区最多 1 人：最近球者可压进大禁区
	setup(PM_PlayOn, 205, 90);
	env.home[1].pos.x = 150; env.home[1].pos.y = 90;
	check("② 最近球者(id1) 可压进大禁区", forward_speed(1, 150, 90, 205, 90), false);
	// ② 非最近球者被挡在罚球区外（135）
	setup(PM_PlayOn, 205, 90);
	env.home[1].pos.x = 150; env.home[1].pos.y = 90;   // 离球更近 → id1 是最近者
	check("② 非最近球者(id2) 挡在罚球区外", forward_speed(2, 150, 90, 205, 90), true);

	// 无回归：球在中场时纪律不干预
	setup(PM_PlayOn, 110, 90);
	check("无回归 中场追球不受影响", forward_speed(1, 60, 90, 110, 90), false);
	// 无回归：球压在蓝队门区里，黄队压到区外等球（160→164 仍前进）
	setup(PM_PlayOn, 215, 95);
	check("无回归 球进门区时压到区外等球", forward_speed(1, 160, 95, 215, 95), false);

	printf("\n%s（%d/%d 通过）\n", fails ? "❌ 有失败用例" : "✅ ALL TESTS PASSED", total - fails, total);
	return fails ? 1 : 0;
}
