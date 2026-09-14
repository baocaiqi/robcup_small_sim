# -*- coding: utf-8 -*-
"""生成《Hnnu 队伍策略介绍》PDF（面向比赛方的策略说明，不含测试/工程内容）。

做法：程序化生成排版好的 HTML（截图以 base64 内嵌，单文件自包含），
再用 Edge/Chrome 无头打印成 PDF。不依赖任何第三方 Python 库。

用法：python tools/py/make_strategy_pdf.py
产物：<工作区根目录>/Hnnu策略介绍.pdf
"""
import base64
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
OUT_DIR = ROOT          # 用户要求：产物直接放工作区根目录
HTML = os.path.join(OUT_DIR, "Hnnu策略介绍.html")
PDF = os.path.join(OUT_DIR, "Hnnu策略介绍.pdf")

# 需要的截图/图（找不到就优雅跳过）
FIGS = [
    ("fig_field", r"build\penalty_shots\review\153147_e01_00_field.png",
     "图 1 · 平台球场窗口：左侧是 220×180 厘米球场与双方各 5 台机器人、球；"
     "右侧是策略菜单、START、TIME/SCORE 与 HELP（HELP 内为平台自带规则原文）。"),
    ("fig_dialog", r"build\penalty_shots\165855_e15_00_win.png",
     "图 2 · 平台主对话框：显示剩余时间与比分，比分格式为（黄队 : 蓝队）。"),
    ("fig_form", r"docs\work\formation12.png",
     "图 3 · 12 种比赛状态的摆位示意（按蓝队坐标绘制，黄队镜像）："
     "开球 / 争球 / 点球 / 任意球 / 门球，以及每种状态里先摆方与后摆方的差别。"),
]

CSS = """
@page { size: A4; margin: 16mm 14mm; }
* { box-sizing: border-box; }
body { font-family: "Microsoft YaHei", "PingFang SC", sans-serif; font-size: 10.5pt;
       line-height: 1.6; color: #1a1a1a; margin: 0; }
h1 { font-size: 21pt; margin: 0 0 4pt 0; }
h2 { font-size: 13.5pt; margin: 16pt 0 6pt 0; padding: 4pt 8pt; background: #eef3fb;
     border-left: 4px solid #2b6cb0; page-break-after: avoid; }
h3 { font-size: 11.5pt; margin: 11pt 0 3pt 0; color: #2b4a7d; page-break-after: avoid; }
p, li { margin: 3pt 0; }
.sub { color: #666; font-size: 9.5pt; margin-bottom: 10pt; }
table { border-collapse: collapse; width: 100%; margin: 5pt 0; font-size: 9.5pt; }
th, td { border: 1px solid #cbd5e0; padding: 3.5pt 5pt; text-align: left; vertical-align: top; }
th { background: #f2f6fc; }
code { font-family: Consolas, "Courier New", monospace; font-size: 9.5pt;
       background: #f5f7fa; padding: 0 2pt; }
pre { background: #f5f7fa; border-left: 3px solid #90b4d8; padding: 5pt 7pt; margin: 5pt 0;
      font-family: Consolas, "Courier New", monospace; font-size: 8.6pt; white-space: pre;
      line-height: 1.45; page-break-inside: avoid; }
figure { margin: 8pt 0; page-break-inside: avoid; text-align: center; }
figure img { max-width: 100%; max-height: 100mm; border: 1px solid #cbd5e0; }
figcaption { font-size: 9pt; color: #555; margin-top: 3pt; text-align: left; }
.box { background: #f7fafc; border: 1px solid #cbd5e0; padding: 6pt 8pt; margin: 6pt 0; }
ul { margin: 3pt 0 3pt 16pt; padding: 0; }
.small { font-size: 9pt; color: #555; }
.pb { page-break-before: always; }
"""

TREE = """strategy_5v5/
├─ include/simuro5/              —— 数据结构与各模块接口
│   ├─ simuro_interface.hpp      平台接口：5 个回调函数 + Environment 结构 + PlayMode 枚举
│   ├─ team.hpp                  队伍上下文：本方是蓝/黄、进攻方向、双方门线位置
│   ├─ geometry.hpp / field_info.hpp  几何工具、场地与门区/罚球区定义
│   ├─ world_model.hpp           世界模型：球与双方机器人状态、球权、攻防状态、计数器
│   ├─ situation.hpp             局势：球权、威胁等级、三个站位参考点
│   ├─ role_assignment.hpp       角色分配
│   ├─ roles.hpp                 五个角色的决策入口
│   ├─ shoot.hpp / pass.hpp      射门决策 / 传球决策的结果结构
│   ├─ defense.hpp               防守工具箱：断球点、盯人打分、二抢一、抢反弹位
│   ├─ motion.hpp / route.hpp    差速轮运动控制 / 避障路径规划
│   └─ formation.hpp             12 种比赛状态的摆位
├─ src/                          —— 策略实现
│   ├─ dll_blue.cpp              蓝队导出壳（平台按此文件名加载）
│   ├─ dll_yellow.cpp            黄队导出壳（同一份策略代码，仅队伍身份不同）
│   ├─ world_model.cpp           把平台数据转成内部状态（含球速跳变滤波）
│   ├─ situation.cpp             计算球权、威胁等级与站位参考点
│   ├─ strategy.cpp              每帧主循环：局势 → 状态机 → 调度五个角色
│   ├─ role_assignment.cpp       五个角色的确定
│   ├─ roles.cpp                 五个角色的决策（门将/主攻/助攻/中场/后卫）
│   ├─ shoot.cpp                 射门几何：净开口角、瞄准点、机会质量
│   ├─ pass.cpp                  传球点选择
│   ├─ defense.cpp               防守工具箱实现
│   ├─ motion.cpp                差速轮控制：速度律、到点定向、路径跟随
│   ├─ route.cpp                 避障路径：网格 A* + 视线拉直
│   ├─ formation.cpp             12 态摆位与摆球
│   ├─ geometry.cpp              基础几何计算
│   └─ offline_test.cpp          离线回归测试（开发用，不参与比赛运行）
└─ tools/                        —— 开发与复盘工具（不参与比赛运行）
    ├─ sim_bench/                无头仿真器（复用同一份策略源码）
    ├─ py/                       录像与日志分析脚本
    └─ sparring_yellow/          陪练黄队（官方 demo 源码 + 纪律补丁，官方源码不入库）
"""

BODY = """
<h1>Hnnu 队伍策略介绍</h1>
<div class="sub">FIRA SimuroSot 5v5 仿真组 · 2026-09-14</div>

<div class="box">
<b>总体思路：</b>本平台没有"踢球"这一动作，所有让球动起来的行为都是<b>用身体推球</b>，
因此机器人的<b>朝向与到位精度</b>直接决定出球方向与速度。我们的策略围绕这一点组织：
<br>· <b>进攻</b>：先判断"能不能射"（射程 + 扣除门将遮挡后的净开口），能射就推穿，射不了就传，被围就脱身；
<br>· <b>防守</b>：门将按球的轨迹预测封堵，其余四人分层——区域断球点、人盯人、二抢一、抢二次落点；
<br>· <b>定位球</b>：12 种状态各自摆位，罚球时先"认出"是我方主罚，再就地转正立刻推；
<br>· <b>纪律</b>：平台对门区聚集、角区推球、死球期推球判罚极重，因此这些作为策略中的硬约束。
</div>

<h2>一、平台与界面</h2>
__FIG_FIELD__
__FIG_DIALOG__
<p>比赛以每秒 40 帧推进（每帧 25 毫秒）：每帧平台把全场状态交给我们的策略，
策略回一次"本方 5 台机器人左右轮各转多快"，平台据此推进下一帧。场地 220×180 厘米，
球门宽 40 厘米（球门线 y ∈ [70,110]），蓝队守右门（x=220），黄队守左门（x=0）。</p>

<h2 class="pb">二、完整代码结构</h2>
<pre>__TREE__</pre>
<table>
<tr><th>模块</th><th>职责（策略层面）</th></tr>
<tr><td>world_model / situation</td><td>把平台数据转成内部状态；计算球权、威胁等级与站位参考点</td></tr>
<tr><td>strategy / role_assignment</td><td>每帧的调度主循环与五个角色的确定</td></tr>
<tr><td>roles</td><td>五个角色的决策：门将、主攻、助攻、中场、后卫</td></tr>
<tr><td>shoot / pass</td><td>射门几何与传球点选择</td></tr>
<tr><td>defense</td><td>防守工具箱：断球点、盯人、二抢一、抢反弹位</td></tr>
<tr><td>motion / route</td><td>差速轮速度控制与避障路径</td></tr>
<tr><td>formation</td><td>12 种比赛状态的摆位与摆球</td></tr>
</table>
<p class="small">蓝、黄两个 DLL 由同一份策略源码编译（仅"队伍身份"一个参数不同），因此两侧策略完全一致。</p>

<h2>三、每帧的策略流程</h2>
<table>
<tr><th>步骤</th><th>内容</th></tr>
<tr><td>① 读世界</td><td>读入球与双方 10 台机器人的位置/朝向（对方速度由相邻两帧差分得到），并对球的"瞬移"做滤波</td></tr>
<tr><td>② 判局势</td><td>计算球权与威胁等级，翻转攻防状态（带防抖），更新三个站位参考点</td></tr>
<tr><td>③ 派活</td><td>0 号门将、1 号主攻、2 号助攻、3 号中场、4 号后卫各自决策</td></tr>
<tr><td>④ 输出</td><td>写出 5 台机器人的左右轮速，由平台推进比赛</td></tr>
</table>
<p><b>角色固定</b>：不做每帧抢球权式的动态改写，而是让每个角色长期负责一类任务，
这样站位稳定，不会出现"两人同时追球、门前无人"的局面。</p>

<h2 class="pb">四、进攻策略</h2>

<h3>4.1 射门决策：先算"门将挡住了多少"，再决定瞄哪</h3>
<p>把球门看成一个扇面：球到两侧门柱的夹角就是<b>门张角</b>；门将按其站位在球门前形成一个遮挡扇面。
两者之差就是<b>净开口</b>，我们瞄准净开口中最宽一段的正中。</p>
<pre>门张角   = |球→左门柱 方位角 − 球→右门柱 方位角| / 2
门将遮挡 = 门将有效半径 8cm 在该距离上对应的张角
净开口   = 门张角扣除遮挡后，剩余的最大连续空隙 → 取其中心作为瞄准方向</pre>
<p><b>射程分三档：</b></p>
<ul>
<li><b>≤70 厘米</b>：无条件射（本平台最容易进球的距离区间）；</li>
<li><b>70～110 厘米</b>：要求净开口 ≥8°、球前 30 厘米内无其他防守者，且机会质量 ≥0.35；</li>
<li><b>&gt;110 厘米</b>：不射，转为带球推进或传球；球贴门线不足 5 厘米时也不射（没有推球空间）。</li>
</ul>
<p><b>机会质量</b>（0～1，用于远射档是否放行）：</p>
<pre>质量 = 0.5 × min(净开口 / 18°, 1) + 0.3 × min((110 − 球到门距离) / 110, 1) + 0.2 × min(球速 / 8厘米每帧, 1)</pre>
<p>三项分别代表"门有多大空档、离门多近、球是否在运动"。<b>执行</b>方式：由于没有踢球动作，
射门是一次"从球后方向前直线推穿"——先在瞄准线上取球后 20 厘米处作为出发位，
机头对准瞄准方向（容差 10°）后直接推穿，使球出射方向等于撞球瞬间的机头方向。</p>

<h3>4.2 传球与带球推进</h3>
<p>射门不可行时，主攻手先评估传球：为每名队友计算接球点，按<b>威胁加权</b>打分
（接球点附近对方球员的距离、是否被贴住、接球后朝球门的推进空间），
取分数最高且传球线路无阻挡者传出。若无合适传球目标（被围困：球周围 25 厘米内有 2 名以上对手，
或最近对手不足 12 厘米），则朝空档方向带球脱离，避免在原地反复推球送出判罚。</p>

<h3>4.3 无球跑位：两个接应点 + 目标滞回</h3>
<p>助攻与中场不追球，而是各自守一个参考点：</p>
<ul>
<li><b>助攻</b>：进攻时位于球前方 40 厘米、偏上 40 厘米；球进入对方罚球区时退回禁区外沿，作为近端出球点；</li>
<li><b>中场</b>：位于中线附近、球后方 40 厘米，兼顾纵深与回防；</li>
<li>两者都<b>不进入对方门区</b>（门区聚集会被判点球）。</li>
</ul>
<p>参考点只在"新目标距当前锚点 ≥15 厘米"时才更新。原因是差速轮不能横移，改方向必须先转身：
目标若每帧微动，机器人会不断微转、走成 S 形且始终到不了位；把目标量化成 15 厘米的台阶后，
机器人以长直线段奔跑，到位后停住等待。（防守方的参考点不设滞回，以便即时跟随球。）</p>

<h3>4.4 反击加速</h3>
<p>丢球后全队按威胁等级回收；一旦重新控球，攻防状态机的防抖计数（连续 3 帧）会让转换慢半拍。
因此在重新控球瞬间开启一个 30 帧（约 0.75 秒）的<b>反击窗口</b>，窗口内助攻与中场豁免回防、立刻前插接应，
使主攻手断球后立即有出球点；窗口结束即恢复正常回防，不会长期留下后防空档。</p>

<h2 class="pb">五、防守策略</h2>

<h3>5.1 门将</h3>
<table>
<tr><th>机制</th><th>做法</th></tr>
<tr><td>轨迹预测</td><td>按球速外推到本方门线，得到"球会从哪个 y 进门"；球贴边墙滚动时按<b>撞墙反射</b>后的轨迹计算落点。</td></tr>
<tr><td>出击深度</td><td>按威胁动态调整：球慢则贴门线；球快则前压到罚球区前缘封角度；对方有人埋伏在罚球区时逐人回缩，防回敲。</td></tr>
<tr><td>近距扑救</td><td>球会进门、到达时间足够短且门将够得着时，扑向门线内侧的预测落点；预测点超出可及范围时改为封住近门柱一侧。</td></tr>
<tr><td>门线封堵</td><td>球的飞行轨迹已在门框范围内时，直接抢占门线上的预测落点（优先于其他站位分支）。</td></tr>
<tr><td>清球</td><td>球在门区附近静止时主动推出：机头未对准则先原地转正，再直线推穿，避免"只蹭不推"。</td></tr>
<tr><td>防乌龙</td><td>球比门将更靠门时（门将处于球的外侧），先横向让开而不朝球推进，避免用身体把球顶入自家球门。</td></tr>
</table>

<h3>5.2 区域防守与断球点</h3>
<p>后卫（本队第五人）守"球—本方球门连线、距门约 50 厘米"的位置；当球朝本方球门滚动时，
改为站在球的实际运动轨迹与本方门前 50 厘米拦截线的交点（断球点），
球朝边墙滚时用反射后的轨迹计算；若自己赶不上该交点，则退守门前中央，不盲目扑球。</p>

<h3>5.3 人盯人</h3>
<p>威胁等级较高时，后卫改为盯人。为对方 5 人各算一个威胁分，取最高分者盯防：</p>
<pre>威胁分 = 50/(到球距离+10) + 25/(到本方门距离+10) + 0.4×(球朝他的速度)×能否够到 + 0.3×(他持球时的朝门速度)</pre>
<ul>
<li>四项分别代表：离球近、靠门近、球正传给他（接球威胁）、他正带球压上（突破威胁）；</li>
<li>站位取"被盯者 ↔ 本方球门"连线上、距被盯者 16 厘米处，并按被盯者速度外推 3 帧（速度前馈），以拦截其前进方向；</li>
<li>若被盯者正在接球（离球 15～40 厘米），则改为站在"球→被盯者"连线上，直接掐断传球线路；</li>
<li>为避免目标来回切换导致原地转圈，换人需要新目标分数超过当前目标 5%；</li>
<li>被盯者离球与离门都超过 40 厘米时判定为"不危险"，放弃盯人回到区域站位。</li>
</ul>

<h3>5.4 二抢一（双人夹击）</h3>
<p>对方持球者带球推进到本方门前 100 厘米内时，从助攻/中场中派<b>离持球者最近的一人</b>上前夹抢
（另一人留在区域保持纵深，避免被一脚直塞打穿）。夹抢点取在持球者前进方向前方 16 厘米处、
并向其所在一侧横向错开 20 厘米，用于封住他向外侧变线的角度，把他赶往中路。
若夹抢点落入门区（守门员专属区域）则退到门区前缘外；本方罚球区内允许进入协防。</p>

<h3>5.5 抢反弹位（二次落点）</h3>
<p>对方射门命中门框范围时，助攻与中场分别前压到罚球区前缘、在预测入球点的上侧与下侧各 30 厘米处，
准备争抢门将扑出或挡出的二次球。仅在球朝本方球门的来速 &gt;8 厘米/帧（判断为射门）时才启动，
避免为慢速带球提前站过去而留出空档。落点按"球从门将哪一侧来就往哪一侧弹"的经验修正，
若该点已被对方补射者占据则向空档侧让开 20 厘米。</p>

<h2 class="pb">六、定位球策略</h2>

<h3>6.1 12 种状态与摆位</h3>
<p>平台把比赛分为 12 种状态：正常比赛、争球（四个分区）、开球、点球、任意球、门球，
每种状态都用"黄/蓝"标明<b>由哪一方主罚</b>。状态开始时平台要求双方先摆好机器人，摆位坐标由我们给出：</p>
__FIG_FORM__
<ul>
<li>开球、任意球、门球：<b>主罚方先摆</b>；点球：<b>防守方先摆</b>；</li>
<li>主罚方的踢球人站在球后方，防守方守门员靠近本方门线、其余人留在本方半场；</li>
<li>摆位坐标按蓝队书写，黄队用镜像公式换算，因此两侧策略完全一致；</li>
<li>门球由我方摆球时，球放在本方门前 10 厘米正中，避免摆在门柱角落导致球无法被推出。</li>
</ul>

<h3>6.2 罚球（点球）</h3>
<p><b>识别</b>：平台在执行阶段不向策略报告"点球"这一状态，因此改用可观测的事实判断：
<b>球静止地停在对方罚球点上</b>（实测该点为门前 39.4 厘米、正中央）。</p>
<p><b>执行</b>：识别为我方主罚后，主罚人就地转正到瞄准方向后<b>立刻推球</b>，
不做长距离后退助跑——复盘显示，后退助跑会让对手有时间逼近并把球断走。
瞄准方向取"门将遮挡之外的空隙"，并在左右两侧之间随机选择，避免被对方门将预判。
<b>防守</b>我方被罚点球时，门将按门线封堵机制站在预测落点上，并按对方主罚人的朝向预估其射门方向。</p>

<h3>6.3 门球与死球</h3>
<p>球在门前静止（门球等）时，门将主动推出而非等待；对手尚未开出时，主攻手站在对方门区外沿等待，
避免因冲入对方门区被判罚；球长时间无人处理（约 2.5 秒）时我们主动处理，防止比赛进入僵局被判争球。</p>

<h2>七、全局调度与纪律约束</h2>

<h3>7.1 球速滤波</h3>
<p>平台在进球、摆位、球出界时会把球瞬移（实测单帧位移可达 117 厘米），
直接差分会得到物理上不可能的速度并误导后续判断，因此单帧位移超过 30 厘米时判定为"复位"，
该帧速度记为零。</p>

<h3>7.2 球权、威胁与攻防状态</h3>
<pre>球权 = 我方最近球员到球的距离 &lt; 对方最近球员到球的距离  且  我方最近球员 &lt; 20 厘米
威胁 = 球在己方罚球区且非我方球权 → 1.0；球在己方半场且非我方球权 → 0.6；我方球权 → 0.1；其它 → 0.3</pre>
<p>攻防状态经"连续 3 帧"防抖后翻转，避免单帧球权抖动导致全队左右横跳。</p>

<h3>7.3 纪律硬约束（策略的一部分）</h3>
<table>
<tr><th>平台判罚</th><th>我们的约束</th></tr>
<tr><td>对方门区内 2 人以上、或单人停留超过 20 周期 → 判对方点球</td><td>主攻手在对方门区内的纯停留计时超限即撤出或传球；其余角色的站位点一律不进入对方门区</td></tr>
<tr><td>在四角禁止推球区内推球 → 每 4 次判给对方 1 球</td><td>球处于角区时不主动推球，交由平台按规则处理；仅在推球合法的区域处理角区附近的球</td></tr>
<tr><td>死球（摆位）期间推球 → 判犯规</td><td>死球期间不接触球，只按规则站好位置；对手开出后恢复进攻</td></tr>
<tr><td>僵局：门区外僵持 100 周期无进展 → 判争球</td><td>球长时间静止无人处理时主动处理，避免双方僵持</td></tr>
</table>

<h2>八、主要算法清单</h2>
<table>
<tr><th>算法</th><th>用在哪里</th><th>要点</th></tr>
<tr><td>净开口角几何（解析解）</td><td>射门瞄准</td><td>门张角扣除门将遮挡张角，取剩余最大空隙中心，得到连续瞄准角</td></tr>
<tr><td>机会质量加权评分</td><td>远射是否放行</td><td>开口 0.5 / 距离 0.3 / 球速 0.2</td></tr>
<tr><td>时间最优制动包线</td><td>到点停（v = √(2as)）</td><td>双积分器时间最优控制的切换曲线，保证恰好停在目标点</td></tr>
<tr><td>均匀网格 A* + 视线拉直</td><td>避障移动</td><td>4 厘米网格、八邻域、八方向启发；拉直为 2～4 个折点</td></tr>
<tr><td>匀速外推 + 边墙反射</td><td>断球点、门将预测、抢反弹位</td><td>球撞 y=0/180 边墙时法向分量反号，折返后计算落点</td></tr>
<tr><td>威胁加权打分</td><td>人盯人目标选择</td><td>离球、离门、球来袭速度、持球突破四项加权</td></tr>
<tr><td>速度前馈截击</td><td>盯人站位</td><td>按被盯者速度外推 3 帧，站到其前进方向前方</td></tr>
<tr><td>差速轮速度耦合</td><td>所有移动</td><td>左右轮速差产生转向；朝向误差大时先转正再推进</td></tr>
</table>

<h2>九、策略参数速查</h2>
<table>
<tr><th>项目</th><th>取值</th></tr>
<tr><td>射程</td><td>≤70cm 无条件射；70～110cm 需净开口 ≥8° 且质量 ≥0.35；&gt;110cm 不射</td></tr>
<tr><td>瞄准容差 / 出发位</td><td>机头对准容差 10°；出发位取球后 20cm</td></tr>
<tr><td>到点速度</td><td>v = min(150, √(2×400×(剩余距离−1.5)))（厘米、厘米/秒）</td></tr>
<tr><td>路径网格</td><td>4 厘米 / 55×45 格；场边留 4 厘米</td></tr>
<tr><td>站位滞回</td><td>15 厘米（防守参考点不设）</td></tr>
<tr><td>盯人</td><td>贴身 16 厘米；外推 3 帧；换人滞回 5%；危险门限 40 厘米；堵传球线 15～40 厘米</td></tr>
<tr><td>二抢一</td><td>离门 100 厘米内启动；横向错开 20 厘米；球在罚球区内时需持球者离门 &lt;45 厘米</td></tr>
<tr><td>抢反弹位</td><td>罚球区前缘（门前 80 厘米）、入球点上下各 30 厘米；球朝门速 &gt;8 厘米/帧才抢</td></tr>
<tr><td>反击窗口</td><td>重新控球起 30 帧（约 0.75 秒）</td></tr>
<tr><td>罚球点</td><td>门前 39.4 厘米、正中央（实测）</td></tr>
</table>

<p class="small">本文件由 <code>tools/py/make_strategy_pdf.py</code> 生成；截图取自平台实际运行画面。</p>
"""


def img_tag(path, caption):
    full = os.path.join(ROOT, path)
    if not os.path.exists(full):
        return ""
    with open(full, "rb") as f:
        b64 = base64.b64encode(f.read()).decode("ascii")
    return (f'<figure><img src="data:image/png;base64,{b64}" />'
            f'<figcaption>{caption}</figcaption></figure>')


def main():
    html = BODY.replace("__TREE__", TREE)
    keys = [k for k, _, _ in FIGS]
    for k, p, c in FIGS:
        html = html.replace("__" + k.upper() + "__", img_tag(p, c))
    for k in keys:                       # 清掉没找到图的占位符
        html = html.replace("__" + k.upper() + "__", "")
    doc = ("<!DOCTYPE html><html><head><meta charset='utf-8'>"
           "<title>Hnnu 队伍策略介绍</title><style>" + CSS + "</style></head>"
           "<body>" + html + "</body></html>")
    with open(HTML, "w", encoding="utf-8") as f:
        f.write(doc)
    print(f"[OK] HTML: {HTML} ({os.path.getsize(HTML)//1024} KB)")

    browsers = [r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
                r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
                r"C:\Program Files\Google\Chrome\Application\chrome.exe",
                r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe"]
    exe = next((b for b in browsers if os.path.exists(b)), None)
    if not exe:
        print("[ERR] 没找到 Edge/Chrome，无法打印 PDF")
        return 1
    url = "file:///" + HTML.replace("\\", "/").replace(" ", "%20")
    subprocess.run([exe, "--headless=new", "--disable-gpu", "--no-pdf-header-footer",
                    "--print-to-pdf=" + PDF, url],
                   capture_output=True, text=True, errors="replace", timeout=180)
    if os.path.exists(PDF):
        print(f"[OK] PDF : {PDF} ({os.path.getsize(PDF)//1024} KB)")
        return 0
    print("[ERR] 打印失败")
    return 1


if __name__ == "__main__":
    sys.exit(main())
