# ============================================================
# relax_sweep.ps1 —— 「放松准度门槛换流畅度」档位扫描（2026-10-06，用户指令）
#
# 为什么这么做（结论先行）：
#   仓库里已经有 230 个可调参数 ⇒ "放松容差"可以**纯参数注入**，不用重编译、不用改源码，
#   所以能一次扫十几个档位、每档跑多个种子，把"流畅度涨了多少 / 代价是多少"量出来再定宽度。
#
# 量什么（三组指标，缺一不可）：
#   流畅度：主攻交接 次/秒（越小越稳）、[无主球追踪] 不朝球 %（越小越果断）、[犹豫追踪] 犹豫 %
#   效果  ：FIT 的 net/gf/ga/poss/shots
#   代价  ：乌龙 个/场（基线里乌龙 = 总失球的 89%，这是最先会被放松准度顶上去的指标）
#
# 仪表来自 build\Release\sim_trace.exe（带 SIMURO5_BRANCH_TRACE 的构建，会打印犹豫/无主球追踪）。
#
# ⚠️ 两个已踩过的坑（都已在脚本里防住）：
#   ① 参数文件带 BOM 会让**第一行被静默忽略**（实测 `未识别 1 个`）⇒ 一律用 -Encoding ascii 写。
#   ② `motion.kStopEps` / `motion.kBrakeAccel` 是 constexpr（motion.hpp:33/34）**不在可调表里**，
#      注入会被静默忽略 ⇒ 脚本开跑前拿 --dump-params 逐个对名字，不存在就报警。
#      （想扫它们必须先改成 TUNABLE 再重编译——那是另一件事，别混进来。）
#
# 用法： pwsh -NoProfile -File tools\sweep\relax_sweep.ps1 [-Games 40] [-Frames 2400] [-Seeds 3] [-Opp scripted]
# ============================================================
param(
    [int]$Games = 40,
    [int]$Frames = 2400,
    [int]$Seeds = 3,
    [string]$Opp = 'scripted'
)
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..')).Path
$exe  = Join-Path $root 'build\Release\sim_trace.exe'
if (-not (Test-Path $exe)) { throw "找不到 $exe（先编译 Release）" }
$tmp  = Join-Path $env:TEMP 'relax_sweep_params.txt'
$dump = Join-Path $env:TEMP 'relax_sweep_allparams.txt'
$outDir = Join-Path $root 'docs\work'
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$csv = Join-Path $outDir 'relax_sweep.csv'

# ---------------- 档位定义 ----------------
# 只挑"每帧主路径命中"且"注释里没写明已被证伪"的旋钮（依据：容差盘点 Top 10）。
# A=站位/对位容差  B=射门/传球准度门槛  C=进攻启动条件
$cfgs = [ordered]@{
    '基线(默认)'                    = @()
    'A1 近距油门帽 20→30'           = @('motion.kCreepMax 30')
    'A2 近距门限 12→18'             = @('motion.kNearDist 18')
    'A3 防守迎球 6/12→10/20'        = @('roles.kDefArriveDist 10', 'roles.kDefFaceAngTol 20')
    'A4 蜂群横偏 6.5→10,后撤11→8'   = @('roles.kHerdLatTol 10', 'roles.kHerdBack 8')
    'A5 接球提前量 6→4,到位8→12'    = @('roles.kRecvLead 4', 'roles.kRecvArriveDist 12')
    'A6 A1+A2+A3 合并'              = @('motion.kCreepMax 30', 'motion.kNearDist 18', 'roles.kDefArriveDist 10', 'roles.kDefFaceAngTol 20')
    'B1 远射开口 9.0→5.0'           = @('shoot.kMinOpen 5.0')
    'B2 传球阻挡阈值 7.4→4.0'       = @('pass.BLOCK_THRESHOLD 4.0')
    'B3 接球晚到容差 5→10帧'        = @('pass.RECEIVER_READY_TOLERANCE 10')
    'B4 射门朝向容差 13.2→25'       = @('roles.kPrepAngTol 25')
    'B5 B1+B2+B3 合并'              = @('shoot.kMinOpen 5.0', 'pass.BLOCK_THRESHOLD 4.0', 'pass.RECEIVER_READY_TOLERANCE 10')
    'C1 威胁降档滞回 10→5'          = @('strategy.kThreatHoldFrames 5')
    'C2 状态滞回 3→2'               = @('strategy.kStateHysteresisFrames 2')
    'C3 反击窗口 30→60'             = @('strategy.kCounterWindowFrames 60')
    'C4 主动截球 关→开'             = @('defense.kEarlyEnable 1')
    'C5 断球可达余量 1.04→1.20'     = @('defense.kReachMargin 1.20')
    'C6 C1+C2+C3+C4 合并'           = @('strategy.kThreatHoldFrames 5', 'strategy.kStateHysteresisFrames 2', 'strategy.kCounterWindowFrames 60', 'defense.kEarlyEnable 1')
    'AB 全合并(不含C4)'             = @('motion.kCreepMax 30', 'motion.kNearDist 18', 'roles.kDefArriveDist 10', 'roles.kDefFaceAngTol 20', 'roles.kHerdLatTol 10', 'roles.kHerdBack 8', 'roles.kRecvLead 4', 'roles.kRecvArriveDist 12', 'shoot.kMinOpen 5.0', 'pass.BLOCK_THRESHOLD 4.0', 'pass.RECEIVER_READY_TOLERANCE 10', 'roles.kPrepAngTol 25')
}

# ---------------- 预检：参数名必须真存在 ----------------
& $exe --dump-params $dump --games 1 --frames 50 --opp scripted 2>&1 | Out-Null
$valid = @{}
Get-Content $dump -Encoding utf8 | ForEach-Object { if ($_ -match '^(\S+)\s') { $valid[$Matches[1]] = $true } }
if ($valid.Count -lt 50) { throw "参数表只解析到 $($valid.Count) 个名字，dump 失败，先别扫" }
$badNames = @()
foreach ($name in $cfgs.Keys) {
    foreach ($ln in $cfgs[$name]) {
        $p = ($ln -split '\s+')[0]
        if (-not $valid.ContainsKey($p)) { $badNames += "$name → $p" }
    }
}
if ($badNames.Count -gt 0) {
    Write-Warning "以下档位含**不在可调表**的参数名（注入会被静默忽略，扫描作废）：`n  $($badNames -join "`n  ")"
    throw "先修档位定义再扫"
}
Write-Host "预检通过：$($valid.Count) 个可调参数，$($cfgs.Count) 个档位 × $Seeds 个种子 × $Games 场 = $($cfgs.Count * $Seeds * $Games) 场`n"

function Get-Num([string]$text, [string]$pattern, [int]$group = 1) {
    $m = [regex]::Match($text, $pattern)
    if (-not $m.Success) { return [double]::NaN }
    return [double]$m.Groups[$group].Value
}

$rows = @()
foreach ($name in $cfgs.Keys) {
    $lines = $cfgs[$name]
    if ($lines.Count -gt 0) { Set-Content -Path $tmp -Value $lines -Encoding ascii }   # ascii = 无 BOM，见文件头坑①
    elseif (Test-Path $tmp) { Remove-Item $tmp -Force }

    $acc = @{ net = 0.0; gf = 0.0; ga = 0.0; poss = 0.0; shots = 0.0; og = 0.0; swap = 0.0; loose = 0.0; hes = 0.0; bad = 0 }
    for ($s = 1; $s -le $Seeds; $s++) {
        $eargs = @('--games', $Games, '--frames', $Frames, '--opp', $Opp, '--seed', $s)
        if ($lines.Count -gt 0) { $eargs += @('--params', $tmp) }
        $txt = (& $exe @eargs 2>&1 | Out-String)
        if ($txt -match '未识别 (\d+) 个') { $acc.bad += [int]$Matches[1] }
        $acc.net   += Get-Num $txt 'FIT games=\d+ net=([-\d.]+)'
        $acc.gf    += Get-Num $txt 'FIT games=\d+ net=[-\d.]+ gf=([-\d.]+)'
        $acc.ga    += Get-Num $txt 'FIT games=\d+ net=[-\d.]+ gf=[-\d.]+ ga=([-\d.]+)'
        $acc.poss  += Get-Num $txt 'FIT games=\d+ net=[-\d.]+ gf=[-\d.]+ ga=[-\d.]+ poss=([-\d.]+)'
        $acc.shots += Get-Num $txt 'shots=([-\d.]+) gaf='
        $acc.og    += Get-Num $txt '乌龙\(我们\): 均 ([\d.]+) 个/场'
        $acc.swap  += Get-Num $txt '主攻交接\(我们\): 均 [\d.]+ 次/场, ([\d.]+) 次/秒'
        $acc.loose += Get-Num $txt '最近者不朝球 \d+（([\d.]+)%）'
        $acc.hes   += Get-Num $txt '\[犹豫追踪\] 争抢帧 \d+，犹豫 \d+（([\d.]+)%）'
    }
    $r = [pscustomobject]@{
        档位 = $name
        net = [math]::Round($acc.net / $Seeds, 3); gf = [math]::Round($acc.gf / $Seeds, 3)
        ga = [math]::Round($acc.ga / $Seeds, 3); og = [math]::Round($acc.og / $Seeds, 3)
        poss = [math]::Round($acc.poss / $Seeds, 2); shots = [math]::Round($acc.shots / $Seeds, 1)
        交接每秒 = [math]::Round($acc.swap / $Seeds, 3)
        不朝球pct = [math]::Round($acc.loose / $Seeds, 1)
        犹豫pct = [math]::Round($acc.hes / $Seeds, 1)
        未识别 = $acc.bad
    }
    $rows += $r
    Write-Host ("完成 {0,-30} net={1,7:N3} ga={2,5:N2} 乌龙={3,5:N2} 交接/s={4,6:N3} 不朝球={5,5:N1}% 犹豫={6,5:N1}%" -f `
        $r.档位, $r.net, $r.ga, $r.og, $r.交接每秒, $r.不朝球pct, $r.犹豫pct)
}

$base = $rows | Where-Object { $_.档位 -like '基线*' } | Select-Object -First 1
$rows | ForEach-Object {
    $_ | Add-Member -NotePropertyName Dnet -NotePropertyValue ([math]::Round($_.net - $base.net, 3)) -Force
    $_ | Add-Member -NotePropertyName Dog -NotePropertyValue ([math]::Round($_.og - $base.og, 3)) -Force
    $_ | Add-Member -NotePropertyName Dswap -NotePropertyValue ([math]::Round($_.交接每秒 - $base.交接每秒, 3)) -Force
    $_ | Add-Member -NotePropertyName Dloose -NotePropertyValue ([math]::Round($_.不朝球pct - $base.不朝球pct, 1)) -Force
}
$rows | Export-Csv -Path $csv -NoTypeInformation -Encoding utf8
Write-Host "`n=== 汇总（Δ 相对基线；流畅度看 Δ交接/Δ不朝球，越小越好；代价看 Δnet/Δ乌龙）==="
$rows | Format-Table 档位, net, Dnet, ga, og, Dog, poss, shots, 交接每秒, Dswap, 不朝球pct, Dloose, 犹豫pct, 未识别 -AutoSize
Write-Host "CSV: $csv"
