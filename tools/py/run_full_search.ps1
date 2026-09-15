# run_full_search.ps1 — 主搜索驱动（进攻组 → 防守组 → 组合验收）
#
# 用户 2026-09-16 批准：两组全跑，约 2 小时机器时间。
# 协议（见 docs/work/RL参数搜索规格.md + 轮次 74 的教训）：
#   · 每任务 20 局（原 10 局噪声太大，3 种子 × 10 局分不清"真提升"和"运气"）
#   · 训练集种子 1/2/3；留出集 A=1001-1003、B=2001-2003（两套独立集，B 是新加的）
#   · 起点用已验证的 best_params_pilot.txt（两独立集复现 +1.23/+1.32 球的那个），
#     不是从默认值从零搜 —— 已经证实有效的方向不重复浪费算力
#   · 纪律罚 0.25 软罚 + 3 倍宽松硬限（0.35+1.5 倍硬判死的教训：会误杀有效解）
#
# 用法：pwsh -File tools\py\run_full_search.ps1
$ErrorActionPreference = "Continue"
$env:PYTHONIOENCODING = "utf-8"
$T = "tools\py\tune_es.py"
$W = "docs\work"
$PY = "python"

function Step($title, $cmd) {
    Write-Output ""
    Write-Output ("=" * 70)
    Write-Output "### $title"
    Write-Output ("=" * 70)
    $t0 = Get-Date
    Invoke-Expression $cmd
    if ($LASTEXITCODE -ne 0) {
        Write-Output "!!! 步骤失败(exit $LASTEXITCODE)，后续步骤可能不可靠：$title"
    }
    Write-Output ("--- 用时 {0:N1} 分钟" -f ((Get-Date) - $t0).TotalMinutes)
}

# ---- 1) 重算基线：必须与候选用同样的 --games 20（否则不是严格配对比较）----
Step "基线-训练集(种子1,2,3, 20局/任务)" "$PY $T baseline --group both --games 20 --seeds 1 2 3 --workers 8 --baseline $W\tune_base_train20.json"
Step "基线-留出集A(1001-1003, 20局)"      "$PY $T baseline --group both --games 20 --seeds 1001 1002 1003 --workers 8 --baseline $W\tune_base_A20.json"
Step "基线-留出集B(2001-2003, 20局)"      "$PY $T baseline --group both --games 20 --seeds 2001 2002 2003 --workers 8 --baseline $W\tune_base_B20.json"

# ---- 2) 进攻组搜索（20 个参数）----
Step "搜索-进攻组(20参数, 16种群, 15代)" "$PY $T search --group attack --games 20 --seeds 1 2 3 --pop 16 --gens 15 --workers 8 --init $W\best_params_pilot.txt --baseline $W\tune_base_train20.json --out $W\best_attack.txt"

# ---- 3) 防守组搜索（18 个参数，从进攻组结果出发）----
Step "搜索-防守组(18参数, 16种群, 15代)" "$PY $T search --group defense --games 20 --seeds 1 2 3 --pop 16 --gens 15 --workers 8 --init $W\best_attack.txt --baseline $W\tune_base_train20.json --out $W\best_defense.txt"

# ---- 4) 合并攻防参数，做三方验收（训练集 / 留出A / 留出B）----
Get-Content "$W\best_attack.txt", "$W\best_defense.txt" | Set-Content "$W\best_combined.txt"
Step "验收-攻击组@留出A" "$PY $T eval --group attack --games 20 --params $W\best_attack.txt --holdout --holdout-seeds 1001 1002 1003 --workers 8 --baseline $W\tune_base_A20.json"
Step "验收-攻击组@留出B" "$PY $T eval --group attack --games 20 --params $W\best_attack.txt --holdout --holdout-seeds 2001 2002 2003 --workers 8 --baseline $W\tune_base_B20.json"
Step "验收-组合@训练集"  "$PY $T eval --group both --games 20 --params $W\best_combined.txt --workers 8 --baseline $W\tune_base_train20.json"
Step "验收-组合@留出A"   "$PY $T eval --group both --games 20 --params $W\best_combined.txt --holdout --holdout-seeds 1001 1002 1003 --workers 8 --baseline $W\tune_base_A20.json"
Step "验收-组合@留出B"   "$PY $T eval --group both --games 20 --params $W\best_combined.txt --holdout --holdout-seeds 2001 2002 2003 --workers 8 --baseline $W\tune_base_B20.json"

Write-Output ""
Write-Output "=== 全部完成 ==="
Get-ChildItem "$W\best_*.txt" | ForEach-Object { "  $($_.Name)  $($_.LastWriteTime.ToString('HH:mm'))" }
