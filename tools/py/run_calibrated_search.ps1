# run_calibrated_search.ps1 — 在**已定标的仿真**上重跑攻防参数搜索（第二阶段）
#
# 为什么要有第二阶段：第一阶段的搜索跑在"球更黏、墙更弹、脚本对手慢一半"的旧仿真上，
# 结论不一定能迁移到真机。定标把仿真对齐到真机统计量之后，需要在新地基上重搜一遍。
#
# 与第一阶段（run_full_search.ps1）的区别：
#   · 用 build_calib 里**打了 sim.* 补丁**的 sim_bench（tune_es.py --binary 指定）
#   · 先把定标出的仿真参数写回源码并重新编译 → 仿真从此就是定标版，不用每次带参数文件
#   · 基线必须在新仿真上重算（物理变了，旧基线作废）
#
# 用法：& "$PWD\tools\py\run_calibrated_search.ps1"
$ErrorActionPreference = "Continue"
$env:PYTHONIOENCODING = "utf-8"
$T = "tools\py\tune_es.py"
$W = "docs\work"
$BIN = "build_calib\Release\sim_bench.exe"
$PY = "python"

function Step($title, $cmd) {
    Write-Output ""
    Write-Output ("=" * 70)
    Write-Output "### $title"
    Write-Output ("=" * 70)
    $t0 = Get-Date
    Invoke-Expression $cmd
    if ($LASTEXITCODE -ne 0) { Write-Output "!!! 步骤失败(exit $LASTEXITCODE)：$title" }
    Write-Output ("--- 用时 {0:N1} 分钟" -f ((Get-Date) - $t0).TotalMinutes)
}

# ---- 0) 把定标出的仿真参数写回源码，并重新编译（仿真从此是定标版）----
Step "写回定标参数到源码" "$PY tools\py\apply_params.py $W\sim_params_calibrated.txt --dry-run"
Step "写回定标参数（真写）" "$PY tools\py\apply_params.py $W\sim_params_calibrated.txt --yes"
Step "重新编译定标版仿真" "cmake --build `"$PWD\build_calib`" --config Release --target sim_bench"

# ---- 1) 新仿真上的回归体检：不注入参数时损失应≈定标值 ----
Step "定标后体检（损失应接近定标结果）" "$PY tools\py\calibrate_sim.py --eval-only --games 3 --binary $BIN"

# ---- 2) 新仿真上重算三套基线 ----
Step "基线-训练集(1,2,3)" "$PY $T baseline --group both --games 20 --seeds 1 2 3 --workers 8 --binary $BIN --baseline $W\cal_base_train20.json"
Step "基线-留出A(1001-1003)" "$PY $T baseline --group both --games 20 --seeds 1001 1002 1003 --workers 8 --binary $BIN --baseline $W\cal_base_A20.json"
Step "基线-留出B(2001-2003)" "$PY $T baseline --group both --games 20 --seeds 2001 2002 2003 --workers 8 --binary $BIN --baseline $W\cal_base_B20.json"

# ---- 3) 定标后重搜：进攻组 → 防守组（起点用第一阶段验证过的 pilot 参数，让它自己决定要不要保留）----
Step "搜索-进攻组(定标版)" "$PY $T search --group attack --games 20 --seeds 1 2 3 --pop 16 --gens 15 --workers 8 --binary $BIN --init $W\best_params_pilot.txt --baseline $W\cal_base_train20.json --out $W\cal_best_attack.txt"
Step "搜索-防守组(定标版)" "$PY $T search --group defense --games 20 --seeds 1 2 3 --pop 16 --gens 15 --workers 8 --binary $BIN --init $W\cal_best_attack.txt --baseline $W\cal_base_train20.json --out $W\cal_best_defense.txt"

# ---- 4) 合并 + 三方验收（训练集 / 留出A / 留出B）----
Get-Content "$W\cal_best_attack.txt", "$W\cal_best_defense.txt" | Set-Content "$W\cal_best_combined.txt"
Step "验收-组合@训练集" "$PY $T eval --group both --games 20 --params $W\cal_best_combined.txt --workers 8 --binary $BIN --baseline $W\cal_base_train20.json"
Step "验收-组合@留出A"  "$PY $T eval --group both --games 20 --params $W\cal_best_combined.txt --holdout --holdout-seeds 1001 1002 1003 --workers 8 --binary $BIN --baseline $W\cal_base_A20.json"
Step "验收-组合@留出B"  "$PY $T eval --group both --games 20 --params $W\cal_best_combined.txt --holdout --holdout-seeds 2001 2002 2003 --workers 8 --binary $BIN --baseline $W\cal_base_B20.json"

Write-Output ""
Write-Output "=== 第二阶段完成 ==="
Get-ChildItem "$W\cal_best_*.txt" -ErrorAction SilentlyContinue | ForEach-Object { "  $($_.Name)  $($_.LastWriteTime.ToString('HH:mm'))" }
