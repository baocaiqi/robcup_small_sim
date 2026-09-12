# ============================================================
# fix_demo_yellow.ps1 — 官方 demo（黄队）出包助手：/MT 静态运行库 + 校验（默认只检查）
#
# ⚠️ 为什么有这个脚本（2026-09-11 发现）：
#   `C:\Strategy\.tmp\demo_yellow` 那份 CMake 工程是按 **CMake 默认(/MD)** 出包的，
#   产物导入表里带 VCRUNTIME140.dll / api-ms-win-crt-*；
#   而官方 demo 与我们的蓝队 DLL 都是 **/MT**（导入表只有 KERNEL32.dll）。
#   本机装了 VC++ 运行库，所以 /MD 版能跑（2026-09-11 09:44 场实测正常）；
#   但换到没装 VC++ 2015-2022 x86 运行库的机器（比赛现场低配机）会 LoadLibrary 失败
#   → 策略加载不上、机器人不动（docs/02 红线）。出包给别人的黄队 DLL 建议 /MT。
#
# ⚠️ 不要用旧脚本/旧目录出包：v1（五道闸，build\demo_yellow_patch 里那份源码）已被
#   第 48 轮的 v2 取代（删掉④门将保护圈、⑤死球全队退回，并修了门球落点 70→90）。
#   本脚本**只从 C:\Strategy\.tmp\demo_yellow 当前源码出包**，不碰仓库里的旧副本。
#
# 用法（仓库根目录；平台要先关掉）：
#     pwsh -File tools\ps1\fix_demo_yellow.ps1                 # 只检查（默认，绝不写 C:\Strategy）
#     pwsh -File tools\ps1\fix_demo_yellow.ps1 -Build          # 出 /MT 包 + 校验，但仍不部署
#     pwsh -File tools\ps1\fix_demo_yellow.ps1 -Build -Deploy  # 备份现役后覆盖部署
# ============================================================
param(
    [switch]$Build,
    [switch]$Deploy
)
$ErrorActionPreference = 'Stop'
$env:PYTHONIOENCODING = 'utf-8'
[Console]::OutputEncoding = [Text.Encoding]::UTF8

$root    = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$py      = Join-Path $root 'tools\py'
$proj    = 'C:\Strategy\.tmp\demo_yellow'          # 当前源码（v2 补丁版）
$official= 'C:\Strategy\backup_official\Strategy4Yellow.dll'
$deploy  = 'C:\Strategy\Strategy4Yellow.dll'
$out     = Join-Path $env:TEMP 'demo_yellow_mt_out'

function Get-Imports([string]$dll) {
    python "$py\pe_imports.py" $dll
}
function Get-Exports([string]$a, [string]$b) {
    python "$py\pe_exports.py" $a $b
}

Write-Host "[检查] 现役黄队 DLL 的运行库依赖 ..." -ForegroundColor Cyan
if (Test-Path $deploy) { Get-Imports $deploy } else { Write-Host "  （还没有 $deploy）" }

if (-not $Build) {
    Write-Host "`n提示：加 -Build 出 /MT 包（输出到 $out），加 -Deploy 才会覆盖现役 DLL。" -ForegroundColor Yellow
    exit 0
}

Write-Host "[构建] $proj → $out（CMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded）..." -ForegroundColor Cyan
cmake -S $proj -B $out -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreaded | Select-Object -Last 2
cmake --build $out --config Release | Select-String -Pattern 'error|\.dll$|\.exe$'
$new = Join-Path $out 'Release\Strategy4Yellow.dll'
if (-not (Test-Path $new)) { throw "构建失败：找不到 $new" }

Write-Host "[校验] 导入表（应只有 KERNEL32.dll）/ 导出表（应与官方 5 个 mangled 名一致）..." -ForegroundColor Cyan
Get-Imports $new
Get-Exports $official $new
if ($LASTEXITCODE -ne 0) { throw "导出符号与官方 demo 不一致——平台按 mangled 名加载，不能部署" }

if (-not $Deploy) {
    Write-Host "`n[完成] /MT 包已就绪：$new（未部署；加 -Deploy 才覆盖）" -ForegroundColor Green
    exit 0
}

$running = Get-Process SimuroSot5, WorldModel -ErrorAction SilentlyContinue
if ($running) {
    Write-Host "⚠️ 平台正在运行，先关掉再部署（DLL 被占用）" -ForegroundColor Red
    $running | Select-Object Name, Id
    exit 2
}
$bakDir = 'C:\Strategy\backup_' + (Get-Date -Format 'yyyyMMdd')
New-Item -ItemType Directory -Force -Path $bakDir | Out-Null
Copy-Item $deploy "$bakDir\Strategy4Yellow_pre_mt_$(Get-Date -Format 'HHmm').dll" -Force
Copy-Item $new $deploy -Force
Get-FileHash $new, $deploy -Algorithm SHA1 | Select-Object Hash, Path
Write-Host "✅ 已部署 /MT 版（旧版已备份到 $bakDir）" -ForegroundColor Green
