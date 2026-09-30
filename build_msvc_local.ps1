# Local 32-bit MSVC Release build + offline_test for D:\robcup\strategy_5v5
$ErrorActionPreference = "Continue"

$repo = $PSScriptRoot
$vcvars = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat"
$cmake = "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
if (-not (Test-Path $cmake)) { $cmake = (Get-Command cmake).Source }

Write-Host "repo  = $repo"
Write-Host "cmake = $cmake"

# --- import MSVC x86 environment into this process -------------------------
$envFile = Join-Path $env:TEMP "vcenv_robcup.txt"
if (Test-Path $envFile) { Remove-Item $envFile -Force }
cmd /c "call `"$vcvars`" x86 >nul 2>&1 && set > `"$envFile`""
if (-not (Test-Path $envFile)) { throw "vcvarsall failed" }
Get-Content $envFile | ForEach-Object {
    if ($_ -match '^([^=]+)=(.*)$') {
        [System.Environment]::SetEnvironmentVariable($matches[1], $matches[2])
    }
}
Remove-Item $envFile -Force
Write-Host "VSCMD_ARG_TGT_ARCH = $env:VSCMD_ARG_TGT_ARCH"

# --- build (cache already configured for this source dir) ------------------
& $cmake --build "$repo\build" --config Release
Write-Host "BUILD_EXIT=$LASTEXITCODE"
if ($LASTEXITCODE -ne 0) { throw "build failed ($LASTEXITCODE)" }

Write-Host "`n=== artifacts ==="
Get-ChildItem "$repo\build\bin\Release" -File | Select-Object Name, Length, LastWriteTime | Format-Table -AutoSize
