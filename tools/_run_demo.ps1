# _run_demo.ps1 — 跑一局「我方(蓝) vs 官方demo(黄)」，检测比赛结束并回报 .rlg
param([int]$TimeoutSec = 560)
$ErrorActionPreference = 'SilentlyContinue'
Add-Type -AssemblyName UIAutomationClient
Add-Type -AssemblyName UIAutomationTypes
Add-Type @'
using System;
using System.Runtime.InteropServices;
public class W {
  [DllImport("user32.dll")] public static extern bool SetCursorPos(int X, int Y);
  [DllImport("user32.dll")] public static extern void mouse_event(uint f, uint dx, uint dy, uint d, UIntPtr e);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
}
'@

$logFile = "C:\Strategy\_auto_run.log"
function Log($m) { $ts = Get-Date -Format 'HH:mm:ss'; Add-Content -Path $logFile -Value "$ts $m" }

function Click-Button($name) {
    $p = Get-Process SimuroSot5 -ErrorAction SilentlyContinue
    if (-not $p) { Log "NO SimuroSot5 process"; return $false }
    [W]::SetForegroundWindow($p.MainWindowHandle) | Out-Null
    Start-Sleep -Milliseconds 300
    $root = [System.Windows.Automation.AutomationElement]::RootElement
    $cond = New-Object System.Windows.Automation.PropertyCondition([System.Windows.Automation.AutomationElement]::ProcessIdProperty, $p.Id)
    $win = $root.FindFirst([System.Windows.Automation.TreeScope]::Children, $cond)
    if (-not $win) { Log "NO window"; return $false }
    $nc = New-Object System.Windows.Automation.PropertyCondition([System.Windows.Automation.AutomationElement]::NameProperty, $name)
    $btn = $win.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $nc)
    if (-not $btn) { Log "NO button $name"; return $false }
    $r = $btn.Current.BoundingRectangle
    $cx = [int]($r.X + $r.Width/2); $cy = [int]($r.Y + $r.Height/2)
    [W]::SetCursorPos($cx, $cy) | Out-Null
    Start-Sleep -Milliseconds 150
    [W]::mouse_event(0x02,0,0,0,[UIntPtr]::Zero)
    Start-Sleep -Milliseconds 80
    [W]::mouse_event(0x04,0,0,0,[UIntPtr]::Zero)
    Log "Clicked $name at $cx,$cy"
    return $true
}

Log "=== _run_demo start TimeoutSec=$TimeoutSec ==="
$startTime = Get-Date

$p = Get-Process SimuroSot5 -ErrorAction SilentlyContinue
if (-not $p) {
    Log "Launching SimuroSot5.exe"
    Start-Process -FilePath "C:\Strategy\SimuroSot5.exe" -WorkingDirectory "C:\Strategy"
    Start-Sleep -Seconds 7
    $p = Get-Process SimuroSot5 -ErrorAction SilentlyContinue
    if (-not $p) { Log "LAUNCH FAILED"; "RESULT:LAUNCH_FAILED" | Out-File -Encoding ascii C:\Strategy\_run_demo_result.txt; exit 1 }
}

if (-not (Click-Button 'Start (S)')) { Log "click Start failed"; "RESULT:START_FAILED" | Out-File -Encoding ascii C:\Strategy\_run_demo_result.txt; exit 1 }
Log "Game started, polling for completion"

# 只看 startTime 之后新写的 .rlg（本局），避免误判旧文件
$deadline = (Get-Date).AddSeconds($TimeoutSec)
$result = $null
while ((Get-Date) -lt $deadline) {
    $latest = Get-ChildItem C:\Strategy\*.rlg | Where-Object { $_.LastWriteTime -gt $startTime } | Sort-Object LastWriteTime | Select-Object -Last 1
    if ($latest) {
        $s1 = $latest.Length
        Start-Sleep -Seconds 25
        $latest.Refresh()
        $s2 = $latest.Length
        Log "poll: $($latest.Name) size $s1 -> $s2"
        # 结束判定：尺寸不再增长 且 已超过最小帧数(避免 0 字节桩)
        if ($s1 -gt 100000 -and $s1 -eq $s2) { $result = $latest.FullName; break }
    } else {
        Log "poll: no new rlg yet"
        Start-Sleep -Seconds 5
    }
}
if (-not $result) { $result = (Get-ChildItem C:\Strategy\*.rlg | Where-Object { $_.LastWriteTime -gt $startTime } | Sort-Object LastWriteTime | Select-Object -Last 1).FullName }
Log "result rlg: $result"
"RESULT:$result" | Out-File -Encoding ascii C:\Strategy\_run_demo_result.txt
Click-Button 'Close (C)' | Out-Null
Log "=== _run_demo done ==="
