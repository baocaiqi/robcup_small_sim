# run_sim.ps1 — 自动化运行 N 局自博弈并记录进度
param([int]$Games = 3, [int]$WaitSec = 540)
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

Log "=== run_sim start: Games=$Games WaitSec=$WaitSec ==="

# 确保平台已启动
$p = Get-Process SimuroSot5 -ErrorAction SilentlyContinue
if (-not $p) {
    Log "Launching SimuroSot5.exe"
    Start-Process -FilePath "C:\Strategy\SimuroSot5.exe" -WorkingDirectory "C:\Strategy"
    Start-Sleep -Seconds 6
    $p = Get-Process SimuroSot5 -ErrorAction SilentlyContinue
    if (-not $p) { Log "LAUNCH FAILED"; exit 1 }
}

for ($g = 1; $g -le $Games; $g++) {
    Log "--- Game ${g} / ${Games}: clicking Start ---"
    if (-not (Click-Button 'Start (S)')) { Log "click Start failed"; break }
    Log "Game $g started, waiting ${WaitSec}s"
    Start-Sleep -Seconds $WaitSec
    Log "Game $g wait done"
}

Log "--- all games done, clicking Close ---"
Click-Button 'Close (C)' | Out-Null
Log "=== run_sim done ==="
