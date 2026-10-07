# probe_teamname.ps1 —— 读出任意策略 DLL 自称的队名（不启动平台）
# 原理：DLL 导出 SetYellowTeamName/SetBlueTeamName(char*)，平台就是靠它拿队名的。
# 必须用 32 位 PowerShell 运行（x64 载入不了 x86 DLL）：
#   C:\Windows\SysWOW64\WindowsPowerShell\v1.0\powershell.exe -File tools\ps1\probe_teamname.ps1
$ErrorActionPreference = 'Continue'
Add-Type -TypeDefinition @"
using System;
using System.Runtime.InteropServices;
public class KP {
  [DllImport("kernel32", SetLastError=true, CharSet=CharSet.Ansi)] public static extern IntPtr LoadLibraryA(string p);
  [DllImport("kernel32", SetLastError=true, CharSet=CharSet.Ansi)] public static extern IntPtr GetProcAddress(IntPtr h, string n);
  [DllImport("kernel32")] public static extern bool FreeLibrary(IntPtr h);
  public delegate void NameSetter(IntPtr buf);   // 具体委托类型（泛型在 PS5.1 里不能用于 GetDelegateForFunctionPointer）
}
"@
$cands = @(
  'C:\Strategy\Strategy4Yellow.dll',
  'C:\Strategy\backup_20261006\Strategy4Yellow_pre_b1.dll',
  'C:\Strategy\backup_20260912\Strategy4Yellow_before_demoname_20260930_1829.dll',
  'C:\Strategy\backup_20260912\Strategy4Yellow_before_opponent_20260930_1848.dll',
  'C:\Strategy\backup_20260912\Strategy4Yellow_before_restore_demo_20260930_1841.dll',
  'C:\Strategy\backup_20260912\Strategy4Yellow_before_r97_deploy_20260930_1819.dll',
  'D:\robcup5v5足球仿真组小型\strategy_5v5\robcup_small_sim\build\bin\Release\Strategy4Yellow.dll',
  'C:\Strategy\backup_official\Strategy4Yellow.dll',
  'C:\Strategy\hnnu_build\Strategy4Yellow.dll',
  'C:\Strategy\src\Strategy4Yellow\Release\Strategy4Yellow.dll'
)
foreach ($f in $cands) {
  if (-not (Test-Path $f)) { continue }
  $h = [KP]::LoadLibraryA($f)
  if ($h -eq [IntPtr]::Zero) { "--- {0}  [加载失败]" -f $f; continue }
  $out = @()
  foreach ($fn in @('SetYellowTeamName', '?SetYellowTeamName@@YAXPAD@Z', 'SetBlueTeamName', '?SetBlueTeamName@@YAXPAD@Z')) {
    $p = [KP]::GetProcAddress($h, $fn)
    if ($p -eq [IntPtr]::Zero) { continue }
    $del = [Runtime.InteropServices.Marshal]::GetDelegateForFunctionPointer($p, [KP+NameSetter])
    $buf = [Runtime.InteropServices.Marshal]::AllocHGlobal(256)
    for ($i = 0; $i -lt 256; $i++) { [Runtime.InteropServices.Marshal]::WriteByte($buf, $i, 0) }
    try { $del.Invoke($buf); $name = [Runtime.InteropServices.Marshal]::PtrToStringAnsi($buf) } catch { $name = '<调用异常>' }
    $short = if ($fn -like '*Yellow*') { 'Yellow' } else { 'Blue' }
    $out += ("{0} = '{1}'" -f $short, $name)
    [Runtime.InteropServices.Marshal]::FreeHGlobal($buf)
  }
  "--- {0}  ({1} B)" -f $f, (Get-Item $f).Length
  if ($out.Count) { $out | ForEach-Object { "      $_" } } else { "      （无队名导出）" }
  [void][KP]::FreeLibrary($h)
}