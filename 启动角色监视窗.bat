@echo off
chcp 65001 >nul
rem ============================================================
rem 双击这个文件即可打开「角色实时监视窗」（我方 5 台标出角色名）
rem   · 不用开终端、不用记命令；窗口关掉就停止
rem   · 若提示"等待黑匣子数据…" → 说明平台没在跑，或平台需要重启
rem     （重启后才会加载新版策略 DLL）
rem ============================================================
cd /d "%~dp0"
echo 正在打开角色监视窗……
where python >nul 2>nul
if %errorlevel%==0 (
    python "tools\py\role_monitor.py"
) else (
    "%LOCALAPPDATA%\Programs\Python\Python311-32\python.exe" "tools\py\role_monitor.py"
)
if errorlevel 1 (
    echo.
    echo [出错] 上面是错误信息。常见原因：
    echo   1) 没装 Python 或没加到 PATH
    echo   2) 没装依赖：pip install pillow
    echo.
    pause
)
