@echo off
chcp 65001 >nul
rem ============================================================
rem 双击这个文件 = 打开「赛后回放器」（读记录下来的整段数据，可手动拖 / 可自动播放）
rem   · 默认回放最新一段（黑匣子 CSV 的最后一个 session）
rem   · 想回放更早的：编辑下面这行的 --session 2 / 3 …
rem   · 操作：空格=播放/暂停  ←/→=逐帧  Shift+←/→=1秒  滑块=手动拖  「下一个死球」=跳到下次摆位
rem ============================================================
cd /d "%~dp0"
echo 正在打开赛后回放器……
where python >nul 2>nul
if %errorlevel%==0 (
    python "tools\py\replay_player.py" --session 1
) else (
    "%LOCALAPPDATA%\Programs\Python\Python311-32\python.exe" "tools\py\replay_player.py" --session 1
)
if errorlevel 1 (
    echo.
    echo [出错] 常见原因：1) 没装 Python  2) 没装 pillow（pip install pillow）
    echo        3) 黑匣子 CSV 不存在（C:\Strategy\hnnu_blackbox.csv）
    pause
)
