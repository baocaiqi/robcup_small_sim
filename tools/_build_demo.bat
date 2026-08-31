@echo off
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars32.bat" >nul
if not exist "C:\Strategy\_demo_build" mkdir "C:\Strategy\_demo_build"
cd /d "C:\Strategy\src\Strategy4Yellow"
cl /nologo /O2 /MT /LD /EHsc /DWIN32 /DNDEBUG /D_WINDOWS /D_USRDLL /DSTRATEGY4YELLOW_EXPORTS ^
  /FI"C:\Strategy\_demo_build\_abs_shim.h" ^
  /Fo"C:\Strategy\_demo_build\\" /Fe"C:\Strategy\_demo_build\Strategy4Yellow.dll" ^
  Strategy4Yellow.cpp dllmain.cpp stdafx.cpp
echo EXITCODE=%ERRORLEVEL%
endlocal
