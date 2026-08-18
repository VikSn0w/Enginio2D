@echo off
REM Configure, build and run without needing cmake/ninja on PATH or a
REM Developer Command Prompt. Pass "norun" to build only.
setlocal
cd /d "%~dp0"

set "VS=C:\Program Files\Microsoft Visual Studio\18\Community"
set "CMAKE=%VS%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "NINJA=%VS%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"

if not exist "%VS%\VC\Auxiliary\Build\vcvars64.bat" (
    echo Visual Studio not found at "%VS%" - edit the VS variable in this script.
    exit /b 1
)
call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1

"%CMAKE%" -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_MAKE_PROGRAM="%NINJA%" || exit /b 1
"%CMAKE%" --build build || exit /b 1

if /i "%~1"=="norun" exit /b 0
"%~dp0build\Enginio2D.exe"
