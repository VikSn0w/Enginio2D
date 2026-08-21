@echo off
REM Configure, build and run without needing cmake/ninja on PATH or a
REM Developer Command Prompt. Pass "norun" to build only.
REM
REM Visual Studio is found with vswhere, which ships with every installation
REM since 2017 and is always at the same place. Hard-coding an install path
REM works on exactly one machine: the edition, the year and the drive all
REM change from one to the next, and the script then reports that Visual
REM Studio is missing on a machine that has it.
setlocal
cd /d "%~dp0"

set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
set "VS="
if exist "%VSWHERE%" (
    for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath 2^>nul`) do set "VS=%%i"
    REM A Build Tools install has no VC.Tools component id under some SKUs,
    REM so fall back to whatever the newest installation is.
    if not defined VS (
        for /f "usebackq delims=" %%i in (`"%VSWHERE%" -latest -products * -property installationPath 2^>nul`) do set "VS=%%i"
    )
)
if not defined VS if defined VSINSTALLDIR set "VS=%VSINSTALLDIR%"

if not defined VS (
    echo Could not find Visual Studio. Set VSINSTALLDIR to the installation
    echo directory ^(the one containing VC\Auxiliary\Build\vcvars64.bat^) and
    echo run this again.
    exit /b 1
)
if not exist "%VS%\VC\Auxiliary\Build\vcvars64.bat" (
    echo Found "%VS%" but it has no C++ toolset - install the
    echo "Desktop development with C++" workload.
    exit /b 1
)

set "CMAKE=%VS%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
set "NINJA=%VS%\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
REM Fall back to whatever is on PATH if the bundled copies are not installed.
if not exist "%CMAKE%" set "CMAKE=cmake.exe"
if not exist "%NINJA%" set "NINJA=ninja.exe"

call "%VS%\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1

"%CMAKE%" -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_MAKE_PROGRAM="%NINJA%" || exit /b 1
"%CMAKE%" --build build || exit /b 1

if /i "%~1"=="norun" exit /b 0
"%~dp0build\Enginio2D.exe"
