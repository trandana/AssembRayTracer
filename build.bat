@echo off
rem Builds src\raytracer.asm into build\raytracer.exe (x64 MASM, AVX-512, no CRT).
rem Usage:  build.bat          build only
rem         build.bat run      build, render, and open render.bmp
setlocal
cd /d "%~dp0"

rem Find the newest Visual Studio / Build Tools that has the C++ toolchain.
set "VSPATH="
pushd "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"
for /f "usebackq delims=" %%i in (`.\vswhere.exe -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSPATH=%%i"
popd
if not defined VSPATH (
    echo Visual Studio with C++ tools not found
    exit /b 1
)
call "%VSPATH%\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul 2>nul || exit /b 1

if not exist build mkdir build
ml64 /nologo /c /Zi /Fobuild\raytracer.obj /Flbuild\raytracer.lst src\raytracer.asm || exit /b 1
link /nologo /subsystem:windows /entry:main /nodefaultlib /debug /out:build\raytracer.exe build\raytracer.obj kernel32.lib user32.lib gdi32.lib || exit /b 1

if /i "%1"=="run" (
    start "" build\raytracer.exe
)

