@echo off
rem Builds dist\ChillerDiagGui\ (PyInstaller) and dist\ChillerDiagGui-Setup-<version>.exe (Inno Setup).
setlocal
cd /d "%~dp0"

set "PY=.venv38\Scripts\python.exe"
if not exist "%PY%" (
    echo Python 3.8 venv not found at %PY% - see README "Running from source".
    exit /b 1
)

set "ISCC="
for %%P in ("%LOCALAPPDATA%\Programs\Inno Setup 6\ISCC.exe" "%ProgramFiles(x86)%\Inno Setup 6\ISCC.exe" "%ProgramFiles%\Inno Setup 6\ISCC.exe") do (
    if exist %%P set "ISCC=%%~P"
)
if not defined ISCC (
    echo Inno Setup 6 not found - install it from https://jrsoftware.org/isdl.php
    exit /b 1
)

echo === Building app with PyInstaller ===
"%PY%" -m PyInstaller --noconfirm ChillerDiagGui.spec || goto :fail

echo === Building installer with Inno Setup ===
"%ISCC%" /Q installer\ChillerDiagGui.iss || goto :fail

echo.
echo Done. Installer:
dir /b dist\ChillerDiagGui-Setup-*.exe
exit /b 0

:fail
echo.
echo Build failed.
exit /b 1
