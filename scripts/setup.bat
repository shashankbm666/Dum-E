@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0.."
title Dum-E Robotic Arm Setup

echo ================================================================
echo    Dum-E 4-DOF Robotic Arm - Automated 1-Click Installer
echo ================================================================
echo.

:: 1. Check Python
where python >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Python was not found in your system PATH!
    echo Please install Python 3.10 or 3.11 from https://www.python.org/downloads/
    echo Make sure to CHECK the box: "Add Python to PATH" during installation.
    echo.
    pause
    exit /b 1
)

for /f "tokens=*" %%v in ('python --version 2^>^&1') do set PY_VER=%%v
echo [1/3] Found %PY_VER%

:: 2. Create Virtual Environment
echo [2/3] Setting up Python virtual environment (.venv)...
if not exist ".venv" (
    python -m venv .venv
    if %ERRORLEVEL% NEQ 0 (
        echo [ERROR] Failed to create virtual environment!
        pause
        exit /b 1
    )
)
echo       Virtual environment ready.

:: 3. Install Dependencies
echo [3/3] Installing required libraries (OpenCV, Flask, NumPy, Requests)...
call .venv\Scripts\activate.bat
python -m pip install --upgrade pip --quiet
python -m pip install -r requirements.txt
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Failed to install dependencies!
    pause
    exit /b 1
)

echo.
echo ================================================================
echo    SETUP COMPLETE! Everything is ready to run.
echo    To launch the Command Center, double-click: scripts\run.bat
echo ================================================================
echo.
pause
