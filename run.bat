@echo off
setlocal
title Dum-E Robotic Arm Command Center

if not exist ".venv\Scripts\activate.bat" (
    echo [ERROR] Virtual environment not found. Please run setup.bat first!
    pause
    exit /b 1
)

echo Starting Dum-E Command Center...
call .venv\Scripts\activate.bat

:: Launch default web browser after 1 second
start "" cmd /c "timeout /t 2 >nul & start http://localhost:8000/"

:: Run server
python run_server.py
pause
