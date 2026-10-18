@echo off
cd /d "%~dp0.."
set "PATH=C:\Users\Milan Gaurav\.local\git\cmd;%PATH%"
echo Pushing Dum-E to GitHub (https://github.com/shashankbm666/Dum-E)...
echo.
git push -u origin main
echo.
pause
