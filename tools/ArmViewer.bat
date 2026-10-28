@echo off
rem Launch the arm pose viewer. Optional arguments are passed through, e.g.
rem   ArmViewer.bat -Port COM5
rem   ArmViewer.bat -Replay build\potlog6.txt
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0ArmViewer.ps1" %*
if errorlevel 1 pause
