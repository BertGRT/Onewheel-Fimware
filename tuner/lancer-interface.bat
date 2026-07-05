@echo off
REM  Lance la passerelle + interface web de reglage Onewheel (via ST-Link).
REM  Prerequis : ST-Link branche, carte alimentee (moteurs debranches pour regler).
REM  Ensuite : ouvre http://127.0.0.1:8666/ dans Chrome/Edge/Firefox.
echo Demarrage de la passerelle Onewheel...
echo Ouvre ensuite :  http://127.0.0.1:8666/
echo (Ctrl+C dans cette fenetre pour arreter)
echo.
python C:\onewheel\tuner\bridge.py
pause
