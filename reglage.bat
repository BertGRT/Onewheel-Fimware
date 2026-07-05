@echo off
REM  Interface web de reglage Onewheel (via ST-Link). Roues en l'air !
REM  Ouvre ensuite http://127.0.0.1:8666/ dans ton navigateur.
title Onewheel Tuner
echo ============================================================
echo   INTERFACE DE REGLAGE ONEWHEEL (ST-Link)
echo   Ouvre dans ton navigateur :  http://127.0.0.1:8666/
echo   Ctrl+C ici pour arreter.
echo ============================================================
echo.
python "C:\onewheel\tuner\bridge.py"
echo.
echo (passerelle arretee)
pause
