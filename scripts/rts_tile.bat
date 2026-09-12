@echo off
rem rts_tile.bat - double-click to start the whole system in three windows:
rem VM1 central top-left, VM3 railway top-right, VM2 intersections along the
rem bottom. Runs rts_tile.ps1 without needing a PowerShell execution policy.
rem
rem   rts_tile.bat        real time (90 s cycle)
rem   rts_tile.bat 5      the demonstration speed (18 s cycle)
set SPEED=%1
if "%SPEED%"=="" set SPEED=1
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0rts_tile.ps1" -Speed %SPEED%
if errorlevel 1 pause
