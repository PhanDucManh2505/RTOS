@echo off
rem rts_stop.bat - double-click to stop every RTS process on all three nodes.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0rts_stop.ps1"
pause
