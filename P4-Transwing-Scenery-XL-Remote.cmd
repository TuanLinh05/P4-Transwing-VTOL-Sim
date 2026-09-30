@echo off
setlocal

set "P4_LAUNCHER=%~dp0Simulation\control_station\start_control_station.ps1"
if not exist "%P4_LAUNCHER%" (
  echo Khong tim thay launcher: "%P4_LAUNCHER%"
  pause
  exit /b 1
)

echo Starting P4 Transwing XL scenery (1 km)...
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%P4_LAUNCHER%" -SceneryXL %*
if errorlevel 1 (
  echo XL scenery launcher failed.
  pause
)
endlocal
