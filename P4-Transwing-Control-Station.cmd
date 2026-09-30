@echo off
setlocal
set "P4_LAUNCHER=%~dp0Simulation\control_station\start_control_station.ps1"

if not exist "%P4_LAUNCHER%" (
  echo Khong tim thay launcher:
  echo %P4_LAUNCHER%
  pause
  exit /b 1
)

"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" ^
  -NoProfile ^
  -ExecutionPolicy Bypass ^
  -File "%P4_LAUNCHER%" %*

if errorlevel 1 (
  echo.
  echo P4 Transwing Control Station khong khoi dong duoc.
  pause
)
endlocal
