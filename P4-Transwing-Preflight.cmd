@echo off
setlocal
set "P4_PREFLIGHT=%~dp0Simulation\tools\preflight.ps1"

if not exist "%P4_PREFLIGHT%" (
  echo Khong tim thay preflight:
  echo %P4_PREFLIGHT%
  pause
  exit /b 1
)

"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" ^
  -NoProfile ^
  -ExecutionPolicy Bypass ^
  -File "%P4_PREFLIGHT%" %*

if errorlevel 1 (
  echo.
  echo Kiem tra P4 Transwing khong dat. Xem cac dong [FAIL] o tren.
  pause
) else (
  echo.
  echo Moi truong P4 Transwing da san sang.
)
endlocal
