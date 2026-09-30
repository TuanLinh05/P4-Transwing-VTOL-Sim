@echo off
setlocal
set "P4_SYNC=%~dp0Simulation\tools\sync_from_tsclient.ps1"

if not exist "%P4_SYNC%" (
  echo Khong tim thay sync script:
  echo %P4_SYNC%
  pause
  exit /b 1
)

echo P4 Transwing source sync dang chay. Nhan Ctrl+C de dung.
"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" ^
  -NoProfile ^
  -ExecutionPolicy Bypass ^
  -File "%P4_SYNC%" ^
  -Watch

endlocal
