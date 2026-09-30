@echo off
setlocal
set "P4_PUSH=%~dp0Simulation\tools\push_to_tsclient.ps1"

if not exist "%P4_PUSH%" (
  echo Khong tim thay push script:
  echo %P4_PUSH%
  pause
  exit /b 1
)

"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" ^
  -NoProfile ^
  -ExecutionPolicy Bypass ^
  -File "%P4_PUSH%"

if errorlevel 1 pause
endlocal
