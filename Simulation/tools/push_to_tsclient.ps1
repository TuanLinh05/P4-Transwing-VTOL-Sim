[CmdletBinding()]
param(
    # In an RDP session, the remote Windows machine sees the development
    # machine's redirected drives through \\tsclient.
    [string]$Destination = "\\tsclient\D\Intern_Data\P4 Transwing Clone Drone"
)

$ErrorActionPreference = "Stop"
$source = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path

if (-not (Test-Path -LiteralPath $Destination -PathType Container)) {
    throw "Khong truy cap duoc may chinh qua RDP: $Destination. Kiem tra Drive redirection va giu ket noi Remote Desktop."
}

# This is deliberately an explicit, one-shot push. /XO protects a newer copy
# on the development machine, and the exclusions keep generated remote state
# from polluting the source workspace. No /MIR means no deletion propagates.
$excludedDirectories = @(
    (Join-Path $source "Simulation\ardupilot\sitl_state"),
    (Join-Path $source "Simulation\experiments\runs"),
    (Join-Path $source "__pycache__")
) | Where-Object { Test-Path -LiteralPath $_ -PathType Container }

$arguments = @(
    $source,
    $Destination,
    "/E",
    "/Z",
    "/FFT",
    "/XO",
    "/XJ",
    "/COPY:DAT",
    "/DCOPY:DAT",
    "/R:2",
    "/W:2",
    "/MT:8",
    "/XF",
    "*.pyc",
    "*.tlog",
    "*.tlog.raw",
    "eeprom.bin"
)
if ($excludedDirectories.Count -gt 0) {
    $arguments += "/XD"
    $arguments += $excludedDirectories
}

Write-Host "Dang day thay doi tu remote ve may chinh..." -ForegroundColor Cyan
& robocopy.exe @arguments
$result = $LASTEXITCODE
if ($result -ge 8) {
    throw "Robocopy that bai (exit code $result)."
}
Write-Host "Da day thay doi ve may chinh (Robocopy code $result)." -ForegroundColor Green
