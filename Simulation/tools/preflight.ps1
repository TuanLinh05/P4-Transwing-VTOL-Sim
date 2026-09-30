[CmdletBinding()]
param(
    [switch]$Deep,
    [string]$Distro
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "P4Environment.ps1")

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$requestedDistro = if ($Distro) {
    $Distro
}
elseif ($env:P4_TRANSWING_WSL_DISTRO) {
    $env:P4_TRANSWING_WSL_DISTRO
}
else {
    ""
}
$resolvedDistro = Resolve-P4WslDistro -Requested $requestedDistro
$wslRepo = ConvertTo-P4WslPath -Distro $resolvedDistro -WindowsPath $repoRoot
$wslPreflight = "$wslRepo/Simulation/tools/preflight_wsl.sh"

Write-Host "P4 Transwing preflight" -ForegroundColor Cyan
Write-Host "  WSL distro : $resolvedDistro"
Write-Host "  Repository : $repoRoot"
Write-Host "  WSL path   : $wslRepo"

$arguments = @("-d", $resolvedDistro, "--", "env", "P4_TRANSWING_REPO=$wslRepo",
    "bash", $wslPreflight)
if ($Deep) {
    $arguments += "--deep"
}

& wsl.exe @arguments
if ($LASTEXITCODE -ne 0) {
    throw "P4 WSL preflight khong dat. Xem cac dong [FAIL] o tren."
}

if (-not (Test-P4TcpPortAvailable -Port 8765)) {
    try {
        $health = Invoke-RestMethod -Uri "http://127.0.0.1:8765/api/health" -TimeoutSec 1
        if ($health.ok -eq $true) {
            Write-Warning "Control Station da chay tren port 8765."
        }
        else {
            throw "unhealthy"
        }
    }
    catch {
        throw "Port Windows 8765 dang bi mot chuong trinh khac su dung."
    }
}
else {
    Write-Host "[ OK ] Windows TCP 8765 is available"
}

Write-Host "P4 Windows/WSL preflight PASSED." -ForegroundColor Green
