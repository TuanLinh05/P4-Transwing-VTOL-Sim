[CmdletBinding()]
param(
    [switch]$NoBrowser,
    [switch]$SkipPreflight,
    [string]$Distro,
    [ValidateSet("Software", "Gpu")]
    [string]$RenderMode = "Software",
    [switch]$Scenery,
    [switch]$SceneryXL
)

$ErrorActionPreference = "Stop"

. (Join-Path $PSScriptRoot "..\tools\P4Environment.ps1")

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
$wslLauncher = "$wslRepo/Simulation/control_station/start_control_station_wsl.sh"
$dashboardUrl = "http://127.0.0.1:8765"
$logDirectory = Join-Path $env:LOCALAPPDATA "P4 Transwing"
$stdoutLog = Join-Path $logDirectory "control-station.log"
$stderrLog = Join-Path $logDirectory "control-station-error.log"
$softwareRendering = if ($RenderMode -eq "Gpu") { "0" } else { "1" }

function Test-ControlStation {
    try {
        $response = Invoke-RestMethod `
            -Uri "$dashboardUrl/api/health" `
            -Method Get `
            -TimeoutSec 1
        return $response.ok -eq $true
    }
    catch {
        return $false
    }
}

function Resolve-MissionPlannerPath {
    $configured = $env:P4_TRANSWING_MISSION_PLANNER
    if ($configured) {
        if (-not (Test-Path -LiteralPath $configured -PathType Leaf)) {
            throw "Khong tim thay Mission Planner: $configured"
        }
        return (Resolve-Path -LiteralPath $configured).Path
    }

    # ArduPilot's native Windows installer normally uses the first path.
    # Detect it automatically so a standard Mission Planner install needs no
    # environment-variable setup. The explicit environment variable remains
    # available for portable/custom installations.
    $candidates = @(
        (Join-Path ${env:ProgramFiles(x86)} "Mission Planner\MissionPlanner.exe"),
        (Join-Path $env:ProgramFiles "Mission Planner\MissionPlanner.exe")
    ) | Where-Object { $_ }

    # MSI installs normally register their install directory even when a user
    # selected a non-default location. Read both 32-bit and 64-bit uninstall
    # registries before giving up; this avoids requiring a manual environment
    # variable for a standard custom install.
    $uninstallKeys = @(
        "HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\*",
        "HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\*",
        "HKLM:\Software\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\*"
    )
    foreach ($uninstallKey in $uninstallKeys) {
        $registrations = Get-ItemProperty -Path $uninstallKey -ErrorAction SilentlyContinue
        foreach ($registration in $registrations) {
            $displayNameProperty = $registration.PSObject.Properties["DisplayName"]
            if ($null -eq $displayNameProperty -or [string]$displayNameProperty.Value -notmatch "Mission Planner") {
                continue
            }
            $installLocationProperty = $registration.PSObject.Properties["InstallLocation"]
            if ($null -ne $installLocationProperty -and $installLocationProperty.Value) {
                $candidates += Join-Path ([string]$installLocationProperty.Value) "MissionPlanner.exe"
            }
            $displayIconProperty = $registration.PSObject.Properties["DisplayIcon"]
            if ($null -ne $displayIconProperty -and $displayIconProperty.Value) {
                $candidates += (
                    ([string]$displayIconProperty.Value -replace ',\d+\s*$', "").Trim('"')
                )
            }
        }
    }

    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    return $null
}

$stationWasRunning = Test-ControlStation
if (-not $stationWasRunning) {
    New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null

    if (-not (Test-P4TcpPortAvailable -Port 8765)) {
        throw "Port 8765 dang duoc mot chuong trinh khac su dung. Hay dung chuong trinh do truoc."
    }

    if (-not $SkipPreflight) {
        Write-Host "Dang kiem tra moi truong P4 Transwing..." -ForegroundColor Cyan
        & (Join-Path $repoRoot "Simulation\tools\preflight.ps1") `
            -Distro $resolvedDistro
        if ($LASTEXITCODE -ne 0) {
            throw "P4 Transwing preflight khong dat."
        }
    }

    Write-Host "Dang khoi dong P4 Transwing Control Station ($resolvedDistro)..." -ForegroundColor Cyan
    # Start-Process joins ArgumentList entries into a single command line.
    # Quote the WSL script explicitly because this repository path contains
    # spaces ("P4 Transwing Clone Drone").
    $wslArgumentLine = "-d $resolvedDistro -- bash `"$wslLauncher`""
    $previousWslEnv = $env:WSLENV
$previousMissionPlanner = $env:P4_TRANSWING_MISSION_PLANNER
$previousSoftwareRendering = $env:P4_TRANSWING_SOFTWARE_RENDERING
$previousGazeboWorld = $env:P4_TRANSWING_GAZEBO_WORLD
try {
        # Gazebo runs under WSLg. Keep the conservative software renderer as
        # the normal default, but explicitly forward an opt-in GPU mode to WSL.
        $env:P4_TRANSWING_SOFTWARE_RENDERING = $softwareRendering
        $wslEnvEntries = @(
            ($previousWslEnv -split ":") |
            Where-Object {
                $_ -and $_ -notmatch "^P4_TRANSWING_(?:MISSION_PLANNER|SOFTWARE_RENDERING|GAZEBO_WORLD)(?:/.*)?$"
            }
        )

        if ($SceneryXL) {
            $env:P4_TRANSWING_GAZEBO_WORLD = "scenery_xl"
            $wslEnvEntries += "P4_TRANSWING_GAZEBO_WORLD"
        }
        elseif ($Scenery) {
            $env:P4_TRANSWING_GAZEBO_WORLD = "scenery"
            $wslEnvEntries += "P4_TRANSWING_GAZEBO_WORLD"
        }

        # WSLENV's /p modifier converts an optional Windows Mission Planner
        # path (including a different drive) into the matching /mnt/... path.
        $missionPlannerPath = Resolve-MissionPlannerPath
        if ($missionPlannerPath) {
            $env:P4_TRANSWING_MISSION_PLANNER = $missionPlannerPath
            $wslEnvEntries += "P4_TRANSWING_MISSION_PLANNER/p"
        }
        $wslEnvEntries += "P4_TRANSWING_SOFTWARE_RENDERING"
        $env:WSLENV = $wslEnvEntries -join ":"

        Start-Process `
            -FilePath "wsl.exe" `
            -ArgumentList $wslArgumentLine `
            -WindowStyle Hidden `
            -RedirectStandardOutput $stdoutLog `
            -RedirectStandardError $stderrLog | Out-Null
    }
    finally {
        $env:WSLENV = $previousWslEnv
        $env:P4_TRANSWING_MISSION_PLANNER = $previousMissionPlanner
        $env:P4_TRANSWING_SOFTWARE_RENDERING = $previousSoftwareRendering
        $env:P4_TRANSWING_GAZEBO_WORLD = $previousGazeboWorld
    }

    $deadline = (Get-Date).AddSeconds(45)
    while ((Get-Date) -lt $deadline -and -not (Test-ControlStation)) {
        Start-Sleep -Milliseconds 350
    }

    if (-not (Test-ControlStation)) {
        $detail = ""
        if (Test-Path $stderrLog) {
            $detail = (Get-Content $stderrLog -Tail 12) -join [Environment]::NewLine
        }
        throw "Dashboard khong khoi dong trong 45 giay.`n$detail`nLog: $stderrLog"
    }
}

if ($RenderMode -eq "Gpu") {
    if ($stationWasRunning) {
        Write-Host "Control Station da chay; GPU mode se chi co hieu luc sau khi dung backend WSL va mo lai launcher GPU." -ForegroundColor Yellow
    }
    else {
        Write-Host "Gazebo GUI: dang yeu cau GPU rendering qua WSLg." -ForegroundColor Cyan
        Write-Host "Neu cua so den/treo, dung stack va mo lai launcher mac dinh (software)." -ForegroundColor Yellow
    }
}

Write-Host "P4 Transwing Control Station: $dashboardUrl" -ForegroundColor Green
Write-Host "Log backend: $stdoutLog"

if (-not $NoBrowser) {
    Start-Process $dashboardUrl
}
