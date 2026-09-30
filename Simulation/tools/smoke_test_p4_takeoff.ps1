[CmdletBinding()]
param(
    [ValidateRange(3.0, 20.0)]
    [double]$TargetAltitudeM = 5.0,
    [string]$Distro
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$baseUrl = "http://127.0.0.1:8765"
$simulationStarted = $false
$takeoffPassed = $false

function Get-P4State {
    Invoke-RestMethod -Uri "$baseUrl/api/state" -Method Get -TimeoutSec 3
}

function Invoke-P4Post {
    param(
        [Parameter(Mandatory)]
        [string]$Path,
        [hashtable]$Body = @{}
    )

    $config = Invoke-RestMethod -Uri "$baseUrl/api/config" -Method Get -TimeoutSec 3
    $headers = @{
        Origin = $baseUrl
        "X-P4-Transwing-CSRF" = $config.csrfToken
    }
    Invoke-RestMethod `
        -Uri "$baseUrl$Path" `
        -Method Post `
        -Headers $headers `
        -ContentType "application/json" `
        -Body ($Body | ConvertTo-Json -Compress) `
        -TimeoutSec 25
}

try {
    try {
        $health = Invoke-RestMethod -Uri "$baseUrl/api/health" -Method Get -TimeoutSec 2
    }
    catch {
        $health = $null
    }

    if (-not $health.ok) {
        $launcher = Join-Path $repoRoot "Simulation\control_station\start_control_station.ps1"
        $launcherArgs = @{ NoBrowser = $true }
        if ($Distro) {
            $launcherArgs.Distro = $Distro
        }
        & $launcher @launcherArgs
    }

    $state = Get-P4State
    if ($state.simulation.status -in @("starting", "running", "degraded", "stopping")) {
        throw "A simulation stack is already active. Stop it before the takeoff smoke test."
    }

    Invoke-P4Post `
        -Path "/api/simulation/start" `
        -Body @{
            profile = "baseline"
            openMissionPlanner = $false
            autoMissionPlanner = $false
        } | Out-Null
    $simulationStarted = $true

    $readyDeadline = (Get-Date).AddSeconds(75)
    do {
        Start-Sleep -Milliseconds 500
        $state = Get-P4State
        if ($state.simulation.status -eq "error") {
            throw "Simulation startup failed: $($state.simulation.error)"
        }
    } until ($state.vehicle.prearm.ready -or (Get-Date) -ge $readyDeadline)
    if (-not $state.vehicle.prearm.ready) {
        throw "Vehicle did not become pre-arm ready within 75 seconds."
    }

    Invoke-P4Post `
        -Path "/api/vehicle/takeoff" `
        -Body @{ altitude = $TargetAltitudeM } | Out-Null

    # Match the Control Station's conservative 0.2 m/s climb-time model.
    # A 20 m test can legitimately take longer than 90 s after the near-target
    # RC3 taper begins, especially on slower remote WSL machines.
    $takeoffTimeoutSeconds = [Math]::Max(90.0, ($TargetAltitudeM / 0.2) + 60.0)
    $takeoffDeadline = (Get-Date).AddSeconds($takeoffTimeoutSeconds)
    $maxAbsPitch = 0.0
    $maxAbsRoll = 0.0
    do {
        Start-Sleep -Milliseconds 500
        $state = Get-P4State
        $maxAbsPitch = [Math]::Max(
            $maxAbsPitch,
            [Math]::Abs([double]$state.vehicle.attitude.pitch)
        )
        $maxAbsRoll = [Math]::Max(
            $maxAbsRoll,
            [Math]::Abs([double]$state.vehicle.attitude.roll)
        )
        if ($state.vehicle.operation.status -eq "failed") {
            throw $state.vehicle.operation.message
        }
    } until (
        $state.vehicle.operation.status -eq "completed" -or
        (Get-Date) -ge $takeoffDeadline
    )

    $completionToleranceM = [Math]::Max(
        0.4,
        [Math]::Min(0.8, $TargetAltitudeM * 0.04)
    )
    $minimumAcceptedAltitude = $TargetAltitudeM - $completionToleranceM
    if (
        $state.vehicle.operation.status -ne "completed" -or
        [double]$state.vehicle.position.relativeAlt -lt $minimumAcceptedAltitude
    ) {
        throw (
            "Takeoff did not reach the accepted altitude. " +
            "Reported=$($state.vehicle.position.relativeAlt) m, " +
            "required>=$minimumAcceptedAltitude m."
        )
    }

    $hoverSamples = 1..5 | ForEach-Object {
        Start-Sleep -Seconds 1
        Get-P4State
    }
    $hoverAltitudes = $hoverSamples |
        ForEach-Object { [double]$_.vehicle.position.relativeAlt }
    $hoverPitches = $hoverSamples |
        ForEach-Object { [Math]::Abs([double]$_.vehicle.attitude.pitch) }
    $hoverRange = (
        ($hoverAltitudes | Measure-Object -Maximum).Maximum -
        ($hoverAltitudes | Measure-Object -Minimum).Minimum
    )
    $hoverMaxPitch = ($hoverPitches | Measure-Object -Maximum).Maximum
    if ($hoverRange -gt 0.30 -or $hoverMaxPitch -gt 10.0) {
        throw (
            "Post-takeoff hover was not stable: " +
            "altitude range=$([Math]::Round($hoverRange, 2)) m, " +
            "max pitch=$([Math]::Round($hoverMaxPitch, 1)) deg."
        )
    }

    $takeoffPassed = $true
    Write-Host "P4 ground-takeoff smoke test PASSED." -ForegroundColor Green
    $summary = (
        "Target {0:N1} m; reached {1:N2} m; max transient pitch {2:N1} deg; " +
        "max roll {3:N1} deg; 5 s hover range {4:N2} m; " +
        "max hover pitch {5:N1} deg."
    ) -f `
        $TargetAltitudeM, `
        $state.vehicle.position.relativeAlt, `
        $maxAbsPitch, `
        $maxAbsRoll, `
        $hoverRange, `
        $hoverMaxPitch
    Write-Host $summary
}
finally {
    if ($simulationStarted) {
        try {
            $state = Get-P4State
            if ($state.vehicle.armed) {
                Invoke-P4Post -Path "/api/vehicle/qland" | Out-Null
                $landDeadline = (Get-Date).AddSeconds(55)
                do {
                    Start-Sleep -Seconds 1
                    $state = Get-P4State
                } until (
                    [double]$state.vehicle.position.relativeAlt -le 0.15 -or
                    (Get-Date) -ge $landDeadline
                )
                if ([double]$state.vehicle.position.relativeAlt -le 0.15) {
                    for ($attempt = 0; $attempt -lt 5 -and $state.vehicle.armed; $attempt++) {
                        try {
                            Invoke-P4Post -Path "/api/vehicle/disarm" | Out-Null
                        }
                        catch {
                            Start-Sleep -Seconds 1
                        }
                        $state = Get-P4State
                    }
                }
            }
        }
        catch {
            Write-Warning "Landing cleanup could not complete: $($_.Exception.Message)"
        }
        finally {
            try {
                Invoke-P4Post -Path "/api/simulation/stop" | Out-Null
            }
            catch {
                Write-Warning "Simulation cleanup could not complete: $($_.Exception.Message)"
            }
        }
    }

    if (-not $takeoffPassed) {
        Write-Host "P4 ground-takeoff smoke test FAILED." -ForegroundColor Red
    }
}
