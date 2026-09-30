[CmdletBinding()]
param(
    [ValidateRange(25.0, 60.0)]
    [double]$TakeoffAltitudeM = 30.0,
    [ValidateRange(1550, 1900)]
    [int]$FixedWingThrottle = 1750,
    [ValidateRange(20.0, 75.0)]
    [double]$FixedWingHoldSeconds = 45.0,
    [string]$Distro
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$baseUrl = "http://127.0.0.1:8765"
$simulationStarted = $false
$vehicleAirborne = $false
$transitionRequested = $false
$testPassed = $false

function Get-P4State {
    # Gazebo under WSL can briefly starve the dashboard process while the
    # physics loop catches up. A state read is diagnostic only, so tolerate a
    # short pause rather than falsely terminating an otherwise safe flight.
    Invoke-RestMethod -Uri "$baseUrl/api/state" -Method Get -TimeoutSec 15
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
        throw "A simulation stack is already active. Stop it before the transition smoke test."
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
        -Body @{ altitude = $TakeoffAltitudeM } | Out-Null
    $vehicleAirborne = $true

    $takeoffTimeout = [Math]::Max(120.0, ($TakeoffAltitudeM / 0.2) + 60.0)
    $takeoffDeadline = (Get-Date).AddSeconds($takeoffTimeout)
    do {
        Start-Sleep -Milliseconds 500
        $state = Get-P4State
        if ($state.vehicle.operation.status -eq "failed") {
            throw $state.vehicle.operation.message
        }
    } until (
        $state.vehicle.operation.status -eq "completed" -or
        (Get-Date) -ge $takeoffDeadline
    )
    if ($state.vehicle.operation.status -ne "completed") {
        throw "VTOL takeoff did not complete before the transition test."
    }

    $startAltitude = [double]$state.vehicle.position.relativeAlt
    if ($startAltitude -lt ($TakeoffAltitudeM - 1.5)) {
        throw "Transition altitude is too low: $startAltitude m."
    }

    # Exercise the same bounded transition endpoint used by the dashboard.
    # The backend owns RC during the transition, watches attitude / altitude,
    # and automatically requests QLOITER if a safety bound is crossed.
    Invoke-P4Post `
        -Path "/api/vehicle/transition-fw" `
        -Body @{
            throttle = $FixedWingThrottle
            minimumAltitude = 25
        } | Out-Null
    $transitionRequested = $true
    $transitionRequestedAt = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()

    $fbwbDeadline = (Get-Date).AddSeconds(7)
    do {
        Start-Sleep -Milliseconds 200
        $state = Get-P4State
    } until (
        [string]$state.vehicle.mode -eq "FBWB" -or
        $state.vehicle.operation.status -eq "failed" -or
        (Get-Date) -ge $fbwbDeadline
    )
    if ($state.vehicle.operation.status -eq "failed") {
        throw $state.vehicle.operation.message
    }
    if ([string]$state.vehicle.mode -ne "FBWB") {
        throw "Bounded transition endpoint did not enter FBWB."
    }

    $transitionDeadline = (Get-Date).AddSeconds(75)
    $fixedWingHoldDeadline = $null
    $minAltitude = $startAltitude
    $maxGroundspeed = 0.0
    $maxAbsRoll = 0.0
    $maxAbsPitch = 0.0
    $transitionAirspeedReached = $false
    $transitionDone = $false
    do {
        Start-Sleep -Milliseconds 250
        $state = Get-P4State
        if (-not $state.vehicle.connected) {
            throw "MAVLink disconnected during fixed-wing transition."
        }
        if (-not $state.vehicle.armed) {
            throw "Vehicle disarmed during fixed-wing transition."
        }
        if ($state.vehicle.operation.status -eq "failed") {
            throw $state.vehicle.operation.message
        }
        if ([string]$state.vehicle.mode -ne "FBWB") {
            throw "Unexpected mode during transition: $($state.vehicle.mode)"
        }

        $altitude = [double]$state.vehicle.position.relativeAlt
        $groundspeed = [double]$state.vehicle.hud.groundspeed
        $roll = [Math]::Abs([double]$state.vehicle.attitude.roll)
        $pitch = [Math]::Abs([double]$state.vehicle.attitude.pitch)
        $minAltitude = [Math]::Min($minAltitude, $altitude)
        $maxGroundspeed = [Math]::Max($maxGroundspeed, $groundspeed)
        $maxAbsRoll = [Math]::Max($maxAbsRoll, $roll)
        $maxAbsPitch = [Math]::Max($maxAbsPitch, $pitch)

        foreach ($statusMessage in $state.vehicle.statusMessages) {
            if ([long]$statusMessage.timestamp -lt $transitionRequestedAt) {
                continue
            }
            if ([string]$statusMessage.text -like "Transition airspeed reached*") {
                $transitionAirspeedReached = $true
            }
            if (([string]$statusMessage.text -like "Transition*done*") -or
                ([string]$statusMessage.text -like "Transition FW done*")) {
                $transitionDone = $true
            }
        }

        if ($altitude -lt 12.0 -or $altitude -lt ($startAltitude - 15.0)) {
            throw "Unsafe altitude loss during transition: $altitude m."
        }
        if ($roll -gt 35.0 -or $pitch -gt 25.0) {
            throw "Unsafe attitude during transition: roll=$roll, pitch=$pitch."
        }

        if (
            $null -eq $fixedWingHoldDeadline -and
            $state.vehicle.operation.name -eq "transition_fw" -and
            $state.vehicle.operation.status -eq "completed"
        ) {
            $fixedWingHoldDeadline = (Get-Date).AddSeconds(
                $FixedWingHoldSeconds
            )
        }
        if (
            $null -eq $fixedWingHoldDeadline -and
            (Get-Date) -ge $transitionDeadline
        ) {
            throw "Bounded transition endpoint did not complete within 75 seconds."
        }
    } until (
        $null -ne $fixedWingHoldDeadline -and
        (Get-Date) -ge $fixedWingHoldDeadline
    )

    if ($maxGroundspeed -lt 26.0) {
        throw "Transition did not build enough groundspeed: $maxGroundspeed m/s."
    }
    if (-not $transitionAirspeedReached) {
        throw "ArduPilot did not confirm the transition airspeed threshold."
    }
    if (-not $transitionDone) {
        throw "ArduPilot did not confirm a completed fixed-wing transition."
    }

    $testPassed = $true
    Write-Host "P4 fixed-wing transition smoke test PASSED." -ForegroundColor Green
    $resultSummary = (
        "Start altitude {0:N1} m; minimum {1:N1} m; max speed {2:N1} m/s; " +
        "max roll {3:N1} deg; max pitch {4:N1} deg; ArduPilot transition done."
    ) -f $startAltitude, $minAltitude, $maxGroundspeed, $maxAbsRoll, $maxAbsPitch
    Write-Host $resultSummary
}
finally {
    if ($vehicleAirborne) {
        try {
            # Remove forward throttle before commanding the VTOL recovery.
            if ($transitionRequested) {
                Invoke-P4Post `
                    -Path "/api/vehicle/center" `
                    -Body @{ throttle = 1500 } | Out-Null
            }
            Invoke-P4Post -Path "/api/vehicle/abort" | Out-Null
            Start-Sleep -Seconds 8
            $state = Get-P4State
            if ($state.vehicle.armed) {
                Invoke-P4Post -Path "/api/vehicle/qland" | Out-Null
                $landDeadline = (Get-Date).AddSeconds(75)
                do {
                    Start-Sleep -Seconds 1
                    $state = Get-P4State
                } until (
                    [double]$state.vehicle.position.relativeAlt -le 0.15 -or
                    (Get-Date) -ge $landDeadline
                )
                if ([double]$state.vehicle.position.relativeAlt -le 0.35) {
                    for (
                        $attempt = 0;
                        $attempt -lt 8 -and $state.vehicle.armed;
                        $attempt++
                    ) {
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
            Write-Warning "Transition recovery could not complete: $($_.Exception.Message)"
        }
    }

    if ($simulationStarted) {
        try {
            Invoke-P4Post -Path "/api/simulation/stop" | Out-Null
        }
        catch {
            Write-Warning "Simulation cleanup could not complete: $($_.Exception.Message)"
        }
    }

    if (-not $testPassed) {
        Write-Host "P4 fixed-wing transition smoke test FAILED." -ForegroundColor Red
    }
}
