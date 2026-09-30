[CmdletBinding()]
param(
    # The RDP redirected drive of the development machine. Override this when
    # the source project is stored on a different drive or in another folder.
    [string]$Source = "\\tsclient\D\Intern_Data\P4 Transwing Clone Drone",
    [ValidateRange(2, 3600)][int]$IntervalSeconds = 5,
    [switch]$Watch
)

$ErrorActionPreference = "Stop"
$destination = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path

function Invoke-P4SourceSync {
    if (-not (Test-Path -LiteralPath $Source -PathType Container)) {
        throw "Khong truy cap duoc source qua RDP: $Source. Kiem tra Drive redirection va giu ket noi Remote Desktop."
    }

    # This is deliberately one-way and non-destructive: do not use /MIR here.
    # SITL logs, EEPROM, local .git data and remote-only diagnostics survive a
    # sync. Robocopy still updates every source file whose contents/timestamp
    # changed and creates newly added files/directories.
    $arguments = @(
        $Source,
        $destination,
        "/E",
        "/Z",
        "/FFT",
        # Do not overwrite a file edited more recently on the remote machine.
        # This makes the pull watcher safe to keep open while a developer
        # prepares an explicit push back to the host.
        "/XO",
        "/XJ",
        "/COPY:DAT",
        "/DCOPY:DAT",
        "/R:2",
        "/W:2",
        "/MT:8"
    )
    & robocopy.exe @arguments
    $result = $LASTEXITCODE
    if ($result -ge 8) {
        throw "Robocopy that bai (exit code $result)."
    }
    Write-Host "[$(Get-Date -Format 'HH:mm:ss')] Dong bo xong (Robocopy code $result)." -ForegroundColor Green
}

do {
    try {
        Invoke-P4SourceSync
    }
    catch {
        Write-Warning $_.Exception.Message
        if (-not $Watch) {
            exit 1
        }
    }

    if ($Watch) {
        Start-Sleep -Seconds $IntervalSeconds
    }
} while ($Watch)
