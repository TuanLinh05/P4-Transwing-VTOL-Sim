Set-StrictMode -Version Latest

function Get-P4WslDistros {
    $output = (& wsl.exe --list --quiet 2>$null) -replace "`0", ""
    return @($output | ForEach-Object { $_.Trim() } | Where-Object { $_ })
}

function Resolve-P4WslDistro {
    param([string]$Requested)

    $distros = @(Get-P4WslDistros)
    if ($distros.Count -eq 0) {
        throw "Khong tim thay WSL distro nao. Hay cai Ubuntu 22.04 hoac 24.04."
    }

    if ($Requested) {
        if ($distros -notcontains $Requested) {
            throw "Khong tim thay WSL distro '$Requested'. Da cai: $($distros -join ', ')."
        }
        return $Requested
    }

    foreach ($preferred in @("Ubuntu-22.04", "Ubuntu-24.04", "Ubuntu")) {
        if ($distros -contains $preferred) {
            return $preferred
        }
    }

    $ubuntu = @($distros | Where-Object { $_ -match "^Ubuntu" })
    if ($ubuntu.Count -eq 1) {
        return $ubuntu[0]
    }

    throw "Khong the tu chon WSL distro. Dat P4_TRANSWING_WSL_DISTRO. Da cai: $($distros -join ', ')."
}

function ConvertTo-P4WslPath {
    param(
        [Parameter(Mandatory = $true)][string]$Distro,
        [Parameter(Mandatory = $true)][string]$WindowsPath
    )

    # Do not pass a backslash-containing Windows path to `wslpath` through
    # wsl.exe. Some WSL/Windows combinations strip those backslashes before
    # the Linux command receives them (for example C:\\Users becomes C:Users).
    # Project files are intentionally supported from ordinary local drives;
    # translate that stable form here, then verify it in the selected distro.
    $fullPath = [System.IO.Path]::GetFullPath($WindowsPath)
    if ($fullPath -notmatch '^(?<drive>[A-Za-z]):\\(?<relative>.*)$') {
        throw "Duong dan project phai nam tren o dia Windows duoc WSL mount (/mnt/<drive>): $WindowsPath"
    }

    $drive = $Matches.drive.ToLowerInvariant()
    $relative = $Matches.relative -replace '\\', '/'
    $converted = "/mnt/$drive/$relative"

    & wsl.exe -d $Distro -- test -d $converted 2>$null
    if ($LASTEXITCODE -ne 0) {
        throw "WSL khong truy cap duoc thu muc project: $fullPath (thu duong dan WSL: $converted)"
    }
    return $converted
}

function Test-P4TcpPortAvailable {
    param([int]$Port)

    $listener = $null
    try {
        $listener = [System.Net.Sockets.TcpListener]::new(
            [System.Net.IPAddress]::Loopback,
            $Port
        )
        $listener.Start()
        return $true
    }
    catch {
        return $false
    }
    finally {
        if ($listener) {
            $listener.Stop()
        }
    }
}
