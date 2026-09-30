<#!
.SYNOPSIS
  Dependency-free Windows bridge for MKR P4RC frames.

This is the fallback for machines where Python installation is blocked by
organization policy. It reads the MKR USB CDC COM port and forwards the newest
P4RC line to the local Control Station at a bounded rate.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Port,
    [int]$Baud = 115200,
    [string]$Url = "http://127.0.0.1:8765"
)

$ErrorActionPreference = "Stop"
$baseUrl = $Url.TrimEnd('/')

# Windows PowerShell 5.1 does not always load System.Net.Http automatically.
try {
    Add-Type -AssemblyName System.Net.Http -ErrorAction Stop
}
catch {
    throw "Khong nap duoc .NET System.Net.Http: $($_.Exception.Message)"
}

function New-HttpClient {
    $handler = [System.Net.Http.HttpClientHandler]::new()
    $client = [System.Net.Http.HttpClient]::new($handler)
    $client.Timeout = [TimeSpan]::FromMilliseconds(750)
    return $client
}

function Get-CsrfToken {
    param([System.Net.Http.HttpClient]$Client)
    $text = $Client.GetStringAsync("$baseUrl/api/config").GetAwaiter().GetResult()
    $config = $text | ConvertFrom-Json
    if (-not $config.csrfToken) {
        throw "Control Station khong tra ve csrfToken."
    }
    return [string]$config.csrfToken
}

function Send-P4RcFrame {
    param(
        [System.Net.Http.HttpClient]$Client,
        [string]$CsrfToken,
        [string]$Line
    )
    $json = @{ line = $Line } | ConvertTo-Json -Compress
    $content = [System.Net.Http.StringContent]::new(
        $json,
        [System.Text.Encoding]::UTF8,
        "application/json"
    )
    $request = [System.Net.Http.HttpRequestMessage]::new(
        [System.Net.Http.HttpMethod]::Post,
        "$baseUrl/api/controller/external/frame"
    )
    $request.Content = $content
    $request.Headers.TryAddWithoutValidation("Origin", $baseUrl) | Out-Null
    $request.Headers.TryAddWithoutValidation("X-P4-Transwing-CSRF", $CsrfToken) | Out-Null
    try {
        $response = $Client.SendAsync($request).GetAwaiter().GetResult()
        $body = $response.Content.ReadAsStringAsync().GetAwaiter().GetResult()
        return [pscustomobject]@{
            Status = [int]$response.StatusCode
            Body = $body
        }
    }
    finally {
        $request.Dispose()
        $content.Dispose()
    }
}

$client = New-HttpClient
$serial = $null
try {
    $csrf = Get-CsrfToken -Client $client
    $serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
    $serial.ReadTimeout = 5
    $serial.NewLine = "`n"
    $serial.Handshake = [System.IO.Ports.Handshake]::None
    # Native USB CDC boards commonly start streaming only after DTR is asserted.
    $serial.DtrEnable = $true
    $serial.RtsEnable = $false
    $serial.Open()
    Start-Sleep -Milliseconds 1200
    Write-Host "Forwarding MKR P4RC from $Port to $baseUrl. Press Ctrl+C to stop."

    $latest = $null
    $rxBuffer = ""
    $lastSend = [DateTime]::UtcNow
    $lastReport = [DateTime]::UtcNow
    $ingested = 0
    $forwarded = 0
    $lastError = ""

    while ($true) {
        try {
            # ReadExisting is preferable to BytesToRead/ReadLine for an RDP
            # redirected COM port: the redirected driver can report zero bytes
            # until its internal buffer is polled.
            $chunk = $serial.ReadExisting()
            if ($chunk) {
                $rxBuffer += $chunk
                while ($rxBuffer.Contains("`n")) {
                    $newline = $rxBuffer.IndexOf("`n")
                    $line = $rxBuffer.Substring(0, $newline).Trim()
                    $rxBuffer = $rxBuffer.Substring($newline + 1)
                    if ($line.StartsWith("P4RC,")) {
                        $latest = $line
                        $ingested++
                    }
                }
            }
        }
        catch [System.IO.IOException] {
            throw "Doc COM that bai: $($_.Exception.Message)"
        }

        $now = [DateTime]::UtcNow
        if ($latest -and (($now - $lastSend).TotalMilliseconds -ge 100)) {
            $result = Send-P4RcFrame -Client $client -CsrfToken $csrf -Line $latest
            if ($result.Status -eq 403) {
                $csrf = Get-CsrfToken -Client $client
                $result = Send-P4RcFrame -Client $client -CsrfToken $csrf -Line $latest
            }
            if ($result.Status -ge 400) {
                $detail = $result.Body
                try { $detail = ($result.Body | ConvertFrom-Json).error } catch { }
                $message = "HTTP $($result.Status) ($detail)"
                if ($message -ne $lastError) {
                    Write-Warning "Control Station rejected frame: $message"
                    $lastError = $message
                }
            }
            else {
                $forwarded++
            }
            $lastSend = $now
        }

        if (($now - $lastReport).TotalSeconds -ge 2) {
            $seconds = ($now - $lastReport).TotalSeconds
            Write-Host ("P4RC USB ingest rate: {0:N1} Hz" -f ($ingested / $seconds))
            Write-Host ("P4RC forward rate: {0:N1} Hz" -f ($forwarded / $seconds))
            $ingested = 0
            $forwarded = 0
            $lastReport = $now
        }
        Start-Sleep -Milliseconds 2
    }
}
catch {
    Write-Error $_
    exit 1
}
finally {
    if ($serial) {
        if ($serial.IsOpen) { $serial.Close() }
        $serial.Dispose()
    }
    $client.Dispose()
}
