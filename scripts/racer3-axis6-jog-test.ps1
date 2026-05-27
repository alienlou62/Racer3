param(
    [string]$RepoRoot = "C:\Users\JP\racer3-rmp-basic-motion",
    [int]$ReadyTimeoutSeconds = 45,
    [double]$Velocity = 0.002,
    [int]$JogMilliseconds = 400
)

$ErrorActionPreference = "Stop"

function Write-Log {
    param([string]$Message)
    $timestamp = Get-Date -Format "yyyy-MM-dd HH:mm:ss.fff"
    $line = "$timestamp $Message"
    [Console]::WriteLine($line)
    [System.IO.File]::AppendAllText($script:LogPath, $line + [Environment]::NewLine)
}

Set-Location $RepoRoot
$env:PATH = "C:\RSI\11.0.5;$env:PATH"

$script:LogPath = Join-Path $RepoRoot "axis6-jog-test.log"
Remove-Item $script:LogPath -ErrorAction SilentlyContinue

$exe = Join-Path $RepoRoot "build-vs2022\Release\racer3-basic-motion.exe"
if (-not (Test-Path $exe)) {
    throw "Backend executable not found: $exe"
}

if ($Velocity -le 0 -or $Velocity -gt 0.010) {
    throw "Velocity must be > 0 and <= 0.010 user-units/sec for this safety test."
}

if ($JogMilliseconds -le 0 -or $JogMilliseconds -gt 1000) {
    throw "JogMilliseconds must be > 0 and <= 1000 for this safety test."
}

$script:Ready = New-Object System.Threading.ManualResetEventSlim($false)
$script:JogStarted = New-Object System.Threading.ManualResetEventSlim($false)
$script:JogStopped = New-Object System.Threading.ManualResetEventSlim($false)

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = $exe
$psi.Arguments = "--session-server"
$psi.WorkingDirectory = $RepoRoot
$psi.UseShellExecute = $false
$psi.RedirectStandardInput = $true
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true
$psi.CreateNoWindow = $true

$p = New-Object System.Diagnostics.Process
$p.StartInfo = $psi
$p.EnableRaisingEvents = $true

$outHandler = [System.Diagnostics.DataReceivedEventHandler]{
    param($sender, $eventArgs)
    if ($null -ne $eventArgs.Data) {
        Write-Log "[stdout] $($eventArgs.Data)"

        if ($eventArgs.Data.Contains('"type":"session_ready"')) {
            [void]$script:Ready.Set()
        }
        if ($eventArgs.Data.Contains('"type":"session_jog_velocity_started"')) {
            [void]$script:JogStarted.Set()
        }
        if ($eventArgs.Data.Contains('"type":"session_jog_velocity_stopped"')) {
            [void]$script:JogStopped.Set()
        }
    }
}

$errHandler = [System.Diagnostics.DataReceivedEventHandler]{
    param($sender, $eventArgs)
    if ($null -ne $eventArgs.Data) {
        Write-Log "[stderr] $($eventArgs.Data)"
    }
}

$p.add_OutputDataReceived($outHandler)
$p.add_ErrorDataReceived($errHandler)

try {
    Write-Log "Starting backend-owned Axis 6 jog test."
    Write-Log "Executable: $exe"
    Write-Log "Velocity=$Velocity user-units/sec; duration=$JogMilliseconds ms."
    Write-Log "Keep e-stop ready. Expected motion should be tiny."

    [void]$p.Start()
    $p.BeginOutputReadLine()
    $p.BeginErrorReadLine()

    $deadline = (Get-Date).AddSeconds($ReadyTimeoutSeconds)
    while (-not $script:Ready.IsSet) {
        if ($p.HasExited) {
            throw "Backend exited before session_ready. ExitCode=$($p.ExitCode)"
        }
        if ((Get-Date) -ge $deadline) {
            throw "Timed out waiting for session_ready after $ReadyTimeoutSeconds seconds."
        }
        Start-Sleep -Milliseconds 250
    }

    Write-Log "Session ready. Sending jog_velocity_start."
    $velocityText = $Velocity.ToString([System.Globalization.CultureInfo]::InvariantCulture)
    $startJson = '{"type":"jog_velocity_start","axis":6,"direction":"positive","velocity":' + $velocityText + '}'
    $p.StandardInput.WriteLine($startJson)
    $p.StandardInput.Flush()

    if (-not $script:JogStarted.Wait([TimeSpan]::FromSeconds(5))) {
        Write-Log "Did not observe session_jog_velocity_started within 5 seconds; sending stop anyway."
    }

    Start-Sleep -Milliseconds $JogMilliseconds

    Write-Log "Sending jog_velocity_stop."
    $p.StandardInput.WriteLine('{"type":"jog_velocity_stop"}')
    $p.StandardInput.Flush()

    if (-not $script:JogStopped.Wait([TimeSpan]::FromSeconds(5))) {
        Write-Log "Did not observe session_jog_velocity_stopped within 5 seconds."
    }

    Start-Sleep -Seconds 2

    Write-Log "Sending shutdown."
    $p.StandardInput.WriteLine('{"type":"shutdown"}')
    $p.StandardInput.Flush()

    if (-not $p.WaitForExit(10000)) {
        Write-Log "Backend did not exit after shutdown; killing process."
        $p.Kill()
        [void]$p.WaitForExit(5000)
    }

    Write-Log "Backend exit code: $($p.ExitCode)"
    exit $p.ExitCode
}
catch {
    Write-Log "SCRIPT ERROR: $($_.Exception.Message)"

    if ($null -ne $p -and -not $p.HasExited) {
        try {
            Write-Log "Sending jog_velocity_stop after error."
            $p.StandardInput.WriteLine('{"type":"jog_velocity_stop"}')
            $p.StandardInput.Flush()
        } catch {
            Write-Log "Stop write failed: $($_.Exception.Message)"
        }

        Start-Sleep -Milliseconds 500

        try {
            Write-Log "Attempting graceful shutdown after error."
            $p.StandardInput.WriteLine('{"type":"shutdown"}')
            $p.StandardInput.Flush()
        } catch {
            Write-Log "Shutdown write failed: $($_.Exception.Message)"
        }

        Start-Sleep -Milliseconds 500

        if (-not $p.HasExited) {
            Write-Log "Killing backend process after error."
            try { $p.Kill() } catch {}
        }
    }

    exit 1
}
finally {
    if ($null -ne $p) {
        try { $p.remove_OutputDataReceived($outHandler) } catch {}
        try { $p.remove_ErrorDataReceived($errHandler) } catch {}
        $p.Dispose()
    }
}
