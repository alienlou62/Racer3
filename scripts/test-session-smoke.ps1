$ErrorActionPreference = "Stop"

cd "C:\Users\JP\racer3-rmp-basic-motion"
$env:PATH = "C:\RSI\11.0.0;$env:PATH"

$log = "session-smoke-test.log"
Remove-Item $log -ErrorAction SilentlyContinue

$ready = [System.Threading.ManualResetEventSlim]::new($false)

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = ".\build-vs2022\Release\racer3-basic-motion.exe"
$psi.Arguments = "--session-server"
$psi.WorkingDirectory = "C:\Users\JP\racer3-rmp-basic-motion"
$psi.UseShellExecute = $false
$psi.RedirectStandardInput = $true
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true

$p = New-Object System.Diagnostics.Process
$p.StartInfo = $psi

$p.add_OutputDataReceived({
    param($sender, $eventArgs)
    if ($null -ne $eventArgs.Data) {
        $eventArgs.Data | Tee-Object -FilePath "session-smoke-test.log" -Append
        if ($eventArgs.Data.Contains('"type":"session_ready"')) {
            $ready.Set()
        }
    }
})

$p.add_ErrorDataReceived({
    param($sender, $eventArgs)
    if ($null -ne $eventArgs.Data) {
        "[stderr] $($eventArgs.Data)" | Tee-Object -FilePath "session-smoke-test.log" -Append
    }
})

try {
    "Starting session server smoke test..." | Tee-Object -FilePath $log -Append

    $p.Start() | Out-Null
    $p.BeginOutputReadLine()
    $p.BeginErrorReadLine()

    if (-not $ready.Wait([TimeSpan]::FromSeconds(45))) {
        throw "Session did not become ready in 45 seconds."
    }

    "Session became ready. Sending shutdown." | Tee-Object -FilePath $log -Append
    $p.StandardInput.WriteLine('{"type":"shutdown"}')
    $p.WaitForExit(10000) | Out-Null

    if (-not $p.HasExited) {
        "Backend did not exit after shutdown; killing process." | Tee-Object -FilePath $log -Append
        $p.Kill()
    }

    "Backend exit code: $($p.ExitCode)" | Tee-Object -FilePath $log -Append
}
catch {
    "SCRIPT ERROR: $($_.Exception.Message)" | Tee-Object -FilePath $log -Append

    if ($null -ne $p -and -not $p.HasExited) {
        try { $p.StandardInput.WriteLine('{"type":"shutdown"}') } catch {}
        Start-Sleep -Milliseconds 500
        if (-not $p.HasExited) {
            try { $p.Kill() } catch {}
        }
    }

    exit 1
}
