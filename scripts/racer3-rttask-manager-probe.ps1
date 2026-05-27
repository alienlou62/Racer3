param(
    [string]$RsiRoot = "C:\RSI\11.0.5",
    [string]$SettingsFile = "config\racer3-settings-rttask-probe.xml",
    [string]$RapidServerAddress = "http://127.0.0.1:51061",
    [string]$ManagerLabel = "Racer3JogProbe",
    [string]$RsiProtoPath = "",
    [string]$PrebuiltRslPath = "",
    [string]$PrebuiltDllPath = "",
    [string]$RequiredAdvancingGlobal = "",
    [string]$VsInstallPath = "",
    [string]$IntimeRoot = "",
    [int]$FeedbackIterations = 5,
    [int]$FeedbackDelayMs = 500,
    [switch]$BuildRtTasks,
    [switch]$ConfigureIntimeBuild,
    [switch]$CleanRtTaskBuild,
    [switch]$CleanRuntime,
    [switch]$StartRapidServer,
    [switch]$ReadFeedback,
    [switch]$ExerciseJogIntentReadback,
    [switch]$RequireStatusReadiness,
    [switch]$PrepareEndpointMotionForRtTaskJog,
    [switch]$ArmRtTaskJogMotion,
    [int]$JogAxis = 6,
    [double]$JogStepUserUnits = 0.0002,
    [double]$JogSpeedUserUnitsPerSecond = 0.002,
    [double]$JogStepMeters = [double]::NaN,
    [double]$JogSpeedMetersPerSecond = [double]::NaN,
    [switch]$UseInstalledSampleRtTaskFunctions,
    [switch]$NoIntimeMsBuildTargetsOverlay,
    [switch]$NoSessionServer
)

$ErrorActionPreference = "Stop"

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $repoRoot

$env:PATH = "$RsiRoot;$env:PATH"
if ([string]::IsNullOrWhiteSpace($IntimeRoot)) {
    if (![string]::IsNullOrWhiteSpace($env:INTIME)) {
        $IntimeRoot = $env:INTIME
    }
    else {
        $IntimeRoot = "C:\Program Files (x86)\INtime"
    }
}
$IntimeRoot = $IntimeRoot.TrimEnd('\')

if ($UseInstalledSampleRtTaskFunctions) {
    Write-Host "Installed RSI sample RTTaskFunctions mode requested. This mode is no-motion and skips the custom Racer3 RTTaskFunctions build/deploy path."
    $SettingsFile = "config\racer3-settings-rttask-installed-sample.xml"
    $ManagerLabel = "Racer3InstalledSampleProbe"
    $RequiredAdvancingGlobal = "counter"
    $PrebuiltRslPath = ""
    $PrebuiltDllPath = ""
    $BuildRtTasks = $false
    $ConfigureIntimeBuild = $false
    $CleanRtTaskBuild = $false
    $ExerciseJogIntentReadback = $false
    $RequireStatusReadiness = $false
    $PrepareEndpointMotionForRtTaskJog = $false
    $ArmRtTaskJogMotion = $false
}

if (![double]::IsNaN($JogStepMeters)) {
    Write-Warning "-JogStepMeters is accepted as a compatibility alias for this first direct-axis proof, but the RTTask Axis::MoveRelative command uses axis user-units. Interpreting $JogStepMeters as user-units."
    $JogStepUserUnits = $JogStepMeters
}

if (![double]::IsNaN($JogSpeedMetersPerSecond)) {
    Write-Warning "-JogSpeedMetersPerSecond is accepted as a compatibility alias for this first direct-axis proof, but the RTTask Axis::MoveRelative command uses axis user-units/sec. Interpreting $JogSpeedMetersPerSecond as user-units/sec."
    $JogSpeedUserUnitsPerSecond = $JogSpeedMetersPerSecond
}

if ($ArmRtTaskJogMotion) {
    $ExerciseJogIntentReadback = $true
    $RequireStatusReadiness = $true

    $maxJogStepUserUnits = 0.001
    $maxJogSpeedUserUnitsPerSecond = 0.005
    if ($JogAxis -lt 1 -or $JogAxis -gt 6) {
        throw "-JogAxis must be in operator-facing range 1..6. Got $JogAxis."
    }
    if ($JogStepUserUnits -eq 0.0 -or [Math]::Abs($JogStepUserUnits) -gt $maxJogStepUserUnits) {
        throw "-JogStepUserUnits must be nonzero and <= $maxJogStepUserUnits in magnitude. Got $JogStepUserUnits."
    }
    if ($JogSpeedUserUnitsPerSecond -le 0.0 -or $JogSpeedUserUnitsPerSecond -gt $maxJogSpeedUserUnitsPerSecond) {
        throw "-JogSpeedUserUnitsPerSecond must be > 0 and <= $maxJogSpeedUserUnitsPerSecond. Got $JogSpeedUserUnitsPerSecond."
    }
}

function Write-Step([string]$message) {
    Write-Host ""
    Write-Host "==> $message"
}

function Invoke-Native {
    param(
        [string]$FilePath,
        [string[]]$Arguments
    )

    Write-Host "Command: $FilePath $($Arguments -join ' ')"
    & $FilePath @Arguments
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw ("Command failed with exit code {0}: {1} {2}" -f $exitCode, $FilePath, ($Arguments -join ' '))
    }
}

function Resolve-Tool([string]$name) {
    $cmd = Get-Command $name -ErrorAction SilentlyContinue
    if ($null -eq $cmd) {
        throw "Required tool '$name' was not found on PATH. Current PATH starts with: $($env:PATH.Split(';')[0])"
    }
    return $cmd.Source
}

function Get-RapidServerUri {
    try {
        $uri = [System.Uri]$RapidServerAddress
    }
    catch {
        throw "RapidServerAddress must be a valid URI such as http://127.0.0.1:51061. Current value: $RapidServerAddress"
    }

    if ($uri.Port -lt 49152) {
        throw "rapidserver requires a gRPC port >= 49152. Current RapidServerAddress uses port $($uri.Port): $RapidServerAddress"
    }

    return $uri
}

function Test-TcpConnect {
    param(
        [string]$HostName,
        [int]$Port,
        [int]$TimeoutMs = 1000
    )

    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $async = $client.BeginConnect($HostName, $Port, $null, $null)
        if (!$async.AsyncWaitHandle.WaitOne($TimeoutMs, $false)) {
            return $false
        }
        $client.EndConnect($async)
        return $true
    }
    catch {
        return $false
    }
    finally {
        $client.Close()
    }
}

function Wait-RapidServerListening {
    param(
        [string]$HostName,
        [int]$Port,
        [int]$TimeoutSeconds = 15
    )

    $deadline = (Get-Date).AddSeconds($TimeoutSeconds)
    do {
        if (Test-TcpConnect -HostName $HostName -Port $Port -TimeoutMs 500) {
            return $true
        }
        Start-Sleep -Milliseconds 500
    } while ((Get-Date) -lt $deadline)

    return $false
}

function Write-FileTailIfPresent {
    param(
        [string]$Path,
        [int]$Tail = 80
    )

    if (Test-Path -LiteralPath $Path) {
        Write-Host ""
        Write-Host "--- Tail: $Path ---"
        Get-Content -LiteralPath $Path -Tail $Tail
    }
}

function Stop-IfRunning([string]$processName) {
    $processes = Get-Process -Name $processName -ErrorAction SilentlyContinue
    foreach ($process in $processes) {
        Write-Host "Stopping stale process $($process.ProcessName) pid=$($process.Id)"
        Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
    }

    if ($processes) {
        $deadline = (Get-Date).AddSeconds(10)
        do {
            Start-Sleep -Milliseconds 200
            $remaining = Get-Process -Name $processName -ErrorAction SilentlyContinue
            if (!$remaining) {
                return
            }
        } while ((Get-Date) -lt $deadline)

        $remainingIds = (Get-Process -Name $processName -ErrorAction SilentlyContinue | ForEach-Object { $_.Id }) -join ', '
        throw "Timed out waiting for stale process '$processName' to exit. Remaining pids: $remainingIds"
    }
}


function Start-EndpointMotionPrepSession {
    param(
        [int]$ReadyTimeoutSeconds = 60
    )

    $exe = Join-Path $repoRoot "build-vs2022\Release\racer3-basic-motion.exe"
    if (!(Test-Path -LiteralPath $exe)) {
        throw "Endpoint prep requested, but backend executable was not found: $exe. Build the Release backend first."
    }

    $stdout = Join-Path $repoRoot "rttask-endpoint-prearm-session.log"
    $stderr = Join-Path $repoRoot "rttask-endpoint-prearm-session.err.log"
    Remove-Item -LiteralPath $stdout -Force -ErrorAction SilentlyContinue
    Remove-Item -LiteralPath $stderr -Force -ErrorAction SilentlyContinue

    $args = @(
        "--prearm-hold",
        "--velocity", "0.002",
        "--diagnostics",
        "--prearm-hold-seconds", "30"
    )

    Write-Host "Command: $exe $($args -join ' ')"
    Write-Host "This starts the proven bottom-to-top endpoint pre-arm path, polls for prearm_ready, then leaves amps enabled during the RTTask proof."

    $process = Start-Process `
        -FilePath $exe `
        -ArgumentList $args `
        -WorkingDirectory $repoRoot `
        -RedirectStandardOutput $stdout `
        -RedirectStandardError $stderr `
        -PassThru `
        -WindowStyle Hidden

    $deadline = (Get-Date).AddSeconds($ReadyTimeoutSeconds)

    while ((Get-Date) -lt $deadline) {
        $process.Refresh()

        if ($process.HasExited) {
            Write-FileTailIfPresent -Path $stdout
            Write-FileTailIfPresent -Path $stderr
            throw "Endpoint pre-arm hold exited before prearm_ready. ExitCode=$($process.ExitCode)"
        }

        if (Test-Path -LiteralPath $stdout) {
            $outText = Get-Content -LiteralPath $stdout -Raw -ErrorAction SilentlyContinue
            if ($outText -match '"type":"prearm_ready"') {
                Write-Host "Endpoint pre-arm hold reached prearm_ready. Amps should remain enabled while RTTask feedback/jog proof runs."
                Write-Host "Logs:"
                Write-Host "  $stdout"
                Write-Host "  $stderr"

                return [pscustomobject]@{
                    Process = $process
                    StdOutLog = $stdout
                    StdErrLog = $stderr
                    TimedHold = $true
                }
            }
        }

        Start-Sleep -Milliseconds 500
    }

    Write-FileTailIfPresent -Path $stdout
    Write-FileTailIfPresent -Path $stderr

    try {
        $process.Refresh()
        if (-not $process.HasExited) {
            Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
        }
    }
    catch {
    }

    throw "Timed out waiting for endpoint pre-arm prearm_ready after $ReadyTimeoutSeconds seconds."
}

function Stop-EndpointMotionPrepSession {
    param($State)

    if ($null -eq $State -or $null -eq $State.Process) {
        return
    }

    $process = $State.Process

    try {
        $process.Refresh()
        if (-not $process.HasExited) {
            Write-Host "Waiting for timed endpoint pre-arm hold to shut down cleanly and disable amps."
            if (-not $process.WaitForExit(35000)) {
                Write-Warning "Endpoint pre-arm timed hold did not exit cleanly; killing process."
                Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
            }
        }
    }
    catch {
        Write-Warning "Endpoint pre-arm cleanup failed: $($_.Exception.Message)"
        try {
            $process.Refresh()
            if (-not $process.HasExited) {
                Stop-Process -Id $process.Id -Force -ErrorAction SilentlyContinue
            }
        }
        catch {
        }
    }
    finally {
        try { $process.Dispose() } catch {}
        Write-FileTailIfPresent -Path $State.StdOutLog
        Write-FileTailIfPresent -Path $State.StdErrLog
    }
}

function Resolve-VisualStudioInstallPath {
    if (![string]::IsNullOrWhiteSpace($VsInstallPath)) {
        $resolved = Resolve-Path -LiteralPath $VsInstallPath -ErrorAction Stop
        return $resolved.Path
    }

    $knownInstallPaths = @(
        (Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\2022\BuildTools"),
        (Join-Path $env:ProgramFiles "Microsoft Visual Studio\2022\Community"),
        (Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\2022\Community"),
        (Join-Path $env:ProgramFiles "Microsoft Visual Studio\2022\Professional"),
        (Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\2022\Professional")
    )
    foreach ($candidate in $knownInstallPaths) {
        if (Test-Path -LiteralPath (Join-Path $candidate "MSBuild\Current\Bin\MSBuild.exe")) {
            return $candidate
        }
    }

    $vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path -LiteralPath $vswhere) {
        $installPath = & $vswhere -latest -products * -requires Microsoft.Component.MSBuild -property installationPath
        if (![string]::IsNullOrWhiteSpace($installPath)) {
            return $installPath
        }
    }

    throw "Could not locate a Visual Studio 2022 installation with MSBuild. Pass -VsInstallPath if Visual Studio is installed in a custom location."
}

function Resolve-MSBuild {
    $msbuildCommand = Get-Command msbuild.exe -ErrorAction SilentlyContinue
    if ($null -ne $msbuildCommand) {
        return $msbuildCommand.Source
    }

    $installPath = Resolve-VisualStudioInstallPath
    $candidate = Join-Path $installPath "MSBuild\Current\Bin\MSBuild.exe"
    if (Test-Path -LiteralPath $candidate) {
        return $candidate
    }

    throw "Could not locate MSBuild.exe."
}

function Resolve-VsVCTargetsPath {
    $installPath = Resolve-VisualStudioInstallPath
    $candidate = Join-Path $installPath "MSBuild\Microsoft\VC\v170"
    if (Test-Path -LiteralPath (Join-Path $candidate "Microsoft.Cpp.Default.props")) {
        return $candidate
    }

    throw "Could not locate Visual Studio C++ v170 MSBuild targets under $installPath."
}

function Resolve-IntimeVsPlatformPath {
    $candidate = Join-Path $IntimeRoot "vstudio170\platforms\intime"
    if (Test-Path -LiteralPath (Join-Path $candidate "platform.default.props")) {
        return $candidate
    }

    throw "Could not locate INtime Visual Studio 2022 platform files under $candidate. Check -IntimeRoot."
}

function Test-VsV143BuildTools {
    param(
        [string]$VCTargetsPath
    )

    $toolsetProps = Join-Path $VCTargetsPath "Platforms\Win32\PlatformToolsets\v143\Toolset.props"
    $vsInstallPath = Resolve-VisualStudioInstallPath
    $clCompiler = Get-ChildItem -LiteralPath (Join-Path $vsInstallPath "VC\Tools\MSVC") -Recurse -Filter "cl.exe" -ErrorAction SilentlyContinue |
        Where-Object { $_.FullName -match "\\bin\\Hostx64\\x86\\cl.exe$" } |
        Select-Object -First 1

    return ((Test-Path -LiteralPath $toolsetProps) -and ($null -ne $clCompiler))
}

function Test-VsIntimePlatformIntegrated {
    param(
        [string]$VCTargetsPath
    )

    return (Test-Path -LiteralPath (Join-Path $VCTargetsPath "Platforms\INtime\PlatformToolsets\v143\toolset.props"))
}

function Write-VsIntimeToolchainDiagnostics {
    param(
        [string]$VCTargetsPath
    )

    $intimePlatformPath = Resolve-IntimeVsPlatformPath
    Write-Host "Visual Studio C++ targets: $VCTargetsPath"
    Write-Host "VS v143 Win32/x64 C++ tools present: $(Test-VsV143BuildTools -VCTargetsPath $VCTargetsPath)"
    Write-Host "INtime platform integrated into VS targets: $(Test-VsIntimePlatformIntegrated -VCTargetsPath $VCTargetsPath)"
    Write-Host "INtime VS2022 platform source: $intimePlatformPath"
}

function Get-MissingIntimeHeaders {
    $includeRoot = Join-Path $IntimeRoot "rt\include"
    $requiredHeaders = @("rtwin32.h")
    $missing = @()

    foreach ($header in $requiredHeaders) {
        if (!(Test-Path -LiteralPath (Join-Path $includeRoot $header))) {
            $missing += $header
        }
    }

    return $missing
}

function Test-IntimeCDevHeaders {
    $missing = @(Get-MissingIntimeHeaders)
    if ($missing.Count -eq 0) {
        Write-Host "INtime CDEV headers present in $(Join-Path $IntimeRoot 'rt\include')."
        return $true
    }

    Write-Warning "INtime CDEV headers are incomplete under $(Join-Path $IntimeRoot 'rt\include'): missing $($missing -join ', ')."
    Write-Warning "This is not an INtime/NodeA missing condition. The MSBuild INtime platform/toolset path can be resolved, but the compile cannot produce a custom .rsl until the installed CDEV headers are complete."
    Write-Warning "RSI notes that INtime CDEV 7.1.24270.1 omitted rtwin32.h and cannot build Real-Time Task projects. Install CDEV 7.1.25030.3 or newer from the RSI customer portal, then rerun this probe."
    return $false
}

function New-IntimeMsBuildTargetsOverlay {
    $vsTargetsPath = Resolve-VsVCTargetsPath
    $intimePlatformPath = Resolve-IntimeVsPlatformPath
    Write-VsIntimeToolchainDiagnostics -VCTargetsPath $vsTargetsPath

    if (Test-VsIntimePlatformIntegrated -VCTargetsPath $vsTargetsPath) {
        Write-Host "Visual Studio already exposes Platform=INtime with PlatformToolset=v143; no local MSBuild targets overlay is needed."
        return ($vsTargetsPath.TrimEnd('\') + '\')
    }

    $buildRoot = Join-Path $repoRoot "build-vs2022-rttask"
    $overlayRoot = Join-Path $buildRoot "msbuild-vctargets-overlay"
    $overlayFullPath = [System.IO.Path]::GetFullPath($overlayRoot)
    $buildFullPath = [System.IO.Path]::GetFullPath($buildRoot)

    if (!$overlayFullPath.StartsWith($buildFullPath, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "Refusing to recreate MSBuild targets overlay outside the RTTask build directory: $overlayFullPath"
    }

    if (Test-Path -LiteralPath $overlayFullPath) {
        Remove-Item -LiteralPath $overlayFullPath -Recurse -Force
    }

    New-Item -ItemType Directory -Path $overlayFullPath -Force | Out-Null
    Get-ChildItem -LiteralPath $vsTargetsPath -Force | Copy-Item -Destination $overlayFullPath -Recurse -Force

    $platformsDir = Join-Path $overlayFullPath "Platforms"
    $intimeDestination = Join-Path $platformsDir "INtime"
    New-Item -ItemType Directory -Path $intimeDestination -Force | Out-Null
    Get-ChildItem -LiteralPath $intimePlatformPath -Force | Copy-Item -Destination $intimeDestination -Recurse -Force

    $overlayWithSlash = $overlayFullPath.TrimEnd('\') + '\'
    Write-Host "Created local MSBuild VCTargets overlay:"
    Write-Host "  VS targets:      $vsTargetsPath"
    Write-Host "  INtime platform: $intimePlatformPath"
    Write-Host "  Overlay:         $overlayWithSlash"
    return $overlayWithSlash
}

function Write-IntimeBuildDiagnostics {
    param(
        [string]$VCTargetsPath = ""
    )

    Write-Step "INtime build diagnostics"

    $project = Join-Path $repoRoot "build-vs2022-rttask\INtime\RTTaskFunctions.vcxproj"
    if (Test-Path -LiteralPath $project) {
        Write-Host "Generated project: $project"
        Get-Content -LiteralPath $project |
            Select-String -Pattern "PlatformToolset|OutDir|TargetExt|ConfigurationType|RTTaskFunctions|Import" |
            ForEach-Object { Write-Host "  $($_.Line.Trim())" }
    }
    else {
        Write-Host "Generated project is not present yet: $project"
    }

    if (![string]::IsNullOrWhiteSpace($VCTargetsPath)) {
        Write-Host "MSBuild VCTargetsPath override: $VCTargetsPath"
        Write-Host "Overlay INtime platform present: $(Test-Path -LiteralPath (Join-Path $VCTargetsPath 'Platforms\INtime\platform.default.props'))"
    }

    $missingHeaders = @(Get-MissingIntimeHeaders)
    if ($missingHeaders.Count -gt 0) {
        Write-Host "Missing INtime CDEV headers: $($missingHeaders -join ', ')"
    }
}

function Backup-InstalledRtTaskFunctionsArtifact {
    param(
        [string]$Extension,
        [string]$Reason = "custom Racer3 RTTaskFunctions deployment"
    )

    $installedArtifact = Join-Path $RsiRoot "RTTaskFunctions.$Extension"
    if (!(Test-Path -LiteralPath $installedArtifact)) {
        return
    }

    $backupDir = Join-Path $repoRoot "artifacts\rttask-backups"
    New-Item -ItemType Directory -Path $backupDir -Force | Out-Null
    $timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
    $backupPath = Join-Path $backupDir "RTTaskFunctions.installed-sample.$timestamp.$Extension"
    Copy-Item -LiteralPath $installedArtifact -Destination $backupPath -Force
    Write-Host "Backed up existing RTTaskFunctions.$Extension before ${Reason}:"
    Write-Host "  $backupPath"
}

function Backup-InstalledRtTaskFunctionsRsl {
    param(
        [string]$Reason = "custom Racer3 RTTaskFunctions deployment"
    )

    Backup-InstalledRtTaskFunctionsArtifact -Extension "rsl" -Reason $Reason
}

function Backup-InstalledRtTaskFunctionsDll {
    param(
        [string]$Reason = "custom Racer3 RTTaskFunctions metadata deployment"
    )

    Backup-InstalledRtTaskFunctionsArtifact -Extension "dll" -Reason $Reason
}

function Test-RtTaskArtifactContainsText {
    param(
        [string]$Path,
        [string]$Text
    )

    if (!(Test-Path -LiteralPath $Path)) {
        return $false
    }

    $bytes = [System.IO.File]::ReadAllBytes($Path)
    $ascii = [System.Text.Encoding]::ASCII.GetString($bytes)
    return $ascii.Contains($Text)
}

function Find-InstalledSampleRtTaskFunctionsBackup {
    param(
        [string]$Extension
    )

    $backupDir = Join-Path $repoRoot "artifacts\rttask-backups"
    if (!(Test-Path -LiteralPath $backupDir)) {
        return $null
    }

    $candidates = Get-ChildItem -LiteralPath $backupDir -Filter "RTTaskFunctions.installed-sample.*.$Extension" -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending

    foreach ($candidate in $candidates) {
        if ((Test-RtTaskArtifactContainsText -Path $candidate.FullName -Text "FollowSensor") -and
            !(Test-RtTaskArtifactContainsText -Path $candidate.FullName -Text "Racer3BasicHeartbeat")) {
            return $candidate.FullName
        }
    }

    return $null
}

function Restore-InstalledSampleRtTaskFunctionsArtifactIfNeeded {
    param(
        [string]$Extension
    )

    $runtimeArtifact = Join-Path $RsiRoot "RTTaskFunctions.$Extension"
    $hasInstalledSampleMetadata = (Test-RtTaskArtifactContainsText -Path $runtimeArtifact -Text "FollowSensor") -and
        !(Test-RtTaskArtifactContainsText -Path $runtimeArtifact -Text "Racer3BasicHeartbeat")

    if ($hasInstalledSampleMetadata) {
        return
    }

    $backup = Find-InstalledSampleRtTaskFunctionsBackup -Extension $Extension
    if ([string]::IsNullOrWhiteSpace($backup)) {
        throw "Installed-sample mode needs a stock RTTaskFunctions.$Extension, but $runtimeArtifact is missing or appears to contain custom Racer3 metadata and no stock backup was found in artifacts\rttask-backups."
    }

    $backupDir = Join-Path $repoRoot "artifacts\rttask-backups"
    New-Item -ItemType Directory -Path $backupDir -Force | Out-Null
    if (Test-Path -LiteralPath $runtimeArtifact) {
        $timestamp = Get-Date -Format "yyyyMMdd-HHmmss"
        $modeSwitchBackup = Join-Path $backupDir "RTTaskFunctions.before-installed-sample-restore.$timestamp.$Extension"
        Copy-Item -LiteralPath $runtimeArtifact -Destination $modeSwitchBackup -Force
        Write-Host "Backed up current RTTaskFunctions.$Extension before restoring installed-sample artifact:"
        Write-Host "  $modeSwitchBackup"
    }

    Copy-Item -LiteralPath $backup -Destination $runtimeArtifact -Force
    Write-Host "Restored installed-sample RTTaskFunctions.$Extension from:"
    Write-Host "  $backup"
}

function Write-RtTaskFunctionsArtifactInventory {
    Write-Host ""
    Write-Host "RTTaskFunctions artifacts in ${RsiRoot}:"
    foreach ($extension in @("rsl", "dll", "pdb", "lib")) {
        $artifact = Join-Path $RsiRoot "RTTaskFunctions.$extension"
        if (Test-Path -LiteralPath $artifact) {
            Get-Item -LiteralPath $artifact |
                Select-Object FullName, Length, LastWriteTime |
                Format-Table -AutoSize
        }
        else {
            Write-Host "  <missing> $artifact"
        }
    }
}

function Clear-RtTaskLogs {
    $logs = @(
        (Join-Path $env:USERPROFILE "rttaskmanager.log"),
        (Join-Path $env:USERPROFILE "rttaskapi.log"),
        (Join-Path $repoRoot "rttaskmanager.log"),
        (Join-Path $repoRoot "rttaskapi.log"),
        (Join-Path $repoRoot "rapidserver-rttask-probe.log"),
        (Join-Path $repoRoot "rapidserver-rttask-probe.err.log"),
        (Join-Path $repoRoot "rttask-manager-feedback.log")
    )

    foreach ($log in $logs) {
        Remove-Item -LiteralPath $log -Force -ErrorAction SilentlyContinue
    }
}

function Invoke-DirectIntimeBuildFallback {
    param(
        [string]$Configuration = "Release",
        [string]$VCTargetsPath = ""
    )

    $project = Join-Path $repoRoot "build-vs2022-rttask\INtime\RTTaskFunctions.vcxproj"
    if (!(Test-Path -LiteralPath $project)) {
        Write-Warning "Direct INtime fallback skipped: $project does not exist."
        return
    }

    $msbuild = Resolve-MSBuild
    Write-Host "Direct INtime fallback build: $project"
    Write-Host "This bypasses CMake ZERO_CHECK project references and mirrors the laser-demo assumption that the RTTask library artifact is the only build product needed for rsiconfig."

    $arguments = @(
        $project,
        "/p:Configuration=$Configuration",
        "/p:Platform=INtime",
        "/p:BuildProjectReferences=false",
        "/m"
    )
    if (![string]::IsNullOrWhiteSpace($VCTargetsPath)) {
        $arguments += "/p:VCTargetsPath=$VCTargetsPath"
    }

    Invoke-Native $msbuild $arguments
}

function Invoke-DirectHostMetadataBuild {
    param(
        [string]$Configuration = "Release"
    )

    $project = Join-Path $repoRoot "build-vs2022-rttask\RTTaskFunctions.vcxproj"
    if (!(Test-Path -LiteralPath $project)) {
        throw "Host RTTaskFunctions project was not generated: $project"
    }

    $msbuild = Resolve-MSBuild
    Write-Host "Host metadata DLL build: $project"
    Write-Host "This builds the Windows RTTaskFunctions.dll companion used by RapidCodeRemote/global metadata discovery. The INtime RTTaskManager still executes RTTaskFunctions.rsl."

    Invoke-Native $msbuild @(
        $project,
        "/p:Configuration=$Configuration",
        "/p:Platform=x64",
        "/p:BuildProjectReferences=false",
        "/m"
    )
}

Write-Step "Racer3 laser-style RTTaskManager bring-up / feedback probe"
Write-Host "Repo root:          $repoRoot"
Write-Host "RSI root:           $RsiRoot"
Write-Host "Settings:           $SettingsFile"
Write-Host "RapidServer:        $RapidServerAddress"
Write-Host "Manager label:      $ManagerLabel"
Write-Host "INtime root:        $IntimeRoot"
Write-Host "Installed sample:  $UseInstalledSampleRtTaskFunctions"
Write-Host "Required global:   $RequiredAdvancingGlobal"
Write-Host "Jog readback test: $ExerciseJogIntentReadback"
Write-Host "Status readiness:  $RequireStatusReadiness"
Write-Host "Endpoint pre-arm: $PrepareEndpointMotionForRtTaskJog"
Write-Host "Armed RTTask jog:  $ArmRtTaskJogMotion"
if ($PrepareEndpointMotionForRtTaskJog) {
    Write-Warning "Endpoint pre-arm requested. This starts the proven bottom-to-top all-axis pre-arm hold path to clear faults and enable amps before RTTask jog proof. Keep e-stop ready."
}
if ($ArmRtTaskJogMotion) {
    Write-Warning "ARMED RTTask jog proof requested. This sends one tiny Axis::MoveRelative intent only after RTTask readiness gates pass. Keep e-stop ready."
    Write-Host "  JogAxis(operator-facing): $JogAxis"
    Write-Host "  JogStepUserUnits:        $JogStepUserUnits"
    Write-Host "  JogSpeedUserUnits/sec:   $JogSpeedUserUnitsPerSecond"
}
Write-Host "This mirrors the working laser demo pipeline: build/deploy RTTask library, start rapidserver, run rsiconfig settings XML, then read manager/task/global feedback through RapidCodeRemote."
Write-Host "It does not replace config\racer3-settings.xml and it does not command robot motion."

if ($CleanRuntime) {
    Write-Step "Cleaning stale RSI/RMP process state and RTTask logs"
    Stop-IfRunning "racer3-basic-motion"
    Stop-IfRunning "rapidserver"
    Stop-IfRunning "rsiconfig"
    Stop-IfRunning "RTTaskManager"
    Stop-IfRunning "rttaskmanager"
    Clear-RtTaskLogs
}

$intimeVCTargetsPath = ""

if ($UseInstalledSampleRtTaskFunctions) {
    Write-Step "Verifying installed RSI sample RTTaskFunctions.rsl"
    foreach ($extension in @("rsl", "dll")) {
        Restore-InstalledSampleRtTaskFunctionsArtifactIfNeeded -Extension $extension
    }
    Write-RtTaskFunctionsArtifactInventory
}

if (![string]::IsNullOrWhiteSpace($PrebuiltRslPath)) {
    Write-Step "Deploying prebuilt custom RTTaskFunctions.rsl"
    $resolvedPrebuiltRsl = Resolve-Path -LiteralPath $PrebuiltRslPath -ErrorAction Stop
    $rslDestination = Join-Path $RsiRoot "RTTaskFunctions.rsl"
    Backup-InstalledRtTaskFunctionsRsl -Reason "prebuilt custom library deployment"
    Copy-Item -LiteralPath $resolvedPrebuiltRsl.Path -Destination $rslDestination -Force
    Write-Host "Copied $($resolvedPrebuiltRsl.Path) to $rslDestination"
}

if (![string]::IsNullOrWhiteSpace($PrebuiltDllPath)) {
    Write-Step "Deploying prebuilt custom RTTaskFunctions.dll host metadata companion"
    $resolvedPrebuiltDll = Resolve-Path -LiteralPath $PrebuiltDllPath -ErrorAction Stop
    $dllDestination = Join-Path $RsiRoot "RTTaskFunctions.dll"
    Backup-InstalledRtTaskFunctionsDll -Reason "prebuilt custom metadata DLL deployment"
    Copy-Item -LiteralPath $resolvedPrebuiltDll.Path -Destination $dllDestination -Force
    Write-Host "Copied $($resolvedPrebuiltDll.Path) to $dllDestination"
}

if ($BuildRtTasks) {
    Write-Step "Building/deploying custom Racer3 RTTaskFunctions for RTTaskManager"
    Backup-InstalledRtTaskFunctionsRsl -Reason "custom build"
    Backup-InstalledRtTaskFunctionsDll -Reason "custom host metadata build"

    if ($CleanRtTaskBuild) {
        Write-Host "Removing build-vs2022-rttask so the INtime child configure cannot reuse stale cache/options."
        Remove-Item -LiteralPath "build-vs2022-rttask" -Recurse -Force -ErrorAction SilentlyContinue
    }

    if ($ConfigureIntimeBuild) {
        if (!$NoIntimeMsBuildTargetsOverlay) {
            $intimeVCTargetsPath = New-IntimeMsBuildTargetsOverlay
        }

        if (![string]::IsNullOrWhiteSpace($PrebuiltRslPath)) {
            Write-Host "Prebuilt .rsl was provided. Skipping INtime CDEV header preflight for manual/prebuilt library deployment."
        }
        elseif (!(Test-IntimeCDevHeaders)) {
            Write-IntimeBuildDiagnostics -VCTargetsPath $intimeVCTargetsPath
            throw "INtime CDEV headers are incomplete, so RTTaskFunctions.rsl cannot be built on this machine yet. Install the updated INtime CDEV package, or pass -PrebuiltRslPath with a known-good RTTaskFunctions.rsl."
        }
    }

    $configureArgs = @(
        "-S", ".",
        "-B", "build-vs2022-rttask",
        "-G", "Visual Studio 17 2022",
        "-A", "x64",
        "-DRSI_SDK_ROOT=$($RsiRoot -replace '\\','/')",
        "-DRACER3_BUILD_RTTASKS=ON",
        "-DRACER3_RTTASK_RUNTIME_DIR=$($RsiRoot -replace '\\','/')"
    )

    if ($ConfigureIntimeBuild) {
        $configureArgs += "-DRACER3_CONFIGURE_INTIME_RTTASKS=ON"
        Write-Host "INtime companion build requested. NodeA is assumed to already be configured on this PC."
        Write-Host "The INtime child build is used only to produce RTTaskFunctions.rsl for the settings-XML RTTaskManager path."
    }
    if (![string]::IsNullOrWhiteSpace($intimeVCTargetsPath)) {
        $configureArgs += "-DRACER3_INTIME_VCTARGETS_PATH=$($intimeVCTargetsPath -replace '\\','/')"
    }

    Invoke-Native "cmake" $configureArgs

    $previousVCTargetsPath = $env:VCTargetsPath
    try {
        if (![string]::IsNullOrWhiteSpace($intimeVCTargetsPath)) {
            $env:VCTargetsPath = $intimeVCTargetsPath
        }

        Invoke-Native "cmake" @("--build", "build-vs2022-rttask", "--config", "Release", "--target", "RTTaskFunctions")
    }
    catch {
        Write-Warning "CMake RTTask build failed: $($_.Exception.Message)"
        Write-IntimeBuildDiagnostics -VCTargetsPath $intimeVCTargetsPath
        if ($ConfigureIntimeBuild) {
            Invoke-DirectIntimeBuildFallback -Configuration "Release" -VCTargetsPath $intimeVCTargetsPath
        }
        else {
            throw
        }
    }
    finally {
        $env:VCTargetsPath = $previousVCTargetsPath
    }

    Invoke-DirectHostMetadataBuild -Configuration "Release"

    if ($ConfigureIntimeBuild -and -not (Test-Path -LiteralPath (Join-Path $RsiRoot "RTTaskFunctions.rsl"))) {
        Write-Warning "RTTaskFunctions.rsl was not found after CMake build. Trying direct INtime project fallback once."
        Invoke-DirectIntimeBuildFallback -Configuration "Release" -VCTargetsPath $intimeVCTargetsPath
    }

    Write-RtTaskFunctionsArtifactInventory

    if ($ConfigureIntimeBuild -and -not (Test-Path -LiteralPath (Join-Path $RsiRoot "RTTaskFunctions.rsl"))) {
        throw "RTTaskFunctions.rsl was not produced in $RsiRoot. rsiconfig/INtime cannot load the Racer3 RTTask library until this .rsl exists."
    }

    if (!(Test-Path -LiteralPath (Join-Path $RsiRoot "RTTaskFunctions.dll"))) {
        throw "RTTaskFunctions.dll was not produced in $RsiRoot. RapidCodeRemote cannot discover the Racer3 RTTask global metadata until this host metadata companion exists."
    }
}

$rapidServerUri = Get-RapidServerUri
$rapidServerHost = $rapidServerUri.Host
$rapidServerPort = $rapidServerUri.Port

if ($StartRapidServer) {
    Write-Step "Starting rapidserver"
    $rapidserver = Resolve-Tool "rapidserver.exe"
    $stdout = Join-Path $repoRoot "rapidserver-rttask-probe.log"
    $stderr = Join-Path $repoRoot "rapidserver-rttask-probe.err.log"
    $alreadyRunning = Get-Process -Name rapidserver -ErrorAction SilentlyContinue

    if ($alreadyRunning) {
        $rapidserverIds = ($alreadyRunning | ForEach-Object { $_.Id }) -join ', '
        Write-Host "rapidserver is already running: $rapidserverIds"
        if (!(Wait-RapidServerListening -HostName $rapidServerHost -Port $rapidServerPort -TimeoutSeconds 5)) {
            throw "rapidserver is running, but $RapidServerAddress is not accepting TCP connections. Stop stale rapidserver processes or choose the active port/address."
        }
    }
    else {
        Remove-Item -LiteralPath $stdout -Force -ErrorAction SilentlyContinue
        Remove-Item -LiteralPath $stderr -Force -ErrorAction SilentlyContinue

        $rapidServerArgs = @("-grpc_port", [string]$rapidServerPort)
        Write-Host "Command: $rapidserver $($rapidServerArgs -join ' ')"
        Write-Host "WorkingDirectory: $RsiRoot"
        Start-Process `
            -FilePath $rapidserver `
            -ArgumentList $rapidServerArgs `
            -WorkingDirectory $RsiRoot `
            -RedirectStandardOutput $stdout `
            -RedirectStandardError $stderr `
            -WindowStyle Hidden

        Start-Sleep -Seconds 2
        $startedProcess = Get-Process -Name rapidserver -ErrorAction SilentlyContinue
        if (!$startedProcess) {
            Write-FileTailIfPresent -Path $stdout
            Write-FileTailIfPresent -Path $stderr
            throw "rapidserver exited before it could listen on $RapidServerAddress. On this PC, avoid port 50061 because Windows may reserve 50060-50159; use 51061."
        }

        if (!(Wait-RapidServerListening -HostName $rapidServerHost -Port $rapidServerPort -TimeoutSeconds 15)) {
            Write-FileTailIfPresent -Path $stdout
            Write-FileTailIfPresent -Path $stderr
            throw "rapidserver started but did not accept TCP connections at $RapidServerAddress. Check rapidserver-rttask-probe logs."
        }

        Write-Host "rapidserver is listening at $RapidServerAddress. Logs:"
        Write-Host "  $stdout"
        Write-Host "  $stderr"
    }
}
elseif ($ReadFeedback) {
    if (!(Wait-RapidServerListening -HostName $rapidServerHost -Port $rapidServerPort -TimeoutSeconds 3)) {
        throw "-ReadFeedback was requested, but $RapidServerAddress is not accepting connections. Start rapidserver first, for example: C:\RSI\11.0.5\rapidserver.exe -grpc_port $rapidServerPort"
    }
}

Write-Step "Running rsiconfig with cloned RTTaskManager-enabled Racer3 settings"
$resolvedSettings = Resolve-Path -LiteralPath $SettingsFile -ErrorAction Stop
$rsiconfig = Join-Path $RsiRoot "rsiconfig.exe"
if (!(Test-Path -LiteralPath $rsiconfig)) {
    $rsiconfig = Resolve-Tool "rsiconfig.exe"
}

Push-Location $RsiRoot
try {
    Write-Host "This is the same architecture as the laser demo: rapidserver is verified first, then rsiconfig loads the RTTaskManagers block and submits XML-declared tasks."
    Invoke-Native $rsiconfig @($resolvedSettings.Path, "--verbose")
}
finally {
    Pop-Location
}

$endpointPrepSession = $null
if ($PrepareEndpointMotionForRtTaskJog) {
    Write-Step "Preparing endpoint/session motion prerequisites for RTTask jog"
    Write-Warning "This is explicit pre-arm setup. It reuses racer3-basic-motion --prearm-hold to clear faults, enable Axis 1..6 bottom-to-top, and keep amps enabled only for the RTTask proof."
    $endpointPrepSession = Start-EndpointMotionPrepSession -ReadyTimeoutSeconds 60
}

try {
if ($ReadFeedback) {
    Write-Step "Reading RTTaskManager/task/global feedback through rapidserver / RapidCodeRemote"
    $toolProject = Join-Path $repoRoot "tools\Racer3RtTaskManagerFeedback\Racer3RtTaskManagerFeedback.csproj"
    if (!(Test-Path -LiteralPath $toolProject)) {
        throw "Missing feedback tool project: $toolProject"
    }

    $dotnetArgs = @(
        "run",
        "--project", $toolProject,
        "--",
        "--address", $RapidServerAddress,
        "--manager-label", $ManagerLabel,
        "--iterations", $FeedbackIterations,
        "--delay-ms", $FeedbackDelayMs
    )

    if (![string]::IsNullOrWhiteSpace($RsiProtoPath)) {
        $dotnetArgs = @(
            "run",
            "/p:RSIProtoPath=$RsiProtoPath",
            "--project", $toolProject,
            "--",
            "--address", $RapidServerAddress,
            "--manager-label", $ManagerLabel,
            "--iterations", $FeedbackIterations,
            "--delay-ms", $FeedbackDelayMs
        )
    }

    if (![string]::IsNullOrWhiteSpace($RequiredAdvancingGlobal)) {
        $dotnetArgs += @(
            "--required-advancing-global", $RequiredAdvancingGlobal
        )
    }

    if (!$UseInstalledSampleRtTaskFunctions -and $ManagerLabel -eq "Racer3JogProbe") {
        $dotnetArgs += "--require-custom-racer3-globals"
    }

    if ($ExerciseJogIntentReadback) {
        $dotnetArgs += "--exercise-jog-intent"
    }

    if ($ArmRtTaskJogMotion) {
        $dotnetArgs += @(
            "--allow-motion",
            "--jog-axis", $JogAxis,
            "--jog-step-user-units", $JogStepUserUnits,
            "--jog-speed-user-units-per-second", $JogSpeedUserUnitsPerSecond
        )
    }

    if ($RequireStatusReadiness) {
        $dotnetArgs += "--require-status-readiness"
    }

    Write-Host "Command: dotnet $($dotnetArgs -join ' ')"
    & dotnet @dotnetArgs | Tee-Object -FilePath (Join-Path $repoRoot "rttask-manager-feedback.log")
    $dotnetExitCode = $LASTEXITCODE
    if ($dotnetExitCode -ne 0) {
        Write-RtTaskFunctionsArtifactInventory
        throw "Feedback reader failed with exit code $dotnetExitCode."
    }
}
}
finally {
    Stop-EndpointMotionPrepSession -State $endpointPrepSession
}

Write-Step "RTTaskManager log tails"
$logCandidates = @(
    (Join-Path $env:USERPROFILE "rttaskmanager.log"),
    (Join-Path $env:USERPROFILE "rttaskapi.log"),
    (Join-Path $repoRoot "rttaskmanager.log"),
    (Join-Path $repoRoot "rttaskapi.log"),
    (Join-Path $repoRoot "rapidserver-rttask-probe.log"),
    (Join-Path $repoRoot "rapidserver-rttask-probe.err.log"),
    (Join-Path $repoRoot "rttask-manager-feedback.log")
)
foreach ($log in $logCandidates) {
    if (Test-Path -LiteralPath $log) {
        Write-Host ""
        Write-Host "--- Tail: $log ---"
        Get-Content -LiteralPath $log -Tail 80
    }
}

if ($NoSessionServer) {
    Write-Step "Skipping Racer3 session server as requested"
    return
}

Write-Step "Starting Racer3 persistent session server after RTTaskManager rsiconfig"
Write-Host "Paste only JSON session commands here, such as {`"type`":`"status`"} or {`"type`":`"shutdown`"}."
& .\build-vs2022\Release\racer3-basic-motion.exe --session-server --step 0.05 --velocity 0.05 --return-warn 0.00025 --return-fail 0.001
