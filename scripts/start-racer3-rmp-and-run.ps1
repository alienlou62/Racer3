param(
    [ValidateSet("dry-run", "enable-only", "tiny-motion", "dual-motion", "all-motion", "joint-vector", "robot-model-probe", "robot-pose-probe", "kinematics-dry-run", "cartesian-vector", "cartesian-trace")]
    [string]$Mode = "enable-only",

    [string]$PrimaryNic = "",

    [switch]$Build,

    [switch]$SkipRsiconfig,

    [switch]$ConfirmMotion,

    [switch]$DryRun,

    [switch]$PositionOnly,

    [switch]$CompactMotion,

    [switch]$AppendMotion,

    [switch]$TrajectoryMotion,

    [switch]$EndpointOnly,

    [switch]$SegmentGoal,

    [double]$Step = 0.05,

    [double]$Velocity = 0.05,

    [double]$ReturnWarn = 0.00025,

    [double]$ReturnFail = 0.001,

    [switch]$Diagnostics,

    [string]$Joints = "",

    [string]$Cartesian = "0,0,0,0,0,0",

    [string]$CartesianTrace = ""
)

$ErrorActionPreference = "Stop"

function Write-Step {
    param([string]$Message)
    Write-Host ""
    Write-Host "==> $Message" -ForegroundColor Cyan
}

$RepoRoot = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path (Join-Path $RepoRoot "CMakeLists.txt"))) {
    $RepoRoot = Get-Location
}

$SettingsPath = Join-Path $RepoRoot "config\racer3-settings.xml"
$ExePath = Join-Path $RepoRoot "build-vs2022\Release\racer3-basic-motion.exe"

Write-Step "Racer3 RMP startup"
Write-Host "Repo root: $RepoRoot"
Write-Host "Settings:  $SettingsPath"
Write-Host "Exe:       $ExePath"
Write-Host "Mode:      $Mode"
Write-Host "Step:      $Step user units"
Write-Host "Velocity:  $Velocity user-units/sec"
Write-Host "ReturnWarn: $ReturnWarn user units"
Write-Host "ReturnFail: $ReturnFail user units"
if ($Mode -eq "joint-vector") { Write-Host "Joints:    $Joints" }
if ($Mode -eq "kinematics-dry-run" -or $Mode -eq "cartesian-vector") { Write-Host "Cartesian: $Cartesian" }
if ($Mode -eq "cartesian-trace") { Write-Host "CartesianTrace: $CartesianTrace" }
if ($Mode -eq "cartesian-vector" -or $Mode -eq "cartesian-trace") { Write-Host "PositionOnly: $PositionOnly" }
if ($Mode -eq "cartesian-vector" -or $Mode -eq "cartesian-trace") { Write-Host "CompactMotion: $CompactMotion" }
if ($Mode -eq "cartesian-vector" -or $Mode -eq "cartesian-trace") { Write-Host "AppendMotion: $AppendMotion" }
if ($Mode -eq "cartesian-vector" -or $Mode -eq "cartesian-trace") { Write-Host "TrajectoryMotion: $TrajectoryMotion" }
if ($Mode -eq "cartesian-vector" -or $Mode -eq "cartesian-trace") { Write-Host "EndpointOnly: $EndpointOnly" }
if ($Mode -eq "cartesian-vector" -or $Mode -eq "cartesian-trace") { Write-Host "SegmentGoal: $SegmentGoal" }
Write-Host "Diag:      $Diagnostics"
Write-Host "Dry run:   $DryRun"

$env:PATH = "C:\RSI\11.0.0;$env:PATH"

if (-not (Get-Command rsiconfig -ErrorAction SilentlyContinue)) {
    throw "rsiconfig.exe was not found. Confirm C:\RSI\11.0.0 is installed and on PATH."
}

if ($Build) {
    Write-Step "Building racer3-basic-motion"
    Push-Location $RepoRoot
    cmake --build build-vs2022 --config Release
    Pop-Location
}

if (-not (Test-Path $ExePath)) {
    throw "Executable not found: $ExePath. Build first with: cmake --build build-vs2022 --config Release"
}

if (-not $SkipRsiconfig) {
    if (-not (Test-Path $SettingsPath)) {
        throw "Missing settings file: $SettingsPath. Generate it once from a working RapidSetup/RMP state with: rsiconfig -generate .\config\racer3-settings.xml --verbose"
    }

    Write-Step "Starting/configuring RMP with rsiconfig"
    Push-Location (Split-Path -Parent $SettingsPath)

    if ([string]::IsNullOrWhiteSpace($PrimaryNic)) {
        Write-Host "Running: rsiconfig racer3-settings.xml --verbose"
        rsiconfig ".\racer3-settings.xml" --verbose
    }
    else {
        Write-Host "Running: rsiconfig racer3-settings.xml --verbose --primary-nic $PrimaryNic"
        rsiconfig ".\racer3-settings.xml" --verbose --primary-nic $PrimaryNic
    }

    Pop-Location

    Write-Step "Waiting briefly after rsiconfig"
    Start-Sleep -Seconds 3
}
else {
    Write-Step "Skipping rsiconfig by request"
}

Write-Step "Running racer3-basic-motion"

$ExeArgs = @()

if ($Step -le 0) {
    throw "-Step must be greater than zero."
}

if ($Velocity -le 0) {
    throw "-Velocity must be greater than zero."
}

if ($ReturnWarn -lt 0) {
    throw "-ReturnWarn must be zero or greater."
}

if ($ReturnFail -le 0) {
    throw "-ReturnFail must be greater than zero."
}

if ($ReturnWarn -gt $ReturnFail) {
    throw "-ReturnWarn must be less than or equal to -ReturnFail."
}

switch ($Mode) {
    "dry-run" {
        $ExeArgs += "--dry-run"
    }
    "enable-only" {
        if ($DryRun) {
            $ExeArgs += "--dry-run"
        }
        else {
            $ExeArgs += "--enable-only"
        }
    }
    "tiny-motion" {
        if ($DryRun) {
            $ExeArgs += "--dry-run"
            $ExeArgs += "--tiny-motion"
        }
        else {
            $ExeArgs += "--tiny-motion"
            if ($ConfirmMotion) {
                $ExeArgs += "--confirm-motion"
            }
            else {
                throw "tiny-motion requires -ConfirmMotion. Keep the robot area clear and E-stop ready."
            }
        }
    }
    "dual-motion" {
        if ($DryRun) {
            $ExeArgs += "--dry-run"
            $ExeArgs += "--dual-motion"
        }
        else {
            $ExeArgs += "--dual-motion"
            if ($ConfirmMotion) {
                $ExeArgs += "--confirm-motion"
            }
            else {
                throw "dual-motion requires -ConfirmMotion. Keep the robot area clear and E-stop ready."
            }
        }
    }
    "all-motion" {
        if ($DryRun) {
            $ExeArgs += "--dry-run"
            $ExeArgs += "--all-motion"
        }
        else {
            $ExeArgs += "--all-motion"
            if ($ConfirmMotion) {
                $ExeArgs += "--confirm-motion"
            }
            else {
                throw "all-motion requires -ConfirmMotion. Keep the robot area clear and E-stop ready."
            }
        }
    }
    "robot-model-probe" {
        if ($DryRun) {
            $ExeArgs += "--dry-run"
            $ExeArgs += "--robot-model-probe"
        }
        else {
            $ExeArgs += "--robot-model-probe"
        }
    }
    "robot-pose-probe" {
        if ($DryRun) {
            $ExeArgs += "--dry-run"
            $ExeArgs += "--robot-pose-probe"
        }
        else {
            $ExeArgs += "--robot-pose-probe"
        }
    }
    "kinematics-dry-run" {
        if ($DryRun) {
            $ExeArgs += "--dry-run"
            $ExeArgs += "--kinematics-dry-run"
        }
        else {
            $ExeArgs += "--kinematics-dry-run"
        }
        $ExeArgs += "--cartesian"
        $ExeArgs += $Cartesian
    }
    "cartesian-vector" {
        if ($DryRun) {
            $ExeArgs += "--dry-run"
        }
        $ExeArgs += "--cartesian-vector"
        if ($PositionOnly) { $ExeArgs += "--position-only" }
        if ($CompactMotion) { $ExeArgs += "--compact-motion" }
        if ($AppendMotion) { $ExeArgs += "--append-motion" }
        if ($TrajectoryMotion) { $ExeArgs += "--trajectory-motion" }
        if ($EndpointOnly) { $ExeArgs += "--endpoint-only" }
        if ($SegmentGoal) { $ExeArgs += "--segment-goal" }
        $ExeArgs += "--cartesian"
        $ExeArgs += $Cartesian
        if ($ConfirmMotion) { $ExeArgs += "--confirm-motion" }
    }
    "cartesian-trace" {
        if ([string]::IsNullOrWhiteSpace($CartesianTrace)) {
            throw 'cartesian-trace requires -CartesianTrace "x,y,z,0,0,0;...".'
        }

        if ($DryRun) {
            $ExeArgs += "--dry-run"
        }
        $ExeArgs += "--cartesian-trace"
        if ($PositionOnly) { $ExeArgs += "--position-only" }
        if ($CompactMotion) { $ExeArgs += "--compact-motion" }
        if ($EndpointOnly) { $ExeArgs += "--endpoint-only" }
        $ExeArgs += "--cartesian-waypoints"
        $ExeArgs += $CartesianTrace
        if ($ConfirmMotion) { $ExeArgs += "--confirm-motion" }
    }
    "joint-vector" {
        if ([string]::IsNullOrWhiteSpace($Joints)) {
            throw 'joint-vector requires -Joints "j1,j2,j3,j4,j5,j6".'
        }

        if ($DryRun) {
            $ExeArgs += "--dry-run"
            $ExeArgs += "--joint-vector"
        }
        else {
            $ExeArgs += "--joint-vector"
            if ($ConfirmMotion) {
                $ExeArgs += "--confirm-motion"
            }
            else {
                throw "joint-vector requires -ConfirmMotion. Keep the robot area clear and E-stop ready."
            }
        }

        $ExeArgs += "--joints"
        $ExeArgs += $Joints
    }
}

$ExeArgs += "--step"
$ExeArgs += ([string]$Step)
$ExeArgs += "--velocity"
$ExeArgs += ([string]$Velocity)
$ExeArgs += "--return-warn"
$ExeArgs += ([string]$ReturnWarn)
$ExeArgs += "--return-fail"
$ExeArgs += ([string]$ReturnFail)

if ($Diagnostics) {
    $ExeArgs += "--diagnostics"
}

Write-Host "Exe args:  $($ExeArgs -join ' ')"

Push-Location $RepoRoot
& $ExePath @ExeArgs
$ExitCode = $LASTEXITCODE
Pop-Location

Write-Step "Finished"
Write-Host "racer3-basic-motion exit code: $ExitCode"
exit $ExitCode
