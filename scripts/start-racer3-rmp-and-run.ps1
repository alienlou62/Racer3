param(
    [ValidateSet("dry-run", "enable-only", "tiny-motion", "dual-motion", "all-motion")]
    [string]$Mode = "enable-only",

    [string]$PrimaryNic = "",

    [switch]$Build,

    [switch]$SkipRsiconfig,

    [switch]$ConfirmMotion,

    [switch]$DryRun,

    [double]$Step = 0.05,

    [double]$Velocity = 0.05,

    [switch]$Diagnostics
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
}

$ExeArgs += "--step"
$ExeArgs += ([string]$Step)
$ExeArgs += "--velocity"
$ExeArgs += ([string]$Velocity)

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
