param(
    [ValidateSet("dry-run", "enable-only", "tiny-motion")]
    [string]$Mode = "enable-only",

    [string]$PrimaryNic = "",

    [switch]$Build,

    [switch]$SkipRsiconfig,

    [switch]$ConfirmMotion
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

switch ($Mode) {
    "dry-run" {
        $ExeArgs += "--dry-run"
    }
    "enable-only" {
        $ExeArgs += "--enable-only"
    }
    "tiny-motion" {
        $ExeArgs += "--tiny-motion"
        if ($ConfirmMotion) {
            $ExeArgs += "--confirm-motion"
        }
        else {
            throw "tiny-motion requires -ConfirmMotion. Keep the robot area clear and E-stop ready."
        }
    }
}

Push-Location $RepoRoot
& $ExePath @ExeArgs
$ExitCode = $LASTEXITCODE
Pop-Location

Write-Step "Finished"
Write-Host "racer3-basic-motion exit code: $ExitCode"
exit $ExitCode
