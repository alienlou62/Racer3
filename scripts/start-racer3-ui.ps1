param(
    [string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$ProjectPath = Join-Path $RepoRoot "ui\Racer3MotionUi\Racer3MotionUi.csproj"

Push-Location $RepoRoot
try {
    dotnet run --project $ProjectPath --configuration $Configuration
}
finally {
    Pop-Location
}
