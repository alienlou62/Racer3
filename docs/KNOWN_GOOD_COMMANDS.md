# Known Good Commands

## Check Git State

cd C:\Users\JP\racer3-rmp-basic-motion
git branch --show-current
git log -1 --oneline --decorate
git status --short
git diff --stat

## Build C++ Motion App

cd C:\Users\JP\racer3-rmp-basic-motion
$env:PATH = "C:\RSI\11.0.5;$env:PATH"
cmake --build .\build-vs2022 --config Release --target racer3-basic-motion

## Build UI

cd C:\Users\JP\racer3-rmp-basic-motion\ui\Racer3MotionUi
dotnet build

## Stop Old Racer/RMP/RapidServer Processes

Get-Process racer3-basic-motion,rapidserver,rsiconfig,RMPNetwork,RMP,rttaskmanager -ErrorAction SilentlyContinue | Stop-Process -Force -ErrorAction SilentlyContinue

## Known-Good Xbox Jog Run Command

Do not add --xbox-tool-lead-down; that experiment was rolled back.

Use the validated command from the latest project chat or update this section when the run command changes.

Current key parameters:
- --keyboard-cartesian-jog-endpoint-only
- --xbox-controller
- --xbox-soft-limit-near-full-range
- --cartesian-jog-linear-speed 0.025
- --keyboard-base-rotate-speed 0.028
- --cartesian-jog-angular-speed 0.105
- --cartesian-jog-gain-x 5.5
- --cartesian-jog-gain-y 0.90
- --cartesian-jog-gain-z 1.35
- --cartesian-jog-max-joint-velocity 0.060

## Export Handoff Zip

cd C:\Users\JP\racer3-rmp-basic-motion
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\export-handoff.ps1

## Commit Working Changes

cd C:\Users\JP\racer3-rmp-basic-motion
git status --short
git add -A
git commit -m "Describe the working change"
git push origin rttasks

## Xbox controller jog fast baseline

Validated fast Xbox controller jog profile after speed tuning.

Use this as the current baseline before testing more speed or end-effector drift improvements.

```powershell
cd C:\Users\JP\racer3-rmp-basic-motion

Write-Host "`n== Stop old Racer/RMP/RapidServer processes =="
Get-Process racer3-basic-motion,rapidserver,rsiconfig,RMPNetwork,RMP,rttaskmanager -ErrorAction SilentlyContinue |
  Stop-Process -Force -ErrorAction SilentlyContinue

Start-Sleep -Seconds 3

$env:PATH = "C:\RSI\11.0.5;$env:PATH"

Write-Host "`n== Start RMP with rsiconfig =="
cd .\config
& C:\RSI\11.0.5\rsiconfig.exe .\racer3-settings.xml --verbose
if ($LASTEXITCODE -ne 0) {
  throw "rsiconfig failed with exit code $LASTEXITCODE"
}
cd ..

Write-Host "`n== Wait for RMP shared memory/status to settle =="
Start-Sleep -Seconds 8

Write-Host "`n== Start Xbox controller jog: fast baseline =="
.\build-vs2022\Release\racer3-basic-motion.exe `
  --keyboard-cartesian-jog-endpoint-only `
  --xbox-controller `
  --xbox-soft-limit-near-full-range `
  --cartesian-jog-linear-speed 0.045 `
  --keyboard-base-rotate-speed 0.040 `
  --cartesian-jog-angular-speed 0.200 `
  --cartesian-jog-gain-x 4.5 `
  --cartesian-jog-gain-y 0.90 `
  --cartesian-jog-gain-z 1.10 `
  --cartesian-jog-max-joint-velocity 0.070 `
  --keyboard-startup-delay-seconds 15 `
  --confirm-keyboard-cartesian-jog

Write-Host "`nJog process exited with code: $LASTEXITCODE"
