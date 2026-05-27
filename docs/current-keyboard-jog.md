# Current Keyboard Cartesian Jog

This is the current best working baseline for live Racer 3 keyboard jog testing. Preserve this behavior unless a future test explicitly replaces it.

## Controls

- `A` / `D`: direct base rotate / aim the robot base.
- `W` / `S`: X reach/retract in the current base-facing vertical plane.
- `R` / `F`: Z up/down.
- `I` / `K`, `J` / `L`, `U` / `O`: wrist/tool orientation controls.
- `H`: return to the run-start joint pose while keeping jog mode alive.
- `Q` or `Esc`: exit, disable amps, and clear faults.

Key release sends a zero `MultiAxis::MoveVelocitySCurve` command and keeps amps enabled. Normal release should not abort the group.

## W/S Reach Behavior

`W` and `S` are tuned for base-facing X reach/retract. The current intended behavior is:

- J1, J4, and J6 are held/locked during W/S.
- J5 is allowed as a pitch/orientation compensator.
- End effector orientation is maintained during X reach.
- A stronger Z-lock is active during X reach.
- X reach speed currently looks good with the validated command below.

Vertical `R` / `F` jog should also allow J5 compensation.

Important implementation markers in `src/Racer3BasicMotion.cpp` include `J5 allowed`, `PlanarForwardZHold`, `ZHold`, `MoveVelocitySCurve`, `H-home`, `base rotate`, `keyboard Cartesian`, and `Smooth mode`.

## Known Good Run Command

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

Write-Host "`n== Wait for RMP shared memory to settle =="
Start-Sleep -Seconds 8

Write-Host "`n== Start current validated jog test =="
.\build-vs2022\Release\racer3-basic-motion.exe `
  --keyboard-cartesian-jog-endpoint-only `
  --cartesian-jog-linear-speed 0.024 `
  --keyboard-base-rotate-speed 0.026 `
  --cartesian-jog-angular-speed 0.10 `
  --cartesian-jog-gain-x 6.5 `
  --cartesian-jog-gain-y 0.80 `
  --cartesian-jog-gain-z 0.85 `
  --cartesian-jog-max-joint-velocity 0.060 `
  --keyboard-startup-delay-seconds 15 `
  --confirm-keyboard-cartesian-jog

Write-Host "`nJog process exited with code: $LASTEXITCODE"
```

## Current Smoke Sequence

Run this with the robot in a safe test envelope and an E-stop ready:

1. `F` short hold, release.
2. `L` or `J` short hold to point the tool.
3. `W` for about 0.3 seconds, release.
4. `S` for about 0.3 seconds, release.
5. `W` for about 0.6 seconds, release.
6. `S` for about 0.6 seconds, release.
7. `H`.
8. `Q`.

## Safety Notes

- Use this only with a clear robot envelope, manual supervision, and an E-stop ready.
- Do not run live robot tests from a stale process tree; stop old Racer/RMP/RapidServer processes first.
- Do not regress release-stop behavior: normal key release should stop motion cleanly while keeping amps enabled.
- Do not regress `H` behavior: H-home should return to the run-start joint pose and keep jog mode active.
- Do not require robot hardware for documentation or cleanup validation. Live hardware testing is separate.
