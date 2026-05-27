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
  --cartesian-jog-gain-z 1.20 `
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
- Live jog now has a conservative software soft-limit guard based on the OpenRAVE XML joint limits with a 5-degree reserve. Jog commands are scaled/stopped before modeled limits, while motion away from a limit remains allowed.
- Do not require robot hardware for documentation or cleanup validation. Live hardware testing is separate.

## Optional Xbox 360 controller test mode

The keyboard jog mode can also poll an Xbox 360-compatible controller through XInput when launched with `--xbox-controller`.
Keyboard controls remain active in this mode.

Initial controller mapping:

- Left stick Y: X reach/retract in the current base-facing vertical plane.
- Left stick X: direct base/J1 rotate.
- Right stick Y: faster Z up/down.
- Right stick X: direct J6 yaw aim.
- LB/RB: direct J4 roll aim.
- LT/RT: direct J5 pitch aim.
- A: smooth stop / idle.
- Y: H-home return to the run-start joint pose.
- B or Back: exit jog mode safely.

The first controller pass intentionally chooses one dominant translation/base command at a time from the two sticks.
This keeps the joystick path close to the validated keyboard behavior and avoids diagonal stick input accidentally entering the old Cartesian Y path. Manual wrist inputs are direct joint jogs for J4/J5/J6; use LT/RT to point the tool forward, release, then use left stick Y to reach while the Cartesian solver preserves the selected tool orientation.

Recommended first controller test command:

```powershell
.\build-vs2022\Release\racer3-basic-motion.exe `
  --keyboard-cartesian-jog-endpoint-only `
  --xbox-controller `
  --cartesian-jog-linear-speed 0.020 `
  --keyboard-base-rotate-speed 0.022 `
  --cartesian-jog-angular-speed 0.09 `
  --cartesian-jog-gain-x 5.5 `
  --cartesian-jog-gain-y 0.80 `
  --cartesian-jog-gain-z 1.20 `
  --cartesian-jog-max-joint-velocity 0.060 `
  --keyboard-startup-delay-seconds 15 `
  --confirm-keyboard-cartesian-jog
```

Start slower than the validated keyboard profile, confirm there is no stick drift at center, then increase toward the keyboard profile after the mapping is verified.

## Soft-limit test-window mode

After live limit testing showed the modeled J4/Roll limit was too optimistic for the configured drive/RMP limits, controller limit validation should use the temporary test-window mode first:

```powershell
--xbox-soft-limit-test-window
```

This mode creates a narrow wrist-only software window around the run-start pose before jogging:

- J4/Roll: +/-20 degrees from run-start
- J5/Pitch: +/-20 degrees from run-start
- J6/Yaw: +/-30 degrees from run-start

Use this mode to prove the software guard stops/scales motion before approaching real limits. Expected behavior is that motion into the test-window edge stops or scales, while the opposite direction still moves away. Do not continue real-limit testing until this test-window behavior is confirmed.


## Near-full operator soft-limit mode

After the temporary test-window guard has been validated, use the broader near-full operator range for practical Xbox jogging:

```powershell
--xbox-soft-limit-near-full-range
```

This mode is intended to give the operator almost the full useful motion range while still avoiding hard/drive limits. It is mutually exclusive with `--xbox-soft-limit-test-window`. Initial conservative ranges are:

- J1/Base: modeled XML range with about 10 degrees reserve
- J2/J3 arm joints: modeled XML range with about 10 degrees reserve
- J4/Roll: +/-120 degrees, intentionally below the modeled +/-180 degrees because earlier live testing showed the modeled J4 range was too optimistic for the current drive/RMP setup
- J5/Pitch: modeled XML range with about 10 degrees reserve
- J6/Yaw: free-spinning/unlimited in operator jog mode; verify jog/stop/H-home behavior, but do not treat yaw as a soft-limit validation axis

Treat this as an operator soft range, not a hard-limit test. Approach the edges gradually and verify that the guard scales/stops while motion away remains available.

### Soft-limit test-window validation note

When `--xbox-soft-limit-test-window` is enabled, direct base/wrist velocity commands are refreshed while held so the software guard is re-evaluated continuously. This is required for limit testing: a held RB/LB/LT/RT/right-stick command must stop at the temporary window edge, not only after the user releases and presses the same direction again.

Expected behavior near a test-window edge:

- Continuing farther into the blocked direction prints a soft-limit stop message and sends no motion.
- Moving the opposite direction is still allowed.
- H-home remains available.
