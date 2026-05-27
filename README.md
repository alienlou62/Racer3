# Racer3 RMP Basic Motion

RMP/RapidCode control project for the Comau Racer 3 robot. The repo currently contains the C++ backend, RMP/axis configuration, live keyboard jog scripts, RTTASK investigation notes, and an Avalonia UI prototype.

The current best working live baseline is the backend-owned keyboard Cartesian jog mode documented in [docs/current-keyboard-jog.md](docs/current-keyboard-jog.md).

## Current Focus

- Preserve the working keyboard Cartesian jog behavior in `src/Racer3BasicMotion.cpp`.
- Keep live robot startup scripts and configuration easy to find.
- Keep generated logs, build outputs, and one-off backups out of the repo root.
- Keep hardware testing opt-in and explicitly confirmed.

## Prerequisites

- Windows development machine
- RSI RapidCode SDK installed (tested with `C:\RSI\11.0.5`)
- Physical or simulated Comau Racer 3 robot controller available on the RMP network
- A safe robot environment and E-stop ready

## Build

```powershell
cd C:\Users\JP\racer3-rmp-basic-motion
$env:PATH = "C:\RSI\11.0.5;$env:PATH"
cmake --build .\build-vs2022 --config Release --target racer3-basic-motion
```

The expected binary is `build-vs2022\Release\racer3-basic-motion.exe`.

## Run

For the validated keyboard jog command and smoke sequence, see [docs/current-keyboard-jog.md](docs/current-keyboard-jog.md).

Default and diagnostic modes remain intentionally conservative. Real robot motion should use explicit confirmation flags and a safe test envelope.

## Project Structure

- `src/`: C++ backend and Racer 3 motion logic.
- `config/`: RMP axis, MultiAxis, kinematics, and RTTASK-related XML/JSON configuration.
- `scripts/`: startup, smoke-test, jog-test, and probe scripts.
- `docs/`: safety, current keyboard jog, RTTASK notes, and architecture notes.
- `ui/`: Avalonia motion UI prototype.
- `rttasks/`: RTTASK source experiments.
- `tools/`: helper tools, including RTTASK manager feedback utilities.
- `cmake/`: CMake helper modules for RSI/RapidSoftware/INtime integration.
- `archive/`: local cleanup holding area for old logs and source backups.

## Notes

- Always keep the robot under manual control while testing.
- Do not change the current W/S, J5 compensation, Z-lock, H-home, or release-stop behavior without an explicit live-test reason.
- Root logs and source backup snapshots belong under `archive/`, not at the repo root.

## Safe Test Commands

Dry run only, no RMP connection:

```cmd
racer3-basic-motion.exe --dry-run
```

Enable then disable only, no motion:

```cmd
racer3-basic-motion.exe --enable-only
```

Tiny relative joint-space motion, requires explicit confirmation flag:

```cmd
racer3-basic-motion.exe --tiny-motion --confirm-motion
```

Default behavior is enable-only. Real motion is opt-in.
