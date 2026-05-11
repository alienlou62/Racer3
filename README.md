# Racer3 RMP Basic Motion Demo

Basic RMP/RapidCode project for moving the Comau Racer 3 through a simple all-axis motion demo.

## What this does

- Connects to the RSI RMP controller
- Clears faults
- Enables amplifiers
- Creates a six-axis MultiAxis group for joint-space motion
- Executes a small low-speed relative joint motion sequence
- Returns the robot to its starting pose
- Disables amplifiers safely

## Prerequisites

- Windows development machine
- RSI RapidCode SDK installed (tested with `C:\RSI\11.0.0`)
- Physical or simulated Comau Racer 3 robot controller available on the RMP network
- A safe robot environment and E-stop ready

## Build

```powershell
mkdir build
cd build
cmake .. -DRSI_SDK_ROOT="C:/RSI/11.0.0"
cmake --build . --config Release
```

If the SDK is installed in a different location, set `RSI_SDK_ROOT` to the SDK root directory.

## Run

```powershell
cd build
.\Release\racer3-basic-motion.exe
```

The program prints a safety warning and waits for Enter before sending motion commands.

## Project structure

- `src/main.cpp` — simple application entry point and top-level error handling
- `src/Racer3BasicMotion.h` / `src/Racer3BasicMotion.cpp` — RSI-specific motion logic
- `config/axes.json` — conservative axis mapping and demo motion settings
- `docs/safety.md` — safety guidance for physical robot testing
- `CMakeLists.txt` — build configuration for RSI RapidCode

## Notes

- Motion uses very small relative joint offsets (±5°) for safety.
- `config/axes.json` now includes conservative Racer3 joint limits and historical hardware parameters from the legacy RapidRobot test driver.
- The current implementation does not yet parse `config/axes.json`; it provides the starter configuration for later milestones.
- Always keep the robot under manual control while testing.