# CODEX: Racer3 RMP Basic Motion Demo

## Project Overview

This repository is a starter demo for the Comau Racer 3 robot using the RSI RMP / RapidCode SDK.
The focus is on safe joint-space motion for six axes using a minimal and conservative C++ implementation.

## Key goals

- Connect to the RSI MotionController
- Clear drive faults
- Enable amplifier outputs only after initialization
- Execute a small coordinated six-axis motion sequence
- Return to the starting joint pose
- Disable amplifiers on normal exit or on error

## Design

- `src/main.cpp` handles the user warning, manual confirmation, and top-level exception handling.
- `src/Racer3BasicMotion.h` / `src/Racer3BasicMotion.cpp` encapsulate the RSI/RMP-specific controller and motion logic.
- `CMakeLists.txt` provides a simple Windows build configuration for the RSI RapidCode SDK.
- `config/axes.json` contains the conservative axis mapping and demo motion plan for later configuration support.
- `docs/safety.md` captures the safety rules and recommended robot test behavior.

## RMP / RapidCode API

The implementation is based on the installed RSI SDK headers from `C:\RSI\11.0.5`.
It uses:

- `RSI::RapidCode::MotionController::Create()` to create the controller object.
- `MotionController::ClearFaults()` to clear axis and motion faults.
- `MotionController::AmpEnableSet(true)` to enable amplifiers.
- `MotionController::MultiAxisGet(0)` to create a coordinated multi-axis group.
- `MultiAxis::AxesAdd(...)` to add all six robot axes.
- `MultiAxis::MoveVectorRelative(...)` for small coordinated relative joint moves.
- `MultiAxis::MotionDoneWait(...)` to wait for motion completion.
- `MotionController::AmpEnableSet(false)` and `Delete()` for clean shutdown.

## Safety and conservative defaults

- The demo uses very small relative offsets of ±5 units.
- The motion limits are intentionally low.
- The robot is not commanded to large joint displacements.
- A manual Enter confirmation is required before motion begins.
- Fault clearing and amplifier disable are included in the shutdown path.

## Next milestones

- Load axis mapping and motion limits from `config/axes.json`.
- Add a dry-run mode that prints motion commands instead of executing them.
- Add Cartesian motion support once kinematics, tool, and base frames are validated.
- Add path-following and simulation validation later.
