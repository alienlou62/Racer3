# Racer3 Armed Session Mode Plan

## Purpose

The current Racer3 UI is useful for safe one-shot shape tracing, but every live run starts a new process, runs `rsiconfig`, connects to RMP, enables amplifiers, executes one trace, returns to zero, disables amplifiers, clears faults, and exits. That is safe, but it makes multi-shape demos and keyboard end-effector control slow.

The next major demo milestone is a persistent **Armed Session Mode**. In this mode the UI starts one backend process, the backend configures and connects once, enables amplifiers once, keeps them enabled for the duration of the session, accepts multiple validated trace or jog commands, and disables amplifiers only when the operator explicitly shuts the session down or a timeout/fault path requires it.

## Current one-shot lifecycle

1. UI launches `scripts/start-racer3-rmp-and-run.ps1`.
2. PowerShell runs `rsiconfig` and configures the RMP network.
3. `racer3-basic-motion.exe` starts.
4. The backend connects to `MotionController` and maps Axis 1..6 into MultiAxis 6.
5. The backend validates the requested Cartesian trace.
6. With `--confirm-motion`, the backend enables amplifiers.
7. The backend streams the outbound PVT trace.
8. The backend streams the return-to-zero PVT path.
9. The backend verifies return-to-zero tolerance.
10. The backend disables amplifiers, clears faults, releases controller objects, and exits.

This lifecycle must remain available as the default one-shot path.

## Desired armed session lifecycle

1. Operator clicks **Start Armed Session**.
2. UI starts a persistent backend process, likely `racer3-basic-motion.exe --session-server`.
3. Backend optionally runs/configures RMP once or assumes the wrapper did it once.
4. Backend connects to `MotionController` once.
5. Backend maps axes, configures user units, clears faults, and enables amplifiers once.
6. Backend reports `ArmedIdle`.
7. UI sends trace and jog commands one at a time.
8. Backend validates every motion command before executing it.
9. Backend holds the final pose after each command by default.
10. Backend keeps amplifiers enabled between commands.
11. Operator clicks **Shutdown Session**.
12. Backend stops motion if needed, disables amplifiers, clears faults, releases controller objects, and exits.

## Why not remove `disableAmplifiers()` from one-shot mode

The existing one-shot executable is designed to own exactly one motion attempt and then exit. Simply skipping `disableAmplifiers()` would leave unclear controller ownership when the process exits or throws. It would also mix two different safety models in one path.

Armed Session Mode should be a new explicit persistent process mode. That process owns the connected controller and enabled amplifiers until a structured shutdown path disables them.

## Safety rules

- Armed Session Mode must require an explicit operator action.
- Amplifiers may stay enabled only while the session process is alive and healthy.
- Every trace or jog command must pass backend validation before motion.
- Chatbot planning must not directly execute motion.
- Keyboard jog must be disabled unless Jog Mode is enabled and the session is armed.
- Stop Motion must stop the active motion command as quickly as the backend/API safely allows.
- Shutdown Session must disable amplifiers and clear faults.
- An idle timeout should automatically disable amplifiers and transition the session out of armed state.
- The physical E-stop remains the primary emergency stop.

## Proposed session states

```text
Disconnected
Starting
ArmedIdle
MotionRunning
Stopping
Faulted
ShuttingDown
Exited
```

## Proposed JSON line protocol

The simplest first implementation can use newline-delimited JSON over stdin/stdout between the Avalonia UI and the persistent backend process. A local TCP socket can be added later if needed.

### Hello

```json
{ "type": "hello" }
```

Expected response:

```json
{
  "type": "hello_ack",
  "robot": "Comau Racer3",
  "mode": "armed_session",
  "state": "ArmedIdle"
}
```

### Status

```json
{ "type": "status" }
```

Expected response:

```json
{
  "type": "status",
  "state": "ArmedIdle",
  "ampsEnabled": true,
  "motionRunning": false,
  "lastCommandAccepted": true
}
```

### Trace

```json
{
  "type": "trace",
  "confirmMotion": true,
  "positionOnly": true,
  "endpointOnly": true,
  "velocity": 0.04,
  "holdFinalPose": true,
  "returnToZero": false,
  "waypoints": [
    { "x": 0.5, "y": 0.05, "z": -0.55, "roll": 0, "pitch": 0, "yaw": 0 },
    { "x": 0.5, "y": -0.05, "z": -0.55, "roll": 0, "pitch": 0, "yaw": 0 }
  ]
}
```

### Cartesian jog

```json
{
  "type": "cartesian_jog",
  "confirmMotion": true,
  "dx": 0.005,
  "dy": 0.0,
  "dz": 0.0,
  "velocity": 0.025,
  "holdFinalPose": true
}
```

### Stop

```json
{ "type": "stop" }
```

### Shutdown

```json
{ "type": "shutdown" }
```

## UI behavior

The UI should eventually expose:

- **Start Armed Session**: starts the persistent backend and waits for `ArmedIdle`.
- **Shutdown Session**: disables amplifiers, clears faults, and exits the session.
- **Stop Motion**: stops the current session motion command.
- **Keep amps enabled during armed session**: default on for this mode.
- **Hold final pose after each command**: default on for interactive jogging and multi-shape demos.
- **Return to zero after each command**: optional compatibility behavior.
- **Enable Jog Mode**: gates keyboard jog input.
- Jog step and jog velocity fields.

## Keyboard jog mapping

Initial mapping:

```text
W / S: Z up / down
A / D: Y left / right
Q / E: X forward / back
Arrow keys: Y/Z plane
Space: stop
```

First implementation should use one key press per small validated jog command. Continuous hold-to-move can come later after stop behavior and latency are proven safe.

## Implementation phases

1. **Docs and UI placeholders**: document armed session mode and show planned controls. No live session behavior.
2. **Backend `--session-server` skeleton**: start process, print ready, parse JSON commands, support `hello`, `status`, and `shutdown` without motion.
3. **Backend armed startup/shutdown**: connect/configure once, enable amps once, keep alive, disable amps on shutdown.
4. **Session trace command**: reuse existing cartesian-trace validation and PVT execution, but keep amps enabled after completion.
5. **UI persistent session service**: start process, send JSON, read status, expose session state.
6. **Multi-shape session runs**: send multiple shape traces without restarting RMP or disabling amps.
7. **Keyboard jog**: send small validated Cartesian jog commands while armed.
8. **Idle timeout and fault handling**: disable amps on inactivity, fault, or lost process.
9. **Optional digital twin export/ROS2 bridge**: publish the same session commands to simulator consumers.


## Implemented Phase 2 skeleton

This revision adds the first non-motion implementation step for the local persistent process:

```powershell
.\build-vs2022\Release\racer3-basic-motion.exe --session-server
```

The skeleton command loop:

1. Starts without connecting to RMP.
2. Does not run `rsiconfig`.
3. Does not create a MotionController.
4. Does not enable amplifiers.
5. Prints a JSON `session_ready` event.
6. Accepts `hello`, `status`, `stop`, and `shutdown` commands over stdin.
7. Rejects `trace` and `cartesian_jog` commands with a clear no-motion response.
8. Exits cleanly on `shutdown`.

The Avalonia UI now starts this skeleton process from the **Start Armed Session** button and controls it with **Stop Motion** and **Shutdown Session**. This validates the UI-to-persistent-process architecture before any RMP or amp-enable behavior is added.

The next backend phase should replace the skeleton state with a real armed lifecycle:

```text
session process starts
-> run/configure RMP once
-> connect MotionController once
-> configure axes once
-> clear faults
-> enable amps once
-> report ARMED_IDLE
-> accept validated trace/jog commands
-> keep amps enabled until shutdown/timeout/fault
```

## Phase 2 implementation note: armed session startup/shutdown

The first live armed-session milestone keeps the session command loop simple but changes startup ownership:

- `Start Armed Session` runs `rsiconfig` once from the UI service.
- The UI starts `racer3-basic-motion --session-server` with `C:\RSI\11.0.5` prepended to PATH so RapidCode runtime DLLs are available.
- The C++ session process connects `MotionController` once, maps/configures axes once, clears faults, enables all six amps once, isolates the six-axis MultiAxis group, then reports `session_ready` with `armed=true` and `ampsEnabled=true`.
- `status` reports that the session is armed and amps are enabled.
- `stop` aborts active MultiAxis motion if any but keeps amps enabled.
- `trace` and `cartesian_jog` remain rejected until the next motion-command phase.
- `shutdown` aborts if needed, disables amps, clears faults, releases the controller, and exits.

This phase intentionally proves persistent ownership of the controller and one-time amp enable before adding motion commands to the session protocol.
