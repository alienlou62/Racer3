# Racer3 RTTask Jog Migration Notes

This note was written against the latest local Racer3 zip after the stable v14 jog fallback. The older laser demo was used only as an RTTask structure reference.

## Current safe Racer3 base

The current product path should keep the persistent armed session as the startup foundation:

- `rsiconfig` starts/configures the RMP network once.
- The C++ persistent session connects RapidCode, clears the startup home/fault condition, maps MultiAxis 6, enables all six amps, and keeps them enabled.
- Backend-owned Cartesian Z- jog currently uses endpoint-only IK plus non-append 500 ms `MovePVT` spans.
- v13 raw host-loop APPEND was intentionally backed out because the first APPEND attempt produced RapidCode path error 3856 and disabled amps.

The v14 fallback is safe enough for continued testing, but it still visibly inches because every span returns to `MotionDone` before the next span is generated.

## What the laser demo shows

The laser demo has the RTTask pattern that Racer3 should follow next:

1. `rttasks/rttaskglobals.h` defines a `GlobalData` structure with `RSI_GLOBAL(...)` members.
2. The same header exports `GlobalMemberOffsetGet`, `GlobalNamesFill`, and `GlobalMemberTypeGet` so the RTTask manager and host can discover globals.
3. `rttasks/rttaskfunctions.cpp` exports tasks via an `RSI_TASK(...)` wrapper.
4. RTTask code gets real-time objects with `RTMotionControllerGet()`, `RTMultiAxisGet(...)`, and `RTAxisGet(...)` instead of constructing a new controller.
5. The laser `Initialize` task prepares globals and motion objects once; cyclic tasks then use globals as intent/state transfer.
6. The laser settings XML registers an `RTTaskManager` and submits multiple tasks with explicit priority, period, and repeat settings.

The laser motion task still issues simple `MoveSCurve` targets, so it is a structural reference rather than the final Racer3 Cartesian jog algorithm.

## RapidCode docs takeaways

- RTTask samples submit an initialization task first, then cyclic repeat-forever tasks with fixed periods.
- RTTask globals are the intended bridge between the host/UI and the real-time task library.
- The PVT sample uses coordinated `position`, `velocity`, and `time` arrays and `MovePVT(...)`.
- The streaming buffer sample is more relevant for smooth jogging than single complete PVT moves: it discusses controller sample timing, sync interrupts, keeping a desired number of points buffered, and generating the amount of motion data required since the last sample.

## v15 scope

This patch intentionally adds a no-motion RTTask scaffold only:

- `rttasks/rttaskglobals.h`
- `rttasks/rttaskfunctions.cpp`
- optional `RTTaskFunctions` CMake target
- RTTask manager XML fragment example

The new tasks do not call:

- `rsiconfig`
- `AxisAdd` / remap
- `ClearFaults`
- `AmpEnableSet`
- `Abort`
- `MovePVT`, `MoveSCurve`, `MoveVelocity`, or any motion command

The first validation is only:

1. Can the Racer3 RTTask DLL build?
2. Can the RTTask manager load it?
3. Can `Racer3Initialize` run once?
4. Can `Racer3StatusSampler` increment heartbeat and read MultiAxis 6 plus Axis 0..5 state?
5. Can `Racer3JogIntentMonitor` observe host-written jog intent globals?

## Intended next patches

### v16: host probe only

Add backend commands such as:

- `rttask_probe_start`
- `rttask_probe_status`
- `rttask_probe_stop`

These should create/discover an RTTask manager, submit `Racer3Initialize`, submit `Racer3StatusSampler`, and read heartbeat/status globals back as JSON. Still no motion.

### v17: jog intent bridge

Wire UI/backend `jog_cartesian_start` and `jog_cartesian_stop` to RTTask globals while leaving v14 as the default motion path. The RTTask should only observe intent and report transitions.

### v18+: real RTTask jog motion

Move the current host-side jog loop into a cyclic task. The eventual smooth path should generate joint targets from endpoint-only Cartesian intent on a deterministic period and should avoid host-side wait-for-done gaps. The exact RapidCode motion primitive needs to be validated after the RTTask heartbeat/global bridge is proven.

## v16 probe bridge

The first patch after confirming the RTTask DLL builds adds host/session commands only:

```json
{"type":"rttask_probe_start"}
{"type":"rttask_probe_status"}
{"type":"rttask_probe_stop"}
```

This is intentionally no-motion. It mirrors the laser demo structure by submitting a one-shot `Racer3Initialize` task followed by cyclic `Racer3StatusSampler` and `Racer3JogIntentMonitor` tasks. The host reads shared globals such as `heartbeat`, `lastSampleCounter`, `samplePeriodSeconds`, and joint snapshots to prove the task manager, DLL loading, global metadata, and controller object access are correct before any jog motion moves into RTTask code.

Default DLL lookup is:

```text
build-vs2022-rttask\rttasks\Release\RTTaskFunctions.dll
```

A custom library directory can be passed for probe experiments:

```json
{"type":"rttask_probe_start","libraryDirectory":"C:/Users/JP/racer3-rmp-basic-motion/build-vs2022-rttask/rttasks/Release","statusPeriodMs":10,"intentPeriodMs":10}
```

The v14 non-append PVT jog remains the only live jog path while this probe is validated. The RTTask probe does not call `rsiconfig`, `AxisAdd`, `ClearFaults`, `AmpEnableSet`, `Abort`, `TriggeredModify`, or any motion primitive.

## v17 heartbeat-only probe hardening

The first no-motion RTTask bridge run proved that the manager could be created and stopped without disturbing the armed session, but the returned globals stayed at zero. v17 therefore separates two bring-up questions:

1. **Does the RTTask DLL function dispatch execute at all?**
   - `Racer3BasicHeartbeat` increments `basicHeartbeat` and `heartbeat` without calling `RTMotionControllerGet`, `RTAxisGet`, `RTMultiAxisGet`, or any motion/setup API.
2. **Can RTTask code safely sample Racer3 controller objects?**
   - `Racer3Initialize`, `Racer3StatusSampler`, and `Racer3JogIntentMonitor` still do the no-motion controller/MultiAxis/axis sampling.

Expected v17 probe status behavior:

```json
{"type":"session_rttask_probe_status","basicHeartbeat":10,"basicHeartbeatExecutionCount":10,"heartbeat":10}
```

If `basicHeartbeat` advances but `initializationCount`, `statusExecutionCount`, or motion-state fields do not, the next fix should focus on RTTask-side access to `MotionControllerGet` / `MultiAxisGet(6)` / `AxisGet(0..5)`. If `basicHeartbeat` also stays zero, the issue is lower level: task naming, library loading, RTTask manager creation parameters, or DLL export/metadata registration.


## v18 RTTask deployment/dispatch correction

The v17 probe proved that creating the manager and reading globals was safe, but
the task counters stayed at zero.  The next correction follows the laser demo more
closely: build/deploy `RTTaskFunctions.dll` into the RMP runtime directory
(`C:/RSI/11.0.5` by default) instead of relying on a build-tree DLL path.  The
probe now also submits an official-sample-compatible `Increment` task that only
increments the shared global `counter`.

Expected progression:

1. `counter` and `incrementExecutionCount` increase.
2. `basicHeartbeat` and `basicHeartbeatExecutionCount` increase.
3. `Racer3Initialize` and controller-object sampler globals begin to update.

If only `counter`/`basicHeartbeat` move, task dispatch is good and the next issue
is RTTask-side `MotionControllerGet` / `MultiAxisGet(6)` access.  If neither
changes, the library deployment/task-manager configuration is still wrong.

## v19 RTTask probe correction: laser-style task loading

The v18 probe showed that a regular MSVC `RTTaskFunctions.dll` could be copied to `C:/RSI/11.0.5`, but the RTTask functions still did not execute when the host created an INtime manager: the official-sample `counter` and Racer3 heartbeat globals stayed at zero and all submitted task execution counts stayed negative.

The laser demo uses a different and important convention:

- the RTTask library is deployed into the RTTask runtime directory;
- per-task `LibraryDirectory` is left empty so the manager resolves the task library from `RTTaskDirectory`;
- the RTTask manager platform must match the binary format of the task library.

For the current Racer3 scaffold, the optional CMake target builds a normal Windows/MSVC DLL. That DLL is suitable for a native/no-motion dispatch probe, not for an INtime manager expecting an INtime `.rsl`. v19 therefore changes the no-motion probe default to:

- `managerPlatform=native`
- `RTTaskDirectory=C:/RSI/11.0.5`
- empty per-task `LibraryDirectory`

Use `managerPlatform=intime` only after adding/building an INtime `.rsl` RTTaskFunctions target. The final smooth jog path still needs a real RTTask/INtime or controller-side cyclic implementation; this v19 step is only meant to prove function export, task dispatch, and shared globals with the binary format we actually build today.


## v20: laser-style RTTaskManager startup

The laser demo does not rely on ad-hoc host-only task submission as the primary runtime path. Its `settings.xml` contains an `RTTaskManagers` block, deploys the task library into the RMP runtime directory, leaves per-task `LibraryDirectory` empty, and lets `rsiconfig` start the manager and its configured tasks.

Racer3 now has a separate probe settings file for this pattern:

```text
config/racer3-settings-rttask-probe.xml
```

This file clones the normal Racer3 network/axis/MultiAxis configuration and adds a laser-style `RTTaskManager` with:

- `RTTaskDirectory = C:\RSI\11.0.5`
- `Platform = INtime`
- `NodeName = NodeA`
- `LibraryName = RTTaskFunctions`
- empty per-task `LibraryDirectory`
- configured tasks: `Racer3Initialize`, `Increment`, `Racer3BasicHeartbeat`, `Racer3StatusSampler`, and `Racer3JogIntentMonitor`

The normal stable UI path should continue to use `config/racer3-settings.xml`. Use the RTTask probe settings only when validating task-manager startup/feedback.

Helper script:

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\racer3-rttask-manager-probe.ps1 -BuildRtTasks -ConfigureIntimeBuild
```

If INtime tooling is not available yet, omit `-ConfigureIntimeBuild` to build only the host DLL and validate deployment paths. For the real task-manager path, the expected deliverable is an INtime `.rsl` in the RMP runtime directory, matching the RTTaskManager platform.

The next milestone after confirming the configured manager starts and reports task feedback is to connect host/UI jog intent to RTTask globals, still without RTTask-owned motion. Only after the global bridge is verified should smooth cyclic jog generation move into RTTask code.

## v23 laser-style RTTaskManager bring-up guardrails

The old laser demo remains the reference architecture for Racer3 RTTask work:

1. Build and deploy the RTTask function library to the RMP runtime directory.
2. Start `rapidserver` as a long-running process.
3. Run `rsiconfig` with a settings XML that contains the `RTTaskManagers` block.
4. Let `rsiconfig` create/start the RTTaskManager and submit the XML-declared tasks.
5. Read manager status, task IDs, task execution counts, and shared globals through RapidCodeRemote/gRPC.

Racer3 should not move jog generation into RTTasks until this no-motion manager feedback path is proven. The stable non-RTTask jog fallback remains separate and should continue using `config/racer3-settings.xml`.

The v23 helper script is intentionally a Windows/Racer equivalent of the laser startup pattern. It can clean stale RSI processes/logs, build/deploy `RTTaskFunctions.rsl`, start `rapidserver`, run `rsiconfig` with `config/racer3-settings-rttask-probe.xml`, and read back `counter`, `basicHeartbeat`, `heartbeat`, task IDs, and task execution counts through RapidCodeRemote.
