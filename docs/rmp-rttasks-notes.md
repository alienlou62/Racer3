# RMP / RTTasks Notes

The uploaded laser demo uses RMP startup and RTTasks in this pattern:

1. Clean stale RSI processes and shared memory.
2. Start `rapidserver`.
3. Run `rsiconfig settings.xml --cpu-affinity ... --primary-nic ...`.
4. The settings XML defines controller/network configuration and RTTask managers.
5. RTTask code uses `RTMotionControllerGet()`, `RTMultiAxisGet()`, and `RTAxisGet()` inside `RSI_TASK(...)` functions.

This Racer3 starter project is currently a normal RapidCode C++ application, not an RTTask library. It uses `MotionController::Create()` and commands a 6-axis MultiAxis group from the application process.

That is acceptable for the first enable/disable test, but it assumes the RMP controller/network/server stack has already been started and configured for the Racer 3 outside this executable.

Do not add RTTasks until the basic standalone enable/disable test works. RTTasks are the next architecture step if we need deterministic cyclic logic running under the RMP task manager.

## Racer3 RTTaskManager Probe Notes

The RTTaskManager proof path should mirror the laser demo:

1. Build and deploy `RTTaskFunctions.rsl`.
2. Start `rapidserver` separately.
3. Run `rsiconfig config\racer3-settings-rttask-probe.xml --verbose`.
4. Let the settings XML create `Racer3JogProbe`.
5. Read manager state, task IDs, execution counts, and globals through RapidCodeRemote/gRPC.

On this PC, CMake generated the INtime child project with `Platform=INtime`, `TargetExt=.rsl`, and `PlatformToolset=v143`, but the selected Visual Studio MSBuild targets did not include an `INtime` platform folder. The probe script now creates a local `build-vs2022-rttask\msbuild-vctargets-overlay` by combining the Visual Studio `v170` targets with `C:\Program Files (x86)\INtime\vstudio170\platforms\intime`.

After that overlay, the next blocker is the installed INtime CDEV headers: `C:\Program Files (x86)\INtime\rt\include\rtwin32.h` and `rt.h` are missing. RSI published that INtime CDEV 7.1.24270.1 omitted `rtwin32.h`; update to CDEV 7.1.25030.3 or newer, then rerun the probe.
