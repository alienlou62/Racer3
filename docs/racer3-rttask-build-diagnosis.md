# Racer3 RTTaskFunctions INtime Build Diagnosis

This note records the RTTask build comparison used by `scripts/racer3-rttask-manager-probe.ps1`. It is intentionally limited to the no-motion RTTaskManager bring-up path.

## Runtime Architecture Reference

The laser demo is the runtime architecture reference, not a direct Windows build-system reference. Its checked-in `config/settings.xml` declares an `RTTaskManagers` block with:

- `RTTaskDirectory=/rsi`
- `Platform=Linux`
- empty `LibraryDirectory` per task
- task entries that name `FunctionName`, `LibraryName=RTTaskFunctions`, `UserLabel`, `Priority`, `Repeats`, `Period`, `Phase`, and `EnableTiming`

The laser startup scripts follow:

```text
clean stale RSI state
start rapidserver
run rsiconfig settings.xml
let rsiconfig create the RTTaskManager and XML-declared tasks
UI/service discovers the manager and reads task/global feedback
```

Racer mirrors that runtime flow in `config/racer3-settings-rttask-probe.xml` with `Platform=INtime`, `NodeName=NodeA`, `RTTaskDirectory=C:\RSI\11.0.5`, and `LibraryName=RTTaskFunctions`.

## Build References Inspected

The installed RSI Windows reference is `C:\RSI\11.0.5\examples\RTTaskFunctions`.

Important facts from that project:

- It builds its own `RTTaskFunctions` shared library; it does not rely on only the preinstalled `RTTaskFunctions.rsl`.
- Its `CMakeLists.txt` includes `C:\RSI\11.0.5\cmake\RSI.cmake`.
- On Windows, `INTIME_BUILD_ENABLED` runs `configure_cmake_for_intime(...)`.
- The generated host target builds the child target with:

```text
cmake --build <build>\INtime --target RTTaskFunctions --config Release
```

- The generated INtime project is `Release|INtime`, `ConfigurationType=DynamicLibrary`, `PlatformToolset=v143`, `TargetExt=.rsl`, `OutDir=C:\RSI\11.0.5\`.

The generated Racer project has the same key shape:

- `build-vs2022-rttask\INtime\RTTaskFunctions.vcxproj`
- `Release|INtime`
- `ConfigurationType=DynamicLibrary`
- `PlatformToolset=v143`
- `TargetExt=.rsl`
- `OutDir=C:\RSI\11.0.5\`

## Diagnosis Questions

1. Why is Racer3's generated INtime vcxproj using `PlatformToolset=v143`?

   Because the build is generated with `Visual Studio 17 2022`. CMake writes VS2022 projects with the VS2022 toolset, `v143`. The installed RSI sample generated the same `PlatformToolset=v143`.

2. What Platform/Toolset/props does the laser demo use for its working RTTask build?

   The checked-in laser demo runtime settings use `Platform=Linux`, so the laser startup path does not use a Visual Studio `PlatformToolset`. Its repository contains Windows/INtime helper files, but the startup XML and scripts are Linux-oriented. The Windows toolset reference for this PC is the installed RSI `examples\RTTaskFunctions` project, which generates `Platform=INtime` and `PlatformToolset=v143`.

3. Does the laser demo build with Visual Studio generator, Unix Makefiles, Ninja, or a custom INtime compiler path?

   The laser demo repo does not include a startup build command or CMake preset. Its CMake and scripts are Linux-oriented (`/rsi`, `/etc/laser_demo`, `pkg-config`, OpenCV, Pylon, shell scripts), so the generator is whatever the Linux build invocation chooses. It is not evidence that Windows/INtime should use Ninja or a custom compiler path.

4. Does the laser demo rely on RSI's sample `RTTaskFunctions.rsl` already installed, or does it build its own `.rsl`?

   The laser demo builds its own library target named `RTTaskFunctions` from its `rttasks` sources. In its Linux settings, the manager loads `RTTaskFunctions` from `/rsi`. Separately, the installed RSI sample artifacts in `C:\RSI\11.0.5` include `RTTaskFunctions.dll`, `.lib`, and `.rsl`, and Racer now follows that same Option 2 pattern: the basename stays `RTTaskFunctions`, while the compiled contents come from Racer3 `rttasks` sources.

5. What exact build command creates the installed RSI sample `.rsl` shape?

   After configuring `examples\RTTaskFunctions` with the VS2022 generator, the generated host target invokes:

```text
cmake --build <sample-build>\INtime --target RTTaskFunctions --config Release
```

   The child project writes `RTTaskFunctions.rsl` to `C:\RSI\11.0.5`.

6. What is the smallest change to make Racer3 build/deploy `RTTaskFunctions.rsl` the same way?

   Use the standard laser-demo library target/basename `RTTaskFunctions` for Racer3 custom RTTask source, and make MSBuild able to resolve `Platform=INtime` + `PlatformToolset=v143`. On this PC, VS v143 C++ tools exist for Win32/x64, but the selected VS `VCTargetsPath` does not contain `Platforms\INtime`. INtime's VS2022 platform files exist under `C:\Program Files (x86)\INtime\vstudio170\platforms\intime`. The probe script now creates a local `build-vs2022-rttask\msbuild-vctargets-overlay` that merges the VS `v170` targets with INtime's `Platforms\INtime`, then passes that as `VCTargetsPath`.

## Current Verified Build State

Without the overlay, both the installed RSI sample project and Racer's generated INtime project fail with:

```text
MSB8020: The build tools for v143 (Platform Toolset = 'v143') cannot be found
```

With the overlay, MSBuild resolves `v143` for `Platform=INtime` and reaches compilation. The next verified blocker is:

```text
C:\Program Files (x86)\INtime\rt\include\rtbase.h(11,10):
fatal error C1083: Cannot open include file: 'rtwin32.h'
```

That is not an INtime/NodeA missing condition and not a missing VS v143 component. It means the installed INtime CDEV headers are incomplete for custom RTTask `.rsl` compilation. RSI notes that CDEV `7.1.24270.1` omitted `rtwin32.h`; install CDEV `7.1.25030.3` or newer, then rerun the Racer probe.

## Installed RSI Sample Bridge Probe

While the custom Racer3 build of `RTTaskFunctions.rsl` is blocked by the local INtime CDEV headers, the installed-sample bridge probe uses the currently installed `C:\RSI\11.0.5\RTTaskFunctions.rsl` to validate the runtime path independently from the custom Racer3 RTTask build.

The bridge settings file is `config/racer3-settings-rttask-installed-sample.xml`. It keeps the Racer3 network/axis/MultiAxis configuration, creates an INtime RTTaskManager on `NodeA`, labels the manager `Racer3InstalledSampleProbe`, and loads only the installed `RTTaskFunctions` library. It declares only the sample `Initialize` and `Increment` tasks.

The script option is:

```powershell
-UseInstalledSampleRtTaskFunctions
```

That option switches the settings file, switches the expected manager label, requires the sample `counter` global to advance, and skips the custom `RTTaskFunctions` build/deploy path. Once Racer3 custom `RTTaskFunctions.rsl` is deployed, this installed-sample probe is no longer a pristine stock-sample proof unless the backed-up stock file is restored. This is intentionally no-motion and exists only to prove `rapidserver` + `rsiconfig` + RTTaskManager + feedback readback.
