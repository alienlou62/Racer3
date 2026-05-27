# Racer3 RTTask Library Naming

Racer3 now follows the laser demo RTTask naming convention. The RTTask source is project-specific, but the deployed RTTask function library basename remains:

```text
RTTaskFunctions
```

On Windows/INtime, the custom Racer3 build therefore produces:

```text
C:\RSI\11.0.5\RTTaskFunctions.rsl
```

The settings XML should use:

```xml
<LibraryName>RTTaskFunctions</LibraryName>
<LibraryDirectory />
```

This matches the laser demo pattern where the library name stays stable and the exported task functions inside the library define the project behavior. For Racer3, the custom library exports the sample-compatible `Initialize` and `Increment` entries plus Racer-specific no-motion entries such as `Racer3BasicHeartbeat`, `Racer3StatusSampler`, and `Racer3JogIntentMonitor`.

Because RSI also installs a stock sample file at the same path, `scripts/racer3-rttask-manager-probe.ps1` backs up the existing `RTTaskFunctions.rsl` before deploying a prebuilt or locally built custom Racer3 version. The backup is written under:

```text
artifacts\rttask-backups
```

The installed-sample bridge remains useful before the custom file is deployed. After deployment, `RTTaskFunctions.rsl` refers to the custom Racer3 contents unless the stock backup is restored.
