# Racer3 Installed-Sample RTTaskManager Probe

This is a no-motion bridge probe for validating the Racer3 RTTaskManager runtime path while the custom Racer3 build of `RTTaskFunctions.rsl` is blocked by the local INtime CDEV header/toolchain issue.

Before the custom Racer3 library is built, it intentionally loads the installed RSI sample library:

```text
C:\RSI\11.0.5\RTTaskFunctions.rsl
```

It does **not** prove the Racer3-specific contents of that library. After Option 2 is used, the same filename will be overwritten by the custom Racer3 `RTTaskFunctions.rsl`, matching the laser demo naming pattern. The script backs up the existing installed sample before deploying a custom build or prebuilt `.rsl`.

## What this proves

The probe validates the same runtime shape used by the laser demo:

1. Clean stale RSI / RTTask processes and logs.
2. Start or reuse `rapidserver` on a verified listening port.
3. Run `rsiconfig` against a settings XML containing an `RTTaskManagers` block.
4. Let `rsiconfig` create the RTTaskManager and XML-declared tasks.
5. Read RTTaskManager, task, and global feedback through RapidCodeRemote / `rapidserver`.

Passing this probe means the INtime NodeA + RTTaskManager + `rapidserver` + `rsiconfig` + feedback path is healthy. It does not prove the custom Racer3 RTTask library build; that remains the separate `RTTaskFunctions.rsl` build task.

## Settings XML

Use:

```text
config\racer3-settings-rttask-installed-sample.xml
```

Key RTTaskManager values:

```text
UserLabel=Racer3InstalledSampleProbe
RTTaskDirectory=C:\RSI\11.0.5
Platform=INtime
NodeName=NodeA
LibraryName=RTTaskFunctions
LibraryDirectory=<empty>
```

Declared tasks:

```text
Initialize
Increment
```

The installed RSI sample `Increment` task advances the `counter` global. The feedback reader therefore requires `counter` to exist and advance when `-UseInstalledSampleRtTaskFunctions` is used.

## Run command

From the Racer3 repo root on the Windows 10 robot PC:

```powershell
cd C:\Users\JP\racer3-rmp-basic-motion
$env:PATH = "C:\RSI\11.0.5;$env:PATH"

powershell -ExecutionPolicy Bypass -File .\scripts\racer3-rttask-manager-probe.ps1 `
  -UseInstalledSampleRtTaskFunctions `
  -CleanRuntime `
  -StartRapidServer `
  -ReadFeedback `
  -NoSessionServer
```


### Windows rapidserver port note

On the current Racer3 Windows PC, Windows excludes the default rapidserver range that includes `50061`. Use `51061` instead. The probe script defaults to:

```text
http://127.0.0.1:51061
```

The script starts rapidserver with:

```powershell
C:\RSI\11.0.5\rapidserver.exe -grpc_port 51061
```

and verifies the TCP listener before running `rsiconfig`. This keeps the startup order aligned with the laser demo: rapidserver first, then rsiconfig/network/RTTaskManager, then feedback/readback.

## Expected pass criteria

The command should show:

- `C:\RSI\11.0.5\RTTaskFunctions.rsl` exists.
- `rapidserver` starts or is already running.
- `rsiconfig` accepts `config\racer3-settings-rttask-installed-sample.xml`.
- `Racer3InstalledSampleProbe` is discovered through RapidCodeRemote.
- Task IDs are visible.
- `Initialize` and `Increment` are visible in task feedback.
- `counter` is present and its delta is positive.
- The script exits successfully without starting the Racer3 session server.

## Failure diagnosis

Do not switch back to jog code if this fails. Inspect:

- `rsiconfig` output.
- `rapidserver-rttask-probe.log` and `.err.log`.
- `%USERPROFILE%\rttaskmanager.log`.
- `%USERPROFILE%\rttaskapi.log`.
- `rttask-manager-feedback.log`.
- Whether `Racer3InstalledSampleProbe` was created.
- Whether the installed sample tasks loaded from `RTTaskFunctions`.
- Whether the `counter` global is present and advancing.

## Safety boundary

This patch does not touch robot motion code, UI jog code, endpoint-only jog code, or the stable `config\racer3-settings.xml` path. It is only a runtime bring-up/readback proof.
