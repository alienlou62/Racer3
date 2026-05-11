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
