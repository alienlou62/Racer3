# Project Journal

## 2026-05-28 - Xbox jog soft-limit baseline stabilized

Branch: rttasks
Known-good branch: host-jog-stable
Known-good tag: host-jog-xbox-softlimits-working-v1

Result:
- Validated host-side keyboard/Xbox live jog baseline is working.

Worked:
- Keyboard jog works.
- Xbox 360 controller jog works.
- X reach/retract works well.
- Z jog is smoother/faster than the earlier baseline.
- Base rotation works as the operator-friendly aiming control.
- Direct J5 pitch works for pointing the tool/end effector.
- Direct J4 roll works.
- Direct J6 yaw works.
- J6 remains intentionally free-spinning/unlimited.
- H-home works and returns close to zero.
- B/Back exits cleanly, disables amps, and clears faults.
- Release-to-center / release-to-idle behavior is clean.
- Software soft joint-limit guard scales/stops motion into limits while allowing motion away from limits.

Rolled back:
- Experimental tool-leading / envelope posture mode was tested and did not behave correctly.
- It was rolled back and should not be reintroduced casually.

Repo cleanup:
- Branches were cleaned up.
- Current expected branches are main, rttasks, and host-jog-stable.
- Stable restore tag was pushed as host-jog-xbox-softlimits-working-v1.

Next:
- Continue safe speed tuning.
- Add controller tuning options.
- Improve UI feedback for active jog state and soft-limit warnings.
- Continue controlled soft-limit testing.
- Later evaluate RTTasks migration path.

## Template - New Entry

## YYYY-MM-DD - Short title

Branch:
<branch>

Commit:
<hash> <message>

Goal:
- <goal>

Result:
- <pass/fail/partial>

Worked:
- <item>

Failed / watch:
- <item>

Next:
- <item>
