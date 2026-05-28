# Current Handoff State

Project: Racer3 RMP basic motion project.

Source of truth: Git/repo files, this handoff file, docs/PROJECT_JOURNAL.md, and docs/KNOWN_GOOD_COMMANDS.md.

Active branch: rttasks
Stable known-good branch: host-jog-stable
Known-good tag: host-jog-xbox-softlimits-working-v1

Current validated baseline:
- Host-side keyboard/Xbox live jog through racer3-basic-motion.exe.
- Keyboard jog works.
- Xbox 360 controller jog works.
- X reach/retract works well.
- Z up/down jog works and has been tuned smoother/faster than the earlier baseline.
- Left stick X base rotation is the operator-friendly aiming control.
- LT/RT direct J5 pitch works for pointing the tool/end effector.
- LB/RB direct J4 roll works.
- Right stick X direct J6 yaw works.
- J6 is intentionally treated as free-spinning/unlimited.
- Y button H-home works and keeps jog mode active.
- B/Back exits cleanly, disables amps, and clears faults.
- Release-to-center / release-to-idle behavior is clean.
- Software soft joint-limit guard is active.

Soft-limit guard summary:
- Uses modeled XML/OpenRAVE joint limits.
- Applies reserve before hard limits.
- Checks current joint position plus short lookahead before sending jog velocity.
- Scales or stops commands moving farther into a soft limit.
- Still allows motion away from the limit.
- Applies to X reach/retract, Z jog, base rotation, and direct J4/J5/J6 wrist jogs.

Known failed / rolled back work:
- Do not casually reintroduce --xbox-tool-lead-down.
- The experimental tool-leading / envelope posture mode made TCP motion behave incorrectly and was rolled back.

Current next goal:
- Continue safe speed tuning.
- Improve UI feedback for soft-limit warnings and active jog state.
- Add controller tuning options such as deadzone, wrist speed, Z speed, base speed, and precision/fast mode.
- Continue controlled soft-limit testing.
- Later evaluate which parts of the live jog pipeline should move into RTTasks.

Do not touch unless explicitly requested:
- host-jog-stable branch.
- J6 free-spin/unlimited behavior.
- H-home behavior.
- Clean exit / amp disable / fault clear behavior.
- Known-good keyboard jog behavior.
- Known-good Xbox controller mapping.
- Soft-limit guard behavior.

New chat instructions:
Upload the latest handoff zip and say: Use this handoff as source of truth. Read CURRENT_HANDOFF_STATE.md, docs/PROJECT_JOURNAL.md, and docs/KNOWN_GOOD_COMMANDS.md first, then summarize current state before suggesting changes.
