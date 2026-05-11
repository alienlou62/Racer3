# Safety Guidance

This demo is intentionally conservative and built for initial verification of joint-space motion.

## Safety rules

- Assume the robot may move unexpectedly.
- Keep a safe distance from the robot and any tools.
- Ensure the E-stop is available and functional.
- Use very low velocity, acceleration, and deceleration.
- Start with small motions only.
- Do not bypass drive safety or E-stop behavior.
- Do not run on a live robot without a qualified operator present.

## Recommended test pattern

- Confirm the robot is in a safe starting pose.
- Run the program with the robot powered but unloaded.
- Watch the robot move through small joint offsets.
- Verify the robot returns to its starting pose.
- Disable the amplifiers before exiting.

## Notes

- The current demo uses relative joint motion so the robot returns to the starting pose.
- The JSON file in `config/axes.json` now contains conservative Racer3 joint limits and motion parameters based on legacy RapidRobot test-driver data.
- Always verify axis units and motion parameters against the RMP controller setup before increasing motion size.
