# Racer3 Basic Test Checklist

## Current safe order

1. Build the project.
2. Run dry-run first:

   ```cmd
   racer3-basic-motion.exe --dry-run
   ```

3. Confirm the printed relative offsets are tiny and net back to zero.
4. Confirm the RMP controller/network/server stack is already running and configured for the Racer 3.
5. Run enable-only next:

   ```cmd
   racer3-basic-motion.exe --enable-only
   ```

6. Only after enable-only succeeds, run the tiny motion test:

   ```cmd
   racer3-basic-motion.exe --tiny-motion --confirm-motion
   ```

## Important

The tiny motion sequence assumes RMP user units are degrees. Do not run motion until the configured axis user units and actual positions are verified.

The first real hardware milestone is enable then disable with no motion.
