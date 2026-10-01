# First wave: CH1 through CH4

Each servo moves from its current commanded position to its active home, then
minimum, maximum, and back home. The order is shoulder (CH4), elbow (CH3), wrist
(CH2), and wrist rotation (CH1), repeated twice. The second pass starts at home.
Claw (CH0) receives no new wave commands and retains its enabled/disabled state.

Targets come from the active calibration: saved flash values loaded at startup,
including any subsequent RAM edits made with min, max, or markhome. All endpoints
are validated before movement; out-of-range calibration blocks the wave.
The wave finishes at home rather than returning to its original starting pose.

Each leg lasts at least one second and scales with pulse distance at an average
175 us/second. Smooth easing has a peak rate of 1.5 times that average. Total time
depends on calibration and starting commands; waveplan prints an estimate.
This traverses the full configured range, so verify clearance for the full path.

Upload with servo power off and the mechanism supported. Establish a clear pose:
use armhome ONLY if physically supported in the verified home pose, or enable each
joint at a known clear starting pulse. Initial arming can jump. Ensure the base is
stable/supported even though it does not participate in the wave.

Send `waveplan` to print the targets without movement. With clearance around the
whole arm and slack in the cables, send `wave` to execute. All of CH1..4 must be
enabled. A target outside any exploration or recorded limit rejects the entire wave.

- `stop` + Enter: cancel immediately on receipt and hold last commanded positions;
  no automatic return after stopping.
- `x` + Enter: cancel wave and disable the selected joint only.
- `offall` or `!`: cancel and disable all outputs; support the arm against falling.
  `!` needs no Enter, but press Enter afterward to discard the rest of that line.
- Communication faults cancel the wave; outputs may still hold their last command.
  Software cannot guarantee a stop if the I2C connection fails.

While waving, read-only commands and channel selection work; jogging, arming,
saving and calibration edits are blocked. No wave starts automatically on reset.
`gohome` remains removed. Saved calibration uses the same arm-v2 namespace.

## Adjusting the gesture later

At the top of main.cpp, WAVE_CHANNELS sets the order (4,3,2,1).
WAVE_US_PER_SECOND controls the average rate of pulse change; WAVE_MIN_LEG_MS
sets the minimum duration per leg, and WAVE_PASSES sets repetitions. Pulse widths
are not angles or measured physical positions. Use waveplan to inspect the active
targets before running a changed calibration.

The code checks pulse limits, not self-collision or supply current. A clear home
pose does not establish clearance for every wave excursion. Check that supply
voltage remains stable with all supporting joints enabled; the old 0.5 A
single-servo test limit may not support this load.
