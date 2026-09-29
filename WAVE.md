# First wave: CH1 through CH4

This is a small sequential motion for initial testing, not yet a full raised-hand
gesture. Starting from the current commanded pose, the shoulder (CH4) moves
+75 us then returns; elbow (CH3), wrist (CH2) and wrist rotation (CH1) do the same.
The sequence repeats twice, about 32 seconds total. Claw (CH0) and base (CH5)
receive no new wave commands; their existing enabled/disabled state is retained.

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
WAVE_OFFSET_US follows that same order. Positive values increase pulse width;
negative values reverse the excursion. WAVE_LEG_MS sets the time for each outward
or return movement, and WAVE_PASSES sets repetitions. Initially use the 75 us
offsets and inspect the actual directions. Wrist CH2 positive was observed to bend
toward the front of the hand. Directions of other joints depend on mounting.

The code checks pulse limits, not self-collision or supply current. A clear home
pose does not establish clearance for every wave excursion. Check that supply
voltage remains stable with all supporting joints enabled; the old 0.5 A
single-servo test limit may not support this load.
