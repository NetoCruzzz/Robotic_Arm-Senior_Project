# Pickup development: HOME movement first

This increment adds pose representation and coordinated HOME movement to the
existing firmware. It does not add pickup automation or untested object poses.

## Calibration and channel count

The source currently configures CH0 through CH4 only. CH5 was removed previously;
its confirmed MIN/HOME/MAX values are needed before restoring six-servo control.
Do not use this build as a complete six-servo pickup controller.

| Channel | Joint | Default MIN | Default HOME | Default MAX |
|---|---|---:|---:|---:|
| 0 | Claw | 1100 | 1800 | 2600 |
| 1 | Wrist rotation | 1000 | 1800 | 2600 |
| 2 | Wrist | 1000 | 1750 | 2500 |
| 3 | Elbow | 1000 | 1800 | 2600 |
| 4 | Shoulder | 350 | 1800 | 2450 |

All values are microseconds. Existing saved `arm-v2` calibration takes precedence;
`list` prints the active values. Earlier CH0/CH4 migrations were removed. A new one-time correction sets only
CH0 MAX to the user-confirmed 2600 us, preserving its MIN/HOME and other channels.
CH0 also has a 2600 us exploration ceiling. A separate one-time update sets
CH4 MIN to 350 us, preserving its saved HOME/MAX; its exploration floor is 350 us. Invalid stored calibration blocks movement.

CH0 MAX is confirmed CLOSED at 2600 us. OPEN remains unconfirmed. No claw
motion commands or approach, pickup, lift, or drop poses have been assigned.

## New commands

- `homeplan`: print active HOME targets, limits, enable state, and a time estimate
  when a valid enabled starting pose is available. Does not move anything.
- `pose`: print every configured joint's last commanded pulse and enable state.
  These are not measurements; disabled values may be stale.
- `H`: move all configured joints together to their active HOME values. Requires
  all joints already enabled and valid starting/target poses. Does not arm outputs.
- `stop`: cancel a pose move or wave and hold the last issued commands.
- `x`: cancel motion and disable the selected joint.
- `!` or `offall`: cancel motion and disable all outputs. Support the mechanism.

`H` is distinct from the existing lowercase `home` calibration-recording command.
The existing `armhome` command still enables outputs immediately at HOME and is
only appropriate when the mechanism is already supported in that verified pose.

## First physical test

1. Confirm the actual channel layout and CH5 calibration before testing a full
   six-servo arm. Upload only with servo power off and the arm supported.
2. Send `list`, `homeplan`, and `pose`. Compare active calibration with your notes.
3. Establish a known clear, enabled starting pose with the existing controls.
   Loss of servo power invalidates the assumed physical starting position.
4. For the first HOME movement test, use an empty claw and only a small, clear
   displacement from verified HOME using existing 5 us jogs. Check that coordinated
   motion back to HOME has clearance along the entire path.
5. Send `homeplan`, then `H`. Test `stop` during movement. A later `H` starts from
   the last issued commands. `!` disables outputs if needed.
6. Report the `list` output, confirmed CH5 values, and confirmed claw mapping.
   After HOME is verified, the next increment can capture and preview one
   physically taught APPROACH pose, then test its movement separately.

Motion uses shared smoothstep progress and a 20 ms frame interval. Duration is
at least one second and scales with the largest pulse displacement for a planned
peak pulse-change rate of 100 us/s. PWM quantization and frame delays affect actual
steps. Limits bound pulse targets, not collisions, grip force, or physical speed.

## Reused code and next stages

The new `ArmPose`, `validatePose`, `moveArmSmooth`, and `goHome` use the existing
`joints`, `commanded`, `enabled`, `writeJoint`, and verified PWM output path.
No movement starts at boot. Calibration edits and competing motion commands are
blocked during a pose move; stop commands remain responsive.

After HOME and claw mapping are confirmed, teach and validate one pose at a time:
APPROACH, PICKUP, LIFT, DROP approach, and DROP. Only after each transition is
physically verified should these be joined into the one-object sequence.
Zone commands 1–6 and camera integration are not implemented in this increment.
