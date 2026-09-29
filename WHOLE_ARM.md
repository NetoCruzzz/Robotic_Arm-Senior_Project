# Six-joint arm controller

ESP32 GPIO21 SDA / GPIO22 SCL; PCA9685 address 0x41, 50 Hz.
Use the existing PlatformIO environment and Adafruit driver library.

| Channel | Name | Min us | Home us | Max us |
|---|---|---:|---:|---:|
|0|Claw|1000|1800|2600|
|1|Wrist Rot|1000|1800|2600|
|2|Wrist|1000|1750|2500|
|3|Elbow|1000|1800|2600|
|4|Shoulder|1000|1800|2600|
|5|Base|1000|1800|2600|

These are the user's tested values. Joint limits do not prevent all self-collisions.
Do not run a combined trajectory until its entire path is clear.

## Startup

Upload with servo power off and the arm supported. All outputs start disabled.
The controller cannot sense actual joint position. It does not move automatically
on reset or upload. Support/place the unpowered mechanism at its verified home pose
without forcing gears. Only then enable servo power and send `armhome` followed by
Enter. Initial engagement may still correct position abruptly. Alternatively, use
`ch N` and `arm <known-clear-pulse>` to establish each joint individually; other
enabled joints keep holding. Do not invent initial pulses for an unknown pose.

If servo power is removed, physical positions may change even though the ESP32
remembers its commands. Send `offall` and establish the physical starting pose
again before restoring power. Stored command values are not feedback.

## Commands (115200 baud, Enter after each line)

- `ch 0` through `ch 5`: select a joint without disabling the others.
- `arm 1800`: enable selected joint at a known clear pulse, only if disabled.
- `+` / `-`: jog selected enabled joint by 5 us.


- `stop`: retain the last commanded positions; no timed movement is active.
- `x`: disable selected joint; other joints keep holding.
- `offall`: disable all outputs. The arm may fall; support it.
- `!`: immediate all-output disable without Enter. Press Enter afterward to clear
  the discarded input line. This does not cut electrical power. An I2C fault can
  prevent disabling; the supply output switch remains the physical power cutoff.
- `min`, `max`, `markhome`: record current selected pulse; no movement.
- `home`: legacy alias for markhome, NOT a motion command. Use armhome only from a supported home pose with all outputs disabled.
- `save`: persist selected joint settings; `list`: print the six settings.
- `diag`: register readback; `?`: command help.

A write/readback fault blocks further movement and reports uncertainty;
other joints may still hold their last PWM. Remove power with the mechanism supported.

## Settings and persistence

Edit DEFAULTS for the six fallback values. A fresh `arm-v2` Preferences namespace
uses this table, then saved arm-v2 calibration takes precedence on later boots.
The old `servo-cal` namespace is untouched, including the old calibration.
Changing DEFAULTS does not override already-saved arm-v2 settings.

Multi-joint motion/holding can need much more current than a single-joint test.
The earlier 0.5 A test limit is not a verified whole-arm supply requirement. Confirm
stable servo voltage under the actual load; software cannot measure the bench supply.
VCC stays 3.3 V. Servo-rated power goes to V+ with shared ground.

## What is not implemented

No automatic homing from arbitrary startup poses, position sensing, collision
detection, or extend/open/return sequence. There is no timed home transition; armhome immediately commands the home targets. Establish any necessary
intermediate poses before creating the full sequence.
