# ESP32 robot arm controller

PlatformIO / Arduino firmware for a six-joint robot arm using an ESP32 and an
Adafruit PCA9685 PWM servo driver.

## Hardware

- Board: ESP32 Dev Module (`esp32dev`)
- I2C: GPIO21 SDA, GPIO22 SCL
- PCA9685: address `0x41`, PWM frequency 50 Hz
- Serial monitor: 115200 baud
- Channels: 0 claw, 1 wrist rotation, 2 wrist, 3 elbow, 4 shoulder, 5 base

## Build and upload

Open this folder with PlatformIO, or run:

```sh
pio run
pio run --target upload
pio device monitor
```

Upload with servo power off and the arm supported. Outputs start disabled.
Send `?` for commands and `list` for calibration. Establish a known clear physical
pose before enabling servos; the controller has no position or collision feedback.

## Calibration

Saved ESP32 flash values override `DEFAULTS` in `src/main.cpp`. This version
includes a one-time correction of channel 0 to minimum 1100 us and maximum
3000 us, preserving its saved home. Later calibration saves remain persistent.
The `min` and `max` commands record the current pulse; `save` stores the selected
channel's settings. Pulse limits are configuration values, not verified mechanical
travel limits.

## Project notes

- [Whole-arm operation](WHOLE_ARM.md)
- [Wave commands](WAVE.md)
- [Earlier servo testing](SERVO_TEST.md)

These notes include earlier tuning values. Consult `src/main.cpp` for current
wave parameters and `list` for the device's saved calibration.
