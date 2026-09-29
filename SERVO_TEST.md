# ESP32 + PCA9685 servo test

PlatformIO environment: `esp32dev`, Arduino framework. Adafruit PWM Servo Driver dependency is already configured.

## Wiring

| ESP32 / supply | PCA9685 |
| --- | --- |
| GPIO 21 | SDA |
| GPIO 22 | SCL |
| 3.3V | VCC (logic supply) |
| GND | GND |
| External servo-rated supply positive (typically 5V; check servo) | V+ |
| External supply negative | GND, shared with ESP32 |

Plug the servo into channel 0: signal to PWM, positive to V+, ground to GND. Check your servo's wire colors/pinout. OE must be low for outputs to work (many boards include a pull-down). Do not power the servo from the ESP32 3.3V pin. Use an external supply rated for the servo's current, including stall current.

Reference: https://learn.adafruit.com/16-channel-pwm-servo-driver/pinouts

## Run

1. Open the Test project in PlatformIO and upload using the Upload button.
2. Open Serial Monitor at 115200 baud; press the ESP32 RESET button if the startup message was missed.
3. Send `c` to command the nominal center (1500 us).
4. Send `s` to repeat center, 1250 us, center, 1750 us every 1.5 seconds.
5. Send `x` to disable control pulses, or `h` for help. Disabling pulses does not disconnect servo power or guarantee braking.

The program starts with all PCA9685 outputs disabled. It assumes a typical positional hobby servo. Actual angles depend on servo calibration; this test does not claim 0/90/180 degrees. A continuous-rotation servo interprets pulse width as speed/direction instead, and its neutral point may differ from 1500 us.

Adjust SDA_PIN, SCL_PIN, PCA_ADDRESS, SERVO_CHANNEL and pulse limits at the top of src/main.cpp to match your hardware. Start with the servo unloaded and stop if it binds or buzzes at a limit. If the driver is not found, check the connections/address and reset after fixing them.

Build: `pio run`
Upload: `pio run --target upload`
Monitor: `pio device monitor --baud 115200`
