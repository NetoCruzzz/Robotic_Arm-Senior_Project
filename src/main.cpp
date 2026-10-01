#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>
#include <stdlib.h>
#include <string.h>

constexpr uint8_t PCA_ADDRESS = 0x41;
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr int EXPLORATION_MIN_US = 1000;                                                               // This a set minimum pulse width.
constexpr int EXPLORATION_MAX_US = 2600;                                                               // This a set maximum pulse width.
constexpr int JOG_US = 5;                                                                              // At 50 Hz the PCA9685 resolves about 4.9 us per tick.
Adafruit_PWMServoDriver pwm(PCA_ADDRESS);                                                              // This creates a controller object named pwm and initializes it with the I2C address of the PCA9685.
Preferences prefs;                                                                                     // This creates a "Preferences" object for storing, calibration, data in flash. Ensuring we don't lose data when device is powered off.

// A struct group to calibrate each joint's min, home, and max range to avoid collisions.
struct Calibration {
    int low, home, high;
    Calibration(int l = 0, int h = 0, int u = 0) : low(l), home(h), high(u) {}
};
// Channel map: 0 Claw, 1 Wrist Rot, 2 Wrist.
constexpr int JOINT_COUNT = 5;
const char *JOINT_NAMES[] = {"Claw", "Wrist Rot", "Wrist", "Elbow", "Shoulder"};                // These names are used in the serial command interface.
// User-supplied limits; these do not detect collisions between joints.
const Calibration DEFAULTS[JOINT_COUNT] = {                                                             // These are the default calibration values for each joint, specified in microseconds. Each joint has a min, home, and max value that defines its range of motion.
    {1100,1800,2600}, {1000,1800,2600}, {1000,1750,2500},
    {1000,1800,2600}, {350,1800,2450}
};
Calibration joints[JOINT_COUNT];                                                                        // Creates an array of 5 calibration slots — one for each joint — to store min, home, and max limits while the program runs.
int commanded[JOINT_COUNT] = {};                                                                        // This array holds the last commanded pulse width for each joint, allowing the program to track the current position of each joint in microseconds.
bool enabled[JOINT_COUNT] = {};                                                                         // This array tracks whether each joint is currently enabled or disabled. When a joint = true, it can receive commands to move; when false, it will not respond to movement commands.

// Pose values are pulse widths in microseconds, indexed by PCA9685 channel.
// Keep the configured channel count until CH5 calibration is confirmed.
struct ArmPose { int servo[JOINT_COUNT]; };
ArmPose poseStart = {}, poseTarget = {};
bool poseMoving = false;
uint32_t poseStarted = 0, poseLastFrame = 0, poseDuration = 0;
constexpr uint32_t POSE_MIN_MS = 1000;
constexpr uint32_t POSE_PEAK_US_PER_SECOND = 100;

// Wave targets use the active calibration, in microseconds, NOT angles.
// Move shoulder -> elbow -> wrist -> wrist rotation, then repeat once.
constexpr int WAVE_CHANNELS[] = {4, 3, 2, 1};
constexpr int WAVE_JOINT_COUNT = sizeof(WAVE_CHANNELS) / sizeof(WAVE_CHANNELS[0]);
constexpr int WAVE_LEGS_PER_JOINT = 4; // Current -> home -> min -> max -> home.
constexpr uint32_t WAVE_MIN_LEG_MS = 1000;
constexpr uint32_t WAVE_US_PER_SECOND = 175; // Average pulse change; eased peak is 1.5x.
constexpr int WAVE_PASSES = 2;
bool waving = false;
int waveTargets[WAVE_JOINT_COUNT][WAVE_LEGS_PER_JOINT + 1];
int waveLeg = 0;
uint32_t waveLegStarted = 0, waveLastFrame = 0;
// No position feedback: a reset or loss of servo power invalidates physical position.
// Boot never commands movement. Establish each starting position with arm first.
int channel = 0;
// Per-channel commanded[] replaces the old single pulse variable.
// Per-channel enabled[] lets other joints hold while selecting a new channel.
bool ready = false;
bool storageReady = false;
char line[64];
size_t used = 0;
bool overflow = false;

// Pulse widths in microseconds; these are exploration bounds, not measured travel limits.
int explorationMin(int ch) { return ch == 4 ? 350 : EXPLORATION_MIN_US; }

int explorationMax(int ch) { return ch == 4 ? 2450 : EXPLORATION_MAX_US; }

void keyFor(char *key, int ch, char field) { snprintf(key, 12, "c%d%c", ch, field); }

void printTable()
{
    Serial.println("channel,min_us,home_us,max_us (0 = unrecorded)");
    for (int i = 0; i < JOINT_COUNT; ++i)
        Serial.printf("%d,%d,%d,%d\n", i, joints[i].low, joints[i].home, joints[i].high);
}

void help()
{
    Serial.println("Send one command per line (Enter):");
    Serial.println("diag       read PCA9685 configuration and selected channel registers");
    Serial.println("ch 0..4    select joint; other joints KEEP HOLDING");
    Serial.println("arm <us>   enable at a known clear pulse");
    Serial.println("+ / -      ONE 5 us step; no automatic sweep");
    Serial.println("min / max / markhome   record current pulse (home is a legacy alias)");
    Serial.println("armhome    enable all at home ONLY when physically supported in that pose");
    Serial.println("homeplan   preview active HOME values; no movement");
    Serial.println("H          slowly move enabled joints together to HOME");
    Serial.println("pose       print current commands for physical pose testing");
    Serial.println("waveplan   preview CH4->3->2->1 wave targets; no movement");
    Serial.println("wave       two passes: each CH4..1 goes home -> min -> max -> home");
    Serial.println("stop       cancel motion; hold last commanded positions");
    Serial.println("offall or !  disable ALL pulses; ! works immediately without Enter");
    Serial.println("save       persist selected channel's complete limits to ESP32 flash");
    Serial.println("list       print all recorded values; ? = help");
    Serial.println("x          disable pulses; support the joint first (power remains on)");
    Serial.printf("Default exploration range: %d..%d us; these are NOT proven safe.\n",
                  EXPLORATION_MIN_US, EXPLORATION_MAX_US);
    Serial.println("Configured limits also apply. Motion is NOT collision-aware. No automatic boot movement.");
}

void status()
{
    Serial.printf("CH %d | %s | command %d us | min %d home %d max %d\n",
                  channel, enabled[channel] ? "OUTPUT ENABLED (no position feedback)" : "DISABLED", commanded[channel],
                  joints[channel].low, joints[channel].home, joints[channel].high);
}
// Recorded endpoints further restrict the exploration range; zero means unset.
bool allowed(int value)
{
    const Calibration &c = joints[channel];
    return value >= explorationMin(channel) && value <= explorationMax(channel) &&
           (!c.low || value >= c.low) && (!c.high || value <= c.high);
}

bool readRegister(uint8_t reg, uint8_t &value)
{
    Wire.beginTransmission(PCA_ADDRESS);
    Wire.write(reg);
    uint8_t error = Wire.endTransmission(false);
    if (error) {
        Serial.printf("I2C address/register error %u at 0x%02X\n", error, reg);
        return false;
    }
    if (Wire.requestFrom(PCA_ADDRESS, uint8_t(1)) != 1) {
        Serial.printf("I2C read failed at register 0x%02X\n", reg);
        return false;
    }
    value = Wire.read();
    return true;
}

void diagnostics()
{
    uint8_t mode1, mode2, prescale, bytes[4];
    if (!readRegister(0x00, mode1) || !readRegister(0x01, mode2) ||
        !readRegister(0xFE, prescale)) return;
    for (int i = 0; i < 4; ++i)
        if (!readRegister(0x06 + 4 * channel + i, bytes[i])) return;
    Serial.printf("DIAG CH %d: MODE1=0x%02X MODE2=0x%02X PRESCALE=%u ON=%u OFF=%u\n",
        channel, mode1, mode2, prescale,
        unsigned(bytes[0] | (bytes[1] << 8)), unsigned(bytes[2] | (bytes[3] << 8)));
    Serial.println("Register values are NOT a measurement of the output waveform or servo position.");
}
// MATCH proves the settings arrived; it does NOT prove the servo moved.
bool writeVerifiedPulse(int microseconds)
{
    uint8_t mode, prescale;
    if (!readRegister(0x00, mode) || !readRegister(0xFE, prescale)) return false;
    if ((mode & 0x50) || prescale < 110 || prescale > 130) {
        Serial.println("Unexpected clock/sleep configuration. Stop and send diag.");
        return false;
    }
    // Same 25 MHz nominal oscillator conversion as the library, rounded to a tick.
    uint16_t ticks = uint16_t((double(microseconds) * 25.0 / (prescale + 1)) + 0.5);
    if (pwm.setPWM(channel, 0, ticks) != 0) {
        Serial.println("PWM write FAILED."); return false;
    }
    uint8_t bytes[4];
    for (int i = 0; i < 4; ++i)
        if (!readRegister(0x06 + 4 * channel + i, bytes[i])) return false;
    unsigned on = bytes[0] | (bytes[1] << 8);
    unsigned off = bytes[2] | (bytes[3] << 8);
    if (on != 0 || off != ticks) Serial.println("PWM readback mismatch.");
    return on == 0 && off == ticks;
}

void reportOutputFault()
{
    waving = false;
    poseMoving = false;
    ready = false;
    Serial.println("Output state uncertain; further jogs blocked. Turn servo power off, capture diag, then reset.");
}

// Disable outputs even after a fault; communication failure may prevent stopping.
void disableAll()
{
    waving = false;
    poseMoving = false;
    bool ok = true;
    for (int i = 0; i < 16; ++i) {
        if (pwm.setPWM(i, 0, 4096) != 0) ok = false;
        else if (i < JOINT_COUNT) enabled[i] = false;
    }
    if (!ok) reportOutputFault();
    Serial.println(ok ? "All pulses disabled. Servo power remains on; support the arm." : "Stop NOT confirmed. Cut servo power.");
}
// Use the existing verified writer for a particular channel without changing selection.
bool writeJoint(int ch, int value)
{
    if (ch < 0 || ch >= JOINT_COUNT) return false;
    int selected = channel;
    channel = ch;
    bool ok = allowed(value) && writeVerifiedPulse(value);
    channel = selected;
    if (!ok) { reportOutputFault(); return false; }
    commanded[ch] = value;
    return true;
}

ArmPose homePose()
{
    ArmPose home = {};
    for (int ch = 0; ch < JOINT_COUNT; ++ch) home.servo[ch] = joints[ch].home;
    return home;
}

void printPose()
{
    Serial.println("Commanded pulses only; physical position is NOT measured.");
    for (int ch = 0; ch < JOINT_COUNT; ++ch)
        Serial.printf("CH%d %s: %d us (%s)\n", ch, JOINT_NAMES[ch],
                      commanded[ch], enabled[ch] ? "enabled" : "disabled/stale");
}

bool validatePose(const ArmPose &target)
{
    for (int ch = 0; ch < JOINT_COUNT; ++ch) {
        const Calibration &c = joints[ch];
        if (c.low < explorationMin(ch) || c.high > explorationMax(ch) ||
            c.low >= c.high || c.home < c.low || c.home > c.high ||
            target.servo[ch] < c.low || target.servo[ch] > c.high) {
            Serial.printf("Pose blocked: invalid calibration or target for CH%d.\n", ch);
            return false;
        }
    }
    return true;
}

uint32_t coordinatedDuration(const ArmPose &from, const ArmPose &to)
{
    uint32_t largest = 0;
    for (int ch = 0; ch < JOINT_COUNT; ++ch) {
        uint32_t distance = abs(to.servo[ch] - from.servo[ch]);
        if (distance > largest) largest = distance;
    }
    // Smoothstep's peak slope is 1.5: cap the planned peak pulse rate.
    uint32_t duration = (largest * 1500UL + POSE_PEAK_US_PER_SECOND - 1) /
                        POSE_PEAK_US_PER_SECOND;
    return duration < POSE_MIN_MS ? POSE_MIN_MS : duration;
}

void previewHome()
{
    ArmPose home = homePose();
    Serial.printf("HOME preview: %d configured joints; no movement.\n", JOINT_COUNT);
    for (int ch = 0; ch < JOINT_COUNT; ++ch)
        Serial.printf("CH%d %s: min %d, HOME %d, max %d us; command %d (%s)\n",
                      ch, JOINT_NAMES[ch], joints[ch].low, home.servo[ch],
                      joints[ch].high, commanded[ch], enabled[ch] ? "enabled" : "disabled");
    if (!validatePose(home)) return;
    ArmPose current = {};
    for (int ch = 0; ch < JOINT_COUNT; ++ch) {
        if (!enabled[ch]) {
            Serial.println("H requires every configured joint enabled at a known clear position.");
            return;
        }
        current.servo[ch] = commanded[ch];
    }
    if (!validatePose(current)) return;
    Serial.printf("Estimated HOME move: %lu ms. Verify clearance along the entire path.\n",
                  (unsigned long)coordinatedDuration(current, home));
}

bool moveArmSmooth(const ArmPose &target)
{
    if (!ready || waving || poseMoving) {
        Serial.println("Pose blocked: driver unavailable or another motion is active.");
        return false;
    }
    if (!validatePose(target)) return false;
    ArmPose current = {};
    bool changed = false;
    for (int ch = 0; ch < JOINT_COUNT; ++ch) {
        if (!enabled[ch]) {
            Serial.printf("Pose blocked: CH%d disabled; establish its starting position first.\n", ch);
            return false;
        }
        current.servo[ch] = commanded[ch];
        changed = changed || current.servo[ch] != target.servo[ch];
    }
    if (!validatePose(current)) return false;
    if (!changed) {
        Serial.println("Already at target commands; physical position is not measured.");
        return true;
    }
    // No output changes until every joint and both endpoints have passed validation.
    poseStart = current;
    poseTarget = target;
    poseDuration = coordinatedDuration(current, target);
    poseStarted = poseLastFrame = millis();
    poseMoving = true;
    Serial.printf("Coordinated move started (%lu ms). stop=hold, !=disable all.\n",
                  (unsigned long)poseDuration);
    return true;
}

bool goHome()
{
    return moveArmSmooth(homePose());
}

void updateArmMotion()
{
    if (!poseMoving || !ready) return;
    uint32_t now = millis();
    if (now - poseLastFrame < 20) return;
    poseLastFrame = now;
    uint32_t elapsed = now - poseStarted;
    float t = elapsed >= poseDuration ? 1.0f : float(elapsed) / poseDuration;
    float eased = t * t * (3.0f - 2.0f * t);
    for (int ch = 0; ch < JOINT_COUNT; ++ch) {
        int next = elapsed >= poseDuration ? poseTarget.servo[ch] :
                   int(poseStart.servo[ch] +
                       (poseTarget.servo[ch] - poseStart.servo[ch]) * eased + 0.5f);
        if (next != commanded[ch] && !writeJoint(ch, next)) return;
    }
    if (elapsed >= poseDuration) {
        poseMoving = false;
        Serial.println("Pose complete; joints hold their target commands.");
    }
}
uint32_t waveDuration(int from, int to)
{
    uint32_t distance = from > to ? from - to : to - from;
    uint32_t duration = (distance * 1000UL + WAVE_US_PER_SECOND - 1) / WAVE_US_PER_SECOND;
    return duration < WAVE_MIN_LEG_MS ? WAVE_MIN_LEG_MS : duration;
}

// Validate ALL endpoints before making any movement; never silently clip a wave.
bool prepareWave()
{
    if (!ready) { Serial.println("Driver not ready."); return false; }
    uint32_t totalMs = 0;
    for (int i = 0; i < WAVE_JOINT_COUNT; ++i) {
        int ch = WAVE_CHANNELS[i];
        if (!enabled[ch]) {
            Serial.printf("Wave blocked: CH%d (%s) must be enabled at a known clear position.\n", ch, JOINT_NAMES[ch]);
            return false;
        }
        const Calibration &c = joints[ch];
        int low = max(explorationMin(ch), c.low);
        int high = min(explorationMax(ch), c.high);
        if (!c.low || !c.home || !c.high || c.low >= c.high ||
            c.low < low || c.high > high || c.home < low || c.home > high ||
            commanded[ch] < low || commanded[ch] > high) {
            Serial.printf("Wave blocked: CH%d needs valid min/home/max and a starting command inside %d..%d us.\n", ch, low, high);
            return false;
        }
        waveTargets[i][0] = commanded[ch];
        waveTargets[i][1] = c.home;
        waveTargets[i][2] = c.low;
        waveTargets[i][3] = c.high;
        waveTargets[i][4] = c.home;
        for (int pass = 0; pass < WAVE_PASSES; ++pass) {
            for (int leg = 0; leg < WAVE_LEGS_PER_JOINT; ++leg) {
                int from = pass > 0 && leg == 0 ? c.home : waveTargets[i][leg];
                totalMs += waveDuration(from, waveTargets[i][leg + 1]);
            }
        }
    }
    for (int i = 0; i < WAVE_JOINT_COUNT; ++i)
        Serial.printf("CH%d %s: current %d -> home %d -> min %d -> max %d -> home %d us\n",
                      WAVE_CHANNELS[i], JOINT_NAMES[WAVE_CHANNELS[i]],
                      waveTargets[i][0], waveTargets[i][1], waveTargets[i][2],
                      waveTargets[i][3], waveTargets[i][4]);
    Serial.printf("%d passes, about %lu seconds. Full configured travel; physical collisions are NOT detected.\n",
                  WAVE_PASSES, (unsigned long)((totalMs + 999) / 1000));
    return true;
}
void updateWave()
{
    if (!waving || !ready) return;
    uint32_t now = millis();
    if (now - waveLastFrame < 20) return;
    waveLastFrame = now;
    int legsPerPass = WAVE_JOINT_COUNT * WAVE_LEGS_PER_JOINT;
    int pass = waveLeg / legsPerPass;
    int i = (waveLeg / WAVE_LEGS_PER_JOINT) % WAVE_JOINT_COUNT;
    int leg = waveLeg % WAVE_LEGS_PER_JOINT;
    int from = pass > 0 && leg == 0 ? waveTargets[i][4] : waveTargets[i][leg];
    int to = waveTargets[i][leg + 1];
    uint32_t duration = waveDuration(from, to);
    uint32_t elapsed = now - waveLegStarted;
    float t = elapsed >= duration ? 1.0f : float(elapsed) / duration;
    float eased = t * t * (3.0f - 2.0f * t); // Smooth start/end without overshoot.
    int next = int(from + (to - from) * eased + 0.5f);
    int ch = WAVE_CHANNELS[i];
    if (next != commanded[ch] && !writeJoint(ch, next)) return;
    if (elapsed >= duration) {
        ++waveLeg;
        waveLegStarted = millis(); // Late frames never skip an entire leg.
        if (waveLeg >= legsPerPass * WAVE_PASSES) {
            waving = false;
            Serial.println("Wave complete; CH1..4 are at their home commands and keep holding.");
        }
    }
}
void command(char *text)
{
    char *name = strtok(text, " \t");
    if (!name) return;
    char *argument = strtok(nullptr, " \t");
    char *extra = strtok(nullptr, " \t");
    bool takesNumber = !strcmp(name, "ch") || !strcmp(name, "arm");
    long value = 0;
    if (extra || (takesNumber && !argument) || (!takesNumber && argument)) {
        Serial.println("Invalid command arguments. Send ? for help."); return;
    }
    if (takesNumber) {
        char *end;
        value = strtol(argument, &end, 10);
        if (*end || end == argument || value < 0 || value > 60000) {
            Serial.println("Invalid number."); return;
        }
    }
    if (!strcmp(name, "?")) { help(); return; }
    if (!strcmp(name, "stop")) { waving = false; poseMoving = false; Serial.println("Motion stopped; enabled joints hold their last commanded positions."); return; }
    if (!strcmp(name, "offall")) { disableAll(); return; }
    if (!strcmp(name, "list")) { printTable(); return; }
    if (!strcmp(name, "diag")) { diagnostics(); return; }
    if (!strcmp(name, "pose")) { printPose(); return; }
    if (!strcmp(name, "homeplan")) { previewHome(); return; }
    if (!ready) { Serial.println("PCA9685 unavailable. Check wiring and reset."); return; }
    // Keep stop/disable, status and channel selection responsive while moving.
    // Block edits and jogs so they cannot invalidate the captured start positions.
    if ((waving || poseMoving) && strcmp(name, "ch") && strcmp(name, "x")) {
        Serial.println("Motion is running. Send stop before another motion or calibration edit."); return;
    }
    if (!strcmp(name, "H")) { goHome(); return; }
    if (!strcmp(name, "wave") || !strcmp(name, "waveplan")) {
        if (!prepareWave()) return;
        if (!strcmp(name, "wave")) {
            waveLeg = 0; waveLegStarted = millis(); waveLastFrame = waveLegStarted;
            waving = true;
            Serial.println("Wave started. stop=hold, !=disable all.");
        }
        return;
    }
    if (!strcmp(name, "armhome")) {
        for (int i = 0; i < JOINT_COUNT; ++i) {
            if (enabled[i]) { Serial.println("armhome requires all outputs disabled. Support the arm before using offall."); return; }
        }
        // Initial pulses cannot be ramped from an unknown physical position.
        for (int i = 0; i < JOINT_COUNT; ++i) {
            if (!writeJoint(i, joints[i].home)) return;
            enabled[i] = true;
        }
        Serial.println("All home outputs enabled. Physical pose is not measured.");
        return;
    }
    int &pulse = commanded[channel];
    bool &armed = enabled[channel];
    if (!strcmp(name, "x")) {
        waving = false;
        poseMoving = false;
        if (pwm.setPWM(channel, 0, 4096) != 0) { reportOutputFault(); return; }
        armed = false;
    } else if (!strcmp(name, "ch")) {
        // Selecting another joint never disables or moves any output.
        if (value >= JOINT_COUNT) { Serial.println("Channel must be 0..4."); return; }
        channel = value;
    } else if (!strcmp(name, "arm")) {
        if (armed) { Serial.println("Already armed. Use + or - for small steps."); return; }
        if (!allowed(value)) { Serial.println("Outside exploration/recorded limits."); return; }
        if (!writeVerifiedPulse(value)) { reportOutputFault(); return; }
        pulse = value; armed = true;
    } else if (!strcmp(name, "+") || !strcmp(name, "-")) {
        if (!armed) { Serial.println("Select a known clear starting pulse with arm first."); return; }
        int next = pulse + (!strcmp(name, "+") ? JOG_US : -JOG_US);
        if (!allowed(next)) { Serial.println("Limit reached; no movement commanded."); return; }
        if (!writeVerifiedPulse(next)) { reportOutputFault(); return; }
        pulse = next;
    } else if (!strcmp(name, "min") || !strcmp(name, "max") || !strcmp(name, "home") || !strcmp(name, "markhome")) {
        if (!armed) { Serial.println("Arm and position the joint first."); return; }
        Calibration candidate = joints[channel];
        if (!strcmp(name, "min")) candidate.low = pulse;
        if (!strcmp(name, "max")) candidate.high = pulse;
        if (!strcmp(name, "home") || !strcmp(name, "markhome")) candidate.home = pulse;
        if ((candidate.low && candidate.high && candidate.low >= candidate.high) ||
            (candidate.home && candidate.low && candidate.home < candidate.low) ||
            (candidate.home && candidate.high && candidate.home > candidate.high)) {
            Serial.println("Invalid ordering; require min < max and home inside limits."); return;
        }
        joints[channel] = candidate;
        Serial.println("Recorded in RAM. Use save when min, home and max are set.");
    } else if (!strcmp(name, "save")) {
        Calibration &c = joints[channel];
        if (!c.low || !c.home || !c.high) { Serial.println("Record min, home and max first."); return; }
        if (!storageReady) { Serial.println("Flash unavailable; copy list output instead."); return; }
        char key[12];
        keyFor(key, channel, 'l'); bool ok = prefs.putInt(key, c.low) == sizeof(int);
        keyFor(key, channel, 'h'); ok = (prefs.putInt(key, c.home) == sizeof(int)) && ok;
        keyFor(key, channel, 'u'); ok = (prefs.putInt(key, c.high) == sizeof(int)) && ok;
        Serial.println(ok ? "Saved to ESP32 flash. Copy list output for your notes too." : "Save failed; copy list output.");
    } else { Serial.println("Unknown command. Send ? for help."); return; }
    status();
}

void setup()
{
    Serial.begin(115200); delay(500);
    Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(100000); Wire.setTimeOut(50);
    Serial.println("\nFive-joint arm control | PCA9685 0x41");
    ready = pwm.begin();
    if (!ready) { Serial.println("PCA9685 not found at 0x41."); return; }
    pwm.setPWMFreq(50); delay(10);
    // 4096 sets the PCA9685 FULL-OFF bit.
    for (int i = 0; i < 16; ++i) {
        if (pwm.setPWM(i, 0, 4096) != 0) { reportOutputFault(); return; }
    }
    // New namespace keeps the old servo-cal values untouched. First boot uses the
    // five rows supplied by the user; later save commands persist here instead.
    for (int i = 0; i < JOINT_COUNT; ++i) joints[i] = DEFAULTS[i];
    storageReady = prefs.begin("arm-v2", false);
    // User-confirmed CH0 MAX is CLOSED at 2600 us. OPEN is not yet confirmed.
    // Apply this correction once, preserving MIN/HOME and later calibration saves.
    if (storageReady && !prefs.getBool("c0max2600-v1", false)) {
        int low = prefs.getInt("c0l", DEFAULTS[0].low);
        int home = prefs.getInt("c0h", DEFAULTS[0].home);
        if (low < explorationMin(0) || low >= 2600 || home < low || home > 2600) {
            ready = false;
            Serial.println("CH0 saved MIN/HOME incompatible with MAX 2600; outputs remain disabled.");
            return;
        }
        if (prefs.putInt("c0u", 2600) != sizeof(int) ||
            prefs.putBool("c0max2600-v1", true) != sizeof(bool)) {
            ready = false;
            Serial.println("Could not save CH0 MAX correction; outputs remain disabled.");
            return;
        }
        Serial.println("CH0 MAX corrected to 2600 us (CLOSED); MIN/HOME preserved.");
    }
    // User-requested CH4 MIN update; preserve saved HOME/MAX and later saves.
    if (storageReady && !prefs.getBool("c4min350-v1", false)) {
        int home = prefs.getInt("c4h", DEFAULTS[4].home);
        int high = prefs.getInt("c4u", DEFAULTS[4].high);
        if (high <= 350 || high > explorationMax(4) || home < 350 || home > high) {
            ready = false;
            Serial.println("CH4 saved HOME/MAX incompatible with MIN 350; outputs remain disabled.");
            return;
        }
        if (prefs.putInt("c4l", 350) != sizeof(int) ||
            prefs.putBool("c4min350-v1", true) != sizeof(bool)) {
            ready = false;
            Serial.println("Could not save CH4 MIN update; outputs remain disabled.");
            return;
        }
        Serial.println("CH4 MIN updated to 350 us; HOME/MAX preserved.");
    }
    if (storageReady) for (int i = 0; i < JOINT_COUNT; ++i) {
        char key[12];
        keyFor(key, i, 'l'); int low = prefs.getInt(key, DEFAULTS[i].low);
        keyFor(key, i, 'h'); int home = prefs.getInt(key, DEFAULTS[i].home);
        keyFor(key, i, 'u'); int high = prefs.getInt(key, DEFAULTS[i].high);
        if (low >= explorationMin(i) && high <= explorationMax(i) && low < high && home >= low && home <= high) {
            joints[i] = {low, home, high};
        } else {
            ready = false;
            Serial.printf("Invalid stored calibration CH %d; movement blocked.\n", i);
        }
    }
    if (!storageReady) Serial.println("Flash unavailable: using supplied defaults; save unavailable.");
    for (int i = 0; i < JOINT_COUNT; ++i) Serial.printf("CH%d = %s\n", i, JOINT_NAMES[i]);
    Serial.println("All outputs disabled. Support the arm before enabling or disabling a joint.");
    help(); status();
}

void loop()
{
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '!') { disableAll(); used = 0; overflow = true; continue; }
        // Enter can arrive as CR, LF, or both; empty lines do nothing.
        if (c == '\r' || c == '\n') {
            if (overflow) Serial.println("Input line discarded; send a fresh command.");
            else { line[used] = 0; command(line); }
            used = 0; overflow = false;
        } else if (c == '\b' || c == 127) {
            if (used && !overflow) --used;
        } else if (!overflow) {
            if (used < sizeof(line) - 1) line[used++] = c;
            else overflow = true;
        }
    }
    updateWave();
    updateArmMotion();
    delay(1); // Keep serial commands responsive.
}
