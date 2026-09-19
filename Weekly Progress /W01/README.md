# Project README


---

### 1. Objective
i needed to control the boxing bots using blynk app

---

### 2. Components Used
the boxing bots kit 

---

### 3. Working Principle
the phone is connected using router to the robot
---

### 5. Code
/*
 * ESP32 Blynk-Controlled Differential Drive RC Robot
 * ---------------------------------------------------
 * Board:    ESP32 DevKit (ESP32 Arduino Core 3.x)
 * Library:  Blynk (IoT / Blynk.Cloud), install "Blynk" by Volodymyr
 *           Shymanskyy via Library Manager (the current Blynk IoT major
 *           version, not the old "Blynk (legacy)" package).
 * Driver:   L298N Dual H-Bridge Motor Driver
 *
 * Wiring:
 *   Left  motors -> ENA = GPIO13, IN1 = GPIO12, IN2 = GPIO14
 *   Right motors -> ENB = GPIO25, IN3 = GPIO27, IN4 = GPIO26
 *   Buzzer -> GPIO23
 *   LED (toggle) -> GPIO22
 *   Red reverse/backup light -> GPIO21
 *
 * Blynk app widgets (create these in the Blynk IoT console/app):
 *   Widget 1: Joystick, mode = SPLIT.
 *     - Y output -> Virtual Pin V0 (drive: up = forward, down = reverse)
 *     - X output -> any spare/unused virtual pin (not read by this sketch)
 *   Widget 2: Joystick, mode = SPLIT.
 *     - X output -> Virtual Pin V1 (steer: right = positive, left = negative)
 *     - Y output -> any spare/unused virtual pin (not read by this sketch)
 *   Both joysticks: set the output range to -100..100 in the widget's
 *     datastream settings so the values match DEADZONE_RAW/STICK_RAW_MAX
 *     below.
 *   Widget 3: Button, mode = PUSH -> Virtual Pin V2 (horn: buzzer sounds
 *     while held).
 *   Widget 4: Button/Switch, mode = SWITCH -> Virtual Pin V3 (LED toggle).
 *
 * Controls:
 *   Joystick 1 (V0) -> Drive (forward / reverse)
 *   Joystick 2 (V1) -> Steering (left / right)
 *   Button   (V2)   -> Horn / buzzer (on while pressed)
 *   Switch   (V3)   -> LED toggle (on/off)
 *   (automatic)     -> Red light on GPIO21 while the car is driving backward
 *
 * Behavior:
 *   - Proportional drive + steering mixed using standard differential
 *     (tank) drive mixing: left = drive + steer, right = drive - steer.
 *   - Joystick dead zone (rescaled) eliminates drift with no jump at the edge.
 *   - Motor speed changes are ramped (slewed) toward the target speed
 *     instead of snapping instantly, for smooth acceleration/deceleration.
 *   - Fail-safe: if the WiFi/Blynk connection drops, both motors and the
 *     buzzer stop immediately.
 */

#define BLYNK_TEMPLATE_ID   "TMPL374LRDIkl"
#define BLYNK_TEMPLATE_NAME "rc car"
#define BLYNK_AUTH_TOKEN    "g_GaTcpC_QsuzWseWG9sAMIOpDtambu8"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "PRAVEENA";
char pass[] = "Praveena@123";

// Virtual pin assignments (must match the widgets configured in the Blynk app)
#define VPIN_DRIVE V0
#define VPIN_STEER V1
#define VPIN_HORN  V2
#define VPIN_LED   V3

// ----------------------------------------------------------------------
// PWM compatibility shim: ESP32 Arduino Core 3.x uses the pin-based LEDC
// API (ledcAttach/ledcWrite(pin,...)). Core 2.x uses the channel-based API
// (ledcSetup/ledcAttachPin/ledcWrite(channel,...)). This block picks the
// right one automatically so the sketch compiles on either core version.
// ----------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL) && \
    ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  #define USE_NEW_LEDC_API 1
#else
  #define USE_NEW_LEDC_API 0
#endif

#if !USE_NEW_LEDC_API
static const uint8_t LEFT_PWM_CHANNEL  = 0;
static const uint8_t RIGHT_PWM_CHANNEL = 1;
#endif

// ----------------------------------------------------------------------
// Pin definitions
// ----------------------------------------------------------------------
static const uint8_t PIN_ENA = 13;  // Left motors  - PWM enable
static const uint8_t PIN_IN1 = 12;  // Left motors  - direction A
static const uint8_t PIN_IN2 = 14;  // Left motors  - direction B

static const uint8_t PIN_ENB = 25;  // Right motors - PWM enable
static const uint8_t PIN_IN3 = 27;  // Right motors - direction A
static const uint8_t PIN_IN4 = 26;  // Right motors - direction B

static const uint8_t PIN_BUZZER = 23; // Buzzer: on while horn button is held
static const uint8_t PIN_LED    = 22; // LED: follows the Blynk switch state
static const uint8_t PIN_REVERSE_LED = 21; // Red backup light: on while reversing

// ----------------------------------------------------------------------
// PWM configuration
// ----------------------------------------------------------------------
static const uint32_t PWM_FREQ_HZ  = 20000;  // 20 kHz - inaudible, smooth for motors
static const uint8_t  PWM_RES_BITS = 8;       // 0-255 duty resolution
static const int16_t  PWM_MAX      = 255;

// ----------------------------------------------------------------------
// Joystick / control tuning
// ----------------------------------------------------------------------
// Blynk joystick widgets in this sketch are expected to output -100..100.
static const int32_t STICK_RAW_MAX = 100;
static const int32_t DEADZONE_RAW  = 8;     // Dead zone applied to raw joystick readings

static const int16_t RAMP_STEP       = 35;  // PWM units changed per ramp interval
static const uint32_t RAMP_INTERVAL_MS = 10; // How often the ramp step is applied

// Motor trim: compensates for mechanical mismatch between the left and
// right sides. If the robot drifts RIGHT while driving straight, raise
// RIGHT_TRIM (or lower LEFT_TRIM); if it drifts LEFT, do the opposite.
static const float LEFT_TRIM  = 1.00f;
static const float RIGHT_TRIM = 1.06f;

static const uint32_t DEBUG_PRINT_INTERVAL_MS = 200;

// ----------------------------------------------------------------------
// Globals
// ----------------------------------------------------------------------
int32_t driveRaw = 0; // Latest raw value from the drive joystick (V0), -100..100
int32_t steerRaw = 0; // Latest raw value from the steer joystick (V1), -100..100

int16_t targetLeftPWM  = 0;
int16_t targetRightPWM = 0;

int16_t currentLeftPWM  = 0;
int16_t currentRightPWM = 0;

unsigned long lastRampTime  = 0;
unsigned long lastDebugTime = 0;

bool ledState = false;

// ----------------------------------------------------------------------
// Forward declarations
// ----------------------------------------------------------------------
void setLeftMotor(int16_t speedSigned);
void setRightMotor(int16_t speedSigned);
void stopMotors();
void recomputeMixing();
void updateRamp();
void printDebug();
int16_t rampTowards(int16_t current, int16_t target, int16_t step);
float clampf(float v, float lo, float hi);
float applyDeadzoneScaled(int32_t raw, int32_t deadzone, int32_t maxRaw);

// ----------------------------------------------------------------------
// Blynk virtual pin handlers
// ----------------------------------------------------------------------
BLYNK_WRITE(VPIN_DRIVE) {
  driveRaw = param.asInt();
  recomputeMixing();
}

BLYNK_WRITE(VPIN_STEER) {
  steerRaw = param.asInt();
  recomputeMixing();
}

BLYNK_WRITE(VPIN_HORN) {
  bool hornOn = param.asInt() != 0;
  digitalWrite(PIN_BUZZER, hornOn ? HIGH : LOW);
}

BLYNK_WRITE(VPIN_LED) {
  ledState = param.asInt() != 0;
  digitalWrite(PIN_LED, ledState ? HIGH : LOW);
}

BLYNK_CONNECTED() {
  Blynk.virtualWrite(VPIN_LED, ledState ? 1 : 0);
}

// ----------------------------------------------------------------------
// setup()
// ----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("ESP32 Blynk Differential Drive Robot - starting up");

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_REVERSE_LED, OUTPUT);
  digitalWrite(PIN_REVERSE_LED, LOW);

#if USE_NEW_LEDC_API
  ledcAttach(PIN_ENA, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttach(PIN_ENB, PWM_FREQ_HZ, PWM_RES_BITS);
#else
  ledcSetup(LEFT_PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_ENA, LEFT_PWM_CHANNEL);
  ledcSetup(RIGHT_PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_ENB, RIGHT_PWM_CHANNEL);
#endif

  stopMotors();

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  lastRampTime  = millis();
  lastDebugTime = millis();
}

// ----------------------------------------------------------------------
// loop()
// ----------------------------------------------------------------------
void loop() {
  Blynk.run();

  if (!Blynk.connected()) {
    driveRaw = 0;
    steerRaw = 0;
    targetLeftPWM  = 0;
    targetRightPWM = 0;
    currentLeftPWM  = 0;
    currentRightPWM = 0;
    stopMotors();
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_REVERSE_LED, LOW);
  }

  updateRamp();
  printDebug();
}

// ----------------------------------------------------------------------
// recomputeMixing() - applies dead zone, mixes drive/steer, sets targets
// ----------------------------------------------------------------------
void recomputeMixing() {
  // NOTE: the drive axis is inverted here because pushing the Blynk
  // joystick "up" (positive Y) was driving the car backward.
  float driveInput = -applyDeadzoneScaled(driveRaw, DEADZONE_RAW, STICK_RAW_MAX);
  float steerInput = applyDeadzoneScaled(steerRaw, DEADZONE_RAW, STICK_RAW_MAX);

  driveInput = clampf(driveInput, -1.0f, 1.0f);
  steerInput = clampf(steerInput, -1.0f, 1.0f);

  float leftNorm  = driveInput + steerInput;
  float rightNorm = driveInput - steerInput;

  float maxMag = fmaxf(fabsf(leftNorm), fmaxf(fabsf(rightNorm), 1.0f));
  leftNorm  /= maxMag;
  rightNorm /= maxMag;

  leftNorm  = clampf(leftNorm  * LEFT_TRIM,  -1.0f, 1.0f);
  rightNorm = clampf(rightNorm * RIGHT_TRIM, -1.0f, 1.0f);

  targetLeftPWM  = (int16_t)round(leftNorm  * PWM_MAX);
  targetRightPWM = (int16_t)round(rightNorm * PWM_MAX);
}

// ----------------------------------------------------------------------
// updateRamp() - smoothly slews current PWM toward target PWM
// ----------------------------------------------------------------------
void updateRamp() {
  unsigned long now = millis();
  if (now - lastRampTime < RAMP_INTERVAL_MS) {
    return;
  }
  lastRampTime = now;

  currentLeftPWM  = rampTowards(currentLeftPWM, targetLeftPWM, RAMP_STEP);
  currentRightPWM = rampTowards(currentRightPWM, targetRightPWM, RAMP_STEP);

  setLeftMotor(currentLeftPWM);
  setRightMotor(currentRightPWM);

  bool reversing = (currentLeftPWM + currentRightPWM) < 0;
  digitalWrite(PIN_REVERSE_LED, reversing ? HIGH : LOW);
}

int16_t rampTowards(int16_t current, int16_t target, int16_t step) {
  if (current < target) {
    current += step;
    if (current > target) current = target;
  } else if (current > target) {
    current -= step;
    if (current < target) current = target;
  }
  return current;
}

// ----------------------------------------------------------------------
// Motor helper functions
// ----------------------------------------------------------------------
void setLeftMotor(int16_t speedSigned) {
  speedSigned = (int16_t)constrain(speedSigned, -PWM_MAX, PWM_MAX);

  if (speedSigned > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (speedSigned < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }

#if USE_NEW_LEDC_API
  ledcWrite(PIN_ENA, (uint32_t)abs(speedSigned));
#else
  ledcWrite(LEFT_PWM_CHANNEL, (uint32_t)abs(speedSigned));
#endif
}

void setRightMotor(int16_t speedSigned) {
  speedSigned = (int16_t)constrain(speedSigned, -PWM_MAX, PWM_MAX);

  if (speedSigned > 0) {
    digitalWrite(PIN_IN3, HIGH);
    digitalWrite(PIN_IN4, LOW);
  } else if (speedSigned < 0) {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, HIGH);
  } else {
    digitalWrite(PIN_IN3, LOW);
    digitalWrite(PIN_IN4, LOW);
  }

#if USE_NEW_LEDC_API
  ledcWrite(PIN_ENB, (uint32_t)abs(speedSigned));
#else
  ledcWrite(RIGHT_PWM_CHANNEL, (uint32_t)abs(speedSigned));
#endif
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
#if USE_NEW_LEDC_API
  ledcWrite(PIN_ENA, 0);
  ledcWrite(PIN_ENB, 0);
#else
  ledcWrite(LEFT_PWM_CHANNEL, 0);
  ledcWrite(RIGHT_PWM_CHANNEL, 0);
#endif
}

// ----------------------------------------------------------------------
// Debug output
// ----------------------------------------------------------------------
void printDebug() {
  unsigned long now = millis();
  if (now - lastDebugTime < DEBUG_PRINT_INTERVAL_MS) {
    return;
  }
  lastDebugTime = now;

  Serial.print("Drive: ");
  Serial.print(driveRaw);
  Serial.print("  Steer: ");
  Serial.print(steerRaw);
  Serial.print("  LeftPWM: ");
  Serial.print(currentLeftPWM);
  Serial.print("  RightPWM: ");
  Serial.print(currentRightPWM);
  Serial.print("  Blynk: ");
  Serial.println(Blynk.connected() ? "connected" : "DISCONNECTED");
}

// ----------------------------------------------------------------------
// Utility
// ----------------------------------------------------------------------
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float applyDeadzoneScaled(int32_t raw, int32_t deadzone, int32_t maxRaw) {
  int32_t absRaw = abs(raw);
  if (absRaw <= deadzone) {
    return 0.0f;
  }
  float scaled = (float)(absRaw - deadzone) / (float)(maxRaw - deadzone);
  scaled = clampf(scaled, 0.0f, 1.0f);
  return (raw < 0) ? -scaled : scaled;
}
---

