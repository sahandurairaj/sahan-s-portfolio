#define BLYNK_TEMPLATE_ID   "TMPL3oqinQ4os"
#define BLYNK_TEMPLATE_NAME "THEBOXINGBITS"
#define BLYNK_AUTH_TOKEN    "9ZZIEDbM1pZ8rwYB2zaZfS0QIXqt0LrP"

#include <Bluepad32.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

const char* ssid     = "KAL-M Robotics";
const char* password = "Kenmiles15";

const int motor1_enable_pin   = 13;
const int motor2_enable_pin   = 25;
const int motor1_forward_pin  = 12;
const int motor1_backward_pin = 14;
const int motor2_forward_pin  = 27;
const int motor2_backward_pin = 26;
//const int buzzer               = 15;

const int motor_range_max =  200;
const int motor_range_min = -200;

const int servo1_pin = 2;
const int servo2_pin = 4;
const int servo3_pin = 5;
const int servo4_pin = 18;
const int servo5_pin = 19;

Servo servo1, servo2, servo3, servo4, servo5;

ControllerPtr myControllers[BP32_MAX_GAMEPADS] = { nullptr };
bool controllerConnected = false;

int  blynk_j1_x = 127, blynk_j1_y = 127;
int  blynk_j2_x = 127, blynk_j2_y = 127;
bool blynk_fwd   = false;
bool blynk_bwd   = false;
bool blynk_left  = false;
bool blynk_right = false;

bool hookInProgress      = false;
const bool blynkLocked   = true;   // Blynk-only mode, gamepad permanently blocked

// ═════════════════════════════════════════════════════════════════════════════
//  HOOK SEQUENCES
// ═════════════════════════════════════════════════════════════════════════════

void performL1Action() {
  if (hookInProgress) return;
  hookInProgress = true;
  servo1.write(90);  servo2.write(90);
  servo3.write(90);  servo4.write(90);
  servo5.write(45);  servo2.write(40);  servo1.write(180);
  delay(400);
  servo1.write(90);  servo2.write(90);
  servo3.write(90);  servo4.write(90);  servo5.write(90);
  delay(100);
  hookInProgress = false;
}

void performR1Action() {
  if (hookInProgress) return;
  hookInProgress = true;
  servo1.write(90);  servo2.write(90);
  servo3.write(90);  servo4.write(90);  servo5.write(90);
  delay(100);
  servo5.write(135); servo3.write(0);   servo4.write(130);
  delay(400);
  servo1.write(90);  servo2.write(90);
  servo3.write(90);  servo4.write(90);  servo5.write(90);
  delay(100);
  hookInProgress = false;
}

// ═════════════════════════════════════════════════════════════════════════════
//  BLYNK VIRTUAL PIN HANDLERS
// ═════════════════════════════════════════════════════════════════════════════

BLYNK_WRITE(V0)  { blynk_j1_x  = param.asInt(); }
BLYNK_WRITE(V1)  { blynk_j1_y  = param.asInt(); }
BLYNK_WRITE(V2)  { blynk_j2_x  = param.asInt(); }
BLYNK_WRITE(V3)  { blynk_j2_y  = param.asInt(); }
BLYNK_WRITE(V4)  { blynk_fwd   = param.asInt(); }
BLYNK_WRITE(V5)  { blynk_bwd   = param.asInt(); }
BLYNK_WRITE(V6)  { blynk_left  = param.asInt(); }
BLYNK_WRITE(V7)  { blynk_right = param.asInt(); }
BLYNK_WRITE(V8)  { if (param.asInt()) performL1Action(); }
BLYNK_WRITE(V9)  { if (param.asInt()) performR1Action(); }

// ═════════════════════════════════════════════════════════════════════════════
//  MOTOR HELPERS
// ═════════════════════════════════════════════════════════════════════════════

void controlMotor(int forwardPin, int backwardPin, int speed) {
  if (speed > 0) {
    analogWrite(forwardPin,  speed);
    analogWrite(backwardPin, 0);
  } else if (speed < 0) {
    analogWrite(forwardPin,  0);
    analogWrite(backwardPin, -speed);
  } else {
    analogWrite(forwardPin,  0);
    analogWrite(backwardPin, 0);
  }
}

void stopAllMotorsAndServos() {
  controlMotor(motor1_forward_pin, motor1_backward_pin, 0);
  controlMotor(motor2_forward_pin, motor2_backward_pin, 0);
  digitalWrite(motor1_enable_pin, LOW);
  digitalWrite(motor2_enable_pin, LOW);
  servo1.write(90); servo2.write(90); servo3.write(90);
  servo4.write(90); servo5.write(90);
}

void applyMovement(bool fwd, bool bwd, bool left, bool right) {
  if (fwd) {
    digitalWrite(motor1_enable_pin, HIGH);
    digitalWrite(motor2_enable_pin, HIGH);
    controlMotor(motor1_forward_pin, motor1_backward_pin,  motor_range_max);
    controlMotor(motor2_forward_pin, motor2_backward_pin,  motor_range_max);
  } else if (bwd) {
    digitalWrite(motor1_enable_pin, HIGH);
    digitalWrite(motor2_enable_pin, HIGH);
    controlMotor(motor1_forward_pin, motor1_backward_pin,  motor_range_min);
    controlMotor(motor2_forward_pin, motor2_backward_pin,  motor_range_min);
  } else if (left) {
    digitalWrite(motor1_enable_pin, HIGH);
    digitalWrite(motor2_enable_pin, HIGH);
    controlMotor(motor1_forward_pin, motor1_backward_pin,  motor_range_min);
    controlMotor(motor2_forward_pin, motor2_backward_pin,  motor_range_max);
  } else if (right) {
    digitalWrite(motor1_enable_pin, HIGH);
    digitalWrite(motor2_enable_pin, HIGH);
    controlMotor(motor1_forward_pin, motor1_backward_pin,  motor_range_max);
    controlMotor(motor2_forward_pin, motor2_backward_pin,  motor_range_min);
  } else {
    digitalWrite(motor1_enable_pin, LOW);
    digitalWrite(motor2_enable_pin, LOW);
    controlMotor(motor1_forward_pin, motor1_backward_pin,  0);
    controlMotor(motor2_forward_pin, motor2_backward_pin,  0);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  BLUEPAD32 CALLBACKS
// ═════════════════════════════════════════════════════════════════════════════

void onConnectedController(ControllerPtr ctl) {
  if (blynkLocked) return;  // Reject all BT controller connections
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      controllerConnected = true;
      break;
    }
  }
  for (int i = 0; i < 2; i++) {
    digitalWrite(buzzer, HIGH); delay(500);
    digitalWrite(buzzer, LOW);  delay(500);
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      controllerConnected = false;
      stopAllMotorsAndServos();
      break;
    }
  }
  digitalWrite(buzzer, HIGH); delay(500);
  digitalWrite(buzzer, LOW);
}

// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  pinMode(motor1_forward_pin,  OUTPUT);
  pinMode(motor1_backward_pin, OUTPUT);
  pinMode(motor2_forward_pin,  OUTPUT);
  pinMode(motor2_backward_pin, OUTPUT);
  pinMode(motor1_enable_pin,   OUTPUT);
  pinMode(motor2_enable_pin,   OUTPUT);
  pinMode(buzzer,              OUTPUT);

  servo1.attach(servo1_pin);
  servo2.attach(servo2_pin);
  servo3.attach(servo3_pin);
  servo4.attach(servo4_pin);
  servo5.attach(servo5_pin);

  stopAllMotorsAndServos();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect();
}

// ═════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
  Blynk.run();

  BP32.update();  // Still call to keep BT stack alive, but data is never used

  if (!hookInProgress) {
    applyMovement(blynk_fwd, blynk_bwd, blynk_left, blynk_right);
    servo1.write(map(blynk_j1_y, 0,   255, 0, 180));
    servo2.write(map(blynk_j1_x, 255,   0, 0, 180));
    servo3.write(map(blynk_j2_y, 255,   0, 0, 180));
    servo4.write(map(blynk_j2_x, 255,   0, 0, 180));
  }

  delay(2);
}