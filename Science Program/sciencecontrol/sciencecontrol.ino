#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

/*
  UNO-friendly design:
  - USB Serial menu uses Serial (D0/D1 reserved).
  - PCA9685 uses I2C (A4/A5).
  - RoboClaws are driven via RC/servo PWM inputs (no UART needed).
*/

// ========================= TYPES (MUST BE ABOVE ANY SIGNATURES) =========================
struct ServoState {
  uint8_t channel;
  float currentDeg;
  float targetDeg;
  float speedDegPerSec;
  bool moving;

  uint16_t minUs;
  uint16_t maxUs;
  uint16_t centerUs;

  unsigned long lastUpdateMs;
};

struct Relay {
  const char* label;
  uint8_t pin;
  bool state;
};

enum class MenuState {
  MAIN,
  SERVO_MENU,
  SERVO_1,
  SERVO_2,
  SERVO_3,
  PUMPS_UV_MENU,
  PUMPS_UV_CONTROL,
  ACTUATOR_MENU,
  DRILL_MENU,
  DRILL_M1,
  DRILL_M2,
  DRILL_M3,
  DRILL_AUTO
};

enum class ActDir { STOP, FWD, REV };

struct ActuatorState {
  ActDir dir;
  uint8_t speed; // 0..255
};

enum DrillSeqState {
  DRILL_STEP1,
  DRILL_STEP1_STOP,
  DRILL_STEP2,
  DRILL_STEP2_STOP,
  DRILL_STEP3,
  DRILL_STEP3_STOP,
  DRILL_STEP4,
  DRILL_STEP4_STOP,
  DRILL_STEP5,
  DRILL_STEP5_STOP,
  DRILL_DONE,
  DRILL_ABORTED
};

struct DrillMotorState {
  int8_t direction;   // -1 rev, 0 stop, +1 fwd
  uint8_t speedPct;   // 0..100 magnitude
  int16_t signedPct;  // -100..100 actual command (derived)
};

// ========================= PIN MAP (UNO SAFE) =========================
// I2C: A4 SDA, A5 SCL

// Linear Actuator (H-bridge, 3 pins)
#define ACT_IN1 2
#define ACT_IN2 3
#define ACT_PWM 5  // PWM pin

// Relays (5 pins)
Relay relays[] = {
  {"pump1",   6,  false},
  {"pump2",   7,  false},
  {"pump3",   8,  false},
  {"stirrer", 9,  false},
  {"uv_led",  10, false},
};
static const uint8_t RELAY_COUNT = sizeof(relays) / sizeof(relays[0]);

// Drill PWM pins (servo pulses to RoboClaw RC inputs)
#define DRILL_M1_PIN 4
#define DRILL_M2_PIN 11
#define DRILL_M3_PIN 12

// Servo pulse calibration for RoboClaw RC input
static const uint16_t DRILL_US_MIN    = 1000; // -100%
static const uint16_t DRILL_US_CENTER = 1500; // 0%
static const uint16_t DRILL_US_MAX    = 2000; // +100%

// ========================= GLOBALS =========================

// PCA9685 servos
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);
static const uint16_t SERVO_FREQ = 50;
ServoState servos[3];

// Linear actuator
ActuatorState actuator = { ActDir::STOP, 150 };

// Menus
MenuState menuState = MenuState::MAIN;
int selectedRelay = -1;

// Drill PWM “motors”
Servo drillServo1, drillServo2, drillServo3;
DrillMotorState drillMotors[3] = {
  {0, 40, 0},  // default speed 40%
  {0, 40, 0},
  {0, 40, 0},
};

// Drill auto sequence state
bool drillAutoEnabled = false;     // running sequence
bool drillPaused      = false;
bool drillControlMode = false;     // manual mode (like your original)
bool drillStopReq     = false;

DrillSeqState drillSeqState = DRILL_STEP1;
unsigned long drillStateStartMs = 0;

// ========================= HELPERS =========================
static uint16_t usToTicks(uint16_t us, uint16_t freq) {
  uint32_t ticks = (uint32_t)us * (uint32_t)freq * 4096UL;
  ticks = (ticks + 500000UL) / 1000000UL;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

static uint16_t degToUs(const ServoState &s, float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  float t = deg / 180.0f;
  return (uint16_t)(s.minUs + t * (float)(s.maxUs - s.minUs));
}

static void writeServoDeg(uint8_t idx, float deg) {
  ServoState &s = servos[idx];
  s.currentDeg = deg;
  uint16_t us = degToUs(s, deg);
  uint16_t ticks = usToTicks(us, SERVO_FREQ);
  pca.setPWM(s.channel, 0, ticks);
}

static void applyRelay(uint8_t i) {
  digitalWrite(relays[i].pin, relays[i].state ? HIGH : LOW);
}

static void actuatorApply() {
  switch (actuator.dir) {
    case ActDir::STOP:
      digitalWrite(ACT_IN1, LOW);
      digitalWrite(ACT_IN2, LOW);
      break;
    case ActDir::FWD:
      digitalWrite(ACT_IN1, HIGH);
      digitalWrite(ACT_IN2, LOW);
      break;
    case ActDir::REV:
      digitalWrite(ACT_IN1, LOW);
      digitalWrite(ACT_IN2, HIGH);
      break;
  }
  analogWrite(ACT_PWM, actuator.speed);
}

static const __FlashStringHelper* actDirToStr(ActDir d) {
  switch (d) {
    case ActDir::STOP: return F("STOP");
    case ActDir::FWD:  return F("FWD");
    case ActDir::REV:  return F("REV");
  }
  return F("?");
}

static String readLineNonBlocking() {
  static String buf;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = buf;
      buf = "";
      out.trim();
      return out;
    } else {
      buf += c;
      if (buf.length() > 200) {
        buf = "";
        Serial.println(F("ERR: line too long"));
        return "";
      }
    }
  }
  return "";
}

static bool isInteger(const String& s) {
  if (s.length() == 0) return false;
  int i = 0;
  if (s[0] == '-' || s[0] == '+') i = 1;
  for (; i < (int)s.length(); i++) if (!isDigit(s[i])) return false;
  return true;
}

static int16_t clampPct(int v) {
  if (v < -100) return -100;
  if (v >  100) return  100;
  return (int16_t)v;
}

static uint16_t pctToUs(int16_t pct) {
  pct = clampPct(pct);
  if (pct == 0) return DRILL_US_CENTER;

  if (pct > 0) {
    // map 0..100 to center..max
    return (uint16_t)(DRILL_US_CENTER + (uint32_t)(DRILL_US_MAX - DRILL_US_CENTER) * (uint32_t)pct / 100UL);
  } else {
    // map -100..0 to min..center
    int16_t ap = (int16_t)(-pct);
    return (uint16_t)(DRILL_US_CENTER - (uint32_t)(DRILL_US_CENTER - DRILL_US_MIN) * (uint32_t)ap / 100UL);
  }
}

static void drillApplyMotor(uint8_t i) {
  if (i > 2) return;

  DrillMotorState &m = drillMotors[i];
  int16_t cmd = 0;
  if (m.direction > 0) cmd = (int16_t)m.speedPct;
  else if (m.direction < 0) cmd = (int16_t)(-m.speedPct);
  else cmd = 0;
  m.signedPct = cmd;

  uint16_t us = pctToUs(cmd);

  if (i == 0) drillServo1.writeMicroseconds(us);
  if (i == 1) drillServo2.writeMicroseconds(us);
  if (i == 2) drillServo3.writeMicroseconds(us);
}

static void drillSetMotorPct(uint8_t i, int16_t signedPct) {
  if (i > 2) return;
  signedPct = clampPct(signedPct);

  DrillMotorState &m = drillMotors[i];
  if (signedPct > 0) { m.direction = +1; m.speedPct = (uint8_t)signedPct; }
  else if (signedPct < 0) { m.direction = -1; m.speedPct = (uint8_t)(-signedPct); }
  else { m.direction = 0; m.speedPct = 0; }

  drillApplyMotor(i);
}

static void drillStopAll() {
  drillSetMotorPct(0, 0);
  drillSetMotorPct(1, 0);
  drillSetMotorPct(2, 0);
}

static void drillEnterState(DrillSeqState s) {
  drillSeqState = s;
  drillStateStartMs = millis();

  switch (s) {
    case DRILL_STEP1:      Serial.println(F("DRILL SEQ: STEP1 (M1=+40%, M2=+40%) 42.2s")); break;
    case DRILL_STEP1_STOP: Serial.println(F("DRILL SEQ: STEP1 stop")); break;
    case DRILL_STEP2:      Serial.println(F("DRILL SEQ: STEP2 (M1=+40%, M3=-40%) 21s")); break;
    case DRILL_STEP2_STOP: Serial.println(F("DRILL SEQ: STEP2 stop")); break;
    case DRILL_STEP3:      Serial.println(F("DRILL SEQ: STEP3 (M1=-40%, M3=-20%) 39s")); break;
    case DRILL_STEP3_STOP: Serial.println(F("DRILL SEQ: STEP3 stop")); break;
    case DRILL_STEP4:      Serial.println(F("DRILL SEQ: STEP4 (M2=-40%) 17.5s")); break;
    case DRILL_STEP4_STOP: Serial.println(F("DRILL SEQ: STEP4 stop")); break;
    case DRILL_STEP5:      Serial.println(F("DRILL SEQ: STEP5 (M1=-40%, M2=-40%) 26.7s")); break;
    case DRILL_STEP5_STOP: Serial.println(F("DRILL SEQ: STEP5 stop")); break;
    case DRILL_DONE:       Serial.println(F("DRILL SEQ: DONE")); break;
    case DRILL_ABORTED:    Serial.println(F("DRILL SEQ: ABORTED")); break;
  }
}

// ========================= MENU PRINTS =========================
static void printMainMenu() {
  Serial.println();
  Serial.println(F("=== MAIN MENU ==="));
  Serial.println(F("1) Servos"));
  Serial.println(F("2) Pumps and UV"));
  Serial.println(F("3) Linear Actuator"));
  Serial.println(F("4) Drill"));
  Serial.println(F("h) Help"));
  Serial.println(F("Type option:"));
}

static void printServoSelectMenu() {
  Serial.println();
  Serial.println(F("=== SERVO MENU ==="));
  Serial.println(F("1) Servo 1"));
  Serial.println(F("2) Servo 2"));
  Serial.println(F("3) Servo 3"));
  Serial.println(F("b) Back"));
}

static void printServoControlMenu(uint8_t idx) {
  Serial.println();
  Serial.print(F("=== SERVO ")); Serial.print(idx + 1); Serial.println(F(" ==="));
  Serial.println(F("f | r | stop | goto <deg> | speed <deg/s> | status | b"));
}

static void printPumpsUvMenu() {
  Serial.println();
  Serial.println(F("=== PUMPS AND UV ==="));
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    Serial.print(i + 1); Serial.print(F(") "));
    Serial.print(relays[i].label);
    Serial.print(F(" state="));
    Serial.println(relays[i].state ? F("ON") : F("OFF"));
  }
  Serial.println(F("Pick by name/number, or b"));
}

static void printPumpsUvControlMenu() {
  Serial.println();
  Serial.print(F("=== CONTROL: ")); Serial.print(relays[selectedRelay].label); Serial.println(F(" ==="));
  Serial.println(F("on | off | toggle | status | b"));
}

static void printActuatorMenu() {
  Serial.println();
  Serial.println(F("=== LINEAR ACTUATOR ==="));
  Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

static void printActuatorStatus() {
  Serial.print(F("dir=")); Serial.print(actDirToStr(actuator.dir));
  Serial.print(F(" speed=")); Serial.println(actuator.speed);
}

static void printDrillMenu() {
  Serial.println();
  Serial.println(F("=== DRILL ==="));
  Serial.println(F("1) Motor 1"));
  Serial.println(F("2) Motor 2"));
  Serial.println(F("3) Motor 3"));
  Serial.println(F("4) Auto Sequence"));
  Serial.println(F("b) Back"));
  Serial.println(F("Tip: type 'status' to see motor states."));
}

static void printDrillMotorMenu(uint8_t i) {
  Serial.println();
  Serial.print(F("=== DRILL MOTOR ")); Serial.print(i + 1); Serial.println(F(" ==="));
  Serial.println(F("f | r | stop | speed <0-100> | set <-100..100> | status | b"));
}

static void printDrillAutoMenu() {
  Serial.println();
  Serial.println(F("=== DRILL AUTO SEQUENCE ==="));
  Serial.println(F("start | pause | resume | stop | status | b"));
}

static void drillPrintMotorStatus(uint8_t i) {
  DrillMotorState &m = drillMotors[i];
  Serial.print(F("M")); Serial.print(i + 1);
  Serial.print(F(" dir="));
  Serial.print(m.direction > 0 ? F("FWD") : (m.direction < 0 ? F("REV") : F("STOP")));
  Serial.print(F(" speed=")); Serial.print(m.speedPct);
  Serial.print(F("% cmd=")); Serial.print(m.signedPct);
  Serial.println(F("%"));
}

static void drillPrintAllStatus() {
  drillPrintMotorStatus(0);
  drillPrintMotorStatus(1);
  drillPrintMotorStatus(2);

  Serial.print(F("AUTO=")); Serial.print(drillAutoEnabled ? F("ON") : F("OFF"));
  Serial.print(F(" paused=")); Serial.print(drillPaused ? F("YES") : F("NO"));
  Serial.print(F(" controlMode=")); Serial.print(drillControlMode ? F("YES") : F("NO"));
  Serial.print(F(" stopReq=")); Serial.println(drillStopReq ? F("YES") : F("NO"));
  Serial.print(F(" seqState=")); Serial.println((int)drillSeqState);
}

// ========================= SERVO BACKGROUND UPDATE =========================
static void updateServos() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < 3; i++) {
    ServoState &s = servos[i];
    if (!s.moving) continue;

    unsigned long dtMs = now - s.lastUpdateMs;
    if (dtMs < 20) continue;
    s.lastUpdateMs = now;

    float dt = dtMs / 1000.0f;
    float step = s.speedDegPerSec * dt;
    if (step <= 0) step = 0.1f;

    float diff = s.targetDeg - s.currentDeg;
    if (abs(diff) <= step) {
      s.currentDeg = s.targetDeg;
      s.moving = false;
      writeServoDeg(i, s.currentDeg);
    } else {
      s.currentDeg += (diff > 0 ? step : -step);
      writeServoDeg(i, s.currentDeg);
    }
  }
}

// ========================= DRILL AUTO UPDATE =========================
static void drillUpdateSequence() {
  if (!drillAutoEnabled) return;
  if (drillPaused) return;
  if (drillControlMode) return;

  if (drillStopReq) {
    if (drillSeqState != DRILL_ABORTED) {
      drillEnterState(DRILL_ABORTED);
      drillStopAll();
    }
    return;
  }

  unsigned long now = millis();

  // These match your last working UART logic, converted to %:
  // 13000 -> ~40%, 6500 -> ~20%
  switch (drillSeqState) {
    case DRILL_STEP1:
      drillSetMotorPct(0, +40);
      drillSetMotorPct(1, +40);
      if (now - drillStateStartMs >= 42200UL) { drillStopAll(); drillEnterState(DRILL_STEP1_STOP); }
      break;

    case DRILL_STEP1_STOP:
      drillEnterState(DRILL_STEP2);
      break;

    case DRILL_STEP2:
      drillSetMotorPct(0, +40);
      drillSetMotorPct(2, -40);
      if (now - drillStateStartMs >= 21000UL) { drillStopAll(); drillEnterState(DRILL_STEP2_STOP); }
      break;

    case DRILL_STEP2_STOP:
      drillEnterState(DRILL_STEP3);
      break;

    case DRILL_STEP3:
      drillSetMotorPct(0, -40);
      drillSetMotorPct(2, -20);
      if (now - drillStateStartMs >= 39000UL) { drillStopAll(); drillEnterState(DRILL_STEP3_STOP); }
      break;

    case DRILL_STEP3_STOP:
      drillEnterState(DRILL_STEP4);
      break;

    case DRILL_STEP4:
      drillSetMotorPct(1, -40);
      if (now - drillStateStartMs >= 17500UL) { drillStopAll(); drillEnterState(DRILL_STEP4_STOP); }
      break;

    case DRILL_STEP4_STOP:
      drillEnterState(DRILL_STEP5);
      break;

    case DRILL_STEP5:
      drillSetMotorPct(0, -40);
      drillSetMotorPct(1, -40);
      if (now - drillStateStartMs >= 26700UL) { drillStopAll(); drillEnterState(DRILL_STEP5_STOP); }
      break;

    case DRILL_STEP5_STOP:
      drillEnterState(DRILL_DONE);
      drillAutoEnabled = false; // stop running after done
      break;

    case DRILL_DONE:
    case DRILL_ABORTED:
      drillAutoEnabled = false;
      break;
  }
}

// ========================= COMMAND HANDLERS =========================
static void servoGoTo(uint8_t idx, float deg) {
  if (deg < 0) deg = 0;
  if (deg > 180) deg = 180;
  servos[idx].targetDeg = deg;
  servos[idx].moving = true;
  servos[idx].lastUpdateMs = millis();
}

static void handleServoControl(uint8_t idx, const String& cmd) {
  if (cmd == "b") { menuState = MenuState::SERVO_MENU; printServoSelectMenu(); return; }
  if (cmd.equalsIgnoreCase("f")) { servoGoTo(idx, 180); return; }
  if (cmd.equalsIgnoreCase("r")) { servoGoTo(idx, 0); return; }
  if (cmd.equalsIgnoreCase("stop")) { servos[idx].moving = false; return; }
  if (cmd.equalsIgnoreCase("status")) { printServoControlMenu(idx); return; }

  if (cmd.startsWith("goto ")) { servoGoTo(idx, cmd.substring(5).toFloat()); return; }
  if (cmd.startsWith("speed ")) {
    float spd = cmd.substring(6).toFloat();
    if (spd <= 0) spd = 1;
    servos[idx].speedDegPerSec = spd;
    return;
  }
  Serial.println(F("Unknown servo cmd"));
}

static int findRelayByName(const String& name) {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (name.equalsIgnoreCase(relays[i].label)) return i;
  }
  return -1;
}

static void handlePumpsUvMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }

  int idx = -1;
  if (isInteger(cmd)) {
    int n = cmd.toInt();
    if (n >= 1 && n <= (int)RELAY_COUNT) idx = n - 1;
  } else {
    idx = findRelayByName(cmd);
  }

  if (idx < 0) { Serial.println(F("Unknown selection")); return; }

  selectedRelay = idx;
  menuState = MenuState::PUMPS_UV_CONTROL;
  printPumpsUvControlMenu();
}

static void handlePumpsUvControl(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::PUMPS_UV_MENU; printPumpsUvMenu(); return; }

  if (cmd.equalsIgnoreCase("on"))  { relays[selectedRelay].state = true;  applyRelay(selectedRelay); return; }
  if (cmd.equalsIgnoreCase("off")) { relays[selectedRelay].state = false; applyRelay(selectedRelay); return; }
  if (cmd.equalsIgnoreCase("toggle")) { relays[selectedRelay].state = !relays[selectedRelay].state; applyRelay(selectedRelay); return; }

  if (cmd.equalsIgnoreCase("status")) {
    Serial.println(relays[selectedRelay].state ? F("ON") : F("OFF"));
    return;
  }

  Serial.println(F("Use on/off/toggle/status/b"));
}

static void handleActuatorMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }

  if (cmd.equalsIgnoreCase("f")) { actuator.dir = ActDir::FWD; actuatorApply(); printActuatorStatus(); return; }
  if (cmd.equalsIgnoreCase("r")) { actuator.dir = ActDir::REV; actuatorApply(); printActuatorStatus(); return; }
  if (cmd.equalsIgnoreCase("stop")) { actuator.dir = ActDir::STOP; actuatorApply(); printActuatorStatus(); return; }

  if (cmd.startsWith("speed ")) {
    int sp = cmd.substring(6).toInt();
    if (sp < 0) sp = 0;
    if (sp > 255) sp = 255;
    actuator.speed = (uint8_t)sp;
    actuatorApply();
    printActuatorStatus();
    return;
  }

  if (cmd.equalsIgnoreCase("status")) { printActuatorStatus(); return; }
  Serial.println(F("Unknown actuator cmd"));
}

// ---- Drill menus ----
static void handleDrillMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (cmd == "1") { menuState = MenuState::DRILL_M1; printDrillMotorMenu(0); return; }
  if (cmd == "2") { menuState = MenuState::DRILL_M2; printDrillMotorMenu(1); return; }
  if (cmd == "3") { menuState = MenuState::DRILL_M3; printDrillMotorMenu(2); return; }
  if (cmd == "4") { menuState = MenuState::DRILL_AUTO; printDrillAutoMenu(); return; }
  if (cmd.equalsIgnoreCase("status")) { drillPrintAllStatus(); return; }
  Serial.println(F("Pick 1/2/3/4, status, or b"));
}

static void handleDrillMotor(uint8_t i, const String& cmd) {
  if (cmd == "b") { menuState = MenuState::DRILL_MENU; printDrillMenu(); return; }
  if (cmd.equalsIgnoreCase("status")) { drillPrintMotorStatus(i); return; }

  // Manual control cancels auto sequence movement (but doesn't erase state)
  drillControlMode = true;
  drillAutoEnabled = false;
  drillPaused = false;

  if (cmd.equalsIgnoreCase("f")) { drillMotors[i].direction = +1; drillApplyMotor(i); return; }
  if (cmd.equalsIgnoreCase("r")) { drillMotors[i].direction = -1; drillApplyMotor(i); return; }
  if (cmd.equalsIgnoreCase("stop")) { drillMotors[i].direction = 0; drillApplyMotor(i); return; }

  if (cmd.startsWith("speed ")) {
    int v = cmd.substring(6).toInt();
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    drillMotors[i].speedPct = (uint8_t)v;
    // re-apply at new magnitude
    drillApplyMotor(i);
    return;
  }

  if (cmd.startsWith("set ")) {
    int v = cmd.substring(4).toInt();
    drillSetMotorPct(i, (int16_t)v);
    return;
  }

  Serial.println(F("Use: f | r | stop | speed <0-100> | set <-100..100> | status | b"));
}

static void handleDrillAuto(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::DRILL_MENU; printDrillMenu(); return; }

  if (cmd.equalsIgnoreCase("status")) { drillPrintAllStatus(); return; }

  if (cmd.equalsIgnoreCase("start")) {
    drillStopReq = false;
    drillPaused = false;
    drillControlMode = false;
    drillAutoEnabled = true;
    drillStopAll();
    drillEnterState(DRILL_STEP1);
    Serial.println(F("OK: Drill auto sequence started."));
    return;
  }

  if (cmd.equalsIgnoreCase("pause")) {
    drillPaused = true;
    Serial.println(F("OK: paused (motors keep current command)."));
    return;
  }

  if (cmd.equalsIgnoreCase("resume")) {
    if (drillSeqState == DRILL_ABORTED || drillSeqState == DRILL_DONE) {
      Serial.println(F("NOTE: done/aborted. Use 'start' to restart."));
      return;
    }
    drillPaused = false;
    drillAutoEnabled = true;
    drillControlMode = false;
    Serial.println(F("OK: resumed."));
    return;
  }

  if (cmd.equalsIgnoreCase("stop")) {
    drillStopReq = true;
    drillAutoEnabled = false;
    drillPaused = false;
    drillEnterState(DRILL_ABORTED);
    drillStopAll();
    Serial.println(F("OK: stopped + aborted."));
    return;
  }

  Serial.println(F("Use: start | pause | resume | stop | status | b"));
}

// ========================= MAIN HANDLER =========================
static void handleMain(const String& cmd) {
  if (cmd == "1" || cmd.equalsIgnoreCase("servos")) {
    menuState = MenuState::SERVO_MENU;
    printServoSelectMenu();
    return;
  }
  if (cmd == "2" || cmd.equalsIgnoreCase("pumps") || cmd.equalsIgnoreCase("uv")) {
    menuState = MenuState::PUMPS_UV_MENU;
    printPumpsUvMenu();
    return;
  }
  if (cmd == "3" || cmd.equalsIgnoreCase("actuator") || cmd.equalsIgnoreCase("linear")) {
    menuState = MenuState::ACTUATOR_MENU;
    printActuatorMenu();
    printActuatorStatus();
    return;
  }
  if (cmd == "4" || cmd.equalsIgnoreCase("drill")) {
    menuState = MenuState::DRILL_MENU;
    printDrillMenu();
    return;
  }
  if (cmd == "h" || cmd.equalsIgnoreCase("help")) {
    printMainMenu();
    return;
  }
  Serial.println(F("Unknown option"));
}

static void handleServoSelect(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (cmd == "1") { menuState = MenuState::SERVO_1; printServoControlMenu(0); return; }
  if (cmd == "2") { menuState = MenuState::SERVO_2; printServoControlMenu(1); return; }
  if (cmd == "3") { menuState = MenuState::SERVO_3; printServoControlMenu(2); return; }
  Serial.println(F("Pick 1/2/3 or b"));
}

// ========================= SETUP / LOOP =========================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // PCA9685
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ);

  // PCA servos init
  for (uint8_t i = 0; i < 3; i++) {
    servos[i].channel = i; // channels 0,1,2
    servos[i].minUs = 500;
    servos[i].maxUs = 2500;
    servos[i].centerUs = 1500;
    servos[i].speedDegPerSec = 60.0f;
    servos[i].currentDeg = 90.0f;
    servos[i].targetDeg = 90.0f;
    servos[i].moving = false;
    servos[i].lastUpdateMs = millis();
    writeServoDeg(i, 90.0f);
  }

  // Relays init
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(relays[i].pin, OUTPUT);
    relays[i].state = false;
    applyRelay(i);
  }

  // Actuator init
  pinMode(ACT_IN1, OUTPUT);
  pinMode(ACT_IN2, OUTPUT);
  pinMode(ACT_PWM, OUTPUT);
  actuatorApply();

  // Drill PWM init (attach + neutral)
  drillServo1.attach(DRILL_M1_PIN);
  drillServo2.attach(DRILL_M2_PIN);
  drillServo3.attach(DRILL_M3_PIN);
  drillStopAll();

  Serial.println(F("\nSystem ready (UNO + PWM RoboClaw mode)."));
  Serial.println(F("Drill PWM pins: M1=D4, M2=D11, M3=D12 (1000us=-100%, 1500us=0%, 2000us=+100%)."));
  printMainMenu();
}

void loop() {
  // Background updates
  updateServos();
  drillUpdateSequence();

  // Menu input
  String cmd = readLineNonBlocking();
  if (cmd.length() == 0) return;

  // Global shortcut
  if (cmd.equalsIgnoreCase("main")) {
    menuState = MenuState::MAIN;
    printMainMenu();
    return;
  }

  switch (menuState) {
    case MenuState::MAIN:             handleMain(cmd); break;

    case MenuState::SERVO_MENU:       handleServoSelect(cmd); break;
    case MenuState::SERVO_1:          handleServoControl(0, cmd); break;
    case MenuState::SERVO_2:          handleServoControl(1, cmd); break;
    case MenuState::SERVO_3:          handleServoControl(2, cmd); break;

    case MenuState::PUMPS_UV_MENU:    handlePumpsUvMenu(cmd); break;
    case MenuState::PUMPS_UV_CONTROL: handlePumpsUvControl(cmd); break;

    case MenuState::ACTUATOR_MENU:    handleActuatorMenu(cmd); break;

    case MenuState::DRILL_MENU:       handleDrillMenu(cmd); break;
    case MenuState::DRILL_M1:         handleDrillMotor(0, cmd); break;
    case MenuState::DRILL_M2:         handleDrillMotor(1, cmd); break;
    case MenuState::DRILL_M3:         handleDrillMotor(2, cmd); break;
    case MenuState::DRILL_AUTO:       handleDrillAuto(cmd); break;
  }
}