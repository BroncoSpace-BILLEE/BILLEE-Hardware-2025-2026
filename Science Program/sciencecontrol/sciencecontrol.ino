#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

/*
  Robot Controller — Arduino UNO + PCA9685 + RoboClaw (RC-PWM mode)
  ------------------------------------------------------------------
  SERVO MODEL (360-degree continuous rotation, e.g. DIY Mall 25Kg/cm):
    These are NOT positional servos. PWM pulse width controls speed+direction:
      1500 µs  = STOP  (neutral)
      1500–2000 µs = forward,  faster toward 2000
      1000–1500 µs = reverse,  faster toward 1000

  Therefore ALL positional logic (currentDeg, targetDeg, updateServos) has
  been removed.  Control is direct:
    f              → forward at current speed%
    r              → reverse at current speed%
    stop           → neutral pulse (1500 µs) — servo physically stops
    speed <0-100>  → set speed percentage (default 50)
    set <-100,100> → direct signed % command
    status         → show current command

  I2C:   A4 SDA, A5 SCL
  Serial: D0/D1 reserved for USB
*/

// ======================= TYPES =======================
struct ServoState {
  uint8_t  channel;       // PCA9685 channel 0-2
  int8_t   direction;     // -1 rev, 0 stop, +1 fwd
  uint8_t  speedPct;      // 0..100  magnitude
  int16_t  commandedPct;  // -100..100 last value written to hardware
};

struct Relay {
  const char* label;
  uint8_t pin;
  bool state;
};

enum class MenuState {
  MAIN,
  SERVO_MENU, SERVO_1, SERVO_2, SERVO_3,
  PUMPS_UV_MENU, PUMPS_UV_CONTROL,
  ACTUATOR_MENU,
  DRILL_MENU, DRILL_M1, DRILL_M2, DRILL_M3, DRILL_AUTO
};

enum class ActDir { STOP, FWD, REV };
struct ActuatorState { ActDir dir; uint8_t speed; };

enum DrillSeqState {
  DRILL_STEP1, DRILL_STEP1_STOP,
  DRILL_STEP2, DRILL_STEP2_STOP,
  DRILL_STEP3, DRILL_STEP3_STOP,
  DRILL_STEP4, DRILL_STEP4_STOP,
  DRILL_STEP5, DRILL_STEP5_STOP,
  DRILL_DONE,  DRILL_ABORTED
};

struct DrillMotorState {
  int8_t  direction;
  uint8_t speedPct;
  int16_t signedPct;
};

// ======================= PIN MAP =======================
#define ACT_IN1 2
#define ACT_IN2 3
#define ACT_PWM 5

Relay relays[] = {
  {"pump1",   6,  false},
  {"pump2",   7,  false},
  {"pump3",   8,  false},
  {"stirrer", 9,  false},
  {"uv_led",  10, false},
};
static const uint8_t RELAY_COUNT = sizeof(relays) / sizeof(relays[0]);

#define DRILL_M1_PIN 4
#define DRILL_M2_PIN 11
#define DRILL_M3_PIN 12

static const uint16_t DRILL_US_MIN    = 1000;
static const uint16_t DRILL_US_CENTER = 1500;
static const uint16_t DRILL_US_MAX    = 2000;

// ======================= PCA9685 SERVO CONSTANTS =======================
static const uint16_t SERVO_US_MIN   = 1000;  // full reverse
static const uint16_t SERVO_US_STOP  = 1500;  // neutral / stop
static const uint16_t SERVO_US_MAX   = 2000;  // full forward
static const uint16_t SERVO_PWM_FREQ = 50;    // Hz

// ======================= GLOBALS =======================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

ServoState servos[3] = {
  {0, 0, 50, 0},
  {1, 0, 50, 0},
  {2, 0, 50, 0},
};

ActuatorState actuator = { ActDir::STOP, 150 };

MenuState menuState = MenuState::MAIN;
int selectedRelay   = -1;

Servo drillServo1, drillServo2, drillServo3;
DrillMotorState drillMotors[3] = {
  {0, 40, 0},
  {0, 40, 0},
  {0, 40, 0},
};

bool drillAutoEnabled = false;
bool drillPaused      = false;
bool drillControlMode = false;
bool drillStopReq     = false;

DrillSeqState drillSeqState     = DRILL_STEP1;
unsigned long drillStateStartMs = 0;

// ======================= PCA9685 HELPERS =======================

// Convert microseconds to PCA9685 12-bit tick count
static uint16_t usToTicks(uint16_t us) {
  uint32_t t = (uint32_t)us * SERVO_PWM_FREQ * 4096UL;
  t = (t + 500000UL) / 1000000UL;
  if (t > 4095) t = 4095;
  return (uint16_t)t;
}

// Convert signed percent (-100..100) to pulse width in us
// 0% = 1500us (stop), +100% = 2000us (full fwd), -100% = 1000us (full rev)
static uint16_t pctToServUs(int16_t pct) {
  if (pct >  100) pct =  100;
  if (pct < -100) pct = -100;
  if (pct == 0) return SERVO_US_STOP;
  if (pct > 0)
    return (uint16_t)(SERVO_US_STOP +
      (uint32_t)(SERVO_US_MAX - SERVO_US_STOP) * (uint32_t)pct / 100UL);
  uint16_t ap = (uint16_t)(-pct);
  return (uint16_t)(SERVO_US_STOP -
    (uint32_t)(SERVO_US_STOP - SERVO_US_MIN) * (uint32_t)ap / 100UL);
}

// Write a signed-percent command directly to a PCA9685 channel
static void servoWritePct(uint8_t idx, int16_t pct) {
  if (idx > 2) return;
  if (pct >  100) pct =  100;
  if (pct < -100) pct = -100;
  servos[idx].commandedPct = pct;
  uint16_t us    = pctToServUs(pct);
  uint16_t ticks = usToTicks(us);
  pca.setPWM(servos[idx].channel, 0, ticks);
}

// Apply current direction+speed of a ServoState to hardware
static void servoApply(uint8_t idx) {
  ServoState &s = servos[idx];
  int16_t pct = 0;
  if      (s.direction > 0) pct =  (int16_t)s.speedPct;
  else if (s.direction < 0) pct = -(int16_t)s.speedPct;
  servoWritePct(idx, pct);
}

// Send 1500us neutral — servo physically stops
static void servoStop(uint8_t idx) {
  servos[idx].direction = 0;
  servoWritePct(idx, 0);
}

// ======================= OTHER HELPERS =======================
static void applyRelay(uint8_t i) {
  digitalWrite(relays[i].pin, relays[i].state ? HIGH : LOW);
}

static void actuatorApply() {
  switch (actuator.dir) {
    case ActDir::STOP: digitalWrite(ACT_IN1, LOW);  digitalWrite(ACT_IN2, LOW);  break;
    case ActDir::FWD:  digitalWrite(ACT_IN1, HIGH); digitalWrite(ACT_IN2, LOW);  break;
    case ActDir::REV:  digitalWrite(ACT_IN1, LOW);  digitalWrite(ACT_IN2, HIGH); break;
  }
  analogWrite(ACT_PWM, actuator.speed);
}

static const __FlashStringHelper* actDirStr(ActDir d) {
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
      String out = buf; buf = ""; out.trim(); return out;
    }
    buf += c;
    if (buf.length() > 200) {
      buf = ""; Serial.println(F("ERR: line too long")); return "";
    }
  }
  return "";
}

static bool isInteger(const String& s) {
  if (s.length() == 0) return false;
  int i = (s[0] == '-' || s[0] == '+') ? 1 : 0;
  for (; i < (int)s.length(); i++) if (!isDigit(s[i])) return false;
  return true;
}

static void splitCmd(const String& in, String& word, String& args) {
  String s = in; s.trim();
  int sp = s.indexOf(' ');
  if (sp < 0) { word = s; args = ""; }
  else { word = s.substring(0, sp); args = s.substring(sp + 1); args.trim(); }
}

static String lowerTrim(const String& in) {
  String s = in; s.trim(); s.toLowerCase(); return s;
}

// ======================= DRILL (RoboClaw PWM) =======================
static int16_t clampPct(int v) {
  if (v < -100) return -100;
  if (v >  100) return  100;
  return (int16_t)v;
}

static uint16_t drillPctToUs(int16_t pct) {
  pct = clampPct(pct);
  if (pct == 0) return DRILL_US_CENTER;
  if (pct > 0)
    return (uint16_t)(DRILL_US_CENTER +
      (uint32_t)(DRILL_US_MAX - DRILL_US_CENTER) * (uint32_t)pct / 100UL);
  uint16_t ap = (uint16_t)(-pct);
  return (uint16_t)(DRILL_US_CENTER -
    (uint32_t)(DRILL_US_CENTER - DRILL_US_MIN) * (uint32_t)ap / 100UL);
}

static void drillApplyMotor(uint8_t i) {
  if (i > 2) return;
  DrillMotorState &m = drillMotors[i];
  int16_t cmd = (m.direction > 0) ? (int16_t)m.speedPct :
                (m.direction < 0) ? -(int16_t)m.speedPct : 0;
  m.signedPct = cmd;
  uint16_t us = drillPctToUs(cmd);
  if (i == 0) drillServo1.writeMicroseconds(us);
  if (i == 1) drillServo2.writeMicroseconds(us);
  if (i == 2) drillServo3.writeMicroseconds(us);
}

static void drillSetMotorPct(uint8_t i, int16_t pct) {
  if (i > 2) return;
  pct = clampPct(pct);
  DrillMotorState &m = drillMotors[i];
  if      (pct > 0) { m.direction =  1; m.speedPct = (uint8_t) pct; }
  else if (pct < 0) { m.direction = -1; m.speedPct = (uint8_t)-pct; }
  else              { m.direction =  0; m.speedPct = 0; }
  drillApplyMotor(i);
}

static void drillStopAll() {
  drillSetMotorPct(0,0); drillSetMotorPct(1,0); drillSetMotorPct(2,0);
}

static void drillEnterState(DrillSeqState s) {
  drillSeqState = s;
  drillStateStartMs = millis();
  switch (s) {
    case DRILL_STEP1:      Serial.println(F("DRILL: STEP1 M1+40 M2+40 42.2s")); break;
    case DRILL_STEP1_STOP: Serial.println(F("DRILL: STEP1 stop")); break;
    case DRILL_STEP2:      Serial.println(F("DRILL: STEP2 M1+40 M3-40 21s")); break;
    case DRILL_STEP2_STOP: Serial.println(F("DRILL: STEP2 stop")); break;
    case DRILL_STEP3:      Serial.println(F("DRILL: STEP3 M1-40 M3-20 39s")); break;
    case DRILL_STEP3_STOP: Serial.println(F("DRILL: STEP3 stop")); break;
    case DRILL_STEP4:      Serial.println(F("DRILL: STEP4 M2-40 17.5s")); break;
    case DRILL_STEP4_STOP: Serial.println(F("DRILL: STEP4 stop")); break;
    case DRILL_STEP5:      Serial.println(F("DRILL: STEP5 M1-40 M2-40 26.7s")); break;
    case DRILL_STEP5_STOP: Serial.println(F("DRILL: STEP5 stop")); break;
    case DRILL_DONE:       Serial.println(F("DRILL: DONE")); break;
    case DRILL_ABORTED:    Serial.println(F("DRILL: ABORTED")); break;
  }
}

static void drillUpdateSequence() {
  if (!drillAutoEnabled || drillPaused || drillControlMode) return;
  if (drillStopReq) {
    if (drillSeqState != DRILL_ABORTED) { drillEnterState(DRILL_ABORTED); drillStopAll(); }
    return;
  }
  unsigned long now = millis();
  switch (drillSeqState) {
    case DRILL_STEP1:
      drillSetMotorPct(0,+40); drillSetMotorPct(1,+40);
      if (now-drillStateStartMs>=42200UL){drillStopAll();drillEnterState(DRILL_STEP1_STOP);}
      break;
    case DRILL_STEP1_STOP: drillEnterState(DRILL_STEP2); break;
    case DRILL_STEP2:
      drillSetMotorPct(0,+40); drillSetMotorPct(2,-40);
      if (now-drillStateStartMs>=21000UL){drillStopAll();drillEnterState(DRILL_STEP2_STOP);}
      break;
    case DRILL_STEP2_STOP: drillEnterState(DRILL_STEP3); break;
    case DRILL_STEP3:
      drillSetMotorPct(0,-40); drillSetMotorPct(2,-20);
      if (now-drillStateStartMs>=39000UL){drillStopAll();drillEnterState(DRILL_STEP3_STOP);}
      break;
    case DRILL_STEP3_STOP: drillEnterState(DRILL_STEP4); break;
    case DRILL_STEP4:
      drillSetMotorPct(1,-40);
      if (now-drillStateStartMs>=17500UL){drillStopAll();drillEnterState(DRILL_STEP4_STOP);}
      break;
    case DRILL_STEP4_STOP: drillEnterState(DRILL_STEP5); break;
    case DRILL_STEP5:
      drillSetMotorPct(0,-40); drillSetMotorPct(1,-40);
      if (now-drillStateStartMs>=26700UL){drillStopAll();drillEnterState(DRILL_STEP5_STOP);}
      break;
    case DRILL_STEP5_STOP: drillEnterState(DRILL_DONE); drillAutoEnabled=false; break;
    case DRILL_DONE: case DRILL_ABORTED: drillAutoEnabled=false; break;
  }
}

// ======================= MENU PRINTS =======================
static void printMainMenu() {
  Serial.println(F("\n=== MAIN MENU ==="));
  Serial.println(F("1) Servos  2) Pumps/UV  3) Actuator  4) Drill  h) Help"));
}

static void printServoSelectMenu() {
  Serial.println(F("\n=== SERVO MENU ==="));
  Serial.println(F("1) Servo 1   2) Servo 2   3) Servo 3   b) Back"));
}

static void printServoControlMenu(uint8_t idx) {
  ServoState &s = servos[idx];
  Serial.print(F("\n=== SERVO ")); Serial.print(idx+1);
  Serial.println(F(" (continuous rotation) ==="));
  Serial.println(F("  f             spin forward at current speed"));
  Serial.println(F("  r             spin reverse at current speed"));
  Serial.println(F("  stop          STOP (1500us neutral pulse)"));
  Serial.println(F("  speed <0-100> set speed % (applies immediately if running)"));
  Serial.println(F("  set <-100,100> direct signed command"));
  Serial.println(F("  status | b"));
  Serial.print(F("Now: cmd=")); Serial.print(s.commandedPct);
  Serial.print(F("% ("));  Serial.print(pctToServUs(s.commandedPct));
  Serial.println(F("us)"));
}

static void printServoStatus(uint8_t idx) {
  ServoState &s = servos[idx];
  Serial.print(F("S")); Serial.print(idx+1);
  Serial.print(F(": ")); Serial.print(s.direction>0?F("FWD"):s.direction<0?F("REV"):F("STOP"));
  Serial.print(F(" speed=")); Serial.print(s.speedPct);
  Serial.print(F("% cmd=")); Serial.print(s.commandedPct);
  Serial.print(F("% pulse=")); Serial.print(pctToServUs(s.commandedPct));
  Serial.println(F("us"));
}

static void printPumpsUvMenu() {
  Serial.println(F("\n=== PUMPS AND UV ==="));
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    Serial.print(i+1); Serial.print(F(") "));
    Serial.print(relays[i].label); Serial.print(F(" = "));
    Serial.println(relays[i].state ? F("ON") : F("OFF"));
  }
  Serial.println(F("Pick by name/number, or b"));
}

static void printActuatorStatus() {
  Serial.print(F("dir=")); Serial.print(actDirStr(actuator.dir));
  Serial.print(F(" speed=")); Serial.println(actuator.speed);
}

static void printDrillMenu() {
  Serial.println(F("\n=== DRILL ==="));
  Serial.println(F("1) Motor1  2) Motor2  3) Motor3  4) Auto  b) Back"));
}

static void printDrillMotorMenu(uint8_t i) {
  Serial.print(F("\n=== DRILL MOTOR ")); Serial.print(i+1); Serial.println(F(" ==="));
  Serial.println(F("f | r | stop | speed <0-100> | set <-100..100> | status | b"));
}

static void printDrillAutoMenu() {
  Serial.println(F("\n=== DRILL AUTO ==="));
  Serial.println(F("start | pause | resume | stop | status | b"));
}

static void drillPrintMotorStatus(uint8_t i) {
  DrillMotorState &m = drillMotors[i];
  Serial.print(F("M")); Serial.print(i+1); Serial.print(F(": "));
  Serial.print(m.direction>0?F("FWD"):m.direction<0?F("REV"):F("STOP"));
  Serial.print(F(" ")); Serial.print(m.speedPct);
  Serial.print(F("% cmd=")); Serial.print(m.signedPct); Serial.println(F("%"));
}

// ======================= SERVO COMMAND HANDLER =======================
static void handleServoControl(uint8_t idx, const String& cmdRaw) {
  String word, args;
  splitCmd(cmdRaw, word, args);
  String w = lowerTrim(word);

  if (w == "b") { menuState = MenuState::SERVO_MENU; printServoSelectMenu(); return; }

  if (w == "f") {
    servos[idx].direction = +1;
    servoApply(idx);
    Serial.print(F("FWD ")); Serial.print(servos[idx].speedPct);
    Serial.print(F("% -> ")); Serial.print(pctToServUs(servos[idx].commandedPct));
    Serial.println(F("us"));
    return;
  }

  if (w == "r") {
    servos[idx].direction = -1;
    servoApply(idx);
    Serial.print(F("REV ")); Serial.print(servos[idx].speedPct);
    Serial.print(F("% -> ")); Serial.print(pctToServUs(servos[idx].commandedPct));
    Serial.println(F("us"));
    return;
  }

  if (w == "stop") {
    servoStop(idx);
    Serial.println(F("STOP -> 1500us"));
    return;
  }

  if (w == "speed") {
    args.trim();
    if (args.length() == 0 || !isInteger(args)) {
      Serial.println(F("ERR: speed <0-100>")); return;
    }
    int v = args.toInt();
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    servos[idx].speedPct = (uint8_t)v;
    // Apply immediately if servo is already running
    if (servos[idx].direction != 0) servoApply(idx);
    Serial.print(F("speed=")); Serial.print(v); Serial.print(F("%"));
    if (servos[idx].direction != 0) {
      Serial.print(F(" -> ")); Serial.print(pctToServUs(servos[idx].commandedPct)); Serial.print(F("us"));
    }
    Serial.println();
    return;
  }

  if (w == "set") {
    args.trim();
    if (args.length() == 0 || !isInteger(args)) {
      Serial.println(F("ERR: set <-100..100>")); return;
    }
    int v = args.toInt();
    if (v >  100) v =  100;
    if (v < -100) v = -100;
    if      (v > 0) { servos[idx].direction =  1; servos[idx].speedPct = (uint8_t) v; }
    else if (v < 0) { servos[idx].direction = -1; servos[idx].speedPct = (uint8_t)-v; }
    else            { servos[idx].direction =  0; servos[idx].speedPct = 0; }
    servoWritePct(idx, (int16_t)v);
    Serial.print(F("set ")); Serial.print(v);
    Serial.print(F("% -> ")); Serial.print(pctToServUs((int16_t)v));
    Serial.println(F("us"));
    return;
  }

  if (w == "status") { printServoStatus(idx); return; }

  Serial.println(F("? Use: f | r | stop | speed <0-100> | set <-100,100> | status | b"));
}

// ======================= OTHER HANDLERS =======================
static void handleServoSelect(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (cmd == "1") { menuState = MenuState::SERVO_1; printServoControlMenu(0); return; }
  if (cmd == "2") { menuState = MenuState::SERVO_2; printServoControlMenu(1); return; }
  if (cmd == "3") { menuState = MenuState::SERVO_3; printServoControlMenu(2); return; }
  Serial.println(F("Pick 1/2/3 or b"));
}

static int findRelayByName(const String& name) {
  for (uint8_t i = 0; i < RELAY_COUNT; i++)
    if (name.equalsIgnoreCase(relays[i].label)) return i;
  return -1;
}

static void handlePumpsUvMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  int idx = -1;
  if (isInteger(cmd)) { int n=cmd.toInt(); if(n>=1&&n<=(int)RELAY_COUNT) idx=n-1; }
  else idx = findRelayByName(cmd);
  if (idx < 0) { Serial.println(F("Unknown")); return; }
  selectedRelay = idx;
  menuState = MenuState::PUMPS_UV_CONTROL;
  Serial.print(F("\n=== ")); Serial.print(relays[idx].label); Serial.println(F(" ==="));
  Serial.println(F("on | off | toggle | status | b"));
}

static void handlePumpsUvControl(const String& cmd) {
  if (cmd == "b") { menuState=MenuState::PUMPS_UV_MENU; printPumpsUvMenu(); return; }
  if (cmd.equalsIgnoreCase("on"))     { relays[selectedRelay].state=true;  applyRelay(selectedRelay); Serial.println(F("ON"));  return; }
  if (cmd.equalsIgnoreCase("off"))    { relays[selectedRelay].state=false; applyRelay(selectedRelay); Serial.println(F("OFF")); return; }
  if (cmd.equalsIgnoreCase("toggle")) { relays[selectedRelay].state=!relays[selectedRelay].state; applyRelay(selectedRelay); Serial.println(relays[selectedRelay].state?F("ON"):F("OFF")); return; }
  if (cmd.equalsIgnoreCase("status")) { Serial.println(relays[selectedRelay].state?F("ON"):F("OFF")); return; }
  Serial.println(F("on | off | toggle | status | b"));
}

static void handleActuatorMenu(const String& cmd) {
  if (cmd == "b")                     { menuState=MenuState::MAIN; printMainMenu(); return; }
  if (cmd.equalsIgnoreCase("f"))      { actuator.dir=ActDir::FWD;  actuatorApply(); printActuatorStatus(); return; }
  if (cmd.equalsIgnoreCase("r"))      { actuator.dir=ActDir::REV;  actuatorApply(); printActuatorStatus(); return; }
  if (cmd.equalsIgnoreCase("stop"))   { actuator.dir=ActDir::STOP; actuatorApply(); printActuatorStatus(); return; }
  if (cmd.equalsIgnoreCase("status")) { printActuatorStatus(); return; }
  if (cmd.startsWith("speed ")) {
    int sp=cmd.substring(6).toInt(); if(sp<0)sp=0; if(sp>255)sp=255;
    actuator.speed=(uint8_t)sp; actuatorApply(); printActuatorStatus(); return;
  }
  Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

static void handleDrillMenu(const String& cmd) {
  if (cmd=="b")                       { menuState=MenuState::MAIN;       printMainMenu();       return; }
  if (cmd=="1")                       { menuState=MenuState::DRILL_M1;   printDrillMotorMenu(0);return; }
  if (cmd=="2")                       { menuState=MenuState::DRILL_M2;   printDrillMotorMenu(1);return; }
  if (cmd=="3")                       { menuState=MenuState::DRILL_M3;   printDrillMotorMenu(2);return; }
  if (cmd=="4")                       { menuState=MenuState::DRILL_AUTO; printDrillAutoMenu();  return; }
  if (cmd.equalsIgnoreCase("status")) { drillPrintMotorStatus(0);drillPrintMotorStatus(1);drillPrintMotorStatus(2); return; }
  Serial.println(F("1/2/3/4, status, or b"));
}

static void handleDrillMotor(uint8_t i, const String& cmd) {
  if (cmd=="b") { menuState=MenuState::DRILL_MENU; printDrillMenu(); return; }
  if (cmd.equalsIgnoreCase("status")) { drillPrintMotorStatus(i); return; }
  drillControlMode=true; drillAutoEnabled=false; drillPaused=false;
  if (cmd.equalsIgnoreCase("f"))    { drillMotors[i].direction= 1; drillApplyMotor(i); return; }
  if (cmd.equalsIgnoreCase("r"))    { drillMotors[i].direction=-1; drillApplyMotor(i); return; }
  if (cmd.equalsIgnoreCase("stop")) { drillMotors[i].direction= 0; drillApplyMotor(i); return; }
  if (cmd.startsWith("speed ")) {
    int v=cmd.substring(6).toInt(); if(v<0)v=0; if(v>100)v=100;
    drillMotors[i].speedPct=(uint8_t)v; drillApplyMotor(i); return;
  }
  if (cmd.startsWith("set ")) {
    int v=cmd.substring(4).toInt(); drillSetMotorPct(i,(int16_t)v); return;
  }
  Serial.println(F("f | r | stop | speed <0-100> | set <-100..100> | status | b"));
}

static void handleDrillAuto(const String& cmd) {
  if (cmd=="b") { menuState=MenuState::DRILL_MENU; printDrillMenu(); return; }
  if (cmd.equalsIgnoreCase("status")) {
    drillPrintMotorStatus(0); drillPrintMotorStatus(1); drillPrintMotorStatus(2);
    Serial.print(F("AUTO=")); Serial.print(drillAutoEnabled?F("ON"):F("OFF"));
    Serial.print(F(" paused=")); Serial.println(drillPaused?F("YES"):F("NO")); return;
  }
  if (cmd.equalsIgnoreCase("start")) {
    drillStopReq=false; drillPaused=false; drillControlMode=false; drillAutoEnabled=true;
    drillStopAll(); drillEnterState(DRILL_STEP1);
    Serial.println(F("OK: started")); return;
  }
  if (cmd.equalsIgnoreCase("pause"))  { drillPaused=true; Serial.println(F("paused")); return; }
  if (cmd.equalsIgnoreCase("resume")) {
    if (drillSeqState==DRILL_ABORTED||drillSeqState==DRILL_DONE) {
      Serial.println(F("Use 'start'")); return;
    }
    drillPaused=false; drillAutoEnabled=true; drillControlMode=false;
    Serial.println(F("resumed")); return;
  }
  if (cmd.equalsIgnoreCase("stop")) {
    drillStopReq=true; drillAutoEnabled=false; drillPaused=false;
    drillEnterState(DRILL_ABORTED); drillStopAll();
    Serial.println(F("stopped")); return;
  }
  Serial.println(F("start | pause | resume | stop | status | b"));
}

static void handleMain(const String& cmd) {
  if (cmd=="1"||cmd.equalsIgnoreCase("servos"))                                   { menuState=MenuState::SERVO_MENU;    printServoSelectMenu(); return; }
  if (cmd=="2"||cmd.equalsIgnoreCase("pumps")||cmd.equalsIgnoreCase("uv"))        { menuState=MenuState::PUMPS_UV_MENU; printPumpsUvMenu();     return; }
  if (cmd=="3"||cmd.equalsIgnoreCase("actuator")||cmd.equalsIgnoreCase("linear")) { menuState=MenuState::ACTUATOR_MENU; Serial.println(F("\n=== ACTUATOR ===\nf|r|stop|speed<0-255>|status|b")); printActuatorStatus(); return; }
  if (cmd=="4"||cmd.equalsIgnoreCase("drill"))                                    { menuState=MenuState::DRILL_MENU;    printDrillMenu();       return; }
  if (cmd=="h"||cmd.equalsIgnoreCase("help"))                                     { printMainMenu(); return; }
  Serial.println(F("?"));
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pca.begin();
  pca.setPWMFreq(SERVO_PWM_FREQ);
  delay(10);

  // Send 1500us neutral to all PCA9685 servo channels at startup
  for (uint8_t i = 0; i < 3; i++) servoStop(i);

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(relays[i].pin, OUTPUT);
    relays[i].state = false;
    applyRelay(i);
  }

  pinMode(ACT_IN1, OUTPUT); pinMode(ACT_IN2, OUTPUT); pinMode(ACT_PWM, OUTPUT);
  actuatorApply();

  drillServo1.attach(DRILL_M1_PIN);
  drillServo2.attach(DRILL_M2_PIN);
  drillServo3.attach(DRILL_M3_PIN);
  drillStopAll();

  Serial.println(F("\n=== System Ready ==="));
  Serial.println(F("Servos on PCA9685 ch0-2: continuous rotation"));
  Serial.println(F("  1500us=STOP  >1500=FWD  <1500=REV"));
  Serial.println(F("Drill PWM: M1=D4 M2=D11 M3=D12"));
  printMainMenu();
}

void loop() {
  // No servo background update needed — all servo commands write directly to PCA9685
  drillUpdateSequence();

  String cmd = readLineNonBlocking();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("main")) { menuState=MenuState::MAIN; printMainMenu(); return; }

  switch (menuState) {
    case MenuState::MAIN:             handleMain(cmd);            break;
    case MenuState::SERVO_MENU:       handleServoSelect(cmd);     break;
    case MenuState::SERVO_1:          handleServoControl(0, cmd); break;
    case MenuState::SERVO_2:          handleServoControl(1, cmd); break;
    case MenuState::SERVO_3:          handleServoControl(2, cmd); break;
    case MenuState::PUMPS_UV_MENU:    handlePumpsUvMenu(cmd);     break;
    case MenuState::PUMPS_UV_CONTROL: handlePumpsUvControl(cmd);  break;
    case MenuState::ACTUATOR_MENU:    handleActuatorMenu(cmd);    break;
    case MenuState::DRILL_MENU:       handleDrillMenu(cmd);       break;
    case MenuState::DRILL_M1:         handleDrillMotor(0, cmd);   break;
    case MenuState::DRILL_M2:         handleDrillMotor(1, cmd);   break;
    case MenuState::DRILL_M3:         handleDrillMotor(2, cmd);   break;
    case MenuState::DRILL_AUTO:       handleDrillAuto(cmd);       break;
  }
}
