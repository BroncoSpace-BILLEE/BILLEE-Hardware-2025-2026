#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*
  Robot Controller — Elegoo MEGA 2560 R3 + PCA9685
  =================================================
  PIN MAP:
  ┌─────────────────┬──────────────────────────────────────┐
  │ I2C             │ D20 (SDA), D21 (SCL) → PCA9685       │
  ├─────────────────┼──────────────────────────────────────┤
  │ Relays          │ D22 pump1, D23 pump2, D24 pump3       │
  │                 │ D25 stirrer, D26 uv_led               │
  ├─────────────────┼──────────────────────────────────────┤
  │ Linear Actuator │ D2 IN1, D3 IN2, D4 PWM~              │
  ├─────────────────┼──────────────────────────────────────┤
  │ Drill           │ D5 IN1, D6 IN2, D7 PWM~              │
  ├─────────────────┼──────────────────────────────────────┤
  │ Drill Module 1  │ D8 IN1, D9 IN2, D10 PWM~             │
  ├─────────────────┼──────────────────────────────────────┤
  │ Drill Module 2  │ D11 IN1, D12 IN2, D44 PWM~           │
  ├─────────────────┼──────────────────────────────────────┤
  │ PCA9685 ch0-2   │ 360° continuous-rotation servos       │
  └─────────────────┴──────────────────────────────────────┘

  Servo PWM model (continuous rotation):
    1500 µs = STOP
    >1500   = forward  (faster toward 2000)
    <1500   = reverse  (faster toward 1000)

  All four H-bridges (Actuator, Drill, Drill Module 1, Drill Module 2)
  use the same control pattern:
    f              → forward at current speed
    r              → reverse at current speed
    stop           → stop
    speed <0-255>  → set PWM speed
    status         → show state
    b              → back to main menu
*/

// ======================= TYPES =======================
struct ServoState {
  uint8_t  channel;
  int8_t   direction;
  uint8_t  speedPct;
  int16_t  commandedPct;
};

struct Relay {
  const char* label;
  uint8_t pin;
  bool state;
};

enum class ActDir { STOP, FWD, REV };

struct HBridge {
  ActDir  dir;
  uint8_t speed;
  uint8_t pinIN1;
  uint8_t pinIN2;
  uint8_t pinPWM;
};

enum class MenuState {
  MAIN,
  SERVO_MENU, SERVO_1, SERVO_2, SERVO_3, SERVO_4,
  PUMPS_UV_MENU, PUMPS_UV_CONTROL,
  ACTUATOR_MENU,
  DRILL_MENU,
  DRILLMOD_SELECT,     // parent: pick mod1 / mod2 / combined
  DRILLMOD1_MENU,
  DRILLMOD2_MENU,
  DRILLMOD_COMBINED    // controls both mod1+mod2 together
};

// ======================= PIN DEFINITIONS =======================

// Linear actuator
#define ACT_IN1   2
#define ACT_IN2   3
#define ACT_PWM   4

// Drill
#define DRILL_IN1   5
#define DRILL_IN2   6
#define DRILL_PWM   7

// Drill Module 1
#define DRILLMOD1_IN1   8
#define DRILLMOD1_IN2   9
#define DRILLMOD1_PWM   10

// Drill Module 2
#define DRILLMOD2_IN1   11
#define DRILLMOD2_IN2   12
#define DRILLMOD2_PWM   44

// Relays
#define RELAY_PIN_1  22
#define RELAY_PIN_2  23
#define RELAY_PIN_3  25
#define RELAY_PIN_4  24
#define RELAY_PIN_5  26

// I2C on Mega: D20 (SDA), D21 (SCL) — used automatically by Wire.h

// ======================= PCA9685 SERVO CONSTANTS =======================
static const uint16_t SERVO_US_STOP  = 1500;
static const uint16_t SERVO_US_MAX   = 2000;
static const uint16_t SERVO_US_MIN   = 1000;
static const uint16_t SERVO_PWM_FREQ = 50;

// ======================= GLOBALS =======================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

ServoState servos[4] = {
  {0, 0, 50, 0},
  {1, 0, 50, 0},
  {2, 0, 50, 0},
  {3, 0, 50, 0},
};

Relay relays[] = {
  {"pump1",   RELAY_PIN_1, false},
  {"pump2",   RELAY_PIN_2, false},
  {"pump3",   RELAY_PIN_3, false},
  {"stirrer", RELAY_PIN_4, false},
  {"uv_led",  RELAY_PIN_5, false},
};
static const uint8_t RELAY_COUNT = sizeof(relays) / sizeof(relays[0]);

HBridge actuator  = { ActDir::STOP, 150, ACT_IN1,      ACT_IN2,      ACT_PWM      };
HBridge drill     = { ActDir::STOP, 150, DRILL_IN1,    DRILL_IN2,    DRILL_PWM    };
HBridge drillMod1 = { ActDir::STOP, 150, DRILLMOD1_IN1, DRILLMOD1_IN2, DRILLMOD1_PWM };
HBridge drillMod2 = { ActDir::STOP, 150, DRILLMOD2_IN1, DRILLMOD2_IN2, DRILLMOD2_PWM };

MenuState menuState = MenuState::MAIN;
int selectedRelay   = -1;

// ======================= PCA9685 HELPERS =======================
static uint16_t usToTicks(uint16_t us) {
  uint32_t t = (uint32_t)us * SERVO_PWM_FREQ * 4096UL;
  t = (t + 500000UL) / 1000000UL;
  if (t > 4095) t = 4095;
  return (uint16_t)t;
}

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

static void servoWritePct(uint8_t idx, int16_t pct) {
  if (idx > 3) return;
  if (pct >  100) pct =  100;
  if (pct < -100) pct = -100;
  servos[idx].commandedPct = pct;
  pca.setPWM(servos[idx].channel, 0, usToTicks(pctToServUs(pct)));
}

static void servoApply(uint8_t idx) {
  ServoState &s = servos[idx];
  int16_t pct = 0;
  if      (s.direction > 0) pct =  (int16_t)s.speedPct;
  else if (s.direction < 0) pct = -(int16_t)s.speedPct;
  servoWritePct(idx, pct);
}

static void servoStop(uint8_t idx) {
  servos[idx].direction = 0;
  servoWritePct(idx, 0);
}

// ======================= H-BRIDGE HELPERS =======================
static void hbridgeApply(HBridge &h) {
  switch (h.dir) {
    case ActDir::STOP: digitalWrite(h.pinIN1, LOW);  digitalWrite(h.pinIN2, LOW);  break;
    case ActDir::FWD:  digitalWrite(h.pinIN1, HIGH); digitalWrite(h.pinIN2, LOW);  break;
    case ActDir::REV:  digitalWrite(h.pinIN1, LOW);  digitalWrite(h.pinIN2, HIGH); break;
  }
  analogWrite(h.pinPWM, h.speed);
}

static void hbridgeInit(HBridge &h) {
  pinMode(h.pinIN1, OUTPUT);
  pinMode(h.pinIN2, OUTPUT);
  pinMode(h.pinPWM, OUTPUT);
  h.dir = ActDir::STOP;
  hbridgeApply(h);
}

static const __FlashStringHelper* dirStr(ActDir d) {
  switch (d) {
    case ActDir::STOP: return F("STOP");
    case ActDir::FWD:  return F("FWD");
    case ActDir::REV:  return F("REV");
  }
  return F("?");
}

static void printHBridgeStatus(const char* label, HBridge &h) {
  Serial.print(label);
  Serial.print(F(": dir=")); Serial.print(dirStr(h.dir));
  Serial.print(F(" speed=")); Serial.println(h.speed);
}

// Shared H-bridge command handler — returns false if command not recognised
static bool handleHBridge(HBridge &h, const char* label, const String& cmd) {
  if (cmd.equalsIgnoreCase("f")) {
    h.dir = ActDir::FWD;  hbridgeApply(h); printHBridgeStatus(label, h); return true;
  }
  if (cmd.equalsIgnoreCase("r")) {
    h.dir = ActDir::REV;  hbridgeApply(h); printHBridgeStatus(label, h); return true;
  }
  if (cmd.equalsIgnoreCase("stop")) {
    h.dir = ActDir::STOP; hbridgeApply(h); printHBridgeStatus(label, h); return true;
  }
  if (cmd.equalsIgnoreCase("status")) {
    printHBridgeStatus(label, h); return true;
  }
  if (cmd.startsWith("speed ")) {
    int sp = cmd.substring(6).toInt();
    if (sp < 0) sp = 0; if (sp > 255) sp = 255;
    h.speed = (uint8_t)sp;
    hbridgeApply(h); printHBridgeStatus(label, h); return true;
  }
  return false;
}

// ======================= RELAY HELPERS =======================
static void applyRelay(uint8_t i) {
  digitalWrite(relays[i].pin, relays[i].state ? HIGH : LOW);
}

// ======================= SERIAL INPUT =======================
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

// ======================= MENU PRINTS =======================
static void printMainMenu() {
  Serial.println(F("\n=== MAIN MENU ==="));
  Serial.println(F("1) Servos"));
  Serial.println(F("2) Pumps and UV"));
  Serial.println(F("3) Linear Actuator"));
  Serial.println(F("4) Drill"));
  Serial.println(F("5) Drill Modules"));
  Serial.println(F("h) Help"));
}

static void printHBridgeMenu(const char* label) {
  Serial.print(F("\n=== ")); Serial.print(label); Serial.println(F(" ==="));
  Serial.println(F("f             forward"));
  Serial.println(F("r             reverse"));
  Serial.println(F("stop          stop"));
  Serial.println(F("speed <0-255> set PWM speed"));
  Serial.println(F("status        show current state"));
  Serial.println(F("b             back"));
}

static void printServoSelectMenu() {
  Serial.println(F("\n=== SERVO MENU ==="));
  Serial.println(F("1) Servo 1   2) Servo 2   3) Servo 3   4) Servo 4   b) Back"));
}

static void printServoControlMenu(uint8_t idx) {
  ServoState &s = servos[idx];
  Serial.print(F("\n=== SERVO ")); Serial.print(idx + 1);
  Serial.println(F(" (continuous rotation) ==="));
  Serial.println(F("f              spin forward at current speed"));
  Serial.println(F("r              spin reverse at current speed"));
  Serial.println(F("stop           STOP (1500us neutral)"));
  Serial.println(F("speed <0-100>  set speed % (applies immediately if running)"));
  Serial.println(F("set <-100,100> direct signed command"));
  Serial.println(F("status | b"));
  Serial.print(F("Now: cmd=")); Serial.print(s.commandedPct);
  Serial.print(F("% (")); Serial.print(pctToServUs(s.commandedPct));
  Serial.println(F("us)"));
}

static void printServoStatus(uint8_t idx) {
  ServoState &s = servos[idx];
  Serial.print(F("S")); Serial.print(idx + 1);
  Serial.print(F(": ")); Serial.print(s.direction > 0 ? F("FWD") : s.direction < 0 ? F("REV") : F("STOP"));
  Serial.print(F(" speed=")); Serial.print(s.speedPct);
  Serial.print(F("% cmd=")); Serial.print(s.commandedPct);
  Serial.print(F("% pulse=")); Serial.print(pctToServUs(s.commandedPct));
  Serial.println(F("us"));
}

static void printPumpsUvMenu() {
  Serial.println(F("\n=== PUMPS AND UV ==="));
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    Serial.print(i + 1); Serial.print(F(") "));
    Serial.print(relays[i].label);
    Serial.print(F(" = ")); Serial.println(relays[i].state ? F("ON") : F("OFF"));
  }
  Serial.println(F("Pick by name/number, or b"));
}

// ======================= COMMAND HANDLERS =======================

static void printDrillModSelectMenu() {
  Serial.println(F("\n=== DRILL MODULES ==="));
  Serial.println(F("1) Drill Module 1  (individual)"));
  Serial.println(F("2) Drill Module 2  (individual)"));
  Serial.println(F("3) Combined        (both together)"));
  Serial.println(F("b) Back to main"));
}

static void handleDrillModSelect(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (cmd == "1") {
    menuState = MenuState::DRILLMOD1_MENU;
    printHBridgeMenu("DRILL MODULE 1"); printHBridgeStatus("DRILL MODULE 1", drillMod1); return;
  }
  if (cmd == "2") {
    menuState = MenuState::DRILLMOD2_MENU;
    printHBridgeMenu("DRILL MODULE 2"); printHBridgeStatus("DRILL MODULE 2", drillMod2); return;
  }
  if (cmd == "3") {
    menuState = MenuState::DRILLMOD_COMBINED;
    Serial.println(F("\n=== DRILL MODULES COMBINED ==="));
    Serial.println(F("Commands apply to BOTH Module 1 and Module 2 simultaneously."));
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
    printHBridgeStatus("DRILL MODULE 1", drillMod1);
    printHBridgeStatus("DRILL MODULE 2", drillMod2);
    return;
  }
  Serial.println(F("Pick 1/2/3 or b"));
}

static void handleServoControl(uint8_t idx, const String& cmdRaw) {
  String word, args;
  splitCmd(cmdRaw, word, args);
  String w = lowerTrim(word);

  if (w == "b") { menuState = MenuState::SERVO_MENU; printServoSelectMenu(); return; }

  if (w == "f") {
    servos[idx].direction = +1; servoApply(idx);
    Serial.print(F("FWD ")); Serial.print(servos[idx].speedPct);
    Serial.print(F("% -> ")); Serial.print(pctToServUs(servos[idx].commandedPct));
    Serial.println(F("us")); return;
  }
  if (w == "r") {
    servos[idx].direction = -1; servoApply(idx);
    Serial.print(F("REV ")); Serial.print(servos[idx].speedPct);
    Serial.print(F("% -> ")); Serial.print(pctToServUs(servos[idx].commandedPct));
    Serial.println(F("us")); return;
  }
  if (w == "stop") {
    servoStop(idx); Serial.println(F("STOP -> 1500us")); return;
  }
  if (w == "speed") {
    args.trim();
    if (args.length() == 0 || !isInteger(args)) { Serial.println(F("ERR: speed <0-100>")); return; }
    int v = args.toInt();
    if (v < 0) v = 0; if (v > 100) v = 100;
    servos[idx].speedPct = (uint8_t)v;
    if (servos[idx].direction != 0) servoApply(idx);
    Serial.print(F("speed=")); Serial.print(v); Serial.print(F("%"));
    if (servos[idx].direction != 0) {
      Serial.print(F(" -> ")); Serial.print(pctToServUs(servos[idx].commandedPct)); Serial.print(F("us"));
    }
    Serial.println(); return;
  }
  if (w == "set") {
    args.trim();
    if (args.length() == 0 || !isInteger(args)) { Serial.println(F("ERR: set <-100..100>")); return; }
    int v = args.toInt();
    if (v >  100) v =  100;
    if (v < -100) v = -100;
    if      (v > 0) { servos[idx].direction =  1; servos[idx].speedPct = (uint8_t) v; }
    else if (v < 0) { servos[idx].direction = -1; servos[idx].speedPct = (uint8_t)-v; }
    else            { servos[idx].direction =  0; servos[idx].speedPct = 0; }
    servoWritePct(idx, (int16_t)v);
    Serial.print(F("set ")); Serial.print(v);
    Serial.print(F("% -> ")); Serial.print(pctToServUs((int16_t)v));
    Serial.println(F("us")); return;
  }
  if (w == "status") { printServoStatus(idx); return; }

  Serial.println(F("? Use: f | r | stop | speed <0-100> | set <-100,100> | status | b"));
}

static void handleServoSelect(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (cmd == "1") { menuState = MenuState::SERVO_1; printServoControlMenu(0); return; }
  if (cmd == "2") { menuState = MenuState::SERVO_2; printServoControlMenu(1); return; }
  if (cmd == "3") { menuState = MenuState::SERVO_3; printServoControlMenu(2); return; }
  if (cmd == "4") { menuState = MenuState::SERVO_4; printServoControlMenu(3); return; }
  Serial.println(F("Pick 1/2/3/4 or b"));
}

static int findRelayByName(const String& name) {
  for (uint8_t i = 0; i < RELAY_COUNT; i++)
    if (name.equalsIgnoreCase(relays[i].label)) return i;
  return -1;
}

static void handlePumpsUvMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  int idx = -1;
  if (isInteger(cmd)) { int n = cmd.toInt(); if (n >= 1 && n <= (int)RELAY_COUNT) idx = n - 1; }
  else idx = findRelayByName(cmd);
  if (idx < 0) { Serial.println(F("Unknown")); return; }
  selectedRelay = idx;
  menuState = MenuState::PUMPS_UV_CONTROL;
  Serial.print(F("\n=== ")); Serial.print(relays[idx].label); Serial.println(F(" ==="));
  Serial.println(F("on | off | toggle | status | b"));
}

static void handlePumpsUvControl(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::PUMPS_UV_MENU; printPumpsUvMenu(); return; }
  if (cmd.equalsIgnoreCase("on"))     { relays[selectedRelay].state = true;  applyRelay(selectedRelay); Serial.println(F("ON"));  return; }
  if (cmd.equalsIgnoreCase("off"))    { relays[selectedRelay].state = false; applyRelay(selectedRelay); Serial.println(F("OFF")); return; }
  if (cmd.equalsIgnoreCase("toggle")) { relays[selectedRelay].state = !relays[selectedRelay].state; applyRelay(selectedRelay); Serial.println(relays[selectedRelay].state ? F("ON") : F("OFF")); return; }
  if (cmd.equalsIgnoreCase("status")) { Serial.println(relays[selectedRelay].state ? F("ON") : F("OFF")); return; }
  Serial.println(F("on | off | toggle | status | b"));
}

static void handleActuatorMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (!handleHBridge(actuator, "ACTUATOR", cmd))
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

static void handleDrillMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (!handleHBridge(drill, "DRILL", cmd))
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

static void handleDrillMod1Menu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::DRILLMOD_SELECT; printDrillModSelectMenu(); return; }
  if (!handleHBridge(drillMod1, "DRILL MODULE 1", cmd))
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

static void handleDrillMod2Menu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::DRILLMOD_SELECT; printDrillModSelectMenu(); return; }
  if (!handleHBridge(drillMod2, "DRILL MODULE 2", cmd))
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

// Combined: every command applies to BOTH mod1 and mod2 simultaneously
static void handleDrillModCombined(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::DRILLMOD_SELECT; printDrillModSelectMenu(); return; }

  if (cmd.equalsIgnoreCase("status")) {
    printHBridgeStatus("DRILL MODULE 1", drillMod1);
    printHBridgeStatus("DRILL MODULE 2", drillMod2);
    return;
  }
  // For all other commands, apply to both and report both
  bool ok1 = handleHBridge(drillMod1, "DRILL MODULE 1", cmd);
  bool ok2 = handleHBridge(drillMod2, "DRILL MODULE 2", cmd);
  if (!ok1 && !ok2)
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

static void handleMain(const String& cmd) {
  if (cmd == "1" || cmd.equalsIgnoreCase("servos")) {
    menuState = MenuState::SERVO_MENU; printServoSelectMenu(); return;
  }
  if (cmd == "2" || cmd.equalsIgnoreCase("pumps") || cmd.equalsIgnoreCase("uv")) {
    menuState = MenuState::PUMPS_UV_MENU; printPumpsUvMenu(); return;
  }
  if (cmd == "3" || cmd.equalsIgnoreCase("actuator") || cmd.equalsIgnoreCase("linear")) {
    menuState = MenuState::ACTUATOR_MENU;
    printHBridgeMenu("LINEAR ACTUATOR"); printHBridgeStatus("ACTUATOR", actuator); return;
  }
  if (cmd == "4" || cmd.equalsIgnoreCase("drill")) {
    menuState = MenuState::DRILL_MENU;
    printHBridgeMenu("DRILL"); printHBridgeStatus("DRILL", drill); return;
  }
  if (cmd == "5" || cmd.equalsIgnoreCase("drillmodules") || cmd.equalsIgnoreCase("drill modules")) {
    menuState = MenuState::DRILLMOD_SELECT; printDrillModSelectMenu(); return;
  }
  if (cmd == "h" || cmd.equalsIgnoreCase("help")) { printMainMenu(); return; }
  Serial.println(F("?"));
}

// ======================= SETUP / LOOP =======================
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // PCA9685
  pca.begin();
  pca.setPWMFreq(SERVO_PWM_FREQ);
  delay(10);
  for (uint8_t i = 0; i < 4; i++) servoStop(i);

  // Relays
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(relays[i].pin, OUTPUT);
    relays[i].state = false;
    applyRelay(i);
  }

  // H-bridges — all start stopped
  hbridgeInit(actuator);
  hbridgeInit(drill);
  hbridgeInit(drillMod1);
  hbridgeInit(drillMod2);

  Serial.println(F("\n=== System Ready (Elegoo MEGA 2560) ==="));
  Serial.println(F("PCA9685 ch0-2   : servos  (I2C: D20 SDA, D21 SCL)"));
  Serial.println(F("Relays          : D22-D26"));
  Serial.println(F("Linear Actuator : D2(IN1) D3(IN2) D4(PWM)"));
  Serial.println(F("Drill           : D5(IN1) D6(IN2) D7(PWM)"));
  Serial.println(F("Drill Module 1  : D8(IN1) D9(IN2) D10(PWM)"));
  Serial.println(F("Drill Module 2  : D11(IN1) D12(IN2) D44(PWM)"));
  printMainMenu();
}

void loop() {
  String cmd = readLineNonBlocking();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("main")) { menuState = MenuState::MAIN; printMainMenu(); return; }

  switch (menuState) {
    case MenuState::MAIN:             handleMain(cmd);               break;
    case MenuState::SERVO_MENU:       handleServoSelect(cmd);        break;
    case MenuState::SERVO_1:          handleServoControl(0, cmd);    break;
    case MenuState::SERVO_2:          handleServoControl(1, cmd);    break;
    case MenuState::SERVO_3:          handleServoControl(2, cmd);    break;
    case MenuState::SERVO_4:          handleServoControl(3, cmd);    break;
    case MenuState::PUMPS_UV_MENU:    handlePumpsUvMenu(cmd);        break;
    case MenuState::PUMPS_UV_CONTROL: handlePumpsUvControl(cmd);     break;
    case MenuState::ACTUATOR_MENU:    handleActuatorMenu(cmd);       break;
    case MenuState::DRILL_MENU:       handleDrillMenu(cmd);          break;
    case MenuState::DRILLMOD_SELECT:  handleDrillModSelect(cmd);     break;
    case MenuState::DRILLMOD1_MENU:   handleDrillMod1Menu(cmd);      break;
    case MenuState::DRILLMOD2_MENU:   handleDrillMod2Menu(cmd);      break;
    case MenuState::DRILLMOD_COMBINED: handleDrillModCombined(cmd);  break;
  }
}
