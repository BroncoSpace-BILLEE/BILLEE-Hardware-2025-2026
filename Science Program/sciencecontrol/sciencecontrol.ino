#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/*
  Robot Controller — Arduino UNO R3 + PCA9685
  ============================================
  Pin map:
    I2C         : A4 (SDA), A5 (SCL)  → PCA9685
    Relays      : D6, D7, D8, D9, D10
    Actuator    : D2 (IN1), D3 (IN2), D5 (PWM)
    Drill       : D13 (IN1), D12 (IN2), D11 (PWM)  ← H-bridge, same as actuator
    PCA9685 ch0-2 → 360° continuous-rotation servos

  Servo PWM model (continuous rotation):
    1500 µs = STOP
    >1500   = forward  (faster toward 2000)
    <1500   = reverse  (faster toward 1000)
*/

// ======================= TYPES =======================
struct ServoState {
  uint8_t  channel;      // PCA9685 channel
  int8_t   direction;    // -1 rev, 0 stop, +1 fwd
  uint8_t  speedPct;     // 0-100 magnitude
  int16_t  commandedPct; // last value sent to hardware
};

struct Relay {
  const char* label;
  uint8_t pin;
  bool state;
};

enum class ActDir { STOP, FWD, REV };

struct HBridge {
  ActDir  dir;
  uint8_t speed;   // 0-255 PWM
  uint8_t pinIN1;
  uint8_t pinIN2;
  uint8_t pinPWM;
};

enum class MenuState {
  MAIN,
  SERVO_MENU, SERVO_1, SERVO_2, SERVO_3,
  PUMPS_UV_MENU, PUMPS_UV_CONTROL,
  ACTUATOR_MENU,
  DRILL_MENU,
  DRILL_MODULE_MENU
};

// ======================= PIN DEFINITIONS =======================
// Linear actuator H-bridge
#define ACT_IN1  2
#define ACT_IN2  3
#define ACT_PWM  5

// Drill H-bridge
#define DRILL_IN1  13
#define DRILL_IN2  12
#define DRILL_PWM  11

// Drill Module H-bridge (A0=IN1, A1=IN2 — no PWM, ENA tied to 5V, full speed only)
#define DRILLMOD_IN1  A0
#define DRILLMOD_IN2  A1

// ======================= PCA9685 SERVO CONSTANTS =======================
static const uint16_t SERVO_US_STOP  = 1500;
static const uint16_t SERVO_US_MAX   = 2000;
static const uint16_t SERVO_US_MIN   = 1000;
static const uint16_t SERVO_PWM_FREQ = 50;

// ======================= GLOBALS =======================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

ServoState servos[3] = {
  {0, 0, 50, 0},
  {1, 0, 50, 0},
  {2, 0, 50, 0},
};

Relay relays[] = {
  {"pump1",   6,  false},
  {"pump2",   7,  false},
  {"pump3",   8,  false},
  {"stirrer", 9,  false},
  {"uv_led",  10, false},
};
static const uint8_t RELAY_COUNT = sizeof(relays) / sizeof(relays[0]);

HBridge actuator    = { ActDir::STOP, 150, ACT_IN1,   ACT_IN2,   ACT_PWM   };
HBridge drill       = { ActDir::STOP, 150, DRILL_IN1, DRILL_IN2, DRILL_PWM };

// Drill module — no PWM pin, ENA tied permanently to 5V on the H-bridge board
struct DrillModule {
  ActDir  dir;
  uint8_t pinIN1;
  uint8_t pinIN2;
};
DrillModule drillModule = { ActDir::STOP, DRILLMOD_IN1, DRILLMOD_IN2 };

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
  if (idx > 2) return;
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

// DrillModule helpers (no PWM — full speed only, ENA tied to 5V on board)
static void drillModuleApply() {
  switch (drillModule.dir) {
    case ActDir::STOP: digitalWrite(drillModule.pinIN1, LOW);  digitalWrite(drillModule.pinIN2, LOW);  break;
    case ActDir::FWD:  digitalWrite(drillModule.pinIN1, HIGH); digitalWrite(drillModule.pinIN2, LOW);  break;
    case ActDir::REV:  digitalWrite(drillModule.pinIN1, LOW);  digitalWrite(drillModule.pinIN2, HIGH); break;
  }
}

static void drillModuleInit() {
  pinMode(drillModule.pinIN1, OUTPUT);
  pinMode(drillModule.pinIN2, OUTPUT);
  drillModule.dir = ActDir::STOP;
  drillModuleApply();
}

static void printDrillModuleStatus() {
  Serial.print(F("DRILL MODULE: dir=")); Serial.println(dirStr(drillModule.dir));
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

// Generic H-bridge command handler — used by both actuator and drill menus
// Returns true if handled, false if unknown command
static bool handleHBridge(HBridge &h, const char* label, const String& cmd) {
  if (cmd.equalsIgnoreCase("f")) {
    h.dir = ActDir::FWD; hbridgeApply(h); printHBridgeStatus(label, h); return true;
  }
  if (cmd.equalsIgnoreCase("r")) {
    h.dir = ActDir::REV; hbridgeApply(h); printHBridgeStatus(label, h); return true;
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
  Serial.println(F("5) Drill Module"));
  Serial.println(F("h) Help"));
}

static void printHBridgeMenu(const char* label) {
  Serial.print(F("\n=== ")); Serial.print(label); Serial.println(F(" ==="));
  Serial.println(F("f            forward"));
  Serial.println(F("r            reverse"));
  Serial.println(F("stop         stop"));
  Serial.println(F("speed <0-255> set PWM speed"));
  Serial.println(F("status       show current state"));
  Serial.println(F("b            back"));
}

static void printServoSelectMenu() {
  Serial.println(F("\n=== SERVO MENU ==="));
  Serial.println(F("1) Servo 1   2) Servo 2   3) Servo 3   b) Back"));
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
static void handleServoControl(uint8_t idx, const String& cmdRaw) {
  String word, args;
  splitCmd(cmdRaw, word, args);
  String w = lowerTrim(word);

  if (w == "b")    { menuState = MenuState::SERVO_MENU; printServoSelectMenu(); return; }

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
    servoStop(idx);
    Serial.println(F("STOP -> 1500us")); return;
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

static void printDrillModuleMenu() {
  Serial.println(F("\n=== DRILL MODULE ==="));
  Serial.println(F("(Full speed only — ENA tied to 5V)"));
  Serial.println(F("f       forward"));
  Serial.println(F("r       reverse"));
  Serial.println(F("stop    stop"));
  Serial.println(F("status  show current state"));
  Serial.println(F("b       back"));
  printDrillModuleStatus();
}

static void handleDrillModuleMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (cmd.equalsIgnoreCase("f"))      { drillModule.dir = ActDir::FWD;  drillModuleApply(); printDrillModuleStatus(); return; }
  if (cmd.equalsIgnoreCase("r"))      { drillModule.dir = ActDir::REV;  drillModuleApply(); printDrillModuleStatus(); return; }
  if (cmd.equalsIgnoreCase("stop"))   { drillModule.dir = ActDir::STOP; drillModuleApply(); printDrillModuleStatus(); return; }
  if (cmd.equalsIgnoreCase("status")) { printDrillModuleStatus(); return; }
  Serial.println(F("f | r | stop | status | b"));
}

static void handleMain(const String& cmd) {
  if (cmd == "1" || cmd.equalsIgnoreCase("servos")) {
    menuState = MenuState::SERVO_MENU; printServoSelectMenu(); return;
  }
  if (cmd == "2" || cmd.equalsIgnoreCase("pumps") || cmd.equalsIgnoreCase("uv")) {
    menuState = MenuState::PUMPS_UV_MENU; printPumpsUvMenu(); return;
  }
  if (cmd == "3" || cmd.equalsIgnoreCase("actuator") || cmd.equalsIgnoreCase("linear")) {
    menuState = MenuState::ACTUATOR_MENU; printHBridgeMenu("LINEAR ACTUATOR");
    printHBridgeStatus("ACTUATOR", actuator); return;
  }
  if (cmd == "4" || cmd.equalsIgnoreCase("drill")) {
    menuState = MenuState::DRILL_MENU; printHBridgeMenu("DRILL");
    printHBridgeStatus("DRILL", drill); return;
  }
  if (cmd == "5" || cmd.equalsIgnoreCase("drillmodule") || cmd.equalsIgnoreCase("drill module")) {
    menuState = MenuState::DRILL_MODULE_MENU; printDrillModuleMenu(); return;
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
  for (uint8_t i = 0; i < 3; i++) servoStop(i);

  // Relays
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(relays[i].pin, OUTPUT);
    relays[i].state = false;
    applyRelay(i);
  }

  // H-bridges
  hbridgeInit(actuator);
  hbridgeInit(drill);
  drillModuleInit();

  Serial.println(F("\n=== System Ready ==="));
  Serial.println(F("PCA9685 ch0-2  : continuous rotation servos"));
  Serial.println(F("Actuator       : D2(IN1) D3(IN2) D5(PWM)"));
  Serial.println(F("Drill          : D13(IN1) D12(IN2) D11(PWM)"));
  Serial.println(F("Drill Module   : A0(IN1) A1(IN2)  [ENA->5V, full speed]"));
  printMainMenu();
}

void loop() {
  String cmd = readLineNonBlocking();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("main")) { menuState = MenuState::MAIN; printMainMenu(); return; }

  switch (menuState) {
    case MenuState::MAIN:             handleMain(cmd);            break;
    case MenuState::SERVO_MENU:       handleServoSelect(cmd);     break;
    case MenuState::SERVO_1:          handleServoControl(0, cmd); break;
    case MenuState::SERVO_2:          handleServoControl(1, cmd); break;
    case MenuState::SERVO_3:          handleServoControl(2, cmd); break;
    case MenuState::PUMPS_UV_MENU:    handlePumpsUvMenu(cmd);     break;
    case MenuState::PUMPS_UV_CONTROL: handlePumpsUvControl(cmd);  break;
    case MenuState::ACTUATOR_MENU:    handleActuatorMenu(cmd);       break;
    case MenuState::DRILL_MENU:       handleDrillMenu(cmd);          break;
    case MenuState::DRILL_MODULE_MENU: handleDrillModuleMenu(cmd);   break;
  }
}
