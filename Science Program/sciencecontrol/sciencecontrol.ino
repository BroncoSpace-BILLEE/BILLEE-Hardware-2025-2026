#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Things to fix (implemented below)
// 1) 300 degree servo not working at all  -> change PCA freq to 60Hz + adjust pulse range + add pulse debug command
// 2) Combined for Module 1 and 2 they are moving in the opposite direction -> add invert flag for one module in combined mode

/*
  Robot Controller — Elegoo MEGA 2560 R3 + PCA9685 + Dual RoboClaw (Packet Serial)
  ============================================================================

  Serial Monitor:
    USB Serial -> Serial (115200)

  I2C:
    D20 SDA, D21 SCL -> PCA9685 @0x40

  Relays:
    D22 pump1, D23 pump2, D25 pump3, D24 stirrer, D26 uv_led

  RoboClaw #1 (Drill + Linear Actuator) [Packet Serial on Serial1]
    Mega D18(TX1) -> RoboClaw S1(RX)
    Mega D19(RX1) <- RoboClaw S2(TX)   (optional)
    GND <-> GND (required!)
    RoboClaw M1 -> DRILL
    RoboClaw M2 -> LINEAR ACTUATOR

  RoboClaw #2 (Drill Module 1 + Drill Module 2) [Packet Serial on Serial2]
    Mega D16(TX2) -> RoboClaw S1(RX)
    Mega D17(RX2) <- RoboClaw S2(TX)   (optional)
    GND <-> GND (required!)
    RoboClaw M1 -> DRILL MODULE 1
    RoboClaw M2 -> DRILL MODULE 2

  Motion Studio settings (both RoboClaws):
    Packet Serial Address: 128 (0x80)
    Baudrate: 2400

  Menu commands for each motor:
    f | r | stop | speed <0-255> | status | b

  Drill Modules menu extra:
    test   -> spins Mod1 then Mod2 briefly (RoboClaw #2 sanity test)

  Servo 3 extra debug:
    pulsemin <us>   -> set servo3 min pulse (persist RAM)
    pulsemax <us>   -> set servo3 max pulse (persist RAM)
    pulse <us>      -> write raw pulse (us) to servo3 (for tuning)
*/

// ======================= TYPES =======================
struct ServoState {
  uint8_t  channel;
  int8_t   direction;     // continuous servos: -1 rev, 0 stop, +1 fwd
  uint8_t  speedPct;      // 0..100
  int16_t  commandedPct;  // -100..100
  bool     isPositional;
  uint16_t positionDeg;   // 0..300
};

struct Relay {
  const char* label;
  uint8_t pin;
  bool state;
};

enum class ActDir { STOP, FWD, REV };

enum class MenuState {
  MAIN,
  SERVO_MENU, SERVO_1, SERVO_2, SERVO_3,
  PUMPS_UV_MENU, PUMPS_UV_CONTROL,
  ACTUATOR_MENU,
  DRILL_MENU,
  DRILLMOD_SELECT,
  DRILLMOD1_MENU,
  DRILLMOD2_MENU,
  DRILLMOD_COMBINED
};

// ======================= PIN DEFINITIONS =======================
#define DRILLMOD1_ESTOP 47
#define DRILLMOD2_ESTOP 49

#define RELAY_PIN_1     22
#define RELAY_PIN_2     23
#define RELAY_PIN_3     25
#define RELAY_PIN_4     24
#define RELAY_PIN_5     26

// ======================= PCA9685 SERVO CONSTANTS =======================
static const uint16_t SERVO_US_STOP  = 1500;
static const uint16_t SERVO_US_MAX   = 2000;
static const uint16_t SERVO_US_MIN   = 1000;

// FIX #1: many “300°” servos behave better with a slightly narrower pulse range.
// Start here; tune using pulsemin/pulsemax commands if needed.
static uint16_t SERVO3_US_MIN  =  600;
static uint16_t SERVO3_US_MAX  = 2400;
static const uint16_t SERVO3_DEG_MAX = 300;

// FIX #1: PCA9685 frequency. 50 works for lots of servos, but many position servos are happier at ~60Hz.
static const uint16_t SERVO_PWM_FREQ = 60;

// ======================= ROBOCLAW PACKET SERIAL CONSTANTS =======================
static const uint32_t ROBOCLAW_BAUD = 2400;
static const uint8_t  RC_ADDR_1 = 0x80; // 128
static const uint8_t  RC_ADDR_2 = 0x80; // 128 (OK because different UART)

// FIX #2: one module is physically reversed relative to the other.
// In COMBINED mode only, invert one so they move together.
// If your combined mode still fights, flip this to the other motor.
static const bool DRILLMOD1_INVERT = false;
static const bool DRILLMOD2_INVERT = true;

// ======================= ROBOCLAW TYPES =======================
struct RoboMotor {
  const char* label;
  Stream* port;
  uint8_t addr;
  uint8_t motorIndex; // 1=M1, 2=M2
  ActDir dir;
  uint8_t speed;      // 0..255
};

// Forward declarations
static void robomotorApply(RoboMotor &m);
static void printRoboStatus(RoboMotor &m);
static bool handleRoboMotorCmd(RoboMotor &m, const String& cmd);

static void printServoSelectMenu();
static void printServoControlMenu(uint8_t idx);
static void printServo3ControlMenu();
static void printServoStatus(uint8_t idx);
static void printServo3Status();
static void handleServoSelect(const String& cmd);
static void handleServoControl(uint8_t idx, const String& cmdRaw);
static void handlePumpsUvMenu(const String& cmd);
static void handlePumpsUvControl(const String& cmd);

// ======================= GLOBALS =======================
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

ServoState servos[3] = {
  {   0,   0,   50,   0,    false,      0 },
  {   1,   0,   50,   0,    false,      0 },
  {   2,   0,    0,   0,    true,     150 },
};

Relay relays[] = {
  {"pump1",   RELAY_PIN_1, false},
  {"pump2",   RELAY_PIN_2, false},
  {"pump3",   RELAY_PIN_3, false},
  {"stirrer", RELAY_PIN_4, false},
  {"uv_led",  RELAY_PIN_5, false},
};
static const uint8_t RELAY_COUNT = sizeof(relays) / sizeof(relays[0]);

// RoboClaw #1 (Serial1): Drill=M1, Actuator=M2
RoboMotor drill    = { "DRILL",           &Serial1, RC_ADDR_1, 1, ActDir::STOP, 150 };
RoboMotor actuator = { "LINEAR ACTUATOR", &Serial1, RC_ADDR_1, 2, ActDir::STOP, 150 };

// RoboClaw #2 (Serial2): DrillMod1=M1, DrillMod2=M2
RoboMotor drillMod1 = { "DRILL MODULE 1", &Serial2, RC_ADDR_2, 1, ActDir::STOP, 255 };
RoboMotor drillMod2 = { "DRILL MODULE 2", &Serial2, RC_ADDR_2, 2, ActDir::STOP, 255 };

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
  if (idx > 2 || servos[idx].isPositional) return;
  if (pct >  100) pct =  100;
  if (pct < -100) pct = -100;
  servos[idx].commandedPct = pct;
  pca.setPWM(servos[idx].channel, 0, usToTicks(pctToServUs(pct)));
}

static void servoApply(uint8_t idx) {
  if (servos[idx].isPositional) return;
  ServoState &s = servos[idx];
  int16_t pct = 0;
  if      (s.direction > 0) pct =  (int16_t)s.speedPct;
  else if (s.direction < 0) pct = -(int16_t)s.speedPct;
  servoWritePct(idx, pct);
}

static void servoStop(uint8_t idx) {
  if (servos[idx].isPositional) return;
  servos[idx].direction = 0;
  servoWritePct(idx, 0);
}

static uint16_t servo3DegToUs(uint16_t deg) {
  if (deg > SERVO3_DEG_MAX) deg = SERVO3_DEG_MAX;
  return (uint16_t)(SERVO3_US_MIN +
    (uint32_t)(SERVO3_US_MAX - SERVO3_US_MIN) * (uint32_t)deg / SERVO3_DEG_MAX);
}

static void servo3WriteDeg(uint16_t deg) {
  if (deg > SERVO3_DEG_MAX) deg = SERVO3_DEG_MAX;
  servos[2].positionDeg = deg;
  pca.setPWM(servos[2].channel, 0, usToTicks(servo3DegToUs(deg)));
}

static void servo3WritePulseUs(uint16_t us) {
  // raw pulse write for tuning/debug
  if (us < 400) us = 400;
  if (us > 2600) us = 2600;
  pca.setPWM(servos[2].channel, 0, usToTicks(us));
}

// ======================= ROBOCLAW PACKET SERIAL (CRC16) =======================
// Classic Packet Serial:
//   0: M1 Forward  (1 byte speed 0..127)
//   1: M1 Backward (1 byte speed 0..127)
//   4: M2 Forward  (1 byte speed 0..127)
//   5: M2 Backward (1 byte speed 0..127)

static uint16_t rc_crc16_update(uint16_t crc, uint8_t data) {
  crc ^= (uint16_t)data << 8;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
    else crc <<= 1;
  }
  return crc;
}

static void rc_cmd(Stream &s, uint8_t addr, uint8_t cmd, const uint8_t *data, uint8_t dataLen) {
  uint16_t crc = 0;
  crc = rc_crc16_update(crc, addr);
  crc = rc_crc16_update(crc, cmd);

  s.write(addr);
  s.write(cmd);

  for (uint8_t i = 0; i < dataLen; i++) {
    s.write(data[i]);
    crc = rc_crc16_update(crc, data[i]);
  }

  s.write((uint8_t)(crc >> 8));
  s.write((uint8_t)(crc & 0xFF));
}

static uint8_t speed255_to_127(uint8_t sp) {
  return (uint8_t)((uint16_t)sp * 127U / 255U);
}

static const __FlashStringHelper* dirStr(ActDir d) {
  switch (d) {
    case ActDir::STOP: return F("STOP");
    case ActDir::FWD:  return F("FWD");
    case ActDir::REV:  return F("REV");
  }
  return F("?");
}

// Apply optional invert (used for combined mode correction)
static ActDir maybeInvertDir(ActDir d, bool invert) {
  if (!invert) return d;
  if (d == ActDir::FWD) return ActDir::REV;
  if (d == ActDir::REV) return ActDir::FWD;
  return ActDir::STOP;
}

static void robomotorApply(RoboMotor &m) {
  uint8_t sp = (m.dir == ActDir::STOP) ? 0 : speed255_to_127(m.speed);

  uint8_t cmd = 0;
  if (m.motorIndex == 1) {
    if      (m.dir == ActDir::FWD) cmd = 0;
    else if (m.dir == ActDir::REV) cmd = 1;
    else                           cmd = 0;
  } else {
    if      (m.dir == ActDir::FWD) cmd = 4;
    else if (m.dir == ActDir::REV) cmd = 5;
    else                           cmd = 4;
  }

  uint8_t data[1] = { sp };
  rc_cmd(*m.port, m.addr, cmd, data, 1);
}

static void printRoboStatus(RoboMotor &m) {
  Serial.print(m.label);
  Serial.print(F(": dir=")); Serial.print(dirStr(m.dir));
  Serial.print(F(" speed=")); Serial.println(m.speed);
}

static bool handleRoboMotorCmd(RoboMotor &m, const String& cmd) {
  if (cmd.equalsIgnoreCase("f")) {
    m.dir = ActDir::FWD;  robomotorApply(m); printRoboStatus(m); return true;
  }
  if (cmd.equalsIgnoreCase("r")) {
    m.dir = ActDir::REV;  robomotorApply(m); printRoboStatus(m); return true;
  }
  if (cmd.equalsIgnoreCase("stop")) {
    m.dir = ActDir::STOP; robomotorApply(m); printRoboStatus(m); return true;
  }
  if (cmd.equalsIgnoreCase("status")) {
    printRoboStatus(m); return true;
  }
  if (cmd.startsWith("speed ")) {
    int sp = cmd.substring(6).toInt();
    if (sp < 0) sp = 0; if (sp > 255) sp = 255;
    m.speed = (uint8_t)sp;
    robomotorApply(m);
    printRoboStatus(m);
    return true;
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

static void printMotorMenu(const char* label) {
  Serial.print(F("\n=== ")); Serial.print(label); Serial.println(F(" ==="));
  Serial.println(F("f             forward"));
  Serial.println(F("r             reverse"));
  Serial.println(F("stop          stop"));
  Serial.println(F("speed <0-255> set speed"));
  Serial.println(F("status        show current state"));
  Serial.println(F("b             back"));
}

static void printDrillModSelectMenu() {
  Serial.println(F("\n=== DRILL MODULES ==="));
  Serial.println(F("1) Drill Module 1  (individual)"));
  Serial.println(F("2) Drill Module 2  (individual)"));
  Serial.println(F("3) Combined        (both together)"));
  Serial.println(F("b) Back to main"));
  Serial.println(F("test              (spin Mod1 then Mod2 briefly)"));
}

static void drillModulesTest() {
  Serial.println(F("[TEST] RoboClaw #2 Mod1 forward @120 for 1s"));
  drillMod1.speed = 120; drillMod1.dir = ActDir::FWD; robomotorApply(drillMod1);
  delay(1000);
  drillMod1.dir = ActDir::STOP; robomotorApply(drillMod1);
  delay(250);

  Serial.println(F("[TEST] RoboClaw #2 Mod2 forward @120 for 1s"));
  drillMod2.speed = 120; drillMod2.dir = ActDir::FWD; robomotorApply(drillMod2);
  delay(1000);
  drillMod2.dir = ActDir::STOP; robomotorApply(drillMod2);

  Serial.println(F("[TEST] Done"));
}

static void printServoSelectMenu() {
  Serial.println(F("\n=== SERVO MENU ==="));
  Serial.println(F("1) Servo 1  (continuous rotation)"));
  Serial.println(F("2) Servo 2  (continuous rotation)"));
  Serial.println(F("3) Servo 3  (300-degree positional)"));
  Serial.println(F("b) Back"));
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

static void printServo3ControlMenu() {
  ServoState &s = servos[2];
  Serial.println(F("\n=== SERVO 3 (300-degree positional) ==="));
  Serial.println(F("pos <0-300>      move to angle in degrees"));
  Serial.println(F("centre           move to 150 deg (centre)"));
  Serial.println(F("min              move to 0 deg   (full CCW)"));
  Serial.println(F("max              move to 300 deg (full CW)"));
  Serial.println(F("pulse <us>       write raw pulse width (debug)"));
  Serial.println(F("pulsemin <us>    set servo3 min pulse (default 600)"));
  Serial.println(F("pulsemax <us>    set servo3 max pulse (default 2400)"));
  Serial.println(F("status           show current position + pulses"));
  Serial.println(F("b                back"));
  Serial.print(F("Now: pos="));  Serial.print(s.positionDeg);
  Serial.print(F("deg ("));      Serial.print(servo3DegToUs(s.positionDeg));
  Serial.println(F("us)"));
  Serial.print(F("Range: ")); Serial.print(SERVO3_US_MIN);
  Serial.print(F("..")); Serial.print(SERVO3_US_MAX);
  Serial.println(F("us"));
}

static void printServo3Status() {
  ServoState &s = servos[2];
  Serial.print(F("S3: pos="));
  Serial.print(s.positionDeg);
  Serial.print(F("deg  pulse="));
  Serial.print(servo3DegToUs(s.positionDeg));
  Serial.print(F("us  range="));
  Serial.print(SERVO3_US_MIN);
  Serial.print(F(".."));
  Serial.print(SERVO3_US_MAX);
  Serial.println(F("us"));
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
static void handleDrillModSelect(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (cmd.equalsIgnoreCase("test")) { drillModulesTest(); return; }

  if (cmd == "1") { menuState = MenuState::DRILLMOD1_MENU; printMotorMenu("DRILL MODULE 1"); printRoboStatus(drillMod1); return; }
  if (cmd == "2") { menuState = MenuState::DRILLMOD2_MENU; printMotorMenu("DRILL MODULE 2"); printRoboStatus(drillMod2); return; }
  if (cmd == "3") {
    menuState = MenuState::DRILLMOD_COMBINED;
    Serial.println(F("\n=== DRILL MODULES COMBINED ==="));
    Serial.println(F("Commands apply to BOTH Module 1 and Module 2 simultaneously."));
    Serial.println(F("NOTE: combined mode applies invert flags so both move same physical direction."));
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
    printRoboStatus(drillMod1);
    printRoboStatus(drillMod2);
    return;
  }
  Serial.println(F("Pick 1/2/3, test, or b"));
}

static void handleServoControlContinuous(uint8_t idx, const String& cmdRaw) {
  String word, args;
  splitCmd(cmdRaw, word, args);
  String w = lowerTrim(word);

  if (w == "b") { menuState = MenuState::SERVO_MENU; printServoSelectMenu(); return; }
  if (w == "f") { servos[idx].direction = +1; servoApply(idx); Serial.println(F("FWD")); return; }
  if (w == "r") { servos[idx].direction = -1; servoApply(idx); Serial.println(F("REV")); return; }
  if (w == "stop") { servoStop(idx); Serial.println(F("STOP")); return; }

  if (w == "speed") {
    args.trim();
    if (args.length() == 0 || !isInteger(args)) { Serial.println(F("ERR: speed <0-100>")); return; }
    int v = args.toInt();
    if (v < 0) v = 0; if (v > 100) v = 100;
    servos[idx].speedPct = (uint8_t)v;
    if (servos[idx].direction != 0) servoApply(idx);
    Serial.print(F("speed=")); Serial.print(v); Serial.println(F("%"));
    return;
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
    Serial.print(F("set ")); Serial.println(v);
    return;
  }

  if (w == "status") { printServoStatus(idx); return; }
  Serial.println(F("? Use: f | r | stop | speed <0-100> | set <-100,100> | status | b"));
}

static void handleServoControl3(const String& cmdRaw) {
  String word, args;
  splitCmd(cmdRaw, word, args);
  String w = lowerTrim(word);

  if (w == "b") { menuState = MenuState::SERVO_MENU; printServoSelectMenu(); return; }
  if (w == "centre" || w == "center") { servo3WriteDeg(150); Serial.println(F("Moved to 150deg")); return; }
  if (w == "min") { servo3WriteDeg(0); Serial.println(F("Moved to 0deg")); return; }
  if (w == "max") { servo3WriteDeg(SERVO3_DEG_MAX); Serial.println(F("Moved to 300deg")); return; }

  if (w == "pos") {
    args.trim();
    if (args.length() == 0 || !isInteger(args)) { Serial.println(F("ERR: pos <0-300>")); return; }
    int v = args.toInt();
    if (v < 0) v = 0;
    if (v > (int)SERVO3_DEG_MAX) v = (int)SERVO3_DEG_MAX;
    servo3WriteDeg((uint16_t)v);
    Serial.print(F("Moved to ")); Serial.print(v); Serial.println(F("deg"));
    return;
  }

  // NEW: tuning helpers
  if (w == "pulse") {
    args.trim();
    if (args.length() == 0 || !isInteger(args)) { Serial.println(F("ERR: pulse <us>")); return; }
    int us = args.toInt();
    servo3WritePulseUs((uint16_t)us);
    Serial.print(F("Pulse=")); Serial.print(us); Serial.println(F("us"));
    return;
  }
  if (w == "pulsemin") {
    args.trim();
    if (args.length() == 0 || !isInteger(args)) { Serial.println(F("ERR: pulsemin <us>")); return; }
    int us = args.toInt();
    if (us < 400) us = 400;
    if (us > (int)SERVO3_US_MAX - 50) us = (int)SERVO3_US_MAX - 50;
    SERVO3_US_MIN = (uint16_t)us;
    Serial.print(F("SERVO3_US_MIN=")); Serial.println(SERVO3_US_MIN);
    return;
  }
  if (w == "pulsemax") {
    args.trim();
    if (args.length() == 0 || !isInteger(args)) { Serial.println(F("ERR: pulsemax <us>")); return; }
    int us = args.toInt();
    if (us > 2600) us = 2600;
    if (us < (int)SERVO3_US_MIN + 50) us = (int)SERVO3_US_MIN + 50;
    SERVO3_US_MAX = (uint16_t)us;
    Serial.print(F("SERVO3_US_MAX=")); Serial.println(SERVO3_US_MAX);
    return;
  }

  if (w == "status") { printServo3Status(); return; }
  Serial.println(F("? Use: pos <0-300> | centre | min | max | pulse <us> | pulsemin <us> | pulsemax <us> | status | b"));
}

static void handleServoControl(uint8_t idx, const String& cmdRaw) {
  if (idx == 2) handleServoControl3(cmdRaw);
  else          handleServoControlContinuous(idx, cmdRaw);
}

static void handleServoSelect(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (cmd == "1") { menuState = MenuState::SERVO_1; printServoControlMenu(0); return; }
  if (cmd == "2") { menuState = MenuState::SERVO_2; printServoControlMenu(1); return; }
  if (cmd == "3") { menuState = MenuState::SERVO_3; printServo3ControlMenu(); return; }
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
  if (!handleRoboMotorCmd(actuator, cmd))
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

static void handleDrillMenu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::MAIN; printMainMenu(); return; }
  if (!handleRoboMotorCmd(drill, cmd))
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

static void handleDrillMod1Menu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::DRILLMOD_SELECT; printDrillModSelectMenu(); return; }
  if (!handleRoboMotorCmd(drillMod1, cmd))
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

static void handleDrillMod2Menu(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::DRILLMOD_SELECT; printDrillModSelectMenu(); return; }
  if (!handleRoboMotorCmd(drillMod2, cmd))
    Serial.println(F("f | r | stop | speed <0-255> | status | b"));
}

// FIX #2 is implemented here: combined applies invert flags so both move together physically.
static void handleDrillModCombined(const String& cmd) {
  if (cmd == "b") { menuState = MenuState::DRILLMOD_SELECT; printDrillModSelectMenu(); return; }

  if (cmd.equalsIgnoreCase("status")) {
    printRoboStatus(drillMod1);
    printRoboStatus(drillMod2);
    Serial.print(F("Combined invert: Mod1=")); Serial.print(DRILLMOD1_INVERT ? F("YES") : F("NO"));
    Serial.print(F(" Mod2=")); Serial.println(DRILLMOD2_INVERT ? F("YES") : F("NO"));
    return;
  }

  // We parse commands here and then apply with inversion,
  // instead of calling handleRoboMotorCmd twice directly.
  if (cmd.equalsIgnoreCase("f") || cmd.equalsIgnoreCase("r") || cmd.equalsIgnoreCase("stop") || cmd.startsWith("speed ")) {
    // copy current
    ActDir desiredDir = ActDir::STOP;
    bool dirChange = false;
    bool speedChange = false;

    if (cmd.equalsIgnoreCase("f")) { desiredDir = ActDir::FWD; dirChange = true; }
    else if (cmd.equalsIgnoreCase("r")) { desiredDir = ActDir::REV; dirChange = true; }
    else if (cmd.equalsIgnoreCase("stop")) { desiredDir = ActDir::STOP; dirChange = true; }
    else if (cmd.startsWith("speed ")) {
      int sp = cmd.substring(6).toInt();
      if (sp < 0) sp = 0; if (sp > 255) sp = 255;
      drillMod1.speed = (uint8_t)sp;
      drillMod2.speed = (uint8_t)sp;
      speedChange = true;
    }

    if (dirChange) {
      drillMod1.dir = maybeInvertDir(desiredDir, DRILLMOD1_INVERT);
      drillMod2.dir = maybeInvertDir(desiredDir, DRILLMOD2_INVERT);
    }

    // apply both
    robomotorApply(drillMod1);
    robomotorApply(drillMod2);

    if (dirChange || speedChange) {
      Serial.println(F("[COMBINED] Applied to both modules (with invert correction)."));
      printRoboStatus(drillMod1);
      printRoboStatus(drillMod2);
    }
    return;
  }

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
    printMotorMenu("LINEAR ACTUATOR"); printRoboStatus(actuator); return;
  }
  if (cmd == "4" || cmd.equalsIgnoreCase("drill")) {
    menuState = MenuState::DRILL_MENU;
    printMotorMenu("DRILL"); printRoboStatus(drill); return;
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

  Serial1.begin(ROBOCLAW_BAUD); // D18/D19
  Serial2.begin(ROBOCLAW_BAUD); // D16/D17
  delay(200);

  pca.begin();
  pca.setPWMFreq(SERVO_PWM_FREQ);
  delay(10);

  for (uint8_t i = 0; i < 2; i++) servoStop(i);
  servo3WriteDeg(servos[2].positionDeg);

  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(relays[i].pin, OUTPUT);
    relays[i].state = false;
    applyRelay(i);
  }

  // You said you aren't using E-stops right now; keep inputs pulled up to avoid floating
  pinMode(DRILLMOD1_ESTOP, INPUT_PULLUP);
  pinMode(DRILLMOD2_ESTOP, INPUT_PULLUP);

  // Stop all motors at boot
  drill.dir = ActDir::STOP;        robomotorApply(drill);
  actuator.dir = ActDir::STOP;     robomotorApply(actuator);
  drillMod1.dir = ActDir::STOP;    robomotorApply(drillMod1);
  drillMod2.dir = ActDir::STOP;    robomotorApply(drillMod2);

  Serial.println(F("\n=== System Ready (MEGA 2560 + Dual RoboClaw Packet Serial) ==="));
  Serial.println(F("Packet Serial addr=128 (0x80), baud=2400"));
  Serial.println(F("RoboClaw #1: Serial1 D18(TX1)->S1, D19(RX1)<-S2"));
  Serial.println(F("RoboClaw #2: Serial2 D16(TX2)->S1, D17(RX2)<-S2"));

  Serial.println(F("\nServo3 tweaks: PCA9685 freq=60Hz, default pulse range 600..2400us"));
  Serial.println(F("If Servo3 still doesn't move, use: pulse 1500  / pulsemin 700 / pulsemax 2300"));
  Serial.println(F("Combined mode invert: Mod1="));
  Serial.println(DRILLMOD1_INVERT ? F("YES") : F("NO"));
  Serial.print(F(" Mod2="));
  Serial.println(DRILLMOD2_INVERT ? F("YES") : F("NO"));

  printMainMenu();
}

void loop() {
  String cmd = readLineNonBlocking();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("main")) { menuState = MenuState::MAIN; printMainMenu(); return; }

  switch (menuState) {
    case MenuState::MAIN:              handleMain(cmd);               break;
    case MenuState::SERVO_MENU:        handleServoSelect(cmd);        break;
    case MenuState::SERVO_1:           handleServoControl(0, cmd);    break;
    case MenuState::SERVO_2:           handleServoControl(1, cmd);    break;
    case MenuState::SERVO_3:           handleServoControl(2, cmd);    break;
    case MenuState::PUMPS_UV_MENU:     handlePumpsUvMenu(cmd);        break;
    case MenuState::PUMPS_UV_CONTROL:  handlePumpsUvControl(cmd);     break;
    case MenuState::ACTUATOR_MENU:     handleActuatorMenu(cmd);       break;
    case MenuState::DRILL_MENU:        handleDrillMenu(cmd);          break;
    case MenuState::DRILLMOD_SELECT:   handleDrillModSelect(cmd);     break;
    case MenuState::DRILLMOD1_MENU:    handleDrillMod1Menu(cmd);      break;
    case MenuState::DRILLMOD2_MENU:    handleDrillMod2Menu(cmd);      break;
    case MenuState::DRILLMOD_COMBINED: handleDrillModCombined(cmd);   break;
  }
}