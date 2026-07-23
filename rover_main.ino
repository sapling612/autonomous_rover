/*
  Rover — USB + SMS Command Console ( Mega 2560 + SIM7000 on Serial1 @ 9600 )

  Includes:
   - 4x Ultrasonic (F/R/L/B)
   - Motor drive (IN-only pins), forward/back (front & back safety), precise L/R turns
   - Spin calibration values W_SPIN (deg/s) + T0_US (us) persisted to EEPROM
   - Forward speed V_FWD (m/s) persisted to EEPROM via CALV/MOVED
   - Phone number persisted to EEPROM with MAGIC header (robust to random bytes)
   - USB Serial command console @ 250000 baud
   - SMS command console via inbox polling (execute same commands; reply via SMS)
   - Bluetooth joystick (HC-05 on Serial2) using JF/JB/JL/JR/JS commands

  Commands (USB or SMS):
    US                    -> read all ultrasonics
    USF / USR / USL / USB -> read one
    F <feet>              -> forward distance (with avoidance)
    B <feet>              -> backward distance (with rear safety)
    L <deg> / R <deg>     -> left/right turn by degrees (uses W_SPIN, T0_US)
    SETVFWD <m/s>         -> set forward speed constant manually
    CALV                  -> auto-calibrate V_FWD (1 second drive)
    MOVED <feet>          -> tell CALV how far it moved
    SETW <deg/s>          -> set spin rate constant (50..5000) & save
    SETT0 <us>            -> set turn overhead (0..200000) & save
    READALL               -> print V_FWD, W_SPIN, T0_US
    STOPCM <cm>           -> set safety stop distance
    SMS "message"         -> send an SMS to stored/active number
    SMSUS                 -> SMS a sensor report

  USB-only (but allowed by SMS too if you want):
    SMSNUM <number>       -> set active phone (+15551234567)
    SAVENUM               -> save active phone to EEPROM
    READNUM               -> show phone in EEPROM
    CLRNUM                -> clear phone in EEPROM
    HELP / ?              -> menu

  Joystick (Bluetooth HC-05 on Serial2):
    JOYON / JOYOFF        -> enable/disable joystick mode
    JF / JB / JL / JR     -> live forward/back/left/right
    JS                    -> stop

  Notes:
   - SIM7000 on Serial1: TX1=18 to SIM RX, RX1=19 to SIM TX, GND common.
*/

#include <EEPROM.h>
#include <math.h>

// ---------- Motor pins (IN-only) ----------
const int RR_IN1 = 4;
const int RR_IN2 = 5;
const int LR_IN1 = 7;
const int LR_IN2 = 6;
const int FR_IN1 = 8;
const int FR_IN2 = 9;
const int FL_IN1 = 10;
const int FL_IN2 = 11;

// ---------- Ultrasonic pins ----------
const int US_TRIG_F = 23, US_ECHO_F = 22;
const int US_TRIG_B = 25, US_ECHO_B = 24;
const int US_TRIG_L = 27, US_ECHO_L = 26;
const int US_TRIG_R = 29, US_ECHO_R = 28;

// ---------- Ultrasonic streaming (USB only) ----------
bool gUSStream = false;
unsigned long lastUSStreamMs = 0;
const unsigned long US_STREAM_PERIOD_MS = 200;  // print every 200 ms

// ---------- Serials ----------
const unsigned long USB_BAUD   = 250000; // Serial (monitor)
const unsigned long MODEM_BAUD = 9600;   // Serial1 (SIM7000)

// Bluetooth on Serial2 (TX2=16, RX2=17)
HardwareSerial &BT = Serial2;

// ---------- Modem ready state ----------
bool gModemReady = false;
unsigned long lastModemInitAttemptMs = 0;
const unsigned long MODEM_RETRY_MS = 8000;  // retry every 8 s if not ready

// ---------- Forward/turn config ----------
const int   RAMP_MS = 300;     // settle for straight moves
float       STOP_CM = 25.0;    // safety stop (used F & B)
float       V_FWD   = 0.642f;  // m/s (set via SETVFWD or CALV/MOVED)
float       W_SPIN  = 211.9f;  // deg/s (set via SETW)
long        T0_US   = 4800;    // microseconds (set via SETT0)

// ---------- Obstacle avoidance tuning ----------
const float FWD_STEP_M        = 0.50f;  // forward step size for F
const float AVOID_BACKUP_M    = 0.20f;  // backup distance before avoiding
const float AVOID_SIDESTEP_M  = 0.40f;  // sideways distance
const float AVOID_FORWARD_M   = 0.60f;  // forward distance along obstacle

// ---------- EEPROM map ----------
const uint32_t MAGIC = 0xC0FFEE42;
struct CalLog { uint32_t magic; uint32_t run_ms; }; // reserved region
const int EE_CALLOG_ADDR = 0;
const int EE_W_ADDR      = EE_CALLOG_ADDR + sizeof(CalLog); // float W_SPIN
const int EE_T0_ADDR     = EE_W_ADDR + sizeof(float);       // long  T0_US

// ----- Phone blob with its own MAGIC -----
const uint32_t PHONE_MAGIC = 0xFACEB00C;
const int EE_PHONE_ADDR    = EE_T0_ADDR + sizeof(long);  // start of phone block
#define PHONE_MAX_LEN   24  // up to 23 chars + null-ish
const int PHONE_BLOCK_SIZE = 4 + 1 + PHONE_MAX_LEN;      // magic + len + chars

// ----- V_FWD stored after phone block -----
const int EE_V_ADDR = EE_PHONE_ADDR + PHONE_BLOCK_SIZE;

static String gPhone = "";  // active phone in RAM

// ---------- Drive straight trim (1.0 = full power) ----------
const float TRIM_LEFT  = 1.00f;  // reduce this if LEFT side is too strong
const float TRIM_RIGHT = 0.80f;  // reduce this if RIGHT side is too strong
const unsigned long TRIM_PERIOD_MS = 20;  // mini-PWM period for trim

// ---------- SMS inbox polling ----------
unsigned long lastSMSPollMs = 0;
const unsigned long SMS_POLL_EVERY_MS = 500;   // poll inbox every 0.5 s

// ---------- Joystick mode (Bluetooth HC-05 on Serial2) ----------
bool gJoystickMode = false;              // true while joystick is "in charge"
unsigned long lastJoystickCmdMs = 0;     // last time we saw a J* command
const unsigned long JOY_TIMEOUT_MS = 300; // ms with no J* → coast/stop

// ---------- CALV state ----------
bool  gCalvPending = false;
float gCalvTime_s  = 1.0f;  // currently always 1 second

// Forward declarations
String handleCommand(const String &lineIn, bool fromSMS, const String &smsSender);
void driveForwardMeters(float m);
void driveBackwardMeters(float m);
void driveForwardMetersChunk(float m, bool frontSafety);
bool avoidObstacleAroundFront(float *remainingPtr);
void runCalV();
void turnLeftDeg(float deg);
void turnRightDeg(float deg);

// ---------- Helpers: motors ----------
void allCoast(){
  digitalWrite(RR_IN1,LOW); digitalWrite(RR_IN2,LOW);
  digitalWrite(LR_IN1,LOW); digitalWrite(LR_IN2,LOW);
  digitalWrite(FR_IN1,LOW); digitalWrite(FR_IN2,LOW);
  digitalWrite(FL_IN1,LOW); digitalWrite(FL_IN2,LOW);
}

void allForward(){
  // Right forward (apply TRIM_RIGHT)
  if (TRIM_RIGHT >= 1.0f) {
    digitalWrite(RR_IN1,LOW);
    digitalWrite(RR_IN2,HIGH);
    digitalWrite(FR_IN1,LOW);
    digitalWrite(FR_IN2,HIGH);
  } else {
    analogWrite(RR_IN2, (int)(255 * TRIM_RIGHT));
    analogWrite(FR_IN2, (int)(255 * TRIM_RIGHT));
    digitalWrite(RR_IN1,LOW);
    digitalWrite(FR_IN1,LOW);
  }

  // Left forward (apply TRIM_LEFT)
  if (TRIM_LEFT >= 1.0f) {
    digitalWrite(LR_IN1,HIGH);
    digitalWrite(LR_IN2,LOW);
    digitalWrite(FL_IN1,LOW);
    digitalWrite(FL_IN2,HIGH);
  } else {
    analogWrite(LR_IN1, (int)(255 * TRIM_LEFT));
    analogWrite(FL_IN2, (int)(255 * TRIM_LEFT));
    digitalWrite(LR_IN2,LOW);
    digitalWrite(FL_IN1,LOW);
  }
}

void spinLeft(){
  digitalWrite(RR_IN1,LOW);  digitalWrite(RR_IN2,HIGH);
  digitalWrite(FR_IN1,LOW);  digitalWrite(FR_IN2,HIGH);
  digitalWrite(LR_IN1,LOW);  digitalWrite(LR_IN2,HIGH);
  digitalWrite(FL_IN1,HIGH); digitalWrite(FL_IN2,LOW);
}

void spinRight(){
  digitalWrite(RR_IN1,HIGH); digitalWrite(RR_IN2,LOW);
  digitalWrite(FR_IN1,HIGH); digitalWrite(FR_IN2,LOW);
  digitalWrite(LR_IN1,HIGH); digitalWrite(LR_IN2,LOW);
  digitalWrite(FL_IN1,LOW);  digitalWrite(FL_IN2,HIGH);
}

// Raw "always backward" (no timing / no safety) for joystick mode
void allBackwardRaw() {
  digitalWrite(RR_IN1,HIGH); digitalWrite(RR_IN2,LOW);
  digitalWrite(FR_IN1,HIGH); digitalWrite(FR_IN2,LOW);
  digitalWrite(LR_IN1,LOW);  digitalWrite(LR_IN2,HIGH);
  digitalWrite(FL_IN1,HIGH); digitalWrite(FL_IN2,LOW);
}

// Joystick-mode helpers: direct, continuous motion
void joyStop() {
  allCoast();
}

void joyForward() {
  allForward();
}

void joyBackward() {
  allBackwardRaw();
}

void joyTurnLeft() {
  spinLeft();
}

void joyTurnRight() {
  spinRight();
}

// Drive forward for a fixed duration with left/right trim and optional front safety
void forwardTrimmedForMs(unsigned long durationMs, bool frontSafety){
  if (durationMs == 0) return;

  unsigned long start = millis();
  while (millis() - start < durationMs){
    unsigned long now   = millis();
    unsigned long phase = now % TRIM_PERIOD_MS;

    unsigned long leftOnMs  = (unsigned long)(TRIM_LEFT  * TRIM_PERIOD_MS);
    unsigned long rightOnMs = (unsigned long)(TRIM_RIGHT * TRIM_PERIOD_MS);

    // ----- Right side -----
    if (phase < rightOnMs){
      // right forward
      digitalWrite(RR_IN1,LOW);  digitalWrite(RR_IN2,HIGH);
      digitalWrite(FR_IN1,LOW);  digitalWrite(FR_IN2,HIGH);
    } else {
      // right coast
      digitalWrite(RR_IN1,LOW);  digitalWrite(RR_IN2,LOW);
      digitalWrite(FR_IN1,LOW);  digitalWrite(FR_IN2,LOW);
    }

    // ----- Left side -----
    if (phase < leftOnMs){
      // left forward
      digitalWrite(LR_IN1,HIGH); digitalWrite(LR_IN2,LOW);
      digitalWrite(FL_IN1,LOW);  digitalWrite(FL_IN2,HIGH);
    } else {
      // left coast
      digitalWrite(LR_IN1,LOW);  digitalWrite(LR_IN2,LOW);
      digitalWrite(FL_IN1,LOW);  digitalWrite(FL_IN2,LOW);
    }

    // Safety check on front if enabled
    if (frontSafety){
      long cm = ultraCM(US_TRIG_F, US_ECHO_F);
      if (cm > 0 && cm < (long)STOP_CM){
        Serial.println("Safety stop (front, trimmed).");
        break;
      }
    }

    delay(2); // keep loop from spinning insanely fast
  }

  allCoast();
  delay(RAMP_MS);
}

// ---------- Ultrasonic ----------
long ultraCM(int trig, int echo){
  digitalWrite(trig,LOW); 
  delayMicroseconds(2);
  digitalWrite(trig,HIGH); 
  delayMicroseconds(10);
  digitalWrite(trig,LOW);

  long dur = pulseIn(echo, HIGH, 30000UL);

  if (!dur) {
    // No echo within 30 ms → treat as "very far away"
    return 1000;            // 10 m, bigger than anything you care about
  }

  return dur / 58;          // normal cm conversion
}

// ---------- EEPROM: phone with MAGIC ----------
String normalizePhone(const String &s) {
  String out; out.reserve(s.length());
  for (size_t i=0;i<s.length();++i){
    char c = s[i];
    if ((c >= '0' && c <= '9') || (c=='+' && out.length()==0)) out += c;
  }
  return out;
}

bool eeReadPhone(String &out) {
  uint32_t magic;
  EEPROM.get(EE_PHONE_ADDR, magic);
  if (magic != PHONE_MAGIC) return false;

  uint8_t len = EEPROM.read(EE_PHONE_ADDR + 4);
  if (len == 0 || len >= PHONE_MAX_LEN) return false;

  char buf[PHONE_MAX_LEN];
  for (uint8_t i=0; i<len; ++i) {
    buf[i] = (char)EEPROM.read(EE_PHONE_ADDR + 5 + i);
  }
  buf[len] = '\0';
  out = String(buf);
  return true;
}

void eeWritePhone(const String &num) {
  String s = num; s.trim();
  if (s.length() >= PHONE_MAX_LEN) s = s.substring(0, PHONE_MAX_LEN-1);
  uint8_t len = (uint8_t)s.length();

  EEPROM.put(EE_PHONE_ADDR, PHONE_MAGIC);               // magic
  EEPROM.update(EE_PHONE_ADDR + 4, len);                // length
  for (uint8_t i=0; i<len; ++i) {                       // chars
    EEPROM.update(EE_PHONE_ADDR + 5 + i, (uint8_t)s[i]);
  }
  for (uint8_t i=len; i<PHONE_MAX_LEN-1; ++i) {         // clear rest
    EEPROM.update(EE_PHONE_ADDR + 5 + i, 0);
  }
}

void eeClearPhone() {
  uint32_t blank = 0xFFFFFFFF;
  EEPROM.put(EE_PHONE_ADDR, blank);
}

// ---------- EEPROM: spin/T0/V_FWD ----------
void eeSaveW(float w){ EEPROM.put(EE_W_ADDR, w); }
bool eeLoadW(float &wOut){
  float w; EEPROM.get(EE_W_ADDR, w);
  if (w > 0.0f && w < 5000.0f){ wOut=w; return true; }
  return false;
}

void eeSaveT0(long t){ EEPROM.put(EE_T0_ADDR, t); }
bool eeLoadT0(long &tOut){
  long t; EEPROM.get(EE_T0_ADDR, t);
  if (t>=0 && t<=200000){ tOut=t; return true; }
  return false;
}

float clampW(float w){ if (w<50) w=50; if (w>5000) w=5000; return w; }
long  clampT0(long t){ if (t<0) t=0; if (t>200000) t=200000; return t; }

void eeSaveV(float v){ EEPROM.put(EE_V_ADDR, v); }
bool eeLoadV(float &vOut){
  float v; EEPROM.get(EE_V_ADDR, v);
  if (v > 0.05f && v < 3.0f){ vOut = v; return true; }
  return false;
}

// ---------- SIM7000 helpers ----------
void mFlush(){ while(Serial1.available()) Serial1.read(); }

void mLine(const String&s){ Serial1.print(s); Serial1.print("\r"); }

String modemRaw(const String &cmd, uint16_t timeoutMs) {
  mFlush();
  mLine(cmd);  // sends cmd + "\r"

  unsigned long t0 = millis();
  String buf;
  while (millis() - t0 < timeoutMs) {
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      buf += c;
    }
  }
  Serial.print("MODEM resp for [");
  Serial.print(cmd);
  Serial.println("]:");
  Serial.println(buf);
  Serial.println("----");
  return buf;
}

bool waitToken(const char* token, uint16_t ms){
  unsigned long t0 = millis();
  String buf;
  while (millis()-t0 < ms){
    while (Serial1.available()){
      char c = (char)Serial1.read();
      buf += c;
      if (buf.indexOf(token)>=0) return true;
    }
  }
  return false;
}

bool waitOK(uint16_t ms){ return waitToken("OK", ms); }

bool waitPrompt(uint16_t ms){
  unsigned long t0 = millis();
  while (millis()-t0 < ms){
    while (Serial1.available()){
      if ((char)Serial1.read() == '>') return true;
    }
  }
  return false;
}

bool waitOKOnSerial1(uint16_t ms) {
  unsigned long t0 = millis();
  String buf;
  while (millis() - t0 < ms) {
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      buf += c;
      Serial.write(c);   // echo to USB so you can see it
      if (buf.indexOf("OK") >= 0) {
        return true;
      }
    }
  }
  return false;
}

bool probeBaud(long baud) {
  Serial.print("Probing baud ");
  Serial.print(baud);
  Serial.println("...");

  Serial1.end();
  delay(50);
  Serial1.begin(baud);
  mFlush();

  // Give modem a moment after baud change
  delay(100);

  // Try AT a few times at this baud
  for (int i = 0; i < 4; i++) {
    mLine("AT");
    Serial.println(">> AT");
    if (waitOKOnSerial1(500)) {
      Serial.print("\nBaud ");
      Serial.print(baud);
      Serial.println(" works!");
      return true;
    }
    delay(200);
  }

  Serial.print("No OK at ");
  Serial.println(baud);
  return false;
}

bool setCPMSToSM(uint16_t timeoutMs = 3000) {
  for (int i = 0; i < 3; i++) {
    Serial.println("[SMS] Setting CPMS to SM...");
    mLine("AT+CPMS=\"SM\",\"SM\",\"SM\"");
    if (waitOK(timeoutMs)) {
      Serial.println("[SMS] CPMS -> SM OK");
      return true;
    }
    Serial.println("[SMS] CPMS SM failed, retrying...");
    delay(500);
  }
  Serial.println("[SMS] CPMS SM FAILED after retries");
  return false;           // <-- no gModemReady flip here
}

// Robust modem init: keep trying AT, then disable power-save stuff
bool modemInit() {
  mFlush();
  Serial.println("Init modem (quick)…");

  // --- 1) Try AT a few times quickly (~2–3 s total) ---
  bool gotAT = false;
  for (int i = 0; i < 5 && !gotAT; i++) {
    mLine("AT");
    Serial.println(">> AT");
    gotAT = waitOK(400);   // wait up to 400 ms for "OK"
    if (!gotAT) {
      delay(200);          // short pause between pokes
    }
  }

  if (!gotAT) {
    Serial.println("modemInit: no AT response, giving up.");
    return false;
  }

  Serial.println("modemInit: AT OK, doing quick config…");

  // helper: fire command, ignore if no OK (non-fatal)
  auto poke = [&](const char *cmd, uint16_t ms) {
    mLine(cmd);
    Serial.print(">> ");
    Serial.println(cmd);
    waitOK(ms);   // we don't care if it times out, just best effort
  };

  // --- 2) Minimal but important setup, with short timeouts ---
  poke("ATE0",               500);   // echo off
  poke("AT+CMEE=2",          500);   // verbose errors
  poke("AT+CMGF=1",         1000);   // SMS text mode
  poke("AT+CSCS=\"GSM\"",    500);
  poke("AT+CSMP=17,167,0,0", 500);
  poke("AT+CNMI=1,0,0,0,0",  500);   // we poll inbox manually
  poke("AT+CMGF=1", 1000);          // force SMS text mode (IMPORTANT)
  poke("AT+CSCS=\"GSM\"", 500);     // ensure GSM 7-bit alphabet (IMPORTANT)

  // Stronger, retried CPMS instead of best-effort poke
  setCPMSToSM();

  mLine("AT+CREG?");
  waitOK(1000);
  mLine("AT+CSQ");
  waitOK(1000);

  // keep modem awake (also short, non-fatal)
  poke("AT+CSCLK=0",         500);
  poke("AT+CPSMS=0",         500);
  poke("AT+CEDRXS=0",        500);

  Serial.println("modemInit: done (quick).");
  return true;
}

bool smsSendTo(const String &number, const String &text){
  if (!number.length()) return false;
  mFlush();
  Serial1.print("AT+CMGS=\""); Serial1.print(number); Serial1.print("\"\r");
  if (!waitPrompt(4000)) {
    Serial.println("[SMS] No '>' prompt for CMGS.");
    return false;
  }
  Serial1.print(text);
  delay(30);
  Serial1.write((char)26); // Ctrl+Z

  unsigned long t0 = millis();
  String resp;
  while (millis()-t0 < 15000){
    while (Serial1.available()){
      char c = (char)Serial1.read();
      resp += c;
      if (resp.indexOf("OK")>=0) {
        Serial.println("[SMS] CMGS OK:");
        Serial.println(resp);
        return true;
      }
      if (resp.indexOf("ERROR")>=0 || resp.indexOf("+CMS ERROR")>=0) {
        Serial.println("[SMS] CMGS ERROR:");
        Serial.println(resp);
        return false;
      }
    }
  }

  Serial.println("[SMS] CMGS timeout, resp was:");
  Serial.println(resp);
  return false;
}

// Poll unread inbox; for each SMS: parse sender & body, run command, reply
void pollSMSInbox() {
  if (!gModemReady) return;  // keep this

  Serial.println("[SMS] pollSMSInbox() called");

  // Just ask for unread SMS. No extra AT, no double CMGL.
  mFlush();
  mLine("AT+CMGL=\"REC UNREAD\"");

  String resp;
  unsigned long t0 = millis();

  // Read for up to ~200 ms
  while (millis() - t0 < 100UL) {
    bool gotSomething = false;
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      resp += c;
      gotSomething = true;
    }
    if (!gotSomething) {
      delay(5);
    }
  }

  Serial.print("[SMS] raw CMGL resp (len=");
  Serial.print(resp.length());
  Serial.println("):");
  Serial.println("----- BEGIN CMGL -----");
  Serial.println(resp);
  Serial.println("----- END CMGL -----");

  if (!resp.length()) {
    Serial.println("[SMS] No response at all from CMGL – reconfiguring SMS...");
    // Try to gently re-assert SMS config
    mLine("AT+CMGF=1");
    waitOK(1000);
    mLine("AT+CSCS=\"GSM\"");
    waitOK(1000);
    setCPMSToSM();
    return;
  }

  if (resp.indexOf("ERROR") >= 0) {
    Serial.println("[SMS] CMGL ERROR – reconfiguring SMS...");
    mLine("AT+CMGF=1");
    waitOK(1000);
    mLine("AT+CSCS=\"GSM\"");
    waitOK(1000);
    setCPMSToSM();
    return;
  }

  // If there's no +CMGL:, it's just "OK" → no unread
  if (resp.indexOf("+CMGL:") < 0) {
    Serial.println("[SMS] No +CMGL: found → no unread messages.");
    return;
  }

  int pos = 0;
  while (true) {
    int h = resp.indexOf("+CMGL:", pos);
    if (h < 0) break;
    int nl = resp.indexOf('\n', h);
    if (nl < 0) nl = resp.length();
    String header = resp.substring(h, nl);

    // index
    int colon = header.indexOf(':');
    int comma = header.indexOf(',', colon + 1);
    int idx = -1;
    if (comma > colon) {
      String idxs = header.substring(colon + 1, comma);
      idxs.trim();
      idx = idxs.toInt();
    }

    // sender (third quoted field: "status","sender",...)
    String sender = "";
    int q = header.indexOf('"');
    for (int k = 0; k < 2 && q >= 0; k++) {
      q = header.indexOf('"', q + 1);
    }
    if (q >= 0) {
      int q2 = header.indexOf('"', q + 1);
      if (q2 > q) sender = header.substring(q + 1, q2);
    }

    int bodyStart = nl + 1;
    int bodyEnd = resp.indexOf('\n', bodyStart);
    if (bodyEnd < 0) bodyEnd = resp.length();
    String body = resp.substring(bodyStart, bodyEnd);
    body.trim();
    body.replace("\r", "");
    body.replace("\n", "");

    Serial.print("[SMS] parsed entry idx=");
    Serial.print(idx);
    Serial.print(" sender=[");
    Serial.print(sender);
    Serial.print("] body=[");
    Serial.print(body);
    Serial.println("]");

    if (!body.length()) {
      Serial.println("[SMS] Empty body, skipping this entry.");
      pos = bodyEnd + 1;
      continue;
    }

    pos = bodyEnd + 1;

    // Run command
    String result = handleCommand(body, /*fromSMS*/true, sender);

    // Delete SMS
    if (idx >= 0) {
      Serial.print("[SMS] Deleting SMS idx=");
      Serial.println(idx);
      mLine(String("AT+CMGD=") + idx);
      waitOK(1000);
    }

    // Optional reply
    if (sender.length() && result.length()) {
      String msg = result;
      if (msg.length() > 150) msg = msg.substring(0, 150);
      bool ok = smsSendTo(sender, msg);
      Serial.print("[SMS] Reply send result: ");
      Serial.println(ok ? "OK" : "FAIL");
    }
  }
}

// ---------- Console I/O ----------
String readUSBLine(){
  static String buf;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (buf.length()) {
        String out = buf;
        buf = "";
        out.trim();
        return out;
      }
      // ignore empty newlines
    } else {
      buf += c;
    }
  }
  return String();
}

String readBTLine(){
  static String buf;

  while (BT.available()) {
    char c = BT.read();
    if (c == '\r' || c == '\n') {
      if (buf.length()) {
        String out = buf;
        buf = "";
        out.trim();
        Serial.print("BT RAW: ");
        Serial.println(out);
        return out;
      }
    } else {
      buf += c;
    }
  }
  return String();
}

bool parseQuoted(const String &line, String &out){
  int q1 = line.indexOf('"'); if (q1<0) return false;
  int q2 = line.lastIndexOf('"'); if (q2<=q1) return false;
  out = line.substring(q1+1, q2);
  return true;
}

void driveForwardMeters(float mTarget){
  if (V_FWD <= 0.0f){
    Serial.println("V_FWD not set.");
    return;
  }
  if (mTarget <= 0.0f) return;

  float remaining = mTarget;

  while (remaining > 0.0f){
    // Check front sensor BEFORE moving
    long front = ultraCM(US_TRIG_F, US_ECHO_F);
    if (front > 0 && front < (long)STOP_CM){
      Serial.println("Obstacle ahead, attempting avoidance...");
      if (!avoidObstacleAroundFront(&remaining)){
        Serial.println("Avoidance failed, aborting F command.");
        break;
      }
      continue; // after avoidance, try again toward remaining
    }

    // Step distance
    float step = FWD_STEP_M;
    if (step > remaining) step = remaining;

    // Move one step with normal front safety
    driveForwardMetersChunk(step, true);

    remaining -= step;
    if (remaining < 0) remaining = 0;
  }
}

// ---------- High-level forward with avoidance ----------
void driveForwardMetersChunk(float m, bool frontSafety){
  if (V_FWD <= 0.0f){
    Serial.println("V_FWD not set.");
    return;
  }
  if (m <= 0) return;

  unsigned long need = (unsigned long)(1000.0f * m / V_FWD);

  // Use trimmed forward motion instead of raw allForward()
  forwardTrimmedForMs(need, frontSafety);
}

// ---------- Backward (with back safety) ----------
void driveBackwardMeters(float m){
  // All wheels backward
  digitalWrite(RR_IN1,HIGH); digitalWrite(RR_IN2,LOW);
  digitalWrite(FR_IN1,HIGH); digitalWrite(FR_IN2,LOW);
  digitalWrite(LR_IN1,LOW);  digitalWrite(LR_IN2,HIGH);
  digitalWrite(FL_IN1,HIGH); digitalWrite(FL_IN2,LOW);

  if (V_FWD <= 0.0f){
    Serial.println("V_FWD not set.");
    allCoast();
    return;
  }

  if (m <= 0) { allCoast(); return; }

  unsigned long need = (unsigned long)(1000.0f * m / V_FWD);
  unsigned long t0 = millis();
  while (millis() - t0 < need){
    long cm = ultraCM(US_TRIG_B, US_ECHO_B);
    if (cm > 0 && cm < (long)STOP_CM){
      Serial.println("Safety stop (back).");
      break;
    }
    delay(10);
  }
  allCoast();
  delay(RAMP_MS);
}

// ---------- Obstacle avoidance routine ----------
bool avoidObstacleAroundFront(float *remainingPtr){
  if (!remainingPtr) return false;

  // 1) Back up a bit to gain clearance
  Serial.println("Avoid: backing up...");
  driveBackwardMeters(AVOID_BACKUP_M);

  // 2) Look left and right
  long left  = ultraCM(US_TRIG_L, US_ECHO_L);
  long right = ultraCM(US_TRIG_R, US_ECHO_R);

  bool leftValid  = (left  >= 0);
  bool rightValid = (right >= 0);

  bool goLeft;
  if (leftValid && rightValid){
    goLeft = (left > right); // more space = more distance
  } else if (leftValid){
    goLeft = true;
  } else if (rightValid){
    goLeft = false;
  } else {
    Serial.println("Avoid: no valid side readings, giving up.");
    return false;
  }

  if (goLeft) Serial.println("Avoid: going around on LEFT side.");
  else        Serial.println("Avoid: going around on RIGHT side.");

  // 3) First turn 90° toward chosen side
  if (goLeft) turnLeftDeg(90.0f);
  else        turnRightDeg(90.0f);

  // 4) Move sideways AVOID_SIDESTEP_M
  driveForwardMetersChunk(AVOID_SIDESTEP_M, false);  // no front safety (lateral)

  // 5) Turn back to original heading (facing "forward" again)
  if (goLeft) turnRightDeg(90.0f);
  else        turnLeftDeg(90.0f);

  // 6) Move forward along the obstacle for AVOID_FORWARD_M
  driveForwardMetersChunk(AVOID_FORWARD_M, true);    // front safety in case of another obstacle

  // 7) Re-center approximately: step sideways back to original lane
  if (goLeft){
    // we are left of original line; move right to recenter
    turnRightDeg(90.0f);
    driveForwardMetersChunk(AVOID_SIDESTEP_M, false);
    turnLeftDeg(90.0f);
  } else {
    // we are right of original line; move left to recenter
    turnLeftDeg(90.0f);
    driveForwardMetersChunk(AVOID_SIDESTEP_M, false);
    turnRightDeg(90.0f);
  }

  // We roughly advanced AVOID_FORWARD_M in the original direction
  *remainingPtr -= AVOID_FORWARD_M;
  if (*remainingPtr < 0) *remainingPtr = 0;

  Serial.print("Avoid: finished, remaining forward target ≈ ");
  Serial.print(*remainingPtr, 2);
  Serial.println(" m");

  return true;
}

// ---------- CALV: 1-second run ----------
void runCalV(){
  Serial.println("CALV: running forward for 1.0 s (trimmed)...");
  forwardTrimmedForMs(1000UL, /*frontSafety*/false);
  Serial.println("CALV: done. Measure distance and send MOVED <feet>.");
}

// ---------- Turning ----------
void turnLeftDeg(float deg){
  if (W_SPIN <= 0.0f){ Serial.println("W_SPIN not set."); return; }
  long usec = (long)(1000000.0 * (deg / W_SPIN)) - T0_US;
  if (usec < 0) usec = 0;

  unsigned long t0 = micros();
  spinLeft();
  while ((long)(micros() - t0) < usec) { }
  unsigned long actual_us = micros() - t0;

  allCoast();
  delay(120);

  Serial.print("TURN LEFT actual ms = ");
  Serial.println(actual_us / 1000.0);
}

void turnRightDeg(float deg){
  if (W_SPIN <= 0.0f){ Serial.println("W_SPIN not set."); return; }
  long usec = (long)(1000000.0 * (deg / W_SPIN)) - T0_US;
  if (usec < 0) usec = 0;

  unsigned long t0 = micros();
  spinRight();
  while ((long)(micros() - t0) < usec) { }
  unsigned long actual_us = micros() - t0;

  allCoast();
  delay(120);

  Serial.print("TURN RIGHT actual ms = ");
  Serial.println(actual_us / 1000.0);
}

// ---------- Pretty printers ----------
String usAllString(){
  long f = ultraCM(US_TRIG_F, US_ECHO_F);
  long r = ultraCM(US_TRIG_R, US_ECHO_R);
  long l = ultraCM(US_TRIG_L, US_ECHO_L);
  long b = ultraCM(US_TRIG_B, US_ECHO_B);
  String s = "US(cm) F=" + String(f) + " R=" + String(r) + " L=" + String(l) + " B=" + String(b);
  return s;
}
void printUSAll(){ Serial.println(usAllString()); }

void printHelp(){
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  US / USF / USR / USL / USB");
  Serial.println("  F <feet> / B <feet>");
  Serial.println("  L <deg> / R <deg>");
  Serial.println("  SETVFWD <m/s>   | SETW <deg/s>  | SETT0 <us>");
  Serial.println("  CALV / MOVED <feet>");
  Serial.println("  READALL         | STOPCM <cm>");
  Serial.println("  SMSNUM <num>    | SAVENUM | READNUM | CLRNUM");
  Serial.println("  SMS \"text\"      | SMSUS");
  Serial.println("  JOYON / JOYOFF  | JF/JB/JL/JR/JS");
  Serial.println("  HELP / ?");
}

String runDiagnet() {
  auto doCmd = [&](const char *cmd, uint16_t ms) -> String {
    String buf = modemRaw(cmd, ms);
    return buf;
  };

  String out = "Running DIAGNET...\n";
  doCmd("AT",        500);
  doCmd("AT+CPIN?",  1000);
  doCmd("AT+CREG?",  1000);
  doCmd("AT+CEREG?", 1000);
  doCmd("AT+CSQ",    1000);
  doCmd("AT+CMGF?",  1000);
  doCmd("AT+CPMS?",  1500);
  out += "Done.";
  return out;
}

String handleCommand(const String &lineIn, bool fromSMS, const String &smsSender){
  String line = lineIn;
  line.trim();

  Serial.print("CMD src=");
  Serial.print(fromSMS ? "SMS" : "USB/BT");
  Serial.print(" line=[");
  Serial.print(line);
  Serial.println("]");

  // ---------- Command parsing ----------
  String op, arg;
  String lineU = line; 
  lineU.toUpperCase();

  // Decide if this is a move command (F/B/L/R followed by space or digit or '-')
  bool isMove = false;
  if (lineU.length() > 0) {
    char c0 = lineU[0];
    char c1 = (lineU.length() > 1) ? lineU[1] : '\0';
    if (c0=='F' || c0=='B' || c0=='L' || c0=='R') {
      if (c1=='\0' || c1==' ' || (c1>='0' && c1<='9') || c1=='-') {
        isMove = true;
      }
    }
  }

  if (isMove) {
    // Case 1: there is a space (F 12, L 90)
    int sp = line.indexOf(' ');
    if (sp >= 0) {
      op  = line.substring(0, sp);
      arg = line.substring(sp + 1);
    }
    // Case 2: no space, like F12 or R45
    else {
      op  = line.substring(0, 1);    // "F", "B", "L", "R"
      arg = line.substring(1);       // rest is number (maybe blank)
    }
  } else {
    // Normal split for non-move commands:
    int sp = line.indexOf(' ');
    if (sp >= 0) {
      op  = line.substring(0, sp);
      arg = line.substring(sp + 1);
    } else {
      op  = line;
      arg = "";
    }
  }

  op.trim();
  arg.trim();
  String opU = op; opU.toUpperCase();

  // ---------- Common return helper ----------
  auto ret = [&](const String &s)->String{
    if (!fromSMS) Serial.println(s);
    return s;
  };

  // ---------- Commands ----------
  if (opU == "DIAGNET") {
    String s = runDiagnet();
    return ret(s);
  }

  if (opU == "US"){
    String s = usAllString();
    if (!fromSMS) Serial.println(s);
    return fromSMS ? s : String();
  }

  if (opU=="USF" || opU=="USR" || opU=="USL" || opU=="USB"){
    int t=-1,e=-1;
    if (opU=="USF"){ t=US_TRIG_F; e=US_ECHO_F; }
    if (opU=="USR"){ t=US_TRIG_R; e=US_ECHO_R; }
    if (opU=="USL"){ t=US_TRIG_L; e=US_ECHO_L; }
    if (opU=="USB"){ t=US_TRIG_B; e=US_ECHO_B; }
    long cm = ultraCM(t,e);
    return ret(opU + " = " + String(cm) + " cm");
  }

  // ---- Joystick mode control (from BT or USB) ----
if (opU=="JOYON") {
  gJoystickMode = true;   // joystick-only mode
  joyStop();              // make sure we start from a stop
  return ret("Joystick mode ENABLED (SMS polling paused)");
}

if (opU=="JOYOFF") {
  gJoystickMode = false;  // text/SMS-only mode
  joyStop();              // cut any motion the joystick was doing
  return ret("Joystick mode DISABLED (SMS polling resumed)");
}


 // ---- Joystick live commands (JF/JB/JL/JR/JS) ----
if (opU=="JF" || opU=="JB" || opU=="JL" || opU=="JR" || opU=="JS") {
  // If joystick mode is OFF, just ignore joystick packets completely
  if (!gJoystickMode) {
    return String();  // silent ignore
  }

  // Joystick mode is ON → these packets directly drive the robot
  lastJoystickCmdMs = millis();

  if (opU=="JS") {
    joyStop();
    return ret("JOY: STOP");
  }
  if (opU=="JF") {
    joyForward();
    return ret("JOY: FWD");
  }
  if (opU=="JB") {
    joyBackward();
    return ret("JOY: BACK");
  }
  if (opU=="JL") {
    joyTurnLeft();
    return ret("JOY: LEFT");
  }
  if (opU=="JR") {
    joyTurnRight();
    return ret("JOY: RIGHT");
  }
}


  if (opU=="F"){
    float ft = arg.toFloat();
    if (ft<=0) return ret("Usage: F <feet>");
    float m = ft * 0.3048f;
    driveForwardMeters(m);
    return ret("F done (with avoidance)");
  }

  if (opU=="B"){
    float ft = arg.toFloat();
    if (ft<=0) return ret("Usage: B <feet>");
    driveBackwardMeters(ft*0.3048f);
    return ret("B done");
  }

  if (opU=="L"){
    float d = arg.toFloat();
    if (d<=0) return ret("Usage: L <deg>");
    turnLeftDeg(d);
    return ret("L done");
  }

  if (opU=="R"){
    float d = arg.toFloat();
    if (d<=0) return ret("Usage: R <deg>");
    turnRightDeg(d);
    return ret("R done");
  }

  // ---- V_FWD: manual set + calibration ----
  if (opU=="SETVFWD"){
    float v = arg.toFloat();
    if (v<=0) return ret("Usage: SETVFWD <m/s>");
    V_FWD = v;
    eeSaveV(V_FWD);
    return ret("V_FWD=" + String(V_FWD,3) + " m/s (saved)");
  }

  if (opU=="CALV"){
    gCalvPending = true;
    gCalvTime_s  = 1.0f;
    runCalV();
    return ret("CALV: Drove 1.0 s. Measure distance and send MOVED <feet>.");
  }

  if (opU=="MOVED"){
    if (!gCalvPending) return ret("No calibration pending. Send CALV first.");
    float feet = arg.toFloat();
    if (feet <= 0) return ret("Usage: MOVED <feet>");
    float m = feet * 0.3048f;
    float v = m / gCalvTime_s;
    if (v < 0.05f || v > 3.0f){
      gCalvPending = false;
      return ret("Calibration value out of range. Try CALV again with clear floor.");
    }
    V_FWD = v;
    eeSaveV(V_FWD);
    gCalvPending = false;
    String s = "V_FWD set to " + String(V_FWD,3) + " m/s (saved).";
    return ret(s);
  }

  // ---- Spin params ----
  if (opU=="SETW"){
    float w = arg.toFloat();
    if (w<=0) return ret("Usage: SETW <deg/s>");
    W_SPIN = clampW(w); 
    eeSaveW(W_SPIN);
    return ret("W_SPIN=" + String(W_SPIN,1) + " deg/s (saved)");
  }

  if (opU=="SETT0"){
    long t = arg.toInt();
    T0_US = clampT0(t); 
    eeSaveT0(T0_US);
    return ret("T0_US=" + String(T0_US) + " us (saved)");
  }

  if (opU=="READALL"){
    String s = "V_FWD=" + String(V_FWD,3) + " m/s, W_SPIN=" + String(W_SPIN,1) +
               " deg/s, T0_US=" + String(T0_US) + " us";
    return ret(s);
  }

  if (opU=="STOPCM"){
    float s = arg.toFloat();
    if (s<5) return ret("STOPCM must be >=5");
    STOP_CM = s;
    return ret("STOP_CM=" + String(STOP_CM));
  }

  // ---- Phone / SMS helpers ----
  if (opU=="SMSNUM"){
    if (!arg.length()){
      return ret("Active: " + (gPhone.length()? gPhone : String("<none>")));
    } else {
      String cleaned = normalizePhone(arg);
      if (!cleaned.length()) return ret("Give number like +15551234567");
      gPhone = cleaned;
      return ret("Active set: " + gPhone);
    }
  }

  if (opU=="SAVENUM"){
    if (!gPhone.length()) return ret("No active number.");
    eeWritePhone(gPhone);
    return ret("Saved: " + gPhone);
  }

  if (opU=="READNUM"){
    String s;
    if (eeReadPhone(s)) return ret("EEPROM: " + s);
    else return ret("No phone stored.");
  }

  if (opU=="CLRNUM"){
    eeClearPhone();
    return ret("Phone cleared.");
  }

  if (opU=="SMS"){
    if (!gPhone.length()) return ret("No destination set (SMSNUM + SAVENUM).");
    String msg;
    if (!parseQuoted(lineIn, msg)) return ret("Usage: SMS \"text\"");
    bool ok = smsSendTo(gPhone, msg);
    return ret(ok? "SMS sent." : "SMS failed.");
  }

  if (opU=="SMSUS"){
    if (!gPhone.length()) return ret("No destination set.");
    String msg = usAllString();
    bool ok = smsSendTo(gPhone, msg);
    return ret(ok? "SMS sent." : "SMS failed.");
  }

  if (opU=="HELP" || opU=="?"){
    if (!fromSMS) printHelp();
    return fromSMS ? String("HELP over SMS is truncated; use USB for full menu.") : String();
  }

  // ---- Raw modem AT passthrough ----
  if (opU == "MODEM" || opU == "ATRAW") {
    if (!arg.length()) {
      return ret("Usage: MODEM <AT command>");
    }
    // Send whatever is in arg as an AT command to Serial1
    String resp = modemRaw(arg, 1500);
    // For USB we already printed it; for SMS we could echo a short note
    return fromSMS ? String("MODEM cmd sent.") : String();
  }

  return ret("Unknown. Try HELP.");
}

// ---------- Setup ----------
void setup(){
  pinMode(RR_IN1,OUTPUT); pinMode(RR_IN2,OUTPUT);
  pinMode(LR_IN1,OUTPUT); pinMode(LR_IN2,OUTPUT);
  pinMode(FR_IN1,OUTPUT); pinMode(FR_IN2,OUTPUT);
  pinMode(FL_IN1,OUTPUT); pinMode(FL_IN2,OUTPUT);
  allCoast();

  pinMode(US_TRIG_F,OUTPUT); pinMode(US_ECHO_F,INPUT); digitalWrite(US_TRIG_F,LOW);
  pinMode(US_TRIG_B,OUTPUT); pinMode(US_ECHO_B,INPUT); digitalWrite(US_TRIG_B,LOW);
  pinMode(US_TRIG_L,OUTPUT); pinMode(US_ECHO_L,INPUT); digitalWrite(US_TRIG_L,LOW);
  pinMode(US_TRIG_R,OUTPUT); pinMode(US_ECHO_R,INPUT); digitalWrite(US_TRIG_R,LOW);

  Serial.begin(USB_BAUD);
  delay(200);

  Serial1.begin(MODEM_BAUD);

  // Bluetooth serial for HC-05 (slave)
  BT.begin(9600);

  // Let the SIM fully boot (~6s)
  delay(6000);

  while (Serial && Serial.read() >= 0) {}

  float w; if (eeLoadW(w))  W_SPIN = w;
  long  t; if (eeLoadT0(t)) T0_US  = t;
  float v; if (eeLoadV(v))  V_FWD  = v;

  String stored;
  if (eeReadPhone(stored)) {
    gPhone = stored;
    Serial.println("Phone: " + gPhone);
  } else {
    Serial.println("No phone in EEPROM (use SMSNUM + SAVENUM).");
  }

  Serial.println("Init modem…");

  // record when we FIRST tried init (for retry logic later)
  lastModemInitAttemptMs = millis();

  gModemReady = modemInit();

  if (gModemReady) {
    Serial.println("Modem OK.");

    // Make absolutely sure CPMS is SM before deleting
    if (setCPMSToSM()) {
      Serial.println("Clearing old SMS in SM (1,4)...");
      mLine("AT+CMGD=1,4");   // delete messages 1..4 in SM
      waitOK(2000);
    } else {
      Serial.println("WARNING: Could not set CPMS to SM in setup(), skipping CMGD");
    }

    // Optional: one priming poll AFTER everything is configured
    pollSMSInbox();

  } else {
    Serial.println("Modem init failed (still running USB control, will retry in loop).");
  }

  printHelp();
}

// ---------- Loop ----------
void loop(){
  // ---- USB console commands ----
  String usb = readUSBLine();
  if (usb.length()){
    handleCommand(usb, /*fromSMS*/false, String());
  }

  // ---- Bluetooth commands (HC-05 on Serial2) ----
  String bt = readBTLine();
  if (bt.length()){
    handleCommand(bt, /*fromSMS*/false, String());
  }

  // ---- Ultrasonic streaming over USB ----
  if (gUSStream && (millis() - lastUSStreamMs >= US_STREAM_PERIOD_MS)) {
    lastUSStreamMs = millis();
    Serial.println(usAllString());
  }

  // ---- Joystick safety timeout ----
  if (gJoystickMode) {
    if (millis() - lastJoystickCmdMs > JOY_TIMEOUT_MS) {
      // No fresh J* packets recently → coast to a stop
      joyStop();
      // We leave gJoystickMode = true, so next J* still counts as joystick mode
    }
  }

  // ---- SMS polling + modem re-init ----
  if (gModemReady && !gJoystickMode) {
    // Only poll inbox if modem is known-good AND joystick isn't actively driving
    if (millis() - lastSMSPollMs >= SMS_POLL_EVERY_MS) {
      lastSMSPollMs = millis();
      pollSMSInbox();
    }
  } else if (!gModemReady) {
    // Modem not ready → periodically retry modemInit()
    if (millis() - lastModemInitAttemptMs >= MODEM_RETRY_MS) {
      lastModemInitAttemptMs = millis();
      Serial.println("[MODEM] Not ready, retrying modemInit()...");

      gModemReady = modemInit();

      if (gModemReady) {
        Serial.println("[MODEM] Re-init OK.");

        // Re-assert SMS storage after re-init
        if (setCPMSToSM()) {
          Serial.println("[MODEM] CPMS SM OK after re-init.");
        } else {
          Serial.println("[MODEM] CPMS SM FAILED after re-init.");
        }
      } else {
        Serial.println("[MODEM] Re-init FAILED.");
      }
    }
  }
}