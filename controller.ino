#include <AltSoftSerial.h>
#include <math.h>

// ---------------- Joystick + button pins ----------------
const int JOY_X_PIN = A1;   // left/right
const int JOY_Y_PIN = A0;   // forward/back
const int BTN_PIN   = 2;    // joystick button (active LOW, uses INPUT_PULLUP)

// ---------------- Bluetooth (HC-05 master) ----------------
// AltSoftSerial on UNO:  RX = 8, TX = 9  (fixed by library)
AltSoftSerial BT;

// ---------------- Button toggle / debounce ----------------
bool joystickEnabled = false;   // starts OFF

bool lastButtonReading = HIGH;  // raw reading last loop
bool buttonState       = HIGH;  // debounced state
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // ms

// ---------------- Joystick mapping ----------------
const int JOY_CENTER      = 512;  // nominal center
const int JOY_DEADBAND    = 80;   // ignore tiny wiggles
const int JOY_MAX_DELTA   = 512;  // rough max distance from center

// How often we send commands when joystick is enabled
const unsigned long SEND_PERIOD_MS = 50;   // 20 Hz
unsigned long lastSendMs = 0;

// Max step sizes (not used for J* mode, but kept if we ever go back)
const float MAX_STEP_FEET = 0.50f;   // full forward/back per packet
const float MAX_TURN_DEG  = 45.0f;   // full turn per packet

// ---------------- Helper: map joystick -> -1..1 with deadband ----------------
float axisToUnit(int raw) {
  int delta = raw - JOY_CENTER;

  // Deadband
  if (abs(delta) < JOY_DEADBAND) return 0.0f;

  // Shrink range by deadband
  int sign = (delta > 0) ? 1 : -1;
  int magRaw = abs(delta) - JOY_DEADBAND;

  float u = (float)magRaw / (float)(JOY_MAX_DELTA - JOY_DEADBAND);
  if (u > 1.0f) u = 1.0f;
  return sign * u;  // in [-1, 1]
}

// ---------------- Send one joystick-based command ----------------
void sendJoystickCommand() {
  int rawX = analogRead(JOY_X_PIN);
  int rawY = analogRead(JOY_Y_PIN);

  float ux = axisToUnit(rawX);    // left/right
  float uy = axisToUnit(rawY);   // forward/back (invert so up = +)

  // Tiny deadband on top of axisToUnit, just to be safe
  if (fabs(ux) < 0.05f) ux = 0.0f;
  if (fabs(uy) < 0.05f) uy = 0.0f;

  // If stick basically centered → tell rover to STOP
  if (ux == 0.0f && uy == 0.0f) {
    BT.println("JS");
    Serial.println("CMD -> ROVER: JS (stop)");
    return;
  }

  // Decide: translation (F/B) vs rotation (L/R)
  if (fabs(uy) >= fabs(ux)) {
    // -------- Forward / Backward --------
    if (uy > 0) {
      BT.println("JF");
      Serial.println("CMD -> ROVER: JF");
    } else {
      BT.println("JB");
      Serial.println("CMD -> ROVER: JB");
    }
  } else {
    // -------- Left / Right turn --------
    if (ux < 0) {
      BT.println("JL");
      Serial.println("CMD -> ROVER: JL");
    } else {
      BT.println("JR");
      Serial.println("CMD -> ROVER: JR");
    }
  }
}

// ---------------- Button debounce + toggle ----------------
void handleButtonToggle() {
  int reading = digitalRead(BTN_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;

      // LOW = pressed (since INPUT_PULLUP)
      if (buttonState == LOW) {
        joystickEnabled = !joystickEnabled;
        Serial.print("Joystick mode: ");
        Serial.println(joystickEnabled ? "ENABLED" : "DISABLED");

        if (joystickEnabled) {
          BT.println("JOYON");
          Serial.println("Sent JOYON to rover");
        } else {
          BT.println("JOYOFF");
          BT.println("JS");   // hard STOP when leaving joystick mode
          Serial.println("Sent JOYOFF + JS to rover");
        }
      }
    }
  }

  lastButtonReading = reading;
}

// ---------------- Setup ----------------
void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  while (!Serial) { ; }

  Serial.println("Joystick BT controller starting…");

  // AltSoftSerial for HC-05 master
  BT.begin(9600);   // must match HC-05 data baud

  Serial.println("BT on AltSoftSerial (TX=9, RX=8) @ 9600");
  Serial.println("Press joystick button to ENABLE/DISABLE control.");
}

// ---------------- Main loop ----------------
void loop() {
  // Handle button toggling
  handleButtonToggle();

  // Periodically send joystick commands if enabled
  unsigned long now = millis();
  if (joystickEnabled && (now - lastSendMs >= SEND_PERIOD_MS)) {
    lastSendMs = now;
    sendJoystickCommand();
  }

  // (Optional) bridge incoming BT → Serial for debug
  if (BT.available()) {
    char c = BT.read();
    Serial.write(c);
  }

  // (Optional) bridge Serial → BT for manual typing
  if (Serial.available()) {
    char c = Serial.read();
    BT.write(c);
  }
}