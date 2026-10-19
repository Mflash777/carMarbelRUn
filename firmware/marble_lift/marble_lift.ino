/*
  Marble lift controller
  ----------------------
  Drives the N20 gear-motor that turns the Archimedes screw.

  - Short press:  start / stop (with a soft ramp so marbles don't get flung)
  - Long press:   un-jam - briefly reverses the screw, then carries on
  - Potentiometer sets the speed (MIN_SPEED..MAX_SPEED %)
  - Auto-off after AUTO_OFF_MIN minutes so it doesn't run forever
  - Serial (115200): r = run, s = stop, u = un-jam, + / - = speed, ? = status

  Hardware: Seeed XIAO ESP32-C3 + DRV8833 module + N20 6V gear-motor.
  See ../../README.md for wiring.
*/
#include "config.h"

// ---------- PWM helpers (works on ESP32 Arduino core 2.x and 3.x) ----------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void pwmAttach(int pin, int ch) { (void)ch; ledcAttach(pin, PWM_FREQ_HZ, PWM_BITS); }
static void pwmWrite(int pin, int ch, uint32_t duty) { (void)ch; ledcWrite(pin, duty); }
#else
static void pwmAttach(int pin, int ch) { ledcSetup(ch, PWM_FREQ_HZ, PWM_BITS); ledcAttachPin(pin, ch); }
static void pwmWrite(int pin, int ch, uint32_t duty) { (void)pin; ledcWrite(ch, duty); }
#endif

static const uint32_t PWM_MAX = (1u << PWM_BITS) - 1;

enum State { STOPPED, RUNNING, UNJAM };
static State state = STOPPED;

static int  targetPct  = DEFAULT_SPEED;   // what the pot / serial asks for
static int  currentPct = 0;               // signed: + forward, - reverse
static int  serialTrim = 0;               // +/- from serial commands
static unsigned long runStartedMs = 0;
static unsigned long unjamUntilMs = 0;
static unsigned long lastRampMs   = 0;

// ---------- Motor ----------
static void motorWrite(int pct) {
  // pct in -100..100. Forward: AIN1 = PWM, AIN2 = LOW (fast decay).
  if (MOTOR_REVERSED) pct = -pct;
  uint32_t duty = (uint32_t)(abs(pct) * PWM_MAX / 100);
  if (pct > 0)      { pwmWrite(PIN_AIN1, 0, duty); pwmWrite(PIN_AIN2, 1, 0); }
  else if (pct < 0) { pwmWrite(PIN_AIN1, 0, 0);    pwmWrite(PIN_AIN2, 1, duty); }
  else              { pwmWrite(PIN_AIN1, 0, 0);    pwmWrite(PIN_AIN2, 1, 0); }
  digitalWrite(PIN_NSLEEP, pct != 0 ? HIGH : LOW);
}

static int readSpeedPct() {
  int pct = DEFAULT_SPEED;
#if USE_POT
  long raw = 0;
  for (int i = 0; i < 8; i++) raw += analogRead(PIN_POT);
  raw /= 8;                                            // 0..4095
  pct = MIN_SPEED + (int)((raw * (MAX_SPEED - MIN_SPEED)) / 4095);
#endif
  pct += serialTrim;
  return constrain(pct, MIN_SPEED, MAX_SPEED);
}

// ---------- State changes ----------
static void startRun() {
  if (state != RUNNING) Serial.println(F("RUN"));
  state = RUNNING;
  runStartedMs = millis();
}

static void stopRun() {
  if (state != STOPPED) Serial.println(F("STOP"));
  state = STOPPED;
}

static void startUnjam() {
  Serial.println(F("UNJAM"));
  state = UNJAM;
  currentPct = 0;                                      // stop dead, then reverse
  motorWrite(0);
  delay(80);
  unjamUntilMs = millis() + UNJAM_REVERSE_MS;
}

// ---------- Button (debounced, short / long press) ----------
static void pollButton() {
  static bool lastLevel = HIGH, stable = HIGH, longFired = false;
  static unsigned long changedMs = 0, pressedMs = 0;
  bool level = digitalRead(PIN_BUTTON);
  unsigned long now = millis();
  if (level != lastLevel) { changedMs = now; lastLevel = level; }
  if (now - changedMs < 25) return;                    // debounce
  if (level != stable) {
    stable = level;
    if (stable == LOW) { pressedMs = now; longFired = false; }
    else if (!longFired) {                             // released before long-press
      if (state == RUNNING) stopRun(); else startRun();
    }
  }
  if (stable == LOW && !longFired && now - pressedMs >= LONG_PRESS_MS) {
    longFired = true;
    startUnjam();
  }
}

// ---------- Serial ----------
static void pollSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'r': startRun(); break;
      case 's': stopRun(); break;
      case 'u': startUnjam(); break;
      case '+': serialTrim = min(serialTrim + 5, 70); break;
      case '-': serialTrim = max(serialTrim - 5, -70); break;
      case '?':
        Serial.printf("state=%s target=%d%% now=%d%%\n",
                      state == RUNNING ? "running" : state == UNJAM ? "unjam" : "stopped",
                      targetPct, currentPct);
        break;
      default: break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_NSLEEP, OUTPUT);
  digitalWrite(PIN_NSLEEP, LOW);
  pwmAttach(PIN_AIN1, 0);
  pwmAttach(PIN_AIN2, 1);
#if USE_POT
  analogReadResolution(12);
#endif
  motorWrite(0);
  Serial.println(F("Marble lift ready. r=run s=stop u=unjam +/- speed ?=status"));
  if (START_RUNNING) startRun();
}

void loop() {
  pollButton();
  pollSerial();
  unsigned long now = millis();

  // where we want the motor to be
  int want = 0;
  if (state == RUNNING) {
    targetPct = readSpeedPct();
    want = targetPct;
    if (AUTO_OFF_MIN > 0 && now - runStartedMs > (unsigned long)AUTO_OFF_MIN * 60000UL) {
      Serial.println(F("Auto-off"));
      stopRun();
      want = 0;
    }
  } else if (state == UNJAM) {
    if ((long)(now - unjamUntilMs) < 0) {
      currentPct = -MIN_SPEED - 20;                    // reverse immediately, no ramp
      motorWrite(currentPct);
      return;
    }
    currentPct = 0;
    motorWrite(0);
    delay(80);
    startRun();                                        // resume forward with a ramp
    return;
  }

  // soft ramp toward `want`
  if (now - lastRampMs >= RAMP_MS_PER_PCT) {
    lastRampMs = now;
    if (currentPct < want) {
      // jump straight past the stall zone when starting
      currentPct = (currentPct < MIN_SPEED && want >= MIN_SPEED) ? MIN_SPEED : currentPct + 1;
    } else if (currentPct > want) {
      currentPct = (currentPct <= MIN_SPEED && want == 0) ? 0 : currentPct - 1;
    }
    motorWrite(currentPct);
  }
}
