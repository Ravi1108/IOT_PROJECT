
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>

// =====================================================================
// 1. CONFIGURATION  (every tunable lives here)
// =====================================================================

// ---- Pins ----
constexpr uint8_t PIN_IR_SENSOR  = 27;
constexpr uint8_t PIN_BUTTON     = 4;    // to GND, internal pull-up
constexpr uint8_t PIN_LED_GREEN  = 25;
constexpr uint8_t PIN_LED_YELLOW = 26;
constexpr uint8_t PIN_LED_RED    = 33;
constexpr uint8_t PIN_BUZZER     = 32;   // piezo: GPIO -> 100-330 ohm -> piezo -> GND
constexpr uint8_t PIN_I2C_SDA    = 21;
constexpr uint8_t PIN_I2C_SCL    = 22;

// ---- Polarity (VERIFY with a Serial print before finalising) ----
constexpr uint8_t IR_ACTIVE_LEVEL     = LOW;   // most IR modules: LOW = object detected
constexpr uint8_t BUTTON_ACTIVE_LEVEL = LOW;   // wired to GND

// ---- Buzzer tones (passive piezo needs a frequency, not just ON/OFF) ----
constexpr uint16_t TONE_ALARM_HZ = 2000;
constexpr uint16_t TONE_FAULT_HZ = 3000;       // different pitch so fault sounds distinct from alarm
constexpr uint8_t  BUZZER_LEDC_CHANNEL = 0;    // used only on core 2.x

// ---- Display ----
constexpr uint8_t  OLED_I2C_ADDR = 0x3C;       // some boards use 0x3D
constexpr uint8_t  OLED_WIDTH    = 128;
constexpr uint8_t  OLED_HEIGHT   = 64;
constexpr uint32_t DISPLAY_MIN_INTERVAL_MS = 100;

// ---- Intrusion detection ----
constexpr uint32_t INTRUSION_CONFIRM_MS     = 500;   // uninterrupted detection needed for alarm
constexpr uint32_t SENSOR_DEBOUNCE_MS       = 40;    // raw level must be stable this long
constexpr uint32_t SENSOR_CLEAR_REQUIRED_MS = 1000;  // clear time needed before arming / accepting intrusions
constexpr uint32_t ARMING_DURATION_MS       = 3000;

// ---- Button ----
constexpr uint32_t BTN_DEBOUNCE_MS         = 30;
constexpr uint32_t BTN_LONG_MS             = 2000;
constexpr uint32_t BTN_VERY_LONG_MS        = 5000;
constexpr uint32_t BTN_STUCK_FAULT_MS      = 20000;  // held this long -> FAULT
constexpr uint32_t RESET_CONFIRM_WINDOW_MS = 10000;

// ---- Fault supervision ----
constexpr uint32_t SENSOR_STUCK_DIAG_MS        = 60000; // detected this long while DISARMED -> FAULT
constexpr uint32_t LOOP_STALL_LIMIT_MS         = 750;   // one loop() pass slower than this -> FAULT
constexpr uint32_t ARMING_OVERRUN_TOLERANCE_MS = 500;
constexpr uint32_t FAULT_MIN_DWELL_MS          = 3000;  // minimum time in FAULT before recovery is allowed

// ---- UI ----
constexpr uint32_t FEEDBACK_MS    = 1500;
constexpr uint32_t DIAG_SCREEN_MS = 4000;

// ---- Output patterns: alternating ON/OFF durations in ms, starting with ON ----
constexpr uint16_t PAT_ARMING_LED[]    = {250, 250};
constexpr uint16_t PAT_ALARM_BUZZER[]  = {200, 200};                       // steady beep-beep
constexpr uint16_t PAT_FAULT_LED[]     = {500, 500};
constexpr uint16_t PAT_FAULT_BUZZER[]  = {80, 80, 80, 80, 80, 900};        // triple chirp, long pause

// ---- Compile-time sanity of the configuration itself ----
static_assert(INTRUSION_CONFIRM_MS > SENSOR_DEBOUNCE_MS, "confirm time must exceed sensor debounce");
static_assert(BTN_LONG_MS < BTN_VERY_LONG_MS && BTN_VERY_LONG_MS < BTN_STUCK_FAULT_MS, "button thresholds must be ordered");
static_assert(SENSOR_STUCK_DIAG_MS > INTRUSION_CONFIRM_MS, "stuck timeout must exceed confirm time");
static_assert(ARMING_DURATION_MS >= 1000, "arming countdown needs at least 1 s");

// =====================================================================
// 2. TYPES
// =====================================================================

enum class State : uint8_t { DISARMED, ARMING, ARMED, ALARM, FAULT, COUNT };

enum class FaultCode : uint8_t {
  NONE, INVALID_STATE, BAD_TRANSITION, TIMING, LOOP_STALL, SENSOR_STUCK, BUTTON_STUCK
};

enum class ButtonEvent : uint8_t { NONE, SHORT_PRESS, LONG_PRESS, VERY_LONG_PRESS };

struct SensorStatus {
  bool     raw;           // instantaneous (noisy) reading
  uint32_t rawChangeMs;   // when raw last changed
  bool     detected;      // debounced reading
  uint32_t sinceMs;       // when the debounced value last changed
};

struct ButtonStatus {
  bool     raw;
  uint32_t rawChangeMs;
  bool     pressed;          // debounced
  uint32_t pressStartMs;
  State    pressStartState;  // state when press began (stale-press protection)
  bool     veryLongFired;    // very-long already reported OR press must be ignored on release
};

// Generic non-blocking ON/OFF pattern player (used for LEDs and buzzer).
struct Blinker {
  const uint16_t* steps;
  uint8_t  len;
  uint8_t  idx;
  uint32_t nextMs;
};

// =====================================================================
// 3. STATE (kept in one place)
// =====================================================================

static State     g_state        = State::DISARMED;
static uint32_t  g_stateEnterMs = 0;
static FaultCode g_activeFault  = FaultCode::NONE;
static FaultCode g_lastFault    = FaultCode::NONE;
static uint16_t  g_faultCount   = 0;

static uint32_t  g_intrusionCount   = 0;      // persisted in NVS
static bool      g_intrusionGateOpen = false; // ARMED only: sensor has been clear long enough

static bool      g_resetConfirmPending    = false;
static uint32_t  g_resetConfirmDeadlineMs = 0;

static uint32_t  g_diagUntilMs = 0;
static char      g_feedback[32] = "";
static uint32_t  g_feedbackUntilMs = 0;

static uint32_t  g_lastLoopMs = 0;

static SensorStatus sensor;
static ButtonStatus btn;
static Blinker blkLed, blkBuzzer;

static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static Preferences prefs;
static bool g_displayOk = false;

// =====================================================================
// 4. SMALL HELPERS
// =====================================================================

// Wrap-safe "deadline still in the future" test.
static inline bool futureOf(uint32_t deadline, uint32_t now) { return (int32_t)(deadline - now) > 0; }

template <size_t N> constexpr uint8_t lenOf(const uint16_t (&)[N]) { return (uint8_t)N; }

static bool stateIsValid(State s) { return (uint8_t)s < (uint8_t)State::COUNT; }

static const char* stateName(State s) {
  switch (s) {
    case State::DISARMED: return "DISARMED";
    case State::ARMING:   return "ARMING";
    case State::ARMED:    return "ARMED";
    case State::ALARM:    return "ALARM";
    case State::FAULT:    return "FAULT";
    default:              return "INVALID";
  }
}

static const char* faultName(FaultCode f) {
  switch (f) {
    case FaultCode::INVALID_STATE:  return "INVALID STATE";
    case FaultCode::BAD_TRANSITION: return "BAD TRANSITION";
    case FaultCode::TIMING:         return "TIMING ERROR";
    case FaultCode::LOOP_STALL:     return "LOOP STALL";
    case FaultCode::SENSOR_STUCK:   return "SENSOR STUCK";
    case FaultCode::BUTTON_STUCK:   return "BUTTON STUCK";
    default:                        return "NONE";
  }
}

static void setFeedback(const char* prefix, const char* msg, uint32_t now) {
  snprintf(g_feedback, sizeof(g_feedback), "%s%s", prefix, msg);
  g_feedbackUntilMs = now + FEEDBACK_MS;
  Serial.println(g_feedback);
}
static void accepted(const char* msg, uint32_t now) { setFeedback("OK: ", msg, now); }
static void rejected(const char* msg, uint32_t now) { setFeedback("NO: ", msg, now); }

// =====================================================================
// 5. BLINKER (non-blocking pattern engine)
// =====================================================================

static void blinkerStart(Blinker& b, const uint16_t* steps, uint8_t len, uint32_t now) {
  b.steps = steps; b.len = len; b.idx = 0; b.nextMs = now + steps[0];
}

// Returns true while the pattern is in an ON step. Catches up if loop was late.
static bool blinkerUpdate(Blinker& b, uint32_t now) {
  if (!b.steps || b.len == 0) return false;
  while (!futureOf(b.nextMs, now)) {
    b.idx = (uint8_t)((b.idx + 1) % b.len);
    b.nextMs += b.steps[b.idx];
  }
  return (b.idx % 2) == 0;
}

// =====================================================================
// 6. BUZZER DRIVER (passive piezo, PWM tone; works on core 2.x and 3.x)
// =====================================================================

static void buzzerInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PIN_BUZZER, TONE_ALARM_HZ, 8);
  ledcWriteTone(PIN_BUZZER, 0);
#else
  ledcSetup(BUZZER_LEDC_CHANNEL, TONE_ALARM_HZ, 8);
  ledcAttachPin(PIN_BUZZER, BUZZER_LEDC_CHANNEL);
  ledcWriteTone(BUZZER_LEDC_CHANNEL, 0);
#endif
}

// hz = 0 means silent.
static void buzzerWrite(uint16_t hz) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWriteTone(PIN_BUZZER, hz);
#else
  ledcWriteTone(BUZZER_LEDC_CHANNEL, hz);
#endif
}

// =====================================================================
// 7. PERSISTENT COUNTER (NVS)
// =====================================================================

static void counterLoad() { prefs.begin("secsys", false); g_intrusionCount = prefs.getUInt("count", 0); }
static void counterSave() { prefs.putUInt("count", g_intrusionCount); }   // called once per confirmed intrusion
static void counterReset(uint32_t now) {
  if (g_intrusionCount != 0) { g_intrusionCount = 0; counterSave(); }     // skip write if already 0
  accepted("COUNTER RESET", now);
}

// =====================================================================
// 8. INPUT PROCESSING
// =====================================================================

static bool readIrRaw() { return digitalRead(PIN_IR_SENSOR) == IR_ACTIVE_LEVEL; }

static void sensorInit(uint32_t now) {
  bool r = readIrRaw();
  sensor.raw = sensor.detected = r;
  sensor.rawChangeMs = sensor.sinceMs = now;
}

// Debounce: the debounced value follows raw only after raw has been stable for SENSOR_DEBOUNCE_MS.
static void sensorUpdate(uint32_t now) {
  bool r = readIrRaw();
  if (r != sensor.raw) { sensor.raw = r; sensor.rawChangeMs = now; }
  if (sensor.raw != sensor.detected && (now - sensor.rawChangeMs) >= SENSOR_DEBOUNCE_MS) {
    sensor.detected = sensor.raw;
    sensor.sinceMs  = sensor.rawChangeMs;   // time of the real edge, not of acceptance
  }
}
static uint32_t sensorDetectedMs(uint32_t now)  { return sensor.detected ? (now - sensor.sinceMs) : 0; }
static uint32_t sensorClearMs(uint32_t now)     { return sensor.detected ? 0 : (now - sensor.sinceMs); }
static bool     sensorClearEnough(uint32_t now) { return sensorClearMs(now) >= SENSOR_CLEAR_REQUIRED_MS; }

static bool readButtonRaw() { return digitalRead(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL; }

static void buttonInit(uint32_t now) {
  bool r = readButtonRaw();
  btn.raw = btn.pressed = r;
  btn.rawChangeMs = btn.pressStartMs = now;
  btn.pressStartState = g_state;
  btn.veryLongFired = r;   // button already held at boot: ignore until released once
}

// Classifies presses. SHORT/LONG are reported on RELEASE (so a long press can never also
// be a short press). VERY_LONG is reported once while still held; the release then gives no event.
static ButtonEvent buttonUpdate(uint32_t now) {
  bool r = readButtonRaw();
  if (r != btn.raw) { btn.raw = r; btn.rawChangeMs = now; }

  if (btn.raw != btn.pressed && (now - btn.rawChangeMs) >= BTN_DEBOUNCE_MS) {
    btn.pressed = btn.raw;
    if (btn.pressed) {
      btn.pressStartMs = now;
      btn.pressStartState = g_state;
      btn.veryLongFired = false;
    } else if (!btn.veryLongFired) {
      uint32_t held = now - btn.pressStartMs;
      return (held >= BTN_LONG_MS) ? ButtonEvent::LONG_PRESS : ButtonEvent::SHORT_PRESS;
    }
  } else if (btn.pressed && !btn.veryLongFired && (now - btn.pressStartMs) >= BTN_VERY_LONG_MS) {
    btn.veryLongFired = true;
    return ButtonEvent::VERY_LONG_PRESS;
  }
  return ButtonEvent::NONE;
}

// =====================================================================
// 9. STATE MACHINE CORE
// =====================================================================

// Whitelist of legal transitions. Anything else is a software fault.
static bool transitionAllowed(State from, State to) {
  switch (from) {
    case State::DISARMED: return to == State::ARMING;
    case State::ARMING:   return to == State::ARMED || to == State::DISARMED;
    case State::ARMED:    return to == State::ALARM || to == State::DISARMED;
    case State::ALARM:    return to == State::DISARMED;
    case State::FAULT:    return to == State::DISARMED;   // only via validated recovery
    default:              return false;
  }
}

static void outputsOnStateEnter(State s, uint32_t now) {
  switch (s) {
    case State::ARMING: blinkerStart(blkLed, PAT_ARMING_LED, lenOf(PAT_ARMING_LED), now); break;
    case State::ALARM:  blinkerStart(blkBuzzer, PAT_ALARM_BUZZER, lenOf(PAT_ALARM_BUZZER), now); break;
    case State::FAULT:
      blinkerStart(blkLed, PAT_FAULT_LED, lenOf(PAT_FAULT_LED), now);
      blinkerStart(blkBuzzer, PAT_FAULT_BUZZER, lenOf(PAT_FAULT_BUZZER), now);
      break;
    default: break;
  }
}

static void enterState(State next, uint32_t now) {
  if (next != State::FAULT && !(stateIsValid(g_state) && transitionAllowed(g_state, next))) {
    // Unexpected transition request: refuse it and fail safe.
    Serial.printf("[%lu] ILLEGAL TRANSITION %s -> %s\n", (unsigned long)now, stateName(g_state), stateName(next));
    g_activeFault = g_lastFault = FaultCode::BAD_TRANSITION;
    g_faultCount++;
    next = State::FAULT;
  }
  Serial.printf("[%lu] %s -> %s\n", (unsigned long)now, stateName(g_state), stateName(next));

  g_state = next;
  g_stateEnterMs = now;
  g_resetConfirmPending = false;      // any state change cancels a pending counter reset
  g_intrusionGateOpen = false;        // ARMED must re-prove the sensor is clear

  if (next == State::ALARM) {         // ALARM is entered exactly once per event
    g_intrusionCount++;
    counterSave();
    Serial.printf("[%lu] INTRUSION #%lu\n", (unsigned long)now, (unsigned long)g_intrusionCount);
  }
  outputsOnStateEnter(next, now);
}

static void raiseFault(FaultCode code, uint32_t now) {
  if (g_state == State::FAULT) return;
  g_activeFault = g_lastFault = code;
  g_faultCount++;
  Serial.printf("[%lu] FAULT: %s\n", (unsigned long)now, faultName(code));
  enterState(State::FAULT, now);
}

// =====================================================================
// 10. FAULT SUPERVISION + RECOVERY
// =====================================================================

static void faultCheck(uint32_t now, uint32_t loopGapMs) {
  if (!stateIsValid(g_state)) { raiseFault(FaultCode::INVALID_STATE, now); return; }
  if (g_state == State::FAULT) return;

  // Inconsistent timing: state timestamp in the future, or ARMING never finished.
  if ((int32_t)(now - g_stateEnterMs) < 0) { raiseFault(FaultCode::TIMING, now); return; }
  if (g_state == State::ARMING && (now - g_stateEnterMs) > ARMING_DURATION_MS + ARMING_OVERRUN_TOLERANCE_MS) {
    raiseFault(FaultCode::TIMING, now); return;
  }
  // A single loop pass that took far too long means something blocked the control flow.
  if (loopGapMs > LOOP_STALL_LIMIT_MS) { raiseFault(FaultCode::LOOP_STALL, now); return; }

  // Sensor detected for a very long time while DISARMED: nothing is armed, so it cannot be an
  // intrusion; treated as "blocked or failed sensor". In ARMED/ALARM it is NOT a fault (fail-secure).
  if (g_state == State::DISARMED && sensor.detected && sensorDetectedMs(now) >= SENSOR_STUCK_DIAG_MS) {
    raiseFault(FaultCode::SENSOR_STUCK, now); return;
  }
  // Button held unusually long.
  if (btn.pressed && (now - btn.pressStartMs) >= BTN_STUCK_FAULT_MS) {
    raiseFault(FaultCode::BUTTON_STUCK, now); return;
  }
}

static void runtimeReinit(uint32_t now) {
  sensorInit(now);
  g_resetConfirmPending = false;
  g_intrusionGateOpen = false;
  g_diagUntilMs = 0;
}

static bool faultRecoveryAllowed(uint32_t now, const char** why) {
  if (futureOf(g_stateEnterMs + FAULT_MIN_DWELL_MS, now)) { *why = "WAIT BEFORE RETRY"; return false; }
  switch (g_activeFault) {
    case FaultCode::SENSOR_STUCK:
      if (!sensorClearEnough(now)) { *why = "SENSOR NOT CLEAR"; return false; }
      break;
    case FaultCode::BUTTON_STUCK:
      if (btn.pressed) { *why = "RELEASE BUTTON"; return false; }
      break;
    default: break;   // software faults: dwell time + full re-init is the recovery
  }
  return true;
}

// =====================================================================
// 11. DECISIONS: BUTTON ACTIONS + PERIODIC STATE LOGIC
// =====================================================================

static void onButtonDisarmed(ButtonEvent ev, uint32_t now) {
  if (g_resetConfirmPending) {                      // 2nd step of the deliberate reset sequence
    g_resetConfirmPending = false;
    if (ev == ButtonEvent::LONG_PRESS) counterReset(now);
    else accepted("RESET CANCELLED", now);
    return;
  }
  switch (ev) {
    case ButtonEvent::SHORT_PRESS:
      if (!sensorClearEnough(now)) rejected("SENSOR NOT CLEAR", now);   // also covers "object still present after reset"
      else { accepted("ARMING", now); enterState(State::ARMING, now); }
      break;
    case ButtonEvent::LONG_PRESS:
      g_diagUntilMs = now + DIAG_SCREEN_MS;
      break;
    case ButtonEvent::VERY_LONG_PRESS:              // 1st step: open the confirmation window
      g_resetConfirmPending = true;
      g_resetConfirmDeadlineMs = now + RESET_CONFIRM_WINDOW_MS;
      break;
    default: break;
  }
}

static void onButtonArming(ButtonEvent ev, uint32_t now) {
  if (ev == ButtonEvent::SHORT_PRESS) { accepted("ARMING CANCELLED", now); enterState(State::DISARMED, now); }
  else rejected("NOT ALLOWED", now);
}

static void onButtonArmed(ButtonEvent ev, uint32_t now) {
  if (ev == ButtonEvent::LONG_PRESS) { accepted("DISARMED", now); enterState(State::DISARMED, now); }
  else rejected("HOLD 2s TO DISARM", now);
}

static void onButtonAlarm(ButtonEvent ev, uint32_t now) {
  if (ev == ButtonEvent::LONG_PRESS) { accepted("ALARM RESET", now); enterState(State::DISARMED, now); }
  else rejected("HOLD 2s TO RESET", now);
}

static void onButtonFault(ButtonEvent ev, uint32_t now) {
  if (ev == ButtonEvent::LONG_PRESS) {
    const char* why = nullptr;
    if (faultRecoveryAllowed(now, &why)) {
      runtimeReinit(now);
      g_activeFault = FaultCode::NONE;
      enterState(State::DISARMED, now);        // never straight back to ARMED
      accepted("RECOVERED", now);
    } else rejected(why, now);
  } else if (ev == ButtonEvent::VERY_LONG_PRESS) {
    Serial.println("Safe restart requested");
    Serial.flush();
    ESP.restart();                             // counter is in NVS and survives
  } else rejected("NOT ALLOWED", now);
}

static void handleButtonEvent(ButtonEvent ev, uint32_t now) {
  if (ev == ButtonEvent::NONE) return;
  // A press that began in another state must not act in this one (e.g. held through ARMING -> ARMED).
  if (btn.pressStartState != g_state) { rejected("STALE PRESS", now); return; }
  switch (g_state) {
    case State::DISARMED: onButtonDisarmed(ev, now); break;
    case State::ARMING:   onButtonArming(ev, now);   break;
    case State::ARMED:    onButtonArmed(ev, now);    break;
    case State::ALARM:    onButtonAlarm(ev, now);    break;
    case State::FAULT:    onButtonFault(ev, now);    break;
    default: break;
  }
}

static void stateMachineUpdate(uint32_t now) {
  switch (g_state) {
    case State::DISARMED:
      if (g_resetConfirmPending && !futureOf(g_resetConfirmDeadlineMs, now)) {
        g_resetConfirmPending = false;
        rejected("RESET TIMED OUT", now);
      }
      break;

    case State::ARMING:
      if (sensor.detected) { rejected("OBJECT DETECTED", now); enterState(State::DISARMED, now); }
      else if ((now - g_stateEnterMs) >= ARMING_DURATION_MS) enterState(State::ARMED, now);
      break;

    case State::ARMED:
      if (!g_intrusionGateOpen) {
        // Clear-state requirement: no detection is accepted until the sensor has been clear.
        if (sensorClearEnough(now)) g_intrusionGateOpen = true;
      } else if (sensor.detected && sensorDetectedMs(now) >= INTRUSION_CONFIRM_MS) {
        enterState(State::ALARM, now);          // latched: nothing here ever leaves ALARM
      }
      break;

    case State::ALARM:   // latched until an authorised long press (handled in onButtonAlarm)
    case State::FAULT:   // left only via validated recovery
    default: break;
  }
}

// =====================================================================
// 12. OUTPUTS (render only)
// =====================================================================

static void outputsUpdate(uint32_t now) {
  bool green = false, yellow = false, red = false, buzzer = false;
  switch (g_state) {
    case State::DISARMED: green = true; break;
    case State::ARMING:   yellow = blinkerUpdate(blkLed, now); break;
    case State::ARMED:    yellow = true; break;
    case State::ALARM:    red = true; buzzer = blinkerUpdate(blkBuzzer, now); break;
    case State::FAULT:    red = blinkerUpdate(blkLed, now); buzzer = blinkerUpdate(blkBuzzer, now); break;
    default:              red = true; break;   // unknown state: show something safe
  }
  digitalWrite(PIN_LED_GREEN,  green);
  digitalWrite(PIN_LED_YELLOW, yellow);
  digitalWrite(PIN_LED_RED,    red);

  uint16_t hz = 0;
  if (buzzer) hz = (g_state == State::FAULT) ? TONE_FAULT_HZ : TONE_ALARM_HZ;
  buzzerWrite(hz);   // 0 = silent
}

static void buildScreen(uint32_t now, char* t, char* a, char* b, char* c) {
  const char* sens = sensor.detected ? "DETECTED" : "CLEAR";
  snprintf(b, 32, "Count: %lu", (unsigned long)g_intrusionCount);

  switch (g_state) {
    case State::DISARMED:
      if (g_resetConfirmPending) {
        snprintf(t, 32, "RESET COUNT?");
        snprintf(a, 32, "Hold 2s = CONFIRM");
        snprintf(b, 32, "Short = CANCEL");
        snprintf(c, 32, "Timeout in %lus", (unsigned long)((g_resetConfirmDeadlineMs - now + 999) / 1000));
      } else if (futureOf(g_diagUntilMs, now)) {
        snprintf(t, 32, "DIAGNOSTICS");
        snprintf(a, 32, "Faults: %u", g_faultCount);
        snprintf(b, 32, "Last: %s", faultName(g_lastFault));
        snprintf(c, 32, "Uptime: %lus", (unsigned long)(now / 1000));
      } else {
        snprintf(t, 32, "DISARMED");
        snprintf(a, 32, "Sensor: %s", sens);
        snprintf(c, 32, "Press: ARM");
      }
      break;
    case State::ARMING: {
      uint32_t elapsed = now - g_stateEnterMs;
      uint32_t left = (elapsed >= ARMING_DURATION_MS) ? 1 : (ARMING_DURATION_MS - elapsed + 999) / 1000;
      snprintf(t, 32, "ARMING: %lu", (unsigned long)left);
      snprintf(a, 32, "Sensor: %s", sens);
      snprintf(c, 32, "Press: CANCEL");
      break;
    }
    case State::ARMED:
      snprintf(t, 32, "SYSTEM ARMED");
      if (!g_intrusionGateOpen) snprintf(a, 32, "Sensor: WAIT CLEAR");
      else snprintf(a, 32, "Sensor: %s", sens);
      snprintf(c, 32, "Hold 2s: DISARM");
      break;
    case State::ALARM:
      snprintf(t, 32, "INTRUSION DETECTED");
      if (sensor.detected && sensorDetectedMs(now) >= SENSOR_STUCK_DIAG_MS) snprintf(a, 32, "Sensor HELD - CHECK");
      else snprintf(a, 32, "Sensor: %s", sens);
      snprintf(c, 32, "Hold 2s: RESET");
      break;
    default:   // FAULT or an invalid state value
      snprintf(t, 32, "SYSTEM FAULT");
      snprintf(a, 32, "%s", g_state == State::FAULT ? faultName(g_activeFault) : "BAD STATE");
      snprintf(c, 32, "Hold 2s: RECOVER");
      break;
  }

  // Footer override: hold-progress hint, then transient accept/reject feedback.
  if (btn.pressed && !btn.veryLongFired && (now - btn.pressStartMs) >= BTN_LONG_MS) {
    snprintf(c, 32, "Release = LONG PRESS");
  } else if (futureOf(g_feedbackUntilMs, now)) {
    snprintf(c, 32, "%s", g_feedback);
  }
}

static void displayUpdate(uint32_t now) {
  static uint32_t lastDrawMs = 0;
  static char lastFrame[160] = "";
  if (!g_displayOk || (now - lastDrawMs) < DISPLAY_MIN_INTERVAL_MS) return;
  lastDrawMs = now;

  char t[32], a[32], b[32], c[32], frame[160];
  buildScreen(now, t, a, b, c);
  snprintf(frame, sizeof(frame), "%s|%s|%s|%s", t, a, b, c);
  if (strcmp(frame, lastFrame) == 0) return;      // redraw only on change (I2C transfer ~25 ms)
  strcpy(lastFrame, frame);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(strlen(t) * 12 <= OLED_WIDTH ? 2 : 1);
  display.setCursor(0, 0);  display.print(t);
  display.setTextSize(1);
  display.setCursor(0, 24); display.print(a);
  display.setCursor(0, 36); display.print(b);
  display.setCursor(0, 54); display.print(c);
  display.display();
}

// =====================================================================
// 13. DEBUG FAULT INJECTION (Serial Monitor, 115200) - for test scenario 12
//   f = corrupt state variable   b = illegal transition   t = bad timestamp   s = stall loop 1 s
//   Set ENABLE_FAULT_INJECTION to 0 for a final build.
// =====================================================================
#define ENABLE_FAULT_INJECTION 1

static void serialDebug(uint32_t now) {
#if ENABLE_FAULT_INJECTION
  while (Serial.available()) {
    switch (Serial.read()) {
      case 'f': g_state = (State)99; break;
      case 'b': enterState(State::ARMED, now); break;           // DISARMED->ARMED is illegal
      case 't': g_stateEnterMs = now + 100000UL; break;
      case 's': delay(LOOP_STALL_LIMIT_MS + 250); break;        // deliberate blocking, test only
      default: break;
    }
  }
#endif
}

// =====================================================================
// 14. SETUP / LOOP
// =====================================================================

void setup() {
  // Outputs first, in a known-safe state.
  pinMode(PIN_LED_GREEN, OUTPUT);  digitalWrite(PIN_LED_GREEN, LOW);
  pinMode(PIN_LED_YELLOW, OUTPUT); digitalWrite(PIN_LED_YELLOW, LOW);
  pinMode(PIN_LED_RED, OUTPUT);    digitalWrite(PIN_LED_RED, LOW);
  buzzerInit();                    // silent at boot
  pinMode(PIN_IR_SENSOR, INPUT_PULLUP);   // helps open-collector modules; harmless for push-pull
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  g_displayOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR);
  if (!g_displayOk) Serial.println("OLED not found (check address/wiring) - running headless");

  counterLoad();
  uint32_t now = millis();
  g_state = State::DISARMED;          // always boot into the safe state
  g_stateEnterMs = now;
  sensorInit(now);
  buttonInit(now);
  g_lastLoopMs = now;
  Serial.printf("[%lu] BOOT: DISARMED, stored intrusion count = %lu\n", (unsigned long)now, (unsigned long)g_intrusionCount);
}

void loop() {
  const uint32_t now = millis();
  const uint32_t loopGap = now - g_lastLoopMs;
  g_lastLoopMs = now;

  // 1. inputs
  sensorUpdate(now);
  ButtonEvent ev = buttonUpdate(now);
  serialDebug(now);

  // 2. supervision
  faultCheck(now, loopGap);

  // 3. decisions
  handleButtonEvent(ev, now);
  stateMachineUpdate(now);

  // 4. outputs
  outputsUpdate(now);
  displayUpdate(now);
}
