/*
  Force-sensing grasper: buzzer alarm when grip force passes a threshold

  Wiring
    Load cell -> HX711          HX711 -> Arduino Uno       Buzzer
    Red   -> E+                 VCC -> 5V                  + (long leg) -> D10
    Black -> E-                 GND -> GND                 - (short leg) -> GND
    White -> A-                 DT  -> D3
    Green -> A+                 SCK -> D2

  Library: "HX711 Arduino Library" by Bogdan Necula (Library Manager).

  Serial Monitor / Plotter at 57600 baud. Type a command and press Enter:
    t         re-zero (tare). Do this with nothing touching the finger ring.
    c         calibrate: you will be asked for a known mass in grams
    s300      set the alarm threshold to 300 g (any number)
    m         mute / unmute the buzzer
    f         flip direction (if squeezing shows negative force)
    p         print current settings
  Calibration and threshold are saved, so they survive unplugging.

  Alarm
    below WARN_FRACTION x threshold   silent
    warning zone                      short beep every 0.5 s
    at or above threshold             continuous tone
    It switches off only after force drops HYSTERESIS_FRACTION below the
    level that turned it on, so it doesn't chatter at the threshold.
*/
#include <EEPROM.h>
#include "HX711.h"

// ---------------- pins ----------------
const uint8_t DT_PIN     = 3;
const uint8_t SCK_PIN    = 2;
const uint8_t BUZZER_PIN = 10;

// ---------------- settings ----------------
// Passive buzzer (needs a tone signal) = true. Active buzzer (beeps on its own
// when given 5 V) = false. If yours is silent or only clicks, flip this.
const bool  BUZZER_IS_PASSIVE   = true;
const int   TONE_HZ             = 2500;




const float DEFAULT_THRESHOLD_G = 1000.0;   // used until you set one with "s"
const float WARN_FRACTION       = 0.80;    // warning starts at 80% of threshold
const float HYSTERESIS_FRACTION = 0.10;    // must drop 10% below to switch off
const float SMOOTHING           = 0.30;    // 0..1, higher = faster but noisier

// Rough counts-per-gram for a 5 kg cell at gain 128. Replace by calibrating ("c").
// Negative because squeezing reads negative on this build; "f" flips it if needed.
const float DEFAULT_CAL_FACTOR  = -420.0;

// ---------------- saved settings ----------------
struct Settings {
  uint32_t magic;
  float    calFactor;    // HX711 counts per gram (sign sets the direction)
  float    thresholdG;
  bool     calibrated;
};
const uint32_t MAGIC = 0x47525351;   // changed: resets saved settings once so the flipped default takes effect
Settings cfg;

HX711 scale;
long  zeroOffset = 0;
float forceG = 0;          // smoothed force, grams
bool  muted = false;

enum AlarmState { SILENT, WARNING, ALARM };
AlarmState state = SILENT;

// ---------------- helpers ----------------
void saveSettings() { EEPROM.put(0, cfg); }

void loadSettings() {
  EEPROM.get(0, cfg);
  if (cfg.magic != MAGIC || isnan(cfg.calFactor) || cfg.calFactor == 0 ||
      isnan(cfg.thresholdG) || cfg.thresholdG <= 0) {
    cfg.magic = MAGIC;
    cfg.calFactor = DEFAULT_CAL_FACTOR;
    cfg.thresholdG = DEFAULT_THRESHOLD_G;
    cfg.calibrated = false;
    saveSettings();
  }
}

long readAverage(uint8_t n) {             // blocking, used only for tare/calibration
  long sum = 0;
  for (uint8_t i = 0; i < n; i++) {
    while (!scale.is_ready()) {}
    sum += scale.read();
  }
  return sum / n;
}

void buzzerOn() {
  if (muted) return;
  if (BUZZER_IS_PASSIVE) tone(BUZZER_PIN, TONE_HZ);
  else digitalWrite(BUZZER_PIN, HIGH);
}

void buzzerOff() {
  if (BUZZER_IS_PASSIVE) noTone(BUZZER_PIN);
  digitalWrite(BUZZER_PIN, LOW);
}

void tare() {
  buzzerOff();
  Serial.println(F("# Zeroing... don't touch the finger ring."));
  zeroOffset = readAverage(20);
  forceG = 0;
  state = SILENT;
  Serial.println(F("# Zeroed."));
}

void printSettings() {
  Serial.print(F("# Threshold: ")); Serial.print(cfg.thresholdG, 0);
  Serial.print(F(" g | warning from ")); Serial.print(cfg.thresholdG * WARN_FRACTION, 0);
  Serial.print(F(" g | cal factor ")); Serial.print(cfg.calFactor, 2);
  Serial.print(cfg.calibrated ? F(" (calibrated)") : F(" (NOT calibrated: values are rough)"));
  Serial.print(F(" | buzzer ")); Serial.println(muted ? F("muted") : F("on"));
}

String readLine() {                       // waits for a line typed in the Serial Monitor
  while (!Serial.available()) {}
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

void calibrate() {
  buzzerOff();
  Serial.println(F("# Calibration. Step 1: remove all load, then press Enter."));
  readLine();
  zeroOffset = readAverage(20);
  Serial.println(F("# Step 2: apply a known load in the squeeze direction"));
  Serial.println(F("#   (e.g. hang a known mass from the finger ring), then type its mass in grams and press Enter."));
  float grams = readLine().toFloat();
  if (grams <= 0) { Serial.println(F("# Invalid mass, calibration cancelled.")); return; }
  long counts = readAverage(20) - zeroOffset;
  if (labs(counts) < 100) { Serial.println(F("# Reading barely changed. Check wiring, cancelled.")); return; }
  cfg.calFactor = (float)counts / grams;   // sign makes the squeeze direction positive
  cfg.calibrated = true;
  saveSettings();
  Serial.println(F("# Calibrated and saved. Remove the load and type t to re-zero."));
  printSettings();
}

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = tolower(cmd.charAt(0));
  if (c == 't') tare();
  else if (c == 'c') calibrate();
  else if (c == 'm') { muted = !muted; if (muted) buzzerOff(); printSettings(); }
  else if (c == 'p') printSettings();
  else if (c == 'f') {
    cfg.calFactor = -cfg.calFactor; saveSettings(); forceG = -forceG; state = SILENT;
    Serial.println(F("# Direction flipped and saved."));
    printSettings();
  }
  else if (c == 's') {
    float v = cmd.substring(1).toFloat();
    if (v > 0) { cfg.thresholdG = v; saveSettings(); printSettings(); }
    else Serial.println(F("# Usage: s300 sets the threshold to 300 g"));
  }
}

void updateAlarm(float f) {
  float thr  = cfg.thresholdG;
  float warn = thr * WARN_FRACTION;
  float hys  = thr * HYSTERESIS_FRACTION;

  switch (state) {
    case SILENT:
      if (f >= thr) state = ALARM;
      else if (f >= warn) state = WARNING;
      break;
    case WARNING:
      if (f >= thr) state = ALARM;
      else if (f < warn - hys) state = SILENT;
      break;
    case ALARM:
      if (f < thr - hys) state = (f >= warn - hys) ? WARNING : SILENT;
      break;
  }

  if (state == ALARM) {
    buzzerOn();
  } else if (state == WARNING) {
    // 120 ms beep every 500 ms, timed with millis() so readings never pause
    if ((millis() % 500) < 120) buzzerOn(); else buzzerOff();
  } else {
    buzzerOff();
  }
}

// ---------------- main ----------------
void setup() {
  Serial.begin(57600);
  pinMode(BUZZER_PIN, OUTPUT);
  buzzerOff();
  scale.begin(DT_PIN, SCK_PIN);
  loadSettings();

  Serial.println(F("# Force-sensing grasper"));
  unsigned long t0 = millis();
  while (!scale.is_ready()) {
    if (millis() - t0 > 2000) {
      Serial.println(F("# ERROR: HX711 not responding. Check VCC, GND, DT->D3, SCK->D2."));
      while (true) {}
    }
  }

  // short startup chirp so you know the buzzer works
  buzzerOn(); delay(80); buzzerOff();

  tare();
  printSettings();
  Serial.println(F("# Commands: t tare | c calibrate | s300 threshold | m mute | f flip | p settings"));
}

void loop() {
  handleSerial();

  if (scale.is_ready()) {                       // non-blocking: ~10 or 80 readings/s
    long counts = scale.read() - zeroOffset;
    float g = counts / cfg.calFactor;
    forceG = SMOOTHING * g + (1 - SMOOTHING) * forceG;

    // Plotter-friendly line: three traces
    Serial.print(F("force_g:"));     Serial.print(forceG, 1);
    Serial.print(F("\tthreshold_g:")); Serial.print(cfg.thresholdG, 0);
    Serial.print(F("\twarning_g:"));   Serial.println(cfg.thresholdG * WARN_FRACTION, 0);
  }

  updateAlarm(forceG);
}
