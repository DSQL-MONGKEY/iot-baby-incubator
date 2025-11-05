#include <Arduino.h>

// ===== KONFIG =====
#define RELAY_ACTIVE_LOW true
#define CYCLES 3
#define STARTUP_GRACE_MS 1500
#define STEP_ON_MS 1500
#define STEP_OFF_MS 600
#define ALL_ON_PULSE_MS 800
#define ALL_OFF_PAUSE_MS 1200

#ifndef LED_BUILTIN
#define LED_BUILTIN 2   // mayoritas ESP32 DevKit
#endif

// Mapping awalmu (boleh ganti nasnti saat rewiring)
const uint8_t RELAY_PINS[8] = {18, 33, 32, 27, 26, 25, 2, 0};

// ===== UTIL =====
inline void relayWrite(uint8_t idx, bool on) {
  bool level = RELAY_ACTIVE_LOW ? !on : on;
  digitalWrite(RELAY_PINS[idx], level);
}
inline void allOff() { for (int i = 0; i < 8; i++) relayWrite(i, false); }
inline void allOn()  { for (int i = 0; i < 8; i++) relayWrite(i, true);  }

void heartbeatOnce() {
  static uint32_t t = 0;
  static bool s = false;
  if (millis() - t >= 500) { t = millis(); s = !s; digitalWrite(LED_BUILTIN, s); }
}

// ===== SETUP =====
void setup() {
  // LED indikator
  pinMode(LED_BUILTIN, OUTPUT);
  // Blink cepat 3x tanda program start
  for (int i=0;i<3;i++){ digitalWrite(LED_BUILTIN, HIGH); delay(120); digitalWrite(LED_BUILTIN, LOW); delay(120); }

  // Amankan strapping pins bila masih terpakai
  pinMode(0, OUTPUT); pinMode(2, OUTPUT);
  digitalWrite(0, RELAY_ACTIVE_LOW ? HIGH : LOW); // OFF
  digitalWrite(2, RELAY_ACTIVE_LOW ? HIGH : LOW); // OFF

  for (int i = 0; i < 8; i++) pinMode(RELAY_PINS[i], OUTPUT);
  allOff();

  delay(STARTUP_GRACE_MS);
}

// ===== LOOP =====
void loop() {
  for (int cycle = 0; cycle < CYCLES; cycle++) {
    for (int i = 0; i < 8; i++) {
      relayWrite(i, true);
      for (uint32_t t=0; t<STEP_ON_MS/10; t++){ heartbeatOnce(); delay(10); }
      relayWrite(i, false);
      for (uint32_t t=0; t<STEP_OFF_MS/10; t++){ heartbeatOnce(); delay(10); }
    }
    allOn();
    for (uint32_t t=0; t<ALL_ON_PULSE_MS/10; t++){ heartbeatOnce(); delay(10); }
    allOff();
    for (uint32_t t=0; t<ALL_OFF_PAUSE_MS/10; t++){ heartbeatOnce(); delay(10); }
  }

  // Ulangi terus
}
