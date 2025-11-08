#include <Arduino.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>


// PIN MAPPING
static const bool RELAY_ACTIVE_LOW = true;
const uint8_t RELAY_PINS[8] = { 18, 33, 32, 27, 26, 25, 2, 0 };

// GROUPING AKTUATOR
const uint8_t FAN_CH[] = { 0, 1, 2, 3, 4, 5, };
const uint8_t LAMP_CH[] = { 6, 7 };

// DHT22
#define DHT_PIN 17
#define DHT_TYPE DHT22

// DS18B20
#define ONEWIRE_PIN 4

// GPS NEO-7N
#define GPS_RX_PIN 13
#define GPS_TX_PIN 5

struct Controlfg {
  float temp_on_c = 36.0f;
  float temp_off_c = 35.5f;
  float rh_on_pct = 65.0f;
  float rh_off_pct = 60.0f;
  float ema_alpha = 0.20f;
} cfg;


// HELPERS
inline int ON_LEVEL() {
  return RELAY_ACTIVE_LOW ? LOW : HIGH;
}
inline int OFF_LEVEL() {
  return RELAY_ACTIVE_LOW ? HIGH: LOW;
}

inline void relayOnPin(uint8_t pin) {
  digitalWrite(pin, ON_LEVEL());
}
inline void relayOffPin(uint8_t pin) {
  digitalWrite(pin, OFF_LEVEL());
}


void setGroup(const uint8_t *chs, size_t n, bool on) {
  for (size_t i = 0; i < n; ++i) {
    uint8_t pin = RELAY_PINS[chs[i]];

    if (on) relayOnPin(pin); else relayOffPin(pin); 
  }
}

void allRelaysOff() {
  for (uint8_t pin: RELAY_PINS) relayOffPin(pin);
}

// GLOBALS

DHT dht(DHT_PIN, DHT_TYPE);

OneWire oneWire(ONEWIRE_PIN);
DallasTemperature ds(&oneWire);

TinyGPSPlus gps;
HardwareSerial& GPS_SER = Serial2;

float t_ds_c = NAN;
float t_dht_c = NAN;
float rh_dht = NAN;
float t_main = NAN;

// EMA BUFFER

float ema(float prev, float val, float alpha) {
  if (isnan(val)) return prev;
  if(isnan(prev)) return val;

  return (alpha * val) + (1.0f - alpha) *prev;
}

enum class FanMode: uint8_t { AUTO = 0, FORCE_ON, FORCE_OFF };
FanMode fanMode = FanMode::AUTO;
bool fanState = false;

// LAMP STATE (combined with CH7 & CH8)

bool lampState = false;

// SCHEDULER
uint32_t tNow;

// DHT SCHEDULE
const uint32_t DHT_PERIOD_MS = 2000;
uint32_t tNextDHT = 0;

// DS18B20  ASYNC SCHEDULE
enum class DSState: uint8_t { IDLE = 0, CONVERTING, READY };
DSState dsState = DSState::IDLE;
uint32_t dsReadyAt = 0;
const uint16_t DS_CONV_MS = 750;
uint32_t tNextDSKick = 0;
const uint32_t DS_KICK_PERIOD_MS = 1500;

// GPS PRINT SCHEDULE
uint32_t tNextGpsPrint = 0;
const uint32_t GPS_PRINT_MS= 2000;

// Control Schedule
uint32_t tNextCtrl = 0;
const uint32_t CTRL_PERIOD_MS = 500;

// FAN CONTROL
void applyFan(bool on) {
  setGroup(FAN_CH, sizeof(FAN_CH), on);
  fanState = on;
}

void applyLamp(bool on) {
  setGroup(LAMP_CH, sizeof(LAMP_CH), on);
  lampState = on;
}

void controlLoop() {
  float t = !isnan(t_ds_c) ? t_ds_c : t_dht_c;
  float h = rh_dht;

  t_main = ema(t_main, t, cfg.ema_alpha);

  bool wantOn = fanState;

  if(fanMode == FanMode::AUTO) {
    // combined hysteresis
    if(!isnan(t_main) || (t_main >= cfg.temp_on_c)) wantOn = true;
    if(!isnan(h) && (h >= cfg.rh_on_pct)) wantOn = true;

    if(!isnan(t_main) && (t_main <= cfg.temp_off_c) && !isnan(h) && (h <= cfg.rh_off_pct)) wantOn = false;
  } else if (fanMode == FanMode::FORCE_ON) {
    wantOn = true;
  } else {
    wantOn = false;
  }

  if(wantOn != fanState) {
    applyFan(wantOn);
    Serial.printf("[FAN] mode=%s -> %s\n", fanMode == FanMode::AUTO ? "AUTO" : (fanMode == FanMode::FORCE_ON ? "FORCE_ON" : "FORCE_OFF"), wantOn ? "ON" : "OFF");
  }
}

// TELEMETRY PRINTING

void printTelemetry() {
  double lat = gps.location.isValid() ? gps.location.lat() : NAN;
  double lon = gps.location.isValid() ? gps.location.lng() : NAN;
  int sat = gps.satellites.isValid() ? gps.satellites.value() : -1;

  Serial.printf("{ \"t_ds\":%.2f, \"t_dht\":%.2f, \"rh\":%.1f, \"t_main\":%.2f, "
    "\"fan\":\"%s\", \"mode\":\"%s\", \"lamp\":\"%s\", "
    "\"lat\":%s, \"lon\":%s, \"sat\":%d }\n",
    t_ds_c, t_dht_c, rh_dht, t_main,
    fanState?"ON":"OFF",
    fanMode==FanMode::AUTO?"AUTO":(fanMode==FanMode::FORCE_ON?"FORCE_ON":"FORCE_OFF"),
    lampState?"ON":"OFF",
    isnan(lat)?"null":String(lat,6).c_str(),
    isnan(lon)?"null":String(lon,6).c_str(),
    sat
  );
}

// ================== SERIAL MENU =================
void printHelp() {
  Serial.println(F(
    "\n== Inkubator MVP Control ==\n"
    "m : show menu\n"
    "a : FAN mode AUTO\n"
    "n : FAN FORCE ON\n"
    "f : FAN FORCE OFF\n"
    "l : toggle LAMP group (CH7+CH8)\n"
    "p : print current telemetry once\n"
  ));
}

void handleSerial() {
  while (Serial.available()) {
    int c = tolower(Serial.read());
    switch (c) {
      case 'm': printHelp(); break;
      case 'a': fanMode = FanMode::AUTO;     Serial.println(F("[FAN] mode=AUTO"));     break;
      case 'n': fanMode = FanMode::FORCE_ON; Serial.println(F("[FAN] mode=FORCE_ON")); break;
      case 'f': fanMode = FanMode::FORCE_OFF;Serial.println(F("[FAN] mode=FORCE_OFF"));break;
      case 'l': applyLamp(!lampState);       Serial.printf("[LAMP] %s\n", lampState?"ON":"OFF"); break;
      case 'p': printTelemetry(); break;
      default: break;
    }
  }
}

// ================== SETUP =================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nBooting Inkubator MVP...");

  // Relay outputs -> OFF (failsafe)
  for (uint8_t pin : RELAY_PINS) {
    pinMode(pin, OUTPUT);
    relayOffPin(pin);
  }

  // Sensors init
  dht.begin();

  ds.begin();
  ds.setResolution(12);
  ds.setWaitForConversion(false); // non-blocking conversion

  // GPS init
  GPS_SER.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  printHelp();

  // Kick first DS conversion
  ds.requestTemperatures();
  dsState   = DSState::CONVERTING;
  dsReadyAt = millis() + DS_CONV_MS;
  tNextDSKick = millis() + DS_KICK_PERIOD_MS;

  // Schedule
  tNextDHT = millis();
  tNextCtrl = millis();
  tNextGpsPrint = millis() + GPS_PRINT_MS;
}

// ================== LOOP =================
void loop() {
  tNow = millis();

  // ---- GPS parse (tight loop, non-blocking) ----
  while (GPS_SER.available()) {
    gps.encode(GPS_SER.read());
  }

  // ---- DHT every 2s ----
  if ((int32_t)(tNow - tNextDHT) >= 0) {
    float h = dht.readHumidity();
    float t = dht.readTemperature(); // Celsius
    if (!isnan(h)) rh_dht  = ema(rh_dht,  h, cfg.ema_alpha);
    if (!isnan(t)) t_dht_c = ema(t_dht_c, t, cfg.ema_alpha);
    tNextDHT = tNow + DHT_PERIOD_MS;
  }

  // ---- DS18B20 async state machine ----
  if (dsState == DSState::CONVERTING && (int32_t)(tNow - dsReadyAt) >= 0) {
    // waktu konversi sudah lewat -> baca
    float v = ds.getTempCByIndex(0); // asumsi 1 sensor
    if (v > -127.0f && v < 125.0f) {
      t_ds_c = ema(t_ds_c, v, cfg.ema_alpha);
    }
    dsState = DSState::READY;
  }
  if ((int32_t)(tNow - tNextDSKick) >= 0) {
    ds.requestTemperatures();              // mulai konversi lagi
    dsReadyAt   = tNow + DS_CONV_MS;
    dsState     = DSState::CONVERTING;
    tNextDSKick = tNow + DS_KICK_PERIOD_MS;
  }

  // ---- Control every 500ms ----
  if ((int32_t)(tNow - tNextCtrl) >= 0) {
    controlLoop();
    tNextCtrl = tNow + CTRL_PERIOD_MS;
  }

  // ---- Telemetry print every 2s ----
  if ((int32_t)(tNow - tNextGpsPrint) >= 0) {
    printTelemetry();
    tNextGpsPrint = tNow + GPS_PRINT_MS;
  }

  // ---- Serial command ----
  handleSerial();
}
