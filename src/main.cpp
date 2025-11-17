#include <Arduino.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// WIFI
const char* WIFI_SSID = "POCO X3 NFC";
const char* WIFI_PASSWORD = "123456789";

void ensureWifi() {
  delay(10);
  Serial.println("[WiFi]: Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("[WiFi]: Connected Successfully");
  Serial.println("[WiFi]: IP Address: ");
  Serial.println(WiFi.localIP());
}

// config MQTT
const char* mqtt_server = "d847cd151fbe4985a1bc32cbd787651d.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_username = "esp32.subscriber.publisher";
const char* mqtt_password = "DxESP32Rext";
char clientId[40] = {0}; 

// MQTT Topics
const char* TOPIC_STATE = "/psk/incubator/state";
const char* TOPIC_TELEMETRY = "/psk/incubator/telemetry";
const char* TOPIC_FAN = "/psk/incubator/fan";
const char* TOPIC_LAMP = "/psk/incubator/lamp";
const char* TOPIC_CONTROL_MODE = "/psk/incubator/control-mode";
const char* TOPIC_STATUS = "/psk/incubator/status";

const size_t MQTT_BUFFER = 512;
uint32_t stateRev = 0;

bool needPublishState = true;
bool needPublishTelemetry = false;


WiFiClientSecure espSecureClient;
PubSubClient MqttClient(espSecureClient);

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


// PIN MAPPING
static const bool RELAY_ACTIVE_LOW = true;
const uint8_t RELAY_PINS[8] = { 18, 33, 32, 27, 26, 25, 2, 0 };

// GROUPING AKTUATOR
const uint8_t FAN_CH[] = { 0, 1, 2, 3, 4, 5, };
uint8_t fanArr[6] = {0,0,0,0,0,0};
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

enum class FanMode: uint8_t { AUTO = 0, MANUAL, FORCE_ON, FORCE_OFF };
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
void applyFanGroup(bool on) {
  for(size_t i = 0; i < sizeof(FAN_CH); i++) {
    uint8_t pin = RELAY_PINS[FAN_CH[i]];
    if(on) relayOnPin(pin); else relayOffPin(pin);
    fanArr[i] = on ? 1 : 0;
  }
  needPublishState = true;
  stateRev++;
}

void applyFanArray(const uint8_t arr[6]) {
  for(size_t i = 0; i < 6; ++i) {
    uint8_t pin = RELAY_PINS[FAN_CH[i]];
    if(arr[i]) relayOnPin(pin); else relayOffPin(pin);

    fanArr[i] = arr[i] ? 1 : 0;
    stateRev++;
  }
}

void applyLamp(bool on) {
  setGroup(LAMP_CH, sizeof(LAMP_CH), on);
  lampState = on;
  needPublishState = true;
  stateRev++;
}

// Small JSON builder for FAN & LAMP
void buildFanArrayJson(char out[32]) {
  snprintf(out, 32, "[%u,%u,%u,%u,%u,%u]",
    fanArr[0], fanArr[1], fanArr[2], fanArr[3], fanArr[4], fanArr[5]
  );
}
void buildLampArrayJson(char out[8]) {
  uint8_t v = lampState ? 1 : 0;
  snprintf(out, 8, "[%u,%u]", v, v);
}


// MQTT Callback
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = topic;
  DynamicJsonDocument doc(384);

  DeserializationError err = deserializeJson(doc, payload, length);
  if(err) {
    Serial.printf("[MQTT] JSON parse error: %s (topic=%s)\n", err.c_str(), topic);
    return;
  }
  
  Serial.println("MSG NIHH: \n");
  serializeJson(doc, Serial);
  Serial.println();
  
  auto parseMode = [&]() -> bool {
    if(!doc.containsKey("mode")) return false;
    
    const char* m = doc["mode"];

    if(!m) return false;
    if(!strcasecmp(m, "AUTO")) fanMode = FanMode::AUTO;
    else if(!strcasecmp(m, "MANUAL")) fanMode = FanMode::MANUAL;
    else if(!strcasecmp(m, "FORCE_ON")) fanMode = FanMode::FORCE_ON;
    else if(!strcasecmp(m, "FORCE_OFF")) fanMode = FanMode::FORCE_OFF;
    
    Serial.printf("[MQTT] mode=%s\n", m);
    needPublishState = true;
    stateRev++;
    return true;
  };

  if(t == TOPIC_CONTROL_MODE) {
    parseMode();
    return;
  }
  
  if(t == TOPIC_FAN) {
    bool modeChanged = parseMode();

    if(fanMode != FanMode::MANUAL) {
      Serial.println("[MQTT] ignore fan[]: mode is not MANUAL");
      return;
    }

    if(!doc.containsKey("fan") || !doc["fan"].is<JsonArray>()) {
      Serial.println("[MQTT] fan[] is required");
      return;
    }
    JsonArray a = doc["fan"].as<JsonArray>();

    if(a.size() != 6) {
      Serial.println("[MQTT] fan[] must be 6 elements");
      return;
    }

    uint8_t tmp[6];
    for(size_t i = 0; i < 6; ++i) {
      int v = a[i].is<int>() ? a[i].as<int>() : 0;
      tmp[i] = (v!=0) ? 1 : 0;
    }

    applyFanArray(tmp);
    Serial.println("[MQTT] applied fan[] (MANUAL)");
    return;
  }

  if(t == TOPIC_LAMP) {
    int v = 1;

    if(doc.containsKey("lamp")) {
      if(doc["lamp"].is<bool>()) v = doc["lamp"].as<bool>() ? 1 : 0;
      else if(doc["lamp"].is<int>()) v = doc["lamp"].as<int>() ? 1 : 0;
    }

    if(v == 0) applyLamp(false);
    else if(v == 1) applyLamp(true);
    else Serial.println("[MQTT] lamp payload invalid");
    return;
  }
}

void ensureMqtt() {
  if(MqttClient.connected()) return;
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(clientId, sizeof(clientId),
    "esp32-incubator-%02X%02X%02X%02X%02X%02X",
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
  );
  
  MqttClient.setCallback(mqttCallback);
  MqttClient.setBufferSize(MQTT_BUFFER);

  Serial.print("[MQTT] connecting...");

  bool ok = MqttClient.connect(
    clientId,
    mqtt_username,
    mqtt_password,
    TOPIC_STATUS,
    0,
    true,
    "offline"
  );

  if(!ok) {
    Serial.printf("failed rd=%d\n", MqttClient.state());
    return;
  }
  Serial.println("Connected");

  MqttClient.publish(TOPIC_STATUS, "online", true);

  // subscribe
  MqttClient.publish(TOPIC_STATUS, "online", true);
  MqttClient.subscribe(TOPIC_CONTROL_MODE, 1);
  MqttClient.subscribe(TOPIC_FAN, 1);
  MqttClient.subscribe(TOPIC_LAMP, 1);

  needPublishState = true;
}

void controlLoop() {
  float t = !isnan(t_ds_c) ? t_ds_c : t_dht_c;
  float h = rh_dht;

  t_main = ema(t_main, t, cfg.ema_alpha);

  if(fanMode == FanMode::AUTO) {
    bool wantOn = false;

    // combined hysteresis
    if(!isnan(t_main) && (t_main >= cfg.temp_on_c)) wantOn = true;
    
    if(!isnan(h) && (h >= cfg.rh_on_pct)) wantOn = true;

    if(!isnan(t_main) && (t_main <= cfg.temp_off_c) && !isnan(h) && (h <= cfg.rh_off_pct)) wantOn = false;

    bool currentlyOn = (fanArr[0] || fanArr[1] || fanArr[2] || fanArr[3] || fanArr[4] || fanArr[5]) ? true : false;

    if(wantOn != currentlyOn) {
      applyFanGroup(wantOn);
      Serial.printf("[FAN] mode = AUTO -> %s\n", wantOn ? "ON" : "OFF");
    }
  }
  else if (fanMode == FanMode::FORCE_ON) {
    // safety: paksa semua ON
    bool allOn = (fanArr[0] & fanArr[1] & fanArr[2] & fanArr[3] & fanArr[4] & fanArr[5]) ? true  : false;
    if (!allOn) applyFanGroup(true);
  } else if (fanMode == FanMode::FORCE_OFF) {
    bool anyOn = (fanArr[0] || fanArr[1] || fanArr[2] || fanArr[3] || fanArr[4] || fanArr[5]) ? true : false;
    if (anyOn) applyFanGroup(false);
  }
}

// TELEMETRY PRINTING

// ---------- Pretty printer helpers ----------
const char* modeName(FanMode m) {
  switch (m) {
    case FanMode::AUTO: return "AUTO";
    case FanMode::MANUAL: return "MANUAL";
    case FanMode::FORCE_ON: return "FORCE_ON";
    case FanMode::FORCE_OFF: return "FORCE_OFF";
  }
  return "?";
}

void printDivider() {
  Serial.println(F("----------------------------------------"));
}

void printLineF(const char* label, float v, const char* unit = nullptr, uint8_t dp = 2) {
  Serial.print(label); Serial.print(F(": "));
  if (isnan(v)) {
    Serial.println(F("N/A"));
  } else {
    Serial.print(v, dp);
    if (unit) { Serial.print(' '); Serial.print(unit); }
    Serial.println();
  }
}

void printLineS(const char* label, const char* v) {
  Serial.print(label); Serial.print(F(": "));
  Serial.println(v ? v : "-");
}

void printLineI(const char* label, int v) {
  Serial.print(label); Serial.print(F(": "));
  Serial.println(v);
}

// ---------- BUILD JSON TELEMETRY ----------
String buildTelemetryJson() {
  char buf[MQTT_BUFFER];
  const unsigned long ts = millis()/1000;
  const char* modeStr = (fanMode==FanMode::AUTO)?"AUTO":
                        (fanMode==FanMode::MANUAL)?"MANUAL":
                        (fanMode==FanMode::FORCE_ON)?"FORCE_ON":"FORCE_OFF";

  bool fix = gps.location.isValid();
  int  sat = gps.satellites.isValid() ? gps.satellites.value() : -1;
  double lat = fix ? gps.location.lat() : NAN;
  double lon = fix ? gps.location.lng() : NAN;

  auto fOrNull = [](float v, char* out, int dp){
    if (isnan(v)) strcpy(out,"null"); else dtostrf(v, 0, dp, out);
  };
  char tds[16], tdht[16], tmain[16], rhv[16], lats[24], lons[24], fanJson[32], lampJson[8];
  fOrNull(t_ds_c,   tds,   2);
  fOrNull(t_dht_c,  tdht,  2);
  fOrNull(t_main,   tmain, 2);
  fOrNull(rh_dht,   rhv,   1);
  if (isnan(lat)) strcpy(lats,"null"); else dtostrf(lat, 0, 6, lats);
  if (isnan(lon)) strcpy(lons,"null"); else dtostrf(lon, 0, 6, lons);

  buildFanArrayJson(fanJson);
  buildLampArrayJson(lampJson);

  snprintf(buf, sizeof(buf),
    "{\"v\":1,\"ts\":%lu,"
    "\"t\":{\"ds\":%s,\"dht\":%s,\"main\":%s},"
    "\"rh\":%s,"
    "\"fan\":%s,"
    "\"mode\":\"%s\","
    "\"lamp\":%s,"
    "\"gps\":{\"fix\":%s,\"sat\":%d,\"lat\":%s,\"lon\":%s},"
    "\"rev\":%lu,"
    "\"fw\":\"main-mvp-001\"}",
    ts, tds, tdht, tmain, rhv, fanJson, modeStr, lampJson,
    fix?"true":"false", sat, lats, lons, stateRev
  );
  return String(buf);
}


// ---------- PUBLISH HELPERS ----------
void publishStateNow() {
  char buf[MQTT_BUFFER], fanJson[32],  lampJson[8];

  buildFanArrayJson(fanJson);
  buildLampArrayJson(lampJson);

  const char* modeStr = (fanMode == FanMode::AUTO) ? "AUTO" :
    (fanMode == FanMode::MANUAL) ? "MANUAL" :
    (fanMode == FanMode::FORCE_ON) ? "FORCE_ON" : "FORCE_OFF";

  snprintf(buf, sizeof(buf),
    "{\"v\":1,\"rev\":%lu,\"mode\":\"%s\",\"fan\":%s,\"lamp\":%s}",
    stateRev, modeStr, fanJson, lampJson
  );

  MqttClient.publish(TOPIC_STATE, buf, true);
  needPublishState = false;
}

void publishTelemetryNow() {
  String j = buildTelemetryJson();
  MqttClient.publish(TOPIC_TELEMETRY, j.c_str()); // non-retained
}


void printTelemetry() {
  // Baca GPS jika ada
  bool gpsFix = gps.location.isValid();
  double lat = gpsFix ? gps.location.lat() : NAN;
  double lon = gpsFix ? gps.location.lng() : NAN;
  bool satValid = gps.satellites.isValid();
  int   sat = satValid ? gps.satellites.value() : -1;

  printDivider();
  Serial.println(F("INKUBATOR TELEMETRY"));
  printDivider();

  // Suhu & kelembapan
  printLineF("temp_ds",   t_ds_c,  "°C");
  printLineF("temp_dht",  t_dht_c, "°C");
  printLineF("room_humid", rh_dht, "%");
  printLineF("temp_main", t_main, "°C");

  // Status aktuator
  bool anyOn = (fanArr[0] || fanArr[1] || fanArr[2] || fanArr[3] || fanArr[4] || fanArr[5]);
  printLineS("fan",  anyOn ? "ON" : "OFF");
  printLineS("mode", modeName(fanMode));
  printLineS("lamp", lampState ? "ON" : "OFF");

  // GPS
  if (gpsFix) {
    Serial.print(F("gps_lat: ")); Serial.println(lat, 6);
    Serial.print(F("gps_lon: ")); Serial.println(lon, 6);
  } else {
    Serial.println(F("gps_lat: N/A"));
    Serial.println(F("gps_lon: N/A"));
  }
  printLineI("gps_sat", satValid ? sat : -1);

  printDivider();
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

  // WiFi
  espSecureClient.setCACert(root_ca);
  ensureWifi();
  

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

  // MQTT Setup
  MqttClient.setServer(mqtt_server, mqtt_port);
  ensureMqtt();
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
    publishTelemetryNow();
    tNextGpsPrint = tNow + GPS_PRINT_MS;
  }

  // ---- Serial command ----
  handleSerial();

  // Mqtt
  if(!MqttClient.connected()) {
    while(!MqttClient.connected()) {
      Serial.println("[MQTT] Trying to reconnect MQTT");
      ensureMqtt();
    }
  }
  MqttClient.loop();

  if (needPublishState && MqttClient.connected()) {
    publishStateNow();
  }
}
