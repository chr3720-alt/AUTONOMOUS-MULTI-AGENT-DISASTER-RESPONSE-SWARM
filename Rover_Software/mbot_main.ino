/*
 * ================================================================
 *  M·BOT Firmware — ESP32 Dev Module  v2
 *
 *  Motors:   Left pair  → L298N CH-A  (IN1/IN2/ENA)
 *            Right pair → L298N CH-B  (IN3/IN4/ENB)
 *  LiDAR:    TF Mini → UART2  GPIO16(RX) GPIO17(TX)
 *  CAM:      ESP32-CAM → UART1 GPIO4(RX) GPIO5(TX)
 *            CAM sends "CAM:{json}\n" status frames every ~2s
 *  Power:    7.4V 2S2P LiPo
 *
 *  Libraries needed:
 *    ESPAsyncWebServer  (me-no-dev)
 *    AsyncTCP           (me-no-dev)
 *    ArduinoJson        (Benoit Blanchon) v6+
 *
 *  Board: ESP32 Dev Module, 240MHz, 4MB Default
 *
 *  NEW in v2:
 *   - Parses CAM:{json}\n UART frames from ESP32-CAM
 *   - Forwards cam_alive / cam_ip / cam_fps / cam_rssi
 *     in every telemetry packet to the dashboard
 *   - /status HTTP includes cam fields
 * ================================================================
 */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include "driver/gpio.h"

struct Motors { int L; int R; };

// ── Kill ENA/ENB before setup() runs ─────────────────────────
__attribute__((constructor)) static void killAtBoot() {
  gpio_reset_pin(GPIO_NUM_32);
  gpio_set_direction(GPIO_NUM_32, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_32, 0);
  gpio_reset_pin(GPIO_NUM_33);
  gpio_set_direction(GPIO_NUM_33, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_33, 0);
}

// ── WiFi AP ───────────────────────────────────────────────────
#define AP_SSID  "Robo"
#define AP_PASS  "robo1234"

// ── L298N ─────────────────────────────────────────────────────
#define IN1  25
#define IN2  26
#define IN3  27
#define IN4  14
#define ENA  32
#define ENB  33
#define PWM_CH_L  0
#define PWM_CH_R  1
#define PWM_FREQ  1000
#define PWM_BITS  8

// ── UART ──────────────────────────────────────────────────────
#define LIDAR_RX  16
#define LIDAR_TX  17
#define CAM_RX     4    // receives CAM:{json}\n from ESP32-CAM U0TX
#define CAM_TX     5    // not currently used by CAM side

// ── Tuning ────────────────────────────────────────────────────
#define PWM_MAX      200
#define PWM_HALF     110
#define PWM_TURN     150
#define STOP_CM       30
#define SLOW_CM       80
#define MIN_STR      100
#define LIDAR_STALE  500
#define DEAD_B        5.0f
#define DEAD_G        5.0f
#define THROTTLE_MAX 45.0f
#define STEER_MAX    45.0f
#define ROTATE_MS    800

// ── CAM state (populated from UART bridge) ────────────────────
#define CAM_STALE_MS  6000   // declare cam offline if no frame for 6s
struct CamStatus {
  bool     alive    = false;
  char     ip[20]   = "";
  float    fps      = 0.0f;
  int      rssi     = 0;
  uint32_t lastMs   = 0;
};
CamStatus camStatus;

// ── System state ──────────────────────────────────────────────
enum Mode { MANUAL, AUTO };
Mode mode = MANUAL;

Motors       motorCmd    = {0, 0};
bool         cmdFresh    = false;
portMUX_TYPE motorMux    = portMUX_INITIALIZER_UNLOCKED;

volatile uint16_t lidarDist   = 9999;
volatile uint16_t lidarStr    = 0;
volatile uint32_t lidarLastMs = 0;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
uint32_t lastClientMs  = 0;
bool     wsConnected   = false;  // true only while ≥1 client is active
bool     motorsStopped = true;   // debounce — avoids hammering stopMotors()

// ================================================================
//  MOTOR ARBITER
// ================================================================
void setMotors(int L, int R) {
  L = constrain(L, -PWM_MAX, PWM_MAX);
  R = constrain(R, -PWM_MAX, PWM_MAX);
  digitalWrite(IN1, L >= 0 ? HIGH : LOW);
  digitalWrite(IN2, L >= 0 ? LOW  : HIGH);
  ledcWrite(PWM_CH_L, abs(L));
  digitalWrite(IN3, R >= 0 ? HIGH : LOW);
  digitalWrite(IN4, R >= 0 ? LOW  : HIGH);
  ledcWrite(PWM_CH_R, abs(R));
  motorsStopped = false;
}

void stopMotors() {
  if (motorsStopped) return;  // debounce — don't hammer hardware
  ledcWrite(PWM_CH_L, 0); ledcWrite(PWM_CH_R, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  portENTER_CRITICAL(&motorMux);
  motorCmd = {0, 0};
  portEXIT_CRITICAL(&motorMux);
  motorsStopped = true;
}

// ================================================================
//  MANUAL MODE
// ================================================================
Motors accelToMotors(float b, float g) {
  if (fabsf(b) < DEAD_B) return {0, 0};
  if (fabsf(g) < DEAD_G)  g = 0;

  float throttle = constrain(b / THROTTLE_MAX, -1.0f, 1.0f) * (float)PWM_MAX;
  float steer    = constrain(g / STEER_MAX,    -1.0f, 1.0f);
  float L = throttle + steer * (float)PWM_MAX * 0.6f;
  float R = throttle - steer * (float)PWM_MAX * 0.6f;
  float mx = max(fabsf(L), fabsf(R));
  if (mx > (float)PWM_MAX) { L = L/mx*(float)PWM_MAX; R = R/mx*(float)PWM_MAX; }
  return { (int)L, (int)R };
}

// ================================================================
//  AUTO MODE — LiDAR FSM
// ================================================================
Motors autoDecide(uint16_t dist, uint16_t str) {
  static uint32_t rotateStart = 0;
  static bool     rotating    = false;
  uint32_t now = millis();
  if (now - lidarLastMs > LIDAR_STALE) { rotating = false; return {0, 0}; }
  if (str < MIN_STR)   { rotating = false; return {PWM_HALF, PWM_HALF}; }
  if (dist <= STOP_CM) {
    if (!rotating) { rotateStart = now; rotating = true; }
    if (now - rotateStart < ROTATE_MS) return {-PWM_TURN, PWM_TURN};
    rotating = false;
    return {PWM_HALF, PWM_HALF};
  }
  rotating = false;
  if (dist < SLOW_CM) return {PWM_HALF, PWM_HALF};
  return {PWM_MAX, PWM_MAX};
}

// ================================================================
//  TF MINI PARSER
// ================================================================
void parseLidar() {
  static uint8_t buf[9];
  static uint8_t idx = 0;
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    if (idx == 0 && b != 0x59) continue;
    if (idx == 1 && b != 0x59) { idx = 0; continue; }
    buf[idx++] = b;
    if (idx < 9) continue;
    idx = 0;
    uint16_t sum = 0;
    for (int i = 0; i < 8; i++) sum += buf[i];
    if ((sum & 0xFF) != buf[8]) continue;
    uint16_t d = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    uint16_t s = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
    if (d > 1200) continue;
    if (d == 0) d = 1;
    lidarDist = d; lidarStr = s; lidarLastMs = millis();
  }
}

// ================================================================
//  CAM UART BRIDGE PARSER
//  Reads lines from Serial1 looking for:  CAM:{...}\n
// ================================================================
void parseCamUart() {
  static char lineBuf[256];
  static uint8_t lineIdx = 0;

  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      lineBuf[lineIdx] = '\0';
      lineIdx = 0;
      // Check for CAM: prefix
      if (strncmp(lineBuf, "CAM:", 4) == 0) {
        const char* json = lineBuf + 4;
        StaticJsonDocument<192> doc;
        if (deserializeJson(doc, json) == DeserializationError::Ok) {
          camStatus.alive  = doc["cam_alive"] | false;
          camStatus.fps    = doc["cam_fps"]   | 0.0f;
          camStatus.rssi   = doc["cam_rssi"]  | 0;
          camStatus.lastMs = millis();
          const char* ip   = doc["cam_ip"]    | "";
          strncpy(camStatus.ip, ip, sizeof(camStatus.ip)-1);
          camStatus.ip[sizeof(camStatus.ip)-1] = '\0';
        }
      }
    } else {
      if (lineIdx < sizeof(lineBuf)-1) lineBuf[lineIdx++] = c;
    }
  }

  // Timeout — if no frame for CAM_STALE_MS, mark offline
  if (camStatus.alive && millis() - camStatus.lastMs > CAM_STALE_MS) {
    camStatus.alive = false;
    Serial.println("[CAM UART] Timed out — marking offline");
  }
}

// ================================================================
//  WEBSOCKET
// ================================================================
void onWsEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {

  if (type == WS_EVT_CONNECT) {
    lastClientMs = millis();
    wsConnected  = true;
    Serial.printf("[WS] #%u connected  total=%u\n", client->id(), srv->count());
    return;
  }
  if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] #%u disconnected  remaining=%u\n", client->id(), srv->count() - 1);
    // Only lock out motors if ALL clients gone
    if (srv->count() <= 1) {   // count still includes this client during callback
      wsConnected = false;
      stopMotors();
    }
    return;
  }
  if (type != WS_EVT_DATA) return;
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (!info->final || info->opcode != WS_TEXT || len > 200) return;

  StaticJsonDocument<192> doc;
  if (deserializeJson(doc, data, len) != DeserializationError::Ok) return;
  lastClientMs = millis();

  if (doc.containsKey("mode")) {
    Mode nm = strcmp(doc["mode"] | "MANUAL", "AUTO") == 0 ? AUTO : MANUAL;
    if (nm != mode) {
      mode = nm;
      stopMotors();
      Serial.printf("[MODE] → %s\n", mode == AUTO ? "AUTO" : "MANUAL");
    }
  }

  if (mode == MANUAL && doc.containsKey("b") && doc.containsKey("g")) {
    Motors m = accelToMotors(doc["b"] | 0.0f, doc["g"] | 0.0f);
    portENTER_CRITICAL(&motorMux);
    motorCmd = m; cmdFresh = true;
    portEXIT_CRITICAL(&motorMux);
  }
}

// ================================================================
//  TELEMETRY — now includes cam_ fields
// ================================================================
void broadcastTelemetry() {
  if (ws.count() == 0) return;

  portENTER_CRITICAL(&motorMux);
  Motors snap = motorCmd;
  portEXIT_CRITICAL(&motorMux);

  // Check cam stale here too
  bool camAlive = camStatus.alive && (millis() - camStatus.lastMs < CAM_STALE_MS);

  StaticJsonDocument<256> doc;
  doc["dist"]      = lidarDist;
  doc["str"]       = lidarStr;
  doc["lm"]        = snap.L;
  doc["rm"]        = snap.R;
  doc["mode"]      = mode == AUTO ? "AUTO" : "MANUAL";
  doc["cam_alive"] = camAlive;
  doc["cam_ip"]    = camStatus.ip;
  doc["cam_fps"]   = serialized(String(camStatus.fps, 1));
  doc["cam_rssi"]  = camStatus.rssi;

  char buf[256];
  size_t n = serializeJson(doc, buf);
  ws.textAll(buf, n);
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  digitalWrite(ENA, LOW); pinMode(ENA, OUTPUT); digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW); pinMode(ENB, OUTPUT); digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW); pinMode(IN1, OUTPUT);
  digitalWrite(IN2, LOW); pinMode(IN2, OUTPUT);
  digitalWrite(IN3, LOW); pinMode(IN3, OUTPUT);
  digitalWrite(IN4, LOW); pinMode(IN4, OUTPUT);

  Serial.begin(115200);
  delay(200);
  Serial.println("\n=========================");
  Serial.println("  M·BOT v2 booting...");
  Serial.println("=========================");

  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_BITS); ledcWrite(PWM_CH_L, 0);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_BITS); ledcWrite(PWM_CH_R, 0);
  ledcAttachPin(ENA, PWM_CH_L); ledcWrite(PWM_CH_L, 0);
  ledcAttachPin(ENB, PWM_CH_R); ledcWrite(PWM_CH_R, 0);
  Serial.println("[OK] Motors");

  Serial2.begin(115200, SERIAL_8N1, LIDAR_RX, LIDAR_TX);
  Serial.println("[OK] TF Mini UART2");

  // CAM UART1 — receive status JSON from ESP32-CAM U0
  Serial1.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);
  Serial.println("[OK] CAM UART1  RX=GPIO4");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(100);
  Serial.printf("[OK] AP \"%s\"  %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  if (MDNS.begin("robot")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("[OK] robot.local");
  }

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send(200, "text/plain",
      "M·BOT v2\n"
      "ws://192.168.4.1/ws\n"
      "http://192.168.4.1/status\n"
      "CAM stream: http://192.168.4.2/stream\n");
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* r) {
    bool camAlive = camStatus.alive && (millis() - camStatus.lastMs < CAM_STALE_MS);
    StaticJsonDocument<256> doc;
    doc["dist"]      = lidarDist;
    doc["str"]       = lidarStr;
    doc["mode"]      = mode == AUTO ? "AUTO" : "MANUAL";
    doc["clients"]   = ws.count();
    doc["cam_alive"] = camAlive;
    doc["cam_ip"]    = camStatus.ip;
    doc["cam_fps"]   = serialized(String(camStatus.fps, 1));
    doc["cam_rssi"]  = camStatus.rssi;
    char buf[256]; serializeJson(doc, buf);
    r->send(200, "application/json", buf);
  });

  server.begin();
  Serial.println("[OK] HTTP :80");
  Serial.println("=========================");
  Serial.println("  Ready — waiting for dashboard");
  Serial.println("=========================\n");
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  uint32_t now = millis();

  parseLidar();
  parseCamUart();

  if (!wsConnected) {
    stopMotors();
  } else if (mode == MANUAL) {
    portENTER_CRITICAL(&motorMux);
    bool   fresh = cmdFresh;
    Motors m     = motorCmd;
    cmdFresh = false;
    portEXIT_CRITICAL(&motorMux);
    if (fresh) setMotors(m.L, m.R);
  } else {
    Motors m = autoDecide(lidarDist, lidarStr);
    portENTER_CRITICAL(&motorMux);
    motorCmd = m;
    portEXIT_CRITICAL(&motorMux);
    setMotors(m.L, m.R);
  }

  // Watchdog: stop motors only if we've had a client before and ALL have been
  // gone for >5s (covers phone screen-lock / brief network hiccup).
  if (wsConnected && ws.count() == 0 && now - lastClientMs > 5000) {
    wsConnected = false;
    stopMotors();
    Serial.println("[WATCHDOG] All clients gone 5s — motors stopped");
  }

  static uint32_t lastTelem = 0;
  if (now - lastTelem >= 200) {
    lastTelem = now;
    broadcastTelemetry();
  }

  // cleanupClients evicts stale sockets — run infrequently to avoid
  // kicking clients that are momentarily slow (e.g. phone screen lock).
  static uint32_t lastCleanup = 0;
  if (now - lastCleanup >= 5000) {
    lastCleanup = now;
    ws.cleanupClients();
  }

  static uint32_t lastLog = 0;
  if (now - lastLog >= 1000) {
    lastLog = now;
    portENTER_CRITICAL(&motorMux);
    Motors snap = motorCmd;
    portEXIT_CRITICAL(&motorMux);
    bool camAlive = camStatus.alive && (millis() - camStatus.lastMs < CAM_STALE_MS);
    Serial.printf("[LOG] dist=%ucm str=%u L=%d R=%d mode=%s ws=%u cam=%s(%.1ffps)\n",
      lidarDist, lidarStr, snap.L, snap.R,
      mode == AUTO ? "AUTO" : "MANUAL",
      ws.count(),
      camAlive ? "OK" : "OFF",
      camStatus.fps);
  }
}
