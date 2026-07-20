/*
 * ================================================================
 *  M·BOT ESP32-CAM — MJPEG Stream Station  v2
 *
 *  Board:    AI Thinker ESP32-CAM
 *  Role:     WiFi STA → joins "Robo" AP → serves MJPEG stream
 *            UART0 → bridge to main ESP32 (status JSON frames)
 *
 *  Stream:   http://192.168.4.2/stream
 *  Snapshot: http://192.168.4.2/capture
 *  Config:   http://192.168.4.2/config
 *  Status:   http://192.168.4.2/status
 *  Ping:     http://192.168.4.2/ping      ← dashboard discovery
 *
 *  Board settings in Arduino IDE:
 *    Board:        AI Thinker ESP32-CAM
 *    Partition:    Huge APP (3MB No OTA)
 *    CPU Freq:     240MHz
 *    Flash Freq:   80MHz
 *    Flash Size:   4MB
 *
 *  Wiring:
 *    GPIO0 → GND to enter flash mode.
 *    Remove jumper after flashing.
 *    U0TXD (GPIO1) → main ESP32 UART1 RX (GPIO4)
 *    U0RXD (GPIO3) → main ESP32 UART1 TX (GPIO5)
 * ================================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "driver/ledc.h"
#include "sdkconfig.h"

// ── WiFi ──────────────────────────────────────────────────────
#define WIFI_SSID        "Robo"
#define WIFI_PASS        "robo1234"
#define WIFI_TIMEOUT_MS  15000

// ── Static IP — CAM always gets 192.168.4.2 ──────────────────
// Gateway = main ESP32 AP address (192.168.4.1)
// Change only if you remap the AP subnet.
static const IPAddress CAM_IP     (192, 168, 4,   2);
static const IPAddress CAM_GW     (192, 168, 4,   1);
static const IPAddress CAM_SUBNET (255, 255, 255, 0);
static const IPAddress CAM_DNS    (192, 168, 4,   1);  // point DNS at gateway

// ── AI Thinker ESP32-CAM pin map ──────────────────────────────
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ── Flash LED ─────────────────────────────────────────────────
#define LED_GPIO  4
#define LED_ON()  digitalWrite(LED_GPIO, HIGH)
#define LED_OFF() digitalWrite(LED_GPIO, LOW)

// ── Stream defaults ───────────────────────────────────────────
#define DEFAULT_FRAMESIZE   FRAMESIZE_VGA
#define DEFAULT_QUALITY     12
#define STREAM_BOUNDARY     "frame"
#define UART_REPORT_MS      2000   // send status over UART every 2s

static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY;
static const char* STREAM_PART_HDR =
  "\r\n--" STREAM_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ── HTTP server ───────────────────────────────────────────────
WebServer server(80);

// ── Globals ───────────────────────────────────────────────────
static uint32_t streamClients  = 0;
static uint32_t totalFrames    = 0;
static float    currentFps     = 0.0f;
static uint32_t lastUartReport = 0;

// ================================================================
//  CORS helper — every response gets these headers
// ================================================================
void addCORS() {
  server.sendHeader("Access-Control-Allow-Origin",  "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// ================================================================
//  CAMERA INIT
// ================================================================
bool initCamera() {
  camera_config_t cfg = {};

  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = Y2_GPIO_NUM;
  cfg.pin_d1       = Y3_GPIO_NUM;
  cfg.pin_d2       = Y4_GPIO_NUM;
  cfg.pin_d3       = Y5_GPIO_NUM;
  cfg.pin_d4       = Y6_GPIO_NUM;
  cfg.pin_d5       = Y7_GPIO_NUM;
  cfg.pin_d6       = Y8_GPIO_NUM;
  cfg.pin_d7       = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;

  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    cfg.frame_size   = DEFAULT_FRAMESIZE;
    cfg.jpeg_quality = DEFAULT_QUALITY;
    cfg.fb_count     = 2;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    Serial.println("[CAM] PSRAM found — dual buffer");
  } else {
    cfg.frame_size   = FRAMESIZE_QVGA;
    cfg.jpeg_quality = 20;
    cfg.fb_count     = 1;
    cfg.fb_location  = CAMERA_FB_IN_DRAM;
    Serial.println("[CAM] No PSRAM — QVGA fallback");
  }

  if (esp_camera_init(&cfg) != ESP_OK) return false;

  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s,    DEFAULT_FRAMESIZE);
  s->set_quality(s,      DEFAULT_QUALITY);
  s->set_brightness(s,   1);
  s->set_saturation(s,   0);
  s->set_gainceiling(s,  GAINCEILING_4X);
  s->set_whitebal(s,     1);
  s->set_awb_gain(s,     1);
  s->set_exposure_ctrl(s,1);
  s->set_aec2(s,         1);
  s->set_ae_level(s,     0);
  s->set_aec_value(s,    300);
  s->set_gain_ctrl(s,    1);
  s->set_agc_gain(s,     0);
  s->set_bpc(s,          0);
  s->set_wpc(s,          1);
  s->set_raw_gma(s,      1);
  s->set_lenc(s,         1);
  s->set_hmirror(s,      0);
  s->set_vflip(s,        0);
  s->set_colorbar(s,     0);
  return true;
}

// ================================================================
//  MJPEG STREAM
// ================================================================
void handleStream() {
  server.client().setTimeout(0);
  WiFiClient client = server.client();

  String hdr = "HTTP/1.1 200 OK\r\nContent-Type: ";
  hdr += STREAM_CONTENT_TYPE;
  hdr += "\r\nAccess-Control-Allow-Origin: *\r\nCache-Control: no-cache\r\n\r\n";
  client.print(hdr);

  Serial.printf("[STREAM] +client  %s\n", client.remoteIP().toString().c_str());
  streamClients++;

  uint32_t frames = 0, t0 = millis();
  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { delay(5); continue; }

    char ph[80];
    snprintf(ph, sizeof(ph), STREAM_PART_HDR, fb->len);
    size_t w = client.print(ph);
    if (w) w = client.write(fb->buf, fb->len);
    esp_camera_fb_return(fb);

    if (!w) break;
    frames++;
    totalFrames++;

    uint32_t el = millis() - t0;
    if (el >= 3000) {
      currentFps = (float)frames / (el / 1000.0f);
      Serial.printf("[STREAM] %.1f FPS\n", currentFps);
      frames = 0; t0 = millis();
    }
  }

  streamClients--;
  Serial.printf("[STREAM] -client  active=%u\n", streamClients);
}

// ================================================================
//  SNAPSHOT
// ================================================================
void handleCapture() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "Camera error"); return; }
  addCORS();
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

// ================================================================
//  CONFIG  GET /config?res=VGA&q=12&flip=0&bright=1
// ================================================================
void handleConfig() {
  sensor_t* s = esp_camera_sensor_get();
  bool changed = false;

  if (server.hasArg("res")) {
    const String& r = server.arg("res");
    framesize_t fs = FRAMESIZE_VGA;
    if      (r=="QQVGA") fs=FRAMESIZE_QQVGA;
    else if (r=="QVGA")  fs=FRAMESIZE_QVGA;
    else if (r=="CIF")   fs=FRAMESIZE_CIF;
    else if (r=="SVGA")  fs=FRAMESIZE_SVGA;
    else if (r=="XGA")   fs=FRAMESIZE_XGA;
    else if (r=="SXGA")  fs=FRAMESIZE_SXGA;
    else if (r=="UXGA")  fs=FRAMESIZE_UXGA;
    s->set_framesize(s, fs); changed = true;
  }
  if (server.hasArg("q")) {
    s->set_quality(s, constrain(server.arg("q").toInt(), 10, 63)); changed=true;
  }
  if (server.hasArg("flip")) {
    int v = server.arg("flip").toInt();
    s->set_vflip(s, v & 1); s->set_hmirror(s, (v>>1)&1); changed=true;
  }
  if (server.hasArg("bright")) {
    s->set_brightness(s, constrain(server.arg("bright").toInt(), -2, 2)); changed=true;
  }

  addCORS();
  String resp = "{\"ok\":"; resp += changed?"true":"false";
  resp += ",\"ip\":\""; resp += WiFi.localIP().toString(); resp += "\"}";
  server.send(200, "application/json", resp);
}

// ================================================================
//  PING — ultra-light discovery endpoint for dashboard
//  Returns 200 + JSON immediately so dashboard can confirm CAM is live
// ================================================================
void handlePing() {
  addCORS();
  String resp = "{\"alive\":true,\"ip\":\"";
  resp += WiFi.localIP().toString();
  resp += "\",\"stream\":\"http://";
  resp += WiFi.localIP().toString();
  resp += "/stream\"}";
  server.send(200, "application/json", resp);
}

// ================================================================
//  STATUS
// ================================================================
void handleStatus() {
  sensor_t* s = esp_camera_sensor_get();
  addCORS();
  String r = "{";
  r += "\"alive\":true,";
  r += "\"ip\":\""      + WiFi.localIP().toString() + "\",";
  r += "\"rssi\":"      + String(WiFi.RSSI())       + ",";
  r += "\"fps\":"       + String(currentFps, 1)     + ",";
  r += "\"clients\":"   + String(streamClients)     + ",";
  r += "\"frames\":"    + String(totalFrames)        + ",";
  r += "\"stream\":\"http://"  + WiFi.localIP().toString() + "/stream\",";
  r += "\"capture\":\"http://" + WiFi.localIP().toString() + "/capture\"";
  if (s) {
    r += ",\"framesize\":"  + String(s->status.framesize);
    r += ",\"quality\":"    + String(s->status.quality);
    r += ",\"vflip\":"      + String(s->status.vflip);
    r += ",\"hmirror\":"    + String(s->status.hmirror);
    r += ",\"brightness\":" + String(s->status.brightness);
  }
  r += "}";
  server.send(200, "application/json", r);
}

// ================================================================
//  UART BRIDGE — report CAM status to main ESP32 every 2s
//  Main ESP32 forwards this to dashboard via WebSocket telemetry
//  Frame format: CAM:{json}\n
// ================================================================
void reportOverUart() {
  String r = "CAM:{";
  r += "\"cam_alive\":true,";
  r += "\"cam_ip\":\"" + WiFi.localIP().toString() + "\",";
  r += "\"cam_fps\":"   + String(currentFps, 1)     + ",";
  r += "\"cam_rssi\":"  + String(WiFi.RSSI());
  r += "}\n";
  Serial.print(r);   // GPIO1/GPIO3 = U0 → wired to main ESP32 UART1
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  // Note: Serial (U0) is also our UART bridge to main ESP32.
  // Use 115200 on both ends.
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=========================");
  Serial.println("  M-BOT CAM v2 booting");
  Serial.println("=========================");

  pinMode(LED_GPIO, OUTPUT);
  LED_OFF();
  for (int i = 0; i < 3; i++) { LED_ON(); delay(80); LED_OFF(); delay(80); }

  if (!initCamera()) {
    Serial.println("[FATAL] Camera init failed — reboot in 5s");
    delay(5000);
    ESP.restart();
  }
  Serial.println("[OK] Camera");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.config(CAM_IP, CAM_GW, CAM_SUBNET, CAM_DNS);  // static IP
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("[WiFi] Connecting to \"%s\" (static %s)", WIFI_SSID, CAM_IP.toString().c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > WIFI_TIMEOUT_MS) {
      Serial.println("\n[WiFi] Timeout — reboot");
      ESP.restart();
    }
    delay(300); Serial.print(".");
    LED_ON(); delay(50); LED_OFF();
  }
  Serial.println();
  Serial.printf("[OK] WiFi  IP=%s  RSSI=%d dBm\n",
    WiFi.localIP().toString().c_str(), WiFi.RSSI());

  server.on("/stream",  HTTP_GET, handleStream);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/config",  HTTP_GET, handleConfig);
  server.on("/status",  HTTP_GET, handleStatus);
  server.on("/ping",    HTTP_GET, handlePing);
  server.on("/",        HTTP_GET, []() {
    server.sendHeader("Location", "/status"); server.send(302);
  });
  // Preflight for browsers
  server.onNotFound([]() {
    if (server.method() == HTTP_OPTIONS) { addCORS(); server.send(204); }
    else { server.send(404, "text/plain", "Not found"); }
  });

  server.begin();
  Serial.println("[OK] HTTP :80");
  Serial.println("=========================");
  Serial.printf("  Stream:  http://%s/stream\n",  WiFi.localIP().toString().c_str());
  Serial.printf("  Ping:    http://%s/ping\n",    WiFi.localIP().toString().c_str());
  Serial.printf("  Config:  http://%s/config\n",  WiFi.localIP().toString().c_str());
  Serial.println("=========================\n");

  LED_ON();
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  server.handleClient();

  // UART bridge report
  uint32_t now = millis();
  if (now - lastUartReport >= UART_REPORT_MS) {
    lastUartReport = now;
    reportOverUart();
  }

  // WiFi watchdog
  static uint32_t lastCheck = 0;
  if (now - lastCheck > 5000) {
    lastCheck = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] Lost — reconnecting");
      LED_OFF();
      WiFi.reconnect();
      uint32_t t0 = millis();
      while (WiFi.status() != WL_CONNECTED && millis()-t0 < 10000) {
        delay(300); LED_ON(); delay(50); LED_OFF();
      }
      if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] Back  IP=%s\n", WiFi.localIP().toString().c_str());
        LED_ON();
      } else {
        Serial.println("[WiFi] Reconnect failed — reboot");
        delay(1000); ESP.restart();
      }
    }
  }
}
