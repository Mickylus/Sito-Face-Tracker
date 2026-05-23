#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "img_converters.h"
#include <ESP32Servo.h>
#include "soc/rtc_wdt.h"

// ─── MODALITÀ WIFI ───────────────────────────────────────────
#define WIFI_AP_MODE true

const char* sta_ssid     = "espcam_SVC";
const char* sta_password = "SVCCAM32";

const char* ap_ssid     = "ESPCAM_SVC";
const char* ap_password = "12345678";
const IPAddress ap_ip(192, 168, 4, 1);
const IPAddress ap_gateway(192, 168, 4, 1);
const IPAddress ap_subnet(255, 255, 255, 0);

// ─── SERVO ───────────────────────────────────────────────────
#define PAN_SERVO_PIN  33
#define TILT_SERVO_PIN 32

Servo servoPan;
Servo servoTilt;

int panAngle  = 90;
int tiltAngle = 90;

String serialBuffer = "";

// ─── PIN CAMERA ──────────────────────────────────────────────
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  21
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    19
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM     5
#define Y2_GPIO_NUM     4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22

// ─── STREAM ──────────────────────────────────────────────────
#define PART_BOUNDARY "fb"

static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;

static const char* STREAM_BOUNDARY =
  "\r\n--" PART_BOUNDARY "\r\n"
  "Content-Type: image/jpeg\r\n"
  "Content-Length: ";

httpd_handle_t stream_httpd = NULL;


// ─── STREAM HANDLER ──────────────────────────────────────────
static esp_err_t stream_handler(httpd_req_t* req) {

  char len_buf[16];

  if (httpd_resp_set_type(req, STREAM_CONTENT_TYPE) != ESP_OK)
    return ESP_FAIL;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) continue;

    size_t llen = snprintf(len_buf, sizeof(len_buf), "%u\r\n\r\n", fb->len);

    bool ok =
      httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)) == ESP_OK &&
      httpd_resp_send_chunk(req, len_buf, llen)                            == ESP_OK &&
      httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len)            == ESP_OK;

    esp_camera_fb_return(fb);

    if (!ok) break;
  }

  return ESP_OK;
}


// ─── SNAPSHOT HANDLER ────────────────────────────────────────
static esp_err_t capture_handler(httpd_req_t* req) {

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);

  return ESP_OK;
}


// ─── SERVER ──────────────────────────────────────────────────
void startCameraServer() {

  httpd_config_t config  = HTTPD_DEFAULT_CONFIG();
  config.server_port     = 80;
  config.ctrl_port       = 32768;
  config.max_uri_handlers = 4;
  config.stack_size      = 4096;  // ← ridotto da 8192

  httpd_uri_t stream_uri  = { "/stream",  HTTP_GET, stream_handler,  NULL };
  httpd_uri_t capture_uri = { "/capture", HTTP_GET, capture_handler, NULL };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &capture_uri);
  }
}


// ─── WIFI ────────────────────────────────────────────────────
void initWiFi() {

#if WIFI_AP_MODE
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
  WiFi.softAP(ap_ssid, ap_password);

  Serial.printf("AP: %s  IP: %s\n", ap_ssid, WiFi.softAPIP().toString().c_str());
  Serial.printf("STREAM: http://%s/stream\n", WiFi.softAPIP().toString().c_str());

#else
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.begin(sta_ssid, sta_password);

  while (WiFi.status() != WL_CONNECTED) { delay(300); }

  Serial.printf("STA IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("STREAM: http://%s/stream\n", WiFi.localIP().toString().c_str());
#endif
}


// ─── SERIAL SERVO ────────────────────────────────────────────
void handleSerialServo() {

  while (Serial.available()) {

    char c = Serial.read();

    if (c == '\n') {

      int comma = serialBuffer.indexOf(',');

      if (comma > 0) {
        panAngle  = constrain(serialBuffer.substring(0, comma).toInt(),    0, 180);
        tiltAngle = constrain(serialBuffer.substring(comma + 1).toInt(),   0, 180);

        servoPan.write(panAngle);
        servoTilt.write(tiltAngle);
      }

      serialBuffer = "";

    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 32) serialBuffer = "";
    }
  }
}


// ─── SETUP ───────────────────────────────────────────────────
void setup() {

  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // 1. Camera
  camera_config_t config;
  config.ledc_channel  = LEDC_CHANNEL_0;
  config.ledc_timer    = LEDC_TIMER_0;
  config.pin_d0        = Y2_GPIO_NUM;
  config.pin_d1        = Y3_GPIO_NUM;
  config.pin_d2        = Y4_GPIO_NUM;
  config.pin_d3        = Y5_GPIO_NUM;
  config.pin_d4        = Y6_GPIO_NUM;
  config.pin_d5        = Y7_GPIO_NUM;
  config.pin_d6        = Y8_GPIO_NUM;
  config.pin_d7        = Y9_GPIO_NUM;
  config.pin_xclk      = XCLK_GPIO_NUM;
  config.pin_pclk      = PCLK_GPIO_NUM;
  config.pin_vsync     = VSYNC_GPIO_NUM;
  config.pin_href      = HREF_GPIO_NUM;
  config.pin_sscb_sda  = SIOD_GPIO_NUM;
  config.pin_sscb_scl  = SIOC_GPIO_NUM;
  config.pin_pwdn      = PWDN_GPIO_NUM;
  config.pin_reset     = RESET_GPIO_NUM;
  config.xclk_freq_hz  = 20000000;
  config.pixel_format  = PIXFORMAT_JPEG;
  config.frame_size    = FRAMESIZE_QVGA;   // 320x240
  config.jpeg_quality  = 15;               // ← alzato (meno dati, più FPS)
  config.fb_count      = 2;               // ← ridotto da 3
  config.fb_location   = CAMERA_FB_IN_PSRAM;
  config.grab_mode     = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Errore camera");
    return;
  }
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  // Sensor tuning minimale
  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s,    FRAMESIZE_QVGA);
  s->set_quality(s,      15);
  s->set_brightness(s,   0);   // ← neutro, meno elaborazione
  s->set_contrast(s,     0);
  s->set_saturation(s,   0);
  s->set_sharpness(s,    0);   // ← disabilitato, carico CPU
  s->set_denoise(s,      0);   // ← disabilitato, carico CPU
  s->set_exposure_ctrl(s,1);
  s->set_aec2(s,         0);   // ← disabilitato AEC2, più leggero
  s->set_ae_level(s,     0);
  s->set_aec_value(s,    300);
  s->set_whitebal(s,     1);
  s->set_awb_gain(s,     1);
  s->set_gain_ctrl(s,    1);
  s->set_hmirror(s,      0);
  s->set_vflip(s,        0);

  // 2. WiFi + server
  initWiFi();
  startCameraServer();

  // 3. Servo
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servoPan.setPeriodHertz(50);
  servoTilt.setPeriodHertz(50);
  servoPan.attach(PAN_SERVO_PIN,  500, 2400);
  servoTilt.attach(TILT_SERVO_PIN, 500, 2400);
  servoPan.write(panAngle);
  servoTilt.write(tiltAngle);

  Serial.println("Sistema pronto");
}


// ─── LOOP ────────────────────────────────────────────────────
void loop() {
  handleSerialServo();
  delay(5);  // ← leggermente aumentato per cedere CPU allo stack WiFi
}