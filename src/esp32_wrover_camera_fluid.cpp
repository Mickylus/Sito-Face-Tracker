/*
#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ─── CONFIGURAZIONE ───────────────────────────────────────────
const char* ssid     = "espcam_SVC";
const char* password = "SVCCAM32";
// ──────────────────────────────────────────────────────────────

// Pin ESP32-Wrover Kit
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

#define PART_BOUNDARY "fb"

static const char* STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY =
  "\r\n--" PART_BOUNDARY "\r\n"
  "Content-Type: image/jpeg\r\n"
  "Content-Length: ";

httpd_handle_t stream_httpd = NULL;

// ─── STREAM HANDLER ───────────────────────────────────────────
static esp_err_t stream_handler(httpd_req_t* req) {
  camera_fb_t* fb = NULL;
  esp_err_t res;
  char len_buf[16];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame fallito");
      continue;  // non interrompere, riprova subito
    }

    // Boundary + header lunghezza in un unico chunk
    size_t llen = snprintf(len_buf, sizeof(len_buf), "%u\r\n\r\n", fb->len);
    if (httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)) != ESP_OK ||
        httpd_resp_send_chunk(req, len_buf, llen) != ESP_OK ||
        httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len) != ESP_OK) {
      esp_camera_fb_return(fb);
      break;
    }

    esp_camera_fb_return(fb);
  }

  return ESP_OK;
}

// ─── CAPTURE HANDLER ──────────────────────────────────────────
static esp_err_t capture_handler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { httpd_resp_send_500(req); return ESP_FAIL; }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, (const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return ESP_OK;
}

// ─── AVVIO SERVER ─────────────────────────────────────────────
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port      = 80;
  config.ctrl_port        = 32768;
  config.max_uri_handlers = 4;
  // Stack più grande per gestire frame pesanti senza crash
  config.stack_size       = 8192;

  httpd_uri_t stream_uri  = { "/stream",  HTTP_GET, stream_handler,  NULL };
  httpd_uri_t capture_uri = { "/capture", HTTP_GET, capture_handler, NULL };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &capture_uri);
  }
}

// ─── SETUP ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);  // disabilita log interni, libera CPU

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  // Clock alto = più FPS
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Con PSRAM (il Wrover ce l'ha): 3 frame buffer in DMA
  // Questo è il segreto per eliminare i micro-freeze:
  // mentre uno viene inviato, il sensore riempie gli altri due
  config.frame_size   = FRAMESIZE_VGA;   // 640x480 — miglior bilancio FPS/qualità
  config.jpeg_quality = 8;               // 4=max qualità, 63=min — 8 è ottimo
  config.fb_count     = 3;              // 3 buffer con PSRAM
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST; // scarta frame vecchi, prendi sempre l'ultimo

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Errore camera: 0x%x\n", err);
    return;
  }

  // ─── Ottimizzazioni sensore OV2640 ─────────────────────────
  sensor_t* s = esp_camera_sensor_get();

  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 8);

  // Immagine
  s->set_brightness(s, 1);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  s->set_sharpness(s, 2);
  s->set_denoise(s, 1);

  // Esposizione automatica
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_ae_level(s, 0);
  s->set_aec_value(s, 300);

  // Bilanciamento del bianco automatico
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, 0);

  // Guadagno automatico
  s->set_gain_ctrl(s, 1);
  s->set_agc_gain(s, 0);
  s->set_gainceiling(s, (gainceiling_t)6);

  // Disabilita effetti inutili
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);

  // ─── Wi-Fi ──────────────────────────────────────────────────
  WiFi.mode(WIFI_STA);

  // Potenza TX massima per connessione stabile
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  WiFi.begin(ssid, password);
  Serial.print("Connessione");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\n─────────────────────────────");
  Serial.println("Connesso!");
  Serial.print("IP:       http://");
  Serial.println(WiFi.localIP());
  Serial.print("Stream:   http://");
  Serial.print(WiFi.localIP());
  Serial.println("/stream");
  Serial.print("Snapshot: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/capture");
  Serial.println("─────────────────────────────");

  startCameraServer();
}

// ─── LOOP ─────────────────────────────────────────────────────
void loop() {
  // Niente nel loop — tutto gestito dall'HTTP server su task separato
  // delay alto per non sprecare CPU
  delay(10000);
}
*/