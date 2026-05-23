/*
#include <Arduino.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ─── CONFIGURAZIONE SERIAL VIDEO ─────────────────────────────
const int SERIAL_BAUD = 2000000; // aumentare se possibile (verifica supporto USB-serial)

// Pin ESP32-Wrover Kit (stessi usati nel progetto originale)
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

// Marker di inizio frame (5 byte), poi 4 byte lunghezza, poi 8 byte timestamp (microsec)
static const char FRAME_MAGIC[] = "FRAME";

// ─── INIZIALIZZA SERVER CAMERA (ma invia su Serial) ──────────
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(100);
  Serial.setDebugOutput(false);
  Serial.println("Starting camera -> Serial streamer");

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

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // riceviamo JPEG già compresso

  // Con PSRAM (Wrover): più frame buffer per evitare micro-freeze
  // Riduci risoluzione e qualità per aumentare FPS
  config.frame_size   = FRAMESIZE_QVGA; // 320x240
  config.jpeg_quality = 12; // valori maggiori = più compressione (4..63)
  config.fb_count     = 1;  // meno buffer = minor latenza
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Errore init camera: 0x%x\n", err);
    while (true) { delay(1000); }
  }

  sensor_t* s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_VGA);
  s->set_quality(s, 8);
  s->set_brightness(s, 1);
  s->set_contrast(s, 1);
  s->set_saturation(s, 0);
  s->set_sharpness(s, 2);
  s->set_denoise(s, 1);

  Serial.println("Camera inizializzata. Pronto a trasmettere frame su Serial.");
}

// ─── LOOP: cattura frame e invia su Serial ──────────────────
void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Frame fallito");
    delay(10);
    return;
  }

  // Timestamp in microsecondi
  uint64_t ts = esp_timer_get_time();

  // Invia header: magic (5), length (4), timestamp (8)
  Serial.write((const uint8_t*)FRAME_MAGIC, 5);
  uint32_t len = (uint32_t)fb->len;
  Serial.write((const uint8_t*)&len, sizeof(len));
  Serial.write((const uint8_t*)&ts, sizeof(ts));

  // Invia payload JPEG (write non-bloccante a livello API, può bloccare se buffer USB pieno)
  size_t sent = Serial.write(fb->buf, fb->len);

  // debug minimale ogni 32 frame
  static int frame_count = 0;
  frame_count++;
  if ((frame_count & 0x1F) == 0) {
    Serial.printf("Sent frame %d, bytes=%u\n", frame_count, (unsigned)sent);
  }

  esp_camera_fb_return(fb);

  // piccola pausa cooperativa (0 = yield)
  delay(0);
}
*/
