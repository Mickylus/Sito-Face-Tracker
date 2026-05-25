#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ─── WIFI ─────────────────────────────
bool STA_MODE = false;
// — STA: prova a connettersi al router —
const char* sta_ssid     = "Diego-WIFI";       // ← cambia
const char* sta_password = "Manukitty1632";   // ← cambia
#define STA_TIMEOUT_MS   10000

// — AP: fallback —
const char* ap_ssid      = "ESPCAM_SVC";
const char* ap_password  = "12345678";
IPAddress   ap_ip(192,168,4,1);
IPAddress   ap_gateway(192,168,4,1);
IPAddress   ap_subnet(255,255,255,0);

bool connected_as_sta = false;

// ─── CAMERA PINS ──────────────────────
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

// ─── WEBSOCKET SERVER ─────────────────
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Task handle per il loop di streaming
TaskHandle_t stream_task_handle = NULL;

// ─── STREAM TASK ──────────────────────
// Gira su Core 0, invia frame a tutti i client WebSocket connessi
void stream_task(void* arg) {

    for (;;) {

        // Nessun client connesso → aspetta
        if (ws.count() == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(1);
            continue;
        }

        // Invia il JPEG come messaggio binario a tutti i client
        ws.binaryAll((uint8_t*)fb->buf, fb->len);

        esp_camera_fb_return(fb);

        // ~25 FPS
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ─── WEBSOCKET EVENTS ─────────────────
void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {

    if (type == WS_EVT_CONNECT) {
        Serial.printf("Client #%u connesso da %s\n",
                      client->id(), client->remoteIP().toString().c_str());

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("Client #%u disconnesso\n", client->id());
    }
}

// ─── CAMERA ───────────────────────────
bool initCamera() {

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0    = Y2_GPIO_NUM;
    config.pin_d1    = Y3_GPIO_NUM;
    config.pin_d2    = Y4_GPIO_NUM;
    config.pin_d3    = Y5_GPIO_NUM;
    config.pin_d4    = Y6_GPIO_NUM;
    config.pin_d5    = Y7_GPIO_NUM;
    config.pin_d6    = Y8_GPIO_NUM;
    config.pin_d7    = Y9_GPIO_NUM;
    config.pin_xclk  = XCLK_GPIO_NUM;
    config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href  = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn  = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count     = 2;
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    if (esp_camera_init(&config) != ESP_OK) {
        Serial.println("Camera init fallita");
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_gainceiling(s, GAINCEILING_4X);
    s->set_sharpness(s, 2);
    s->set_denoise(s, 1);

    return true;
}

// ─── WIFI ─────────────────────────────
void initWiFi() {
    if(STA_MODE){
        Serial.printf("Connessione a \"%s\"...\n", sta_ssid);
        WiFi.mode(WIFI_STA);
        WiFi.begin(sta_ssid, sta_password);

        unsigned long t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < STA_TIMEOUT_MS) {
            delay(250);
            Serial.print(".");
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
            connected_as_sta = true;
            Serial.print("Connesso! IP: ");
            Serial.println(WiFi.localIP());
        }
    }else{
        Serial.println("Stazione AP");
        WiFi.mode(WIFI_AP);
        WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
        WiFi.softAP(ap_ssid, ap_password);
        Serial.print("AP IP: ");
        Serial.println(WiFi.softAPIP());
    }
}

// ─── SETUP ────────────────────────────
void setup() {

    Serial.begin(115200);
    Serial.println("\n=== ESP32-CAM WebSocket ===");

    if (!initCamera()) {
        delay(3000);
        ESP.restart();
    }

    initWiFi();

    // Registra handler WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    // Pagina di test minimale (opzionale)
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/plain", "ESP32-CAM WebSocket OK");
    });

    server.begin();
    Serial.println("Server WebSocket avviato.");

    IPAddress ip = connected_as_sta ? WiFi.localIP() : WiFi.softAPIP();
    Serial.printf("ws://%s/ws\n", ip.toString().c_str());

    // Avvia stream task su Core 0 (Core 1 è usato da Arduino)
    xTaskCreatePinnedToCore(
        stream_task,
        "stream",
        4096,       // stack
        NULL,
        2,          // priorità
        &stream_task_handle,
        0           // Core 0
    );
}

// ─── LOOP ─────────────────────────────
void loop() {

    // Pulizia client WebSocket disconnessi
    ws.cleanupClients();

    if (connected_as_sta && WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi perso — riavvio...");
        delay(1000);
        ESP.restart();
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}