#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Impostazioni
// Imposta la modalità del Wifi, true = connessione a wifi esistente, false = funge da stazione wifi
bool AP_MODE = false;
// Informazioni reti
const char* ssid = "Diego-WIFI";
const char* password = "Manukitty1632";

const char* ap_ssid = "ESPCAM_SVC";
const char* ap_password = "12345678";

// Configurazione camera
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

// Gestione stream
#define BOUNDARY    "frame"
#define PART_HEADER "\r\n--" BOUNDARY "\r\nContent-Type: image/jpeg\r\n\r\n"
#define MAX_CLIENTS 2

httpd_handle_t server = NULL;
static SemaphoreHandle_t cam_sem = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" BOUNDARY);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "25");
    char part[64];
    const int part_len = snprintf(part, sizeof(part), PART_HEADER);
    while(true){
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb){
            vTaskDelay(1);
            continue;
        }
        res = httpd_resp_send_chunk(req, part, part_len);
        if (res != ESP_OK){
            esp_camera_fb_return(fb);
            break;
        }
        res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if(res != ESP_OK){
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));  // ~25 FPS
    }
    xSemaphoreGive(cam_sem);
    return res;
}

//Funzioni
void connectWifi();
void startServer();
bool initCamera();

void setup(){
    Serial.begin(115200);
    connectWifi();
    initCamera();
    startServer();
}

void connectWifi(){
    if(!AP_MODE){
        WiFi.mode(WIFI_STA);
        Serial.printf("Connessione in corso a %s\n",ssid);
        WiFi.begin(ssid,password);
        while(WiFi.status() != WL_CONNECTED){
            delay(250);
            Serial.print(".");
        }
        Serial.println();
        Serial.printf("Connesso a %s.\n",ssid);
        Serial.print("IP:  ");
        Serial.println(WiFi.localIP());
    }else{
        WiFi.mode(WIFI_AP);
        Serial.printf("Inizializzo l'AP %s.",ap_ssid);
        WiFi.softAP(ap_ssid,ap_password);
        Serial.print("AP IP: ");
        Serial.println(WiFi.softAPIP());
    }
}

void startServer() {

    httpd_config_t config    = HTTPD_DEFAULT_CONFIG();
    config.server_port       = 80;
    config.max_open_sockets  = MAX_CLIENTS + 1;
    config.lru_purge_enable  = true;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;
    config.stack_size        = 8192;
    config.task_priority     = tskIDLE_PRIORITY + 1;

    httpd_uri_t uri = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = stream_handler,
        .user_ctx = NULL
    };

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &uri);
        Serial.println("Server avviato su porta 80");
    } else {
        Serial.println("ERRORE: impossibile avviare il server");
    }
}

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

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init fallita: 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_gainceiling(s, GAINCEILING_4X);
    s->set_sharpness(s, 2);
    s->set_denoise(s, 1);

    return true;
}