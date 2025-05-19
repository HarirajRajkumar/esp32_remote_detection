#include <Arduino.h>
#include "esp_camera.h"

// Define camera pins for ESP32-S3
#define PWDN_GPIO_NUM 9
#define RESET_GPIO_NUM 11
#define XCLK_GPIO_NUM 8
#define SIOD_GPIO_NUM 13
#define SIOC_GPIO_NUM 3
#define Y2_GPIO_NUM 7
#define Y3_GPIO_NUM 5
#define Y4_GPIO_NUM 4
#define Y5_GPIO_NUM 6
#define Y6_GPIO_NUM 15
#define Y7_GPIO_NUM 17
#define Y8_GPIO_NUM 18
#define Y9_GPIO_NUM 12
#define VSYNC_GPIO_NUM 10
#define HREF_GPIO_NUM 40
#define PCLK_GPIO_NUM 16

bool setup_camera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_240X240;  // or try FRAMESIZE_QVGA
  config.jpeg_quality = 12;
  config.fb_count = 2;
  
  // Initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }
  
  Serial.println("Camera initialized successfully");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 Camera Test");
  
  if (!setup_camera()) {
    Serial.println("Failed to initialize camera");
    return;
  }
}

void loop() {
  // Capture a frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    return;
  }
  
  // Print frame info
  Serial.printf("Frame captured: %dx%d, %d bytes\n", 
                fb->width, fb->height, fb->len);
  
  // Return the frame buffer
  esp_camera_fb_return(fb);
  
  delay(3000);
}