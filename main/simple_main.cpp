#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "motherboard_detect.hpp"

extern "C" void app_main(void) {
    printf("ESP32-S3 Motherboard Detection\n");

    MotherboardDetector detector;
    bool result = detector.detect(NULL, 224, 224);
    printf("Detection result: %s\n", result ? "Motherboard detected" : "No motherboard");

    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}