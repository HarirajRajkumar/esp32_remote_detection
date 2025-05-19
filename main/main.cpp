/**
 * ESP32-S3 Motherboard Detection using ESP-DL
 * 
 * This example demonstrates how to use ESP-DL to detect ESP32-S3 motherboards
 * in images captured by a camera connected to an ESP32-S3.
 * 
 * Prerequisites:
 * - ESP-IDF v5.3 or later
 * - ESP-DL library
 * - Camera module compatible with ESP32-S3
 * - Trained .espdl model for motherboard detection
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "esp_system.h"
#include "esp_spiffs.h"
#include "nvs_flash.h"

// Include ESP-DL components
#include "dl_constant.hpp"
#include "dl_layer.hpp"
#include "dl_detect_define.hpp"
#include "dl_image.hpp"
#include "dl_tool.hpp"
#include "dl_detect.hpp"
#include "dl_model.hpp"

// Camera pins specific for your ESP32-S3 configuration
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

static const char *TAG = "motherboard_detection";

// Path to your model file
#define MODEL_FILE_PATH "/spiffs/motherboard_detection.espdl"

// Detection parameters
#define DETECTION_THRESHOLD 0.5f
#define IMAGE_WIDTH 224
#define IMAGE_HEIGHT 224

// Class names (update these with your actual class names)
const char* class_names[] = {"motherboard", "background"};

// Function prototypes
bool init_camera();
bool init_spiffs();
void motherboard_detection_task(void *pvParameters);

// Initialize SPIFFS for model storage
bool init_spiffs() {
    ESP_LOGI(TAG, "Initializing SPIFFS");
    
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return false;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return false;
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    
    // Check if model file exists
    FILE* f = fopen(MODEL_FILE_PATH, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open model file: %s", MODEL_FILE_PATH);
        return false;
    }
    fclose(f);
    
    ESP_LOGI(TAG, "SPIFFS initialized successfully");
    return true;
}

// Initialize camera
bool init_camera() {
    ESP_LOGI(TAG, "Initializing camera");
    
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
    
    // Configure XCLK frequency
    config.xclk_freq_hz = 20000000;
    
    // Configure pixel format
    config.pixel_format = PIXFORMAT_RGB565;
    
    // Configure frame size and quality
    config.frame_size = FRAMESIZE_240X240;
    config.jpeg_quality = 12;
    config.fb_count = 2;
    
    // Initialize the camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return false;
    }
    
    // Get camera sensor
    sensor_t * s = esp_camera_sensor_get();
    if (s == NULL) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return false;
    }
    
    // Adjust sensor settings for better performance
    s->set_brightness(s, 1);  // Increase brightness a bit
    s->set_contrast(s, 1);    // Default contrast
    s->set_saturation(s, 0);  // Neutral saturation
    
    ESP_LOGI(TAG, "Camera initialized successfully");
    return true;
}

// Load ESP-DL model
dl::model::Model* load_model() {
    ESP_LOGI(TAG, "Loading model from %s", MODEL_FILE_PATH);
    
    // Load the model
    dl::model::Model* model = dl::model::Model::load(MODEL_FILE_PATH);
    if (model == nullptr) {
        ESP_LOGE(TAG, "Failed to load model");
        return nullptr;
    }
    
    // Print model info
    ESP_LOGI(TAG, "Model loaded successfully");
    ESP_LOGI(TAG, "Model version: %s", model->get_version());
    
    // Get input tensor info
    dl::model::Tensor<int8_t>& input_tensor = model->input();
    ESP_LOGI(TAG, "Input tensor shape: (%d, %d, %d, %d)", 
             input_tensor.dims.n, input_tensor.dims.c, 
             input_tensor.dims.h, input_tensor.dims.w);
    
    return model;
}

// Preprocess image for model input
void preprocess_image(camera_fb_t *fb, dl::model::Tensor<int8_t>& input_tensor) {
    // Create a source RGB565 image from camera frame buffer
    dl::img::Image<uint16_t> *rgb565_image = new dl::img::Image<uint16_t>(
        fb->width,
        fb->height,
        1,
        (uint16_t *)fb->buf
    );
    
    // Convert RGB565 to RGB888
    dl::img::Image<uint8_t> *rgb888_image = dl::img::rgb565_to_rgb888(rgb565_image);
    
    // Resize to model input dimensions
    dl::img::Image<uint8_t> *resized_image = dl::img::resize(
        rgb888_image,
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        dl::img::IMG_RESIZE_BILINEAR
    );
    
    // Normalize image to range [-128, 127] (for int8 quantized model)
    dl::img::Image<int8_t> *normalized_image = dl::img::normalize<uint8_t, int8_t>(
        resized_image,
        -128.0f,  // Output minimum value
        127.0f,   // Output maximum value
        0.0f,     // Input minimum value
        255.0f    // Input maximum value
    );
    
    // Copy normalized image data to input tensor
    memcpy(input_tensor.data, normalized_image->get_data(), normalized_image->get_size() * sizeof(int8_t));
    
    // Clean up
    delete rgb565_image;
    delete rgb888_image;
    delete resized_image;
    delete normalized_image;
}

// Process model output for classification results
void process_output(dl::model::Model* model) {
    // Get output tensor
    dl::model::Tensor<int8_t>& output_tensor = model->output();
    
    // For classification model (output: [1, num_classes])
    if (output_tensor.dims.n == 1 && output_tensor.dims.h == 1 && output_tensor.dims.w == 1) {
        int num_classes = output_tensor.dims.c;
        
        // Find the class with highest score
        int8_t max_score = -128;
        int max_class_id = -1;
        
        for (int i = 0; i < num_classes; i++) {
            int8_t score = output_tensor.data[i];
            if (score > max_score) {
                max_score = score;
                max_class_id = i;
            }
        }
        
        // Convert score to float probability (depends on quantization scale)
        float confidence = (float)(max_score + 128) / 255.0f;  // Example conversion
        
        // Print classification result
        if (max_class_id >= 0 && max_class_id < sizeof(class_names)/sizeof(char*)) {
            ESP_LOGI(TAG, "Detected: %s (Confidence: %.2f)", 
                    class_names[max_class_id], confidence);
            
            // Additional actions based on detection
            if (max_class_id == 0 && confidence > DETECTION_THRESHOLD) {
                // Motherboard detected with high confidence
                ESP_LOGI(TAG, "*** MOTHERBOARD DETECTED! ***");
            } else {
                ESP_LOGI(TAG, "No motherboard detected or low confidence");
            }
        } else {
            ESP_LOGE(TAG, "Invalid class ID: %d", max_class_id);
        }
    }
    // For object detection models, the output format will be different
    else {
        ESP_LOGI(TAG, "Output tensor shape: (%d, %d, %d, %d)", 
                output_tensor.dims.n, output_tensor.dims.c, 
                output_tensor.dims.h, output_tensor.dims.w);
        
        // Process output based on your specific model's output format
        ESP_LOGI(TAG, "This model's output format needs specific processing logic");
    }
}

// Main detection task
void motherboard_detection_task(void *pvParameters) {
    // Initialize SPIFFS
    if (!init_spiffs()) {
        vTaskDelete(NULL);
        return;
    }
    
    // Initialize camera
    if (!init_camera()) {
        vTaskDelete(NULL);
        return;
    }
    
    // Load model
    dl::model::Model* model = load_model();
    if (model == nullptr) {
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Starting detection loop");
    
    while (1) {
        // Capture frame
        ESP_LOGI(TAG, "Capturing image...");
        camera_fb_t *fb = esp_camera_fb_get();
        
        if (!fb) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // Process image and run inference
        uint64_t start_time = esp_timer_get_time();
        
        // Get input tensor from model
        dl::model::Tensor<int8_t>& input_tensor = model->input();
        
        // Preprocess image for model input
        preprocess_image(fb, input_tensor);
        
        // Run inference
        model->run();
        
        // Process output
        process_output(model);
        
        uint64_t end_time = esp_timer_get_time();
        ESP_LOGI(TAG, "Inference time: %llu ms", (end_time - start_time) / 1000);
        
        // Return frame buffer
        esp_camera_fb_return(fb);
        
        // Delay between captures
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
    
    // Clean up
    delete model;
    vTaskDelete(NULL);
}

extern "C" void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP32-S3 Motherboard Detection");
    
    // Create detection task
    xTaskCreatePinnedToCore(
        motherboard_detection_task,
        "motherboard_detection",
        8192,
        NULL,
        5,
        NULL,
        0
    );
}
