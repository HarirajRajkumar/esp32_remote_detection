#include "esp_camera.h"
#include <WiFi.h>
#include "img_converters.h"
#include "SPIFFS.h"
#include "dl_model.hpp"
#include "dl_image.hpp"

// VERSION 3
// ESP32-S3 Motherboard Detection with Custom ESP-DL Model

// ===================
// Select camera model
// ===================

#define DETECTRA_LITE_V2_ESP32S3 // Has PSRAM

#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "Airtel_dora_7968_5G";
const char *password = "air48909";

// Model parameters
#define MODEL_WIDTH 96
#define MODEL_HEIGHT 96
#define MODEL_CHANNELS 3
#define MODEL_PATH "/motherboard_model.espdl"
#define DETECTION_THRESHOLD 0.5f  // Adjust based on your model's performance

// Detection state
bool motherboardDetected = false;
unsigned long lastDetectionTime = 0;
const unsigned long detectionInterval = 1000;  // Check every second

// LED pin for indication
#if defined(LED_GPIO_NUM)
#define STATUS_LED LED_GPIO_NUM
#else
#define STATUS_LED 4  // GPIO 4, adjust as needed for your board
#endif

// Global model variable
dl::model::Model* model = nullptr;

void startCameraServer();
void setupLedFlash(int pin);
bool detectMotherboardWithModel(camera_fb_t *fb);
bool initSPIFFS();
bool loadESPDLModel();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Set up the status LED
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  // Initialize SPIFFS and load model
  if (initSPIFFS()) {
    if (loadESPDLModel()) {
      Serial.println("Model loaded successfully");
    } else {
      Serial.println("Failed to load model, will use simpler detection method");
    }
  } else {
    Serial.println("SPIFFS initialization failed, will use simpler detection method");
  }

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

#if defined(DETECTRA_LITE_V2_ESP32S3)
  s->set_vflip(s, 1);
#endif

// Setup LED Flash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  Serial.println("Motherboard detection is running in the background");
}

void loop() {
  // Check for motherboard detection periodically
  unsigned long currentTime = millis();
  if (currentTime - lastDetectionTime >= detectionInterval) {
    lastDetectionTime = currentTime;
    
    // Get a frame from the camera
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      // Run detection using the model
      motherboardDetected = detectMotherboardWithModel(fb);
      
      // Update LED status
      digitalWrite(STATUS_LED, motherboardDetected ? HIGH : LOW);
      
      // Print detection status
      Serial.print("Motherboard detection: ");
      Serial.println(motherboardDetected ? "DETECTED" : "NOT DETECTED");
      
      // Return the frame buffer
      esp_camera_fb_return(fb);
    }
  }
  
  delay(100); // Small delay to prevent CPU hogging
}

bool initSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialization failed");
    return false;
  }
  
  // List files in SPIFFS
  Serial.println("Files in SPIFFS:");
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
  while (file) {
    Serial.print("  ");
    Serial.print(file.name());
    Serial.print(" (");
    Serial.print(file.size());
    Serial.println(" bytes)");
    file = root.openNextFile();
  }
  
  return true;
}

bool loadESPDLModel() {
  // Check if model file exists
  if (!SPIFFS.exists(MODEL_PATH)) {
    Serial.println("Model file not found");
    return false;
  }
  
  // Load model using ESP-DL
  model = dl::model::Model::load("/spiffs" MODEL_PATH);
  if (model == nullptr) {
    Serial.println("Failed to load model");
    return false;
  }
  
  // Log model information
  Serial.print("Model version: ");
  Serial.println(model->get_version());
  
  // Get input tensor information
  dl::model::Tensor<int8_t>& input_tensor = model->input();
  Serial.printf("Input tensor shape: [%d, %d, %d, %d]\n", 
                input_tensor.dims.n, input_tensor.dims.c, 
                input_tensor.dims.h, input_tensor.dims.w);
  
  return true;
}

bool detectMotherboardWithModel(camera_fb_t *fb) {
  // If model not loaded, fall back to a simpler detection method
  if (model == nullptr) {
    return detectMotherboardSimple(fb);
  }
  
  uint64_t start_time = esp_timer_get_time();
  
  // Convert the camera image to the format needed by the model
  uint8_t *rgb888_buffer = NULL;
  bool need_free = false;
  
  // If JPEG, convert to RGB888
  if (fb->format == PIXFORMAT_JPEG) {
    size_t out_len = fb->width * fb->height * 3; // RGB888 needs 3 bytes per pixel
    rgb888_buffer = (uint8_t *)malloc(out_len);
    if (!rgb888_buffer) {
      Serial.println("Failed to allocate memory for RGB888 conversion");
      return false;
    }
    
    // Convert JPEG to RGB888
    bool converted = jpg2rgb888(fb->buf, fb->len, rgb888_buffer, JPG_SCALE_NONE) == ESP_OK;
    if (!converted) {
      free(rgb888_buffer);
      Serial.println("Failed to convert JPEG to RGB888");
      return false;
    }
    
    need_free = true;
  }
  // If RGB565, convert to RGB888
  else if (fb->format == PIXFORMAT_RGB565) {
    size_t out_len = fb->width * fb->height * 3; // RGB888 needs 3 bytes per pixel
    rgb888_buffer = (uint8_t *)malloc(out_len);
    if (!rgb888_buffer) {
      Serial.println("Failed to allocate memory for RGB888 conversion");
      return false;
    }
    
    // Manual conversion from RGB565 to RGB888
    uint16_t *rgb565 = (uint16_t *)fb->buf;
    for (int i = 0; i < fb->width * fb->height; i++) {
      uint16_t pixel = rgb565[i];
      uint8_t r = (pixel >> 11) & 0x1F;
      uint8_t g = (pixel >> 5) & 0x3F;
      uint8_t b = pixel & 0x1F;
      
      // Convert to 8-bit per channel
      rgb888_buffer[i*3]     = (r * 255) / 31;
      rgb888_buffer[i*3 + 1] = (g * 255) / 63;
      rgb888_buffer[i*3 + 2] = (b * 255) / 31;
    }
    
    need_free = true;
  }
  // If already RGB888, use directly
  else if (fb->format == PIXFORMAT_RGB888) {
    rgb888_buffer = fb->buf;
  }
  else {
    Serial.println("Unsupported format for detection");
    return false;
  }
  
  // Create an ESP-DL image from the RGB888 buffer
  dl::img::Image<uint8_t> *input_image = new dl::img::Image<uint8_t>(
    fb->width, fb->height, 3, rgb888_buffer);
  
  // Resize the image to model input size
  dl::img::Image<uint8_t> *resized_image = dl::img::resize(
    input_image, MODEL_WIDTH, MODEL_HEIGHT, dl::img::IMG_RESIZE_BILINEAR);
  
  // Normalize the image to the range expected by the model (-128 to 127 for int8)
  dl::img::Image<int8_t> *normalized_image = dl::img::normalize<uint8_t, int8_t>(
    resized_image, -128.0f, 127.0f, 0.0f, 255.0f);
  
  // Get the input tensor from the model
  dl::model::Tensor<int8_t>& input_tensor = model->input();
  
  // Copy the processed image data to the input tensor
  memcpy(input_tensor.data, normalized_image->get_data(), 
         MODEL_WIDTH * MODEL_HEIGHT * MODEL_CHANNELS);
  
  // Run inference
  model->run();
  
  // Get the output tensor
  dl::model::Tensor<int8_t>& output_tensor = model->output();
  
  // Process the output for binary classification
  // Assuming output is [batch, classes] where classes = 2 (no motherboard, motherboard)
  int8_t motherboard_score = output_tensor.dims.c > 1 ? output_tensor.data[1] : output_tensor.data[0];
  
  // Convert score to probability (depends on quantization)
  float confidence = (motherboard_score + 128.0f) / 255.0f;
  Serial.printf("Motherboard confidence: %.3f\n", confidence);
  
  // Clean up
  delete normalized_image;
  delete resized_image;
  delete input_image;
  
  if (need_free) {
    free(rgb888_buffer);
  }
  
  uint64_t end_time = esp_timer_get_time();
  Serial.printf("Detection took %llu ms\n", (end_time - start_time) / 1000);
  
  // Return detection result based on confidence threshold
  return confidence > DETECTION_THRESHOLD;
}

// Fallback simple detection method based on color
bool detectMotherboardSimple(camera_fb_t *fb) {
  uint16_t *rgb565_buffer = NULL;
  size_t rgb565_len = 0;
  bool need_free = false;
  
  // If already in RGB565 format, use directly
  if (fb->format == PIXFORMAT_RGB565) {
    rgb565_buffer = (uint16_t *)fb->buf;
    rgb565_len = fb->len / 2; // 2 bytes per pixel
  } 
  // If in JPEG format, convert to RGB565
  else if (fb->format == PIXFORMAT_JPEG) {
    rgb565_buffer = (uint16_t *)malloc(fb->width * fb->height * 2); // 2 bytes per pixel
    if (!rgb565_buffer) {
      Serial.println("Failed to allocate memory for RGB565 conversion");
      return false;
    }
    
    // Convert JPEG to RGB565
    if (jpg2rgb565(fb->buf, fb->len, rgb565_buffer, JPG_SCALE_NONE) == ESP_OK) {
      rgb565_len = fb->width * fb->height;
      need_free = true;
    } else {
      free(rgb565_buffer);
      Serial.println("Failed to convert JPEG to RGB565");
      return false;
    }
  } else {
    Serial.println("Unsupported format for detection");
    return false;
  }
  
  // Count green pixels (typical PCB color)
  int green_pixels = 0;
  int total_pixels = fb->width * fb->height;
  
  // Analyze a subset of pixels to improve performance
  // Check every 8th pixel for efficiency
  for (int i = 0; i < rgb565_len; i += 8) {
    uint16_t pixel = rgb565_buffer[i];
    
    // Extract RGB components from RGB565 format
    uint8_t r = (pixel >> 11) & 0x1F;  // 5 bits for red
    uint8_t g = (pixel >> 5) & 0x3F;   // 6 bits for green
    uint8_t b = pixel & 0x1F;          // 5 bits for blue
    
    // Scale to 0-255 range for easier comparison
    r = (r * 255) / 31;
    g = (g * 255) / 63;
    b = (b * 255) / 31;
    
    // Check if green component is dominant (PCB green detection)
    if (g > r + 20 && g > b + 20) {
      green_pixels++;
    }
  }
  
  // Free the converted buffer if needed
  if (need_free) {
    free(rgb565_buffer);
  }
  
  // Calculate green ratio (accounting for sampling every 8th pixel)
  float green_ratio = (float)(green_pixels * 8) / total_pixels;
  Serial.printf("Green pixel ratio: %.3f\n", green_ratio);
  
  // Return detection result based on threshold
  return green_ratio > 0.15f;
}