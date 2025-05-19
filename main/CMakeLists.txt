idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES motherboard_detect esp32-camera esp_timer nvs_flash spiffs
)