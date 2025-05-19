cmake_minimum_required(VERSION 3.5)

# Include ESP-IDF's project helper macro
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the component directories
set(EXTRA_COMPONENT_DIRS 
    ${CMAKE_CURRENT_LIST_DIR}/components
)

# Set the project name
project(esp32s3_motherboard_detection)

# Set the partition table CSV file
set(PARTITION_TABLE_CSV_PATH ${CMAKE_CURRENT_LIST_DIR}/partitions.csv)

# Enable C++ exceptions
add_compile_options(-fexceptions)