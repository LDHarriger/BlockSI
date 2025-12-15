# CMakeLists.txt for BlockSI Control System

cmake_minimum_required(VERSION 3.16)

# Add Golioth SDK components directory
list(APPEND EXTRA_COMPONENT_DIRS 
    "C:/Users/ldhar/Documents/golioth-firmware-sdk/port/esp_idf/components"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(blocksi_control)