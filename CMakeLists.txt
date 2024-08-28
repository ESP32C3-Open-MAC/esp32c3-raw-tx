# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# These are the 'bare minimum' components required for the ESP-IDF platform. Documentation is here:
# https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html#common-component-requirements
set(COMPONENTS "cxx" "newlib" "freertos" "esp_hw_support" "heap" "log" "soc"
 "hal" "esp_rom" "esp_common" "esp_system" "xtensa/riscv")

# esp_event is required for WiFi event
# esp_wifi is a custom compiled version of the original esp_wifi drivers
# esp_phy is used as is and is a dependency of esp_phy.
# wpa_supplicant is a supplicant library for ESP that is tightly coupled with the WiFi drivers. 
list(APPEND COMPONENTS "esp_event" "esp_wifi" "esp_phy" "wpa_supplicant" "main")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(wifi-raw-tx)
