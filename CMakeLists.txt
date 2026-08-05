cmake_minimum_required(VERSION 3.13)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico CACHE STRING "Board type")
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(breakbeam_sensor C CXX ASM)

pico_sdk_init()

add_executable(breakbeam_sensor
    src/main.cpp
    src/drivers/temp-hum/t-h.cpp
    src/drivers/LED/leds.cpp
)

pico_generate_pio_header(breakbeam_sensor ${CMAKE_CURRENT_LIST_DIR}/src/drivers/WS2812/WS2812.pio)

target_include_directories(breakbeam_sensor PUBLIC src/)

pico_enable_stdio_usb(breakbeam_sensor 1)
pico_enable_stdio_uart(breakbeam_sensor 1)

target_link_libraries(breakbeam_sensor
    pico_stdlib
    hardware_adc
    hardware_i2c
    hardware_pio

)

pico_add_extra_outputs(breakbeam_sensor)
