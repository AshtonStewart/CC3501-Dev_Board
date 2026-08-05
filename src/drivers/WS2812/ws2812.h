#pragma once

#include "pico/stdlib.h"

// Driver for a single WS2812D-F5-1261 addressable RGB LED, driven via PIO.
//
// Implemented in ws2812.c (plain C), but included from main.cpp (C++), so
// the declarations need extern "C" linkage - otherwise the C++ compiler
// expects name-mangled symbols that the C-compiled .o file doesn't provide,
// and the linker fails with "undefined reference" errors.

#ifdef __cplusplus
extern "C" {
#endif

void ws2812_init(uint pin);
void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif
