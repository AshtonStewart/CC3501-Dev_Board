#pragma once
#include <stdint.h>
#include "hardware/pio.h"

#define NUM_LEDS 12
#define LED_PIN  14

void leds_init(PIO pio, uint sm);
void leds_set(int index, uint8_t r, uint8_t g, uint8_t b);
void leds_show();
void leds_clear();

// Extensions
void leds_set_all(uint8_t r, uint8_t g, uint8_t b);
void leds_get(int index, uint8_t *r, uint8_t *g, uint8_t *b);
bool leds_is_dirty();
void leds_set_hsv(int index, float hue, uint8_t sat, uint8_t val);