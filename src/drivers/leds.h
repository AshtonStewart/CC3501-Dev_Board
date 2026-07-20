#pragma once
#include <stdint.h>
#include "hardware/pio.h"

#define NUM_LEDS 12
#define LED_PIN  14

void leds_init(PIO pio, uint sm);
void leds_set(int index, uint8_t r, uint8_t g, uint8_t b);
void leds_show();
void leds_clear();
void leds_get(int index, uint8_t *r, uint8_t *g, uint8_t *b);
void leds_get_pending(int index, uint8_t *r, uint8_t *g, uint8_t *b);