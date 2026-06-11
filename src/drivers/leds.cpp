#include "leds.h"
#include "hardware/pio.h"
#include <cmath>
#include <cstring>

static PIO  _pio;
static uint _sm;
static bool _dirty = false;

static uint8_t led_r[NUM_LEDS];
static uint8_t led_g[NUM_LEDS];
static uint8_t led_b[NUM_LEDS];

void leds_init(PIO pio, uint sm) {
    _pio = pio;
    _sm  = sm;
    memset(led_r, 0, sizeof(led_r));
    memset(led_g, 0, sizeof(led_g));
    memset(led_b, 0, sizeof(led_b));
    leds_show();
}

void leds_set(int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < 0 || index >= NUM_LEDS) return;
    led_r[index] = r;
    led_g[index] = g;
    led_b[index] = b;
    _dirty = true;
}

void leds_show() {
    for (int i = 0; i < NUM_LEDS; i++) {
        // WS2812D expects GRB order; PIO shifts from MSB
        uint32_t word = ((uint32_t)led_g[i] << 24)
                      | ((uint32_t)led_r[i] << 16)
                      | ((uint32_t)led_b[i] <<  8);
        pio_sm_put_blocking(_pio, _sm, word);
    }
    _dirty = false;
}

void leds_clear() {
    memset(led_r, 0, sizeof(led_r));
    memset(led_g, 0, sizeof(led_g));
    memset(led_b, 0, sizeof(led_b));
    leds_show();
}

void leds_set_all(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < NUM_LEDS; i++)
        leds_set(i, r, g, b);
}

void leds_get(int index, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (index < 0 || index >= NUM_LEDS) return;
    *r = led_r[index];
    *g = led_g[index];
    *b = led_b[index];
}

bool leds_is_dirty() {
    return _dirty;
}

void leds_set_hsv(int index, float hue, uint8_t sat, uint8_t val) {
    float s = sat / 255.0f, v = val / 255.0f;
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r = 0, g = 0, b = 0;
    if      (hue < 60)  { r=c; g=x; }
    else if (hue < 120) { r=x; g=c; }
    else if (hue < 180) {      g=c; b=x; }
    else if (hue < 240) {      g=x; b=c; }
    else if (hue < 300) { r=x;      b=c; }
    else                { r=c;      b=x; }
    leds_set(index,
        (uint8_t)((r + m) * 255),
        (uint8_t)((g + m) * 255),
        (uint8_t)((b + m) * 255));
}