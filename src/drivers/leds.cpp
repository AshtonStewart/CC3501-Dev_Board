#include "leds.h"
#include "hardware/pio.h"
#include <cstring>

static PIO     _pio;
static uint    _sm;

// Current committed values (what the LEDs are showing right now)
static uint8_t led_r[NUM_LEDS];
static uint8_t led_g[NUM_LEDS];
static uint8_t led_b[NUM_LEDS];

// Pending values (what will be sent on next leds_show())
static uint8_t pending_r[NUM_LEDS];
static uint8_t pending_g[NUM_LEDS];
static uint8_t pending_b[NUM_LEDS];

void leds_init(PIO pio, uint sm) {
    _pio = pio;
    _sm  = sm;
    memset(led_r, 0, sizeof(led_r));
    memset(led_g, 0, sizeof(led_g));
    memset(led_b, 0, sizeof(led_b));
    memset(pending_r, 0, sizeof(pending_r));
    memset(pending_g, 0, sizeof(pending_g));
    memset(pending_b, 0, sizeof(pending_b));
    leds_show();
}

// Stage a change — does NOT affect the live LEDs yet
void leds_set(int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index < 0 || index >= NUM_LEDS) return;
    pending_r[index] = r;
    pending_g[index] = g;
    pending_b[index] = b;
}

// Commit all pending values to the hardware
void leds_show() {
    for (int i = 0; i < NUM_LEDS; i++) {
        led_r[i] = pending_r[i];
        led_g[i] = pending_g[i];
        led_b[i] = pending_b[i];

        // WS2812D expects GRB order; PIO shifts from MSB
        uint32_t word = ((uint32_t)led_g[i] << 24)
                      | ((uint32_t)led_r[i] << 16)
                      | ((uint32_t)led_b[i] <<  8);
        pio_sm_put_blocking(_pio, _sm, word);
    }
}

void leds_clear() {
    memset(pending_r, 0, sizeof(pending_r));
    memset(pending_g, 0, sizeof(pending_g));
    memset(pending_b, 0, sizeof(pending_b));
    leds_show();
}

// Returns the CURRENT (committed) values
void leds_get(int index, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (index < 0 || index >= NUM_LEDS) return;
    *r = led_r[index];
    *g = led_g[index];
    *b = led_b[index];
}

// Returns the PENDING (not yet committed) values
void leds_get_pending(int index, uint8_t *r, uint8_t *g, uint8_t *b) {
    if (index < 0 || index >= NUM_LEDS) return;
    *r = pending_r[index];
    *g = pending_g[index];
    *b = pending_b[index];
}