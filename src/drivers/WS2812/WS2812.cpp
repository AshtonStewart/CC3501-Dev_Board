#include "ws2812.h"
#include "WS2812.pio.h"
#include "hardware/pio.h"

static PIO ws2812_pio = pio0;
static uint ws2812_sm = 0;

// WS2812 pixels are transmitted MSB-first as G, R, B (8 bits each), packed
// into the top 24 bits of the 32-bit FIFO word.
static inline uint32_t grb_from_rgb(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)g << 24) | ((uint32_t)r << 16) | ((uint32_t)b << 8);
}

void ws2812_init(uint pin) {
    uint offset = pio_add_program(ws2812_pio, &ws2812_program);
    ws2812_sm = pio_claim_unused_sm(ws2812_pio, true);
    ws2812_program_init(ws2812_pio, ws2812_sm, offset, pin, 800000.0f, false);
}

void ws2812_set_color(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t pixel = grb_from_rgb(r, g, b);
    pio_sm_put_blocking(ws2812_pio, ws2812_sm, pixel);
}
