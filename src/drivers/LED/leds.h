#pragma once

#include "pico/stdlib.h"
#include "hardware/pio.h"

// ---------------------------------------------------------------------------
// Colour types
// ---------------------------------------------------------------------------

struct RGB {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

struct HSV {
    float hue;        // 0.0 – 360.0
    float saturation; // 0.0 – 100.0
    float value;      // 0.0 – 100.0
};

// Free function — useful outside the class too
RGB hsv_to_rgb(HSV hsv);

// ---------------------------------------------------------------------------
// LEDDriver class  (extension 1)
// ---------------------------------------------------------------------------

class LEDDriver {
public:
    // Extension 5: num_leds configurable at construction time, not hardcoded
    LEDDriver(PIO pio, uint sm, uint num_leds = 3);
    ~LEDDriver();

    // --- Assessment (points 1-3) ---

    // Point 1 & 2: set one LED colour; buffered until show() is called
    void set(uint index, uint8_t red, uint8_t green, uint8_t blue);
    void set(uint index, RGB colour);

    // Point 2: push all buffered changes to the hardware at once
    void show();

    // Point 3: turn every LED off and push immediately
    void clear();

    // --- Extension ---

    // Extension 2: update a range of LEDs in one call
    void set_range(uint start, uint count, const RGB* colours);

    // Extension 3: query the currently buffered colour of one LED
    RGB  get(uint index) const;

    // Extension 4: true if buffer has been changed since the last show()
    bool is_dirty() const;

    // Extension 6: set a single LED using HSV
    void set_hsv(uint index, HSV colour);

private:
    PIO      _pio;
    uint     _sm;
    uint     _num_leds;
    RGB*     _buffer;
    bool     _dirty;
};