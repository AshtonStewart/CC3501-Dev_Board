#include "leds.h"
#include "pico/stdlib.h"
#include <string.h>
#include <math.h>

// ---------------------------------------------------------------------------
// HSV → RGB conversion  (extension 6)
// ---------------------------------------------------------------------------

RGB hsv_to_rgb(HSV hsv)
{
    float h = hsv.hue;
    float s = hsv.saturation / 100.0f;
    float v = hsv.value      / 100.0f;

    if (s <= 0.0f) {
        uint8_t c = (uint8_t)(v * 255.0f);
        return {c, c, c};
    }

    float h60 = h / 60.0f;
    int   i   = (int)floorf(h60) % 6;
    float f   = h60 - floorf(h60);
    float p   = v * (1.0f - s);
    float q   = v * (1.0f - s * f);
    float t   = v * (1.0f - s * (1.0f - f));

    float r, g, b;
    switch (i) {
        case 0: r = v; g = t; b = p; break;
        case 1: r = q; g = v; b = p; break;
        case 2: r = p; g = v; b = t; break;
        case 3: r = p; g = q; b = v; break;
        case 4: r = t; g = p; b = v; break;
        default: r = v; g = p; b = q; break;
    }

    return {(uint8_t)(r * 255.0f),
            (uint8_t)(g * 255.0f),
            (uint8_t)(b * 255.0f)};
}

// ---------------------------------------------------------------------------
// LEDDriver — constructor / destructor
// ---------------------------------------------------------------------------

LEDDriver::LEDDriver(PIO pio, uint sm, uint num_leds)
    : _pio(pio), _sm(sm), _num_leds(num_leds), _dirty(false)
{
    _buffer = new RGB[_num_leds];
    memset(_buffer, 0, sizeof(RGB) * _num_leds);
}

LEDDriver::~LEDDriver()
{
    delete[] _buffer;
}

// ---------------------------------------------------------------------------
// Assessment
// ---------------------------------------------------------------------------

void LEDDriver::set(uint index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= _num_leds) return;
    _buffer[index] = {red, green, blue};
    _dirty = true;
}

void LEDDriver::set(uint index, RGB colour)
{
    if (index >= _num_leds) return;
    _buffer[index] = colour;
    _dirty = true;
}

void LEDDriver::show()
{
    for (uint i = 0; i < _num_leds; i++) {
        // WS2812 expects GRB, packed into the top 24 bits
        uint32_t word = ((uint32_t)_buffer[i].green << 24)
                      | ((uint32_t)_buffer[i].red   << 16)
                      | ((uint32_t)_buffer[i].blue  <<  8);
        pio_sm_put_blocking(_pio, _sm, word);
    }
    _dirty = false;
}

void LEDDriver::clear()
{
    memset(_buffer, 0, sizeof(RGB) * _num_leds);
    show();
}

// ---------------------------------------------------------------------------
// Extension
// ---------------------------------------------------------------------------

void LEDDriver::set_range(uint start, uint count, const RGB* colours)
{
    for (uint i = 0; i < count && (start + i) < _num_leds; i++) {
        _buffer[start + i] = colours[i];
    }
    _dirty = true;
}

RGB LEDDriver::get(uint index) const
{
    if (index >= _num_leds) return {0, 0, 0};
    return _buffer[index];
}

bool LEDDriver::is_dirty() const
{
    return _dirty;
}

void LEDDriver::set_hsv(uint index, HSV colour)
{
    set(index, hsv_to_rgb(colour));
}