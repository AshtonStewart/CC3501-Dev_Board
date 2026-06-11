#pragma once
#include <stdint.h>
#include <stdbool.h>

// Initialise I2C and the accelerometer.
// Call this once at startup before reading any data.
bool lis3dh_init();

// Read raw X, Y, Z values straight from the sensor.
// These are integers — not in real units yet.
void lis3dh_read_raw(int16_t *x, int16_t *y, int16_t *z);

// Convert a raw integer reading into g-forces.
// e.g. if raw x = 4096, this returns roughly 1.0f (1g)
float lis3dh_to_g(int16_t raw);