#pragma once

#include <stdbool.h>

// Driver for the Sensirion SHT40-BD1B temperature/humidity sensor (I2C addr 0x45).
// See datasheet: https://sensirion.com/media/documents/C7461849/ (SHT4x)

void sht40_init(void);
bool sht40_read(float *temperature, float *humidity);
