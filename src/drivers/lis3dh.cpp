#include "lis3dh.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdio.h>

// -----------------------------------------------
// Hardware config — from your schematic
// -----------------------------------------------
#define SDA_PIN     16
#define SCL_PIN     17
#define ACCEL_ADDR  0x19   // SA0 pulled high on your board

// -----------------------------------------------
// Register addresses — from the LIS3DH datasheet
// -----------------------------------------------
#define WHO_AM_I    0x0F   // should always return 0x33
#define CTRL_REG1   0x20   // controls sample rate and which axes are on
#define CTRL_REG4   0x23   // controls measurement range (±2g, ±4g etc)
#define OUT_X_L     0x28   // first of 6 bytes: X low, X high, Y low...