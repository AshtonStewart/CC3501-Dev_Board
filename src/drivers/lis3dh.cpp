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
#define LIS3DH_WHO_AM_I    0x0F   // should always return 0x33
#define LIS3DH_CTRL_REG1   0x20   // controls sample rate and which axes are on
#define LIS3DH_CTRL_REG4   0x23   // controls measurement range (±2g, ±4g etc)
#define LIS3DH_OUT_X_L     0x28   // first of 6 bytes: X low, X high, Y low...

bool lis3dh_init() {
    i2c_init(i2c0, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // Check WHO_AM_I register — should return 0x33
    uint8_t reg = LIS3DH_WHO_AM_I;
    uint8_t answer = 0;
    i2c_write_blocking(i2c0, ACCEL_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, ACCEL_ADDR, &answer, 1, false);

    if (answer != 0x33) {
        printf("LIS3DH not found! WHO_AM_I returned 0x%02X\n", answer);
        return false;
    }
    printf("LIS3DH found at 0x%02X\n", ACCEL_ADDR);

    // Wake up: 10Hz, normal mode, all axes enabled
    uint8_t ctrl1[2] = {LIS3DH_CTRL_REG1, 0x27};
    i2c_write_blocking(i2c0, ACCEL_ADDR, ctrl1, 2, false);

    // ±2g range, high-res mode, block data update on
    uint8_t ctrl4[2] = {LIS3DH_CTRL_REG4, 0x88};
    i2c_write_blocking(i2c0, ACCEL_ADDR, ctrl4, 2, false);

    return true;
}

void lis3dh_read_raw(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t buf[6];
    uint8_t reg = LIS3DH_OUT_X_L | 0x80;  // auto-increment bit

    i2c_write_blocking(i2c0, ACCEL_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c0, ACCEL_ADDR, buf, 6, false);

    *x = (int16_t)((buf[1] << 8) | buf[0]) >> 4;
    *y = (int16_t)((buf[3] << 8) | buf[2]) >> 4;
    *z = (int16_t)((buf[5] << 8) | buf[4]) >> 4;
}

float lis3dh_to_g(int16_t raw) {
    // In ±2g high-res mode, sensitivity is 1mg/digit, 12-bit = 2048 counts per g
    return raw / 2048.0f;
}