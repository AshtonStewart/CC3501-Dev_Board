#include "drivers/temp-hum/t-h.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// SHT40-BD1B-R2 uses 0x45 (BD1B variant)
#define SHT40_I2C_ADDR      0x45
#define SHT40_CMD_MEASURE   0xFD    // High precision measurement

// GPIO pins chosen earlier
#define SHT40_SDA_PIN       20
#define SHT40_SCL_PIN       21

// I2C peripheral (GPIO20/21 = I2C0)
#define SHT40_I2C           i2c0

// CRC-8 verification using polynomial 0x31, init 0xFF (from datasheet section 4.4)
static bool sht40_check_crc(uint8_t data1, uint8_t data2, uint8_t checksum) {
    uint8_t crc = 0xFF;
    uint8_t data[2] = {data1, data2};

    for (int i = 0; i < 2; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc == checksum;
}

void sht40_init(void) {
    i2c_init(SHT40_I2C, 100000);   // 100 kHz standard mode
    gpio_set_function(SHT40_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SHT40_SCL_PIN, GPIO_FUNC_I2C);
}

// Returns true if read was successful
bool sht40_read(float *temperature, float *humidity) {
    uint8_t cmd = SHT40_CMD_MEASURE;
    uint8_t rx_bytes[6];

    // Send measurement command
    int result = i2c_write_blocking(SHT40_I2C, SHT40_I2C_ADDR, &cmd, 1, false);
    if (result < 0) {
        printf("SHT40: write error\n");
        return false;
    }

    // Wait for measurement (datasheet: ~10ms for high precision)
    sleep_ms(10);

    // Read 6 bytes: [T_MSB, T_LSB, T_CRC, RH_MSB, RH_LSB, RH_CRC]
    result = i2c_read_blocking(SHT40_I2C, SHT40_I2C_ADDR, rx_bytes, 6, false);
    if (result < 0) {
        printf("SHT40: read error\n");
        return false;
    }

    // Verify CRC for both temperature and humidity
    if (!sht40_check_crc(rx_bytes[0], rx_bytes[1], rx_bytes[2])) {
        printf("SHT40: temperature CRC mismatch\n");
        return false;
    }
    if (!sht40_check_crc(rx_bytes[3], rx_bytes[4], rx_bytes[5])) {
        printf("SHT40: humidity CRC mismatch\n");
        return false;
    }

    // Reconstruct raw 16-bit values
    uint16_t t_ticks  = (rx_bytes[0] << 8) | rx_bytes[1];
    uint16_t rh_ticks = (rx_bytes[3] << 8) | rx_bytes[4];

    // Convert using datasheet equations (section 4.6)
    // T (°C) = -45 + 175 * t_ticks / 65535
    // RH (%) = -6  + 125 * rh_ticks / 65535
    *temperature = -45.0f + 175.0f * ((float)t_ticks  / 65535.0f);
    *humidity    =  -6.0f + 125.0f * ((float)rh_ticks / 65535.0f);

    // Clamp humidity to 0-100% (datasheet section 4.6)
    if (*humidity > 100.0f) *humidity = 100.0f;
    if (*humidity <   0.0f) *humidity =   0.0f;

    return true;
}
