#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

#include "WS2812.pio.h" // This header file gets produced during compilation from the WS2812.pio file
#include "drivers/logging/logging.h"
#include <string>

#define LED_PIN 14
#define NUM_LEDS 12
//Addressable RGB strip, each led in the row is based off a position (so first led is 0, second is 1, third is 2, etc)

// Edit this code, add leds.cpp and leds.h then link that to cmakelists.txt to build a good foundation.

//lab 2:



//Lab 3:
    #define grav_const 9.81
    #define baud_rate_in_z 400000
    //stationary object so try to have g of 1 and resolution of2?

    //configure to use I2C or PSI

    #define ACCEL_SCL_SCLK 2 //GPIO4
    #define Accel_SDA_MOSI 3 //GPIO5

    #define ACCEL_ADDR  0x19
    #define CTRL_REG1   0x20
    #define OUT_X_L     0x28
    #define OUT_X_H     0x29
    #define OUT_Y_L     0x2A
    #define OUT_Y_H     0x2B
    #define OUT_Z_L     0x2C
    #define OUT_Z_H     0x2D

    #define SDA_PIN     16    // GPIO16 = Accel_SDA_MOSI — data line, bidirectional
    #define SCL_PIN     17    // GPIO17 = Accel_SCL_SCLK — clock line, driven by RP2040
    //#define slave_address 001100xb


int lab3() {

    //Initialise system

    stdio_init_all();

    i2c_init(i2c0, 400000); //400kHz speed    
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);  // SDA_PIN of 16
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);  // SCL_PIN of 17
    gpio_pull_up(SDA_PIN);   // add this
    gpio_pull_up(SCL_PIN);   // add this

    uint8_t WHO_AM_I = 0x0F;
    uint8_t answer = 0;
    i2c_write_blocking(i2c0, ACCEL_ADDR, &WHO_AM_I, 1, true);
    i2c_read_blocking(i2c0, ACCEL_ADDR, &answer, 1, false);

    // Wake up the accelerometer - 10Hz, normal mode, all axes enabled
    uint8_t ctrl1[2] = {0x20, 0x27};  // CTRL_REG1, value 0x27
    i2c_write_blocking(i2c0, ACCEL_ADDR, ctrl1, 2, false);

    // Set ±2g range, high-res mode, block data update on
    uint8_t ctrl4[2] = {0x23, 0x88};  // CTRL_REG4, value 0x88
    i2c_write_blocking(i2c0, ACCEL_ADDR, ctrl4, 2, false);

    for (;;) {


    if (answer != 0x33) {
            printf("Accelerometer not on correct address and is connected to 0x%02X\n", answer);
        }

    if (answer == 0x33) {
        printf("Accelerometer connected to 0x%02X\n", answer);
    }

    uint8_t buf[6];
    uint8_t reg = 0x28 | 0x80;  // OUT_X_L address, with auto-increment bit set

    // Step 1: tell the chip which register to start reading from
    i2c_write_blocking(i2c0, ACCEL_ADDR, &reg, 1, true);

    // Step 2: read 6 bytes back (X_L, X_H, Y_L, Y_H, Z_L, Z_H)
    i2c_read_blocking(i2c0, ACCEL_ADDR, buf, 6, false);

    // Step 3: NOW you combine them
    int16_t raw_x = (int16_t)((buf[1] << 8) | buf[0]) >> 4;
    int16_t raw_y = (int16_t)((buf[3] << 8) | buf[2]) >> 4;
    int16_t raw_z = (int16_t)((buf[5] << 8) | buf[4]) >> 4;
    
    printf("raw_x: %d, raw_y: %d, raw_z: %d\n", raw_x, raw_y, raw_z);


    for (int i = 0; i < 12; i++) {
    uint32_t colour;

    if (i >= 0 && i <= 3) {
        // left column - inverted x
        if (raw_x < -100)      colour = (255u << 16) << 8;  // red
        else if (raw_x > 100)  colour = (255u << 24);        // green
        else                   colour = (255u << 8);          // blue

    } else if (i >= 4 && i <= 7) {
        // top row - driven by y
        if (raw_y > 100)       colour = (255u << 16) << 8;  // red
        else if (raw_y < -100) colour = (255u << 24);        // green
        else                   colour = (255u << 8);          // blue

    } else {
        // right column (8-11) - x
        if (raw_x > 100)       colour = (255u << 16) << 8;  // red
        else if (raw_x < -100) colour = (255u << 24);        // green
        else                   colour = (255u << 8);          // blue
    }

    pio_sm_put_blocking(pio0, 0, colour);
}
sleep_ms(100);

    }

    return 0;

}


















int lab4_mic_initialise()
{
    int sample_rate = 44100; // 44.1 kHz standard audio sample rate
    int fft_length = 1024; // Number of samples for FFT

    int clkdiv = clock_get_hz(clk_sys) / (baud_rate_in_z * 8); // 8 cycles per bit for WS2812

    int T = 1 + clkdiv; // Periods of the ADC clock

    int adc_clk = 48 * 1000000; // 48 MHz system clock, convert to MHz for ADC
    int T2 = (1 + clkdiv) / adc_clk;

    //try to get T = 1 / (44.1Khz)
    adc_init();
    adc_run(true);
    adc_run(false);

    adc_fifo_get_blocking();

    adc_fifo_drain();
}

int lab4_mic_read()
{
    int num_samples = 1024; // Number of samples to read for FFT
    uint16_t buffer[1024]; // Buffer to hold ADC samples

    adc_fifo_drain(); // Clear any old data from the FIFO
    adc_run(true); // Start the ADC

    for (int i = 0; i < num_samples; i++) {
        buffer[i] = adc_fifo_get_blocking(); // Read a sample from the ADC FIFO
    }

    adc_run(false); // Stop the ADC
    adc_fifo_drain(); // Clear the FIFO again

    // At this point, 'buffer' contains 'num_samples' raw ADC readings that can be processed with an FFT

    return 0;
}

















int main()
{
    stdio_init_all();

    // Initialise PIO0 to control the LED chain
    uint pio_program_offset = pio_add_program(pio0, &ws2812_program);
    ws2812_program_init(pio0, 0, pio_program_offset, LED_PIN, 800000, false);
    uint32_t led_data [1];

    /*
    for (;;) {
        // Test the log system
        log(LogLevel::INFORMATION, "Hello world");

        // Turn on the first LED to be a certain colour
        uint8_t red = 0;
        uint8_t green = 0;
        uint8_t blue = 255;
        led_data[0] = (red << 24) | (green << 16) | (blue << 8);
        pio_sm_put_blocking(pio0, 0, led_data[0]);
        sleep_ms(500);

        // Set the first LED off 
        led_data[0] = 0;
        pio_sm_put_blocking(pio0, 0, led_data[0]);
        sleep_ms(500);
    }
        */

    lab3();

    return 0;

}