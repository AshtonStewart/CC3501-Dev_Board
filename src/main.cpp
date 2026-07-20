#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "drivers/leds.h"
#include "drivers/lis3dh.h"
#include "drivers/microphone.h"

#include "WS2812.pio.h"
#include "drivers/logging/logging.h"
#include <string>
#include <string.h>
#include <cstring>

#include <math.h>
#include "arm_math.h"

#define LED_PIN  14
#define NUM_LEDS 12

#define grav_const      9.81
#define baud_rate_in_z  400000
#define ACCEL_SCL_SCLK  2
#define Accel_SDA_MOSI  3

// Lab 2 

// Helper to read a line over USB serial character by character
static int read_line(char *buf, int max_len) {
    int i = 0;
    while (i < max_len - 1) {
        int c = getchar();
        if (c == EOF) continue;
        if (c == '\n' || c == '\r') {
            if (i > 0) break;  // only stop if we have something
            continue;           // skip leading newlines
        }
        buf[i++] = (char)c;
    }
    buf[i] = '\0';
    return i;
}


int lab2() {
    char input[32];

    // Quick startup animation to confirm LEDs are working
    // Cycles each LED through red, green, blue before entering the menu
    printf("Hello world\n");
    for (int i = 0; i < NUM_LEDS; i++) {
        leds_set(i, 255, 0, 0); // red
        leds_show();
        sleep_ms(5);
        leds_set(i, 0, 255, 0); // green
        leds_show();
        sleep_ms(5);
        leds_set(i, 0, 0, 255); // blue
        leds_show();
        sleep_ms(5);
    }

    // Main interactive loop — keeps running until the board is reset. Note to self: Make it end this function and loop if bootsell is pressed.
    while (true) {

        // Print the menu and wait for user input
        printf("\nOptions: [see] [set] [commit]\n> ");
        fflush(stdout);
        read_line(input, sizeof(input));
        printf("%s\n", input); // echo back what was received
        fflush(stdout);

        // Prints the current colours of the rp2040 leds and the light colours that are to be committed next time commit is used
        if (strcmp(input, "see") == 0) {
            for (int i = 0; i < NUM_LEDS; i++) {
                uint8_t cr, cg, cb, pr, pg, pb;
                leds_get(i, &cr, &cg, &cb);           // what the LED shows now
                leds_get_pending(i, &pr, &pg, &pb);   // what it will show after commit
                printf("%2d. now: R:%3d G:%3d B:%3d   will be: R:%3d G:%3d B:%3d\n",
                       i + 1, cr, cg, cb, pr, pg, pb);
                fflush(stdout);
            }


            // Asks the user which LED (or all) to change, then reads new R/G/B values without committing changes to the pico. 
        
        } else if (strcmp(input, "set") == 0) {
            char num_buf[16];

            printf("LED to change? (1-%d or 'all'): ", NUM_LEDS);
            fflush(stdout);
            read_line(num_buf, sizeof(num_buf));

            // ALL command: set every LED to the same colour ──
            if (strcmp(num_buf, "all") == 0) {

                // Use LED 0's current values as the reference for the prompt
                uint8_t cr, cg, cb;
                leds_get(0, &cr, &cg, &cb);

                printf("New R value for all LEDs? (current: %d): ", cr);
                fflush(stdout);
                read_line(num_buf, sizeof(num_buf));
                int r = atoi(num_buf);
                printf("%d\n", r);
                fflush(stdout);

                printf("New G value for all LEDs? (current: %d): ", cg);
                fflush(stdout);
                read_line(num_buf, sizeof(num_buf));
                int g = atoi(num_buf);
                printf("%d\n", g);
                fflush(stdout);

                printf("New B value for all LEDs? (current: %d): ", cb);
                fflush(stdout);
                read_line(num_buf, sizeof(num_buf));
                int b = atoi(num_buf);
                printf("%d\n", b);
                fflush(stdout);

                // Stage the same colour on every LED
                for (int i = 0; i < NUM_LEDS; i++)
                    leds_set(i, (uint8_t)r, (uint8_t)g, (uint8_t)b);

                printf("All LEDs staged. Use 'commit' to apply.\n");
                fflush(stdout);

            // ── Single LED branch: set just one LED ──
            } else {
                // Convert from 1-based user input to 0-based array index
                int idx = atoi(num_buf) - 1;
                printf("%d\n", idx + 1);
                fflush(stdout);

                // Validate the index is within range
                if (idx < 0 || idx >= NUM_LEDS) {
                    printf("Invalid LED number.\n");
                    fflush(stdout);
                    continue;
                }

                // Read the current committed colour to show as a hint
                uint8_t cr, cg, cb;
                leds_get(idx, &cr, &cg, &cb);

                printf("New R value? (current: %d): ", cr);
                fflush(stdout);
                read_line(num_buf, sizeof(num_buf));
                int r = atoi(num_buf);
                printf("%d\n", r);
                fflush(stdout);

                printf("New G value? (current: %d): ", cg);
                fflush(stdout);
                read_line(num_buf, sizeof(num_buf));
                int g = atoi(num_buf);
                printf("%d\n", g);
                fflush(stdout);

                printf("New B value? (current: %d): ", cb);
                fflush(stdout);
                read_line(num_buf, sizeof(num_buf));
                int b = atoi(num_buf);
                printf("%d\n", b);
                fflush(stdout);

                // Stage the new colour for this LED
                leds_set(idx, (uint8_t)r, (uint8_t)g, (uint8_t)b);
                printf("Staged. Use 'commit' to apply.\n");
                fflush(stdout);
            }

        // Sends all staged values to the hardware, making them visible on the LEDs.
        // Also copies pending values into the current values internally.
        } else if (strcmp(input, "commit") == 0) {
            leds_show();
            printf("Changes committed to LEDs.\n");
            fflush(stdout);

        // Unknown input catching
        } else {
            printf("Unknown command. Try: see / set / commit\n");
            fflush(stdout);
        }
    }
    return 0;
}


// Lab 3

int lab3() {
    // Initialise the accelerometer — abort if it fails
    if (!lis3dh_init()) {
        printf("Accelerometer init failed\n");
        return -1;
    }

    for (;;) {
        // Read raw acceleration values
        int16_t raw_x, raw_y, raw_z;
        lis3dh_read_raw(&raw_x, &raw_y, &raw_z);
        printf("raw_x: %d, raw_y: %d, raw_z: %d\n", raw_x, raw_y, raw_z);
        fflush(stdout);

        // Normalise X and Y to a -1.0 to 1.0 range
        float tilt_x = raw_x / 1000.0f;
        float tilt_y = raw_y / 1000.0f;

        // Clamp to -1.0 / 1.0 so we don't exceed full brightness
        if (tilt_x >  1.0f) tilt_x =  1.0f;
        if (tilt_x < -1.0f) tilt_x = -1.0f;
        if (tilt_y >  1.0f) tilt_y =  1.0f;
        if (tilt_y < -1.0f) tilt_y = -1.0f;

        for (int i = 0; i < NUM_LEDS; i++) {

            // Work out where this LED sits around the ring as an angle (0–360)
            // LED 0 starts at the top and they go clockwise
            float led_angle = (360.0f / NUM_LEDS) * i;
            float led_angle_rad = led_angle * (3.14159f / 180.0f);

            // Project the tilt vector onto this LED's position around the ring.
            // dot product gives +1 when the LED is in the direction of tilt,
            // -1 when it's on the opposite side, 0 when perpendicular.
            float led_x = sinf(led_angle_rad);
            float led_y = cosf(led_angle_rad);
            float dot = led_x * tilt_x + led_y * tilt_y;

            // Apply a square root curve to the magnitude so nearby LEDs snap
            // more aggressively to full brightness rather than falling off gradually.
            // Lower the exponent (e.g. 0.3f) to make even more LEDs hit full brightness,
            // raise it toward 1.0f to restore the original gradual falloff.
            float magnitude = powf(fabsf(dot), 0.5f);

            uint8_t r = 0, g = 0, b = 0;

            if (dot > 0.0f) {
                // High/lifted side — green
                g = (uint8_t)(magnitude * 255.0f);
            } else {
                // Low side — red
                r = (uint8_t)(magnitude * 255.0f);

            }

            // Blue fills in the gap — brightest when perpendicular to the tilt direction
            b = (uint8_t)((1.0f - magnitude) * 255.0f);
            b = ceil(b * 0.5); //reduce colour intensity / brightness
            r = ceil(r * 0.5);
            g = ceil(g * 0.5); 

            // Stage the colour for this LED
            leds_set(i, r, g, b);
        }

        // Commit all staged colours to the hardware
        leds_show();
        sleep_ms(50);
    }

    return 0;
}

// Lab 4

int lab4_mic_initialise() {
    adc_init();
    adc_run(true);
    adc_run(false);
    adc_fifo_get_blocking();
    adc_fifo_drain();

    adc_gpio_init(25);
    adc_select_input(0);

    adc_fifo_setup(
        true,   // enable FIFO
        false,  // no DMA request
        1,      // DREQ threshold
        false,  // no error bit
        false   // keep full 12-bit values
    );

    float clkdiv = (48000000.0f / 44100.0f) - 1.0f;
    adc_set_clkdiv(clkdiv);

    return 0;
}

int lab4_mic_read() {
    int num_samples = 1024;
    uint16_t buffer[1024];

    adc_fifo_drain();
    adc_run(true);

    for (int i = 0; i < num_samples; i++) {
        buffer[i] = adc_fifo_get_blocking();
    }

    adc_run(false);
    adc_fifo_drain();

    return 0;
}

// Main

int main() {
 stdio_init_all();
    sleep_ms(2000);  // wait for USB serial to connect

    arm_status cmsis_test = ARM_MATH_SUCCESS;
    printf("CMSIS-DSP smoke test: %d\n", cmsis_test);

    uint pio_program_offset = pio_add_program(pio0, &ws2812_program);
    ws2812_program_init(pio0, 0, pio_program_offset, LED_PIN, 800000, false);

    leds_init(pio0, 0);

    lab4_mic_initialise();
    lab4_mic_read();

    //lab2();
    return 0;
}