#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "WS2812.pio.h"

#define BUZZERPIN 18
#define BREAKBEAMPIN 6 //digital break-beam sensor input, replaces the ADC light sensor
#define lightpin 14 //WS2812D-F5-1261 checkpoint feedback LED data pin (per schematic)
#define MAX_LAPS 100
#define STARTUP_IGNORE_MS 1500 //ignore detections in this window after boot, assumed to be the car sitting at the start line
#define NUM_LEDS 3
#define LED_BRIGHTNESS 40 //0-255, full brightness is uncomfortably bright at close range

#include "drivers/temp-hum/t-h.h"
#include "drivers/LED/leds.h"

#define TEMP_CHECK_MS 30000   // read temperature/humidity every 30 s

static void read_temp_humidity(void) {
    float temperature, humidity;
    if (sht40_read(&temperature, &humidity)) {
        printf("Temperature: %.2f C | Humidity: %.2f %%RH\n", temperature, humidity);
    }
}

void activate_buzzer(uint32_t timer_interval) {
    for (int i = 0; i < timer_interval; i++) {
                gpio_set_dir(BUZZERPIN, GPIO_OUT);
                //gpio_put(BUZZERPIN, true); //Turn on buzzer
                sleep_ms(1);
                //gpio_put(BUZZERPIN, false); //Turn off buzzer
                sleep_ms(1);
            }
}

// ----- LED feedback from the Pi (disabled) -----
// The Pi sends "LED:GREEN\n" after a clean checkpoint pass, or "LED:RED\n"
// if this checkpoint was reached after skipping the one before it.
// Commented out: depends on drivers/WS2812/ws2812.h, which has been
// superseded by the LEDDriver-based setup below. Port flash_led() onto
// LEDDriver to bring this back.
/*
#include <string.h>
#include "drivers/WS2812/ws2812.h"

#define LED_FLASH_MS 300

static char led_rx_buf[32];
static int led_rx_len = 0;

static void flash_led(uint8_t r, uint8_t g, uint8_t b) {
    ws2812_set_color(r, g, b);
    sleep_ms(LED_FLASH_MS);
    ws2812_set_color(0, 0, 0); // off
}

static void handle_led_command(const char *cmd) {
    if (strcmp(cmd, "LED:GREEN") == 0) {
        flash_led(0, 255, 0);
    } else if (strcmp(cmd, "LED:RED") == 0) {
        flash_led(255, 0, 0);
    }
    // Unrecognised commands are ignored.
}

// Drains any bytes currently waiting on stdin (USB serial from the Pi)
// without blocking. Call this once per main loop iteration.
static void poll_led_command(void) {
    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (led_rx_len > 0) {
                led_rx_buf[led_rx_len] = '\0';
                handle_led_command(led_rx_buf);
                led_rx_len = 0;
            }
        } else if (led_rx_len < (int)sizeof(led_rx_buf) - 1) {
            led_rx_buf[led_rx_len++] = (char)c;
        }
    }
}
*/

static void set_all_leds(LEDDriver &leds, RGB colour) {
    for (uint i = 0; i < NUM_LEDS; i++) {
        leds.set(i, colour);
    }
    leds.show();
}

void detect_car(LEDDriver &leds){
    gpio_init(BREAKBEAMPIN);
    gpio_set_dir(BREAKBEAMPIN, GPIO_IN);
    gpio_init(BUZZERPIN);
    uint32_t timer_interval = 100; // 100 ms interval for timer
    uint32_t time_between_readings = 0;
    uint32_t total_race_time = 0; // cumulative time since the race timer effectively started
    uint32_t elapsed_since_boot = 0; // used only for the startup ignore window
    uint32_t is_same_lap = 0;
    uint32_t lap_count = 0;
    uint32_t lap_times[MAX_LAPS];        // stores each lap's individual time in ms
    uint32_t lap_totals[MAX_LAPS];       // stores cumulative race time at each lap

    uint32_t loop_period_ms   = timer_interval * 2;              // each pass sleeps twice
    uint32_t loops_till_check = TEMP_CHECK_MS / loop_period_ms;  // 30000 / 200 = 150
    uint32_t loops_since_check = 0;

    for (int i = 0; i < MAX_LAPS; i++) {
        lap_times[i] = 0;
        lap_totals[i] = 0;
    }

    for (; lap_count < MAX_LAPS; ) {
        bool car_detected = gpio_get(BREAKBEAMPIN); // true (HIGH) = beam broken / car present. Invert (!gpio_get(...)) if your wiring is active-low.

        set_all_leds(leds, car_detected ? RGB{0, LED_BRIGHTNESS, 0} : RGB{LED_BRIGHTNESS, 0, 0});

        //Decide if a car is in the way of the beam based on the digital reading.

        if (car_detected && elapsed_since_boot >= STARTUP_IGNORE_MS) {
            //printf("Car detected!\n");
            activate_buzzer(timer_interval);

            //Detects if it's a new lap by seeing if the sensor has not been blocked since last blocking
            if (is_same_lap == 0) {
                total_race_time += time_between_readings;
                lap_times[lap_count] = time_between_readings;
                lap_totals[lap_count] = total_race_time;
                printf("Lap %d/%d  |  Total race time: %5d ms  |  Lap time: %5d ms  (%2d sec : %3d ms)\n", lap_count + 1, MAX_LAPS, total_race_time, time_between_readings, time_between_readings / 1000, time_between_readings % 1000);
                is_same_lap = 1;
                lap_count++;
                time_between_readings = 0; // reset so each lap's individual time is measured independently

                // Once array is full, print each lap's total race time and individual duration
                if (lap_count == MAX_LAPS) {
                    printf("\n--- MAX_LAPS reached, lap times ---\n");
                    for (int j = 0; j < MAX_LAPS; j++) {
                        printf("Lap %3d: %5d ms total  (individual: %5d ms, %2d sec : %3d ms)\n", j + 1, lap_totals[j], lap_times[j], lap_times[j] / 1000, lap_times[j] % 1000);
                    }
                }
            }
        } else if (!car_detected) {
            printf("No car.\n");
            sleep_ms(timer_interval);
            sleep_ms(timer_interval);
            is_same_lap = 0;
        }
        time_between_readings += timer_interval + timer_interval; //Increment the time between readings by the timer interval
        elapsed_since_boot += timer_interval + timer_interval;

        if (++loops_since_check >= loops_till_check) {
            loops_since_check = 0;
            read_temp_humidity();
        }

        // poll_led_command(); // disabled along with the Pi-serial LED feedback above

        gpio_set_dir(BUZZERPIN, GPIO_OUT);
        gpio_put(BUZZERPIN, false); //Turn off buzzer
    }

}

int main() {
    stdio_init_all();
    sleep_ms(2000); // give USB serial time to enumerate/reconnect before we start printing
    sht40_init();

    PIO led_pio = pio0;
    uint led_sm = pio_claim_unused_sm(led_pio, true);
    uint led_offset = pio_add_program(led_pio, &ws2812_program);
    ws2812_program_init(led_pio, led_sm, led_offset, lightpin, 800000.0f, false);
    LEDDriver leds(led_pio, led_sm, NUM_LEDS);
    leds.clear();

    detect_car(leds);
}
