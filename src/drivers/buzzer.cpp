#include "drivers/buzzer/buzzer.h"
#include <stdio.h>
#define BUZZERPIN 18
#include "hardware/gpio.h"
#include "pico/stdlib.h"


//Activate buzzer to play tone A for positive result / green light.
void activate_green_buzzer(uint32_t timer_interval) {
    for (int i = 0; i < timer_interval; i++) {
                gpio_set_dir(BUZZERPIN, GPIO_OUT);
                gpio_put(BUZZERPIN, true); //Turn on buzzer
                sleep_ms(1);
                gpio_put(BUZZERPIN, false); //Turn off buzzer
                sleep_ms(1);
            }
}

//Activate buzzer to play different tone for result / red light.
void activate_red_buzzer(uint32_t timer_interval) {
    uint16_t buzzer_time = timer_interval / 2; //Half the total time buzzer plays yet change its frequency to play different tone
    for (int i = 0; i < buzzer_time; i++) {
                gpio_set_dir(BUZZERPIN, GPIO_OUT);
                gpio_put(BUZZERPIN, true); //Turn on buzzer
                sleep_ms(2);
                gpio_put(BUZZERPIN, false); //Turn off buzzer
                sleep_ms(2);
            }
}
