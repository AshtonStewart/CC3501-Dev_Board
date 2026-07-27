#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/adc.h"

#define THRESHOLD 300 //Rough temporary thresh hold for when a car has passed based off light change
#define ambience_high 3605 //gained from monitoring ADC light reading in regular room
#define ambience_low 3600
#define BUZZERPIN 18
#define LIGHTADCPIN 27
#define lightpin 14 //should be same as regular board
#define MAX_LAPS 5

void activate_buzzer(uint32_t timer_interval) {
    for (int i = 0; i < timer_interval; i++) {
                gpio_set_dir(BUZZERPIN, GPIO_OUT);
                gpio_put(BUZZERPIN, true); //Turn on buzzer
                sleep_ms(1);
                gpio_put(BUZZERPIN, false); //Turn off buzzer
                sleep_ms(1);
            } 
}

void detect_car(){
    adc_gpio_init(LIGHTADCPIN);
    adc_select_input(1);
    gpio_init(BUZZERPIN);
    uint32_t timer_interval = 100; // 100 ms interval for timer
    uint32_t time_between_readings = 0;
    uint32_t is_same_lap = 0;
    uint32_t lap_count = 0;
    uint32_t lap_times[MAX_LAPS]; // stores each lap's total (cumulative) time in ms

    
    
    while (true) {
        int Light_Amount = 3605 - adc_read(); // Reverse the reading to get the amount of light detected rather than lack of

        bool monitor_ADC = 0; //Check the ADC readings for debugging purposes
        if (monitor_ADC) {
            printf("ADC reading: %d\n", adc_read());
            printf("light amount: %d\n", Light_Amount);
        }
        
        //Decide if a car is in the way of the light based off quantity of recieved light.
        
        if (Light_Amount < THRESHOLD) {
            //printf("Car detected!\n");
            activate_buzzer(timer_interval);

            //Detects if it's a new lap by seeing if the sensor has not been blocked since last blocking
            if (is_same_lap == 0) {
                //printf("Time between readings: %d ms\n", time_between_readings);
                lap_times[lap_count] = time_between_readings;
                lap_count += 1;
                printf("Lap count: %3d  |  Total time to get lap: %5d ms  (%2d sec : %3d ms)\n", lap_count, time_between_readings, time_between_readings / 1000, time_between_readings % 1000);
                is_same_lap = 1;

                // Once array is full, print each individual lap's duration
                if (lap_count == MAX_LAPS) {
                    printf("\n--- MAX_LAPS reached, individual lap times ---\n");
                    for (int i = 0; i < MAX_LAPS; i++) {
                        uint32_t individual_time = (i == 0) ? lap_times[0] : lap_times[i] - lap_times[i - 1];
                        printf("Lap %3d: %5d ms  (%2d sec : %3d ms)\n", i + 1, individual_time, individual_time / 1000, individual_time % 1000);
                    }
                }
            }
        } else {
            printf("No car.\n");
            sleep_ms(timer_interval);
            is_same_lap = 0;
            //printf("Time between readings: %d ms\n", time_between_readings);
        }
        time_between_readings += timer_interval;

        gpio_set_dir(BUZZERPIN, GPIO_OUT);
        gpio_put(BUZZERPIN, false); //Turn off buzzer
    }

}

int main() {
    stdio_init_all();
    sleep_ms(2000); // give USB serial time to enumerate/reconnect before we start printing
    adc_init();
    detect_car();
}

/*
Could add: 
Light code? - Need pin numbers
*/