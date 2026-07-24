#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/adc.h"

#define THRESHOLD 300 //Rough temporary thresh hold for when a car has passed based off light change
#define ambience_high 3605 //gained from monitoring ADC light reading in regular room
#define ambience_low 3600

void detect_car(){
    adc_gpio_init(27);
    adc_select_input(1);
    
    while (true) {
        int Light_Amount = 3605 - adc_read(); // Reverse the reading to get the amount of light detected rather than lack of
        
        bool monitor_ADC = 0; //Check the ADC readings for debugging purposes
        if (monitor_ADC) {
            printf("ADC reading: %d\n", adc_read());
            printf("light amount: %d\n", Light_Amount);
        }
        
        //Decide if a car is in the way of the light based off quantity of recieved light.
        if (Light_Amount < THRESHOLD) {
            printf("Car detected!\n");
        } else {
            printf("No car.\n");
        }

        sleep_ms(100);
    }

}

int main() {
    stdio_init_all();
    adc_init();
    detect_car();
}