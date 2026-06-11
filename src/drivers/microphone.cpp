#include "microphone.h"

#include "hardware/adc.h"
#include "pico/stdlib.h"

void microphone_init()
{
    const float sample_rate = 44100.0f;
    const float adc_clk = 48000000.0f;

    float clkdiv = (adc_clk / sample_rate) - 1.0f;

    adc_init();

    // Change if microphone is connected elsewhere
    adc_select_input(0);

    adc_set_clkdiv(clkdiv);

    adc_fifo_setup(
        true,
        true,
        1,
        false,
        false
    );
}

void microphone_read(uint16_t *buffer, int num_samples)
{
    adc_fifo_drain();

    adc_run(true);

    for (int i = 0; i < num_samples; i++)
    {
        buffer[i] = adc_fifo_get_blocking();
    }

    adc_run(false);

    adc_fifo_drain();
}