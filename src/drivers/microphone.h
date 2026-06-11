#ifndef MICROPHONE_H
#define MICROPHONE_H

#include <stdint.h>

void microphone_init();
void microphone_read(uint16_t *buffer, int num_samples);

#endif