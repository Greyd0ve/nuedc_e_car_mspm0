#ifndef __GRAYSCALE_H
#define __GRAYSCALE_H

#include <stdint.h>

#define GRAYSCALE_CHANNELS 8U

void Grayscale_Init(void);
uint8_t Grayscale_ReadChannel(uint8_t channel);
void Grayscale_ReadAll(uint8_t raw[GRAYSCALE_CHANNELS]);
uint8_t Grayscale_ReadOne(uint8_t channel);
uint8_t Grayscale_RawOUT(void);

#endif
