#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

void Encoder_Init(void);
int16_t Encoder_GetLeftDelta(void);
int16_t Encoder_GetRightDelta(void);
void Encoder_ClearAll(void);

#endif
